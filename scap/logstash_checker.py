import collections
import math
import statistics
from typing import Optional

from scap import cli, kubernetes, logstash, logstash_checker, targets

# Re-export CheckServiceError from the logstash module so that users
# of LogstashChecker can catch logstash_checker.CheckServiceError without
# needing to import scap.logstash too.
CheckServiceError = logstash.CheckServiceError

# The default of --outlier-zscore and --threshold-zscore.
DEFAULT_ZSCORE = 3


@cli.command(
    "analyze-logstash",
    help="Analyze mediawiki logstash history for a deployment stage and suggest an error count threshold",
)
class LogstashCheckerCommand(cli.Application):
    @cli.argument(
        "stage",
        choices=[kubernetes.CANARIES, kubernetes.PRODUCTION],
        help="Deployment stage to analyze",
    )
    @cli.argument(
        "--try",
        dest="toohigh",
        metavar="INT",
        help="Test historical samples for the selected stage using the supplied threshold value",
        type=int,
    )
    @cli.argument(
        "--scope",
        help="Analyze only the Kubernetes targets of this scope (e.g. 'pretrain'), "
        "which are the ones a logstash_check of that scope supervises. Without it, "
        "the analysis covers the targets of every scope at this stage, and the "
        "bare-metal hosts too.",
    )
    @cli.argument(
        "--wiki",
        help="Count only errors reported for this wiki (e.g. 'testwiki')",
    )
    @cli.argument(
        "--outlier-zscore",
        metavar="FLOAT",
        type=float,
        default=DEFAULT_ZSCORE,
        help="A sample this many standard deviations above the mean is an "
        "outlier. The analysis drops it before it suggests a threshold.",
    )
    @cli.argument(
        "--threshold-zscore",
        metavar="FLOAT",
        type=float,
        default=DEFAULT_ZSCORE,
        help="The suggested threshold is the mean of the filtered samples, "
        "plus this many standard deviations.",
    )
    def main(self, *extra_args):
        logger = self.get_logger()
        if not self.config["logstash_url"]:
            logger.warning("logstash_url is not configured; nothing to check.")
            return

        stage = self.arguments.stage
        scope = self.arguments.scope
        wiki = self.arguments.wiki

        if scope and not self.config["deploy_mw_container_image"]:
            raise SystemExit(
                "--scope selects Kubernetes deployment targets, but "
                "deploy_mw_container_image is False"
            )

        k8s_ops = kubernetes.K8sOps(
            self, update_releases_repo=False, scope={scope} if scope else None
        )

        baremetal_hosts = []
        if scope:
            # Analyze what a logstash_check of this scope supervises: the scope's
            # Kubernetes targets at this stage, and no bare-metal hosts.
            dep_configs = k8s_ops.k8s_deployments_config.supervised_dep_configs(
                scope, stage
            )
        else:
            dep_configs = k8s_ops.get_stage_dep_configs(stage)

            if stage == kubernetes.CANARIES:
                baremetal_hosts = list(
                    set(targets.get("dsh_api_canaries", self.config).all)
                    | set(targets.get("dsh_app_canaries", self.config).all)
                )
            elif stage == kubernetes.PRODUCTION:
                baremetal_hosts = list(
                    set(targets.get("dsh_proxies", self.config).all)
                    | set(targets.get("dsh_targets", self.config).all)
                )

        targets_description = f"scope: {scope or '(all)'}, stage: {stage}"

        if not dep_configs and not baremetal_hosts:
            logger.warning(f"No deployment targets match {targets_description}.")
            return

        logger.info(
            f"Analyzing logstash history for {targets_description}, "
            f"wiki: {wiki or '(all)'}"
        )
        logstash_checker.LogstashChecker(
            self.config["logstash_url"],
            self.config["canary_wait_time"],
            dep_configs,
            baremetal_hosts,
            logger,
            self.config["logstash_credentials_file"],
            wiki,
        ).analyze(
            stage,
            self.arguments.toohigh,
            self.arguments.outlier_zscore,
            self.arguments.threshold_zscore,
        )


class LogstashChecker:
    def __init__(
        self,
        logstash_url,
        window_size,
        k8s_dep_configs,
        baremetal_hosts,
        logger,
        credentials_file=None,
        wiki=None,
    ):
        self.window_size = window_size
        self.k8s_dep_configs = k8s_dep_configs
        self.baremetal_hosts = baremetal_hosts
        self.logger = logger
        # When set, only errors reported for this wiki are counted.
        self.wiki = wiki
        self.logstash = logstash.Logstash(logstash_url, logger, credentials_file)

    def check(self, threshold) -> bool:
        """
        Retrieve matching deployment errors for the last self.window_size seconds.
        If the error count is below threshold, returns True. If at or above the
        threshold, returns False after logging the top 5 errors.
        """
        q = self._build_query()
        r = self.logstash.run_query(q)

        hits_total = r["hits"]["total"]
        count = hits_total["value"]
        hits_rel = hits_total["relation"]
        assert hits_rel in ["eq", "gte"]

        prefix = f"Logstash checker counted {count} error(s) in the last {self.window_size} seconds"

        if count >= threshold:
            self.logger.error("%s. The threshold is %d.", prefix, threshold)
            self._summarize_errors(r)
            return False
        else:
            self.logger.info("%s. OK.", prefix)
            return True

    def analyze(
        self,
        stage,
        toohigh=None,
        outlier_zscore=DEFAULT_ZSCORE,
        threshold_zscore=DEFAULT_ZSCORE,
    ):
        """Analyze historical error counts for a deployment stage and suggest a threshold."""
        HISTORY_DAYS = 90

        orig_samples = samples = self._fetch_history_counts(HISTORY_DAYS)

        if not samples:
            self.logger.warning(
                "No matching logstash records found for stage %s.", stage
            )
            return

        windows = HISTORY_DAYS * 24 * 3600 // self.window_size
        self._report_distribution(orig_samples, windows, HISTORY_DAYS)

        def summarize(samples, description):
            mean = statistics.mean(samples)
            stdev = statistics.stdev(samples)
            self.logger.info(
                "%s: #error-windows: %d, mean: %.2f, stdev: %.2f, max: %d",
                description,
                len(samples),
                mean,
                stdev,
                max(samples),
            )
            return mean, stdev

        if toohigh is None:
            summarize(samples, "Initial                ")

            samples, outliers, cutoff = self._exclude_outliers(samples, outlier_zscore)
            largest = f", max: {max(outliers)}" if outliers else ""
            self.logger.info(
                f"Outliers               : {len(outliers)} error-windows at or "
                f"above "
                f"{cutoff:.2f} (zscore={outlier_zscore:.2f}){largest}"
            )

            mean, stdev = summarize(samples, "After removing outliers")

            toohigh = math.ceil(mean + threshold_zscore * stdev)
            self.logger.info(
                "Suggested alert threshold for stage %s: %d (zscore=%.2f)",
                stage,
                toohigh,
                threshold_zscore,
            )
        else:
            summarize(samples, "History")
            self.logger.info("Testing stage %s with threshold of %d", stage, toohigh)

        count = 0
        for sample in orig_samples:
            if sample >= toohigh:
                count += 1

        self.logger.info(
            f"That would trigger for {count} of {len(orig_samples)} "
            f"error-windows, or {count} of {windows} windows"
        )

    def _report_distribution(self, samples, windows, history_days):
        samples = sorted(samples)

        levels = {}
        for percentile in [50, 75, 90, 95, 99]:
            index = min(len(samples) - 1, percentile * len(samples) // 100)
            levels[percentile] = samples[index]

        self.logger.info(
            "Percentiles of every window with an error: "
            + ", ".join(f"{p}th: {level}" for p, level in levels.items())
            + f", max: {samples[-1]}"
        )

        # Construct a set of levels to report, including the max sample value,
        # then convert to a sorted list.
        levels_to_report = sorted({*levels.values(), samples[-1]})

        windows_at_or_above = {0: windows}
        for level in levels_to_report:
            windows_at_or_above[level] = sum(1 for one in samples if one >= level)

        level_width = max(len(str(level)) for level in windows_at_or_above)
        count_width = max(len(str(count)) for count in windows_at_or_above.values())

        for level, count in sorted(windows_at_or_above.items()):
            period = (
                f", over {history_days} days, {self.window_size} seconds each"
                if not level
                else ""
            )
            self.logger.info(
                f"Windows with {level:>{level_width}} or more errors: "
                f"{count:>{count_width}}{period}"
            )

    ###########
    # Innards #
    ###########

    def _exclude_outliers(self, data, zscore):
        """Splits the data at the mean plus `zscore` standard deviations.

        Returns the samples below the cutoff, the samples at or above it, and
        the cutoff.
        """
        mean = statistics.mean(data)
        stdev = statistics.stdev(data)
        cutoff = mean + stdev * zscore

        res = []
        outliers = []

        for x in data:
            if x >= cutoff:
                outliers.append(x)
            else:
                res.append(x)

        return res, outliers, cutoff

    # A bool clause combines other clauses. The name of the list decides how:
    #   filter    every clause must match
    #   should    at least one clause must match (with minimum_should_match 1)
    #   must_not  no clause may match
    # A list of filter clauses is an AND. A bool with should is an OR.
    #
    # These clauses match a field:
    #   term      the field holds one value
    #   terms     the field holds any of several values
    #   range     the field falls between bounds
    def _deployment_filters(self) -> list:
        deployments_by_release = collections.defaultdict(set)
        for dep_config in self.k8s_dep_configs:
            deployments_by_release[dep_config.release].add(dep_config.namespace)

        release_field = "kubernetes.labels.release.keyword"
        deployment_field = "kubernetes.labels.deployment.keyword"

        return [
            {
                "bool": {
                    "filter": [
                        {"term": {release_field: release}},
                        {"terms": {deployment_field: sorted(deployments)}},
                    ]
                }
            }
            for release, deployments in sorted(deployments_by_release.items())
        ]

    def _baremetal_filter(self) -> Optional[dict]:
        if not self.baremetal_hosts:
            return None

        # Logstash stores baremetal hostnames without the domain suffix.
        hostnames = sorted({host.split(".")[0] for host in self.baremetal_hosts})
        return {"terms": {"host": hostnames}}

    def _build_base_query(self) -> dict:
        """
        Build a query filtering for the relevant deployment targets, record type,
        channel, and (when set) wiki.

        Do not use a query_string clause here. It splits "mw-pretrain" into
        "mw" and "pretrain", and then it matches a namespace that holds
        either part (T435419).

        type, host and level are keyword fields. channel, wiki and the
        kubernetes labels are text fields, so those clauses name the .keyword
        subfield.
        """
        targets = self._deployment_filters()

        baremetal = self._baremetal_filter()
        if baremetal:
            targets.append(baremetal)

        if not targets:
            filters = [{"match_none": {}}]
        else:
            filters = [
                {"bool": {"should": targets, "minimum_should_match": 1}},
                {"term": {"type": "mediawiki"}},
                {"terms": {"channel.keyword": ["exception", "error"]}},
            ]
            if self.wiki:
                # Records without a wiki field do not match.
                filters.append({"term": {"wiki.keyword": self.wiki}})

        return {
            "query": {
                "bool": {
                    "filter": filters,
                    "must_not": [{"terms": {"level": ["DEBUG"]}}],
                }
            },
        }

    def _fetch_history_counts(self, history_days) -> list:
        """
        Fetch per-window error counts for the last history_days days.

        Uses a composite aggregation with pagination to avoid Elasticsearch's
        max_buckets limit, which would be exceeded when scanning large time
        ranges at small (window_size) intervals.
        """
        PAGE_SIZE = 10000
        base_q = self._build_base_query()
        base_q["size"] = 0
        base_q["query"]["bool"]["filter"].append(
            {
                "range": {
                    "@timestamp": {
                        "lte": "now",
                        "gte": f"now-{history_days}d",
                    }
                },
            },
        )

        counts = []
        after_key = None

        while True:
            sources = [
                {
                    "timestamp": {
                        "date_histogram": {
                            "field": "@timestamp",
                            "fixed_interval": f"{self.window_size}s",
                            "time_zone": "UTC",
                        }
                    }
                }
            ]
            composite = {"size": PAGE_SIZE, "sources": sources}
            if after_key is not None:
                composite["after"] = after_key

            q = dict(base_q)
            q["aggs"] = {"counts": {"composite": composite}}

            r = self.logstash.run_query(q)
            buckets = r["aggregations"]["counts"]["buckets"]
            counts.extend(bucket["doc_count"] for bucket in buckets)

            after_key = r["aggregations"]["counts"].get("after_key")
            if after_key is None or len(buckets) < PAGE_SIZE:
                break

        return counts

    def _build_query(self) -> dict:
        q = self._build_base_query()

        # Return up to 500 log records
        q["size"] = 500

        # Only consider records from the last window_size seconds.
        q["query"]["bool"]["filter"].append(
            {
                "range": {
                    "@timestamp": {
                        "lte": "now",
                        "gte": f"now-{self.window_size}s",
                    }
                },
            },
        )

        return q

    def _summarize_errors(self, r):
        hits = collections.Counter()
        for hit in r["hits"]["hits"]:
            message = logstash.error_message(hit["_source"])
            hits[message] += 1

        top = hits.most_common(5)

        msg = [f"Top {len(top)} errors:"]
        for message, count in top:
            msg.append(f"[{count} hits] {message}")

        self.logger.error("%s", "\n".join(msg))
