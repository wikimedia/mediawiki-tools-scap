import collections
import math
import statistics

from scap import cli, kubernetes, logstash, logstash_checker, targets

# Re-export CheckServiceError from the logstash module so that users
# of LogstashChecker can catch logstash_checker.CheckServiceError without
# needing to import scap.logstash too.
CheckServiceError = logstash.CheckServiceError


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
    def main(self, *extra_args):
        logger = self.get_logger()
        if not self.config["logstash_url"]:
            logger.warning("logstash_url is not configured; nothing to check.")
            return
        k8s_ops = kubernetes.K8sOps(self)
        baremetal_hosts = []
        if self.arguments.stage == kubernetes.CANARIES:
            baremetal_hosts = list(
                set(targets.get("dsh_api_canaries", self.config).all)
                | set(targets.get("dsh_app_canaries", self.config).all)
            )
        elif self.arguments.stage == kubernetes.PRODUCTION:
            baremetal_hosts = list(
                set(targets.get("dsh_proxies", self.config).all)
                | set(targets.get("dsh_targets", self.config).all)
            )
        logger.info("Analyzing logstash history for stage: %s", self.arguments.stage)
        logstash_checker.LogstashChecker(
            self.config["logstash_url"],
            self.config["canary_wait_time"],
            k8s_ops.get_stage_dep_configs(self.arguments.stage),
            baremetal_hosts,
            logger,
        ).analyze(self.arguments.stage, self.arguments.toohigh)


class LogstashChecker:
    def __init__(
        self,
        logstash_url,
        window_size,
        k8s_dep_configs,
        baremetal_hosts,
        logger,
    ):
        self.window_size = window_size
        self.k8s_dep_configs = k8s_dep_configs
        self.baremetal_hosts = baremetal_hosts
        self.logger = logger
        self.logstash = logstash.Logstash(logstash_url, logger)

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

    def analyze(self, stage, toohigh=None):
        """Analyze historical error counts for a deployment stage and suggest a threshold."""
        # The Zscore used to filter outliers from the history samples
        OUTLIER_ZSCORE = 3
        # The Zscore used to suggest an error count threshold.
        THRESHOLD_ZSCORE = 3
        HISTORY_DAYS = 90

        orig_samples = samples = self._fetch_history_counts(HISTORY_DAYS)

        if not samples:
            self.logger.warn("No matching logstash records found for stage %s.", stage)
            return

        def summarize(samples, description):
            mean = statistics.mean(samples)
            stdev = statistics.stdev(samples)
            self.logger.info(
                "%s: #samples: %d, min: %d, mean: %.2f, stdev: %.2f, max: %d",
                description,
                len(samples),
                min(samples),
                mean,
                stdev,
                max(samples),
            )
            return mean, stdev

        if toohigh is None:
            summarize(samples, "Initial                ")
            samples, outliers = self._exclude_outliers(samples, OUTLIER_ZSCORE)
            mean, stdev = summarize(samples, "After removing outliers")

            toohigh = math.ceil(mean + THRESHOLD_ZSCORE * stdev)
            self.logger.info(
                "Suggested alert threshold for stage %s: %d (zscore=%.2f)",
                stage,
                toohigh,
                THRESHOLD_ZSCORE,
            )
        else:
            summarize(samples, "History")
            self.logger.info("Testing stage %s with threshold of %d", stage, toohigh)

        count = 0
        for sample in orig_samples:
            if sample >= toohigh:
                count += 1

        pct = count / len(orig_samples) * 100

        self.logger.info(
            "That would trigger for %d of %d samples (%.2f%%)",
            count,
            len(orig_samples),
            pct,
        )

    ###########
    # Innards #
    ###########

    def _exclude_outliers(self, data, zscore):
        mean = statistics.mean(data)
        stdev = statistics.stdev(data)
        toohigh = mean + stdev * zscore

        res = []
        outliers = []

        for x in data:
            if x >= toohigh:
                outliers.append(x)
            else:
                res.append(x)

        return res, outliers

    def _one_of_query(self, key, values) -> str:
        return f"{key}:(" + " OR ".join(values) + ")"

    def _build_deployment_query(self) -> str:
        deployments_by_release = collections.defaultdict(set)
        for dep_config in self.k8s_dep_configs:
            deployments_by_release[dep_config.release].add(dep_config.namespace)

        clauses = [
            f"(kubernetes.labels.release:{release} AND {self._one_of_query('kubernetes.labels.deployment', sorted(deployments))})"
            for release, deployments in sorted(deployments_by_release.items())
        ]

        return " OR ".join(clauses)

    def _build_baremetal_query(self) -> str:
        if not self.baremetal_hosts:
            return ""

        # Logstash stores baremetal hostnames without the domain suffix.
        hostnames = sorted({host.split(".")[0] for host in self.baremetal_hosts})
        return self._one_of_query("host", hostnames)

    def _build_base_query(self) -> dict:
        """
        Build a query filtering for the relevant deployment targets, record type, and channel.
        """

        deployment_terms = []
        deployment_query = self._build_deployment_query()
        baremetal_query = self._build_baremetal_query()

        if deployment_query:
            deployment_terms.append(f"({deployment_query})")
        if baremetal_query:
            deployment_terms.append(baremetal_query)

        filters = []

        if deployment_terms:
            query = (
                f"({' OR '.join(deployment_terms)}) AND type:mediawiki "
                "AND channel:(exception OR error)"
            )
            filters.append({"query_string": {"query": query}})
        else:
            filters.append({"match_none": {}})

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
            hit = hit["_source"]
            message = hit.get("normalized_message") or hit.get("message")

            if message.startswith("[{reqId}] {exception_url}   "):
                message = message[len("[{reqId}] {exception_url}   ") :]

            hits[message] += 1

        top = hits.most_common(5)

        msg = [f"Top {len(top)} errors:"]
        for message, count in top:
            msg.append(f"[{count} hits] {message}")

        self.logger.error("%s", "\n".join(msg))
