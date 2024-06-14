import collections
import json
import logging
import math
import os
import statistics
import urllib3

from scap import cli, kubernetes, log, logstash_checker, targets


class CheckServiceError(Exception):
    pass


@cli.command(
    "analyze-logstash",
    help="Analyze mediawiki canary logstash history and suggest an error count threshold",
)
class LogstashCheckerCommand(cli.Application):
    @cli.argument(
        "--try",
        dest="toohigh",
        metavar="INT",
        help="Test historical samples using the supplied value as the canary_threshold",
        type=int,
    )
    def main(self, *extra_args):
        k8s_canary_namespaces = kubernetes.K8sOps(self).get_canary_namespaces()

        baremetal_canaries = list(
            set(targets.get("dsh_api_canaries", self.config).all)
            | set(targets.get("dsh_app_canaries", self.config).all)
        )

        checker = logstash_checker.LogstashChecker(
            self.config["logstash_host"],
            self.config["canary_wait_time"],
            baremetal_canaries,
            k8s_canary_namespaces,
            self.get_logger(),
        )
        checker.analyze(self.arguments.toohigh)


class LogstashChecker:
    def __init__(
        self,
        logstash_host,
        window_size,
        baremetal_canaries,
        k8s_canary_namespaces,
        logger,
    ):
        self.logstash_host = logstash_host
        self.window_size = window_size
        self.baremetal_canaries = baremetal_canaries
        self.k8s_canary_namespaces = k8s_canary_namespaces
        self.logger = logger

    def check(self, threshold) -> bool:
        """
        Retrieve canary errors for the last self.window_size seconds.  If the
        error count is below threshold, returns True.  If at or above the threshold,
        returns False after logging the top 5 errors.
        """
        q = self._build_query()
        r = self._run_query(q)

        hits_total = r["hits"]["total"]
        count = hits_total["value"]
        hits_rel = hits_total["relation"]
        assert hits_rel in ["eq", "gte"]

        prefix = f"Logstash checker Counted {count} error(s) in the last {self.window_size} seconds"

        if count >= threshold:
            self.logger.error("%s. The threshold is %d.", prefix, threshold)
            self._summarize_errors(r)
            return False
        else:
            self.logger.info("%s. OK.", prefix)
            return True

    def analyze(self, toohigh=None):
        # The Zscore used to filter outliers from the history samples
        OUTLIER_ZSCORE = 3
        # The Zscore used to suggest an error count threshold.
        THRESHOLD_ZSCORE = 3

        r = self._run_query(self._build_history_query())

        orig_samples = samples = [
            bucket["doc_count"] for bucket in r["aggregations"]["counts"]["buckets"]
        ]

        if not samples:
            self.logger.warn("No matching logstash records found.")
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
                "Suggested alert threshold: %d (zscore=%.2f)", toohigh, THRESHOLD_ZSCORE
            )
        else:
            summarize(samples, "History")
            self.logger.info("Testing with canary_threshold of %d", toohigh)

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

    def _build_base_query(self) -> dict:
        """
        Build a query filtering for the relevant hosts/labels, record type, and channel.
        """

        host_query = []

        if self.k8s_canary_namespaces:
            host_query.append(
                "(kubernetes.labels.release:canary AND "
                + self._one_of_query(
                    "kubernetes.labels.deployment", self.k8s_canary_namespaces
                )
                + ")"
            )

        if self.baremetal_canaries:
            # Strip domain from hostname since logstash records hold unqualified hostnames.
            bm_canaries = [canary.split(".")[0] for canary in self.baremetal_canaries]
            host_query.append(self._one_of_query("host", bm_canaries))

        host_query = " OR ".join(host_query)

        query = f"({host_query}) AND type:mediawiki AND channel:(exception OR error)"

        return {
            "query": {
                "bool": {
                    "filter": [
                        {"query_string": {"query": query}},
                    ],
                    "must_not": [{"terms": {"level": ["DEBUG"]}}],
                }
            },
        }

    def _build_history_query(self) -> dict:
        q = self._build_base_query()

        # No log records needed.
        q["size"] = 0

        # We just care about counts
        q["aggs"] = {
            "counts": {
                "date_histogram": {
                    "field": "@timestamp",
                    "fixed_interval": f"{self.window_size}s",
                    "time_zone": "UTC",
                    "min_doc_count": 1,
                }
            }
        }

        return q

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

    def _run_query(self, query_object) -> dict:
        self.logger.debug("logstash query: %s", json.dumps(query_object))

        try:
            pool = urllib3.PoolManager(
                retries=1,
                timeout=10,
                ca_certs="/etc/ssl/certs/ca-certificates.crt",
                cert_reqs="CERT_REQUIRED",
            )
            logstash_search_url = os.path.join(
                self.logstash_host, "logstash-*", "_search"
            )
            response = pool.urlopen(
                "POST",
                logstash_search_url,
                headers={"Content-Type": "application/json"},
                body=json.dumps(query_object),
            )
            resp = response.data.decode("utf-8")
            log.log_large_message(
                f"logstash response {resp}", self.logger, logging.DEBUG
            )
            r = json.loads(resp)
        except urllib3.exceptions.SSLError:
            raise CheckServiceError("Invalid certificate")
        except (
            urllib3.exceptions.ConnectTimeoutError,
            urllib3.exceptions.TimeoutError,
            urllib3.exceptions.ConnectionError,
            urllib3.exceptions.ReadTimeoutError,
        ):
            raise CheckServiceError(
                f"Timeout on connection while downloading {logstash_search_url}"
            )
        except Exception as e:
            raise CheckServiceError(f"Generic connection error: {e}")

        if type(r) is not dict:
            raise ValueError(
                "Unexpected response from %s. Expected a dict but got: %s\n\nQuery was: %s"
                % (logstash_search_url, json.dumps(r), json.dumps(query_object))
            )

        if "error" in r:
            raise ValueError(
                "Logstash request to %s returned error:\n%s\n\nQuery was: %s"
                % (logstash_search_url, r, json.dumps(query_object))
            )

        return r

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
