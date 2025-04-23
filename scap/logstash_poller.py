import collections
import json
import logging
import math
import os
import statistics
import urllib3

from scap import cli, kubernetes, log, logstash_poller, targets


class CheckServiceError(Exception):
    pass


@cli.command(
    "poll-logstash",
    help="Poll MediaWiki logstash history",
)
class LogstashPollCommand(cli.Application):
    @cli.argument(
        "--window",
        dest="window",
        metavar="INT",
        help="Window (seconds into the past) to poll",
        type=int,
    )
    @cli.argument(
        "--plain",
        action="store_true",
        help="Output summary in plaintext",
    )
    def main(self, *extra_args):
        if self.arguments.window:
            window = self.arguments.window
        else:
            window = 600

        logger = self.get_logger()

        poller = logstash_poller.LogstashPoller(
            self.config["logstash_host"],
            window,
            logger,
        )
        results = poller.poll()
        summary = poller.summarize_errors(results)

        if self.arguments.plain:
            print(summary)
        else:
            logger.info(summary)


class LogstashPoller:
    def __init__(
        self,
        logstash_host,
        window_size,
        logger,
    ):
        self.logstash_host = logstash_host
        self.window_size = window_size
        self.logger = logger

    def poll(self) -> bool:
        """
        Retrieve canary errors for the last self.window_size seconds.
        """

        q = self._build_query()
        r = self._run_query(q)

        hits_total = r["hits"]["total"]
        count = hits_total["value"]
        hits_rel = hits_total["relation"]
        assert hits_rel in ["eq", "gte"]

        prefix = f"Logstash poller counted {count} error(s) in the last {self.window_size} seconds"
        self.logger.debug("%s", prefix)

        return r

    ###########
    # Innards #
    ###########

    def _one_of_query(self, key, values) -> str:
        return f"{key}:(" + " OR ".join(values) + ")"

    def _build_base_query(self) -> dict:
        """
        Build a query filtering for the relevant hosts/labels, record type, and channel.
        """

        query = f"type:mediawiki AND channel:(exception OR error)"

        return {
            # TODO: This doesn't work - figure out why.
            # "aggs": {
            #     "by_normalized_message": {
            #         "terms": {
            #             "field": "labels.normalized_message",
            #             "order": {
            #                 "_count": "desc"
            #             }
            #         }
            #     }
            # },
            "query": {
                "bool": {
                    "filter": [
                        { "query_string": {"query": query} },
                    ],
                    "must_not": [
                        { "terms": { "level": ["DEBUG"] } },
                    ],
                }
            },
        }

    def _build_query(self) -> dict:
        q = self._build_base_query()

        # TODO: Really we just want everything
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

    def summarize_errors(self, r):
        hits = collections.Counter()
        for hit in r["hits"]["hits"]:
            hit = hit["_source"]
            message = hit.get("normalized_message") or hit.get("message")
            message = message.replace("\n", " ")

            if message.startswith("[{reqId}] {exception_url}   "):
                message = message[len("[{reqId}] {exception_url}   ") :]

            hits[message] += 1

        hit_counts = hits.most_common()

        msg = []
        for message, count in hit_counts:
            msg.append(f"[{count} hits] {message}")

        return "\n".join(msg)
