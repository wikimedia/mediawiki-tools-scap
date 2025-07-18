import json
import logging
import os
import time

from scap import cli, logstash, utils


LOG_FILE = "logstash_errors.json"
DEFAULT_WINDOW = 600
POLL_INTERVAL = 15


def main(cfg, logger, results_dir, debug_logstash):
    """
    The logstash poller main loop.
    """
    poller = LogstashPoller(
        cfg["logstash_host"], DEFAULT_WINDOW, logger, debug_logstash
    )
    while True:
        try:
            results = poller.poll()
            summary = poller.summarize_errors(results)
            with utils.temp_to_permanent_file(os.path.join(results_dir, LOG_FILE)) as f:
                json.dump(summary, f)
        except Exception as e:
            logger.error(f"Polling error: {e}")

        time.sleep(POLL_INTERVAL)


@cli.command(
    "poll-logstash",
    help="Poll MediaWiki logstash history",
)
class LogstashPollCommand(cli.Application):
    @cli.argument(
        "--window",
        dest="window",
        default=DEFAULT_WINDOW,
        metavar="INT",
        help="Window (seconds into the past) to poll",
        type=int,
    )
    @cli.argument(
        "--debug",
        action="store_true",
        help="Enable debug logging for logstash queries",
    )
    def main(self, *extra_args):
        window = self.arguments.window

        logger = self.get_logger()
        if self.arguments.debug:
            logging.root.handlers[0].setLevel("DEBUG")

        poller = LogstashPoller(
            self.config["logstash_host"],
            window,
            logger,
            self.arguments.debug or self.config["debug_logstash"],
        )
        results = poller.poll()
        summary = poller.summarize_errors(results)

        for message, count in summary.items():
            logger.info(f"{count}\t{message}")


class LogstashPoller:
    def __init__(self, logstash_host, window_size, logger, debug_logstash):
        self.window_size = window_size
        self.logger = logger
        self.logstash = logstash.Logstash(
            logstash_host, logger if debug_logstash else None
        )

    def poll(self) -> dict:
        """
        Retrieve a dictionary containing logstash errors for the last
        self.window_size seconds.

        Example:

        {
          'took': 44,
          'timed_out': False,
          '_shards': {
            'total': 546,
            'successful': 546,
            'skipped': 540,
            'failed': 0
          },
          'hits': {
            'total': {
              'value': 9,
              'relation': 'eq'
            },
            'max_score': 0,
            'hits': [ { ... }, { ... }, ... ]
          }
        }
        """

        q = self._build_query()
        r = self.logstash.run_query(q)

        hits_total = r["hits"]["total"]
        count = hits_total["value"]
        hits_rel = hits_total["relation"]
        assert hits_rel in ["eq", "gte"]

        log_count_msg = f"Logstash poller counted {count} error(s) in the last {self.window_size} seconds"
        self.logger.debug(log_count_msg)

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

        query = "type:mediawiki AND channel:(exception OR error)"

        return {
            # TODO: Figure out how to make aggregations actually-useful:
            "aggs": {
                "normalized_message": {
                    "terms": {
                        "field": "normalized_message.keyword",
                        "order": {"_count": "desc"},
                    },
                },
            },
            "query": {
                "bool": {
                    "filter": [
                        {"query_string": {"query": query}},
                    ],
                    "must_not": [
                        {"terms": {"level": ["DEBUG"]}},
                    ],
                }
            },
        }

    def _build_query(self) -> dict:
        q = self._build_base_query()

        # Longterm TODO:
        #
        #   - Really we just want everything here.
        #   - Per docs, the default is 10:
        #     https://docs.opensearch.org/docs/latest/api-reference/search/
        #   - Unclear at what point you have to paginate or something.

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

    def summarize_errors(self, r):
        errors = {}

        total = r.get("hits", {}).get("total", {}).get("value", 0)

        for hit in r.get("hits", {}).get("hits"):
            hit = hit.get("_source", {})
            message = hit.get("normalized_message") or hit.get("message")

            # Trim some boilerplate:
            if message.startswith("[{reqId}] {exception_url}   "):
                message = message[len("[{reqId}] {exception_url}   ") :]

            # Potential TODO: Use aggregations here instead
            if message not in errors:
                errors[message] = {}
                errors[message]["count"] = 0
                errors[message]["versions"] = []
                # TODO: first seen
                # TODO: last seen
                # TODO: little sparkline / histogram?

            # This could be a set, but sets aren't JSON-serializable
            mwversion = hit.get("mwversion")
            if mwversion and mwversion not in errors[message]["versions"]:
                errors[message]["versions"].append(hit.get("mwversion"))
            errors[message]["count"] += 1

        return {"errors": errors, "total": total}
