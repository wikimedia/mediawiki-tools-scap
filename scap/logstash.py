# -*- coding: utf-8 -*-
"""
    scap.logstash
    ~~~~~~~~~~~~~
    Wraps code for querying a logstash instance.

    Used by logstash_checker and logstash_poller.
"""

import json
import os
import urllib3
import logging

from scap import log


class CheckServiceError(Exception):
    pass


class Logstash:
    def __init__(self, logstash_host, logger):
        self.logstash_host = logstash_host
        self.logger = logger

    def run_query(self, query_object) -> dict:
        """Run a query on the logstash server."""
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
