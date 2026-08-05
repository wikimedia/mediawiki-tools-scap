# -*- coding: utf-8 -*-
"""
    scap.logstash
    ~~~~~~~~~~~~~
    Wraps code for querying a logstash instance.

    Used by logstash_checker and logstash_poller.
"""

import json
import urllib3
import logging

from scap import log


class CheckServiceError(Exception):
    pass


def _read_credentials(credentials_file) -> str:
    """
    Returns the first line of credentials_file that is neither blank nor a
    #-prefixed comment. Any lines after it are ignored.
    """
    try:
        with open(credentials_file) as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#"):
                    return line
    except OSError as e:
        raise ValueError(f"Could not read logstash credentials file: {e}")

    raise ValueError(f'{credentials_file} does not have a "username:password" line')


def _basic_auth_headers(credentials_file) -> dict:
    """
    Returns HTTP basic auth headers built from the credentials in
    credentials_file.
    """
    credentials = _read_credentials(credentials_file)

    username, sep, password = credentials.partition(":")
    if not sep or not username or not password:
        raise ValueError(f'{credentials_file} must have a "username:password" line')

    return urllib3.make_headers(basic_auth=f"{username}:{password}")


class Logstash:
    def __init__(self, logstash_url, logger, credentials_file=None):
        """
        If logger is supplied, the logstash query and response will be
        logged at DEBUG level. Note that query responses are very lengthy and
        are always unpleasantly split across multiple log records, so only
        use this when needed.

        If credentials_file is supplied, it must have a "username:password"
        line, which is used to authenticate queries. A credentials_file that
        cannot be read or parsed raises ValueError.
        """
        self.logstash_url = logstash_url
        self.logger = logger
        self.headers = {"Content-Type": "application/json"}
        if credentials_file:
            self.headers.update(_basic_auth_headers(credentials_file))

    def run_query(self, query_object) -> dict:
        """Run a query on the logstash server."""
        if self.logger:
            self.logger.debug("logstash query: %s", json.dumps(query_object))

        try:
            pool = urllib3.PoolManager(
                retries=1,
                timeout=10,
                ca_certs="/etc/ssl/certs/ca-certificates.crt",
                cert_reqs="CERT_REQUIRED",
            )
            logstash_search_url = f"{self.logstash_url}/logstash-*/_search"
            response = pool.urlopen(
                "POST",
                logstash_search_url,
                headers=self.headers,
                body=json.dumps(query_object),
            )
            resp = response.data.decode("utf-8")
            if self.logger:
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
