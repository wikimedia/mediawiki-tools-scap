import datetime
import logging
import pathlib
import sys

from scap.logstash_poller import LogstashPoller

# The records and the query evaluation live beside the fake-logstash service of
# train-dev, so that one set of records serves both.
sys.path.insert(0, str(pathlib.Path(__file__).parents[2] / "local-dev"))

import fake_logstash  # noqa: E402


NOW = datetime.datetime(2026, 9, 4, 12, 0, 0, tzinfo=datetime.timezone.utc)


def _poller():
    """A poller that asks the fake logstash instead of a real one."""
    poller = LogstashPoller(
        "http://fake-logstash.example",
        fake_logstash.BUCKET_SECONDS,
        logging.getLogger("test.poller"),
        False,
    )
    poller.logstash.run_query = lambda query: fake_logstash.search(query, NOW)
    return poller


def _records_of_the_window() -> int:
    """Every record of the last window, whatever namespace holds it.

    The poller selects no target, so it counts them all. The two records that
    the fake leaves out are of another channel and of level DEBUG.
    """
    return sum(row[4] for row in fake_logstash.K8S_RECORDS) + sum(
        row[3] for row in fake_logstash.BAREMETAL_RECORDS
    )


def test_poll_counts_the_records_of_the_window():
    response = _poller().poll()

    assert response["hits"]["total"]["value"] == _records_of_the_window()


def test_the_query_names_a_whole_value():
    """No clause of the query analyzes the value that it matches.

    A query_string clause did that, and the fake logstash of train-dev
    answers 400 for one now (T435419).
    """
    filters = _poller()._build_query()["query"]["bool"]["filter"]

    assert {"query_string"} & {kind for one in filters for kind in one} == set()


def test_summarize_errors_groups_the_messages():
    poller = _poller()

    summary = poller.summarize_errors(poller.poll())

    assert summary["total"] == _records_of_the_window()
    assert sum(one["count"] for one in summary["errors"].values()) == summary["total"]
    # The summary holds one key for each message of the window.
    assert 1 < len(summary["errors"]) <= len(fake_logstash.MESSAGES)
    # Every record holds the reqId prefix, and a summary trims it.
    for message in summary["errors"]:
        assert not message.startswith(fake_logstash.REQID_PREFIX)


def test_poll_returns_the_aggregation_that_it_asks_for():
    """The poller asks for a count of each message, and ignores it today."""
    buckets = _poller().poll()["aggregations"]["normalized_message"]["buckets"]

    assert 1 < len(buckets) <= len(fake_logstash.MESSAGES)
    assert sum(one["doc_count"] for one in buckets) == _records_of_the_window()
    # The largest count comes first.
    assert [one["doc_count"] for one in buckets] == sorted(
        (one["doc_count"] for one in buckets), reverse=True
    )


def test_summarize_errors_lists_the_versions():
    """The view of the errors in SpiderPig lists the versions of a message.

    The records of the fake held no "mwversion" once, so that view showed no
    version at all.
    """
    poller = _poller()

    summary = poller.summarize_errors(poller.poll())

    for entry in summary["errors"].values():
        assert entry["versions"]
        assert set(entry["versions"]) <= set(fake_logstash.VERSIONS)

    # A deployment moves between two versions, so one message holds both.
    assert any(len(entry["versions"]) > 1 for entry in summary["errors"].values())
