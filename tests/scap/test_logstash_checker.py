import datetime
import logging
import pathlib
import sys
from types import SimpleNamespace
from unittest import mock

from scap.logstash_checker import LogstashChecker

# The records and the query evaluation live beside the fake-logstash service of
# train-dev, so that one set of records serves both.
sys.path.insert(0, str(pathlib.Path(__file__).parents[2] / "local-dev"))

import fake_logstash  # noqa: E402


NOW = datetime.datetime(2026, 9, 4, 12, 0, 0, tzinfo=datetime.timezone.utc)


def _dep_config(namespace, release):
    """The fields of a DepConfig that a logstash query reads."""
    return SimpleNamespace(namespace=namespace, release=release)


def _checker(dep_configs, wiki=None, baremetal_hosts=None):
    """A checker that asks the fake logstash instead of a real one."""
    checker = LogstashChecker(
        "http://fake-logstash.example",
        fake_logstash.BUCKET_SECONDS,
        dep_configs,
        baremetal_hosts or [],
        logging.getLogger("test.logstash"),
        None,
        wiki,
    )
    checker.logstash.run_query = lambda query: fake_logstash.search(query, NOW)
    return checker


def _recent(namespace=None, release=None, wiki=None) -> int:
    """The records of the last window that match, from the records of the fake.

    The expected counts of the tests come from the records themselves, so that
    a change to them does not need a change to every count.
    """
    return sum(
        row[4]
        for row in fake_logstash.K8S_RECORDS
        if (namespace is None or row[0] == namespace)
        and (release is None or row[1] == release)
        and (wiki is None or row[2] == wiki)
    )


def _count(checker) -> int:
    """How many errors one check counts."""
    return fake_logstash.search(checker._build_query(), NOW)["hits"]["total"]["value"]


def _namespaces_reached(checker) -> dict:
    """How many hits of each namespace one check counts."""
    query = checker._build_query()
    res = {}
    for hit in fake_logstash.search(query, NOW)["hits"]["hits"]:
        labels = hit["_source"].get("kubernetes", {}).get("labels", {})
        namespace = labels.get("deployment", "(bare metal)")
        res[namespace] = res.get(namespace, 0) + 1
    return res


def _filters(dep_configs=None, wiki=None) -> list:
    """The filters that a checker builds."""
    if dep_configs is None:
        dep_configs = [_dep_config("mw-pretrain", "main")]
    return _checker(dep_configs, wiki=wiki)._build_base_query()["query"]["bool"][
        "filter"
    ]


def test_no_wiki_filter_by_default():
    assert not [one for one in _filters() if "wiki.keyword" in one.get("term", {})]


def test_wiki_filter():
    assert {"term": {"wiki.keyword": "testwiki"}} in _filters(wiki="testwiki")


def test_the_query_names_a_whole_value():
    """No clause of a query analyzes the value that it matches.

    A query_string splits the value of a text field, so a query for
    mw-pretrain counted the errors of mw-web (T435419).
    """
    supported = {"term", "terms", "range", "bool", "match_none"}

    def kinds_of(filters):
        for one in filters:
            ((kind, body),) = one.items()
            yield kind
            if kind == "bool":
                yield from kinds_of(body.get("filter", []))
                yield from kinds_of(body.get("should", []))

    kinds = set(kinds_of(_filters(wiki="testwiki")))

    assert kinds <= supported, kinds - supported


def test_the_namespace_of_a_check_names_the_keyword_subfield():
    """logstash maps the label as text, and its subfield holds a whole value.

    A term filter on the text field would match a part of the namespace, and
    a subfield that does not exist would match nothing at all, so this pins
    the name of each field.
    """
    assert _filters()[0]["bool"]["should"] == [
        {
            "bool": {
                "filter": [
                    {"term": {"kubernetes.labels.release.keyword": "main"}},
                    {
                        "terms": {
                            "kubernetes.labels.deployment.keyword": ["mw-pretrain"]
                        }
                    },
                ]
            }
        }
    ]


def test_check_counts_one_namespace():
    """A check counts the errors of the namespaces that it supervises.

    The check of a namespace with almost no traffic counted the errors of the
    whole fleet before T435419.
    """
    checker = _checker([_dep_config("mw-pretrain", "main")])

    assert _namespaces_reached(checker) == {
        "mw-pretrain": _recent("mw-pretrain", "main")
    }


def test_a_count_below_the_threshold_passes():
    checker = _checker([_dep_config("mw-web", "main")])

    assert checker.check(1000) is True


def test_a_count_at_the_threshold_fails(caplog):
    checker = _checker([_dep_config("mw-web", "main")])

    with caplog.at_level(logging.ERROR):
        assert checker.check(1) is False

    assert "The threshold is 1" in caplog.text
    # A failure names the errors that it counted.
    assert "Top" in caplog.text
    assert "DBQueryError" in caplog.text


def test_a_release_of_another_stage_is_left_out():
    """The canary release and the main release count apart.

    Both live in one namespace, so only the release clause separates them.
    """
    canary = _namespaces_reached(_checker([_dep_config("mw-web", "canary")]))
    main = _namespaces_reached(_checker([_dep_config("mw-web", "main")]))

    assert canary["mw-web"] == _recent("mw-web", "canary")
    assert main["mw-web"] == _recent("mw-web", "main")


def test_the_wiki_filter_narrows_the_count():
    dep_configs = [_dep_config("mw-web", "main")]
    every_wiki = _count(_checker(dep_configs))
    one_wiki = _count(_checker(dep_configs, wiki="testwiki"))

    assert 0 < one_wiki < every_wiki


def test_a_bare_metal_host_counts_by_its_name():
    host, _wiki, _channel, recent, _older = fake_logstash.BAREMETAL_RECORDS[0]
    checker = _checker([], baremetal_hosts=[f"{host}.eqiad.wmnet"])

    assert _namespaces_reached(checker) == {"(bare metal)": recent}


def test_a_debug_level_or_another_channel_is_left_out():
    """must_not and the channel clause leave those records out.

    The records of the fake hold one of each, in a namespace that the check
    counts, so a count that holds them says the clauses are gone.
    """
    checker = _checker([_dep_config("mw-web", "main")])

    # Both records that the fake leaves out are of mw-web and of the release
    # main, and both arrived in this window, so a count that holds them is
    # larger than the records of that release.
    assert _namespaces_reached(checker)["mw-web"] == _recent("mw-web", "main")


def test_analyze_reads_every_sample(caplog):
    checker = _checker([_dep_config("mw-web", "main")])

    with caplog.at_level(logging.INFO):
        checker.analyze("production")

    samples = checker._fetch_history_counts(90)
    assert f"#samples: {len(samples)}" in caplog.text
    assert "Suggested alert threshold" in caplog.text


def test_analyze_of_a_stage_with_no_target(caplog):
    """A query that names no target reads no sample."""
    checker = _checker([])

    with caplog.at_level(logging.WARNING):
        checker.analyze("canaries")

    assert "No matching logstash records" in caplog.text


def test_history_reads_every_page():
    """The composite aggregation pages until it runs out.

    The fake holds fewer buckets than one page, so this drives the loop with
    responses of its own.
    """
    pages = [
        {
            "aggregations": {
                "counts": {
                    "buckets": [{"doc_count": 1}] * 10000,
                    "after_key": {"timestamp": 1},
                }
            }
        },
        {"aggregations": {"counts": {"buckets": [{"doc_count": 2}] * 3}}},
    ]
    checker = _checker([_dep_config("mw-web", "main")])
    checker.logstash.run_query = mock.Mock(side_effect=pages)

    counts = checker._fetch_history_counts(90)

    assert len(counts) == 10003
    assert sum(counts) == 10006
    assert checker.logstash.run_query.call_count == 2
