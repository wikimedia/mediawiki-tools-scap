from types import SimpleNamespace

from scap.logstash_checker import LogstashChecker


def _query_string(wiki=None) -> str:
    """The query_string clause a checker builds for one pretrain release."""
    dep_configs = [SimpleNamespace(release="main", namespace="mw-pretrain")]
    checker = LogstashChecker(
        "http://logstash.example", 20, dep_configs, [], None, None, wiki
    )
    filters = checker._build_base_query()["query"]["bool"]["filter"]
    return filters[0]["query_string"]["query"]


def test_no_wiki_filter_by_default():
    assert "wiki:" not in _query_string()


def test_wiki_filter():
    assert _query_string("testwiki").endswith(" AND wiki:testwiki")
