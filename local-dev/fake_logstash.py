"""
    fake_logstash
    ~~~~~~~~~~~~~
    Fake log records and query evaluation

    There are two uses of this module:

    * The tests of scap. They drive LogstashChecker against it.
    * The fake-logstash service of train-dev. It serves these responses at
      http://fake-logstash.traindev:5000.

    The train-dev service reads this file from the scap checkout of the
    workspace. One set of records serves both uses. A change here needs no
    new image.

    What this evaluates
    -------------------

    Scap sends these clauses: bool with filter, should and must_not; term and
    terms filters; a range on @timestamp. An unsupported clause raises. A
    fake must not answer a question that it does not understand.
"""

import datetime
import re

# The window that scap gives the date histogram, in seconds. The records that
# this module makes sit on a grid of this size, so a bucket holds a count that
# the tests can predict.
BUCKET_SECONDS = 20

# The history that analyze() reads, in days, and the buckets of it.
HISTORY_DAYS = 90
HISTORY_BUCKETS = HISTORY_DAYS * 24 * 3600 // BUCKET_SECONDS

# How many records share one bucket, in a cycle. The counts differ, so the
# samples of analyze() have a spread, and the largest of them is the outlier
# that the analysis removes.
BUCKET_PATTERN = [1, 1, 2, 1, 1, 3, 1, 1, 2, 9]

# Records of the last window, which a check() counts, and records of the last
# 90 days, which analyze() counts.
#
# Each row is (namespace, release, wiki, channel, recent, older). "recent" is
# the number of records of the last window. "older" is the number that go
# across the history that analyze() reads.
#
# mw-web and mw-debug are the namespaces that train-dev deploys. mw-api-ext is
# not, so it witnesses a query that selects more than it names: a clause for
# mw-web alone must not count these. mw-pretrain holds the few records of a
# namespace with almost no traffic.
#
# The release names of mw-pretrain are those of the other namespaces, as
# production has them, so only the namespace separates the checks of a scope
# from the checks of another one.
K8S_RECORDS = [
    # namespace,     release,       wiki,       channel,     recent, older
    ("mw-web", "main", "enwiki", "exception", 3, 120),
    ("mw-web", "main", "testwiki", "error", 1, 18),
    ("mw-web", "canary", "enwiki", "exception", 1, 36),
    ("mw-api-ext", "main", "enwiki", "error", 2, 60),
    ("mw-api-ext", "canary", "enwiki", "exception", 1, 24),
    ("mw-debug", "pinkunicorn", "testwiki", "exception", 1, 12),
    ("mw-pretrain", "main", "testwiki", "error", 1, 6),
    ("mw-pretrain", "canary", "testwiki", "error", 1, 4),
]

# A record of a bare-metal host, which holds no kubernetes labels. The
# baremetal clause of scap selects it by the name of the host.
BAREMETAL_RECORDS = [
    # host,     wiki,      channel,     recent, older
    ("mw1450", "enwiki", "exception", 1, 30),
]

# Records that every check must leave out: a level that must_not excludes, and
# a channel that the channel clause does not name.
EXCLUDED_RECORDS = [
    # namespace, release, channel,     level
    ("mw-web", "main", "exception", "DEBUG"),
    ("mw-web", "main", "other", "ERROR"),
]

MESSAGES = [
    "[{reqId}] {exception_url}   Wikimedia\\Rdbms\\DBQueryError: Error 1205: "
    "Lock wait timeout exceeded",
    "[{reqId}] {exception_url}   TypeError: Argument 1 passed to "
    "MediaWiki\\Title::newFromText() must be of the type string",
    "[{reqId}] {exception_url}   Wikimedia\\Assert\\PreconditionException: "
    "Precondition failed",
]


def _timestamp(when) -> str:
    return when.strftime("%Y-%m-%dT%H:%M:%S.000Z")


def _record(when, wiki, channel, level="ERROR", index=0) -> dict:
    """One mediawiki record, in the shape that logstash holds."""
    message = MESSAGES[index % len(MESSAGES)]
    return {
        "@timestamp": _timestamp(when),
        "type": "mediawiki",
        "channel": channel,
        "level": level,
        "wiki": wiki,
        "normalized_message": message,
        "message": message.replace("{reqId}", "0c260e85").replace(
            "{exception_url}", "/w/api.php"
        ),
        "exception_url": "/w/api.php",
    }


def _group_sizes(older: int) -> list:
    """How many records each bucket of the history holds.

    >>> _group_sizes(6)
    [1, 1, 2, 1, 1]
    >>> sum(_group_sizes(37)) == 37
    True
    """
    res = []
    made = 0
    while made < older:
        share = min(BUCKET_PATTERN[len(res) % len(BUCKET_PATTERN)], older - made)
        res.append(share)
        made += share
    return res


def _spread(now, recent: int, older: int):
    """Yields the times of `recent` records of this window, then of `older`.

    A record of this window sits a few seconds in the past, so a check of the
    last BUCKET_SECONDS seconds counts it. The older records go across the
    whole history, in groups that BUCKET_PATTERN sizes, so that the samples of
    analyze() hold more than one count.

    >>> now = datetime.datetime(2026, 9, 4, tzinfo=datetime.timezone.utc)
    >>> times = list(_spread(now, 1, 12))
    >>> times[0] == now - datetime.timedelta(seconds=5)
    True
    >>> min(times) < now - datetime.timedelta(days=30)
    True

    An older record stays outside the window of a check, which reads from
    BUCKET_SECONDS ago and holds that moment as well.

    >>> window = now - datetime.timedelta(seconds=BUCKET_SECONDS)
    >>> max(times[1:]) < window
    True
    """
    for _ in range(recent):
        yield now - datetime.timedelta(seconds=5)

    groups = _group_sizes(older)
    # The groups go at even distances, so that a row of a few records covers
    # the history as widely as a row of many.
    step = max(2, HISTORY_BUCKETS // (len(groups) + 1))

    for index, share in enumerate(groups):
        when = now - datetime.timedelta(seconds=BUCKET_SECONDS * step * (index + 1))
        for _ in range(share):
            yield when


def records(now=None) -> list:
    """Returns a list of fake records, newest first.

    >>> len(records()) > 100
    True
    >>> sorted({r["type"] for r in records()})
    ['mediawiki']
    """
    now = now or datetime.datetime.now(datetime.timezone.utc)
    res = []

    for index, row in enumerate(K8S_RECORDS):
        namespace, release, wiki, channel, recent, older = row
        for when in _spread(now, recent, older):
            record = _record(when, wiki, channel, index=index)
            record["kubernetes"] = {
                "labels": {"deployment": namespace, "release": release}
            }
            res.append(record)

    for index, row in enumerate(BAREMETAL_RECORDS):
        host, wiki, channel, recent, older = row
        for when in _spread(now, recent, older):
            record = _record(when, wiki, channel, index=index)
            record["host"] = host
            res.append(record)

    for namespace, release, channel, level in EXCLUDED_RECORDS:
        record = _record(
            now - datetime.timedelta(seconds=5), "enwiki", channel, level=level
        )
        record["kubernetes"] = {"labels": {"deployment": namespace, "release": release}}
        res.append(record)

    res.sort(key=lambda record: record["@timestamp"], reverse=True)
    return res


###################
# Query evaluation #
###################


class UnsupportedQuery(Exception):
    """A query that this module does not evaluate."""


def _field(record, path: str):
    """The value of a dotted field path, or None.

    A ".keyword" suffix names the same value as the field itself.

    >>> _field({"kubernetes": {"labels": {"release": "main"}}},
    ...        "kubernetes.labels.release.keyword")
    'main'
    """
    if path.endswith(".keyword"):
        path = path[: -len(".keyword")]

    value = record
    for part in path.split("."):
        if not isinstance(value, dict) or part not in value:
            return None
        value = value[part]
    return value


def _clause_matches(record, clause: dict, now) -> bool:
    ((kind, body),) = clause.items()

    if kind == "match_none":
        return False

    if kind == "term":
        ((path, wanted),) = body.items()
        return _field(record, path) == wanted

    if kind == "terms":
        ((path, wanted),) = body.items()
        return _field(record, path) in wanted

    if kind == "range":
        ((path, bounds),) = body.items()
        return _in_range(record, path, bounds, now)

    if kind == "bool":
        return _bool_matches(record, body, now)

    raise UnsupportedQuery(f"unsupported clause {kind}")


def _bool_matches(record, body: dict, now) -> bool:
    """A bool clause, which scap nests to name the targets of a check."""
    for clause in body.get("filter", []):
        if not _clause_matches(record, clause, now):
            return False

    for clause in body.get("must_not", []):
        if _clause_matches(record, clause, now):
            return False

    should = body.get("should", [])
    if should:
        if body.get("minimum_should_match", 1) != 1:
            raise UnsupportedQuery("minimum_should_match must be 1")
        if not any(_clause_matches(record, clause, now) for clause in should):
            return False

    return True


def _in_range(record, path: str, bounds: dict, now) -> bool:
    value = _field(record, path)
    if value is None:
        return False

    when = _parse_time(value)

    for key, bound in bounds.items():
        moment = _parse_relative(bound, now)
        if key == "gte" and when < moment:
            return False
        if key == "lte" and when > moment:
            return False
        if key not in ("gte", "lte"):
            raise UnsupportedQuery(f"unsupported range bound {key}")

    return True


def _parse_time(text: str) -> datetime.datetime:
    return datetime.datetime.strptime(text, "%Y-%m-%dT%H:%M:%S.%f%z")


_RELATIVE_RE = re.compile(r"^now(?:-(?P<amount>\d+)(?P<unit>[smhd]))?$")

_UNITS = {"s": "seconds", "m": "minutes", "h": "hours", "d": "days"}


def _parse_relative(text: str, now) -> datetime.datetime:
    """The moment that a bound of the form "now" or "now-90d" names.

    >>> now = datetime.datetime(2026, 9, 4, tzinfo=datetime.timezone.utc)
    >>> _parse_relative("now-1d", now).isoformat()
    '2026-09-03T00:00:00+00:00'
    """
    match = _RELATIVE_RE.match(text)
    if not match:
        raise UnsupportedQuery(f"unsupported time {text!r}")
    if not match.group("amount"):
        return now
    unit = _UNITS[match.group("unit")]
    return now - datetime.timedelta(**{unit: int(match.group("amount"))})


def matches(record, query: dict, now=None) -> bool:
    """Whether a record matches the query of a request."""
    now = now or datetime.datetime.now(datetime.timezone.utc)
    ((kind, body),) = query.items()
    if kind != "bool":
        raise UnsupportedQuery(f"unsupported query {kind}")

    return _bool_matches(record, body, now)


############
# Responses #
############


def _bucket_key(record) -> int:
    """The start of the histogram bucket of a record, in milliseconds."""
    when = _parse_time(record["@timestamp"])
    seconds = int(when.timestamp())
    return (seconds - seconds % BUCKET_SECONDS) * 1000


def search(request: dict, now=None) -> dict:
    """Returns the response to one _search request.

    The response holds as many hits as "size" asks for. It holds an
    aggregation as well when the request sends one.
    """
    now = now or datetime.datetime.now(datetime.timezone.utc)
    matching = [
        record for record in records(now) if matches(record, request["query"], now)
    ]

    res = _hits_response(request, matching)

    if "aggs" in request:
        res["aggregations"] = _aggregations(request["aggs"], matching)

    return res


def _aggregations(aggs: dict, matching: list) -> dict:
    res = {}

    for name, body in aggs.items():
        if "composite" in body:
            res[name] = _composite_buckets(body["composite"], matching)
        elif "terms" in body:
            res[name] = _terms_buckets(body["terms"], matching)
        else:
            raise UnsupportedQuery(f"unsupported aggregation {sorted(body)}")

    return res


def _terms_buckets(terms: dict, matching: list) -> dict:
    """The values of one field, and a count for each. The largest comes first.

    The poller of scap asks for this one.
    """
    order = terms.get("order", {"_count": "desc"})
    if order != {"_count": "desc"}:
        raise UnsupportedQuery(f"unsupported terms order {order}")

    counts = {}
    for record in matching:
        value = _field(record, terms["field"])
        if value is not None:
            counts[value] = counts.get(value, 0) + 1

    ordered = sorted(counts.items(), key=lambda pair: (-pair[1], pair[0]))
    size = terms.get("size", 10)

    return {
        "doc_count_error_upper_bound": 0,
        "sum_other_doc_count": sum(count for _value, count in ordered[size:]),
        "buckets": [
            {"key": value, "doc_count": count} for value, count in ordered[:size]
        ],
    }


def _hits_response(request: dict, matching: list) -> dict:
    size = request.get("size", 10)
    return {
        "took": 7,
        "timed_out": False,
        "_shards": {"total": 1, "successful": 1, "skipped": 0, "failed": 0},
        "hits": {
            "total": {"value": len(matching), "relation": "eq"},
            "max_score": 0,
            "hits": [
                {
                    "_index": "logstash-mediawiki-1-7.0.0-1-2026.09.04",
                    "_id": f"fake{i}",
                    "_score": 0,
                    "_source": record,
                }
                for i, record in enumerate(matching[:size])
            ],
        },
    }


def _composite_buckets(composite: dict, matching: list) -> dict:
    size = composite["size"]
    after = composite.get("after", {}).get("timestamp")

    counts = {}
    for record in matching:
        key = _bucket_key(record)
        counts[key] = counts.get(key, 0) + 1

    keys = sorted(key for key in counts if after is None or key > after)
    page = keys[:size]

    res = {
        "buckets": [
            {"key": {"timestamp": key}, "doc_count": counts[key]} for key in page
        ]
    }
    if len(keys) > size:
        res["after_key"] = {"timestamp": page[-1]}

    return res
