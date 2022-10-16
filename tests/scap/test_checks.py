import logging
import select
import unittest
import pytest
from io import StringIO

from scap import checks


def test_load_valid_config():
    chks = checks.load(
        {
            "checks": {
                "foo": {
                    "type": "command",
                    "command": "/bin/true",
                    "stage": "promote",
                    "timeout": 60,
                },
                "bar": {"type": "command", "command": "/bin/false", "stage": "promote"},
            }
        }
    )

    assert len(chks) == 2

    assert "foo" in chks
    assert isinstance(chks["foo"], checks.Check)
    assert chks["foo"].stage == "promote"
    assert chks["foo"].after == "promote", "Set after as well as stage"
    assert chks["foo"].command == "/bin/true"

    assert "bar" in chks
    assert isinstance(chks["bar"], checks.Check)
    assert chks["bar"].stage == "promote"
    assert chks["foo"].after == "promote", "Set after as well as stage"
    assert chks["bar"].command == "/bin/false"


def test_load_before():
    chks = checks.load({
        "checks": {
            "prepare":
                {"type": "command", "command": "/bin/true", "before": "fetch"}
        }
    })
    assert len(chks) == 1
    assert chks["prepare"].before == 'fetch'
    assert chks["prepare"].stage == 'fetch', "Sets stage for back compat"


def test_load_after():
    chks = checks.load({
        "checks": {
            "complete":
                {"type": "command", "command": "/bin/true", "after": "fetch"}
        }
    })
    assert len(chks) == 1
    assert chks["complete"].after == 'fetch'
    assert chks["complete"].stage == 'fetch', "Sets stage for back compat"


def test_load_after_overrides_stage():
    chks = checks.load({
        "checks": {
            "runs_after_promote":
                {"type": "command", "command": "/bin/true",
                 "stage": "fetch", "after": "promote"}
        }
    })
    assert len(chks) == 1
    assert chks["runs_after_promote"].after == 'promote'
    assert chks["runs_after_promote"].stage == 'promote'


def test_load_before_overrides_stage():
    chks = checks.load({
        "checks": {
            "runs_before_promote":
                {"type": "command", "command": "/bin/true",
                 "before": "promote"}
        }
    })
    assert len(chks) == 1
    assert chks["runs_before_promote"].before == 'promote'
    assert chks["runs_before_promote"].stage == 'promote'


def test_load_tolerates_empty_config():
    chks = checks.load("")

    assert not chks


def test_load_tolerates_empty_checks():
    chks = checks.load({checks: {}})

    assert not chks


def test_load_bad_type():
    with pytest.raises(checks.CheckInvalid):
        checks.load({"checks": {"foo": {"type": "badtype"}}})


def test_custom_type():
    @checks.checktype("custom")
    class CustomCheck(checks.Check):
        pass

    try:
        chks = checks.load(
            {
                "checks": {
                    "foo": {
                        "type": "custom",
                        "command": "/bin/true",
                        "stage": "promote",
                    }
                }
            }
        )

        assert len(chks) == 1

        assert "foo" in chks
        assert isinstance(chks["foo"], CustomCheck)
        assert chks["foo"].stage == "promote"
        assert chks["foo"].command == "/bin/true"

    finally:
        del checks._TYPES["custom"]


@unittest.skipUnless(hasattr(select, "epoll"), "Platforms with epoll only")
class ChecksExecuteTest(unittest.TestCase):
    def setUp(self):
        self.log_stream = StringIO()
        self.log_handler = logging.StreamHandler(self.log_stream)
        self.logger = logging.getLogger("test logger")
        self.logger.setLevel(logging.INFO)
        self.logger.handlers = [self.log_handler]

    def assert_logged(self, msg):
        assert msg in self.log_stream.getvalue()

    def test_execute(self):
        chks = [checks.Check("foo", stage="x", command="echo foo test")]
        result, done = checks.execute(chks, logger=self.logger)

        assert len(done) == 1
        assert result

    def test_execute_failure(self):
        chks = [checks.Check("foo", stage="x", command="false")]

        result, done = checks.execute(chks, logger=self.logger)

        assert len(done) == 1
        assert not result
        self.assert_logged("Check 'foo' failed")

    def test_execute_concurrency(self):
        chks = [
            checks.Check("foo{}".format(i), stage="x", command="sleep 0.1")
            for i in range(4)
        ]

        result, done = checks.execute(chks, logger=self.logger, concurrency=3)

        assert len(done) == 4
        assert result

        assert (done[1].started - done[0].started) < 0.1
        assert (done[2].started - done[0].started) < 0.1
        assert (done[3].started - done[0].started) > 0.1

    def test_execute_timeout(self):
        chks = [checks.Check("foo", stage="x", command="sleep 0.1", timeout=0.01)]

        result, done = checks.execute(chks, logger=self.logger)

        assert not done
        assert not result
        self.assert_logged("Check 'foo' exceeded 0.01s timeout")
