from datetime import datetime
import getpass
import io
import os
import tempfile

from scap import history


def test_strip_common_dirname():
    repos = [
        "http://foo.example/a/repo",
        "http://foo.example/b/repo",
    ]

    assert history.strip_common_dirname(repos) == [
        "a/repo",
        "b/repo",
    ]


def test_update_latest():
    with tempfile.TemporaryDirectory() as tmpdir:
        log = os.path.join(tmpdir, "history.log")
        with open(log, "w") as f:
            f.write(
                (
                    "{"
                    '"checkouts":{"a/repo":{"a-branch":{"a-dir":"foo123"}}},'
                    '"completed":true,'
                    '"timestamp":"2022-03-01 01:02:03",'
                    '"username":"scappy"'
                    "}\n"
                    "{"
                    '"checkouts":{"a/repo":{"a-branch":{"a-dir":"bar123"}}},'
                    '"completed":true,'
                    '"timestamp":"2022-03-03 01:02:03",'
                    '"username":"scappy"'
                    "}\n"
                )
            )

        history.update_latest(log, synced=True)

        with open(log, "r") as f:
            assert f.read() == (
                "{"
                '"checkouts":{"a/repo":{"a-branch":{"a-dir":"foo123"}}},'
                '"completed":true,'
                '"timestamp":"2022-03-01 01:02:03",'
                '"username":"scappy"'
                "}\n"
                "{"
                '"checkouts":{"a/repo":{"a-branch":{"a-dir":"bar123"}}},'
                '"completed":true,'
                '"synced":true,'
                '"timestamp":"2022-03-03 01:02:03",'
                '"username":"scappy"'
                "}\n"
            )


def test_history_load():
    hist = history.History.load(
        io.StringIO(
            (
                "{"
                '"completed":true,'
                '"timestamp":"2022-03-01 01:02:03",'
                '"checkouts":{"a/repo":{"a-branch":{"a-dir":"foo123"}}},'
                '"username":"scappy"'
                "}\n"
                "{"
                '"completed":true,'
                '"timestamp":"2022-03-03 01:02:03",'
                '"checkouts":{"a/repo":{"a-branch":{"a-dir":"bar123"}}},'
                '"username":"scappy"'
                "}\n"
            )
        )
    )

    assert hist.entries == [
        history.Entry(
            username="scappy",
            timestamp=datetime(2022, 3, 1, 1, 2, 3, tzinfo=None),
            checkouts={"a/repo": {"a-branch": {"a-dir": "foo123"}}},
            completed=True,
        ),
        history.Entry(
            username="scappy",
            timestamp=datetime(2022, 3, 3, 1, 2, 3, tzinfo=None),
            checkouts={"a/repo": {"a-branch": {"a-dir": "bar123"}}},
            completed=True,
        ),
    ]


def test_entry_dumps():
    entry = history.Entry(
        username="scappy",
        timestamp=datetime(2022, 2, 18, 18, 29, 8, tzinfo=None),
        checkouts={
            "a/repo": {"a-branch": {"a-dir": "xyz123"}},
        },
    )

    assert entry.dumps() == (
        "{"
        '"checkouts":{"a/repo":{"a-branch":{"a-dir":"xyz123"}}},'
        '"timestamp":"2022-02-18 18:29:08",'
        '"username":"scappy"'
        "}"
    )


def test_entry_loads():
    entry = history.Entry.loads(
        """
        {
            "username": "scappy",
            "timestamp": "2022-02-18 18:29:08",
            "checkouts": {
                "a/repo": {"a-branch": {"a-dir": "xyz123"}},
                "b/repo": {"b-branch": {"b-dir": "xyz123"}}
            }
        }
        """
    )

    assert entry.username == "scappy"
    assert entry.timestamp == datetime(2022, 2, 18, 18, 29, 8, tzinfo=None)
    assert entry.checkouts == {
        "a/repo": {"a-branch": {"a-dir": "xyz123"}},
        "b/repo": {"b-branch": {"b-dir": "xyz123"}},
    }


def test_entry_now():
    entry = history.Entry.now()

    assert entry.username == getpass.getuser()
    assert isinstance(entry.timestamp, datetime)
    assert not entry.completed


def test_entry_lookup():
    entry = history.Entry(
        checkouts={
            "a/repo": {"a-branch": {"a-dir": "abc123"}},
            "b/repo": {"b-branch": {"b-dir": "xyz123"}},
        }
    )

    assert entry.lookup("a/repo", "a-branch", "a-dir") == "abc123"
    assert entry.lookup("b/repo", "b-branch", "b-dir") == "xyz123"
    assert entry.lookup("does-not-exist", "foo", "foo-dir") is None


def test_entry_update():
    entry = history.Entry(
        checkouts={
            "a/repo": {"a-branch": {"a-dir": "abc123"}},
            "b/repo": {"b-branch": {"b-dir": "xyz123"}},
        }
    )

    entry.update("b/repo", "b-branch", "b-dir", "foo")

    assert entry.lookup("b/repo", "b-branch", "b-dir") == "foo"


def test_entry_branch_heads():
    entry = history.Entry(
        checkouts={
            "http://foo.example/a/repo": {
                "aa-branch": {"a-dir": "abc123"},
                "ab-branch": {"a-dir": "abc321"},
            },
            "http://foo.example/b/repo": {"bb-branch": {"b-dir": "xyz123"}},
            "http://foo.example/c/repo": {"cc-branch": {"c-dir": "xyz321"}},
        }
    )

    assert entry.branch_heads(["a/repo", "c/repo"]) == {
        "aa-branch": "abc123",
        "ab-branch": "abc321",
        "cc-branch": "xyz321",
    }
