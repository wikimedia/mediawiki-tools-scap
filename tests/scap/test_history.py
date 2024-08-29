from datetime import datetime, timezone
import os

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


def test_history(tmpdir):
    db_filename = os.path.join(tmpdir, "history.db")
    hist = history.History(db_filename)
    co1 = history.Checkout(
        repo="http://foo.example/a/repo",
        branch="aa-branch",
        directory="a/aa",
        commit_ref="aaa123",
    )
    co2 = history.Checkout(
        repo="http://foo.example/a/repo",
        branch="ab-branch",
        directory="a/ab",
        commit_ref="abb321",
    )
    co3 = history.Checkout(
        repo="http://foo.example/b/repo",
        branch="b-branch",
        directory="bb-dir",
        commit_ref="bbb123",
    )
    co4 = history.Checkout(
        repo="http://foo.example/c/repo",
        branch="cc-branch",
        directory="c-dir",
        commit_ref="ccc321",
    )

    deployment = history.Deployment(
        starttime=datetime.now(timezone.utc),
        endtime=datetime.now(timezone.utc),
        username="bruce",
        completed=True,
        checkouts=[co1, co2, co3, co4],
    )

    assert deployment._branch_heads(["a/repo", "c/repo"]) == {
        "aa-branch": "aaa123",
        "ab-branch": "abb321",
        "cc-branch": "ccc321",
    }

    assert (
        deployment.lookup("http://foo.example/a/repo", "aa-branch", "a/aa") == "aaa123"
    )
    assert (
        deployment.lookup("http://foo.example/b/repo", "b-branch", "bb-dir") == "bbb123"
    )
    assert deployment.lookup("does-not-exist", "foo", "foo-dir") is None

    hist.log(deployment)
