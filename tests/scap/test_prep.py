import os.path
from unittest.mock import patch

import pytest
import scap.cli

scap_prep_dest_dir = "/dest"


@pytest.fixture()
def gerrit_url():
    yield "https://gerrit.wikimedia.org/r"


@pytest.fixture()
def gerrit_push_url():
    yield "ssh://review.example.org"


@pytest.fixture()
def scap_prep_git(gerrit_url, gerrit_push_url):
    scap_prep = scap.cli.Application.factory(["prep", "1.99.0-wmf.33"])
    scap_prep.setup(use_global_config=False)

    # config has been loaded
    assert "gerrit_push_url" in scap_prep.config
    assert scap_prep.config["gerrit_push_url"] == "ssh://gerrit.wikimedia.org:29418/"

    # change config to ensure it is taken in account
    scap_prep.config["gerrit_push_url"] = gerrit_push_url

    repo_name = "some/repository"

    with patch("scap.prep.git") as git:
        scap_prep._clone_or_update_repo(
            os.path.join(scap_prep.config["gerrit_url"], repo_name),
            "wmf/1.99.0.wmf-33",
            scap_prep_dest_dir,
            None,
        )
        yield git


def test_scap_prep_sets_a_push_url(scap_prep_git):
    scap_prep_git.gitcmd.assert_any_call(
        "remote",
        "set-url",
        "--push",
        "origin",
        os.path.join("ssh://review.example.org", "some/repository"),
        cwd=scap_prep_dest_dir,
    )


@pytest.mark.parametrize(
    "gerrit_push_url",
    [
        "ssh://review.example.org",
        "ssh://review.example.org/",
    ],
)
@pytest.mark.parametrize(
    "gerrit_url",
    [
        "https://gerrit.wikimedia.org/r",
        "https://gerrit.wikimedia.org/r/",
    ],
)
def test_scap_prep_pushInsteadOf_urls_are_normalized(scap_prep_git):
    scap_prep_git.gitcmd.assert_any_call(
        "submodule",
        "foreach",
        "git",
        "config",
        "--local",
        "--replace-all",
        "url.ssh://review.example.org.pushInsteadOf",
        "https://gerrit.wikimedia.org/r",
        cwd=scap_prep_dest_dir,
    )
