import json
import subprocess
import tempfile
from logging import Logger
from unittest import mock
from unittest.mock import patch

import pytest
import requests
from requests import HTTPError, Response

from scap import main, utils
from scap.deploy_promote import DeployPromote
from scap.runcmd import FailedCommand, gitcmd

messages_tests = [
    (
        "T777",
        "group3 to 1.42.0-wmf.00  refs T777",
        ("group3 to 1.42.0-wmf.00\n" "\n" "Bug: T777"),
    ),
]


@pytest.fixture
@patch.object(DeployPromote, "__init__", return_value=None)
def deploy_promote(init):
    dp = DeployPromote()
    dp.config = {}
    return dp


@pytest.mark.parametrize("task,announce,commit", messages_tests)
def test_set_messages(task, announce, commit, deploy_promote):
    p = get_deploy_promote_with_messages(task, deploy_promote)

    assert p.announce_message == announce
    assert p.commit_message == commit
    assert "\n" not in p.announce_message


def get_deploy_promote_with_messages(task, p):
    version = "1.42.0-wmf.00"
    train_info = {
        "version": version,
        "task_id": task,
        "status": "open",
        "date": "2025-07-15",
    }

    with tempfile.NamedTemporaryFile(mode="w") as f:
        json.dump(train_info, f)
        f.flush()

        p.config["train_blockers_url"] = "file://{}".format(f.name)
        p.config["web_proxy"] = None
        p.group = "group3"
        p.promote_version = version

        p._set_messages()

    return p


DEPLOYMENT_INFO = {
    "common": [
        {
            "repo": "https://gerrit.example/mediawiki-config",
            "branch": "master",
            "commit_ref": "abc123",
        }
    ],
    "versions": {
        "1.39.0-wmf.19": [
            {
                "repo": "https://gerrit.example/mediawiki/core",
                "branch": "wmf/1.39.0-wmf.19",
                "commit_ref": "def456",
            },
            {
                "repo": "https://gerrit.example/a-library",
                "commit_ref": "789abc",
            },
        ]
    },
}


def wiki_response(version="1.39.0-wmf.19"):
    return {
        "dbname": "testwiki",
        "version": version,
        "branch": "wmf/%s" % version,
        "checkouts": DEPLOYMENT_INFO["common"]
        + DEPLOYMENT_INFO["versions"]["1.39.0-wmf.19"],
    }


def test_version_check(deploy_promote, tmp_path):
    deploy_promote.logger = mock.MagicMock(Logger)
    deploy_promote.promote_version = "1.39.0-wmf.19"
    deploy_promote.config["stage_dir"] = str(tmp_path)

    with open(tmp_path / "deployment-info.json", "w") as f:
        json.dump(DEPLOYMENT_INFO, f)

    with mock.patch.object(requests, "get") as mock_get:
        mock_get.return_value = mock.MagicMock(Response)

        # Set the check versions timeout to zero so that these tests will complete quickly.
        with mock.patch.object(
            deploy_promote, "_get_check_versions_timeout", return_value=0
        ):
            # Version and checkouts match
            mock_get.return_value.json.return_value = wiki_response()
            deploy_promote._check_versions()

            # Checkouts match in any order
            response = wiki_response()
            response["checkouts"] = list(reversed(response["checkouts"]))
            mock_get.return_value.json.return_value = response
            deploy_promote._check_versions()

            # Version does not match
            mock_get.return_value.json.return_value = wiki_response("1.39.0-wmf.18")
            with pytest.raises(SystemExit):
                deploy_promote._check_versions()

            # Version matches but a checkout is on a different commit
            response = wiki_response()
            response["checkouts"] = [
                dict(response["checkouts"][0], commit_ref="0000000")
            ] + response["checkouts"][1:]
            mock_get.return_value.json.return_value = response
            with pytest.raises(SystemExit):
                deploy_promote._check_versions()

            # The wiki reports no deployment information
            mock_get.return_value.json.return_value = {}
            with pytest.raises(SystemExit):
                deploy_promote._check_versions()

            # Request failed
            with mock.patch.object(
                mock_get.return_value, "raise_for_status"
            ) as mock_raise:
                mock_raise.side_effect = HTTPError("500 Server Error")

                with pytest.raises(SystemExit):
                    deploy_promote._check_versions()


def test_version_check_without_staged_version(deploy_promote, tmp_path):
    deploy_promote.logger = mock.MagicMock(Logger)
    deploy_promote.promote_version = "1.39.0-wmf.20"
    deploy_promote.config["stage_dir"] = str(tmp_path)

    with open(tmp_path / "deployment-info.json", "w") as f:
        json.dump(DEPLOYMENT_INFO, f)

    with pytest.raises(SystemExit):
        deploy_promote._check_versions()


@pytest.mark.parametrize(
    "push,error",
    [
        (
            {"side_effect": subprocess.CalledProcessError(1, ["git", "push"])},
            subprocess.CalledProcessError,
        ),
        # The push output has no change number
        ({"return_value": None}, SystemExit),
    ],
)
def test_push_failure_removes_the_local_commit(deploy_promote, tmp_path, push, error):
    gitcmd("init", "--quiet", cwd=tmp_path)
    gitcmd("config", "user.name", "Test", cwd=tmp_path)
    gitcmd("config", "user.email", "test@example.org", cwd=tmp_path)
    gitcmd("commit", "--quiet", "--allow-empty", "-m", "Initial", cwd=tmp_path)
    initial = gitcmd("rev-parse", "HEAD", cwd=tmp_path).strip()
    gitcmd(
        "commit",
        "--quiet",
        "--allow-empty",
        "-m",
        "group1 to 1.42.0-wmf.00\n\nChange-Id: I123",
        cwd=tmp_path,
    )
    deploy_promote.promote_version = "1.42.0-wmf.00"
    deploy_promote._gerritssh = mock.Mock()
    deploy_promote._gerritssh.push_and_collect_change_number.configure_mock(**push)

    with utils.cd(str(tmp_path)):
        with pytest.raises(error):
            deploy_promote._push_patch()

    assert gitcmd("rev-parse", "HEAD", cwd=tmp_path).strip() == initial


@pytest.mark.parametrize(
    "returncode,change_id,reverts",
    [
        (main.ROLLED_BACK_STATUS, "Change-Id: I123", True),
        (1, "Change-Id: I123", False),
        # deploy-promote made no change, so there is nothing to revert
        (main.ROLLED_BACK_STATUS, None, False),
    ],
)
def test_sync_versions_reverts_after_rollback(
    deploy_promote, returncode, change_id, reverts
):
    deploy_promote.logger = mock.MagicMock(Logger)
    deploy_promote.arguments = mock.Mock(pause_after_testserver_sync=False)
    deploy_promote.group = "group1"
    deploy_promote.announce_message = "group1 to 1.42.0-wmf.00  refs T777"
    deploy_promote.version_update_change_id = change_id

    error = subprocess.CalledProcessError(returncode, ["scap", "sync-wikiversions"])
    with (
        mock.patch.object(deploy_promote, "scap_check_call", side_effect=error),
        mock.patch.object(deploy_promote, "_revert_version_update_patch") as revert,
    ):
        with pytest.raises(subprocess.CalledProcessError):
            deploy_promote._sync_versions()

    assert revert.called is reverts


def make_config_repo(path):
    gitcmd("init", "--quiet", cwd=path)
    gitcmd("config", "user.name", "Test", cwd=path)
    gitcmd("config", "user.email", "test@example.org", cwd=path)
    (path / "wikiversions.json").write_text('{"enwiktionary": "php-1.42.0-wmf.99"}\n')
    gitcmd("add", "wikiversions.json", cwd=path)
    gitcmd("commit", "--quiet", "-m", "Initial", cwd=path)
    (path / "wikiversions.json").write_text('{"enwiktionary": "php-1.42.0-wmf.00"}\n')
    gitcmd(
        "commit",
        "--quiet",
        "-a",
        "-m",
        "group1 to 1.42.0-wmf.00\n\nBug: T777\nChange-Id: I123",
        cwd=path,
    )


def test_commit_revert(deploy_promote, tmp_path):
    make_config_repo(tmp_path)
    promote_commit = gitcmd("rev-parse", "HEAD", cwd=tmp_path).strip()
    deploy_promote.commit_message = "group1 to 1.42.0-wmf.00\n\nBug: T777"
    deploy_promote.version_update_change_id = "Change-Id: I123"

    with utils.cd(str(tmp_path)):
        deploy_promote._commit_revert()

    assert (tmp_path / "wikiversions.json").read_text() == (
        '{"enwiktionary": "php-1.42.0-wmf.99"}\n'
    )
    assert gitcmd("log", "-1", "--format=%B", cwd=tmp_path).strip() == (
        'Revert "group1 to 1.42.0-wmf.00"\n'
        "\n"
        f"This reverts commit {promote_commit}.\n"
        "\n"
        "Bug: T777"
    )


def test_commit_revert_with_conflict(deploy_promote, tmp_path):
    make_config_repo(tmp_path)
    (tmp_path / "wikiversions.json").write_text(
        '{"enwiktionary": "php-1.42.0-wmf.01"}\n'
    )
    gitcmd("commit", "--quiet", "-a", "-m", "group1 to 1.42.0-wmf.01", cwd=tmp_path)
    head = gitcmd("rev-parse", "HEAD", cwd=tmp_path).strip()
    deploy_promote.commit_message = "group1 to 1.42.0-wmf.00\n\nBug: T777"
    deploy_promote.version_update_change_id = "Change-Id: I123"

    with utils.cd(str(tmp_path)):
        with pytest.raises(FailedCommand):
            deploy_promote._commit_revert()

    assert gitcmd("rev-parse", "HEAD", cwd=tmp_path).strip() == head
    assert gitcmd("status", "--porcelain", cwd=tmp_path) == ""


@pytest.mark.parametrize(
    "failing_step,action",
    [
        (
            "_commit_revert",
            "You must revert http://gerrit.example/r/105 in Gerrit and merge the revert",
        ),
        (
            "_push_patch",
            "You must revert http://gerrit.example/r/105 in Gerrit and merge the revert",
        ),
        (
            "_merge_patch",
            "You must make sure that http://gerrit.example/r/106 is merged",
        ),
    ],
)
def test_revert_failure_names_the_change(
    deploy_promote, tmp_path, failing_step, action
):
    deploy_promote.logger = mock.MagicMock(Logger)
    deploy_promote.config["stage_dir"] = str(tmp_path)
    deploy_promote.config["gerrit_url"] = "http://gerrit.example/"
    deploy_promote.commit_message = "group1 to 1.42.0-wmf.00\n\nBug: T777"
    deploy_promote.version_update_change_number = "105"
    steps = {
        "_commit_revert": mock.Mock(),
        "_push_patch": mock.Mock(return_value=("Change-Id: I456", "106")),
        "_merge_patch": mock.Mock(),
        "alert": mock.Mock(),
    }
    steps[failing_step].side_effect = SystemExit("Aborting: test failure")

    with mock.patch.multiple(deploy_promote, **steps):
        deploy_promote._revert_version_update_patch()

    deploy_promote.logger.error.assert_called_once_with(
        'Could not revert "group1 to 1.42.0-wmf.00": Aborting: test failure'
    )
    steps["alert"].assert_called_once_with(
        'The revert of "group1 to 1.42.0-wmf.00" failed.\n\n'
        f'MANUAL RECOVERY NEEDED: {action}, then run "scap prep auto".',
        "Acknowledge",
    )
