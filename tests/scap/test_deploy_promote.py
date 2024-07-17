import json
import shlex
import tempfile
from logging import Logger
from unittest import mock
from unittest.mock import patch

import pytest
import requests
from requests import HTTPError, Response

import scap.cli
from scap.deploy_promote import DeployPromote

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


@patch.object(DeployPromote, "_get_git_push_dest", return_value="master")
@patch("scap.deploy_promote._commit_arrived_to_remote", return_value="true")
@patch.dict("os.environ", clear=True)
def test_push_patch(*args):
    scap_deploy_promote = scap.cli.Application.factory(
        ["deploy-promote", "-D", "ssh_auth_sock:/keyholder.sock", "group1"]
    )
    scap_deploy_promote.setup()
    scap_deploy_promote.logger = scap_deploy_promote.get_logger()

    with patch("scap.deploy_promote.gitcmd") as gitcmd:
        # First test
        gitcmd.side_effect = [
            None,  # git push
            "Change-Id: I1234EF",  # git log -1
            None,  # git reset
        ]
        scap_deploy_promote.config["gerrit_push_user"] = "trainbotuser"
        scap_deploy_promote._push_patch()

        gitcmd.assert_any_call("push", "origin", "HEAD:master", env=mock.ANY)

        # With python 3.8 we could retrieve kwargs directly:
        #  push_env = gitcmd.mock_calls[0].kwargs['env']
        name, args, kwargs = gitcmd.mock_calls[0]
        push_env = kwargs["env"]

        assert "GIT_SSH_COMMAND" in push_env
        # Shared user relies on keyholder proxy, we better have it in the env
        assert "SSH_AUTH_SOCK" in push_env
        assert push_env["SSH_AUTH_SOCK"] == "/keyholder.sock"

        ssh_command = shlex.split(push_env["GIT_SSH_COMMAND"])
        assert "-oUser=trainbotuser" in ssh_command
        assert "-n" not in ssh_command

        # Second test
        gitcmd.reset_mock()
        gitcmd.side_effect = [
            None,  # git push
            "Change-Id: I1234EF",  # git log -1
            None,  # git reset
        ]
        scap_deploy_promote.config["gerrit_push_user"] = None
        scap_deploy_promote._push_patch()

        gitcmd.assert_any_call("push", "origin", "HEAD:master", env=mock.ANY)

        # With python 3.8 we could retrieve kwargs directly:
        #  push_env = gitcmd.mock_calls[0].kwargs['env']
        name, args, kwargs = gitcmd.mock_calls[0]
        push_env = kwargs["env"]

        assert "GIT_SSH_COMMAND" not in push_env


def test_version_check(deploy_promote):
    deploy_promote.logger = mock.MagicMock(Logger)
    deploy_promote.promote_version = "1.39.0-wmf.19"

    with mock.patch.object(requests, "get") as mock_get:
        mock_get.return_value = mock.MagicMock(Response)

        # Set the check versions timeout to zero so that these tests will complete quickly.
        with mock.patch.object(
            deploy_promote, "_get_check_versions_timeout", return_value=0
        ):
            # Version matches
            mock_get.return_value.text = (
                '<meta name="generator" content="MediaWiki 1.39.0-wmf.19"/>'
            )
            deploy_promote._check_versions()

            # Version does not match
            mock_get.return_value.text = (
                '<meta name="generator" content="MediaWiki NoVersTooBad"/>'
            )
            with pytest.raises(SystemExit):
                deploy_promote._check_versions()

            # Version could not be found in page
            mock_get.return_value.text = "garbled nonsense dadadddd"
            with pytest.raises(SystemExit):
                deploy_promote._check_versions()

            # Request failed
            with mock.patch.object(
                mock_get.return_value, "raise_for_status"
            ) as mock_raise:
                http_error = HTTPError()
                http_error.response = mock.MagicMock(Response)
                http_error.response.status_code = 500
                mock_raise.side_effect = http_error

                with pytest.raises(SystemExit):
                    deploy_promote._check_versions()
