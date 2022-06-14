import shlex
from unittest.mock import patch
from unittest import mock

import scap.cli
from scap.deploy_promote import DeployPromote
import pytest

messages_tests = [
    (
        "",
        "group3 wikis to 1.42.0-wmf.00",
        "group3 wikis to 1.42.0-wmf.00",
    ),
    (
        "T777",
        "group3 wikis to 1.42.0-wmf.00  refs T777",
        (
            "group3 wikis to 1.42.0-wmf.00\n"
            "\n"
            "Bug: T777"
        ),
     ),
]


@pytest.mark.parametrize("task,announce,commit", messages_tests)
def test_set_messages(task, announce, commit):
    p = get_DeployPromote_with_messages(task)

    assert p.announce_message == announce
    assert p.commit_message == commit
    assert "\n" not in p.announce_message


@patch.object(DeployPromote, '__init__', return_value=None)
def get_DeployPromote_with_messages(task, init):
    with patch.object(DeployPromote, '_get_train_task') as gtt:
        gtt.return_value = task

        p = DeployPromote()
        p.group = "group3"
        p.promote_version = "1.42.0-wmf.00"

        p._set_messages()

    return p


@patch.object(DeployPromote, '_get_git_push_dest', return_value='master')
@patch('scap.deploy_promote._commit_arrived_to_remote', return_value='true')
@patch.dict('os.environ', clear=True)
def test_push_patch(*args):

    scap_deploy_promote = scap.cli.Application.factory(
        ["deploy-promote", "group1"])
    scap_deploy_promote._load_config()
    scap_deploy_promote.config["ssh_auth_sock"] = "/keyholder.sock"
    scap_deploy_promote._setup_environ()
    scap_deploy_promote._setup_loggers()
    scap_deploy_promote.logger = scap_deploy_promote.get_logger()

    with patch('scap.deploy_promote.gitcmd') as gitcmd:
        # First test
        gitcmd.side_effect = [
            None,  # git push
            'Change-Id: I1234EF',  # git log -1
            None,  # git reset
        ]
        scap_deploy_promote.config["gerrit_push_user"] = "trainbotuser"
        scap_deploy_promote._push_patch()

        gitcmd.assert_any_call('push', 'origin', 'HEAD:master', env=mock.ANY)

        # With python 3.8 we could retrieve kwargs directly:
        #  push_env = gitcmd.mock_calls[0].kwargs['env']
        name, args, kwargs = gitcmd.mock_calls[0]
        push_env = kwargs['env']

        assert 'GIT_SSH_COMMAND' in push_env
        # Shared user relies on keyholder proxy, we better have it in the env
        assert 'SSH_AUTH_SOCK' in push_env
        assert push_env['SSH_AUTH_SOCK'] == '/keyholder.sock'

        ssh_command = shlex.split(push_env['GIT_SSH_COMMAND'])
        assert '-oUser=trainbotuser' in ssh_command
        assert '-n' not in ssh_command

        # Second test
        gitcmd.reset_mock()
        gitcmd.side_effect = [
            None,  # git push
            'Change-Id: I1234EF',  # git log -1
            None,  # git reset
        ]
        scap_deploy_promote.config["gerrit_push_user"] = None
        scap_deploy_promote._push_patch()

        gitcmd.assert_any_call('push', 'origin', 'HEAD:master', env=mock.ANY)

        # With python 3.8 we could retrieve kwargs directly:
        #  push_env = gitcmd.mock_calls[0].kwargs['env']
        name, args, kwargs = gitcmd.mock_calls[0]
        push_env = kwargs['env']

        assert 'GIT_SSH_COMMAND' not in push_env
