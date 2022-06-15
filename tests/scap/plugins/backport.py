import shlex
from unittest import mock
from unittest.mock import patch, ANY

import scap.cli


@patch.dict('os.environ', clear=True)
def test_backport_uses_gerrit_push_user_config():
    scap_backport = scap.cli.Application.factory(["backport"])
    scap_backport.setup()

    with mock.patch('subprocess.check_call') as check:
        scap_backport._gerrit_ssh([])
        check.assert_called_with(ANY, env={}, stdout=ANY, stderr=ANY)

    with mock.patch('subprocess.check_call') as check:
        scap_backport.config["gerrit_push_user"] = "trainbotuser"
        scap_backport._gerrit_ssh([])

        # With python 3.8 we could retrieve kwargs directly
        # env = check.mock_calls[0].kwargs['env']
        name, args, kwargs = check.mock_calls[0]
        assert 'GIT_SSH_COMMAND' in kwargs['env']

        ssh_command = shlex.split(kwargs['env']['GIT_SSH_COMMAND'])
        assert '-oUser=trainbotuser' in ssh_command
        assert '-n' not in ssh_command
