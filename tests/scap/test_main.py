from __future__ import absolute_import

import pytest

from scap import cli


@pytest.fixture
def cmd(request):
    """Fully initialized class"""
    # This is quite convoluted.
    # We instantiate a dummy subclass from the factory.
    # By doing this, we return a fully configured
    # application.

    if not hasattr(request, 'param'):
        app = cli.Application.factory([
            'sync', '--is-wrong', 'pinkunicorns', 'are', 'real'
        ])
    else:
        app = cli.Application.factory(request.param)

    # Do some basic initialization (see cli.Application.run)
    app._load_config()
    app._setup_loggers()
    # app._setup_environ()

    return app


def test_init():
    expected_name = 'sync'
    app = cli.Application(expected_name)
    assert app.program_name == expected_name


def test_increment_stat(cmd, mocker):
    assert cmd._stats is None
    stats = mocker.patch.object(cmd, '_stats')

    # Case 1
    cmd.increment_stat('scap')
    stats.increment.assert_any_call('deploy.scap', 1)
    stats.increment.assert_any_call('deploy.all', 1)

    # Case 2
    mocker.resetall()
    cmd.increment_stat('sync-file', all_stat=False, value=5)
    stats.increment.assert_called_once_with('deploy.sync-file', 5)
