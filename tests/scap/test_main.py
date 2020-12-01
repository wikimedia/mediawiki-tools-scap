from __future__ import absolute_import

import pytest
import os
from scap import cli


@pytest.fixture
def cmd(request):
    """Fully initialized class"""
    # This is quite convoluted.
    # We instantiate a dummy subclass from the factory.
    # By doing this, we return a fully configured
    # application.

    if not hasattr(request, "param"):
        app = cli.Application.factory(
            ["sync", "--is-wrong", "pinkunicorns", "are", "real"]
        )
    else:
        app = cli.Application.factory(request.param)

    # Do some basic initialization (see cli.Application.run)
    app._load_config()
    app._setup_loggers()
    return app


def test_init():
    expected_name = "sync"
    app = cli.Application(expected_name)
    assert app.program_name == expected_name


@pytest.mark.parametrize(
    "cmd",
    [
        ["sync", "-w", "7"],
        ["sync", "--canary-wait-time", "30"],
        ["sync", "-w", "92"],
        ["sync", "-D", "canary_wait_time:30"],
    ],
    indirect=True,
)
def test_scap_sync_flags(cmd, mocker):
    # Testing if we are parsing -w propertly. argparse makes sure cw
    # is int or exists if -w flag is set, so no need to test for those.
    isthere = mocker.patch("os.path.exists")
    isthere.return_value = True
    try:
        cmd.main(cmd.extra_arguments)
    except ValueError as ve:
        if "seconds" in str(ve):
            assert cmd.arguments.canary_wait_time in (7, 92)
        elif "defined" in str(ve):
            assert "canary_wait_time" in cmd.arguments.defines[0]
        else:
            assert 0, 'expected "seconds" or "defined" in ValueError, got: %s' % str(ve)
    # if we get a RuntimeError, means our args are ok
    except RuntimeError as re:
        assert "scap-global-lock" in str(re)


def test_increment_stat(cmd, mocker):
    assert cmd._stats is None
    stats = mocker.patch.object(cmd, "_stats")

    # Case 1
    cmd.increment_stat("scap")
    stats.increment.assert_any_call("deploy.scap", 1)
    stats.increment.assert_any_call("deploy.all", 1)

    # Case 2
    mocker.resetall()
    cmd.increment_stat("sync-file", all_stat=False, value=5)
    stats.increment.assert_called_once_with("deploy.sync-file", 5)


api_canaries = ["pigeon.w.o", "crow.w.o", "cormorant.w.o", "flamingo.w.o"]
app_canaries = ["birdy.w.o", "hawk.w.o", "duck.w.o", "chick.w.o"]


def test__get_api_canary_list(cmd, mocker):
    tl = mocker.patch("scap.targets.get")
    tl.return_value.all = api_canaries
    assert cmd._get_api_canary_list() == api_canaries


def test__get_app_canary_list(cmd, mocker):
    tl = mocker.patch("scap.targets.get")
    tl.return_value.all = app_canaries
    assert cmd._get_app_canary_list() == app_canaries


def test__get_canary_list(cmd, mocker):
    canary_targets = mocker.patch("scap.targets.get")

    def side_effect_cb(label, config):
        tgt = mocker.MagicMock()
        if label == "dsh_api_canaries":
            all = api_canaries
        else:
            all = app_canaries
        tgt.all = all
        return tgt

    canary_targets.side_effect = side_effect_cb
    assert sorted(cmd._get_canary_list()) == sorted(app_canaries + api_canaries)


@pytest.mark.parametrize("lock_exists", [True, False])
def test__check_sync_flag(cmd, mocker, lock_exists):
    isthere = mocker.patch("os.path.exists")
    ms = mocker.patch("os.stat")
    ms.return_value = os.stat("/bin/bash")
    mu = mocker.patch("pwd.getpwuid")
    mu.return_value.pw_name = "scap"
    if lock_exists:
        isthere.return_value = lock_exists
        with pytest.raises(IOError) as excinfo:
            cmd._check_sync_flag()
        assert "Blocked" in str(excinfo.value)
    else:
        isthere.return_value = False
        cmd._check_sync_flag()
