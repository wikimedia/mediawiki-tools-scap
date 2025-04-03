import pytest
from scap import cli


@pytest.fixture
def cmd(request):
    """Fully initialized class"""
    # This is quite convoluted.
    # We instantiate a dummy subclass from the factory.
    # By doing this, we return a fully configured
    # application.

    if not hasattr(request, "param"):
        app = cli.Application.factory(["sync-world"])
    else:
        app = cli.Application.factory(request.param)

    # Do some basic initialization
    app.setup()

    return app


def test_init():
    expected_name = "sync"
    app = cli.Application(expected_name)
    assert app.program_name == expected_name


@pytest.mark.parametrize(
    "cmd",
    [
        # Args ok
        ["sync-world", "-n", "-w", "7"],
        # Args ok
        ["sync-world", "-n", "--canary-wait-time", "30"],
        # Args ok
        ["sync-world", "-n", "-w", "92"],
        # Wrong way of specifying canary wait time
        ["sync-world", "-n", "-D", "canary_wait_time:30"],
    ],
    indirect=True,
)
def test_scap_sync_world_flags(cmd):
    # Testing if we are parsing -w propertly. argparse makes sure cw
    # is int or exists if -w flag is set, so no need to test for those.
    try:
        cmd.main(cmd.extra_arguments)
    except ValueError as ve:
        assert "canary_wait_time" in cmd.arguments.defines[0]
        assert "defined" in str(ve)


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


def test_exclude_wikiversions():
    sync_world_app = cli.Application.factory(["sync-world"])
    sync_world_app.setup()

    assert "--exclude-wikiversions.php" in sync_world_app._base_scap_pull_command()

    sync_wikiversions_app = cli.Application.factory(["sync-wikiversions"])
    sync_wikiversions_app.setup()

    assert (
        "--exclude-wikiversions.php"
        not in sync_wikiversions_app._base_scap_pull_command()
    )


def test_handle_exception(cmd, mocker):
    cmd.arguments = mocker.MagicMock()
    cmd.arguments.no_log_message = False

    # Mock the logger
    get_log = mocker.patch("logging.getLogger")
    announcer = get_log.return_value

    valueError = ValueError("test")

    assert cmd._handle_exception(valueError) == 1

    get_log.assert_called_with("scap.announce")
    assert cmd._announce_logger == announcer
    announcer.info.assert_called()
