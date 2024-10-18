import os
import sys
import tempfile

try:
    import mock
except ImportError:
    from unittest import mock

import pytest

from scap import arg, cli, lock, utils, version


@cli.command("dummy")
@mock.patch.dict("os.environ", clear=True)
class DummyApp(cli.Application):
    """Dummy derivative class"""

    @cli.argument("--is-wrong", action="store_true", help="test")
    @cli.argument("message", nargs="*", help="SAL")
    def main(self, *extra_args):
        return 666


@pytest.fixture
@mock.patch.dict("os.environ", clear=True)
def app(name="cmd.exe"):
    """Simple, non-initialized version of the class"""
    with mock.patch("time.time") as timezero:
        timezero.return_value = 0
        application = cli.Application(name)
    return application


@pytest.fixture
@mock.patch.dict("os.environ", clear=True)
def cmd(request):
    """Fully initialized class"""
    # This is quite convoluted.
    # We instantiate a dummy subclass from the factory.
    # By doing this, we return a fully configured
    # application.
    if not hasattr(request, "param"):
        return cli.Application.factory(
            ["dummy", "--is-wrong", "pinkunicorns", "are", "real"]
        )
    return cli.Application.factory(request.param)


names = [
    ("autoexec.bat", "autoexec.bat"),
    ("/tmp/test/sbin/something.sh", "something.sh"),
]


@pytest.mark.parametrize("app_name, expected_name", names)
def test_init(app_name, expected_name):
    app = cli.Application(app_name)
    assert app.program_name == expected_name
    assert app.exe_name == app_name
    # Reset the program_name
    app.program_name = None
    assert app._logger is None
    assert app._stats is None


def test_get_logger(app, mocker):
    logger = mocker.patch("logging.getLogger")
    assert logger.return_value == app.get_logger()
    logger.assert_called_with("cmd.exe")


def test_get_stats(app, mocker):
    assert app._stats is None
    # app.config is *None* at init time. inject the needed values
    app.config = {"statsd_host": "foo_bar", "statsd_port": "678"}
    stats = mocker.patch("scap.log.Stats")
    assert stats.return_value == app.get_stats()
    stats.assert_called_with("foo_bar", 678)


def test_get_lock_file(app):
    # N.B. 'lock_file' needs to be defined in any case in the config. Yuck.
    app.config = {"lock_file": None, "stage_dir": "/srv/mediawiki-staging"}
    # Case 1 - no lockfile nor a git_repo config
    assert app.get_lock_file() == "/var/lock/scap.srv_mediawiki-staging.lock"
    # Case 2 - git_repo defined
    app.config["git_repo"] = "parsoid/deploy"
    assert app.get_lock_file() == "/var/lock/scap.parsoid_deploy.lock"
    # Case 3 - lock file in config
    app.config["lock_file"] = "pinkunicorn"
    assert app.get_lock_file() == "pinkunicorn"


def test_verbose(app):
    # TODO: a fixture using factory?
    pass


def test_get_duration(app):
    assert app.start == 0
    with mock.patch("time.time") as t:
        t.return_value = 100
        assert app.get_duration() == 100


def test_get_script_path(app):
    sys.argv[0] = "/usr/local/sbin/test"
    assert app.get_script_path() == "/usr/local/sbin/scap"


def test_get_keyholder_key(app, mocker):
    exists = mocker.patch("os.path.exists")
    # Case 1: key doesn't exist on disk.
    exists.return_value = False
    app.config = {"ssh_user": "fuzz-buzz"}
    assert app.get_keyholder_key() is None
    # Case 2: key exists for the ssh user
    exists.return_value = True
    assert app.get_keyholder_key() == "/etc/keyholder.d/fuzz_buzz"
    # Case 3: keyholder key defined
    app.config["keyholder_key"] = "pinkunicorn"
    assert app.get_keyholder_key() == "/etc/keyholder.d/pinkunicorn"


def test_get_master_list(app, mocker):
    tl = mocker.patch("scap.targets.get")
    tl.return_value.all = [1, 2, 3]
    assert app.get_master_list() == [1, 2, 3]
    tl.assert_called_with(
        "dsh_masters", app.config, exclude_hosts=None, limit_hosts=None
    )


def test_announce(app, mocker):
    # Case 1: no_log_message is set
    app.arguments = mock.MagicMock()
    app.arguments.no_log_message = True
    app.get_logger = mock.MagicMock()
    logger = app.get_logger.return_value
    app.announce("test")
    logger.info.assert_called_with("test")
    logger.warning.assert_not_called()

    # Case 2: no_log_message is not set
    app.arguments.no_log_message = False
    get_log = mocker.patch("logging.getLogger")
    announcer = get_log.return_value
    app.announce("test %s", "bar")
    get_log.assert_called_with("scap.announce")
    assert app._announce_logger == announcer
    announcer.info.assert_called_with("test %s", "bar")


def test_get_get_realm_specific_filename():
    prefix = "get_realm_specific_filename_test"

    with tempfile.NamedTemporaryFile(prefix=prefix) as t1:
        tmpfile_no_realm = t1.name + ".json"
        tmpfile_with_realm = t1.name + "-testrealm.json"

        # Create empty files.  The contents don't matter
        for filename in (tmpfile_no_realm, tmpfile_with_realm):
            open(filename, "w").close()

        try:
            # Verify realm-specific filename is selected since it exists
            res = utils.get_realm_specific_filename(tmpfile_no_realm, "testrealm")
            assert res == tmpfile_with_realm

            os.remove(tmpfile_with_realm)
            # Now it doesn't exist, so we should get the no-realm file.
            res = utils.get_realm_specific_filename(tmpfile_no_realm, "testrealm")
            assert res == tmpfile_no_realm

            os.remove(tmpfile_no_realm)
            # If no file is found get_realm_specific_filename returns the
            # input filename.
            res = utils.get_realm_specific_filename(tmpfile_no_realm, "testrealm")
            assert res == tmpfile_no_realm

        finally:
            # Just in case.
            for filename in (tmpfile_no_realm, tmpfile_with_realm):
                if os.path.exists(filename):
                    os.remove(filename)


def test_active_wikiversions(app, mocker):
    app.config = {"deploy_dir": "dir", "wmf_realm": "realm", "datacenter": "dc"}
    ga = mocker.patch("scap.utils.get_active_wikiversions")
    app.active_wikiversions()
    ga.assert_called_with("dir", "realm", list)
    app.config["pinkunicorn_dir"] = "pinkunicorn"
    app.active_wikiversions("pinkunicorn")
    ga.assert_called_with("pinkunicorn", "realm", list)


def test_process_arguments_error(app):
    # Case 1: extra args present
    app._argparser = mock.MagicMock()
    app._argparser.error.side_effect = SystemExit(1)
    with pytest.raises(SystemExit):
        app._process_arguments([], ["you", "fail!"])

    app._argparser.error.assert_called_with("extra arguments found: you fail!")


def test_process_arguments(cmd):
    # Case 2: message defined
    assert cmd._process_arguments(cmd.arguments, None) == (cmd.arguments, None)


@pytest.mark.parametrize("cmd", [["dummy"]], indirect=True)
def test_process_arguments_nomsg(cmd):
    # Case 3: no message defined
    args, _ = cmd._process_arguments(cmd.arguments, None)
    assert args.message == "(no justification provided)"


@pytest.mark.parametrize(
    "cmd", [["dummy", "test", "-D", "canary_threshold:30"]], indirect=True
)
def test_load_config(cmd, mocker):
    loader = mocker.patch("scap.config.load")
    cmd._load_config(use_global_config=False)
    loader.assert_called_with(
        cfg_file=cmd.arguments.conf_file,
        environment=cmd.arguments.environment,
        overrides={"canary_threshold": "30"},
        use_global_config=False,
        use_local_config=True,
    )


@pytest.mark.parametrize("cmd", [["dummy", "test", "--no-local-config"]], indirect=True)
def test_load_config_no_local_config(cmd, mocker):
    loader = mocker.patch("scap.config.load")
    cmd._load_config(use_global_config=False)
    loader.assert_called_with(
        cfg_file=cmd.arguments.conf_file,
        environment=cmd.arguments.environment,
        overrides={},
        use_global_config=False,
        use_local_config=False,
    )


def test_setup_environ(cmd):
    cmd._load_config(use_global_config=False)
    # reset ssh_auth_sock and php env vars
    if "SSH_AUTH_SOCK" in os.environ:
        del os.environ["SSH_AUTH_SOCK"]
    if "PHP" in os.environ:
        del os.environ["PHP"]
    cmd.config["php_version"] = None
    # default
    cmd._setup_environ()
    assert "PHP" not in os.environ
    assert "SSH_AUTH_SOCK" not in os.environ
    cmd.config["php_version"] = "pinkunicorn"
    cmd._setup_environ()
    assert "pinkunicorn" == os.environ["PHP"]
    cmd.config["ssh_auth_sock"] = "authsock!"
    cmd._setup_environ()
    assert "SSH_AUTH_SOCK" in os.environ
    assert "authsock!" == os.environ["SSH_AUTH_SOCK"]


@pytest.mark.parametrize("cmd", [["dummy", "--no-shared-authsock"]], indirect=True)
def test_setup_environ_no_auth_sock(cmd):
    cmd._load_config(use_global_config=False)
    cmd.config["ssh_auth_sock"] = "authsock!"
    if "SSH_AUTH_SOCK" in os.environ:
        del os.environ["SSH_AUTH_SOCK"]
    cmd._setup_environ()
    assert "SSH_AUTH_SOCK" not in os.environ


def test_handle_keyboard_interrupt(cmd):
    # Case 1: No announcement if no prior announcement has been made
    cmd.announce = mock.MagicMock(spec=cli.Application.announce)
    cmd.handle_keyboard_interrupt()
    assert cmd.announce.call_count == 0


def test_handle_keyboard_interrupt2(cmd):
    # Case 2: Announcement is made
    cmd.announce("test")
    cmd.announce = mock.MagicMock(spec=cli.Application.announce)
    cmd.handle_keyboard_interrupt()
    assert cmd.announce.call_count == 1


def test_main_not_implemented(app):
    with pytest.raises(NotImplementedError):
        app.main()


def test_handle_exception(app):
    app.get_logger = mock.MagicMock()
    log = app.get_logger.return_value
    valueError = ValueError("test")
    assert app._handle_exception(valueError) == 70
    # Full Backtrace
    assert log.warning.call_count == 1
    assert log.error.call_count == 1

    # Check that the version number is included in the error message
    log.error.assert_called_with(
        "%s failed: <%s> %s (scap version: %s)",
        "cmd.exe",
        "ValueError",
        valueError,
        version.__version__,
    )

    # No backtrace
    log.warning.reset_mock()
    lockFailedError = lock.LockFailedError("test")
    assert app._handle_exception(lockFailedError) == 70
    log.warning.assert_not_called()

    # Check that the version number is included in the error message
    log.error.assert_called_with(
        "%s failed: <%s> %s (scap version: %s)",
        "cmd.exe",
        "LockFailedError",
        lockFailedError,
        version.__version__,
    )


def test_assert_current_user(app, mocker):
    gu = mocker.patch("scap.utils.get_username")
    gu.return_value = "pinkunicorn"
    # This doesn't raise an exception
    app._assert_current_user("pinkunicorn")
    with pytest.raises(RuntimeError):
        app._assert_current_user("baz")


def test_assert_ssh_auth_sock(cmd):
    with pytest.raises(RuntimeError):
        cmd._assert_auth_sock()
    cmd._load_config(use_global_config=False)
    cmd.config["ssh_auth_sock"] = "pinkunicorn"
    cmd._setup_environ()
    cmd._assert_auth_sock()


def test_check_user_auth_sock(cmd):
    cmd._load_config(use_global_config=False)

    assert "gerrit_push_user" in cmd.config
    assert cmd.config["gerrit_push_user"] is None
    with pytest.raises(SystemExit):
        cmd._check_user_auth_sock()

    cmd.config["gerrit_push_user"] = "trainbotuser"
    assert cmd._check_user_auth_sock() is None


def test_factory(cmd):
    assert isinstance(cmd._argparser, arg.ScapArgParser)
    assert cmd.arguments is not None
    assert cmd.extra_arguments is not None
    assert isinstance(cmd, cli.Application)


def test_run(cmd, mocker):
    # Application.run is indecent enough to exit on us.
    # Tell them no.
    ex = mocker.patch("sys.exit")
    uid = mocker.patch("os.geteuid")
    factory = mocker.patch("scap.cli.Application.factory")
    factory.return_value = cmd
    # Ew. Still, only way to test this script.
    cmd._load_config = mock.MagicMock()
    cmd.config = {"block_deployments": False}
    cmd._setup_loggers = mock.MagicMock()
    cmd._setup_environ = mock.MagicMock()
    cmd._handle_exception = mock.MagicMock(return_value=70)
    cmd.handle_keyboard_interrupt = mock.MagicMock(return_value=130)
    # Running as root causes an exception handling
    uid.return_value = 0
    cli.Application.run()
    ex.assert_called_with(70)
    assert factory.call_count == 1
    cmd._load_config.assert_not_called()
    # Reaction to KeyboardInterrupt
    uid.return_value = 1000
    cmd._load_config.side_effect = KeyboardInterrupt()
    cli.Application.run()
    ex.assert_called_with(130)
    cmd._setup_loggers.assert_not_called()
    # Now the happy path
    cmd._load_config.side_effect = None
    cli.Application.run()
    ex.assert_called_with(666)


def test_format_passthrough_args(app):
    assert app.format_passthrough_args() == []

    app.cli_defines = {
        "canary_dashboard_url": "https://somewhere.over.the.rainbow",
        "log_json": False,
    }
    assert sorted(app.format_passthrough_args()) == sorted(
        [
            "-D",
            "canary_dashboard_url:https://somewhere.over.the.rainbow",
            "-D",
            "log_json:False",
        ]
    )
