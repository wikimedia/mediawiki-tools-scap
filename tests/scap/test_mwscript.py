import pytest
import subprocess
from unittest import mock

from scap import cli
from scap import mwscript


@pytest.fixture
def runtime(tmpdir):
    return mwscript.Runtime(
        {
            "datacenter": "foodc",
            "mediawiki_runtime_image": "an.example/mw-php-image",
        },
        "/srv/mediawiki-staging",
        "mw-user",
        temp_dir=tmpdir,
    )


@mock.patch("subprocess.run")
def test_runtime_run_mwscript(mock_run, runtime):
    runtime.run_mwscript(
        "foo.php",
        ["--arg1", "--arg2"],
    )

    # fmt: off
    expected_cmd = [
        "docker", "run",
        "--rm",
        "--attach", "stdout",
        "--attach", "stderr",
        "--user", "mw-user",
        "--mount", "type=bind,source=/srv/mediawiki-staging,target=/srv/mediawiki-staging",
        "--mount", f"type=bind,source={runtime.temp_dir},target={runtime.temp_dir}",
        "--workdir", "/srv/mediawiki-staging",
        "--entrypoint", "/usr/bin/php",
        "--network", "none",
        "--env", "WMF_DATACENTER=foodc",
        "--env", "WMF_MAINTENANCE_OFFLINE=1",
        "an.example/mw-php-image",
        "-d", "display_errors=stderr",
        "-d", "log_errors=off",
        "multiversion/MWScript.php",
        "foo.php",
        "--arg1",
        "--arg2",
    ]

    mock_run.assert_called_with(
        expected_cmd,
        check=False,
    )


@mock.patch("subprocess.run")
def test_runtime_run_mwscript_with_network(mock_run, runtime):
    runtime.run_mwscript(
        "foo.php",
        ["--arg1", "--arg2"],
        network=True,
    )

    # fmt: off
    expected_cmd = [
        "docker", "run",
        "--rm",
        "--attach", "stdout",
        "--attach", "stderr",
        "--user", "mw-user",
        "--mount", "type=bind,source=/srv/mediawiki-staging,target=/srv/mediawiki-staging",
        "--mount", f"type=bind,source={runtime.temp_dir},target={runtime.temp_dir}",
        "--workdir", "/srv/mediawiki-staging",
        "--entrypoint", "/usr/bin/php",
        "--network", "host",
        "--env", "WMF_DATACENTER=foodc",
        "--env", "WMF_MAINTENANCE_OFFLINE=1",
        "an.example/mw-php-image",
        "-d", "display_errors=stderr",
        "-d", "log_errors=off",
        "multiversion/MWScript.php",
        "foo.php",
        "--arg1",
        "--arg2",
    ]

    mock_run.assert_called_with(
        expected_cmd,
        check=False,
    )


@mock.patch("subprocess.run")
def test_runtime_run_shell(mock_run, runtime):
    runtime.run_shell(
        "echo foo",
    )

    # fmt: off
    expected_cmd = [
        "docker", "run",
        "--rm",
        "--attach", "stdout",
        "--attach", "stderr",
        "--user", "mw-user",
        "--mount", "type=bind,source=/srv/mediawiki-staging,target=/srv/mediawiki-staging",
        "--mount", f"type=bind,source={runtime.temp_dir},target={runtime.temp_dir}",
        "--workdir", "/srv/mediawiki-staging",
        "--entrypoint", "/bin/bash",
        "--network", "none",
        "an.example/mw-php-image",
        "-c",
        "echo foo",
    ]

    mock_run.assert_called_with(
        expected_cmd,
        check=False,
    )


@mock.patch("scap.cli.Application.scap_call")
def test_run(mock_scap_call):
    mock_scap_call.return_value = mock.Mock(
        **{
            "stdout": "some stdout\n",
            "stderr": "some stderr\n",
            "returncode": 0,
        }
    )

    mock_app = mock.Mock(
        **{
            "config": {
                "docker_user": "fooser",
                "stage_dir": "/stage/dir",
                "mediawiki_runtime_user": "mw-user",
            },
            "scap_call": mock_scap_call,
        }
    )

    mwscript.run(
        mock_app,
        "foo.php",
        "--arg",
        version="1.2.3",
        network=True,
    )

    mock_scap_call.assert_called_with(
        [
            "mwscript",
            "--no-local-config",
            "--directory",
            "/stage/dir",
            "--user",
            "mw-user",
            "--network",
            "--",
            "foo.php",
            "--wiki=aawiki",
            "--force-version",
            "1.2.3",
            "--arg",
        ],
        user="fooser",
        passthrough_arguments=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )


@mock.patch("scap.cli.Application.scap_call")
def test_run_check_warnings(mock_scap_call):
    mock_scap_call.return_value = mock.Mock(
        **{
            "stdout": "some stdout\n",
            "stderr": (
                "some stderr\n"
                "PHP Notice: foo notice 1\n"
                "Notice: foo notice 2\n"
                "PHP Warning: foo warning 1\n"
                "Warning: foo warning 2\n"
                "some more stderr\n"
            ),
            "returncode": 0,
        }
    )

    mock_app = mock.Mock(
        **{
            "config": {
                "docker_user": "fooser",
                "stage_dir": "/stage/dir",
                "mediawiki_runtime_user": "mw-user",
            },
            "scap_call": mock_scap_call,
        }
    )

    with pytest.raises(SystemExit) as exc_info:
        mwscript.run(
            mock_app,
            "foo.php",
            "--arg1",
            "--arg2",
            check_warnings=True,
        )

    exc_info.match(r"foo.php generated PHP notices/warnings")
    exc_info.match(r"foo notice 1")
    exc_info.match(r"foo notice 2")
    exc_info.match(r"foo warning 1")
    exc_info.match(r"foo warning 2")

    mock_scap_call.assert_called_with(
        [
            "mwscript",
            "--no-local-config",
            "--directory",
            "/stage/dir",
            "--user",
            "mw-user",
            "--",
            "foo.php",
            "--wiki=aawiki",
            "--arg1",
            "--arg2",
        ],
        user="fooser",
        passthrough_arguments=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )


@mock.patch("scap.cli.Application.scap_call")
def test_run_shell(mock_scap_call):
    mock_app = mock.Mock(
        **{
            "config": {
                "docker_user": "fooser",
                "stage_dir": "/stage/dir",
                "mediawiki_runtime_user": "mw-user",
            },
            "scap_call": mock_scap_call,
        }
    )

    mwscript.run_shell(
        mock_app,
        "echo foo {}",
        "bar baz",
    )

    mock_scap_call.assert_called_with(
        [
            "mwshell",
            "--no-local-config",
            "--directory",
            "/stage/dir",
            "--user",
            "mw-user",
            "--",
            "echo foo 'bar baz'",
        ],
        user="fooser",
        passthrough_arguments=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )


@mock.patch("scap.mwscript.Runtime", autospec=True)
def test_scap_mwscript(MockRuntime):
    app = cli.Application.factory(
        [
            "mwscript",
            "--directory",
            "/stage/dir",
            "--user",
            "mw-user",
            "--network",
            "--",
            "foo.php",
            "--wiki=foowiki",
            "--force-version",
            "1.2.3",
            "--arg1",
        ],
    )
    app.setup()

    mock_runtime = MockRuntime.return_value = mock.Mock()

    with pytest.raises(SystemExit) as exc_info:
        app.main()

    mock_runtime.run_mwscript.assert_called_with(
        "foo.php",
        ["--wiki=foowiki", "--force-version", "1.2.3", "--arg1"],
        network=True,
    )

    assert exc_info.value.code is mock_runtime.run_mwscript.return_value.returncode


@mock.patch("scap.mwscript.Runtime", autospec=True)
def test_scap_mwshell(MockRuntime):
    app = cli.Application.factory(
        [
            "mwshell",
            "--directory",
            "/stage/dir",
            "--user",
            "mw-user",
            "--",
            "foo command",
        ],
    )
    app.setup()

    mock_runtime = MockRuntime.return_value = mock.Mock()

    with pytest.raises(SystemExit) as exc_info:
        app.main()

    mock_runtime.run_shell.assert_called_with("foo command")
    assert exc_info.value.code is mock_runtime.run_shell.return_value.returncode
