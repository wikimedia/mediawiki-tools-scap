import io
import pytest
import subprocess
from unittest import mock

from scap import mwscript


@pytest.fixture
def runtime(tmpdir):
    return mwscript.Runtime(
        {
            "datacenter": "foodc",
            "stage_dir": "/srv/mediawiki-staging",
            "mediawiki_runtime_image": "an.example/mw-php-image",
            "mediawiki_runtime_user": "mw-user",
        },
        temp_dir=tmpdir,
    )


@mock.patch("subprocess.Popen")
def test_run_mwscript(mock_popen, runtime):
    mock_popen.return_value = mock.Mock(
        **{
            "stdout": io.StringIO("some stdout\n"),
            "communicate.return_value": ("", "some stderr\n"),
            "returncode": 0,
        }
    )

    proc = runtime.run_mwscript(
        "foo.php",
        ["--arg1", "--arg2"],
        stdout=subprocess.PIPE,
    )

    # fmt: off
    args = [
        "docker", "run",
        "--rm",
        "--attach", "stdin",
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
        "--wiki=aawiki",
        "--arg1",
        "--arg2",
    ]

    mock_popen.assert_called_with(
        args,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    assert proc.stdout == "some stdout\n"
    assert proc.stderr == "some stderr\n"


@mock.patch("subprocess.Popen")
def test_run_mwscript_with_options(mock_popen, runtime):
    mock_popen.return_value = mock.Mock(
        **{
            "stdout": io.StringIO("some stdout\n"),
            "communicate.return_value": ("", "some stderr\n"),
            "returncode": 0,
        }
    )

    proc = runtime.run_mwscript(
        "foo.php",
        ["--arg1", "--arg2"],
        wiki="foowiki",
        version="1.2.3",
        network=True,
        stdout=subprocess.PIPE,
    )

    # fmt: off
    args = [
        "docker", "run",
        "--rm",
        "--attach", "stdin",
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
        "--wiki=foowiki",
        "--force-version",
        "1.2.3",
        "--arg1",
        "--arg2",
    ]

    mock_popen.assert_called_with(
        args,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    assert proc.stdout == "some stdout\n"
    assert proc.stderr == "some stderr\n"


@mock.patch("subprocess.Popen")
def test_run_mwscript_check_warnings(mock_popen, runtime):
    stderr = (
        "some stderr\n"
        "PHP Notice: foo notice 1\n"
        "Notice: foo notice 2\n"
        "PHP Warning: foo warning 1\n"
        "Warning: foo warning 2\n"
        "some more stderr\n"
    )
    mock_popen.return_value = mock.Mock(
        **{
            "stdout": io.StringIO("some stdout\n"),
            "communicate.return_value": ("", stderr),
            "returncode": 0,
        }
    )

    with pytest.raises(SystemExit) as exc_info:
        runtime.run_mwscript(
            "foo.php",
            ["--arg1", "--arg2"],
            check_warnings=True,
        )

    exc_info.match(r"foo.php generated PHP notices/warnings")
    exc_info.match(r"foo notice 1")
    exc_info.match(r"foo notice 2")
    exc_info.match(r"foo warning 1")
    exc_info.match(r"foo warning 2")


@mock.patch("subprocess.Popen")
def test_run_shell(mock_popen, runtime):
    mock_popen.return_value = mock.Mock(
        **{
            "stdout": io.StringIO("some stdout\n"),
            "communicate.return_value": ("", "some stderr\n"),
            "returncode": 0,
        }
    )

    proc = runtime.run_shell(
        "some shell command",
        stdout=subprocess.PIPE,
    )

    # fmt: off
    args = [
        "docker", "run",
        "--rm",
        "--attach", "stdin",
        "--attach", "stdout",
        "--attach", "stderr",
        "--user", "mw-user",
        "--mount", "type=bind,source=/srv/mediawiki-staging,target=/srv/mediawiki-staging",
        "--mount", f"type=bind,source={runtime.temp_dir},target={runtime.temp_dir}",
        "--workdir", "/srv/mediawiki-staging",
        "--entrypoint", "/bin/bash",
        "--network", "none",
        "an.example/mw-php-image",
        "-c", "some shell command",
    ]

    mock_popen.assert_called_with(
        args,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    assert proc.stdout == "some stdout\n"
    assert proc.stderr == "some stderr\n"
