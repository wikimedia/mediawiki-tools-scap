import fcntl
import json
import os
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

try:
    import mock
except ImportError:
    from unittest import mock

import pytest
from scap import lock


def test_lock_create_lock_dir(mocker):
    exists = mocker.patch("os.path.exists")
    exists.return_value = False
    mkdirs = mocker.patch("os.makedirs")

    to_lock = lock.Lock("/a/path/to/lock")
    to_lock._ensure_lock_dir_exists()

    mkdirs.assert_called_with("/a/path/to", 0o775, exist_ok=True)


def test_lock_acquires_lock():
    (lock_file, release_signal_file) = get_temp_filepaths()

    def verify_wait_on_lock():
        verifying_lock = lock.Lock(lock_file, timeout=10)
        with mock.patch("fcntl.lockf", wraps=fcntl.lockf) as lockf:
            with verifying_lock:
                lockf.assert_has_calls(
                    [
                        mock.call(
                            verifying_lock.lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB
                        ),
                        mock.call(verifying_lock.lock_fd, fcntl.LOCK_EX),
                    ]
                )

    lock_proc = acquire_lock_in_subprocess(lock_file, release_signal_file)
    lock_release_thread = start_release_thread(release_signal_file)

    time.sleep(1)
    verify_wait_on_lock()

    lock_release_thread.join()
    lock_proc.wait()
    if lock_proc.returncode != 0:
        pytest.fail("Failed to lock file at start of test")


def test_lock_times_out():
    (lock_file, release_signal_file) = get_temp_filepaths()

    def verify_lock_timeout():
        verifying_lock = lock.Lock(lock_file)
        with mock.patch.object(
            verifying_lock, "_get_deadline_check_interval"
        ) as get_deadline_check_interval:
            with mock.patch.object(verifying_lock, "_get_deadline") as get_deadline:
                get_deadline_check_interval.return_value = 1
                get_deadline.return_value = time.time() + 1
                with pytest.raises(lock.LockFailedError):
                    with verifying_lock:
                        pass

    lock_proc = acquire_lock_in_subprocess(lock_file, release_signal_file)

    time.sleep(1)
    verify_lock_timeout()

    start_release_thread(release_signal_file, 0).join()
    lock_proc.wait()
    if lock_proc.returncode != 0:
        pytest.fail("Failed to lock file at start of test")


def test_lock_records_allow_forced_release(tmp_path):
    lock_file = str(tmp_path / "scap-global-lock")

    with lock.Lock(lock_file, allow_forced_release=True):
        with open(lock_file) as f:
            assert json.load(f)["allow_forced_release"] is True


@pytest.mark.parametrize("allow_forced_release", [True, False])
def test_global_lock_hint_needs_allow_forced_release(
    tmp_path, monkeypatch, caplog, allow_forced_release
):
    global_lock_file = str(tmp_path / "scap-global-lock")
    release_signal_file = str(tmp_path / "release")
    monkeypatch.setattr(lock, "GLOBAL_LOCK_FILE", global_lock_file)
    with open(global_lock_file, "w") as f:
        json.dump(
            {"reason": "lock test", "allow_forced_release": allow_forced_release}, f
        )

    lock_proc = acquire_lock_in_subprocess(global_lock_file, release_signal_file)
    time.sleep(1)
    waiting_lock = lock.Lock(global_lock_file, lock_mode=lock.Lock.SHARED)
    try:
        with mock.patch.object(waiting_lock, "_wait_for_lock"):
            waiting_lock._get_lock()
    finally:
        os.close(waiting_lock.lock_fd)
        Path(release_signal_file).touch()
        lock_proc.wait()

    assert ("scap lock --unlock-all 'lock test'" in caplog.text) == allow_forced_release


def test_signal_gl_release_refuses_lock_without_allow_forced_release(
    tmp_path, monkeypatch
):
    global_lock_file = tmp_path / "scap-global-lock"
    remove_signal_file = tmp_path / "scap-unlock-global"
    monkeypatch.setattr(lock, "GLOBAL_LOCK_FILE", str(global_lock_file))
    monkeypatch.setattr(lock.Lock, "REMOVE_GL_SIGNAL_FILE", str(remove_signal_file))
    global_lock_file.write_text(
        json.dumps({"reason": "Scap is being updated", "allow_forced_release": False})
    )

    assert not lock.Lock.signal_gl_release("testing", io=None)
    assert not remove_signal_file.exists()


def get_temp_filepaths():
    """Return two temporary file paths that do not exist"""
    paths = []
    for _ in range(2):
        fd, path = tempfile.mkstemp()
        os.close(fd)
        os.unlink(path)
        paths.append(path)

    return paths[0], paths[1]


def acquire_lock_in_subprocess(lock_file, release_signal_file):
    return subprocess.Popen(
        [
            sys.executable,
            "-c",
            """
import os
import fcntl
import time

fd = os.open('%s', os.O_WRONLY | os.O_CREAT)
fcntl.lockf(fd, fcntl.LOCK_EX)
while not os.path.exists('%s'):
    time.sleep(1)
    """
            % (lock_file, release_signal_file),
        ]
    )


def start_release_thread(release_signal_file, wait_time=2):
    def release_lock():
        time.sleep(wait_time)
        Path(release_signal_file).touch()

    thread = threading.Thread(target=release_lock)
    thread.daemon = True
    thread.start()
    return thread
