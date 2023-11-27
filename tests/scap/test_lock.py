import fcntl
import os
import subprocess
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

import timeout_decorator


def test_lock_create_lock_dir(mocker):
    exists = mocker.patch("os.path.exists")
    exists.return_value = False
    mkdirs = mocker.patch("os.makedirs")

    to_lock = lock.Lock("/a/path/to/lock")
    to_lock._ensure_lock_dir_exists()

    mkdirs.assert_called_with("/a/path/to", 0o775, exist_ok=True)


@timeout_decorator.timeout(5, use_signals=False)
def test_lock_acquires_lock():
    (lock_file, release_signal_file) = get_temp_filepaths()

    def verify_wait_on_lock():
        verifying_lock = lock.Lock(lock_file.name)
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

    lock_proc = acquire_lock_in_subprocess(lock_file.name, release_signal_file.name)
    lock_release_thread = start_release_thread(release_signal_file.name)

    time.sleep(1)
    verify_wait_on_lock()

    lock_release_thread.join()
    lock_proc.wait()
    if lock_proc.returncode != 0:
        pytest.fail("Failed to lock file at start of test")


@timeout_decorator.timeout(5, use_signals=False)
def test_lock_times_out():
    (lock_file, release_signal_file) = get_temp_filepaths()

    def verify_lock_timeout():
        verifying_lock = lock.Lock(lock_file.name)
        with mock.patch.object(
            verifying_lock, "_get_deadline_check_interval"
        ) as get_deadline_check_interval:
            with mock.patch.object(verifying_lock, "_get_deadline") as get_deadline:
                get_deadline_check_interval.return_value = 1
                get_deadline.return_value = time.time() + 1
                with pytest.raises(lock.LockFailedError):
                    with verifying_lock:
                        pass

    lock_proc = acquire_lock_in_subprocess(lock_file.name, release_signal_file.name)

    time.sleep(1)
    verify_lock_timeout()

    start_release_thread(release_signal_file.name, 0).join()
    lock_proc.wait()
    if lock_proc.returncode != 0:
        pytest.fail("Failed to lock file at start of test")


def get_temp_filepaths():
    file1 = tempfile.NamedTemporaryFile()
    os.unlink(file1.name)
    file2 = tempfile.NamedTemporaryFile()
    os.unlink(file2.name)
    return file1, file2


def acquire_lock_in_subprocess(lock_file, release_signal_file):
    return subprocess.Popen(
        [
            "/usr/bin/python3",
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
