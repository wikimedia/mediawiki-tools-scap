from __future__ import absolute_import

import os
import signal
import subprocess
import tempfile
import threading
import time
from pathlib import Path
from tempfile import mkstemp
from time import sleep

try:
    import mock
except ImportError:
    from unittest import mock

import pytest
from scap import lock

import timeout_decorator


def assert_clears_lock_on(sig):
    # Forge a temporary filename
    (fd, lock_filename) = mkstemp()
    os.close(fd)
    os.unlink(lock_filename)

    def send_signal():
        os.kill(os.getpid(), sig)

    signaler = threading.Thread(target=send_signal)
    signaler.daemon = True

    locker = lock.Lock(lock_filename)
    with mock.patch.object(locker, '_clear_lock', wraps=locker._clear_lock) as clear_lock:
        with locker:
            with pytest.raises(SystemExit) as exit_info:
                signaler.start()
                sleep(1000)
                pytest.fail("Timed out waiting for signal to be handled")

            # On Python 3.6 or later replace with assert_called_once()
            clear_lock.assert_called_once_with()

            assert not os.path.exists(lock_filename)

            assert exit_info.type is SystemExit
            assert exit_info.value.args[0] == 128 + sig


def test_clears_lock_on_sigterm():
    assert_clears_lock_on(signal.SIGTERM)


@timeout_decorator.timeout(5, use_signals=False)
def test_timeout_lock_acquires_lock():
    (lock_file, release_signal_file) = get_temp_filepaths()

    def verify_wait_on_lock():
        verifying_lock = lock.TimeoutLock(lock_file.name)
        with mock.patch.object(
                verifying_lock, '_wait_for_lock', wraps=verifying_lock._wait_for_lock
        ) as wait_for_lock:
            with verifying_lock:
                # On Python 3.6 or later replace with assert_called_once()
                wait_for_lock.assert_called_once_with()

    lock_proc = acquire_lock(lock_file.name, release_signal_file.name)
    lock_release_thread = start_release_thread(release_signal_file.name)

    time.sleep(1)
    verify_wait_on_lock()

    lock_release_thread.join()
    lock_proc.wait()
    if lock_proc.returncode != 0:
        pytest.fail("Failed to lock file at start of test")


@timeout_decorator.timeout(5, use_signals=False)
def test_timeout_lock_times_out():
    (lock_file, release_signal_file) = get_temp_filepaths()

    def verify_lock_timeout():
        verifying_lock = lock.TimeoutLock(lock_file.name)
        with mock.patch.object(
                verifying_lock, '_get_deadline_check_interval'
        ) as get_deadline_check_interval:
            with mock.patch.object(
                    verifying_lock, '_get_deadline'
            ) as get_deadline:
                get_deadline_check_interval.return_value = 1
                get_deadline.return_value = time.time() + 1
                with pytest.raises(lock.LockFailedError):
                    with verifying_lock:
                        pass

    lock_proc = acquire_lock(lock_file.name, release_signal_file.name)

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


def acquire_lock(lock_file, release_signal_file):
    return subprocess.Popen(["/usr/bin/python3", "-c", """
import os
import fcntl
import time

fd = os.open('%s', os.O_WRONLY | os.O_CREAT)
fcntl.lockf(fd, fcntl.LOCK_EX)
while not os.path.exists('%s'):
    time.sleep(1)
    """ % (lock_file, release_signal_file)])


def start_release_thread(release_signal_file, wait_time=2):
    def release_lock():
        time.sleep(wait_time)
        Path(release_signal_file).touch()

    thread = threading.Thread(target=release_lock)
    thread.daemon = True
    thread.start()
    return thread
