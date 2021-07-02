from __future__ import absolute_import

import os
import signal
from tempfile import mkstemp
from time import sleep
import threading

try:
    import mock
except ImportError:
    from unittest import mock

import pytest
from scap import lock


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
    with mock.patch.object(locker, 'clear_lock',
                           wraps=locker.clear_lock) as clear_lock:
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


def test_clears_lock_on_sigint():
    assert_clears_lock_on(signal.SIGINT)


def test_clears_lock_on_sigterm():
    assert_clears_lock_on(signal.SIGTERM)
