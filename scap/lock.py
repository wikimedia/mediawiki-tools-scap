# -*- coding: utf-8 -*-
"""
    scap.lock
    ~~~~~~~~~
    Manages lock/unlock operations for scap

"""
from __future__ import absolute_import

import errno
import fcntl
import os
import signal
import time

import scap.utils as utils

GLOBAL_LOCK_FILE = "/var/lock/scap-global-lock"


class LockFailedError(RuntimeError):
    """Signal that a locking attempt failed."""

    pass


class TimeoutLock:
    """
    File-based exclusive lock. It will block trying to acquire a lock on the file for the specified
    amount of time before timing out.
    """

    DEFAULT_TIMEOUT_IN_MINS = 10
    MIN_TIMEOUT_IN_MINS = 1
    MAX_TIMEOUT_IN_MINS = 60
    LOCK_PERMISSIONS = 0o666

    def __init__(self, lock_file, name="exclusion", reason="no reason given", timeout=DEFAULT_TIMEOUT_IN_MINS):
        self.logger = utils.get_logger()

        self.lock_file = lock_file
        self.name = name
        self.timeout = timeout
        self.lock_fd = None
        self.global_lock_fd = None
        self.reason = reason

        self._ensure_lock_dir_exists()
        self._ensure_sane_timeout()

    def __enter__(self):
        if self.lock_file == GLOBAL_LOCK_FILE:
            self.lock_fd = self._get_lock(self.lock_file, os.O_RDWR)
        else:
            # lock global lock file for reading so a global lock cannot be obtained when any lock is in place
            self.global_lock_fd = self._get_lock(GLOBAL_LOCK_FILE, os.O_RDONLY, fcntl.LOCK_SH)
            self.lock_fd = self._get_lock(self.lock_file, os.O_WRONLY)

        self._write_lock_reason(self.lock_fd)

    def __exit__(self, *args):
        self._release_lock(self.lock_fd)
        self.lock_fd = None

        self._release_lock(self.global_lock_fd)
        self.global_lock_fd = None

    def _release_lock(self, lock_fd):
        if lock_fd:
            fcntl.lockf(lock_fd, fcntl.LOCK_UN)
            os.close(lock_fd)

    def _write_lock_reason(self, lock_fd):
        os.ftruncate(lock_fd, 0)
        os.write(lock_fd, self.reason.encode("UTF-8"))

    def _create_or_open_file(self, lock_file, flag):
        try:
            return os.open(lock_file, flag | os.O_CREAT | os.O_EXCL, TimeoutLock.LOCK_PERMISSIONS)
        except OSError as e:
            if e.errno == errno.EEXIST:
                # The lockfile has already been created. Fall back to opening the existing file
                # without O_CREAT (to avoid permission denied error if we're not the user
                # that created the lockfile).
                return os.open(lock_file, flag, TimeoutLock.LOCK_PERMISSIONS)
            # Something else happened
            raise

    def _get_lock(self, lock_file, flag, lock_type=fcntl.LOCK_EX):
        """
        Acquires a lock on 'lock_file', waiting up until the timeout has been exceeded.
        If sucesssful, returns the file descriptor of the lockfile.  If the timeout is reached,
        a LockFailedError exception is raised.
        """
        try:
            with utils.empty_file_mask():
                lock_fd = self._create_or_open_file(lock_file, flag)
        except OSError as e:
            self.logger.warning("Could not acquire %s lock. Aborting" % self.name)
            raise LockFailedError(e)

        try:
            fcntl.lockf(lock_fd, lock_type | fcntl.LOCK_NB)
        except BlockingIOError:
            self.logger.warning("%s", self._get_lock_excuse(lock_file, True))
            self._wait_for_lock(lock_fd, lock_file, lock_type)

        return lock_fd

    def _get_lock_excuse(self, lock_file, is_waiting):
        """
        Get an excuse for why we couldn't lock the file.

        Read the file and its owner, if we can. Fail gracefully with something
        if we can't read it (most likely permissions)
        """

        bad_user = "a server gremlin"
        excuses = "no excuse given"
        try:
            bad_user = utils.get_username(os.stat(lock_file).st_uid) or bad_user
            excuses = open(lock_file, "r").read() or excuses
        except (IOError, OSError) as failure:
            # Before we raise, let's at least warn what failed
            utils.get_logger().warning(failure)

        if is_waiting:
            excuse = 'Lock "%s" is busy; owner is "%s"; reason is "%s".' \
                     ' Will wait up to %s minute(s) for the lock to be released.' %  \
                     (
                        self.name,
                        bad_user,
                        excuses,
                        self.timeout,
                     )
        else:
            excuse = 'Failed to acquire lock "%s" after waiting for %s minute(s); owner is "%s"; reason is "%s".' % \
                     (
                        self.name,
                        self.timeout,
                        bad_user,
                        excuses,
                     )

        return excuse

    def _ensure_lock_dir_exists(self):
        lock_dir = os.path.dirname(self.lock_file)

        if not os.path.exists(lock_dir):
            try:
                # exist_ok=True prevents error in case of concurrent execution
                os.makedirs(lock_dir, 0o775, exist_ok=True)
            except Exception as e:
                raise Exception("Failed to create required lock dir: \"%s\"" % lock_dir) from e

    def _ensure_sane_timeout(self):
        if not TimeoutLock.MIN_TIMEOUT_IN_MINS <= self.timeout <= TimeoutLock.MAX_TIMEOUT_IN_MINS:
            self.timeout = TimeoutLock.DEFAULT_TIMEOUT_IN_MINS
            self.logger.warning(
                "Supplied timeout for %s lock needs to be in range [1, 60]. Timeout reset to %s"
                " minutes" % (self.name, TimeoutLock.DEFAULT_TIMEOUT_IN_MINS)
            )

    def _wait_for_lock(self, lock_fd, lock_file, lock_type):
        deadline_check_interval = self._get_deadline_check_interval()
        deadline = self._get_deadline()

        def deadline_check(*args):
            if time.time() >= deadline:
                print("", flush=True)
                self.logger.warning("Exceeded lock timeout period")
                excuse = self._get_lock_excuse(lock_file, False)
                raise LockFailedError(excuse + "\nAborting")
            print(".", flush=True, end="")
            signal.alarm(deadline_check_interval)

        signal.signal(signal.SIGALRM, deadline_check)
        signal.alarm(deadline_check_interval)
        try:
            fcntl.lockf(lock_fd, lock_type)
        finally:
            signal.alarm(0)
        if lock_type == fcntl.LOCK_EX:
            self._write_lock_reason(lock_fd)

    def _get_deadline_check_interval(self) -> int:
        return 30

    def _get_deadline(self) -> float:
        return time.time() + self.timeout * 60
