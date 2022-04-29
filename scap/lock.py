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
import sys
import time

import scap.utils as utils


GLOBAL_LOCK_FILE = "/var/lock/scap-global-lock"


class LockFailedError(RuntimeError):
    """Signal that a locking attempt failed."""

    pass


class Lock(object):  # pylint: disable=too-few-public-methods
    """
    Context manager. Acquires a file lock on entry, releases on exit.

    :param filename: File to lock
    :param reason: Reason we're locking a file
    :raises: LockFailedError on failure
    """

    def __init__(self, filename, reason="No reason given", group_write=False):
        self.filename = filename
        if isinstance(reason, str):
            reason = reason.encode("UTF-8")
        assert isinstance(reason, bytes)
        self.reason = reason
        self.locker_pid = os.getpid()

        # If it's a global lock file, let the whole group write. Otherwise,
        # just the original deployer
        if group_write:
            self.lock_perms = 0o664
        else:
            self.lock_perms = 0o644

    def __enter__(self):
        if os.path.exists(GLOBAL_LOCK_FILE):
            raise LockFailedError(get_lock_excuse(GLOBAL_LOCK_FILE))

        try:
            self._acquire_lock()
            signal.signal(signal.SIGTERM, self._sig_handler)
        except OSError as ose:
            if ose.errno is errno.EEXIST:
                details = get_lock_excuse(self.filename)
            else:
                details = 'Failed to acquire lock "%s"; shady reasons "%s"' % (
                    self.filename,
                    ose,
                )
            raise LockFailedError(details)

    def __exit__(self, *args):
        self._clear_lock()

    def _acquire_lock(self):
        lock_fd = None
        try:
            with utils.empty_file_mask():
                lock_fd = os.open(
                    self.filename, os.O_WRONLY | os.O_CREAT | os.O_EXCL, self.lock_perms
                )
            os.write(lock_fd, self.reason)
        finally:
            if lock_fd:
                os.close(lock_fd)

    def _sig_handler(self, signum, *args):
        self._clear_lock()
        sys.exit(128 + signum)

    def _clear_lock(self):
        # T307242: Only attempt to unlock if the current process is the
        # one that created the lockfile.
        if os.getpid() == self.locker_pid:
            try:
                os.unlink(self.filename)
            except FileNotFoundError:
                # Someone else deleted the lock.
                utils.get_logger().warning(
                    "Huh, lock file disappeared before deletion. "
                    + "This should not happen!!!"
                )


def get_lock_excuse(lockfile):
    """
    Get an excuse for why we couldn't lock the file.

    Read the file and its owner, if we can. Fail gracefully with something
    if we can't read it (most likely permissions)

    :param lockfile: Lock file to look for information in
    """

    bad_user = "a server gremlin"
    excuses = "no excuse given"
    try:
        bad_user = utils.get_username(os.stat(lockfile).st_uid) or bad_user
        excuses = open(lockfile, "r").read() or excuses
    except (IOError, OSError) as failure:
        # Before we raise, let's at least warn what failed
        utils.get_logger().warning(failure)
    return 'Failed to acquire lock "%s"; owner is "%s"; reason is "%s"' % (
        lockfile,
        bad_user,
        excuses,
    )


class TimeoutLock:
    """
    File-based exclusive lock. It will block trying to acquire a lock on the file for the specified
    amount of time before timing out
    """

    DEFAULT_TIMEOUT_IN_MINS = 10
    MIN_TIMEOUT_IN_MINS = 1
    MAX_TIMEOUT_IN_MINS = 60

    def __init__(self, lock_file, name="exclusion", timeout=DEFAULT_TIMEOUT_IN_MINS):
        self.logger = utils.get_logger()

        self.lock_file = lock_file
        self.name = name
        self.timeout = timeout
        self.lock_fd = None

        self._ensure_sane_timeout()

    def __enter__(self):
        self.lock_fd = os.open(self.lock_file, os.O_WRONLY | os.O_CREAT)
        try:
            fcntl.lockf(self.lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as e:
            if e.errno in {errno.EAGAIN, errno.EACCES}:
                self.logger.warning(
                    "Could not acquire %s lock. Will wait up to %s minute(s) for the"
                    " lock to be released" % (self.name, self.timeout)
                )
                self._wait_for_lock()

    def __exit__(self, *args):
        if self.lock_fd:
            fcntl.lockf(self.lock_fd, fcntl.LOCK_UN)
            os.close(self.lock_fd)
            self.lock_fd = None

    def _ensure_sane_timeout(self):
        if not TimeoutLock.MIN_TIMEOUT_IN_MINS <= self.timeout <= TimeoutLock.MAX_TIMEOUT_IN_MINS:
            self.timeout = TimeoutLock.DEFAULT_TIMEOUT_IN_MINS
            self.logger.warning(
                "Supplied timeout for %s lock needs to be in range [1, 60]. Timeout reset to %s"
                " minutes" % (self.name, TimeoutLock.DEFAULT_TIMEOUT_IN_MINS)
            )

    def _wait_for_lock(self):
        deadline_check_interval = self._get_deadline_check_interval()
        deadline = self._get_deadline()

        def deadline_check(*args):
            if time.time() >= deadline:
                print("", flush=True)
                raise LockFailedError(
                    "Could not acquire %s lock after waiting for %s minute(s)."
                    " Aborting" % (self.name, self.timeout)
                )
            print(".", flush=True, end="")
            signal.alarm(deadline_check_interval)

        signal.signal(signal.SIGALRM, deadline_check)
        signal.alarm(deadline_check_interval)
        try:
            fcntl.lockf(self.lock_fd, fcntl.LOCK_EX)
        finally:
            signal.alarm(0)

    def _get_deadline_check_interval(self) -> int:
        return 30

    def _get_deadline(self) -> float:
        return time.time() + self.timeout * 60
