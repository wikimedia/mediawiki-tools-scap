# -*- coding: utf-8 -*-
"""
    scap.lock
    ~~~~~~~~~
    Manages lock/unlock operations for scap

"""
from __future__ import absolute_import

import errno
import os
import signal
import sys

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
            signal.signal(signal.SIGINT, self._sig_handler)
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
        if os.path.exists(self.filename):
            try:
                os.unlink(self.filename)
            except OSError:
                # Someone else deleted the lock. Freaky, but we already
                # did our stuff so there's no point in halting execution
                utils.get_logger().warning(
                    "Huh, lock file disappeared before deletion. "
                    + "This is probably fine-ish :)"
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
