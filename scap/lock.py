# -*- coding: utf-8 -*-
"""
    scap.lock
    ~~~~~~~~~
    Manages lock/unlock operations for scap

"""
from __future__ import absolute_import
from __future__ import unicode_literals

import errno
import fcntl
import os

import scap.utils as utils


GLOBAL_LOCK_FILE = '/var/lock/scap-global-lock'


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
    def __init__(self, filename, reason='No reason given', group_write=False):
        self.filename = filename
        self.reason = reason
        self.lock_fd = None

        # If it's a global lock file, let the whole group write. Otherwise,
        # just the original deployer
        if group_write:
            self.lock_perms = 0o664
        else:
            self.lock_perms = 0o644

    def __enter__(self):
        if os.path.exists(GLOBAL_LOCK_FILE):
            raise LockFailedError(get_lock_excuse(GLOBAL_LOCK_FILE))

        # Steal the umask for a bit, can't rely on system
        orig_umask = os.umask(0)
        try:
            self.lock_fd = os.open(
                self.filename,
                os.O_WRONLY | os.O_CREAT | os.O_EXCL,
                self.lock_perms
            )
            fcntl.lockf(self.lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            os.write(self.lock_fd, self.reason)
        except OSError as ose:
            if ose.errno is errno.EEXIST:
                details = get_lock_excuse(self.filename)
            else:
                details = 'Failed to acquire lock "%s"; shady reasons "%s"' % (
                    self.filename, ose)
            raise LockFailedError(details)
        finally:
            # Return the umask
            os.umask(orig_umask)

    def __exit__(self, *args):
        # Return the umask
        if self.lock_fd:
            fcntl.lockf(self.lock_fd, fcntl.LOCK_UN)
            os.close(self.lock_fd)
            if os.path.exists(self.filename):
                try:
                    os.unlink(self.filename)
                except OSError:
                    # Someone else deleted the lock. Freaky, but we already
                    # did our stuff so there's no point in halting execution
                    utils.get_logger().warning(
                        'Huh, lock file disappeared before deletion. ' +
                        'This is probably fine-ish :)'
                    )


def get_lock_excuse(lockfile):
    """
    Get an excuse for why we couldn't lock the file.

    Read the file and its owner, if we can. Fail gracefully with something
    if we can't read it (most likely permissions)

    :param lockfile: Lock file to look for information in
    """

    bad_user = 'a server gremlin'
    excuses = 'no excuse given'
    try:
        bad_user = utils.get_username(os.stat(lockfile).st_uid) or bad_user
        excuses = open(lockfile, 'r').read() or excuses
    except (IOError, OSError) as failure:
        # Before we raise, let's at least warn what failed
        utils.get_logger().warning(failure)
    return 'Failed to acquire lock "%s"; owner is "%s"; reason is "%s"' % (
        lockfile, bad_user, excuses)
