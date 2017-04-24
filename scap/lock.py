# -*- coding: utf-8 -*-
"""
    scap.lock
    ~~~~~~~~~
    Manages lock/unlock operations for scap

"""

import errno
import fcntl
import os
import stat

from . import utils


GLOBAL_LOCK_FILE = '/var/lock/scap-global-lock'


class LockFailedError(Exception):
    """Signal that a locking attempt failed."""
    pass


class Lock():
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

        # Setup permissions. Start with 0444, everyone can read
        self.lock_perms = stat.S_IRUSR | stat.S_IRGRP | stat.S_IROTH
        # Owner can always write
        self.lock_perms |= stat.S_IWUSR
        # If allowed, let group write too
        if group_write:
            self.lock_perms |= stat.S_IWGRP

    def __enter__(self):
        if os.path.exists(GLOBAL_LOCK_FILE):
            raise LockFailedError(self._get_lock_excuse(GLOBAL_LOCK_FILE))

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
        except OSError as e:
            if e.errno is errno.EEXIST:
                details = self.get_lock_excuse(self.filename)
            else:
                details = 'Failed to acquire lock "%s"; shady reasons "%s"' % (
                           self.filename, e)
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

    def get_lock_excuse(file):
        """
        Get an excuse for why we couldn't lock the file.

        Read the file and its owner, if we can. Fail gracefully with something
        if we can't read it (most likely permissions)

        :param file: Lock file to look for information in
        """

        bad_user = 'a server gremlin'
        excuses = 'no excuse given'
        try:
            bad_user = utils.get_username(os.stat(file).st_uid) or bad_user
            excuses = open(file, 'r').readline().strip() or excuses
        except (IOError, OSError) as e:
            # Before we raise, let's at least warn what failed
            utils.get_logger().warning(e)
        return 'Failed to acquire lock "%s"; owner is "%s"; reason is "%s"' % (
                file, bad_user, excuses)
