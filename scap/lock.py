# -*- coding: utf-8 -*-
"""
    scap.lock
    ~~~~~~~~~
    Manages lock/unlock operations for scap

"""
import errno
import fcntl
import json
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

    SHARED = "shared"
    EXCLUSIVE = "exclusive"

    def __init__(self, lock_file, name="exclusion", reason="no reason given", timeout=DEFAULT_TIMEOUT_IN_MINS,
                 lock_mode=EXCLUSIVE):
        self.logger = utils.get_logger()

        self.lock_file = lock_file
        self.name = name
        self.timeout = timeout
        if lock_mode not in (TimeoutLock.SHARED, TimeoutLock.EXCLUSIVE):
            raise LockFailedError("Invalid lock_mode '{}'. Must be TimeoutLock.SHARED or TimeoutLock.EXCLUSIVE", lock_mode)
        self.lock_mode = lock_mode
        self.lock_fd = None
        self.global_lock = None
        self.reason = reason

        self._ensure_lock_dir_exists()
        self._ensure_sane_timeout()

    def __enter__(self):
        if self.lock_file == GLOBAL_LOCK_FILE:
            self._get_lock()
        else:
            self.global_lock = TimeoutLock(GLOBAL_LOCK_FILE, name="global lock", timeout=self.timeout, lock_mode=TimeoutLock.SHARED)
            self.global_lock.__enter__()
            self._get_lock()

    def __exit__(self, *args):
        if self.lock_fd:
            if self.lock_mode == TimeoutLock.EXCLUSIVE:
                self._clear_lock_reason()
            fcntl.lockf(self.lock_fd, fcntl.LOCK_UN)
            os.close(self.lock_fd)
        self.lock_fd = None

        if self.global_lock:
            self.global_lock.__exit__()
            self.global_lock = None

    def _get_lock(self):
        """
        Acquires a lock on self.lock_file, waiting up until the timeout has been exceeded.
        If the timeout is reached a LockFailedError exception is raised.

        If an exclusive lock is successfully acquired, the lock reason will be updated.
        """
        try:
            with utils.empty_file_mask():
                access_mode = os.O_RDWR if self.lock_mode == TimeoutLock.EXCLUSIVE else os.O_RDONLY
                self.lock_fd = self._create_or_open_file(self.lock_file, access_mode)
        except OSError as e:
            self.logger.warning("Could not acquire %s lock. Aborting" % self.name)
            raise LockFailedError(e)

        try:
            fcntl.lockf(self.lock_fd, self._get_lock_type() | fcntl.LOCK_NB)
        except BlockingIOError:
            self.logger.warning(
                '%s Will wait up to %s minute(s) for the lock to be released.' %
                (
                    self._get_lock_reason(),
                    self.timeout,
                )
            )
            self._wait_for_lock()
        if self.lock_mode == TimeoutLock.EXCLUSIVE:
            self._set_lock_reason()

    def _wait_for_lock(self):
        """
        Perform a blocking fnctl.lockf() call.  An exception will be raised
        if the timeout period elapses before acquisition.
        """
        deadline_check_interval = self._get_deadline_check_interval()
        deadline = self._get_deadline()

        def deadline_check(*args):
            if time.time() >= deadline:
                print("", flush=True)
                self.logger.warning("Exceeded lock timeout period")
                raise LockFailedError(
                    'Failed to acquire lock after waiting for %s minute(s); %s\nAborting' %
                    (
                        self.timeout,
                        self._get_lock_reason(),
                    )
                )
            print(".", flush=True, end="")
            signal.alarm(deadline_check_interval)

        signal.signal(signal.SIGALRM, deadline_check)
        signal.alarm(deadline_check_interval)
        try:
            fcntl.lockf(self.lock_fd, self._get_lock_type())
        finally:
            signal.alarm(0)

    def _get_deadline_check_interval(self) -> int:
        return 30

    def _get_deadline(self) -> float:
        return time.time() + self.timeout * 60

    def _create_or_open_file(self, lock_file, access_mode):
        try:
            return os.open(lock_file, access_mode | os.O_CREAT | os.O_EXCL, TimeoutLock.LOCK_PERMISSIONS)
        except OSError as e:
            if e.errno == errno.EEXIST:
                # The lockfile has already been created. Fall back to opening the existing file
                # without O_CREAT (to avoid permission denied error if we're not the user
                # that created the lockfile).
                return os.open(lock_file, access_mode, TimeoutLock.LOCK_PERMISSIONS)
            # Something else happened
            raise

    def _get_lock_type(self):
        return fcntl.LOCK_EX if self.lock_mode == TimeoutLock.EXCLUSIVE else fcntl.LOCK_SH

    def _write_lock_file(self, info):
        os.ftruncate(self.lock_fd, 0)
        os.pwrite(self.lock_fd, json.dumps(info, indent=4).encode("UTF-8"), 0)

    def _clear_lock_reason(self):
        self._write_lock_file({})

    def _set_lock_reason(self):
        info = {
            "locker": utils.get_real_username(),
            "pid": os.getpid(),
            "timestamp_utc": time.asctime(time.gmtime()),
            "reason": self.reason,
            }

        self._write_lock_file(info)

    def _get_lock_reason(self) -> str:
        try:
            sb = os.fstat(self.lock_fd)
            info = json.loads(os.pread(self.lock_fd, sb.st_size, 0).decode("UTF-8"))
        except Exception as e:
            utils.get_logger().warning("Caught %s while reading lock info from %s", e, self.lock_file)
            info = {}

        return '%s is locked by %s (pid %s) on %s; reason is "%s".' % \
            (
                self.name,
                info.get("locker", "?"),
                info.get("pid", "?"),
                info.get("timestamp_utc", "?"),
                info.get("reason", "?"),
            )

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
