# -*- coding: utf-8 -*-
"""
    scap.lock
    ~~~~~~~~~
    Manages lock/unlock operations for scap

"""
import atexit
import errno
import fcntl
import json
import os
import signal
import threading
import time

import scap.utils as utils


GLOBAL_LOCK_FILE = "/var/lock/scap-global-lock"


class LockFailedError(RuntimeError):
    """Signal that a locking attempt failed."""

    pass


logger = utils.get_logger()


class Lock:
    """
    File-based lock with the following features:
     * Exclusive or shared
     * Will block and wait for lock to be released instead of failing immediately. Timeout configurable
    """

    DEFAULT_TIMEOUT = 600
    LOCK_PERMISSIONS = 0o666

    REMOVE_GL_SIGNAL_FILE = "/tmp/scap-unlock-global"
    # The Scap processes involved in releasing a global lock could use file REMOVE_GL_SIGNAL_FILE to communicate. But it's
    # possible the system where Scap is running has set the following kernel option to 1 or 2:
    # https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/tree/Documentation/admin-guide/sysctl/fs.rst?h=v5.15#n239
    # So we use two files for bi-directional communication to make sure we don't run into a problem
    ACK_GL_SIGNAL_FILE = "/tmp/scap-unlock-global-removed"

    SHARED = "shared"
    EXCLUSIVE = "exclusive"

    def __init__(
        self,
        lock_file,
        name="exclusion",
        reason="no reason given",
        timeout=DEFAULT_TIMEOUT,  # Timeout in seconds
        lock_mode=EXCLUSIVE,
        do_global_lock=True,
    ):
        self.lock_file = lock_file
        self.name = name
        self.timeout = timeout
        if lock_mode not in (Lock.SHARED, Lock.EXCLUSIVE):
            raise LockFailedError(
                "Invalid lock_mode '{}'. Must be Lock.SHARED or Lock.EXCLUSIVE",
                lock_mode,
            )
        self.lock_mode = lock_mode
        self.lock_fd = None
        self.do_global_lock = do_global_lock
        self.global_lock = None
        self.reason = reason

        self._ensure_lock_dir_exists()
        self._ensure_sane_timeout()

    def __enter__(self):
        if self.lock_file == GLOBAL_LOCK_FILE:
            self._get_lock()
        else:
            if self.do_global_lock:
                self.global_lock = Lock(
                    GLOBAL_LOCK_FILE,
                    name="global lock",
                    timeout=self.timeout,
                    lock_mode=Lock.SHARED,
                )
                self.global_lock.__enter__()
            self._get_lock()
        return self

    def __exit__(self, *args):
        if self.lock_fd:
            if self.lock_mode == Lock.EXCLUSIVE:
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
                access_mode = (
                    os.O_RDWR if self.lock_mode == Lock.EXCLUSIVE else os.O_RDONLY
                )
                self.lock_fd = self._create_or_open_file(self.lock_file, access_mode)
        except OSError as e:
            logger.warning("Could not acquire %s lock. Aborting" % self.name)
            raise LockFailedError(e)

        try:
            fcntl.lockf(self.lock_fd, self._get_lock_type() | fcntl.LOCK_NB)
        except BlockingIOError:
            feedback = self._get_lock_message()

            if (
                self.lock_file == GLOBAL_LOCK_FILE
                and self._exclusive_lock_is_in_place()
            ):
                feedback += (
                    "\nThe lock is global. If required, it can be forcefully removed by running"
                    """ "scap lock --unlock-all <reason>"."""
                )

            if self.timeout == 0:
                raise LockFailedError(f"{self._get_lock_message()}\nAborting")

            logger.warning(
                "%s\nWill wait up to %s minute(s) for the lock(s) to be released."
                % (
                    feedback,
                    self.timeout // 60,
                )
            )
            self._wait_for_lock()
        if self.lock_mode == Lock.EXCLUSIVE:
            self._set_lock_reason()

    def _wait_for_lock(self):
        """
        Perform a blocking fcntl.lockf() call.  An exception will be raised
        if the timeout period elapses before acquisition.
        """
        deadline_check_interval = self._get_deadline_check_interval()
        deadline = self._get_deadline()

        def deadline_check(*args):
            if time.time() >= deadline:
                print("", flush=True)
                logger.warning("Exceeded lock timeout period")
                raise LockFailedError(
                    "Failed to acquire lock after waiting for %s minute(s); %s\nAborting"
                    % (
                        self.timeout // 60,
                        self._get_lock_message(),
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
        return min(self.timeout, 30)

    def _get_deadline(self) -> float:
        return time.time() + self.timeout

    def _create_or_open_file(self, lock_file, access_mode):
        try:
            return os.open(
                lock_file, access_mode | os.O_CREAT | os.O_EXCL, Lock.LOCK_PERMISSIONS
            )
        except OSError as e:
            if e.errno == errno.EEXIST:
                # The lockfile has already been created. Fall back to opening the existing file
                # without O_CREAT (to avoid permission denied error if we're not the user
                # that created the lockfile).
                return os.open(lock_file, access_mode, Lock.LOCK_PERMISSIONS)
            # Something else happened
            raise

    def _get_lock_type(self):
        return fcntl.LOCK_EX if self.lock_mode == Lock.EXCLUSIVE else fcntl.LOCK_SH

    def _write_lock_file(self, info):
        Lock._write_json_file(self.lock_fd, info)

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

    def _get_lock_message(self) -> str:
        def read_failed(lock, ex):
            logger.warning("Caught %s while reading lock info from %s", ex, lock)

        def get_locks():
            lock_dir, global_lock_basename = os.path.split(GLOBAL_LOCK_FILE)
            return set(
                [
                    os.path.join(lock_dir, lock)
                    for lock in next(os.walk(lock_dir))[2]
                    if "scap" in lock and global_lock_basename != lock
                ]
            )

        def read_lock_info(lock, fd):
            try:
                sb = os.fstat(fd)
                info = json.loads(os.pread(fd, sb.st_size, 0).decode("UTF-8"))
            except Exception as ex:
                read_failed(lock, ex)
                info = {}
            return info

        locks_info = []
        # Shared lock is blocking global
        if (
            self.lock_file == GLOBAL_LOCK_FILE
            and not self._exclusive_lock_is_in_place()
        ):
            # Gather info from all the repository locks
            for lock_file in get_locks():
                lock_fd = None
                try:
                    lock_fd = os.open(lock_file, os.O_RDONLY)
                    locks_info.append(read_lock_info(lock_file, lock_fd))
                except OSError as e:
                    read_failed(lock_file, e)
                finally:
                    if lock_fd:
                        os.close(lock_fd)
        else:
            locks_info = [read_lock_info(self.lock_file, self.lock_fd)]

        # Filter out locks that aren't actually set
        locks_info = [info for info in locks_info if info != {}]
        if not locks_info:
            return "Scap initially detected a lock but the file disappeared. No lock details to show"

        return "\n".join(
            [
                '%s is locked by %s (pid %s) on %s; reason is "%s".'
                % (
                    self.name,
                    info.get("locker", "?"),
                    info.get("pid", "?"),
                    info.get("timestamp_utc", "?"),
                    info.get("reason", "?"),
                )
                for info in locks_info
            ]
        )

    def _exclusive_lock_is_in_place(self):
        try:
            fcntl.lockf(self.lock_fd, fcntl.LOCK_SH | fcntl.LOCK_NB)
            fcntl.lockf(self.lock_fd, fcntl.LOCK_UN)
            return False
        except BlockingIOError:
            return True

    def _ensure_lock_dir_exists(self):
        lock_dir = os.path.dirname(self.lock_file)

        if not os.path.exists(lock_dir):
            try:
                # exist_ok=True prevents error in case of concurrent execution
                os.makedirs(lock_dir, 0o775, exist_ok=True)
            except Exception as e:
                raise Exception(
                    'Failed to create required lock dir: "%s"' % lock_dir
                ) from e

    def _ensure_sane_timeout(self):
        if not 0 <= self.timeout <= 3600:
            self.timeout = Lock.DEFAULT_TIMEOUT
            logger.warning(
                "Supplied timeout for %s lock needs to be in range [0, 3600]. Timeout reset to %s"
                " minutes" % (self.name, Lock.DEFAULT_TIMEOUT // 60)
            )

    @staticmethod
    def signal_gl_release(release_reason, io, confirm=True):
        if os.path.exists(GLOBAL_LOCK_FILE):
            with open(GLOBAL_LOCK_FILE, encoding="UTF-8") as f:
                lock_info = json.loads(f.read())

            if lock_info != {}:
                locker = (lock_info.get("locker", "?"),)
                timestamp_utc = (lock_info.get("timestamp_utc", "?"),)
                reason = (lock_info.get("reason", "?"),)
                details = (
                    "Lock details:\n"
                    "  locker: %s\n" % locker
                    + "  time acquired (UTC): %s\n" % timestamp_utc
                    + "  reason: %s" % reason
                )
                if confirm:
                    prompt = details + "\nClear lock?"
                    if not io.prompt_user_for_confirmation(prompt):
                        utils.abort("Canceled by user")
                else:
                    logger.info(details)

                with utils.empty_file_mask():
                    fd = os.open(
                        Lock.REMOVE_GL_SIGNAL_FILE, os.O_WRONLY | os.O_CREAT, 0o644
                    )
                    atexit.register(os.unlink, Lock.REMOVE_GL_SIGNAL_FILE)
                    release_info = {
                        "releaser": utils.get_real_username(),
                        "reason": release_reason,
                    }
                    Lock._write_json_file(fd, release_info)
                    os.close(fd)

                # Signal file should be picked up right away
                timeout = time.time() + 5
                while time.time() < timeout:
                    if os.path.isfile(Lock.ACK_GL_SIGNAL_FILE):
                        logger.info("Global lock removed")
                        break
                    time.sleep(0.5)
                else:
                    logger.warning(
                        "Could not clear lock. The lock process may not be running, in which case there is no actual lock"
                    )

                return

        logger.info("No global lock set. Nothing to do")

    @staticmethod
    def watch_for_gl_release_signal(release_callback):
        def wait_for_signal(*args):
            while not os.path.isfile(Lock.REMOVE_GL_SIGNAL_FILE):
                time.sleep(0.5)

            with open(Lock.REMOVE_GL_SIGNAL_FILE, encoding="UTF-8") as f:
                release_info = json.loads(f.read())
                releaser = (release_info.get("releaser", "?"),)
                reason = (release_info.get("reason", "?"),)
                logger.info(
                    "Received forced unlock request:\n"
                    "  releaser: %s\n" % releaser + "  reason: %s" % reason
                )

            fd = os.open(Lock.ACK_GL_SIGNAL_FILE, os.O_CREAT, 0o444)
            atexit.register(os.unlink, Lock.ACK_GL_SIGNAL_FILE)
            os.close(fd)

            # Wait for the signaling process to finish its thing before proceeding
            while os.path.isfile(Lock.REMOVE_GL_SIGNAL_FILE):
                time.sleep(0.1)

            release_callback()

        watch_for_release_signal = threading.Thread(target=wait_for_signal)
        watch_for_release_signal.daemon = True
        watch_for_release_signal.start()

    @staticmethod
    def _write_json_file(fd, info):
        os.ftruncate(fd, 0)
        os.pwrite(fd, json.dumps(info, indent=4).encode("UTF-8"), 0)
