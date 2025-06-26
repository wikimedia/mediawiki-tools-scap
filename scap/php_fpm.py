# -*- coding: utf-8 -*-
"""
    scap.php_fpm.py
    ~~~~~~~~~~
    Contains misc utility functions.

"""
import math
import shlex
import subprocess

from scap import log

INSTANCE = None


class PHPRestart(object):
    """
    Handle PHP fpm restarts if needed.
    """

    def __init__(self, cfg, job=None, unsafe=False, logger=None):
        """
        :param cfg: dict - scap configuration
        :param unsafe: boolean - If true, an unsafe service restart
                       (no depool/repool) will be performed.
        :param logger: logger - logger to use to report problems

        Prepares self.cmd for later use by _build_job.
        """
        self.cmd = None
        self.job = job
        self.ssh_user = cfg["ssh_user"]
        self.logger = logger

        restart_script = cfg.get("php_fpm_restart_script")

        if not restart_script:
            if logger:
                logger.debug("php_fpm_restart_script not configured")
            return

        self.cmd = restart_script

        if unsafe:
            self.cmd += " --force"

    def set_progress_queue(self, queue):
        self.progress_queue = queue

    def _build_job(self, targets):
        """
        :param targets: list of targets
        :raises: :class:``AttributeError`` if self.job is not set
        :return: ssh.Job -- used in self.restart_all
        """
        if self.job is None:
            raise AttributeError('php_fpm member "job" is not set')
        self.job.hosts(targets)
        sudo_cmd = "/usr/bin/sudo -u root -- {}".format(self.cmd)
        self.job.command(sudo_cmd)
        self.job.progress(
            log.QueueReporter(name="php-fpm-restart", queue=self.progress_queue)
        )
        return self.job

    def restart_self(self):
        """
        Run php-fpm restart on the localhost

        If the restart fails, we still want to continue the sync
        :return: boolean -- True if an exception was caught.
        """
        if not self.cmd:
            return False

        # We need to run the restart command as root,
        # and while both the deployment group and the ssh_user can run that command,
        # not every deployer has the etcd credentials to depool/pool a server.
        # So we need to run the command as ssh_user

        try:
            cmd = "/usr/bin/sudo -u {} /usr/bin/sudo -u root -- {}".format(
                self.ssh_user, self.cmd
            )
            if self.logger:
                self.logger.debug("Running %s", cmd)
            subprocess.check_call(shlex.split(cmd))
            return False
        except subprocess.CalledProcessError:
            return True

    def restart_all(self, targets):
        """
        Run for all targets
        :param targets: list of servers
        :return: tuple -- (# of successful, # of failed)
        """
        if not self.cmd:
            return (0, 0)  # 0 successful, 0 failed
        ten_percent = get_batch_size(targets)
        return self._build_job(targets).run(batch_size=ten_percent)


def get_batch_size(targets, percentage=10.0):
    """
    Batch size for a given size of targets

    :param targets: list
    :param percentage: float
    :raises: :class:``ValueError`` if targets is empty

    :returns: int batch size based on percentage
    """
    return int(math.ceil(len(targets) * percentage / 100.0))


def restart_helper(targets):
    """
    Wrapper to make pickling with multiprocessing work

    :param targets: list of targets
    :return: tuple -- (# of successful, # of failed)
    """
    if not isinstance(INSTANCE, PHPRestart):
        raise RuntimeError(
            "Attempting to use php_fpm.restart_helper before populating " "INSTANCE!"
        )
    return INSTANCE.restart_all(targets)
