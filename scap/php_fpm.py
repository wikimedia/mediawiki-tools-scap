# -*- coding: utf-8 -*-
"""
    scap.php_fpm.py
    ~~~~~~~~~~
    Contains misc utility functions.

"""
from __future__ import absolute_import

import math
import sys

from scap import log
from scap import utils

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

        if unsafe:
            script = cfg.get('php_fpm_unsafe_restart_script')
            if script:
                self.cmd = "{} --force".format(script)
            else:
                if logger:
                    logger.warn("Unsafe mode requested but "
                                "php_fpm_unsafe_restart_script "
                                "not defined in config")

        elif cfg.get('php_fpm_restart_script'):
            threshold = cfg['php_fpm_opcache_threshold']

            # Override opcache threshold in cases we always want to restart
            if cfg.get('php_fpm_always_restart'):
                threshold = sys.maxsize

            self.cmd = "{} {} {}".format(
                cfg['php_fpm_restart_script'],
                cfg['php_fpm'],
                str(threshold)
            )
        elif logger:
            logger.warn("php_fpm_restart_script not defined in config")

    def _build_job(self, targets):
        """
        :param targets: list of targets
        :raises: :class:``AttributeError`` if self.job is not set
        :return: ssh.Job -- used in self.restart_all
        """
        if self.job is None:
            raise AttributeError('php_fpm member "job" is not set')
        self.job.hosts(targets)
        sudo_cmd = '/usr/bin/sudo -u root -- {}'.format(self.cmd)
        self.job.command(sudo_cmd)
        self.job.progress(log.MuteReporter())
        return self.job

    def restart_self(self):
        """
        Run php-fpm restart on the localhost

        If the restart fails, we still want to continue the sync
        :return: boolean -- True if an exception was caught.
        """
        if not self.cmd:
            return False

        try:
            utils.sudo_check_call(cmd=self.cmd, user='root')
            return False
        except Exception:
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
    num_targets = len(targets)

    if not num_targets:
        raise ValueError('php_fpm expected targets, 0 given')

    return int(math.ceil(len(targets) * percentage/100.0))


def restart_helper(targets):
    """
    Wrapper to make pickling with multiprocessing work

    :param targets: list of targets
    :return: tuple -- (# of successful, # of failed)
    """
    if not isinstance(INSTANCE, PHPRestart):
        raise RuntimeError(
            'Attempting to use php_fpm.restart_helper before populating '
            'INSTANCE!'
        )
    return INSTANCE.restart_all(targets)
