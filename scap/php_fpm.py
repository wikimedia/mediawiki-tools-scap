# -*- coding: utf-8 -*-
"""
    scap.php_fpm.py
    ~~~~~~~~~~
    Contains misc utility functions.

"""
from __future__ import absolute_import

import math

from scap import utils


class PHPRestart(object):
    """
    Handle PHP fpm restarts if needed.
    """
    CHECK_FMT = 'Checking if {phpfpm} opcache restart needed for "{host}"'

    def __init__(self, cfg, job=None):
        """
        :param cfg: dict - scap configuration
        """
        self.cmd = []

        if cfg.get('php_fpm_restart_script'):
            self.cmd = [
                cfg['php_fpm_restart_script'],
                cfg['php_fpm'],
                str(cfg['php_fpm_opcache_threshold'])
            ]
        self.job = job

    def _build_job(self, targets):
        """
        :param targets: list of targets
        :raises: :class:``AttributeError`` if self.job is not set
        """
        if self.job is None:
            raise AttributeError('php_fpm member "job" is not set')
        self.job.hosts(targets)
        self.job.command(self.cmd)

    def restart_self(self, user='mwdeploy'):
        """
        Run php-fpm restart on the localhost

        If the restart fails, we still want to continue the sync
        :return: boolean -- has and error
        """
        if not self.cmd:
            return

        try:
            utils.sudo_check_call(cmd=self.cmd, user=user)
            return False
        except Exception:
            return True

    def restart_all(self, targets):
        """
        Run for all targets
        :param targets: list of servers
        """
        if not self.cmd:
            return
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
