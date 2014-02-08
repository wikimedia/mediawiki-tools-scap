# -*- coding: utf-8 -*-
"""
    scap.tasks
    ~~~~~~~~~~
    Contains functions implementing scap tasks

"""
import logging
import os
import socket
import subprocess

from . import utils


def sync_common(cfg, sync_from=None):
    """Sync MW_COMMON

    Rsync the from ``server::common/`` to the local ``MW_COMMON`` directory.
    If a space separated list of servers is given in ``sync_from`` we will
    attempt to select the "best" one to sync from. If no servers are given or
    all servers given have issues we will fall back to using the server named
    by ``MW_RSYNC_HOST`` in the configuration data.

    :param cfg: Global configuration
    :type cfg: dict
    :param sync_from: Rsync servers to fetch from.
    :type sync_from: list
    """
    logger = logging.getLogger('sync_common')
    target = cfg['MW_COMMON']
    if not os.path.isdir(target):
        raise Exception(('rsync target directory %s not found. '
            'Ask root to create it.') % target)

    # FIXME: Why is this hardcoded?
    # FIXME: Why is this even here? uncommon isn't touched
    uncommon = '/usr/local/apache/uncommon'
    if not os.path.isdir(uncommon):
        raise Exception(('directory %s not found. '
            'Ask root to create it.') % uncommon)

    server = None
    if sync_from:
        server = subprocess.check_output(utils.sudo_args(
            ['/usr/local/bin/find-nearest-rsync'] + sync_from))
    if server is None:
        server = cfg['MW_RSYNC_HOST']
    logger.debug('Rsyncing from %s to %s', server, socket.getfqdn())

    subprocess.check_call(utils.sudo_args(
        ['/usr/local/bin/scap-2', server], user='mwdeploy',
        exports={
            'MW_VERSIONS_SYNC': cfg.get('MW_VERSIONS_SYNC', ''),
            'MW_SCAP_BETA': cfg.get('MW_SCAP_BETA', ''),
        }))
