# -*- coding: utf-8 -*-
"""
    scap.tasks
    ~~~~~~~~~~
    Contains functions implementing scap tasks

"""
import logging
import os
import random
import socket
import subprocess
import tempfile

from . import utils


def check_php_syntax(*paths):
    """Run lint.php on `paths`; raise CalledProcessError if nonzero exit."""
    cmd = '/usr/bin/php -n -dextension=parsekit.so /usr/local/bin/lint.php'
    return subprocess.check_call(cmd.split() + list(paths))


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


def scap(args):
    """Core business logic of scap process."""
    logger = logging.getLogger('scap')
    env = {}
    if args.versions:
        env['MW_VERSIONS_SYNC'] = ' '.join(args.versions)

    with utils.lock('/var/lock/scap'):
        logger.info('Started scap: %s', args.message)

        logger.debug('Checking syntax')
        check_php_syntax('%(MW_COMMON_SOURCE)s/wmf-config' % args.cfg)
        check_php_syntax('%(MW_COMMON_SOURCE)s/multiversion' % args.cfg)

        # Update the current machine so that serialization works. Push
        # wikiversions.dat changes so mwversionsinuse, set-group-write,
        # and mwscript work with the right version of the files.
        sync_common(dict(args.cfg.items() + env.items()))

        # Update list of extension message files and regenerate the
        # localisation cache.
        subprocess.check_call('/usr/local/bin/mw-update-l10n')

        logger.debug('Updating rsync proxies')
        utils.dsh('/usr/local/bin/scap-1', 'scap-proxies', env)

        scap_proxies = utils.read_dsh_hosts_file('scap-proxies')

        # Randomize the order of target machines.
        mw_install_hosts = utils.read_dsh_hosts_file('mediawiki-installation')
        random.shuffle(mw_install_hosts)
        with tempfile.NamedTemporaryFile(delete=False, prefix='scap') as tmp:
            try:
                tmp.write('\n'.join(mw_install_hosts))
                tmp.flush()

                logger.debug('Copying code to Apaches')
                utils.dsh(
                    '/usr/local/bin/scap-1 "%s"' % ' '.join(scap_proxies),
                    tmp.name, env)

                logger.debug('Rebuilding CDB files')
                utils.dsh('/usr/local/bin/scap-rebuild-cdbs', tmp.name, env)
            finally:
                os.remove(tmp.name)

        logger.debug('Building wikiversions.cdb')
        subprocess.check_call('%(MW_COMMON_SOURCE)s/multiversion/'
                              'refreshWikiversionsCDB' % args.cfg)
        logger.debug('Syncing wikiversions.cdb')
        utils.dsh('sudo -u mwdeploy rsync -l %(MW_RSYNC_HOST)s::common/'
            'wikiversions.{dat,cdb} %(MW_COMMON)s' % args.cfg,
            'mediawiki-installation')
