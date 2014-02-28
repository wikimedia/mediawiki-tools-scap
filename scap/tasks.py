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

from . import utils
from . import log
from . import ssh


DEFAULT_RSYNC_ARGS = (
    '/usr/bin/rsync',
    '-a',
    '--delete-delay',
    '--delay-updates',
    '--compress',
    '--delete',
    '--exclude=**/.svn/lock',
    '--exclude=**/.git/objects',
    '--exclude=**/.git/**/objects',
    '--exclude=**/cache/l10n/*.cdb',
    '--no-perms')


def check_php_syntax(*paths):
    """Run lint.php on `paths`; raise CalledProcessError if nonzero exit."""
    cmd = '/usr/bin/php -n -dextension=parsekit.so /usr/local/bin/lint.php'
    return subprocess.check_call(cmd.split() + list(paths))


def sync_common(cfg, sync_from=None):
    """Sync MW_COMMON

    Rsync from ``server::common/`` to the local ``MW_COMMON`` directory.
    If a list of servers is given in ``sync_from`` we will attempt to select
    the "best" one to sync from. If no servers are given or all servers given
    have issues we will fall back to using the server named by
    ``MW_RSYNC_HOST`` in the configuration data.

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

    server = None
    if sync_from:
        server = utils.find_nearest_host(sync_from)
    if server is None:
        server = cfg['MW_RSYNC_HOST']
    server = server.strip()

    # Execute rsync fetch locally via sudo
    rsync = ('sudo', '-u', 'mwdeploy') + DEFAULT_RSYNC_ARGS
    rsync += ('%s::common' % server, cfg['MW_COMMON'])

    logger.debug('Copying to %s from %s', socket.getfqdn(), server)
    stats = log.Stats(cfg['MW_STATSD_HOST'], cfg['MW_STATSD_PORT'])
    with log.Timer('rsync common', stats):
        subprocess.check_call(rsync)


def scap(args):
    """Core business logic of scap process.

    1. Validate php syntax of wmf-config and multiversion
    2. Sync MW_COMMON on localhost with staging area
    3. Update l10n files in staging area
    4. Ask scap proxies to sync with master server
    5. Ask apaches to sync with fastest rsync server
    6. Ask apaches to rebuild l10n CDB files
    7. Update wikiversions.cdb on localhost
    8. Ask apaches to sync wikiversions.cdb

    :param args: Command line arguments and configuration
    :type args: argparse.Namespace
    """
    logger = logging.getLogger('scap')
    stats = log.Stats(args.cfg['MW_STATSD_HOST'], args.cfg['MW_STATSD_PORT'])

    with utils.lock('/var/lock/scap'):
        logger.info('Started scap: %s', args.message)

        check_php_syntax('%(MW_COMMON_SOURCE)s/wmf-config' % args.cfg,
                         '%(MW_COMMON_SOURCE)s/multiversion' % args.cfg)

        # Update the current machine so that serialization works. Push
        # wikiversions.json changes so mwversionsinuse, set-group-write,
        # and mwscript work with the right version of the files.
        sync_common(args.cfg)

        # Update list of extension message files and regenerate the
        # localisation cache.
        with log.Timer('mw-update-l10n', stats):
            subprocess.check_call('/usr/local/bin/mw-update-l10n')

        # Update rsync proxies
        scap_proxies = utils.read_dsh_hosts_file('scap-proxies')
        with log.Timer('sync-common to proxies', stats):
            ssh.cluster_monitor(scap_proxies, '/usr/local/bin/sync-common')

        # Randomize the order of target machines.
        mw_install_hosts = utils.read_dsh_hosts_file('mediawiki-installation')
        random.shuffle(mw_install_hosts)

        with log.Timer('update apaches', stats) as t:
            ssh.cluster_monitor(mw_install_hosts,
                    ['/usr/local/bin/sync-common'] + scap_proxies)
            t.mark('sync-common to apaches')

            ssh.cluster_monitor(mw_install_hosts,
                '/usr/local/bin/scap-rebuild-cdbs')
            t.mark('scap-rebuild-cdbs')

        with log.Timer('syncing wikiversions.cdb', stats) as t:
            subprocess.check_call('%(MW_COMMON_SOURCE)s/multiversion/'
                'refreshWikiversionsCDB' % args.cfg)
            ssh.cluster_monitor(mw_install_hosts,
                'sudo -u mwdeploy /usr/bin/rsync -l '
                '%(MW_RSYNC_HOST)s::common/wikiversions.{json,cdb} '
                '%(MW_COMMON)s' % args.cfg)
