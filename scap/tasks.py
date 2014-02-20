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
from . import log


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

    # FIXME: Why is this hardcoded?
    # FIXME: Why is this even here? uncommon isn't touched
    uncommon = '/usr/local/apache/uncommon'
    if not os.path.isdir(uncommon):
        raise Exception(('directory %s not found. '
            'Ask root to create it.') % uncommon)

    server = None
    if sync_from:
        server = utils.find_nearest_host(sync_from)
    if server is None:
        server = cfg['MW_RSYNC_HOST']
    server = server.strip()
    versions = cfg.get('MW_VERSIONS_SYNC', '').split()

    # Execute rsync fetch locally via sudo
    rsync = DEFAULT_RSYNC_ARGS
    if versions:
        # Only sync provided versions
        rsync += tuple("--include='php-%s'" % v for v in versions)
        rsync += ("--exclude='php-*/'",)
    rsync += ('%s::common' % server, cfg['MW_COMMON'])

    logger.debug('Copying to %s from %s', socket.getfqdn(), server)
    stats = log.Stats(cfg['MW_STATSD_HOST'], cfg['MW_STATSD_PORT'])
    with log.Timer('rsync common', stats):
        subprocess.check_call(utils.sudo_args(rsync, user='mwdeploy'))


def scap(args):
    """Core business logic of scap process.

    1. Validate php syntax of wmf-config and multiversion
    2. Sync MW_COMMON on localhost with staging area
    3. Update l10n files in staging area
    4. Dsh to scap proxies and sync with localhost
    5. Dsh to apaches and sync with fastest rsync server
    6. Dsh to apaches and rebuild l10n CDB files
    7. Update wikiversions.cdb on localhost
    8. Dsh to apaches and rsync wikiversions.cdb

    :param args: Command line arguments and configuration
    :type args: argparse.Namespace
    """
    logger = logging.getLogger('scap')
    stats = log.Stats(args.cfg['MW_STATSD_HOST'], args.cfg['MW_STATSD_PORT'])
    env = {}
    if args.versions:
        env['MW_VERSIONS_SYNC'] = ' '.join(args.versions)

    with utils.lock('/var/lock/scap'):
        logger.info('Started scap: %s', args.message)

        with log.Timer('php syntax check'):
            check_php_syntax('%(MW_COMMON_SOURCE)s/wmf-config' % args.cfg)
            check_php_syntax('%(MW_COMMON_SOURCE)s/multiversion' % args.cfg)

        # Update the current machine so that serialization works. Push
        # wikiversions.json changes so mwversionsinuse, set-group-write,
        # and mwscript work with the right version of the files.
        sync_common(dict(args.cfg.items() + env.items()))

        # Update list of extension message files and regenerate the
        # localisation cache.
        with log.Timer('mw-update-l10n', stats):
            subprocess.check_call('/usr/local/bin/mw-update-l10n')

        with log.Timer('scap-1 to proxies', stats):
            utils.dsh('/usr/local/bin/scap-1', 'scap-proxies', env)

        scap_proxies = utils.read_dsh_hosts_file('scap-proxies')

        # Randomize the order of target machines.
        mw_install_hosts = utils.read_dsh_hosts_file('mediawiki-installation')
        random.shuffle(mw_install_hosts)
        with tempfile.NamedTemporaryFile(delete=False, prefix='scap') as tmp:
            try:
                tmp.write('\n'.join(mw_install_hosts))
                tmp.flush()

                with log.Timer('update apaches', stats) as t:
                    utils.dsh(
                        '/usr/local/bin/scap-1 %s' % ' '.join(scap_proxies),
                        tmp.name, env)
                    t.mark('scap-1 to apaches')

                    utils.dsh('/usr/local/bin/scap-rebuild-cdbs',
                        tmp.name, env)
                    t.mark('scap-rebuild-cdbs')
            finally:
                os.remove(tmp.name)

        with log.Timer('syncing wikiversions.cdb', stats) as t:
            subprocess.check_call('%(MW_COMMON_SOURCE)s/multiversion/'
                'refreshWikiversionsCDB' % args.cfg)
            t.mark('refreshWikiversionsCDB')

            utils.dsh('sudo -u mwdeploy rsync -l %(MW_RSYNC_HOST)s::common/'
                'wikiversions.{json,cdb} %(MW_COMMON)s' % args.cfg,
                'mediawiki-installation')
            t.mark('rsync wikiversions')
