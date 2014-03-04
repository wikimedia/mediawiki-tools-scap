# -*- coding: utf-8 -*-
"""
    scap.tasks
    ~~~~~~~~~~
    Contains functions implementing scap tasks

"""
import errno
import json
import logging
import os
import random
import socket
import subprocess

from . import cdblib
from . import log
from . import ssh
from . import utils


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


def compile_wikiversions_cdb(cfg):
    """Validate and compile the wikiversions.json file into a CDB database.

    1. Find the realm specific filename for wikiversions.json in staging area
    2. Validate that all versions mentioned in the json exist as directories
       in the staging area
    3. Validate that all wikis listed in the realm specific all.dblist exist
       in the json
    4. Create a temporary CDB file from the json contents
    5. Atomically rename the temporary CDB to the realm specific
       wikiversions.cdb filename

    :param cfg: Global configuration
    """

    # Find the realm specific wikiverisons file names
    base_file = os.path.join(cfg['stage_dir'], 'wikiversions.json')
    json_file = utils.get_realm_specific_filename(
        base_file, cfg['wmf_realm'], cfg['datacenter'])
    cdb_file = '%s.cdb' % os.path.splitext(json_file)[0]

    with open(json_file) as f:
        wikiversions = json.load(f)

    # Validate that all versions in the json file exist locally
    for dbname, version in wikiversions.items():
        version_dir = os.path.join(cfg['stage_dir'], version)
        if not os.path.isdir(version_dir):
            raise IOError(errno.ENOENT, 'Invalid version dir', version_dir)

    # Get the list of all wikis
    all_dblist_file = utils.get_realm_specific_filename(
        os.path.join(cfg['stage_dir'], 'all.dblist'),
        cfg['wmf_realm'], cfg['datacenter'])
    all_dbs = set(line.strip() for line in open(all_dblist_file))

    # Validate that all wikis are in the json file
    missing_dbs = [db for db in wikiversions.keys() if db not in all_dbs]
    if missing_dbs:
        raise KeyError('Missing %d expected dbs in %f: %s' % (
            len(missing_dbs), json_file, ', '.join(missing_dbs)))

    tmp_cdb_file = '%s.tmp' % cdb_file
    if os.path.exists(tmp_cdb_file):
        os.unlink(tmp_cdb_file)

    # Write temp cdb file
    with open(tmp_cdb_file, 'wb') as fp:
        writer = cdblib.Writer(fp)
        for dbname, version in wikiversions.items():
            writer.put(str('ver:%s' % dbname), str(version))
        writer.finalize()
        os.fsync(fp.fileno())

    if not os.path.isfile(tmp_cdb_file):
        raise IOError(errno.ENOENT, 'Failed to create CDB', tmp_cdb_file)

    os.rename(tmp_cdb_file, cdb_file)
    os.chmod(cdb_file, 0664)


def sync_common(cfg, sync_from=None):
    """Sync local deploy dir with upstream rsync server's copy

    Rsync from ``server::common/`` to the local deploy directory.
    If a list of servers is given in ``sync_from`` we will attempt to select
    the "best" one to sync from. If no servers are given or all servers given
    have issues we will fall back to using the server named by
    ``master_rsync`` in the configuration data.

    :param cfg: Global configuration
    :param sync_from: List of rsync servers to fetch from.
    """
    logger = logging.getLogger('sync_common')

    if not os.path.isdir(cfg['deploy_dir']):
        raise Exception(('rsync target directory %s not found. '
            'Ask root to create it.') % cfg['deploy_dir'])

    server = None
    if sync_from:
        server = utils.find_nearest_host(sync_from)
    if server is None:
        server = cfg['master_rsync']
    server = server.strip()

    # Execute rsync fetch locally via sudo
    rsync = ('sudo', '-u', 'mwdeploy') + DEFAULT_RSYNC_ARGS
    rsync += ('%s::common' % server, cfg['deploy_dir'])

    logger.debug('Copying to %s from %s', socket.getfqdn(), server)
    stats = log.Stats(cfg['statsd_host'], int(cfg['statsd_port']))
    with log.Timer('rsync common', stats):
        subprocess.check_call(rsync)


def scap(message, cfg):
    """Core business logic of scap process.

    1. Validate php syntax of wmf-config and multiversion
    2. Sync deploy directory on localhost with staging area
    3. Update l10n files in staging area
    4. Ask scap proxies to sync with master server
    5. Ask apaches to sync with fastest rsync server
    6. Ask apaches to rebuild l10n CDB files
    7. Update wikiversions.cdb on localhost
    8. Ask apaches to sync wikiversions.cdb

    :param message: Reason for running scap
    :param cfg: Configuration settings
    """
    logger = logging.getLogger('scap')
    stats = log.Stats(cfg['statsd_host'], int(cfg['statsd_port']))

    with utils.lock('/var/lock/scap'):
        logger.info('Started scap: %s', message)

        check_php_syntax('%(stage_dir)s/wmf-config' % cfg,
                         '%(stage_dir)s/multiversion' % cfg)

        # Update the current machine so that serialization works. Push
        # wikiversions.json changes so mwversionsinuse, set-group-write,
        # and mwscript work with the right version of the files.
        sync_common(cfg)

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
            compile_wikiversions_cdb(cfg)
            ssh.cluster_monitor(mw_install_hosts,
                'sudo -u mwdeploy /usr/bin/rsync -l '
                '%(master_rsync)s::common/wikiversions.{json,cdb} '
                '%(deploy_dir)s' % cfg)
