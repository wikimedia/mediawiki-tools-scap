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


def purge_l10n_cache(version, cfg):
    """Purge the localization cache for a given version.

    :param version: MediaWiki version (eg '1.23wmf15')
    :param cfg: Global configuration
    :raises: :class:`IOError` if l10n cache dirs for the given version are
             not found
    """
    branch_dir = 'php-%s' % version
    staged_l10n = os.path.join(cfg['stage_dir'], branch_dir, 'cache/l10n')
    deployed_l10n = os.path.join(cfg['deploy_dir'], branch_dir, 'cache/l10n')

    if not os.path.isdir(staged_l10n):
        raise IOError(errno.ENOENT, 'Invalid l10n dir', staged_l10n)

    if not os.path.isdir(deployed_l10n):
        raise IOError(errno.ENOENT, 'Invalid l10n dir', deployed_l10n)

    # Purge from staging directory locally
    # Shell is needed on subprocess to allow wildcard expansion
    # --force option given to rm to ignore missing files
    subprocess.check_call(
        'sudo -u l10nupdate /bin/rm --recursive --force %s/*' % staged_l10n,
        shell=True)

    # Purge from deploy directroy across cluster
    # --force option given to rm to ignore missing files as before
    purge = ssh.Job().role('mediawiki-installation')
    purge.command('sudo -u mwdeploy /bin/rm '
        '--recursive --force %s/*' % deployed_l10n)
    purge.progress('l10n purge').run()


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


def sync_wikiversions(hosts, cfg):
    """Rebuild and sync wikiversions.cdb to the cluster.

    :param hosts: List of hosts to sync to
    :param cfg: Global configuration
    """
    stats = log.Stats(cfg['statsd_host'], int(cfg['statsd_port']))
    with log.Timer('sync_wikiversions', stats):
        compile_wikiversions_cdb(cfg)

        rsync = ssh.Job(hosts).shuffle()
        rsync.command('sudo -u mwdeploy /usr/bin/rsync -l '
            '%(master_rsync)s::common/wikiversions*.{json,cdb} '
            '%(deploy_dir)s' % cfg)
        rsync.progress('sync_wikiversions').run()
