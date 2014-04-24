# -*- coding: utf-8 -*-
"""
    scap.tasks
    ~~~~~~~~~~
    Contains functions implementing scap tasks

"""
import errno
import glob
import itertools
import json
import logging
import multiprocessing
import os
import shutil
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


def compile_wikiversions_cdb(source_tree, cfg):
    """Validate and compile the wikiversions.json file into a CDB database.

    1. Find the realm specific filename for wikiversions.json in staging area
    2. Validate that all versions mentioned in the json exist as directories
       in the staging area
    3. Validate that all wikis listed in the realm specific all.dblist exist
       in the json
    4. Create a temporary CDB file from the json contents
    5. Atomically rename the temporary CDB to the realm specific
       wikiversions.cdb filename

    :param source_tree: Source tree to read file from: 'deploy' or 'stage'
    :param cfg: Dict of global configuration values
    """
    logger = logging.getLogger('compile_wikiversions')

    working_dir = cfg['%s_dir' % source_tree]

    # Find the realm specific wikiverisons file names
    base_file = os.path.join(working_dir, 'wikiversions.json')
    json_file = utils.get_realm_specific_filename(
        base_file, cfg['wmf_realm'], cfg['datacenter'])
    cdb_file = '%s.cdb' % os.path.splitext(json_file)[0]

    with open(json_file) as f:
        wikiversions = json.load(f)

    # Validate that all versions in the json file exist locally
    for dbname, version in wikiversions.items():
        version_dir = os.path.join(working_dir, version)
        if not os.path.isdir(version_dir):
            raise IOError(errno.ENOENT, 'Invalid version dir', version_dir)

    # Get the list of all wikis
    all_dblist_file = utils.get_realm_specific_filename(
        os.path.join(working_dir, 'all.dblist'),
        cfg['wmf_realm'], cfg['datacenter'])
    all_dbs = set(line.strip() for line in open(all_dblist_file))

    # Validate that all wikis are in the json file
    missing_dbs = [db for db in wikiversions.keys() if db not in all_dbs]
    if missing_dbs:
        raise KeyError('Missing %d expected dbs in %f: %s' % (
            len(missing_dbs), json_file, ', '.join(missing_dbs)))

    tmp_cdb_file = '%s.tmp' % cdb_file
    try:
        os.unlink(tmp_cdb_file)
    except OSError:
        pass

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
    logger.debug('Compiled %s to %s', json_file, cdb_file)


def purge_l10n_cache(version, cfg):
    """Purge the localization cache for a given version.

    :param version: MediaWiki version (eg '1.23wmf15')
    :param cfg: Dict of global configuration values
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

    :param cfg: Dict of global configuration values
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
    :param cfg: Dict of global configuration values
    """
    stats = log.Stats(cfg['statsd_host'], int(cfg['statsd_port']))
    with log.Timer('sync_wikiversions', stats):
        compile_wikiversions_cdb('stage', cfg)

        rsync = ssh.Job(hosts).shuffle()
        rsync.command('sudo -u mwdeploy /usr/bin/rsync -l '
            '%(master_rsync)s::common/wikiversions*.{json,cdb} '
            '%(deploy_dir)s' % cfg)
        return rsync.progress('sync_wikiversions').run()


def merge_cdb_updates(directory, pool_size, trust_mtime=False):
    """Update l10n CDB files using JSON data.

    :param directory: L10n cache directory
    :param pool_size: Number of parallel processes to use
    :param trust_mtime: Trust file modification time?
    """
    logger = logging.getLogger('merge_cdb_updates')

    cache_dir = os.path.realpath(directory)
    upstream_dir = os.path.join(cache_dir, 'upstream')

    files = [os.path.splitext(os.path.basename(f))[0]
        for f in glob.glob('%s/*.json' % upstream_dir)]
    if not files:
        logger.warning('Directory %s is empty', upstream_dir)
        return 0

    pool = multiprocessing.Pool(pool_size)
    updated = 0

    reporter = log.ProgressReporter('l10n merge')
    reporter.expect(len(files))
    reporter.start()

    for i, result in enumerate(pool.imap_unordered(
        update_l10n_cdb_wrapper, itertools.izip(
            itertools.repeat(cache_dir),
            files,
            itertools.repeat(trust_mtime))), 1):
        if result:
            updated += 1
        reporter.add_success()

    reporter.finish()
    logger.info('Updated %d CDB files(s) in %s', updated, cache_dir)


def update_l10n_cdb(cache_dir, cdb_file, trust_mtime=False):
    """Update a localization CDB database.

    :param cache_dir: L10n cache directory
    :param cdb_file: L10n CDB database
    :param trust_mtime: Trust file modification time?
    """
    logger = logging.getLogger('update_l10n_cdb')

    md5_path = os.path.join(cache_dir, 'upstream', '%s.MD5' % cdb_file)
    if not os.path.exists(md5_path):
        logger.warning('skipped %s; no md5 file', cdb_file)
        return False

    json_path = os.path.join(cache_dir, 'upstream', '%s.json' % cdb_file)
    if not os.path.exists(json_path):
        logger.warning('skipped %s; no json file', cdb_file)
        return False

    cdb_path = os.path.join(cache_dir, cdb_file)

    json_mtime = os.path.getmtime(json_path)
    if os.path.exists(cdb_path):
        if trust_mtime:
            cdb_mtime = os.path.getmtime(cdb_path)
            # If the CDB was built by this process in a previous sync, the CDB
            # file mtime will have been set equal to the json file mtime.
            need_rebuild = cdb_mtime != json_mtime
        else:
            upstream_md5 = open(md5_path).read(100).strip()
            local_md5 = utils.md5_file(cdb_path)
            need_rebuild = local_md5 != upstream_md5
    else:
        need_rebuild = True

    if need_rebuild:
        with open(json_path) as f:
            data = json.load(f)

        # Write temp cdb file
        tmp_cdb_path = '%s.tmp' % cdb_path
        with open(tmp_cdb_path, 'wb') as fp:
            writer = cdblib.Writer(fp)
            for key, value in data.items():
                writer.put(key.encode('utf-8'), value.encode('utf-8'))
            writer.finalize()
            os.fsync(fp.fileno())

        if not os.path.isfile(tmp_cdb_path):
            raise IOError(errno.ENOENT, 'Failed to create CDB', tmp_cdb_path)

        # Move temp file over old file
        os.chmod(tmp_cdb_path, 0664)
        os.rename(tmp_cdb_path, cdb_path)
        # Set timestamp to match upstream json
        os.utime(cdb_path, (json_mtime, json_mtime))
        return True
    return False


def update_l10n_cdb_wrapper(args):
    """Wrapper for update_l10n_cdb to be used in contexts where only a single
    argument can be provided.

    :param args: Sequence of arguments to pass to update_l10n_cdb
    """
    try:
        return update_l10n_cdb(*args)
    except:
        # Log detailed error; multiprocessing will truncate the stack trace
        logging.getLogger('update_l10n_cdb_wrapper').exception(
            'Failure processing %s', args)
        raise


def update_localization_cache(version, wikidb, verbose, cfg):
    """Update the localization cache for a given MW version.

    :param version: MediaWiki version
    :param wikidb: Wiki running given version
    :param verbose: Provide verbose output
    :param cfg: Global configuration
    """
    logger = logging.getLogger('update_localization_cache')

    def check_sudo(user, cmd):
        """Run a command as a specific user. Reports stdout/stderr of process
        to logger during execution.

        :param user: User to run command as
        :param cmd: Command to execute
        :raises: subprocess.CalledProcessError on non-zero process exit
        """
        # Several of the calls in this module fail if a full shell isn't used
        proc = subprocess.Popen('sudo -u %s -- %s' % (user, cmd),
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True)

        while proc.poll() is None:
            line = proc.stdout.readline().strip()
            if line:
                logger.debug(line)

        if proc.returncode:
            raise subprocess.CalledProcessError(proc.retcode, cmd)

    # Calculate the number of parallel threads
    # Leave a couple of cores free for other stuff
    num_threads = multiprocessing.cpu_count() - 2
    if num_threads < 1:
        num_threads = 1

    verbose_messagelist = ''
    force_rebuild = ''
    quiet_rebuild = '--quiet'
    if verbose:
        verbose_messagelist = '--verbose'
        quiet_rebuild = ''

    extension_messages = os.path.join(
        cfg['stage_dir'], 'wmf-config', 'ExtensionMessages-%s.php' % version)

    if not os.path.exists(extension_messages):
        # Touch the extension_messages file to prevent php require errors
        logger.info('Creating empty %s', extension_messages)
        open(extension_messages, 'a').close()

    cache_dir = os.path.join(
        cfg['stage_dir'], 'php-%s' % version, 'cache', 'l10n')

    if not os.path.exists(os.path.join(cache_dir, 'l10n_cache-en.cdb')):
        # mergeMessageFileList.php needs a l10n file
        logger.info('Bootstrapping l10n cache for %s', version)
        check_sudo('l10nupdate',
            '/usr/local/bin/mwscript rebuildLocalisationCache.php '
            '--wiki="%s" --lang=en --outdir="%s" --quiet' % (
                version, cache_dir))
        # Force subsequent cache rebuild to overwrite bootstrap version
        force_rebuild = '--force'

    logger.info('Updating ExtensionMessages-%s.php', version)
    new_extension_messages = subprocess.check_output(
        'sudo -u apache -- /bin/mktemp', shell=True).strip()
    check_sudo('apache', '/usr/local/bin/mwscript mergeMessageFileList.php '
        '--wiki="%s" --list-file="%s/wmf-config/extension-list" '
        '--output="%s" %s' % (
            wikidb, cfg['stage_dir'], new_extension_messages,
            verbose_messagelist))
    check_sudo('apache', 'chmod 0664 "%s"' % new_extension_messages)
    logger.debug('Copying %s to %s' % (
        new_extension_messages, extension_messages))
    shutil.copyfile(new_extension_messages, extension_messages)
    check_sudo('apache', 'rm "%s"' % new_extension_messages)

    # Update ExtensionMessages-*.php in the local copy.
    deploy_dir = os.path.realpath(cfg['deploy_dir'])
    stage_dir = os.path.realpath(cfg['stage_dir'])
    if stage_dir != deploy_dir:
        logger.debug('Copying to local copy')
        check_sudo('mwdeploy', 'cp "%s" "%s/wmf-config/"' % (
            extension_messages, cfg['deploy_dir']))

    # Rebuild all the CDB files for each language
    logger.info('Updating LocalisationCache for %s '
        'using %s thread(s)' % (version, num_threads))
    check_sudo('l10nupdate', '/usr/local/bin/mwscript '
        'rebuildLocalisationCache.php --wiki="%s" --outdir="%s" '
        '--threads=%s %s %s' % (wikidb, cache_dir, num_threads,
            force_rebuild, quiet_rebuild))

    # Include JSON versions of the CDB files and add MD5 files
    logger.info('Generating JSON versions and md5 files')
    check_sudo('l10nupdate', '/usr/local/bin/refreshCdbJsonFiles '
        '--directory="%s" --threads=%s %s' % (
            cache_dir, num_threads, verbose_messagelist))
