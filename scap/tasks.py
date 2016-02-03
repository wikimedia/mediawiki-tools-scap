# -*- coding: utf-8 -*-
"""
    scap.tasks
    ~~~~~~~~~~
    Contains functions implementing scap tasks

"""
import collections
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
import time
import tempfile

from . import cdblib
from . import git
from . import log
from . import ssh
from . import targets
from . import utils


DEFAULT_RSYNC_ARGS = [
    '/usr/bin/rsync',
    '--archive',
    '--delete-delay',
    '--delay-updates',
    '--compress',
    '--delete',
    '--exclude=**/cache/l10n/*.cdb',
    '--exclude=*.swp',
    '--no-perms',
]


def cache_git_info(version, cfg):
    """Create JSON cache files of git branch information.

    :param version: MediaWiki version (eg '1.23wmf15')
    :param cfg: Dict of global configuration values
    :raises: :class:`IOError` if version directory is not found
    """
    branch_dir = os.path.join(cfg['stage_dir'], 'php-%s' % version)

    if not os.path.isdir(branch_dir):
        raise IOError(errno.ENOENT, 'Invalid branch directory', branch_dir)

    # Create cache directory if needed
    cache_dir = os.path.join(branch_dir, 'cache', 'gitinfo')
    if not os.path.isdir(cache_dir):
        os.mkdir(cache_dir)

    # Create cache for branch
    info = git.info(branch_dir)
    cache_file = git.info_filename(branch_dir, branch_dir, cache_dir)
    with open(cache_file, 'w') as f:
        json.dump(info, f)

    # Create cache for each extension and skin
    for dirname in ['extensions', 'skins']:
        dir = os.path.join(branch_dir, dirname)
        for subdir in utils.iterate_subdirectories(dir):
            try:
                info = git.info(subdir)
            except IOError:
                pass
            else:
                cache_file = git.info_filename(subdir, branch_dir, cache_dir)
                with open(cache_file, 'w') as f:
                    json.dump(info, f)


def check_valid_syntax(*paths):
    """Run php -l in parallel on `paths`; raise CalledProcessError if nonzero
    exit."""
    logger = logging.getLogger('check_php_syntax')
    quoted_paths = ["'%s'" % x for x in paths]
    cmd = (
        "find "
        "-O2 "  # -O2 get -type executed after
        "%s "
        "-not -type d "  # makes no sense to lint a dir named 'less.php'
        "-name '*.php' -or -name '*.inc' -or -name '*.phtml' "
        " -or -name '*.php5' | xargs -n1 -P%d -exec php -l >/dev/null"
    ) % (' '.join(quoted_paths), multiprocessing.cpu_count())
    logger.debug('Running command: `%s`', cmd)
    subprocess.check_call(cmd, shell=True)
    # Check for anything that isn't a shebang before <?php (T92534)
    for path in paths:
        for root, dirs, files in os.walk(path):
            for filename in files:
                abspath = os.path.join(root, filename)
                utils.check_php_opening_tag(abspath)
                utils.check_valid_json_file(abspath)


@utils.log_context('compile_wikiversions')
def compile_wikiversions(source_tree, cfg, logger=None):
    """Validate and compile the wikiversions.json file.

    1. Find the realm specific filename for wikiversions.json in staging area
    2. Validate that all versions mentioned in the json exist as directories
       in the staging area
    3. Validate that all wikis listed in the realm specific all.dblist exist
       in the json
    4. Create a temporary CDB file from the json contents
    5. Create a temporary php file from the json contents
    6. Atomically rename the temporary php to the realm specific
       wikiversions.php filename

    :param source_tree: Source tree to read file from: 'deploy' or 'stage'
    :param cfg: Dict of global configuration values
    """

    working_dir = cfg['%s_dir' % source_tree]

    # Find the realm specific wikiversions file names
    base_file = os.path.join(working_dir, 'wikiversions.json')
    json_file = utils.get_realm_specific_filename(
        base_file, cfg['wmf_realm'], cfg['datacenter'])
    base_name = os.path.splitext(json_file)[0]
    php_file = base_name + '.php'

    with open(json_file) as f:
        wikiversions = json.load(f)

    # Validate that all versions in the json file exist locally
    for dbname, version in wikiversions.items():
        version_dir = os.path.join(working_dir, version)
        if not os.path.isdir(version_dir):
            raise IOError(errno.ENOENT, 'Invalid version dir', version_dir)

    # Get the list of all wikis
    all_dblist_file = utils.get_realm_specific_filename(
        os.path.join(working_dir, 'dblists', 'all.dblist'),
        cfg['wmf_realm'], cfg['datacenter'])
    all_dbs = set(line.strip() for line in open(all_dblist_file))

    # Validate that all wikis are in the json file
    missing_dbs = [db for db in wikiversions.keys() if db not in all_dbs]
    if missing_dbs:
        raise KeyError('Missing %d expected dbs in %s: %s' % (
            len(missing_dbs), json_file, ', '.join(missing_dbs)))

    # Build the php version
    php_code = '<?php\nreturn array(\n%s\n);\n' % json.dumps(
        wikiversions,
        separators=(',', ' => '),
        sort_keys=True,
        indent=4
    ).strip('{}\n')

    tmp_php_file = '%s.tmp' % php_file
    try:
        os.unlink(tmp_php_file)
    except OSError:
        pass

    with open(tmp_php_file, 'wt') as fp:
        fp.write(php_code)
        fp.flush()
        utils.eintr_retry(os.fsync, fp.fileno())

    if not os.path.isfile(tmp_php_file):
        raise IOError(
            errno.ENOENT, 'Failed to create php wikiversions', tmp_php_file)

    os.rename(tmp_php_file, php_file)
    os.chmod(php_file, 0664)
    logger.info('Compiled %s to %s', json_file, php_file)


@utils.log_context('merge_cdb_updates')
def merge_cdb_updates(directory, pool_size, trust_mtime=False, mute=False,
                      logger=None):
    """Update l10n CDB files using JSON data.

    :param directory: L10n cache directory
    :param pool_size: Number of parallel processes to use
    :param trust_mtime: Trust file modification time?
    :param mute: Disable progress indicator
    """

    cache_dir = os.path.realpath(directory)
    upstream_dir = os.path.join(cache_dir, 'upstream')

    files = [os.path.splitext(os.path.basename(f))[0]
             for f in glob.glob('%s/*.json' % upstream_dir)]
    if not files:
        logger.warning('Directory %s is empty', upstream_dir)
        return 0

    pool = multiprocessing.Pool(pool_size)
    updated = 0

    if mute:
        reporter = log.MuteReporter()
    else:
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
    utils.sudo_check_call(
        'l10nupdate', '/bin/rm --recursive --force %s/*' % staged_l10n)

    # Purge from deploy directroy across cluster
    # --force option given to rm to ignore missing files as before
    target_obj = targets.get(cfg)
    target_list = target_obj.get_deploy_groups('dsh_targets')['all_targets']
    purge = ssh.Job(user=cfg['ssh_user']).hosts(target_list)
    purge.command('sudo -u mwdeploy -n -- /bin/rm '
                  '--recursive --force %s/*' % deployed_l10n)
    purge.progress('l10n purge').run()


@utils.log_context('sync_master')
def sync_master(cfg, master, verbose=False, logger=None):
    """Sync local staging dir with upstream rsync server's copy

    Rsync from ``server::common`` to the local staging directory.

    :param cfg: Dict of global configuration values.
    :param master: Master server to sync with
    :param verbose: Enable verbose logging?
    """

    if not os.path.isdir(cfg['stage_dir']):
        raise Exception((
            'rsync target directory %s not found. Ask root to create it '
            '(should belong to root:wikidev).') % cfg['stage_dir'])

    # Execute rsync fetch locally via sudo and wrapper script
    rsync = ['sudo', '-n', '--', '/usr/local/bin/scap-master-sync', master]

    logger.info('Copying to %s from %s', socket.getfqdn(), master)
    logger.debug('Running rsync command: `%s`', ' '.join(rsync))
    stats = log.Stats(cfg['statsd_host'], int(cfg['statsd_port']))
    with log.Timer('rsync master', stats):
        subprocess.check_call(rsync)

    scap_rebuild_cdbs = os.path.join(cfg['bin_dir'], 'scap-rebuild-cdbs')

    # Rebuild the CDB files from the JSON versions
    with log.Timer('rebuild CDB staging files', stats):
        subprocess.check_call(['sudo', '-u', 'l10nupdate', '-n', '--',
                              scap_rebuild_cdbs, '--no-progress', '--staging',
                              '--verbose'])


@utils.log_context('sync_common')
def sync_common(cfg, include=None, sync_from=None, verbose=False, logger=None):
    """Sync local deploy dir with upstream rsync server's copy

    Rsync from ``server::common`` to the local deploy directory.
    If a list of servers is given in ``sync_from`` we will attempt to select
    the "best" one to sync from. If no servers are given or all servers given
    have issues we will fall back to using the server named by
    ``master_rsync`` in the configuration data.

    :param cfg: Dict of global configuration values.
    :param include: List of rsync include patterns to limit the sync to. If
        ``None`` is given the entire ``common`` module on the target rsync
        server will be transferred. Rsync syntax for syncing a directory is
        ``<dirname>/***``.
    :param sync_from: List of rsync servers to fetch from.
    """

    if not os.path.isdir(cfg['deploy_dir']):
        raise Exception((
            'rsync target directory %s not found. Ask root to create it '
            '(should belong to mwdeploy:mwdeploy).') % cfg['deploy_dir'])

    server = None
    if sync_from:
        server = utils.find_nearest_host(sync_from)
    if server is None:
        server = cfg['master_rsync']
    server = server.strip()

    # Execute rsync fetch locally via sudo
    rsync = ['sudo', '-u', 'mwdeploy', '-n', '--'] + DEFAULT_RSYNC_ARGS
    # Exclude .git metadata
    rsync.append('--exclude=**/.git')
    if verbose:
        rsync.append('--verbose')

    if include:
        for path in include:
            rsync.append('--include=/%s' % path)
        # Exclude everything not explicitly included
        rsync.append('--exclude=*')

    rsync.append('%s::common' % server)
    rsync.append(cfg['deploy_dir'])

    logger.info('Copying to %s from %s', socket.getfqdn(), server)
    logger.debug('Running rsync command: `%s`', ' '.join(rsync))
    stats = log.Stats(cfg['statsd_host'], int(cfg['statsd_port']))
    with log.Timer('rsync common', stats):
        subprocess.check_call(rsync)

    # Bug 58618: Invalidate local configuration cache by updating the
    # timestamp of wmf-config/InitialiseSettings.php
    settings_path = os.path.join(
        cfg['deploy_dir'], 'wmf-config', 'InitialiseSettings.php')
    logger.debug('Touching %s', settings_path)
    subprocess.check_call(('sudo', '-u', 'mwdeploy', '-n', '--',
                          '/usr/bin/touch', settings_path))


def sync_wikiversions(hosts, cfg):
    """Rebuild and sync wikiversions.php to the cluster.

    :param hosts: List of hosts to sync to
    :param cfg: Dict of global configuration values
    """
    stats = log.Stats(cfg['statsd_host'], int(cfg['statsd_port']))
    with log.Timer('sync_wikiversions', stats):
        compile_wikiversions('stage', cfg)

        rsync = ssh.Job(hosts, user=cfg['ssh_user']).shuffle()
        rsync.command('sudo -u mwdeploy -n -- /usr/bin/rsync -l '
                      '%(master_rsync)s::common/wikiversions*.{json,php} '
                      '%(deploy_dir)s' % cfg)
        return rsync.progress('sync_wikiversions').run()


@utils.log_context('update_l10n_cdb')
def update_l10n_cdb(cache_dir, cdb_file, trust_mtime=False, logger=None):
    """Update a localization CDB database.

    :param cache_dir: L10n cache directory
    :param cdb_file: L10n CDB database
    :param trust_mtime: Trust file modification time?
    """

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
            need_rebuild = not utils.isclose(cdb_mtime, json_mtime)
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
            utils.eintr_retry(os.fsync, fp.fileno())

        if not os.path.isfile(tmp_cdb_path):
            raise IOError(errno.ENOENT, 'Failed to create CDB', tmp_cdb_path)

        # Move temp file over old file
        os.chmod(tmp_cdb_path, 0664)
        os.rename(tmp_cdb_path, cdb_path)
        # Set timestamp to match upstream json
        os.utime(cdb_path, (json_mtime, json_mtime))
        return True
    return False


@utils.log_context('update_l10n_cdb_wrapper')
def update_l10n_cdb_wrapper(args, logger=None):
    """Wrapper for update_l10n_cdb to be used in contexts where only a single
    argument can be provided.

    :param args: Sequence of arguments to pass to update_l10n_cdb
    """
    try:
        return update_l10n_cdb(*args)
    except:
        # Log detailed error; multiprocessing will truncate the stack trace
        logger.exception('Failure processing %s', args)
        raise


def _call_rebuildLocalisationCache(wikidb, out_dir, use_cores=1,
                                   lang=None, force=False, quiet=False):
    """Helper for update_localization_cache

    :param wikidb: Wiki running given version
    :param out_dir: The output directory
    :param use_cores: The number of cores to run in
    :param lang: The --lang option, or None to omit
    :param force: Whether to pass --force
    :param quiet: Whether to pass --quiet
    """

    with utils.sudo_temp_dir('www-data', 'scap_l10n_') as temp_dir:
        # Seed the temporary directory with the current CDB files
        if glob.glob('%s/*.cdb' % out_dir):
            utils.sudo_check_call(
                'www-data',
                "cp '%(out_dir)s/'*.cdb '%(temp_dir)s'" % {
                    'temp_dir': temp_dir,
                    'out_dir': out_dir
                })

        # Generate the files into a temporary directory as www-data
        utils.sudo_check_call(
            'www-data',
            '/usr/local/bin/mwscript rebuildLocalisationCache.php '
            '--wiki="%(wikidb)s" --outdir="%(temp_dir)s" '
            '--threads=%(use_cores)s %(lang)s %(force)s %(quiet)s' % {
                'wikidb': wikidb,
                'temp_dir': temp_dir,
                'use_cores': use_cores,
                'lang': '--lang ' + lang if lang else '',
                'force': '--force' if force else '',
                'quiet': '--quiet' if quiet else ''
            })

        # Copy the files into the real directory as l10nupdate
        utils.sudo_check_call(
            'l10nupdate',
            'cp -r "%(temp_dir)s"/* "%(out_dir)s"' % {
                'temp_dir': temp_dir,
                'out_dir': out_dir
            })


@utils.log_context('update_localization_cache')
def update_localization_cache(version, wikidb, verbose, cfg, logger=None):
    """Update the localization cache for a given MW version.

    :param version: MediaWiki version
    :param wikidb: Wiki running given version
    :param verbose: Provide verbose output
    :param cfg: Global configuration
    """

    # Calculate the number of parallel threads
    # Leave a couple of cores free for other stuff
    use_cores = max(multiprocessing.cpu_count() - 2, 1)

    verbose_messagelist = ''
    force_rebuild = False
    quiet_rebuild = True
    if verbose:
        verbose_messagelist = '--verbose'
        quiet_rebuild = False

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
        _call_rebuildLocalisationCache(wikidb, cache_dir, use_cores,
                                       lang='en', quiet=True)
        # Force subsequent cache rebuild to overwrite bootstrap version
        force_rebuild = True

    logger.info('Updating ExtensionMessages-%s.php', version)
    new_extension_messages = subprocess.check_output(
        'sudo -u www-data -n -- /bin/mktemp', shell=True).strip()

    # attempt to read extension-list from the branch instead of wmf-config
    ext_list = os.path.join(cfg['stage_dir'], "php-%s" % version,
                            "extension-list")

    if not os.path.isfile(ext_list):
        # fall back to the old location in wmf-config
        ext_list = "%s/wmf-config/extension-list" % cfg['stage_dir']

    utils.sudo_check_call(
        'www-data',
        '/usr/local/bin/mwscript mergeMessageFileList.php '
        '--wiki="%s" --list-file="%s" '
        '--output="%s" %s' % (
            wikidb, ext_list, new_extension_messages, verbose_messagelist))

    utils.sudo_check_call('www-data',
                          'chmod 0664 "%s"' % new_extension_messages)
    logger.debug('Copying %s to %s' % (
        new_extension_messages, extension_messages))
    shutil.copyfile(new_extension_messages, extension_messages)
    utils.sudo_check_call('www-data', 'rm "%s"' % new_extension_messages)

    # Update ExtensionMessages-*.php in the local copy.
    deploy_dir = os.path.realpath(cfg['deploy_dir'])
    stage_dir = os.path.realpath(cfg['stage_dir'])
    if stage_dir != deploy_dir:
        logger.debug('Copying ExtensionMessages-*.php to local copy')
        utils.sudo_check_call(
            'mwdeploy',
            'cp "%s" "%s/wmf-config/"' % (
                extension_messages, cfg['deploy_dir']))

    # Rebuild all the CDB files for each language
    logger.info('Updating LocalisationCache for %s '
                'using %s thread(s)' % (version, use_cores))
    _call_rebuildLocalisationCache(wikidb, cache_dir, use_cores,
                                   force=force_rebuild, quiet=quiet_rebuild)

    # Include JSON versions of the CDB files and add MD5 files
    logger.info('Generating JSON versions and md5 files')
    utils.sudo_check_call(
        'l10nupdate',
        '%s/refreshCdbJsonFiles '
        '--directory="%s" --threads=%s %s' % (
            cfg['bin_dir'], cache_dir, use_cores, verbose_messagelist))


def refresh_cdb_json_files(in_dir, pool_size, verbose):
    """Update json files from corresponding cdb file in parallel

    :param in_dir: directory containing cdb files
    :param pool_size: number of "threads" to use
    :param verbose: output verbosely
    """
    logger = utils.get_logger()
    cdb_files = glob.glob(os.path.join(in_dir, '*.cdb'))
    pool = multiprocessing.Pool(pool_size)

    reporter = log.MuteReporter()
    updated = 0

    if verbose:
        reporter = log.ProgressReporter('cdb update')

    reporter.expect(len(cdb_files))
    reporter.start()

    for result in pool.imap_unordered(refresh_cdb_json_file, cdb_files):
        if result:
            updated += 1
        reporter.add_success()

    reporter.finish()
    logger.info('Updated {} JSON file(s) in {}'.format(updated, in_dir))


def refresh_cdb_json_file(file_path):
    """Rebuild json file from cdb file

    #. Check md5 file saved in upstream against md5 of cdb file
    #. Read cdb file to dict
    #. Write dict to named temporary file
    #. Change permissions on named temporary file
    #. Overwrite upstream json file
    #. Write upstream md5 file
    """
    cdb_dir = os.path.dirname(file_path)
    file_name = os.path.basename(file_path)
    upstream_dir = os.path.join(cdb_dir, 'upstream')
    upstream_md5 = os.path.join(upstream_dir, '{}.MD5'.format(file_name))
    upstream_json = os.path.join(upstream_dir, '{}.json'.format(file_name))

    logger = utils.get_logger()
    logger.debug('Processing: {}'.format(file_name))

    cdb_md5 = utils.md5_file(file_path)
    try:
        with open(upstream_md5, 'r') as f:
            json_md5 = f.read()

        # If the cdb file matches the generated md5,
        # no changes are needed to the json
        if json_md5 == cdb_md5:
            return True
    except IOError:
        pass

    tmp_json = tempfile.NamedTemporaryFile(delete=False)
    with open(file_path, 'r') as fp:
        reader = cdblib.Reader(fp.read())

    out = collections.OrderedDict()
    for k, v in reader.iteritems():
        out[k] = v

    json_data = json.dumps(out, indent=0, separators=(',', ':'))

    # Make python json.dumps match php's json_encode
    # Remove first newline
    json_data = json_data.replace('\n', '', 1)

    # Escape slashes
    json_data = json_data.replace('/', '\/')

    # Remove final newline
    json_data = ''.join(json_data.rsplit('\n', 1))

    tmp_json.write(json_data)
    tmp_json.close()
    os.chmod(tmp_json.name, 0644)
    shutil.move(tmp_json.name, upstream_json)
    logger.debug('Updated: {}'.format(upstream_json))

    with open(upstream_md5, 'w') as md5:
        md5.write(cdb_md5)

    return True


@utils.log_context('service_restart')
def restart_service(service, user='mwdeploy', logger=None):
    logger.debug('Restarting service {}'.format(service))
    cmd_format = 'sudo /usr/sbin/service {} {}'
    utils.sudo_check_call(user, cmd_format.format(service, 'restart'))


@utils.log_context('port_check')
def check_port(port, timeout, interval=3, logger=None):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    start = time.time()
    elapsed = 0.0
    while elapsed < timeout:
        if elapsed > 0:
            time.sleep(interval)

        elapsed = time.time() - start

        if sock.connect_ex(('127.0.0.1', port)) == 0:
            logger.debug('Port {} up in {:.2f}s'.format(port, elapsed))
            return True

        logger.debug('Port {} not up. Waiting {:.2f}s'.format(port, interval))

    raise OSError(
        errno.ENOTCONN,
        'Port {} not up within {:.2f}s'.format(port, timeout)
    )


def check_patch_files(version, cfg):
    """Check to see if there are unmerged patch files from /srv/patches
    for a given revision.

    :param version: MediaWiki version string (e.g., '1.27.0-wmf.8')
    :param cfg: Scap configuration dict
    """

    logger = logging.getLogger('check_patch_files')

    # Patches should live in /srv/patches/[version]
    patch_path = cfg['patch_path']
    if patch_path is None:
        return

    version_base = os.path.join(patch_path, version)

    ext_dir = os.path.join(version_base, 'extensions')
    _, extensions, _ = os.walk(ext_dir).next()

    patches = utils.get_patches(['core'], version_base)
    patches.update(utils.get_patches(extensions, ext_dir))

    git_patch_check = ['/usr/bin/git', 'apply', '--check', '--reverse']
    version_dir = 'php-{}'.format(version)
    apply_dir = os.path.join(cfg['stage_dir'], version_dir)

    for extension, diffs in patches.iteritems():
        diff = '\n'.join(diffs)

        if extension != 'core':
            apply_dir = os.path.join(apply_dir, 'extensions', extension)

        with utils.cd(apply_dir):
            p = subprocess.Popen(git_patch_check,
                                 stdin=subprocess.PIPE,
                                 stdout=subprocess.PIPE)

            p.communicate(diff)

            if p.returncode > 0:
                logger.warn(
                    'Patch(s) for {} have not been applied.'.format(apply_dir))
