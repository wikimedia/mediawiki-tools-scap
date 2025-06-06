# -*- coding: utf-8 -*-
"""
    scap.tasks
    ~~~~~~~~~~
    Contains functions implementing scap tasks

    Copyright © 2014-2017 Wikimedia Foundation and Contributors.

    This file is part of Scap.

    Scap is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, version 3.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""
import collections
import errno
import glob
import itertools
import json
import logging
import multiprocessing
import os
import pwd
import socket
import subprocess
import sys
import time

import scap.cdblib as cdblib
import scap.git as git
import scap.log as log
import scap.mwscript as mwscript
import scap.ssh as ssh
import scap.utils as utils


DEFAULT_RSYNC_ARGS = [
    "/usr/bin/rsync",
    "--archive",
    "--delete-delay",
    "--delay-updates",
    "--compress",
    "--new-compress",
    "--delete",
    "--exclude=*.swp",
    "--no-perms",
    "--stats",
]


RESTART = "restart"
RELOAD = "reload"
DISABLE_SECONDARY = "disable-secondary"


def cache_git_info_helper(subdir, branch_dir, cache_dir):
    try:
        new_info = git.info(subdir)
    except IOError:
        return

    cache_file = git.info_filename(subdir, branch_dir, cache_dir)

    old_info = None

    if os.path.exists(cache_file):
        with open(cache_file, "r") as f:
            old_info = json.load(f)

    if new_info == old_info:
        return

    with open(cache_file, "w") as f:
        json.dump(new_info, f)


def cache_git_info(version, cfg):
    """
    Create JSON cache files of git branch information.

    :param version: MediaWiki version (eg '1.38.0-wmf.20')
    :param cfg: Dict of global configuration values
    :raises: :class:`IOError` if version directory is not found
    """
    branch_dir = os.path.join(cfg["stage_dir"], "php-%s" % version)

    if not os.path.isdir(branch_dir):
        raise IOError(errno.ENOENT, "Invalid branch directory", branch_dir)

    # Create cache directory if needed
    cache_dir = os.path.join(branch_dir, "cache", "gitinfo")
    if not os.path.isdir(cache_dir):
        os.mkdir(cache_dir)

    inputs = []

    inputs.append((branch_dir, branch_dir, cache_dir))
    for dirname in ["extensions", "skins"]:
        full_dir = os.path.join(branch_dir, dirname)
        for subdir in utils.iterate_subdirectories(full_dir):
            inputs.append((subdir, branch_dir, cache_dir))

    # Create cache for core and each extension and skin
    with multiprocessing.Pool(utils.cpus_for_jobs()) as p:
        p.starmap(cache_git_info_helper, inputs)


@utils.log_context("compile_wikiversions")
def compile_wikiversions(source_tree, cfg, logger=None):
    """
    Validate and compile the wikiversions.json file.

    1. Find the realm specific filename for wikiversions.json in specified tree
       (deploy or staging)
    2. Validate that all versions mentioned in the json exist as directories
       in the specified tree.
    3. Create a temporary php file from the json contents
    4. Atomically rename the temporary php to the realm specific
       wikiversions.php filename

    :param source_tree: Source tree to read file from: 'deploy' or 'stage'
    :param cfg: Dict of global configuration values
    """

    working_dir = cfg["%s_dir" % source_tree]

    # Find the realm specific wikiversions file names
    base_file = os.path.join(working_dir, "wikiversions.json")
    json_file = utils.get_realm_specific_filename(base_file, cfg["wmf_realm"])
    base_name = os.path.splitext(json_file)[0]
    php_file = base_name + ".php"

    with open(json_file) as f:
        wikiversions = json.load(f)

    # Validate that all versions in the json file exist locally
    for dbname, version in wikiversions.items():
        version_dir = os.path.join(working_dir, version)
        if not os.path.isdir(version_dir):
            raise IOError(errno.ENOENT, "Invalid/unavailable version dir", version_dir)

    # Build the php version
    php_code = "<?php\nreturn array(\n%s\n);\n" % json.dumps(
        wikiversions, separators=(",", " => "), sort_keys=True, indent=4
    ).strip("{}\n")

    if os.path.exists(php_file):
        with open(php_file) as p:
            if p.read() == php_code:
                # The existing file already has the desired contents,
                # so we don't need to do anything else.
                return

    tmp_php_file = "%s.tmp" % php_file
    try:
        os.unlink(tmp_php_file)
    except OSError:
        pass

    with open(tmp_php_file, "wt") as fp:
        fp.write(php_code)
        fp.flush()
        utils.eintr_retry(os.fsync, fp.fileno())

    if not os.path.isfile(tmp_php_file):
        raise IOError(errno.ENOENT, "Failed to create php wikiversions", tmp_php_file)

    os.rename(tmp_php_file, php_file)
    os.chmod(php_file, 0o664)
    logger.info("Compiled %s to %s", json_file, php_file)


# Called by scap cdb-rebuild (main.py)
@utils.log_context("merge_cdb_updates")
def merge_cdb_updates(directory, pool_size, trust_mtime=False, mute=False, logger=None):
    """
    Update l10n CDB files using JSON data.

    :param directory: L10n cache directory
    :param pool_size: Number of parallel processes to use
    :param trust_mtime: Trust file modification time?
    :param mute: Disable progress indicator
    """

    cache_dir = os.path.realpath(directory)
    upstream_dir = os.path.join(cache_dir, "upstream")

    files = [
        os.path.splitext(os.path.basename(f))[0]
        for f in glob.glob("%s/*.json" % upstream_dir)
    ]
    if not files:
        logger.warning("Directory %s is empty", upstream_dir)
        return 0

    pool = multiprocessing.Pool(pool_size)
    updated = 0

    reporter = log.reporter("l10n merge", mute=mute)
    reporter.expect(len(files))
    reporter.start()

    l10n_update_pool = pool.imap_unordered(
        update_l10n_cdb_wrapper,
        zip(itertools.repeat(cache_dir), files, itertools.repeat(trust_mtime)),
    )
    for i, result in enumerate(l10n_update_pool, 1):
        if result:
            updated += 1
        reporter.add_success()

    reporter.finish()
    logger.info("Updated %d CDB files(s) in %s", updated, cache_dir)
    return 0


@utils.log_context("sync_master")
def sync_master(app, master, verbose=False, logger=None):
    """
    Sync local staging dir with upstream rsync server's copy.

    Rsync from ``server::common`` to the local staging directory.

    :param app: cli.Application
    :param master: Master server to sync with
    :param verbose: Enable verbose logging?
    """

    cfg = app.config

    if not os.path.isdir(cfg["stage_dir"]):
        raise IOError(
            (
                "rsync target directory %s not found. Ask root to create it "
                "(should belong to root:wikidev)."
            )
            % cfg["stage_dir"]
        )

    # Execute rsync fetch locally via sudo and wrapper script
    rsync = ["sudo", "-n", "--", "/usr/local/bin/scap-master-sync"]
    rsync.append(master)

    logger.info(
        "Copying from %s to %s:/srv/mediawiki-staging", master, socket.getfqdn()
    )
    logger.debug("Running rsync command: `%s`", " ".join(rsync))
    with app.Timer("rsync master"):
        subprocess.check_call(rsync)

    # Rebuild the CDB files
    with app.Timer("rebuild CDB staging files"):
        utils.sudo_check_call(
            "www-data",
            "%s cdb-rebuild --master" % app.get_script_path(),
            logger=logger,
            app=app,
        )


# Called via "scap pull"
@utils.log_context("sync_common")
def sync_common(
    app,
    include=None,
    sync_from=None,
    verbose=False,
    logger=None,
    rsync_args=None,
    exclude_wikiversionsphp=False,
):
    """
    Sync local deploy dir with upstream rsync server's copy.

    Rsync from ``server::common`` to the local deploy directory.
    If a list of servers is given in ``sync_from`` we will attempt to select
    the "best" one to sync from. If no servers are given or all servers given
    have issues we will fall back to using the server named by
    ``master_rsync`` in the configuration data.

    :param app: cli.Application
    :param include: List of rsync include patterns to limit the sync to. If
        ``None`` is given the entire ``common`` module on the target rsync
        server will be transferred. Rsync syntax for syncing a directory is
        ``<dirname>/***``.
    :param sync_from: List of rsync servers to fetch from.
    """

    cfg = app.config
    deploy_dir = cfg["deploy_dir"]

    if not os.path.isdir(deploy_dir):
        raise SystemExit(
            (
                "rsync target directory %s not found. Ask root to create it "
                "(should belong to mwdeploy:mwdeploy)."
            )
            % deploy_dir
        )

    # T329857
    if os.path.islink(deploy_dir):
        logger.warning(
            f"{deploy_dir} is a symlink to {os.readlink(deploy_dir)}.  Not pulling."
        )
        sys.exit(0)

    server = None
    if sync_from:
        server = utils.find_nearest_host(sync_from, "rsync")
    if server is None:
        server = cfg["master_rsync"]
    server = server.strip()

    # Execute rsync fetch locally via sudo
    rsync = ["sudo", "-u", "mwdeploy", "-n", "--"] + DEFAULT_RSYNC_ARGS
    # Exclude .git metadata
    rsync.append("--exclude=**/.git")
    # Exclude scap metadata
    rsync.append("--exclude=/scap")

    if exclude_wikiversionsphp:
        if rsync_args and "--delete-excluded" in rsync_args:
            # --delete-excluded mode overrides exclude_wikiversionsphp because
            # we can't allow wikiversions.php to be deleted.
            pass
        else:
            # Exclude wikiversions*.php.  This is rsync'd by sync_wikiversions later.
            rsync.append("--exclude=/wikiversions*.php")

    rsync.append("--exclude=**/cache/l10n/*.cdb")

    if verbose:
        rsync.append("--verbose")

    if include:
        for path in include:
            rsync.append("--include=/%s" % path)
        # Exclude everything not explicitly included
        rsync.append("--exclude=*")

    if rsync_args:
        rsync += rsync_args

    rsync.append("%s::common" % server)
    rsync.append(deploy_dir)

    logger.info(
        "Copying from %s:/srv/mediawiki-staging to %s:%s",
        server,
        socket.getfqdn(),
        deploy_dir,
    )
    logger.debug("Running rsync command: `%s`", " ".join(rsync))
    with app.Timer("rsync common"):
        subprocess.check_call(rsync)

    # Bug 58618: Invalidate local configuration cache by updating the
    # timestamp of wmf-config/InitialiseSettings.php
    settings_path = os.path.join(deploy_dir, "wmf-config", "InitialiseSettings.php")
    logger.debug("Touching %s", settings_path)
    subprocess.check_call(
        ("sudo", "-u", "mwdeploy", "-n", "--", "/usr/bin/touch", settings_path)
    )


def sync_wikiversions(hosts, app, stage: str, key=None):
    """
    Rebuild and sync wikiversions.php to the cluster.

    :param hosts: List of hosts to sync to
    :param app: cli.Application
    :param stage: The deployment stage ("testservers", "canaries", or "prod")
    """
    with app.Timer(f"sync-wikiversions-{stage}"):
        cfg = app.config
        compile_wikiversions("stage", cfg)

        rsync = ssh.Job(hosts, user=cfg["ssh_user"], key=key).shuffle()
        rsync.command(
            "sudo -u mwdeploy -n -- /usr/bin/rsync -l "
            "%(master_rsync)s::common/wikiversions*.{json,php} "
            "%(deploy_dir)s" % cfg
        )
        return rsync.progress(log.reporter("sync-wikiversions")).run()


# Called by update_l10n_cdb_wrapper (below)
@utils.log_context("update_l10n_cdb")
def update_l10n_cdb(cache_dir, cdb_file, trust_mtime=False, logger=None):
    """
    Update a localization CDB database.

    :param cache_dir: L10n cache directory
    :param cdb_file: L10n CDB database
    :param trust_mtime: Trust file modification time?
    """

    md5_path = os.path.join(cache_dir, "upstream", "%s.MD5" % cdb_file)
    if not os.path.exists(md5_path):
        logger.warning("skipped %s; no md5 file", cdb_file)
        return False

    json_path = os.path.join(cache_dir, "upstream", "%s.json" % cdb_file)
    if not os.path.exists(json_path):
        logger.warning("skipped %s; no json file", cdb_file)
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
        tmp_cdb_path = "%s.tmp" % cdb_path
        with open(tmp_cdb_path, "wb") as fp:
            writer = cdblib.Writer(fp)
            for key, value in data.items():
                writer.put(key, value)
            writer.finalize()
            utils.eintr_retry(os.fsync, fp.fileno())

        if not os.path.isfile(tmp_cdb_path):
            raise IOError(errno.ENOENT, "Failed to create CDB", tmp_cdb_path)

        # Move temp file over old file
        os.chmod(tmp_cdb_path, 0o664)
        os.rename(tmp_cdb_path, cdb_path)
        # Set timestamp to match upstream json
        os.utime(cdb_path, (json_mtime, json_mtime))
        return True
    return False


# Called by merge_cdb_updates
@utils.log_context("update_l10n_cdb_wrapper")
def update_l10n_cdb_wrapper(args, logger=None):
    """Wrapper for update_l10n_cdb to be used in contexts where only a single
    argument can be provided.

    :param args: Sequence of arguments to pass to update_l10n_cdb
    """
    try:
        return update_l10n_cdb(*args)
    except Exception:
        # Log detailed error; multiprocessing will truncate the stack trace
        logger.exception("Failure processing %s", args)
        raise


def _call_rebuildLocalisationCache(
    app,
    version,
    out_dir,
    use_cores=1,
    php_l10n=False,
    lang=None,
    force=False,
    quiet=False,
    delay_messageblobstore_purge=False,
):
    """
    Helper for update_localization_cache.

    :param app: Scap cli.Application
    :param version: The train version to build l10n for
    :param out_dir: The output directory
    :param use_cores: The number of cores to run in
    :param lang: The --lang option, or None to omit
    :param force: Whether to pass --force
    :param quiet: Whether to pass --quiet
    """

    # Allow the default l10n language to be controlled by
    # the SCAP_MW_LANG environment variable.
    if lang is None:
        # lang will remain None if SCAP_MW_LANG is not defined.
        lang = os.getenv("SCAP_MW_LANG")

    def _rebuild(store_class, file_extension):
        logging.info("Running rebuildLocalisationCache.php")
        # Passing --skip-message-purge for T263872 (if delay_messageblobstore_purge feature
        # flag is enabled).
        # Note: mwscript runs maintenance scripts from /srv/mediawiki-staging if it exists,
        # otherwise it falls back to /srv/mediawiki.  If rebuildLocalisationCache.php is run
        # from /srv/mediawiki-staging (the usual case), it will update files in
        # /srv/mediawiki-staging/php-<vers>/cache/l10n
        args = [
            "--no-progress",
            "--store-class",
            store_class,
            "--threads",
            use_cores,
        ]

        if lang:
            args += ["--lang", lang]
        if force:
            args += ["--force"]
        if quiet:
            args += ["--quiet"]

        network = True
        # If we're delaying the message purge, we don't strictly need the
        # network or database access
        if delay_messageblobstore_purge:
            network = False
            args += [
                "--no-database",
                "--skip-message-purge",
            ]

        with log.pipe() as out:
            mwscript.run(
                app,
                "rebuildLocalisationCache.php",
                *args,
                version=version,
                network=network,
                stdout=out,
            )
        mwscript.run_shell(app, "chmod 0664 {}/*.cdb", out_dir)

    _rebuild("LCStoreCDB", "cdb")

    # PHP l10n generation feature flag
    if php_l10n:
        _rebuild("LCStoreStaticArray", "php")


def ensure_extension_messages_file(cfg, version, logger):
    """
    Ensure that <staging>/wmf-config/ExtensionMessages-<version>.php exists.
    If it does not exist, create a blank file.

    In either case, the path to the ExtensionMessages-<version>.php file is
    returned.
    """

    extension_messages = os.path.join(
        cfg["stage_dir"], "wmf-config", "ExtensionMessages-%s.php" % version
    )

    if not os.path.exists(extension_messages):
        # Touch the extension_messages file to prevent php require errors
        logger.info("Creating empty %s", extension_messages)
        open(extension_messages, "a").close()

    return extension_messages


@utils.log_context("update_localization_cache")
def update_localization_cache(version, app, logger=None, json=True):
    """
    Update the localization cache for a given MW version.

    :param version: MediaWiki version
    :param wikidb: Wiki running given version
    :param app: Application calling the function
    :param json: Whether to generate JSON/MD5 files from CDBs when finished.
    """

    verbose = app.verbose
    cfg = app.config

    # Calculate the number of parallel threads
    # Leave a couple of cores free for other stuff
    use_cores = utils.cpus_for_jobs()

    verbose_messagelist = ""
    force_rebuild = False
    quiet_rebuild = False
    if verbose:
        verbose_messagelist = "--verbose"

    extension_messages = ensure_extension_messages_file(cfg, version, logger)

    cache_dir = os.path.join(cfg["stage_dir"], "php-%s" % version, "cache", "l10n")

    if os.path.exists(cache_dir):
        # Clean up cruft from any prior interrupted run.
        mwscript.run_shell(app, "rm -f {}/*.tmp.*", cache_dir)

    if not os.path.exists(os.path.join(cache_dir, "l10n_cache-en.cdb")):
        # mergeMessageFileList.php needs a l10n file
        logger.info("Bootstrapping l10n cache for %s", version)
        _call_rebuildLocalisationCache(
            app, version, cache_dir, use_cores, cfg["php_l10n"], lang="en", quiet=True
        )
        # Force subsequent cache rebuild to overwrite bootstrap version
        force_rebuild = True

    logger.info("Updating ExtensionMessages-%s.php", version)
    new_extension_messages = mwscript.run_shell(app, "mktemp").stdout.strip()

    # attempt to read extension-list from the branch instead of wmf-config
    ext_list = os.path.join(cfg["stage_dir"], "php-%s" % version, "extension-list")

    if not os.path.isfile(ext_list):
        # fall back to the old location in wmf-config
        ext_list = "%s/wmf-config/extension-list" % cfg["stage_dir"]

    mwscript.run(
        app,
        "mergeMessageFileList.php",
        "--list-file",
        ext_list,
        "--output",
        new_extension_messages,
        version=version,
        check_warnings=True,
    )

    try:
        mwscript.run_shell(app, "chmod 0664 {}", new_extension_messages)
        with open(new_extension_messages) as f:
            utils.write_file_if_needed(extension_messages, f.read())
    finally:
        mwscript.run_shell(app, "rm {}", new_extension_messages)

    # Rebuild all the CDB files for each language
    logger.info(
        "Updating LocalisationCache for %s " "using %s thread(s)" % (version, use_cores)
    )
    _call_rebuildLocalisationCache(
        app,
        version,
        cache_dir,
        use_cores,
        php_l10n=cfg["php_l10n"],
        force=force_rebuild,
        quiet=quiet_rebuild,
        delay_messageblobstore_purge=cfg["delay_messageblobstore_purge"],
    )

    cache_dir_owner = pwd.getpwuid(os.stat(cache_dir).st_uid).pw_name

    if json:
        # Include JSON versions of the CDB files and add MD5 files
        logger.info(
            "Generating JSON versions and md5 files (as {})".format(cache_dir_owner)
        )
        utils.sudo_check_call(
            user=cache_dir_owner,
            cmd="%s cdb-json-refresh "
            '--directory="%s" --threads=%s %s'
            % (app.get_script_path(), cache_dir, use_cores, verbose_messagelist),
            app=app,
        )


def refresh_cdb_json_files(in_dir, pool_size, verbose):
    """
    Update json files from corresponding cdb file in parallel.

    :param in_dir: directory containing cdb files
    :param pool_size: number of "threads" to use
    :param verbose: output verbosely
    """
    logger = utils.get_logger()
    cdb_files = glob.glob(os.path.join(in_dir, "*.cdb"))
    pool = multiprocessing.Pool(pool_size)

    updated = 0
    reporter = log.reporter("cdb update", mute=not verbose)

    reporter.expect(len(cdb_files))
    reporter.start()

    for result in pool.imap_unordered(refresh_cdb_json_file, cdb_files):
        if result:
            updated += 1
        reporter.add_success()

    reporter.finish()
    logger.info("Updated %s JSON file(s) in %s", updated, in_dir)


def refresh_cdb_json_file(cdb_file_path) -> bool:
    """
    Rebuild json file from cdb file.

    #. Check md5 file saved in upstream against md5 of cdb file
    #. Read cdb file to dict
    #. Write dict to named temporary file
    #. Change permissions on named temporary file
    #. Overwrite upstream json file
    #. Write upstream md5 file

    Returns a boolean indicating whether or not a JSON file was
    (re)created.
    """
    cdb_dir = os.path.dirname(cdb_file_path)
    file_name = os.path.basename(cdb_file_path)
    upstream_dir = os.path.join(cdb_dir, "upstream")
    upstream_md5 = os.path.join(upstream_dir, "{}.MD5".format(file_name))
    upstream_json = os.path.join(upstream_dir, "{}.json".format(file_name))

    logger = utils.get_logger()
    logger.debug("Processing: %s", file_name)

    cdb_md5 = utils.md5_file(cdb_file_path)
    try:
        with open(upstream_md5, "r") as f:
            json_md5 = f.read()

        # If the cdb file matches the generated md5,
        # no changes are needed to the json
        if json_md5 == cdb_md5:
            return False
    except IOError:
        pass

    with open(cdb_file_path, "rb") as fp:
        reader = cdblib.Reader(fp.read())

    out = collections.OrderedDict()
    for k, v in reader.items():
        out[k] = v

    json_data = json.dumps(out, indent=0, separators=(",", ":"))

    # Make python json.dumps match php's json_encode
    # Remove first newline
    json_data = json_data.replace("\n", "", 1)

    # Escape slashes
    json_data = json_data.replace("/", r"\/")

    # Remove final newline
    json_data = "".join(json_data.rsplit("\n", 1))

    with utils.temp_to_permanent_file(upstream_json, mode="wb") as f:
        f.write(json_data.encode())
    logger.debug("Updated: %s", upstream_json)

    with utils.temp_to_permanent_file(upstream_md5) as f:
        f.write(cdb_md5)

    return True


def handle_services(services, require_valid_service=False, on_secondary_host=False):
    """
    Take a comma-separated list of services and restart, reload or disable each of them.

    The idea is to take a string directly from the scap.cfg file that looks
    like:

        jobrunner, jobchron=reload, jenkins=restart:disable-secondary, ...

    and be able to determine what to do with that list.

    If no action is specified the default is "restart".
    When specified, main action can be one of:

    * restart: service will be restarted
    * reload: service will be reloaded

    When specified, secondary action can be one of:

    * disable-secondary: service is disabled on targets marked as secondary

    Note that specifying a secondary action requires a main action
    """

    for service_handle in services.split(","):
        service, action, secondary_action = (service_handle.strip(), RESTART, None)
        if "=" in service:
            service, action = service.split("=")
            if ":" in action:
                action, secondary_action = action.split(":")

        # Can be used to check if service is masked, require_valid_service
        # is False by default to preserve existing behavior
        if require_valid_service and not utils.service_exists(service):
            continue

        if on_secondary_host and secondary_action == DISABLE_SECONDARY:
            disable_service(service)
            continue

        if action == RELOAD:
            reload_service(service)
            continue

        if action == RESTART:
            restart_service(service)
            continue

        reported_action = (
            f"{action}:{secondary_action}" if secondary_action is not None else action
        )
        raise RuntimeError(
            f"""Unknown action "{reported_action}" for service "{service}" """
        )


@utils.log_context("service_restart")
def restart_service(service, logger=None):
    logger.info("Restarting service '{}'".format(service))
    cmd = "sudo -n /usr/sbin/service {} restart".format(service).split()
    subprocess.check_call(cmd)


@utils.log_context("service_reload")
def reload_service(service, logger=None):
    logger.info("Reloading service '{}'".format(service))
    cmd = "sudo -n /usr/sbin/service {} reload".format(service).split()
    subprocess.check_call(cmd)


@utils.log_context("service_disable")
def disable_service(service, logger=None):
    logger.info(f"This is a secondary host. Stopping and disabling service '{service}'")
    # For historical reasons, we have sudo access to stop a service via `/usr/sbin/service`, see:
    # https://gerrit.wikimedia.org/r/plugins/gitiles/operations/puppet/+/4e52262ab5671e694c14600101de0654396e9e14/modules/scap/manifests/target.pp#164
    # But a systemd service can only be disabled with systemctl, so we combine both commands
    subprocess.check_call(f"sudo -n /usr/sbin/service {service} stop".split())
    subprocess.check_call(f"sudo -n /bin/systemctl disable {service}".split())


@utils.log_context("clear_message_blobs")
def clear_message_blobs(app, logger=None):
    """
    Clear MessageBlobStore cache on all wikis

    :param app: Scap cli.Application
    :param logger: logger instance
    """
    logger.info("Running purgeMessageBlobStore.php")

    # Note this script affects all wikis
    mwscript.run(
        app,
        "purgeMessageBlobStore.php",
        wiki=None,
        network=True,
        stdout=sys.stdout,
    )


@utils.log_context("port_check")
def check_port(port, timeout, interval=3, logger=None):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    start = time.time()
    elapsed = 0.0
    while elapsed < timeout:
        if elapsed > 0:
            time.sleep(interval)

        elapsed = time.time() - start

        if sock.connect_ex(("127.0.0.1", port)) == 0:
            logger.debug("Port {} up in {:.2f}s".format(port, elapsed))
            return True

        logger.debug("Port {} not up. Waiting {:.2f}s".format(port, interval))

    raise OSError(errno.ENOTCONN, "Port {} not up within {:.2f}s".format(port, timeout))


def check_patch_files(version, cfg):
    """Check to see if there are unmerged patch files from /srv/patches
    for a given revision.

    :param version: MediaWiki version string (e.g., '1.27.0-wmf.8')
    :param cfg: Scap configuration dict
    """

    logger = logging.getLogger("check_patch_files")

    # Patches should live in /srv/patches/[version]
    patch_path = cfg["patch_path"]
    if patch_path is None:
        return

    version_base = os.path.join(patch_path, version)

    ext_dir = os.path.join(version_base, "extensions")
    _, extensions, _ = next(os.walk(ext_dir))

    patches = utils.get_patches(["core"], version_base)
    patches.update(utils.get_patches(extensions, ext_dir))

    git_patch_check = ["/usr/bin/git", "apply", "--check", "--reverse"]
    version_dir = "php-{}".format(version)
    apply_dir = os.path.join(cfg["stage_dir"], version_dir)

    for extension, diffs in patches.items():
        diff = "\n".join(diffs)

        if extension != "core":
            apply_dir = os.path.join(apply_dir, "extensions", extension)

        with utils.cd(apply_dir):
            p = subprocess.Popen(
                git_patch_check, stdin=subprocess.PIPE, stdout=subprocess.PIPE
            )

            p.communicate(diff)

            if p.returncode > 0:
                logger.warning("Patch(s) for %s have not been applied.", apply_dir)
