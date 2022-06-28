# -*- coding: utf-8 -*-
"""
    scap.main
    ~~~~~~~~~~
    Command wrappers for scap tasks

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
from __future__ import absolute_import
from __future__ import print_function

import argparse
import base64
from concurrent.futures import ProcessPoolExecutor
import errno
import locale
import os
import pathlib
import pwd
import select
import shlex
import socket
import subprocess
import sys
import time
import yaml

from scap import ansi, history
import scap.arg as arg
import scap.cli as cli
import scap.lint as lint
import scap.lock as lock
import scap.log as log
import scap.php_fpm as php_fpm
import scap.ssh as ssh
import scap.targets as targets
import scap.tasks as tasks
import scap.utils as utils
import scap.version as scapversion
from scap.runcmd import mwscript


class AbstractSync(cli.Application):
    """Base class for applications that want to sync one or more files from
    the deployment server to the rest of the cluster."""

    soft_errors = False

    def __init__(self, exe_name):
        super().__init__(exe_name)
        self.include = None
        self.om = None
        self.logo = True
        # A list of hostnames that have been processed by self._perform_sync
        self.already_synced = []

    @cli.argument(
        "--force",
        action="store_true",
        help="Skip canary checks, " "performs ungraceful php-fpm restarts",
    )
    @cli.argument(
        "--stop-before-sync",
        action="store_true",
        help="Perform all operations up to but not including rsyncing to any host",
    )
    @cli.argument(
        "--pause-after-testserver-sync",
        action="store_true",
        help="Pause after syncing testservers and prompt the user to confirm to continue syncing",
    )
    @cli.argument("message", nargs="*", help="Log message for SAL")
    def main(self, *extra_args):
        """Perform a sync operation to the cluster."""
        if self.logo:
            print(ansi.logo())

        self._assert_auth_sock()

        with lock.Lock(self.get_lock_file(), self.arguments.message):
            self._check_sync_flag()
            self._compile_wikiversions()
            self._before_cluster_sync()
            self._update_caches()
            if not self.arguments.force:
                self.get_logger().info("Checking for new runtime errors locally")
                self._check_fatals()
            else:
                self.get_logger().warning("check_fatals Skipped by --force")
            self._build_container_images()
            self._deploy_container_images()

            if self.arguments.stop_before_sync:
                self.get_logger().info("Stopping before sync operations")
                return 0

            self._sync_masters()

            full_target_list = self._get_target_list()

            if not self.arguments.force:
                testservers = utils.list_intersection(self._get_testserver_list(), full_target_list)
                if len(testservers) > 0:
                    self.get_logger().info("Syncing to testservers")
                    self.sync_target(testservers, "testservers")

                    # Not all subclasses of AbstractSync define the --pause-after-testserver-sync argument,
                    # so we can't assume it is in self.arguments.
                    if getattr(self.arguments, "pause_after_testserver_sync", False):
                        utils.prompt_for_approval_or_exit('Changes synced to: %s.\nPlease do any necessary checks '
                                                          'before continuing.\n' % ', '.join(testservers) +
                                                          'Continue with sync? (y/N): ', "Sync cancelled.")

                canaries = utils.list_intersection(self._get_canary_list(), full_target_list)
                if len(canaries) > 0:
                    with log.Timer("sync-check-canaries", self.get_stats()) as timer:
                        self.sync_target(canaries, "canaries")
                        timer.mark("Canaries Synced")
                        # We need a list of lists here.
                        self._restart_php_hostgroups([canaries])
                        self.canary_checks(canaries, timer)
            else:
                self.get_logger().warning("Canaries Skipped by --force")

            # Update proxies
            proxies = utils.list_intersection(self._get_proxy_list(), full_target_list)

            if len(proxies) > 0:
                with log.Timer("sync-proxies", self.get_stats()):
                    sync_cmd = self._apache_sync_command()
                    sync_cmd.append(socket.getfqdn())
                    self._perform_sync("proxies", sync_cmd, proxies)

            # Update apaches
            with log.Timer("sync-apaches", self.get_stats()):
                self._perform_sync("apaches",
                                   self._apache_sync_command(proxies),
                                   full_target_list,
                                   shuffle=True)

            history.update_latest(self.config["history_log"], synced=True)

            self._after_cluster_sync()

        self._after_lock_release()
        if self.soft_errors:
            return 1
        return 0

    def increment_stat(self, stat, all_stat=True, value=1):
        """Increment a stat in deploy.*

        :param stat: String name of stat to increment
        :param all_stat: Whether to increment deploy.all as well
        :param value: How many to increment by, default of 1 is normal
        """
        self.get_stats().increment("deploy.%s" % stat, value)
        if all_stat:
            self.get_stats().increment("deploy.all", value)

    def get_keyholder_key(self):
        """
        Returns scap2-specific deploy key

        This way we can set a key in the default scap config without
        having all non-scap2 repos inherit that configuration.
        """
        key = self.config.get("mediawiki_keyholder_key", None)
        if key:
            return key

        return super().get_keyholder_key()

    def _before_cluster_sync(self):
        pass

    def _update_caches(self):
        self._git_repo()

        # Compute git version information
        with log.Timer("cache_git_info", self.get_stats()):
            for version in self.active_wikiversions("stage"):
                tasks.cache_git_info(version, self.config)

    def _check_fatals(self):
        logger = self.get_logger()

        for version, wikidb in self.active_wikiversions("stage", return_type=dict).items():
            logger.debug("Testing {} with eval.php using {}".format(version, wikidb))
            with utils.suppress_backtrace():
                stderr = mwscript("eval.php", "--wiki", wikidb)
                if stderr:
                    raise SystemExit("'mwscript eval.php --wiki {}' generated unexpected output: {}".format(wikidb, stderr))

    def _check_sync_flag(self):
        sync_flag = os.path.join(self.config["stage_dir"], "sync.flag")
        if os.path.exists(sync_flag):
            stat = os.stat(sync_flag)
            owner = pwd.getpwuid(stat.st_uid).pw_name
            utils.get_logger().error("%s's sync.flag is blocking deployments", owner)
            raise IOError(errno.EPERM, "Blocked by sync.flag", sync_flag)

    def _get_proxy_list(self):
        """Get list of sync proxy hostnames that should be updated before the
        rest of the cluster."""
        return targets.get("dsh_proxies", self.config).all

    def _get_target_list(self):
        """Get list of hostnames that should be updated from the proxies."""
        return list(
            set(self._get_proxy_list())
            | set(targets.get("dsh_targets", self.config).all)
        )

    def _get_testserver_list(self):
        """ Get list of Mediawiki testservers."""
        return targets.get("dsh_testservers", self.config).all

    def _get_api_canary_list(self):
        """Get list of MediaWiki api canaries."""
        return targets.get("dsh_api_canaries", self.config).all

    def _get_app_canary_list(self):
        """Get list of MediaWiki api canaries."""
        return targets.get("dsh_app_canaries", self.config).all

    def _get_canary_list(self):
        """Get list of MediaWiki canary hostnames."""
        return list(set(self._get_api_canary_list()) | set(self._get_app_canary_list()))

    def _sync_masters(self):
        """Sync the staging directory across all other deploy master servers."""

        us = socket.getfqdn()
        other_masters = [master for master in self.get_master_list() if master != us]

        if len(other_masters) > 0:
            self.master_only_cmd("sync-masters", self._master_sync_command())
            self.master_only_cmd("sync-pull-masters", self._base_scap_pull_command())

    def master_only_cmd(self, timer, cmd):
        """
        Run a command on all other master servers than the one we're on

        :param timer: String name to use in timer/logging
        :param cmd: List of command/parameters to be executed
        """

        masters = self.get_master_list()
        with log.Timer(timer, self.get_stats()):
            update_masters = ssh.Job(
                masters, user=self.config["ssh_user"], key=self.get_keyholder_key()
            )
            update_masters.exclude_hosts([socket.getfqdn()])
            update_masters.command(cmd)
            update_masters.progress(log.reporter(timer, self.config["fancy_progress"]))
            succeeded, failed = update_masters.run()
            if failed:
                self.get_logger().warning("%d masters had sync errors", failed)
                self.soft_errors = True

    def _master_sync_command(self):
        """Synchronization command to run on the master hosts."""
        cmd = [self.get_script_path(), "pull-master"]
        if self.verbose:
            cmd.append("--verbose")
        cmd.append(socket.getfqdn())
        lang = os.getenv("SCAP_MW_LANG")
        if lang:
            cmd = ["env", "SCAP_MW_LANG={}".format(lang)] + cmd
        return cmd

    def _base_scap_pull_command(self) -> list:
        """
        Returns (as a list) the basic scap pull command to run on a remote
        target.  Note that no source servers are specified in the command
        so scap pull will default to pull from whatever `master_rsync` is
        defined to be in the scap configuration on the target.

        Subclasses may override this method.
        """
        cmd = [self.get_script_path(), "pull", "--no-php-restart", "--no-update-l10n"]
        if self.verbose:
            cmd.append("--verbose")
        return cmd

    def _apache_sync_command(self, proxies: list = ()) -> list:
        """
        Returns (as a list) the scap pull command to run on mediawiki
        installation targets.  This is comprised of the base scap pull command
        (defined by _base_scap_pull_command) followed by the list of deployment
        masters and the list of proxies.

        :param proxies: A list of proxy hostnames that can be pulled from in addition
                        to the deployment masters. Default is empty (as a tuple to prevent mutable
                        list warning)
        """
        return self._base_scap_pull_command() + utils.list_union(self.get_master_list(), proxies)

    def _perform_sync(self, type: str, command: list, targets: list, shuffle=False):
        """
        :param type: A string like "apaches" or "proxies" naming the type of target.
        :param command: The command to run on the targets, specified as a list.
        :param targets: A list of strings naming hosts to sync.
        :param shuffle: If true, the target host list will be randomized.
        """
        job = ssh.Job(
            targets,
            command=command,
            user=self.config["ssh_user"],
            key=self.get_keyholder_key()
        )

        job.exclude_hosts(self.already_synced)

        if shuffle:
            job.shuffle()

        job.progress(
            log.reporter("sync-{}".format(type), self.config["fancy_progress"])
        )

        jobresults = job.run(return_jobresults=True)
        self.get_logger().info("Per-host sync duration: average %ss, median %ss",
                               locale.format("%.1f", jobresults.average_duration(), grouping=True),
                               locale.format("%.1f", jobresults.median_duration(), grouping=True))

        num_hosts = 0
        total_transferred = 0

        for jobresult in jobresults:
            num_hosts += 1
            total_transferred += utils.parse_rsync_stats(jobresult.output).get("total_transferred_file_size", 0)

        average_transferred = total_transferred // num_hosts if num_hosts else 0

        self.get_logger().info("rsync transfer: average {:n} bytes/host, total {:n} bytes".format(average_transferred, total_transferred))

        failed = jobresults.num_failed
        if failed:
            self.get_logger().warning("%d %s had sync errors", failed, type)
            self.soft_errors = True

        self.already_synced += targets

    def _compile_wikiversions(self):
        """Compile wikiversions.json to wikiversions.php in stage_dir"""
        tasks.compile_wikiversions("stage", self.config)

    def _git_repo(self):
        """Flatten deploy directory into shared git repo."""
        if self.config["scap3_mediawiki"]:
            self.get_logger().info("Setting up deploy git directory")
            cmd = '{} deploy-mediawiki -v "{}"'.format(
                self.get_script_path(), self.arguments.message
            )
            utils.sudo_check_call("mwdeploy", cmd)

    def _after_cluster_sync(self):
        pass

    def _after_lock_release(self):
        pass

    def sync_target(self, targets=None, type=None):
        """
        Sync targets

        :param targets: Iterable of target servers to sync

        :param type: A string like "apaches" or "proxies" naming the type of target.
        """
        sync_cmd = self._apache_sync_command()

        # Go ahead and attempt to restart php for canaries
        if "--no-php-restart" in sync_cmd:
            sync_cmd.remove("--no-php-restart")

        sync_cmd.append(socket.getfqdn())

        self._perform_sync(type, sync_cmd, targets)

    def canary_checks(self, canaries=None, timer=None):
        """
        Run canary checks

        :param canaries: Iterable of canary servers to check
        :param timer: log.Timer
        :raises RuntimeError: on canary check failure
        """
        if not canaries:
            return

        # If more than 1/4 of the canaries failed, stop deployment
        max_failed_canaries = max(len(canaries) / 4, 1)

        swagger_url = self.config["mediawiki_canary_swagger_url"]
        spec_path = self.config["mediawiki_canary_swagger_spec_path"]

        succeeded, failed = tasks.endpoint_canary_checks(
            canaries, swagger_url, spec_path, cores=utils.cpus_for_jobs()
        )

        if failed > max_failed_canaries:
            canary_fail_msg = (
                "Scap failed!: {}/{} canaries failed their endpoint checks" "({}).  WARNING: canaries have not been rolled back."
            ).format(failed, len(canaries), swagger_url)
            self.announce(canary_fail_msg)
            raise RuntimeError(canary_fail_msg)

        time_since_sync = 0

        if timer:
            time_since_sync = timer.mark("Canary Endpoint Check Complete")

        # Needs some time for log errors to happen
        canary_wait_time = self.config["canary_wait_time"]
        remaining_wait_time = canary_wait_time - time_since_sync

        # If the canary endpoint check took less than the wait time we
        # should wait longer
        if remaining_wait_time > 0:
            self.get_logger().info("Waiting for canary traffic...")
            time.sleep(remaining_wait_time)
        # Otherwise Canary endpoint check took more than the wait time
        # we should adjust the logstash canary delay
        else:
            canary_wait_time = time_since_sync

        logstash_canary_checks = {
            "service": self.config["canary_service"],
            "threshold": self.config["canary_threshold"],
            "logstash": self.config["logstash_host"],
            "delay": canary_wait_time,
            "cores": utils.cpus_for_jobs(),
        }

        succeeded, failed = tasks.logstash_canary_checks(
            canaries, **logstash_canary_checks
        )

        if failed > max_failed_canaries:
            canary_fail_msg = (
                "scap failed: average error rate on {}/{} "
                "canaries increased by 10x "
                "(rerun with --force to override this check, "
                "see {} for details)".format(
                    failed, len(canaries), self.config["canary_dashboard_url"]
                )
            )

            self.announce(canary_fail_msg)
            raise RuntimeError(canary_fail_msg)

        # If some canaries failed, explain why we didn't raise a
        # RuntimeError - T173146
        if failed > 0:
            self.get_logger().info(
                "Canary error check failed for {} canaries, less than "
                "threshold to halt deployment ({}/{}), see {} for "
                "details. Continuing...".format(
                    failed,
                    max_failed_canaries + 1,  # + 1 since we use > to compare
                    len(canaries),
                    self.config["canary_dashboard_url"],
                )
            )

    def _setup_php(self):
        """
        Sets up the php_fpm instance if not already initialized.

        Returns True if php_fpm restart has been configured, or False if not.
        """
        if php_fpm.INSTANCE is None:
            php_fpm.INSTANCE = php_fpm.PHPRestart(
                self.config,
                ssh.Job(key=self.get_keyholder_key(), user=self.config["ssh_user"]),
                self.arguments.force,
                self.get_logger(),
            )
        return php_fpm.INSTANCE.cmd is not None

    def _restart_php(self):
        """
        If php_fpm_restart_script is set in the configuration then
        on all dsh groups referenced by the mw_web_clusters config parameter
        (but excluding canaries) do the following:

        Check if php-fpm opcache is full, if so restart php-fpm.  If
        the php_fpm_always_restart config parameter is true, the
        opcache is treated as always full, so php-fpm will always
        restart.

        If the operator invoked scap with the --force flag, restart
        php-fpm unsafely (i.e., without depooling and repooling
        around the service restart).  T243009

        """
        if not self._setup_php():
            return

        # mw_web_clusters is expected to be a comma-separated string naming dsh
        # groups.
        # target_groups will be a list of objects representing representing
        # each group.
        target_groups = targets.DirectDshTargetList("mw_web_clusters", self.config)
        # Convert the list of group objects into a
        # list of lists of targets.
        # We remove the canaries as they should've been already restarted.
        canaries = self._get_canary_list()
        group_hosts = []
        for group in target_groups.groups.values():
            target_hosts = set(group.targets) - set(canaries)
            group_hosts.append(list(target_hosts))
        self._restart_php_hostgroups(group_hosts)

    def _restart_php_hostgroups(self, target_hosts=None):
        """Perform php restart for sets of hosts (if configured).

        Parameter target_hosts is a list of lists of hostnames.
        """
        if not self._setup_php():
            return
        num_hosts = 0
        for grp in target_hosts:
            num_hosts += len(grp)
        with log.Timer("php-fpm-restarts", self.get_stats()):
            self.get_logger().info(
                "Running '{}' on {} host(s)".format(
                    php_fpm.INSTANCE.cmd, num_hosts
                )
            )
            with ProcessPoolExecutor(max_workers=5) as pool:
                results = pool.map(php_fpm.restart_helper, target_hosts)
            for _, failed in results:
                if failed:
                    self.get_logger().warning(
                        "%d hosts had failures restarting php-fpm", failed
                    )

    def _build_container_images(self):
        if not self.config["build_mw_container_image"]:
            return

        logger = self.get_logger()

        release_repo_dir = self.config["release_repo_dir"]

        if release_repo_dir is None:
            raise SystemExit("release_repo_dir must be configured when build_mw_container_image is True")

        release_repo_update_cmd = self.config["release_repo_update_cmd"]

        if release_repo_update_cmd:
            logger.info("Running {}".format(release_repo_update_cmd))
            with utils.suppress_backtrace():
                subprocess.run(release_repo_update_cmd, shell=True, check=True)

        git_base = self.config["gerrit_url"]

        if self.config["operations_mediawiki_config_branch"] == "train-dev":
            # Filthy hacks because a docker container can't resolve the name "gerrit.traindev" even though
            # it can access the IP address and port.
            import urllib.parse
            import socket
            p = urllib.parse.urlparse(self.config["gerrit_url"])
            # hack: Replace the hostname of the gerrit server with its IP address.
            p = p._replace(netloc="{}:{}".format(socket.gethostbyname(p.hostname), p.port))
            git_base = urllib.parse.urlunparse(p)

        with log.Timer("build-and-push-container-images", self.get_stats()):
            make_container_image_dir = os.path.join(release_repo_dir, "make-container-image")
            registry = self.config["docker_registry"]

            dev_ca_crt = ""

            if self.config["mediawiki_image_extra_ca_cert"]:
                with open(self.config["mediawiki_image_extra_ca_cert"], "rb") as f:
                    dev_ca_crt = base64.b64encode(f.read()).decode("utf-8")

            make_parameters = {
                "GIT_BASE": git_base,
                "MW_CONFIG_BRANCH": self.config["operations_mediawiki_config_branch"],
                "workdir_volume": self.config["stage_dir"],
                "mv_image_name": "{}/{}".format(registry, self.config["mediawiki_image_name"]),
                "webserver_image_name": "{}/{}".format(registry, self.config["webserver_image_name"]),
                "MV_BASE_PACKAGES": self.config["mediawiki_image_extra_packages"],
                "MV_EXTRA_CA_CERT": dev_ca_crt,
            }

            with utils.suppress_backtrace():
                cmd = "{} {}".format(
                    self.config["release_repo_build_and_push_images_cmd"],
                    " ".join([shlex.quote("=".join(pair)) for pair in make_parameters.items()]))

                build_logfile = os.path.join(pathlib.Path.home(), "scap-image-build-and-push-log")

                logger.info("Container build/push output redirected to {}".format(build_logfile))

                try:
                    with open(build_logfile, "a") as logstream:
                        log_file_position = logstream.tell()
                        logger.debug("Running {} in {}".format(cmd, make_container_image_dir))
                        subprocess.run(cmd, shell=True, check=True, cwd=make_container_image_dir,
                                       stdout=logstream,
                                       stderr=subprocess.STDOUT)
                except subprocess.CalledProcessError as e:
                    # Print the error message, which contains the command that was executed and its
                    # exit status.
                    logger.error(e)
                    logger.error("Stdout/stderr follows:")
                    with open(build_logfile) as logstream:
                        logstream.seek(log_file_position)
                        logger.error(logstream.read())
                    raise

    # Proof of concept.  Works in train-dev
    def _deploy_container_images(self):
        if not self.config["deploy_mw_container_image"]:
            return

        release_repo_dir = self.config["release_repo_dir"]

        if release_repo_dir is None:
            raise SystemExit("release_repo_dir must be configured when deploy_mw_container_image is True")

        make_container_image_dir = os.path.join(release_repo_dir, "make-container-image")

        mv_build_fqin = utils.read_first_line_from_file(os.path.join(make_container_image_dir, "last-build"))
        httpd_image_fqin = utils.read_first_line_from_file(os.path.join(make_container_image_dir, "webserver", "last-build"))

        self._update_helm_values(mv_build_fqin, httpd_image_fqin)
        self._deploy_images()

    def _update_helm_values(self, mv_build_fqin, httpd_image_fqin):
        registry = self.config["docker_registry"]

        def strip_registry(fqin):
            registry_prefix = registry + "/"
            if fqin.startswith(registry_prefix):
                return fqin[len(registry_prefix):]

            return fqin

        values = {
            "docker": {
                "registry": registry,
            },
            "main_app": {
                "image": strip_registry(mv_build_fqin),
            },
            "mw": {
                "httpd": {
                    "image_tag": strip_registry(httpd_image_fqin),
                }
            },
        }

        # FIXME: How will this work in production? Verify necessary write access
        path = "/etc/helmfile-defaults/mediawiki/releases.yaml"
        utils.write_file_if_needed(path, yaml.dump(values))

    def _deploy_images(self):
        with log.Timer("helmfile -e traindev apply", self.get_stats()):
            with utils.suppress_backtrace():
                # FIXME: This is very spammy
                subprocess.run("helmfile -e traindev apply",
                               shell=True,
                               check=True,
                               # FIXME: configurate
                               cwd="/srv/deployment-charts/helmfile.d/services/mwdebug")


@cli.command("security-check")
class SecurityPatchCheck(cli.Application):
    """
    Check if security patches are applied.

    class to check if patches in ``/srv/patches`` have been applied to the
    active wikiversions
    """

    def main(self, *extra_args):
        for version in self.active_wikiversions():
            tasks.check_patch_files(version, self.config)

        return 0


@cli.command("wikiversions-compile", help=argparse.SUPPRESS)
class CompileWikiversions(cli.Application):
    """Compile wikiversions.json to wikiversions.php."""

    @cli.argument(
        "--staging",
        action="store_true",
        help="Compile wikiversions in staging directory",
    )
    def main(self, *extra_args):
        if self.arguments.staging:
            source_tree = "stage"
        else:
            source_tree = "deploy"
            self._run_as("mwdeploy")
            self._assert_current_user("mwdeploy")

        tasks.compile_wikiversions(source_tree, self.config)
        return 0


@cli.command("wikiversions-inuse")
class MWVersionsInUse(cli.Application):
    """Get a list of the active MediaWiki versions."""

    @cli.argument(
        "--withdb",
        action="store_true",
        help="Add `=wikidb` with some wiki using the version.",
    )
    @cli.argument(
        "--staging",
        action="store_true",
        help="Compile wikiversions in staging directory",
    )
    def main(self, *extra_args):
        if self.arguments.staging:
            source_tree = "stage"
        else:
            source_tree = "deploy"
        versions = self.active_wikiversions(source_tree, return_type=dict)

        if self.arguments.withdb:
            output = [
                "%s=%s" % (version, wikidb) for version, wikidb in versions.items()
            ]
        else:
            output = [str(version) for version in versions.keys()]

        print(" ".join(output))
        return 0


@cli.command("cdb-rebuild", help=argparse.SUPPRESS)
class RebuildCdbs(cli.Application):
    """Rebuild localization cache CDB files from the JSON versions."""

    @cli.argument(
        "--version", type=arg.is_version, help="MediaWiki version (eg 1.27.0-wmf.7)"
    )
    @cli.argument(
        "--no-progress",
        action="store_true",
        dest="mute",
        help="Do not show progress indicator.",
    )
    @cli.argument(
        "--staging", action="store_true", help="Rebuild cdb files in staging directory"
    )
    def main(self, *extra_args):
        user = "mwdeploy"
        source_tree = "deploy"
        root_dir = self.config["deploy_dir"]

        if self.arguments.staging:
            user = "l10nupdate"
            source_tree = "stage"
            root_dir = self.config["stage_dir"]

        self._run_as(user)
        self._assert_current_user(user)

        # Leave some of the cores free for apache processes
        use_cores = utils.cpus_for_jobs()

        versions = self.active_wikiversions(source_tree)

        if self.arguments.version:
            version = self.arguments.version
            if version.startswith("php-"):
                version = version[4:]

            # Assert version is active
            if version not in versions:
                raise IOError(errno.ENOENT, "Version not active", version)

            # Replace list of active versions with the single version selected
            versions = [version]

        # Rebuild the CDB files from the JSON versions
        for version in versions:
            cache_dir = os.path.join(root_dir, "php-%s" % version, "cache", "l10n")
            tasks.merge_cdb_updates(cache_dir, use_cores, True, self.arguments.mute)


@cli.command("sync-world", help="Deploy MediaWiki to the cluster")
class ScapWorld(AbstractSync):
    """
    Deploy MediaWiki to the cluster.

    #. Validate php syntax of wmf-config and multiversion
    #. Compile wikiversions.json to php in staging directory
    #. Update l10n files in staging area
    #. Compute git version information
    #. Ask scap masters to sync with current master
    #. Ask scap proxies to sync with master server
    #. Ask apaches to sync with fastest rsync server (excluding wikiversions.php)
    #. Ask apaches to rebuild l10n CDB files
    #. Ask apaches to sync wikiversions.php
    #. Run refreshMessageBlobs.php
    #. Rolling invalidation of all opcache for php 7.x
    """

    @cli.argument(
        "--force",
        action="store_true",
        help="Skip canary checks, " "performs ungraceful php-fpm restarts",
    )
    @cli.argument(
        "-w",
        "--canary-wait-time",
        dest="canary_wait_time",
        type=int,
        help="Define how long new code will run on the "
        "canary servers (default is 20s)",
        metavar="<time in secs>",
    )
    @cli.argument(
        "--skip-l10n-update",
        action="store_true",
        dest="skip_l10n_update",
        help="Skip update of l10n files",
    )
    @cli.argument(
        "--stop-before-sync",
        action="store_true",
        help="Perform all operations up to but not including rsyncing to any host",
    )
    @cli.argument(
        "--no-logo",
        action="store_false",
        help="Do not print the Scap logo",
        dest="logo",
    )
    @cli.argument(
        "--pause-after-testserver-sync",
        action="store_true",
        help="Pause after syncing testservers and prompt the user to confirm to continue syncing",
    )
    @cli.argument("message", nargs="*", help="Log message for SAL")
    def main(self, *extra_args):
        try:
            if any("canary_wait_time" in s for s in self.arguments.defines):
                raise ValueError(
                    "Canary wait time must be defined with " "-w or --canary-wait-time"
                )
        except TypeError:
            pass

        wait = self.arguments.canary_wait_time
        if wait is not None:
            self.config["canary_wait_time"] = wait

        self.logo = self.arguments.logo

        return super().main(*extra_args)

    def _before_cluster_sync(self):
        self.announce("Started scap: %s", self.arguments.message)

        # Validate php syntax of wmf-config and multiversion
        lint.check_valid_syntax(
            [
                "%(stage_dir)s/wmf-config" % self.config,
                "%(stage_dir)s/multiversion" % self.config,
            ],
            utils.cpus_for_jobs(),
        )

    def _update_caches(self):
        super()._update_caches()

        # Update list of extension message files and regenerate the
        # localisation cache.

        if self.arguments.skip_l10n_update:
            self.get_logger().warn("Skipping l10n-update")
        else:
            with log.Timer("l10n-update", self.get_stats()):
                for version, wikidb in self.active_wikiversions("stage", return_type=dict).items():
                    tasks.update_localization_cache(version, wikidb, self)

    def _base_scap_pull_command(self):
        cmd = super()._base_scap_pull_command()
        # We want to exclude wikiversions.php during the normal rsync.
        # wikiversions.php is handled separately in _after_cluster_sync (below).
        cmd.append("--exclude-wikiversions.php")
        return cmd

    def _after_cluster_sync(self):
        target_hosts = self._get_target_list()

        # Ask apaches to rebuild l10n CDB files
        with log.Timer("scap-cdb-rebuild", self.get_stats()):
            rebuild_cdbs = ssh.Job(
                target_hosts, user=self.config["ssh_user"], key=self.get_keyholder_key()
            )
            rebuild_cdbs.shuffle()
            rebuild_cdbs.command(
                "sudo -u mwdeploy -n -- %s cdb-rebuild" % self.get_script_path()
            )
            rebuild_cdbs.progress(
                log.reporter("scap-cdb-rebuild", self.config["fancy_progress"])
            )
            succeeded, failed = rebuild_cdbs.run()
            if failed:
                self.get_logger().warning(
                    "%d hosts had scap-cdb-rebuild errors", failed
                )
                self.soft_errors = True

        # Update and sync wikiversions.php
        succeeded, failed = tasks.sync_wikiversions(
            target_hosts, self.config, key=self.get_keyholder_key()
        )
        if failed:
            self.get_logger().warning("%d hosts had sync_wikiversions errors", failed)
            self.soft_errors = True

        tasks.clear_message_blobs(self.config)
        self._restart_php()

    def _after_lock_release(self):
        self.announce(
            "Finished scap: %s (duration: %s)",
            self.arguments.message,
            utils.human_duration(self.get_duration()),
        )
        self.increment_stat("scap")

    def _handle_exception(self, ex):
        # Logic copied from cli.py:_handle_exception.  FIXME: There has to be a better
        # way to do this.
        backtrace = True
        if isinstance(ex, lock.LockFailedError) or getattr(
                ex, "_scap_no_backtrace", False
        ):
            backtrace = False

        if backtrace:
            self.get_logger().warning("Unhandled error:", exc_info=True)

        self.announce(
            "scap failed: %s %s (duration: %s)",
            type(ex).__name__,
            ex,
            utils.human_duration(self.get_duration()),
        )
        return 1

    def _before_exit(self, exit_status):
        if self.config:
            self.get_stats().timing("scap.scap", self.get_duration() * 1000)
        return exit_status


@cli.command("pull-master", help=argparse.SUPPRESS)
class SyncMaster(cli.Application):
    """Sync local MediaWiki staging directory with deploy server state."""

    @cli.argument("master", help="Master rsync server to copy from")
    def main(self, *extra_args):
        tasks.sync_master(
            self.config, master=self.arguments.master, verbose=self.verbose
        )
        return 0


@cli.command(
    "pull",
    help="Sync local MediaWiki deployment directory with "
    "deploy server state (formerly sync-common)",
)
class SyncPull(cli.Application):
    """Sync local MediaWiki deployment directory with deploy server state."""

    @cli.argument(
        "--no-update-l10n",
        action="store_false",
        dest="update_l10n",
        help="Do not update l10n cache files.",
    )
    @cli.argument(
        "--exclude-wikiversions.php",
        action="store_true",
        dest="exclude_wikiversions_php",
        help="Do not rsync wikiversions.php.",
    )
    @cli.argument(
        "-i",
        "--include",
        default=None,
        action="append",
        help="Rsync include pattern to limit transfer to."
        " End directories with a trailing `/***`."
        " Can be used multiple times.",
    )
    @cli.argument(
        "--delete-excluded",
        action="store_true",
        help="Also delete local files not found on the master.",
    )
    @cli.argument(
        "--no-php-restart",
        action="store_false",
        dest="php_restart",
        help="Check to see if php needs a restart",
    )
    @cli.argument(
        "servers", nargs=argparse.REMAINDER, help="Rsync server(s) to copy from"
    )
    def main(self, *extra_args):
        rsync_args = ["--delete-excluded"] if self.arguments.delete_excluded else []
        tasks.sync_common(
            self.config,
            include=self.arguments.include,
            sync_from=self.arguments.servers,
            verbose=self.verbose,
            rsync_args=rsync_args,
            exclude_wikiversionsphp=self.arguments.exclude_wikiversions_php,
        )

        if self.arguments.update_l10n:
            with log.Timer("scap-cdb-rebuild", self.get_stats()):
                utils.sudo_check_call(
                    user="mwdeploy",
                    cmd=self.get_script_path() + " cdb-rebuild --no-progress",
                    app=self
                )

        if self.arguments.php_restart:
            fpm = php_fpm.PHPRestart(self.config)
            self.get_logger().info("Checking if php-fpm restart needed")
            failed = fpm.restart_self()
            if failed:
                self.get_logger().warning("php-fpm restart failed!")

        return 0


@cli.command("sync-dir", help=argparse.SUPPRESS)
@cli.command("sync-file")
class SyncFile(AbstractSync):
    """Sync a specific file/directory to the cluster."""

    @cli.argument("--force", action="store_true", help="Skip canary checks")
    @cli.argument(
        "--pause-after-testserver-sync",
        action="store_true",
        help="Pause after syncing testservers and prompt the user to confirm to continue syncing",
    )
    @cli.argument("file", help="File/directory to sync")
    @cli.argument("message", nargs="*", help="Log message for SAL")
    def main(self, *extra_args):
        self.arguments.stop_before_sync = False
        return super().main(*extra_args)

    def _before_cluster_sync(self):
        # assert file exists
        abspath = os.path.join(self.config["stage_dir"], self.arguments.file)
        if not os.path.exists(abspath):
            raise IOError(errno.ENOENT, "File/directory not found", abspath)

        relpath = os.path.relpath(abspath, self.config["stage_dir"])
        if os.path.isdir(abspath):
            relpath = "%s/***" % relpath
        self.include = relpath

        # Notify when syncing a symlink.
        if os.path.islink(abspath):
            symlink_dest = os.path.realpath(abspath)
            self.get_logger().info(
                "%s: syncing symlink, not its target [%s]", abspath, symlink_dest
            )
        else:
            lint.check_valid_syntax(abspath, utils.cpus_for_jobs())

    def _after_cluster_sync(self):
        self._restart_php()

    def _base_scap_pull_command(self):
        cmd = [self.get_script_path(), "pull", "--no-update-l10n", "--no-php-restart"]

        if "/" in self.include:
            parts = self.include.split("/")
            for i in range(1, len(parts)):
                # Include parent directories in sync command or the default
                # exclude will block them and by extension block the target
                # file.
                cmd.extend(["--include", "/".join(parts[:i])])

        cmd.extend(["--include", self.include])
        if self.verbose:
            cmd.append("--verbose")
        return cmd

    def _after_lock_release(self):
        self.announce(
            "Synchronized %s: %s (duration: %s)",
            self.arguments.file,
            self.arguments.message,
            utils.human_duration(self.get_duration()),
        )
        self.increment_stat("sync-file")


@cli.command("sync-l10n")
class SyncL10n(AbstractSync):
    """Sync l10n files for a given branch and rebuild cache files."""

    @cli.argument("--force", action="store_true", help="Skip canary checks")
    @cli.argument(
        "version", type=arg.is_version, help="MediaWiki version (eg 1.27.0-wmf.7)"
    )
    @cli.argument("message", nargs="*", help="Log message for SAL")
    def main(self, *extra_args):
        self.arguments.stop_before_sync = False
        return super().main(*extra_args)

    def _before_cluster_sync(self):
        if self.arguments.version.startswith("php-"):
            self.arguments.version = self.arguments.version[4:]

        # Assert version is active
        if self.arguments.version not in self.active_wikiversions("stage"):
            raise IOError(errno.ENOENT, "Version not active", self.arguments.version)

        # Assert l10n cache dir for version exists
        abspath = os.path.join(
            self.config["stage_dir"], "php-%s/cache/l10n" % self.arguments.version
        )
        if not os.path.isdir(abspath):
            raise IOError(errno.ENOENT, "Directory not found", abspath)

        relpath = os.path.relpath(abspath, self.config["stage_dir"])
        self.include = "%s/***" % relpath

    def _base_scap_pull_command(self):
        cmd = [self.get_script_path(), "pull", "--no-update-l10n", "--no-php-restart"]

        parts = self.include.split("/")
        for i in range(1, len(parts)):
            # Include parent directories in sync command or the default
            # exclude will block them and by extension block the target
            # file.
            cmd.extend(["--include", "/".join(parts[:i])])

        cmd.extend(["--include", self.include])
        if self.verbose:
            cmd.append("--verbose")
        return cmd

    def _after_cluster_sync(self):
        # Rebuild l10n CDB files
        target_hosts = self._get_target_list()
        with log.Timer("scap-cdb-rebuild", self.get_stats()):
            rebuild_cdbs = ssh.Job(
                target_hosts, user=self.config["ssh_user"], key=self.get_keyholder_key()
            )
            rebuild_cdbs.shuffle()
            cdb_cmd = "sudo -u mwdeploy -n -- {} cdb-rebuild --version {}"
            cdb_cmd = cdb_cmd.format(self.get_script_path(), self.arguments.version)
            rebuild_cdbs.command(cdb_cmd)
            rebuild_cdbs.progress(
                log.reporter("scap-cdb-rebuild", self.config["fancy_progress"])
            )
            succeeded, failed = rebuild_cdbs.run()
            if failed:
                self.get_logger().warning(
                    "%d hosts had scap-cdb-rebuild errors", failed
                )
                self.soft_errors = True
        tasks.clear_message_blobs(self.config)
        self._restart_php()

    def _after_lock_release(self):
        self.announce(
            "scap sync-l10n completed (%s) (duration: %s)",
            self.arguments.version,
            utils.human_duration(self.get_duration()),
        )
        self.increment_stat("l10nupdate-sync")


@cli.command("sync-wikiversions")
class SyncWikiversions(AbstractSync):
    """Rebuild and sync wikiversions.php to the cluster."""

    def _update_caches(self):
        """
        Skip this step.

        It currently consists only of cache_git_info and this class should
        attempt to be fast where possible.
        """
        pass

    def _before_cluster_sync(self):
        """
        check for the presence of ExtensionMessages and l10n cache
        for every branch of mediawiki that is referenced in wikiversions.json
        to avoid syncing a branch that is lacking these critical files.
        """
        for version in self.active_wikiversions("stage"):
            ext_msg = os.path.join(
                self.config["stage_dir"],
                "wmf-config",
                "ExtensionMessages-%s.php" % version,
            )
            err_msg = "ExtensionMessages not found in %s" % ext_msg
            utils.check_file_exists(ext_msg, err_msg)

            cache_file = os.path.join(
                self.config["stage_dir"],
                "php-%s" % version,
                "cache",
                "l10n",
                "l10n_cache-en.cdb",
            )
            err_msg = "l10n cache missing for %s" % version
            utils.check_file_exists(cache_file, err_msg)

        # Tell the remaining stages to only rsync wikiversions*.* files.
        self.include = "wikiversions*.*"

    def _after_lock_release(self):
        self.announce(
            "rebuilt and synchronized wikiversions files: %s", self.arguments.message
        )

        self.increment_stat("sync-wikiversions")

    def _after_cluster_sync(self):
        self._restart_php()


@cli.command("cdb-json-refresh", help=argparse.SUPPRESS)
class RefreshCdbJsonFiles(cli.Application):
    """
    Create JSON/MD5 files for all CDB files in a directory.

    This will put a JSON and MD5 file in /upstream for each CDB file.

    This can be combined with rsync and the scap-rebuild-cdbs to
    push out large CDB files with minimal traffic. CDB files change
    drastically with small key/value changes, where as JSON files do not, and
    thus they diff/rdiff much better.

    When pushing updates with rsync, this should be run before running rsync.
    The rsync command should exclude CDB files or at least use
    -ignore-existing. After the rsync is done, scap-rebuild-cdbs can be
    run on each server to apply the updates to the CDB files.
    """

    @cli.argument(
        "-d",
        "--directory",
        required=True,
        type=arg.is_dir,
        help="Directory containing cdb files",
    )
    @cli.argument(
        "-t",
        "--threads",
        default=1,
        type=int,
        help="Number of threads to use to build json/md5 files",
    )
    def main(self, *extra_args):
        cdb_dir = os.path.realpath(self.arguments.directory)
        upstream_dir = os.path.join(cdb_dir, "upstream")
        use_cores = self.arguments.threads

        if not os.path.isdir(cdb_dir):
            raise IOError(errno.ENOENT, "Directory does not exist", cdb_dir)

        if use_cores < 1:
            use_cores = utils.cpus_for_jobs()

        if not os.path.isdir(upstream_dir):
            os.mkdir(upstream_dir)

        tasks.refresh_cdb_json_files(cdb_dir, use_cores, self.verbose)


@cli.command("version", help="Show the version number and exit")
class Version(cli.Application):
    def main(self, *extra_args):
        print(scapversion.__version__)
        return 0


@cli.command("lock", help="Temporarily lock deployment of this repository")
class LockManager(cli.Application):
    """
    Holds a lock open for a given repository.

    examples::

        lock 'Testing something, do not deploy'
    """

    @cli.argument(
        "--all",
        action="store_true",
        help="Lock ALL repositories from deployment. "
        + "With great power comes great responsibility",
    )
    @cli.argument(
        "--time",
        type=int,
        default=3600,
        help="How long to lock deployments, in seconds",
    )
    @cli.argument("message", nargs="*", help="Log message for SAL/lock file")
    def main(self, *extra_args):
        logger = self.get_logger()

        if self.arguments.message == "(no justification provided)":
            logger.fatal("Cannot lock repositories without a reason")
            return 1

        if self.arguments.all:
            lock_path = lock.GLOBAL_LOCK_FILE
            repo = "ALL REPOSITORIES"
        else:
            lock_path = self.get_lock_file()
            repo = self.config["git_repo"]

        got_lock = False
        with lock.Lock(lock_path, self.arguments.message, group_write=True):
            got_lock = True
            self.announce(
                "Locking from deployment [%s]: %s (planned duration: %s)",
                repo,
                self.arguments.message,
                utils.human_duration(self.arguments.time),
            )

            logger.info("Press enter to abort early...")
            try:
                rlist, _, _ = select.select([sys.stdin], [], [], self.arguments.time)
                if rlist:
                    sys.stdin.readline()
            except KeyboardInterrupt:
                pass  # We don't care here

        if got_lock:
            self.announce(
                "Unlocked for deployment [%s]: %s (duration: %s)",
                repo,
                self.arguments.message,
                utils.human_duration(self.get_duration()),
            )

        return 0
