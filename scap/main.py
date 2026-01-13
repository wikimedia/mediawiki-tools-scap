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
import argparse
import errno
from dataclasses import dataclass
import datetime
import locale
import os
import select
import socket
import sys
import time
from typing import Callable, List, Optional
from concurrent.futures import ThreadPoolExecutor

import scap.arg as arg
import scap.checks as checks
import scap.cli as cli
import scap.git as git
import scap.lint as lint
import scap.lock as lock
import scap.log as log
import scap.logstash_checker as logstash_checker
import scap.mwscript as mwscript
import scap.php_fpm as php_fpm
import scap.ssh as ssh
import scap.targets as targets
import scap.tasks as tasks
import scap.utils as utils
import scap.version as scapversion
from scap import ansi, history
from scap.kubernetes import K8sOps, TEST_SERVERS, CANARIES, PRODUCTION, STAGES


@dataclass
class DeploymentStage:
    name: str
    baremetal_targets: List[str]
    pre_check_func: Optional[Callable]
    check_func: Optional[Callable]
    post_check_func: Optional[Callable]


class AbstractSync(cli.Application):
    """Base class for applications that want to sync one or more files from
    the deployment server to the rest of the cluster."""

    soft_errors = False

    def __init__(self, exe_name):
        super().__init__(exe_name)
        self.includes = []
        self.exclude_wikiversions_php = False
        self.om = None
        self.logo = True
        # A list of hostnames that have been processed by self._perform_sync
        self.already_synced = []
        self.already_restarted = set()
        self.k8s_ops = None

    @cli.argument(
        "--force",
        action="store_true",
        help="Skip error checks, testserver and canary checks, performs ungraceful php-fpm restarts",
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
    @cli.argument(
        "--notify-user",
        action="append",
        default=[],
        help="User to notify on IRC after sync to testservers."
        " Can be used multiple times",
    )
    @cli.argument(
        "--k8s-only",
        action="store_true",
        help="Deploy/sync to Kubernetes targets only",
    )
    @cli.argument(
        "--k8s-confirm-diffs",
        action="store_true",
        help="Display and require confirmation of helmfile diffs before proceeding.",
    )
    @cli.argument("message", nargs="*", help="Log message for SAL")
    def main(self, *extra_args):
        """Perform a sync operation to the cluster."""
        if self.logo:
            print(ansi.logo(color=utils.should_colorize_output()))

        os.umask(self.config["umask"])

        self._assert_auth_sock()

        if self.arguments.message == "(no justification provided)":
            new_message = self.input_line("Log message (press enter for none): ")
            if new_message:
                self.arguments.message = new_message

        with self.lock_mediawiki_staging(name="sync"):
            self.k8s_ops = K8sOps(self)

            # Begin by confirming helmfile diffs, if enabled. This avoids having
            # to clean up after operations with side-effects if cancelled.
            self._confirm_k8s_diffs()

            self._compile_wikiversions()
            self._before_cluster_sync()
            self._update_caches()

            if not self.arguments.force:
                self.get_logger().info("Checking for new runtime errors locally")
                self._check_fatals()
            else:
                self.get_logger().warning("check_fatals skipped by --force")

            self._build_and_push_container_images()
            self.k8s_ops.update_helmfile_files()

            if self.arguments.stop_before_sync:
                self.announce_final("Stopping before sync operations")
                return 0

            # Sync masters regardless of the --k8s-only flag since some of the
            # sync'd information affects k8s deployments.
            self._sync_masters()

            if self._k8s_only_sync():
                baremetal_full_target_list = []
                baremetal_testservers = []
                baremetal_canaries = []
            else:
                # Bare-metal targets
                baremetal_full_target_list = self._get_target_list()
                baremetal_testservers = utils.list_intersection(
                    self._get_testserver_list(), baremetal_full_target_list
                )
                baremetal_canaries = utils.list_intersection(
                    self._get_canary_list(), baremetal_full_target_list
                )

            deployment_stages = [
                DeploymentStage(
                    TEST_SERVERS,
                    baremetal_testservers,
                    self._announce_testservers_synced,
                    self.check_testservers,
                    self._pause_after_testserver_sync,
                ),
                DeploymentStage(
                    CANARIES,
                    baremetal_canaries,
                    None,
                    self.canary_checks,
                    None,
                ),
                DeploymentStage(
                    PRODUCTION,
                    baremetal_full_target_list,
                    None,
                    None,
                    self._after_cluster_sync,  # Bare metal php-fpm restarts happen in here
                ),
            ]

            self._init_history()
            try:
                for depstage in deployment_stages:
                    stage = depstage.name

                    k8s_timer_name = (
                        "sync-prod-k8s" if stage == PRODUCTION else f"sync-{stage}-k8s"
                    )

                    # k8s deployment
                    if self.config["deploy_mw_container_image"]:
                        with (
                            self.Timer(k8s_timer_name),
                            self.reported_status(f"Sync k8s {stage}"),
                        ):
                            with utils.suppress_backtrace():
                                self.k8s_ops.deploy_k8s_images_for_stage(stage)

                    # Bare metal deployment
                    if depstage.baremetal_targets:
                        with self.reported_status(f"Sync baremetal {stage}"):
                            if stage == PRODUCTION:
                                self._sync_proxies_and_apaches(
                                    depstage.baremetal_targets
                                )
                            else:
                                with self.Timer(f"sync-{stage}"):
                                    self.sync_targets(depstage.baremetal_targets, stage)

                    if depstage.pre_check_func:
                        depstage.pre_check_func()

                    if depstage.check_func:
                        if self.arguments.force:
                            self.get_logger().warning(
                                "%s checks skipped by --force", stage
                            )
                        else:
                            depstage.check_func(depstage.baremetal_targets)

                    if depstage.post_check_func:
                        depstage.post_check_func()

                self.deployment_log_entry.completed = True
            finally:
                self._finalize_history()

        self._after_lock_release()
        if self.soft_errors:
            return 1
        return 0

    def _init_history(self):
        self.deployment_history = history.History(self.scap_history_dbfile())

        self.deployment_log_entry = history.Deployment(
            starttime=datetime.datetime.now(datetime.timezone.utc),
            username=utils.get_real_username(),
        )

        def _add_checkout(directory):
            branch = git.get_branch(directory)
            commit_ref = git.merge_base(directory, "origin", branch)

            self.deployment_log_entry.checkouts.append(
                history.Checkout(
                    repo=git.remote_get_url(directory),
                    branch=branch,
                    directory=directory,
                    commit_ref=commit_ref,
                )
            )

        staging_dir = self.config["stage_dir"]

        _add_checkout(staging_dir)

        for version in self.active_wikiversions("stage"):
            _add_checkout(os.path.join(staging_dir, f"php-{version}"))

    def _finalize_history(self):
        self.deployment_log_entry.errors = self.soft_errors
        self.deployment_log_entry.endtime = datetime.datetime.now(datetime.timezone.utc)
        self.deployment_history.log(self.deployment_log_entry)

    def _sync_proxies_and_apaches(self, full_target_list):
        # Update proxies
        proxies = utils.list_intersection(self._get_proxy_list(), full_target_list)

        if len(proxies) > 0:
            with self.Timer("sync-proxies"):
                sync_cmd = self._apache_sync_command()
                sync_cmd.append(socket.getfqdn())
                self._perform_sync("proxies", sync_cmd, proxies)

        # Update apaches
        with self.Timer("sync-apaches"):
            self._perform_sync(
                "apaches",
                self._apache_sync_command(proxies),
                full_target_list,
                shuffle=True,
            )

    def increment_stat(self, stat, all_stat=True, value=1):
        """Increment a stat in deploy.*

        :param stat: String name of stat to increment
        :param all_stat: Whether to increment deploy.all as well
        :param value: How many to increment by, default of 1 is normal
        """
        self.get_stats().increment("deploy.%s" % stat, value)
        if all_stat:
            self.get_stats().increment("deploy.all", value)

    def get_keyholder_key(self, *args, **kwargs):
        """
        Returns scap2-specific deploy key

        This way we can set a key in the default scap config without
        having all non-scap2 repos inherit that configuration.
        """
        key = self.config.get("mediawiki_keyholder_key", None)
        if key:
            return key

        return super().get_keyholder_key(*args, **kwargs)

    def _before_cluster_sync(self):
        pass

    def _get_notify_users(self):
        return ", ".join(
            set(
                [utils.get_real_username()] + getattr(self.arguments, "notify_user", [])
            )
        )

    def _get_testservers_synced_message(self, brief=False):
        url = "https://wikitech.wikimedia.org/wiki/Mwdebug"
        if brief:
            what = "Changes"
        else:
            what = f"{self._get_notify_users()}: {self.arguments.message}"

        return f"{what} synced to the testservers (see {url})"

    def _should_pause_after_testserver_sync(self) -> bool:
        # Not all subclasses of AbstractSync define the --pause-after-testserver-sync argument,
        # so we can't assume it is in self.arguments.
        return getattr(self.arguments, "pause_after_testserver_sync", False)

    def _announce_testservers_synced(self):
        if not self._should_pause_after_testserver_sync():
            return

        message = self._get_testservers_synced_message()
        self.announce(f"{message}. Changes can now be verified there.")

    def _pause_after_testserver_sync(self):
        if not self._should_pause_after_testserver_sync():
            return

        self.report_status("Waiting user to confirm testservers deployment")
        message = self._get_testservers_synced_message(brief=True)
        self.prompt_for_approval_or_exit(
            f"{message}\nBefore continuing, please do any necessary checks if you haven't already.\n"
            "Continue with sync?",
            "Sync cancelled.",
        )
        self.announce(f"{self._get_notify_users()}: Continuing with sync")
        self.report_status("Continuing with sync")

    def _k8s_only_sync(self):
        # Not all subclasses of AbstractSync define the --k8s-only option
        return getattr(self.arguments, "k8s_only", False)

    def _confirm_k8s_diffs(self):
        if not getattr(self.arguments, "k8s_confirm_diffs", False):
            return
        logger = self.get_logger()
        logger.info("Collecting helmfile diffs for review")
        for stage in STAGES:
            diffs = self.k8s_ops.helmfile_diffs_for_stage(stage)
            if not diffs:
                logger.warning("No diffs for stage %s", stage)
                continue
            for diff in sorted(
                diffs,
                key=lambda diff: (
                    diff["cluster"],
                    diff["namespace"],
                    diff["release"],
                ),
            ):
                self.output_line(
                    "=== Diff for {cluster}/{namespace}-{release} in {stage} ===\n{diff_stdout}".format(
                        stage=stage, **diff
                    ),
                    sensitive=True,  # Diffs may contain sensitive values.
                )
        self.prompt_for_approval_or_exit(
            "Note: Diffs are relative to the current helm charts and helmfile values. These may "
            "become outdated if new changes are merged during sync.\n"
            "Continue with sync?",
            "Sync cancelled.",
        )

    def _build_and_push_container_images(self):
        if not self.config["build_mw_container_image"]:
            return

        with (
            self.reported_status("Building container images"),
            self.Timer("build-and-push-container-images"),
        ):
            versions = self.get_versions_to_include_in_image()
            if self.config["build_mw_next_container_image"]:
                versions.append("next")

            self.k8s_ops.build_k8s_images(versions)

    def _update_caches(self):
        # Compute git version information
        with self.Timer("cache_git_info"):
            self.cache_git_info()

    def _check_fatals(self):
        logger = self.get_logger()

        for version, wikidb in self.active_wikiversions(
            "stage", return_type=dict
        ).items():
            logger.debug("Testing {} with eval.php using {}".format(version, wikidb))
            with utils.suppress_backtrace():
                proc = mwscript.run(
                    self,
                    "eval.php",
                    wiki=wikidb,
                    check=False,
                    network=True,
                )
                if proc.stderr:
                    raise SystemExit(
                        "'mwscript eval.php --wiki {}' generated unexpected output: {}".format(
                            wikidb, proc.stderr
                        )
                    )

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
        """Get list of MediaWiki testservers."""
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
            with self.reported_status("Syncing to baremetal masters"):
                self.master_only_cmd("sync-masters", self._master_sync_command())

    def master_only_cmd(self, timer, cmd):
        """
        Run a command on all other master servers than the one we're on

        :param timer: String name to use in timer/logging
        :param cmd: List of command/parameters to be executed
        """

        masters = self.get_master_list()
        with self.Timer(timer):
            update_masters = ssh.Job(
                masters, user=self.config["ssh_user"], key=self.get_keyholder_key()
            )
            update_masters.exclude_hosts([socket.getfqdn()])
            update_masters.command(cmd)
            update_masters.progress(log.reporter(timer))
            succeeded, failed = update_masters.run()
            if failed:
                self.get_logger().warning("%d masters had sync errors", failed)
                self.soft_errors = True

    def _master_sync_command(self):
        """Synchronization command to run on the master hosts."""
        cmd = [self.get_script_path(remote=True), "pull-master"]
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
        """
        cmd = [self.get_script_path(remote=True), "pull"]

        cmd.extend(["--no-php-restart", "--no-update-l10n"])
        if self.exclude_wikiversions_php:
            cmd.append("--exclude-wikiversions.php")

        if self.verbose:
            cmd.append("--verbose")

        for include in self.includes:
            cmd.extend(["--include", include])

        return cmd

    def _apache_sync_command(self, proxies: list = (), **kwargs) -> list:
        """
        Returns (as a list) the scap pull command to run on mediawiki
        installation targets.  This is comprised of the base scap pull command
        (defined by _base_scap_pull_command) followed by the list of deployment
        masters and the list of proxies.

        :param proxies: A list of proxy hostnames that can be pulled from in addition
                        to the deployment masters. Default is empty (as a tuple to prevent mutable
                        list warning)
        :param kwargs:  Any remaining keyword arguments are passed on to _base_scap_pull_command.
        """
        return self._base_scap_pull_command(**kwargs) + utils.list_union(
            self.get_master_list(), proxies
        )

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
            key=self.get_keyholder_key(),
        )

        job.exclude_hosts(self.already_synced)

        if shuffle:
            job.shuffle()

        job.progress(log.reporter("sync-{}".format(type)))

        jobresults = job.run(return_jobresults=True)
        self.get_logger().info(
            "Per-host sync duration: average %ss, median %ss",
            locale.format("%.1f", jobresults.average_duration(), grouping=True),
            locale.format("%.1f", jobresults.median_duration(), grouping=True),
        )

        num_hosts = 0
        total_transferred = 0

        for jobresult in jobresults:
            num_hosts += 1
            total_transferred += utils.parse_rsync_stats(jobresult.output).get(
                "total_transferred_file_size", 0
            )

        average_transferred = total_transferred // num_hosts if num_hosts else 0

        self.get_logger().info(
            "rsync transfer: average {:n} bytes/host, total {:n} bytes".format(
                average_transferred, total_transferred
            )
        )

        failed = jobresults.num_failed
        if failed:
            self.get_logger().warning("%d %s had sync errors", failed, type)
            self.soft_errors = True

        self.already_synced += targets

    def _compile_wikiversions(self):
        """Compile wikiversions.json to wikiversions.php in stage_dir"""
        tasks.compile_wikiversions("stage", self.config)

    def _after_cluster_sync(self):
        pass

    def _after_lock_release(self):
        pass

    def _after_sync_rebuild_cdbs(self, target_hosts, stage: str):
        """
        `stage` must be a string like "testservers", "canaries", or "prod".
        """
        # Ask target hosts to rebuild l10n CDB files
        with self.Timer(f"scap-cdb-rebuild-{stage}"):
            rebuild_cdbs = ssh.Job(
                target_hosts, user=self.config["ssh_user"], key=self.get_keyholder_key()
            )
            rebuild_cdbs.shuffle()
            rebuild_cdbs.command(
                "sudo -u mwdeploy -n -- %s cdb-rebuild"
                % self.get_script_path(remote=True)
            )
            rebuild_cdbs.progress(log.reporter("scap-cdb-rebuild"))
            succeeded, failed = rebuild_cdbs.run()
            if failed:
                self.get_logger().warning(
                    "%d hosts had scap-cdb-rebuild errors", failed
                )
                self.soft_errors = True

    def _after_sync_sync_wikiversions(self, target_hosts, stage: str):
        # Update and sync wikiversions.php
        succeeded, failed = tasks.sync_wikiversions(
            target_hosts, self, stage, key=self.get_keyholder_key()
        )
        if failed:
            self.get_logger().warning("%d hosts had sync_wikiversions errors", failed)
            self.soft_errors = True

    def sync_targets(self, targets=None, stage=None):
        """
        This function is used to sync to bare metal testservers and canaries.

        Run scap pull on the targets, including l10n rebuild, wikiversions sync,
        and php-fpm restart. The pull source will be this deploy server.

        :param targets: Iterable of target servers to sync

        :param stage: A string like "testservers" or "canaries".
        """
        sync_cmd = self._apache_sync_command()
        sync_cmd.append(socket.getfqdn())

        self._perform_sync(stage, sync_cmd, targets)
        self._after_sync_rebuild_cdbs(targets, stage)
        self._after_sync_sync_wikiversions(targets, stage)
        self._restart_php_hostgroups([targets], stage)

    def retry_continue_exit(self, description, test_func):
        """
        Runs test_func().  If it returns True, this function returns.
        If test_func() does not return true:

        * If an interactive terminal is not available, self.cancel() is called
          and it is not expected to return.
        * If an interactive terminal is available, ask the user if they want to
          retry the test, continue with deployment anyway, or exit.   If the user
          chooses to retry, this function starts over.  If the user chooses to
          continue, a message is logged and this function returns.  If the user
          chooses to exit, self.cancel() is called and it is not expected to return.

        'description' is used to describe the operation to retry in the
        retry/continue/exit prompt.
        """
        while True:
            if test_func():
                break

            resp = self.prompt_choices(
                # FIXME: If this is a spiderpig job, this should instruct
                # the user to look at the job log for details, ideally with a link.
                f"{description.capitalize()} failed. What do you want to do?",
                {
                    f"Retry {description}": "r",
                    "Ignore failure and proceed with deployment": "i",
                    "Cancel deployment": "c",
                },
                "c",
            )
            if resp == "r":
                # loop around and try again
                continue
            elif resp == "i":
                self.get_logger().info("Continuing with deployment")
                break
            elif resp == "c":
                self.cancel()
                break
            else:
                raise Exception("This should never happen")

    def cancel(self):
        self.announce("Scap cancelled\nWARNING: Nothing has been rolled back.")
        sys.exit(1)

    def canary_checks(self, baremetal_canaries: list):
        """
        Run logstash error rate check (for bare metal and mw-on-k8s).

        :raises SystemExit: on canary check failure
        """

        logger = self.get_logger()

        canary_wait_time = self.config["canary_wait_time"]

        checker = logstash_checker.LogstashChecker(
            self.config["logstash_host"],
            canary_wait_time,
            baremetal_canaries,
            self.k8s_ops.get_canary_namespaces(),
            logger,
        )

        need_sleep = True
        last_check_time = time.time()

        def test_func() -> bool:
            nonlocal need_sleep, last_check_time
            # Ensure that at least `canary_wait_time` seconds have elapsed
            # since the last check, unless there was previously a logstash_checker.CheckServiceError,
            # in which case we skip the wait.
            if need_sleep:
                time_since_last_check = time.time() - last_check_time
                wait_remaining = round(canary_wait_time - time_since_last_check)
                if wait_remaining > 0:
                    self.report_status(
                        f"Waiting {wait_remaining} seconds for canary traffic...",
                        log=True,
                    )
                    time.sleep(wait_remaining)

            try:
                last_check_time = time.time()
                return checker.check(self.config["canary_threshold"])
            except logstash_checker.CheckServiceError as e:
                logger.error("The canary error rate checker failed: %s", e)
                self.report_status("Canary error rate checker failed")
                need_sleep = False
                return False
            else:
                need_sleep = True

        self.retry_continue_exit("canary checks", test_func)

    def check_testservers(self, baremetal_testservers: list):
        """
        Check bare metal and k8s testservers.

        :raises SystemExit: on check failure
        """
        baremetal_check_cmd = self.config["testservers_check_cmd_baremetal"]
        k8s_check_cmd = self.config["testservers_check_cmd_k8s"]
        checkslist = []

        def split_subcommands(cmds: str) -> list:
            subcommands = []
            for cmd in cmds.splitlines():
                cmd = cmd.strip()
                if cmd:
                    subcommands.append(cmd)
            return subcommands

        if baremetal_check_cmd and baremetal_testservers:
            env = os.environ.copy()
            env["BAREMETAL_TESTSERVERS"] = ",".join(baremetal_testservers)
            subcommands = split_subcommands(baremetal_check_cmd)
            for i, cmd in enumerate(subcommands):
                checkslist.append(
                    checks.Check(
                        f"check_testservers_baremetal-{i+1}_of_{len(subcommands)}",
                        command=cmd,
                        timeout=120,
                        shell=True,
                        environment=env,
                    )
                )
        if k8s_check_cmd:
            subcommands = split_subcommands(k8s_check_cmd)
            for i, cmd in enumerate(subcommands):
                checkslist.append(
                    checks.Check(
                        f"check_testservers_k8s-{i+1}_of_{len(subcommands)}",
                        command=cmd,
                        timeout=120,
                        shell=True,
                    )
                )

        if not checkslist:
            return

        logger = self.get_logger()

        with (
            self.Timer("check-testservers"),
            self.reported_status("Checking testservers"),
        ):

            def test_func() -> bool:
                success, jobs = checks.execute(
                    checkslist, logger, concurrency=len(checkslist)
                )
                return success

            self.retry_continue_exit("testserver checks", test_func)

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
        will run it.

        If the operator invoked scap with the --force flag, then --force will
        be passed to php_fpm_restart_script to restart php-fpm unsafely (i.e.,
        without depooling and repooling around the service restart).  T243009

        Targets that have already been restarted will not be restarted again.
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
        group_hosts = []
        for group in target_groups.groups.values():
            target_hosts = set(group.targets) - self.already_restarted
            if target_hosts:
                group_hosts.append(list(target_hosts))
                self.already_restarted |= target_hosts
        self._restart_php_hostgroups(group_hosts, "prod")

    def _restart_php_hostgroups(self, target_hosts, stage: str):
        """Perform php restart for sets of hosts (if configured).

        :param target_hosts: A list of lists of hostnames.
        :param stage: The deployment stage ("testservers", "canaries", or "prod")
        """
        num_hosts = 0
        for grp in target_hosts:
            num_hosts += len(grp)
        if not self._setup_php() or num_hosts == 0:
            return
        with (
            self.Timer(f"php-fpm-restarts-{stage}"),
            self.reported_status("Restarting php-fpm"),
        ):
            self.get_logger().info(
                "Running '{}' on {} host(s)".format(php_fpm.INSTANCE.cmd, num_hosts)
            )
            with log.MultithreadedProgressReportCollection("php-fpm-restart") as q:
                php_fpm.INSTANCE.set_progress_queue(q)
                with ThreadPoolExecutor(max_workers=5) as pool:
                    results = pool.map(php_fpm.restart_helper, target_hosts)
            for _, failed in results:
                if failed:
                    self.get_logger().warning(
                        "%d hosts had failures restarting php-fpm", failed
                    )


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


@cli.command(
    "wikiversions-compile", help=argparse.SUPPRESS, primary_deploy_server_only=True
)
class CompileWikiversions(cli.Application):
    """Compile wikiversions.json to wikiversions.php."""

    # This flag is no longer needed, but left here for compatibility with any existing scripts which
    # might still pass this flag.  T357581, T329857
    @cli.argument(
        "--staging",
        action="store_true",
        help="Compile wikiversions in staging directory.  This flag is deprecated since wikiversions-compile always uses the staging directory.",
    )
    def main(self, *extra_args):
        if self.arguments.staging:
            self.get_logger().warning(
                "The --staging flag is deprecated and will be removed in a future release."
            )

        tasks.compile_wikiversions("stage", self.config)
        return 0


@cli.command("wikiversions-inuse")
class MWVersionsInUse(cli.Application):
    """Get a list of the active MediaWiki versions."""

    @cli.argument(
        "--withdb",
        action="store_true",
        help="Add `=wikidb` with some wiki using the version.",
    )
    # This flag is no longer needed, but left here for compatibility with any existing scripts which
    # might still pass this flag.  T357581, T329857
    @cli.argument(
        "--staging",
        action="store_true",
        help="Read wikiversions from the staging directory.  This flag is deprecated since wikiversions-inuse always uses the staging directory.",
    )
    def main(self, *extra_args):
        if self.arguments.staging:
            self.get_logger().warning(
                "The --staging flag is deprecated and will be removed in a future release."
            )

        versions = self.active_wikiversions("stage", return_type=dict)

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
        "--master",
        action="store_true",
        help="Operate in a mode suitable for a deploy master",
    )
    def main(self, *extra_args):
        if self.arguments.master:
            user = "www-data"
            source_tree = "stage"
            root_dir = self.config["stage_dir"]
        else:
            user = "mwdeploy"
            source_tree = "deploy"
            root_dir = self.config["deploy_dir"]

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


@cli.command(
    "sync-world",
    help="Deploy MediaWiki to the cluster",
    primary_deploy_server_only=True,
    require_tty_multiplexer=True,
)
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
    #. Run purgeMessageBlobStore.php
    #. Rolling invalidation of all opcache for php 7.x
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # We want to exclude wikiversions.php during the normal rsync.
        # wikiversions.php is handled separately in _after_cluster_sync.
        self.exclude_wikiversions_php = True

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
    @cli.argument("-n", action="store_true", help="No-op for running tests")
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
    @cli.argument(
        "--notify-user",
        action="append",
        default=[],
        help="User to notify on IRC after sync to testservers."
        " Can be used multiple times",
    )
    @cli.argument(
        "--k8s-only",
        action="store_true",
        help="Deploy/sync to Kubernetes targets only",
    )
    @cli.argument(
        "--k8s-confirm-diffs",
        action="store_true",
        help="Display and require confirmation of helmfile diffs before proceeding.",
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

        if self.arguments.n:
            return 0

        return super().main(*extra_args)

    def _before_cluster_sync(self):
        self.announce("Started scap sync-world: %s", self.arguments.message)
        self.report_status("Starting scap sync-world")

        # Validate php syntax of wmf-config and multiversion
        lint.check_valid_syntax(
            self,
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
            self.get_logger().warning("Skipping l10n-update")
        else:
            with self.Timer("l10n-update"), self.reported_status("Updating l10n files"):
                for version in self.active_wikiversions("stage"):
                    tasks.update_localization_cache(version, self)

    def _after_cluster_sync(self):
        target_hosts = self._get_target_list()

        self._after_sync_rebuild_cdbs(target_hosts, "prod")
        self._after_sync_sync_wikiversions(target_hosts, "prod")
        self._restart_php()
        tasks.clear_message_blobs(self)

    def _after_lock_release(self):
        self.announce(
            "Finished scap sync-world: %s (duration: %s)",
            self.arguments.message,
            utils.human_duration(self.get_duration()),
        )
        self.increment_stat("scap")

    def _handle_exception_log_message(self, message):
        self.announce(message)

    def _before_exit(self, exit_status):
        if self.config:
            self.get_stats().timing("scap.scap", self.get_duration() * 1000)
        return exit_status


@cli.command("pull-master", help=argparse.SUPPRESS)
class SyncMaster(cli.Application):
    """Sync local MediaWiki staging directory with deploy server state."""

    @cli.argument("master", help="Master rsync server to copy from")
    def main(self, *extra_args):
        tasks.sync_master(self, master=self.arguments.master, verbose=self.verbose)
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
        help="Don't restart php-fpm after the pull.",
    )
    @cli.argument(
        "servers", nargs=argparse.REMAINDER, help="Rsync server(s) to copy from"
    )
    def main(self, *extra_args):
        rsync_args = ["--delete-excluded"] if self.arguments.delete_excluded else []
        tasks.sync_common(
            self,
            include=self.arguments.include,
            sync_from=self.arguments.servers,
            verbose=self.verbose,
            rsync_args=rsync_args,
            exclude_wikiversionsphp=self.arguments.exclude_wikiversions_php,
        )

        if self.arguments.update_l10n:
            with self.Timer("scap-cdb-rebuild"):
                utils.sudo_check_call(
                    user="mwdeploy",
                    cmd=self.get_script_path() + " cdb-rebuild --no-progress",
                    app=self,
                )

        if self.arguments.php_restart:
            fpm = php_fpm.PHPRestart(self.config, logger=self.get_logger())
            self.get_logger().info("Checking if php-fpm restart needed")
            failed = fpm.restart_self()
            if failed:
                self.get_logger().warning("php-fpm restart failed!")

        return 0


@cli.command(
    "sync-dir",
    help=argparse.SUPPRESS,
    primary_deploy_server_only=True,
    require_tty_multiplexer=True,
)
@cli.command("sync-file", primary_deploy_server_only=True, require_tty_multiplexer=True)
class SyncFile(AbstractSync):
    """Sync a specific file/directory to the cluster."""

    @cli.argument("--force", action="store_true", help="Skip canary checks")
    @cli.argument(
        "--pause-after-testserver-sync",
        action="store_true",
        help="Pause after syncing testservers and prompt the user to confirm to continue syncing",
    )
    @cli.argument(
        "--notify-user",
        action="append",
        default=[],
        help="User to notify on IRC after sync to testservers."
        " Can be used multiple times",
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
        include = relpath

        # Notify when syncing a symlink.
        if os.path.islink(abspath):
            symlink_dest = os.path.realpath(abspath)
            self.get_logger().info(
                "%s: syncing symlink, not its target [%s]", abspath, symlink_dest
            )
        else:
            lint.check_valid_syntax(self, abspath, utils.cpus_for_jobs())

        if "/" in include:
            parts = include.split("/")
            for i in range(1, len(parts)):
                # Include parent directories in sync command or the default
                # exclude will block them and by extension block the target
                # file.
                self.includes.append("/".join(parts[:i]))
        self.includes.append(include)

    def _after_cluster_sync(self):
        self._restart_php()

    def _after_lock_release(self):
        self.announce(
            "Synchronized %s: %s (duration: %s)",
            self.arguments.file,
            self.arguments.message,
            utils.human_duration(self.get_duration()),
        )
        self.increment_stat("sync-file")


@cli.command(
    "sync-wikiversions", primary_deploy_server_only=True, require_tty_multiplexer=True
)
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


@cli.command(
    "cdb-json-refresh", help=argparse.SUPPRESS, primary_deploy_server_only=True
)
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


@cli.command(
    "lock",
    help="Temporarily lock deployment of this repository",
    primary_deploy_server_only=True,
    require_tty_multiplexer=True,
)
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
        "--bg",
        action="store_true",
        help="Run in background (detach from terminal) after acquiring the lock.  Only valid with --all.",
    )
    @cli.argument(
        "--unlock-all",
        action="store_true",
        help="Remove global lock for all repositories",
    )
    @cli.argument("message", nargs="*", help="Log message for SAL/lock file")
    def main(self, *extra_args):
        logger = self.get_logger()

        if self.arguments.unlock_all:
            if self.arguments.message == "(no justification provided)":
                logger.fatal("Cannot request to remove global lock without a reason")
                return 1

            self.announce(f"Forcefully removing global lock: {self.arguments.message}")
            lock.Lock.signal_gl_release(self.arguments.message, self.get_io())
            return

        if self.arguments.message == "(no justification provided)":
            logger.fatal("Cannot lock repositories without a reason")
            return 1

        if self.arguments.bg and not self.arguments.all:
            logger.fatal("--bg can only be used with --all")
            return 1

        if self.arguments.all:
            lock_path = lock.GLOBAL_LOCK_FILE
            repo = "ALL REPOSITORIES"
        else:
            try:
                lock_path = self.get_scap3_lock_file()
                repo = self.config["git_repo"]
            except LookupError:
                # The current directory does not appear to be a proper
                # scap3 deploy directory, so fall back to legacy behavior
                # and assume the user means to lock the MediaWiki staging
                # directory.
                lock_path = self.get_mediawiki_staging_lock_file()
                repo = "MediaWiki"

        # self.locker() writes to locker_status_w to signal the parent that
        # the lock has been acquired, and later writes to it again to
        # signal that the lock has been externally released.
        locker_status_r, locker_status_w = os.pipe()

        # self.ui() writes to release_w to tell self.locker() to release the lock
        release_r, release_w = os.pipe()

        # Fork before acquiring the lock (fcntl locks are not inherited across fork)
        pid = os.fork()
        if pid > 0:
            # The parent process handles the UI
            return self.ui(
                repo,
                locker_status_r,
                locker_status_w,
                release_r,
                release_w,
                pid,
                logger,
            )
        else:
            # The child process handles acquiring and holding the lock
            return self.locker(
                lock_path,
                repo,
                locker_status_r,
                locker_status_w,
                release_r,
                release_w,
                logger,
            )

    def ui(
        self,
        repo,
        locker_status_r,
        locker_status_w,
        release_r,
        release_w,
        pid,
        logger,
    ) -> int:
        os.close(locker_status_w)
        os.close(release_r)

        # Wait for self.locker() to signal successful lock acquisition
        lock_acquired = False
        try:
            result = os.read(locker_status_r, 1)
            if result == b"\x01":
                lock_acquired = True
        except OSError:
            pass

        if not lock_acquired:
            logger.error("Failed to acquire lock")
            return 1

        self.announce("Locking from deployment [%s]: %s", repo, self.arguments.message)

        if self.arguments.bg:
            # Background mode - exit immediately after lock is acquired
            return 0

        # Foreground mode - wait for user input or the released signal from self.locker()
        logger.info("Press enter to unlock...")

        try:
            fds = [sys.stdin, locker_status_r]
            rlist, _, _ = select.select(fds, [], [])
            if sys.stdin in rlist:
                try:
                    sys.stdin.readline()
                except KeyboardInterrupt:
                    pass
                try:
                    os.write(release_w, b"\x01")
                except OSError:
                    pass
            else:
                # self.locker() has indicated that is has released the lock
                try:
                    os.read(locker_status_r, 1)  # drain the signal
                except OSError:
                    pass
        except KeyboardInterrupt:
            pass
        finally:
            self.quiet_close(locker_status_r, release_w)

        os.waitpid(pid, 0)
        return 0

    def locker(
        self,
        lock_path,
        repo,
        locker_status_r,
        locker_status_w,
        release_r,
        release_w,
        logger,
    ) -> int:
        os.close(locker_status_r)
        os.close(release_w)

        try:
            with self.lock(lock_path):
                # Notify self.ui() that lock is acquired
                os.write(locker_status_w, b"\x01")

                # Disconnect from the terminal if running in background mode
                if self.arguments.bg:
                    os.setsid()
                    si = open("/dev/null", "r")
                    so = open("/dev/null", "a+")
                    se = open("/dev/null", "a+")
                    os.dup2(si.fileno(), sys.stdin.fileno())
                    os.dup2(so.fileno(), sys.stdout.fileno())
                    os.dup2(se.fileno(), sys.stderr.fileno())

                forced_lock_release_r = None
                forced_lock_release_w = None

                if self.arguments.all:

                    def release_global_lock(*args):
                        # Notify self.ui() that the lock has been released
                        os.write(forced_lock_release_w, b"\x01")

                    forced_lock_release_r, forced_lock_release_w = os.pipe()
                    lock.Lock.watch_for_gl_release_signal(release_global_lock)

                try:
                    fds = []
                    if not self.arguments.bg:
                        fds.append(release_r)
                    if forced_lock_release_r is not None:
                        fds.append(forced_lock_release_r)
                    select.select(fds, [], [])
                except KeyboardInterrupt:
                    pass
                finally:
                    # Notify self.ui() that we're done holding the lock
                    try:
                        os.write(locker_status_w, b"\x00")
                    except OSError:
                        pass
                    self.quiet_close(locker_status_w)

                    if forced_lock_release_r is not None:
                        self.quiet_close(forced_lock_release_r, forced_lock_release_w)

                    self.quiet_close(release_r)
        except Exception as e:
            logger.error("Failed to acquire lock: %s", e)
            # Notify self.ui() of failure by closing the pipe without writing
            self.quiet_close(locker_status_w)
            os._exit(1)

        self.announce(
            "Unlocked for deployment [%s]: %s (duration: %s)",
            repo,
            self.arguments.message,
            utils.human_duration(self.get_duration()),
        )

        return 0

    def quiet_close(self, *fds):
        """Safely close file descriptors, ignoring errors."""
        for fd in fds:
            try:
                os.close(fd)
            except OSError:
                pass
