# -*- coding: utf-8 -*-
"""
    scap.install_world
    ~~~~~~~~~~
    Scap command to install a version (tag) of scap on all known targets. If no version is
    specified, the latest tag is used

    Copyright © 2014-2022 Wikimedia Foundation and Contributors.

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
import os
import pathlib
import random
import re
import socket
import subprocess
import sys
from os.path import expanduser

import packaging.version

from scap import cli, targets, utils, ssh, log
from scap.lock import TimeoutLock, Lock
from scap.runcmd import gitcmd


@cli.command(
    "install-world",
    help="Install scap version on targets"
)
class InstallWorld(cli.Application):
    """
    Scap sub-command to install scap version on targets
    """

    LATEST_NON_SUPPORTED_VERSION = packaging.version.Version('4.8.0')

    def __init__(self, exe_name):
        super().__init__(exe_name)

        self.logger = self.get_logger()
        self.deploy_master = socket.getfqdn()

        self.install_user = None
        self.install_user_home = None
        self.install_user_ssh_key = None
        self.masters = []
        self.targets = []
        self.version = None

    @cli.argument(
        "--version",
        default="latest",
        help="The version tag to be installed",
    )
    @cli.argument(
        "--limit-hosts",
        default="all",
        help="Limit install to hosts matching pattern. A regex can be indicated with a tilde (~)"
             " prefix",
    )
    @cli.argument(
        "--lock-timeout",
        type=int,
        help="Timeout to wait for the install concurrency lock to be released. In minutes",
    )
    @cli.argument(
        "-y", "--yes",
        action="store_true",
        help="Answer yes to all prompts"
    )
    @cli.argument(
        "-s", "--sync-only",
        action="store_true",
        help="Do not select a scap version. Sync out using the version currently in the staging"
             " area"
    )
    @cli.argument(
        "-b", "--batch",
        action="store_true",
        help="Batch mode. Implies --yes and --sync-only. Useful to automate the priming of new scap"
             " targets"
    )
    def main(self, *extra_args):
        if self.arguments.batch:
            self.arguments.yes = True
            self.arguments.sync_only = True
            # This will prevent `announce` from notifying the IRC channel when running in batch
            # mode. Messages will still be logged
            self.arguments.no_log_message = True

        lock_timeout = \
            {"timeout": self.arguments.lock_timeout} if self.arguments.lock_timeout else {}
        # The TimeoutLock serializes potential scap operations running in parallel
        with TimeoutLock(self.config["serializing_lock_file"], name="concurrent install", **lock_timeout):
            # The Lock ensures a scap installation cannot happen during a Mediawiki update
            with Lock(self.get_lock_file(), "Scap is being updated"):
                self._initialize_from_config()
                self._select_targets()
                self._select_version()

                if not self.arguments.yes and not utils.prompt_user_for_confirmation(
                    """Scap version "%s" will be installed on %d host(s). Proceed?"""
                    % (self.version, len(self.targets))
                ):
                    utils.abort("Canceled by user")

                self.announce(
                    """Installing scap version "%s" for %d hosts"""
                    % (self.version, len(self.targets))
                )

                if not self.arguments.sync_only:
                    self._install_local_scap()
                    self._sync_masters_scap_installation()
                self._sync_targets_scap_installation()
                # Filthy hack to have the lib dir automatically added to `sys.path` in targets
                self._create_lib_dir_symlink_on_targets()

                self.announce(
                    """Installation of scap version "%s" completed for %d hosts"""
                    % (self.version, len(self.targets))
                )

    def _initialize_from_config(self):
        self.masters = self.get_master_list()
        self.install_user = self.config["install_ssh_user"]
        self.install_user_home = expanduser("~" + self.install_user)
        self.install_user_ssh_key = "/etc/keyholder.d/%s.pub" % self.install_user

        if not os.path.exists(self.install_user_home):
            utils.abort("""User's home dir at "%s" does not exist""" % self.install_user_home)
        if not os.path.exists(self.install_user_ssh_key):
            utils.abort("""SSH key "%s" does not exist""" % self.install_user_ssh_key)

    def _select_targets(self):
        selected_targets = targets.get("scap_targets", self.config, self.arguments.limit_hosts).all
        self.targets = [target for target in selected_targets if target not in self.masters]

        if not self.targets:
            utils.abort("List of targets is empty")

    def _select_version(self):
        if self.arguments.sync_only:
            self._use_staged_version()
        else:
            self._use_version_from_args()

    def _use_staged_version(self):
        matches = list(
            pathlib.Path(self.install_user_home, "scap", "lib").glob("**/scap/version.py")
        )
        if not matches:
            if self.arguments.batch:
                self._abort("No scap detected in staging area. Cannot proceed in batch mode")
            else:
                self.logger.warn("No scap detected in staging area. Falling back to regular install")
                self._use_version_from_args()
                self.arguments.sync_only = False
        else:
            if len(matches) > 1:
                self._abort(
                    "Somehow found multiple scap installations in staging area (?). Something is"
                    " broken"
                )

            self.version = re.search(r"['\"](\d+\.\d+\.\d+)", matches[0].read_text()).group(1)
            self.logger.info("""Using version "%s" found in staging area""" % self.version)

    def _use_version_from_args(self):
        if self.arguments.version == "latest":
            self.version =\
                gitcmd("tag", "--sort", "-taggerdate", cwd=self.config["scap_source_dir"]).split()[0]
        else:
            self.version = self.arguments.version

        try:
            requested_version = packaging.version.Version(self.version)
            if requested_version <= InstallWorld.LATEST_NON_SUPPORTED_VERSION:
                utils.abort(
                    """Self-install not supported for version "%s" """ % self.arguments.version
                )
        except packaging.version.InvalidVersion:
            utils.abort("""Version "%s" is not valid""" % self.arguments.version)

        try:
            subprocess.run(
                ["git", "rev-parse", "tags/%s" % self.version],
                check=True,
                cwd=self.config["scap_source_dir"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except subprocess.CalledProcessError:
            utils.abort(
                """Version "%s" does not exist""" % self.version
            )

    def _install_local_scap(self):
        self.logger.info("Installing version %s locally" % self.version)

        install_script_path = os.path.join(os.path.dirname(sys.argv[0]), "install_local_version.sh")
        cmd = [
            install_script_path,
            "-u", self.install_user,
            "-t", self.version,
            self.config["scap_source_dir"]
        ]
        with utils.suppress_backtrace():
            subprocess.run(cmd, check=True)

    def _sync_masters_scap_installation(self):
        self.logger.info("Syncing masters")

        rsync_call = _get_scap_rsync_call_for(self.deploy_master, self.install_user_home)

        masters_sync = self._get_ssh_job_for(self.masters)
        masters_sync.exclude_hosts([self.deploy_master])
        masters_sync.command(rsync_call)
        masters_sync.progress(log.reporter("scap-sync-to-masters", self.config["fancy_progress"]))
        _, failed = masters_sync.run()
        if failed:
            self._abort("%d masters failed to sync scap installation" % failed)

    def _sync_targets_scap_installation(self):
        targets_by_master, targets_no_master = self._map_targets_to_master_by_dc()
        self._assign_targets_to_random_master(targets_by_master, targets_no_master)
        self._install_targets(targets_by_master)

    def _map_targets_to_master_by_dc(self) -> (dict, list):
        def select_master(target):
            for master in self.masters:
                domain = re.search(r"^[^.]+.(.+)", master).group(1)
                if domain in target:
                    return master
            return None

        all_targets = {}
        for target in self.targets:
            all_targets.setdefault(select_master(target), list()).append(target)
        targets_by_master = {master: all_targets[master] for master in all_targets.keys() if master}
        targets_no_master = all_targets[None] if None in all_targets else []

        return targets_by_master, targets_no_master

    def _assign_targets_to_random_master(self, targets_by_master, targets):
        for target in targets:
            targets_by_master.setdefault(random.choice(self.masters), list()).append(target)

    def _install_targets(self, targets_by_master):
        for master, tgts in targets_by_master.items():
            self.logger.info('Syncing %d scap targets from "%s"' % (len(tgts), master))

            rsync_call = _get_scap_rsync_call_for(master, self.install_user_home)

            targets_sync = self._get_ssh_job_for(tgts)
            targets_sync.command(rsync_call)
            targets_sync.progress(
                log.reporter("scap-sync-to-targets", self.config["fancy_progress"])
            )
            _, failed = targets_sync.run()
            if failed:
                self._abort("%d targets failed to sync scap installation" % failed)

    def _create_lib_dir_symlink_on_targets(self):
        """
        Hack that creates a symlink pointing to the scap lib dir from the (major.minor) Python3
        version installed on the remote targets. This causes the local interpreter to add the lib
        dir to `sys.path`. Without that dir in the `sys.path`, scap cannot find its deps and fails
        to run
        """

        # Note that scap/bin/python3 is a symlink
        lib_rename_cmd =\
            r"%s/scap/bin/python3 --version | cut -d\' \' -f2 | cut -d. -f1-2"\
            ' | { read version; [ ! "$(ls -d %s/scap/lib/python*)" = %s/scap/lib/python$version ]'\
            ' && ln -s %s/scap/lib/python* %s/scap/lib/python$version || :; }'\
            % tuple([self.install_user_home] * 5)
        lib_rename = self._get_ssh_job_for(self.targets)
        lib_rename.command(lib_rename_cmd)
        lib_rename.progress(
            log.reporter("create-lib-dir-symlink-on-targets", self.config["fancy_progress"])
        )
        _, failed = lib_rename.run()
        if failed:
            self._abort(
                "%d targets failed to create symlink to scap installation lib dir" % failed
            )

    def _get_ssh_job_for(self, hosts) -> ssh.Job:
        return ssh.Job(hosts, user=self.install_user, key=self.install_user_ssh_key)

    def _abort(self, message):
        self.logger.error(message)
        if self.arguments.batch:
            # Exit quietly to avoid the error bubbling up and affecting the caller (e.g. Puppet)
            exit(0)
        utils.abort("Install failed")


def _get_scap_rsync_call_for(master, destination_dir):
    return [
        "/usr/bin/rsync",
        "--archive",
        "--delay-updates",
        "--delete",
        "--delete-delay",
        "--compress",
        "--new-compress",
        "--exclude=*.swp",
        "--exclude=**/__pycache__",
        # "scap-install-staging" is an rsync module defined in the operations/puppet repo
        "%s::scap-install-staging" % master,
        destination_dir
    ]
