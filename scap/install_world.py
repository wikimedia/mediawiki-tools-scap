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
from os.path import expanduser

import packaging.version

from scap import cli, targets, utils, ssh, log
from scap.lock import Lock


@cli.command(
    "install-world",
    help="Install scap version on targets",
    affected_by_blocked_deployments=True,
)
class InstallWorld(cli.Application):
    """
    Scap sub-command to install scap version on targets
    """

    MIN_VERSION = packaging.version.Version("4.42.0")
    # When changing the supported distros, also update the standalone list in install_local_version.sh
    SUPPORTED_DISTRIBUTIONS = ["buster", "bullseye", "bookworm"]
    # The rsync module is defined in the operations/puppet repo
    SCAP_RSYNC_MODULE = "scap-install-staging"
    WHEELS_DIR = "scap-wheels"
    INSTALL_SCAP_SCRIPT = "install_local_version.sh"
    INSTALL_SCAP_SCRIPT_PATH = f"scap/bin/{INSTALL_SCAP_SCRIPT}"

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
        "-v",
        "--version",
        default="latest",
        help="The version tag to be installed",
    )
    @cli.argument(
        "-l",
        "--limit-hosts",
        default="all",
        help="Limit install to hosts matching pattern. A regex can be indicated with a tilde (~)"
        " prefix",
    )
    @cli.argument(
        "-x",
        "--exclude-hosts",
        help="Exclude hosts matching regex. The hosts are removed after --limit-hosts is applied",
    )
    @cli.argument("-y", "--yes", action="store_true", help="Answer yes to all prompts")
    @cli.argument(
        "-i",
        "--install-targets-only",
        action="store_true",
        help="Do not update the scap version. Install targets using the version currently in the"
        " staging area",
    )
    @cli.argument(
        "-b",
        "--batch",
        action="store_true",
        help="Batch mode. Implies --yes and --install-targets-only",
    )
    def main(self, *extra_args):
        if self.arguments.batch:
            self.arguments.yes = True
            self.arguments.install_targets_only = True
            # This will prevent `announce` from notifying the IRC channel when running in batch
            # mode. Messages will still be logged
            self.arguments.no_log_message = True

        # The Lock ensures a scap installation cannot happen during a MediaWiki update
        with Lock(
            self.get_lock_file(), name="install-world", reason="Scap is being updated"
        ):
            self._initialize_from_config()
            self._select_targets()
            self._select_version()

            if not self.arguments.yes and not utils.prompt_user_for_confirmation(
                f"""Scap version "{self.version}" will be installed on {len(self.targets)} host(s). Proceed?"""
            ):
                utils.abort("Canceled by user")

            self.announce(
                f"""Installing scap version "{self.version}" for {len(self.targets)} hosts"""
            )

            if not self.arguments.install_targets_only:
                self._install_local_scap()

                other_masters = list(self.masters)
                other_masters.remove(self.deploy_master)
                if other_masters:
                    self._sync_masters_scap_installation(other_masters)
            self._install_scap_targets()

            self.announce(
                f"""Installation of scap version "{self.version}" completed for {len(self.targets)} hosts"""
            )

    def _initialize_from_config(self):
        self.masters = self.get_master_list()
        self.install_user = self.config["install_ssh_user"]
        self.install_user_home = expanduser("~" + self.install_user)
        self.install_user_ssh_key = f"/etc/keyholder.d/{self.install_user}.pub"

        if not os.path.exists(self.install_user_home):
            utils.abort(f"User's home dir at {self.install_user_home} does not exist")
        if not os.path.exists(self.install_user_ssh_key):
            utils.abort(f"SSH key {self.install_user_ssh_key} does not exist")

    def _select_targets(self):
        selected_targets = targets.get(
            "scap_targets",
            self.config,
            self.arguments.limit_hosts,
            exclude_hosts=self.arguments.exclude_hosts,
        ).all
        self.targets = [
            target for target in selected_targets if target not in self.masters
        ]

        if not self.targets:
            utils.abort("List of targets is empty")

    def _select_version(self):
        if self.arguments.install_targets_only:
            self._use_staged_wheels()
        else:
            self._use_version_from_args()

    def _use_staged_wheels(self):
        staged_versions = set()
        wheels_dir = pathlib.Path(self.install_user_home, InstallWorld.WHEELS_DIR)

        for distro in InstallWorld.SUPPORTED_DISTRIBUTIONS:
            matches = list(pathlib.Path(wheels_dir, distro).glob("Scap-*.whl"))

            if not matches:
                if self.arguments.batch:
                    self._abort(
                        f"""Scap wheels for distro "{distro}" missing in staging area. Cannot proceed in"""
                        " batch mode"
                    )
                else:
                    self.logger.warn(
                        f"""Scap wheels for distro "{distro}" missing in staging area. Falling back to"""
                        " regular update"
                    )
                    self._use_version_from_args()
                    self.arguments.install_targets_only = False
                    return

            if len(matches) > 1:
                self._abort(
                    f"""Found multiple Scap wheels for distro "{distro}" in staging area. Something is broken"""
                )

            staged_versions.add(
                re.search(r"Scap-(\d+\.\d+\.\d+)", matches[0].name).group(1)
            )

        if len(staged_versions) > 1:
            self._abort(
                "Distro wheels in staging area have multiple versions. Something is broken"
            )

        self.version = staged_versions.pop()
        self.logger.info(f"Using version {self.version} found in staging area")

    def _use_version_from_args(self):
        self.version = self.arguments.version

        if self.version != "latest":
            try:
                requested_version = packaging.version.Version(self.version)
                if requested_version < InstallWorld.MIN_VERSION:
                    utils.abort(
                        f"""Self-install with wheels not supported for version "{self.arguments.version}" """
                    )
            except packaging.version.InvalidVersion:
                utils.abort(f"""Version "{self.arguments.version}" is not valid""")

    def _install_local_scap(self):
        self.logger.info(f"""Installing version "{self.version}" locally""")

        install_script_path = (
            f"{self.install_user_home}/{InstallWorld.INSTALL_SCAP_SCRIPT_PATH}"
        )
        cmd = [
            install_script_path,
            "-u",
            self.install_user,
            "--on-deploy",
            "-t",
            self.version,
            "-d",
            ",".join(InstallWorld.SUPPORTED_DISTRIBUTIONS),
        ]

        with utils.suppress_backtrace():
            subprocess.run(cmd, check=True)

    def _sync_masters_scap_installation(self, masters_to_sync):
        self.logger.info("Syncing masters")

        rsync_call = [
            "/usr/bin/rsync",
            "--archive",
            "--delay-updates",
            "--delete",
            "--delete-delay",
            "--compress",
            "--new-compress",
            "--exclude=*.swp",
            "--exclude=**/__pycache__",
            f"{self.deploy_master}::{InstallWorld.SCAP_RSYNC_MODULE}",
            self.install_user_home,
        ]
        masters_sync = self._get_ssh_job_for(masters_to_sync)
        masters_sync.command(rsync_call)
        masters_sync.progress(
            log.reporter("scap-sync-to-masters", self.config["fancy_progress"])
        )
        _, failed = masters_sync.run()
        if failed:
            self._abort(f"{failed} masters failed to sync scap installation")

    def _install_scap_targets(self):
        targets_by_master, targets_no_master = self._map_targets_to_master_by_dc()
        self._assign_targets_to_random_master(targets_by_master, targets_no_master)
        self._sync_targets(targets_by_master)
        self._install_targets()

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
        targets_by_master = {
            master: all_targets[master] for master in all_targets.keys() if master
        }
        targets_no_master = all_targets[None] if None in all_targets else []

        return targets_by_master, targets_no_master

    def _assign_targets_to_random_master(self, targets_by_master, targets):
        for target in targets:
            targets_by_master.setdefault(random.choice(self.masters), list()).append(
                target
            )

    def _sync_targets(self, targets_by_master):
        for master, tgts in targets_by_master.items():
            self.logger.info(
                f"""Syncing installation material to {len(tgts)} scap targets from "{master}" """
            )

            rsync_call = [
                "/usr/bin/rsync",
                "--archive",
                "--delay-updates",
                "--delete",
                "--delete-delay",
                "--compress",
                "--new-compress",
                f"{self.deploy_master}::{InstallWorld.SCAP_RSYNC_MODULE}/{InstallWorld.WHEELS_DIR}/$(lsb_release -cs)",
                f"{self.deploy_master}::{InstallWorld.SCAP_RSYNC_MODULE}/{InstallWorld.INSTALL_SCAP_SCRIPT_PATH}",
                self.install_user_home + "/",
            ]
            targets_sync = self._get_ssh_job_for(tgts)
            targets_sync.command(rsync_call)
            targets_sync.progress(
                log.reporter("scap-sync-to-targets", self.config["fancy_progress"])
            )
            _, failed = targets_sync.run()
            if failed:
                self._abort(
                    f"{failed} targets failed to sync scap installation material"
                )

    def _install_targets(self):
        self.logger.info(f"""Installing {len(self.targets)} scap targets""")

        install_script_path = (
            f"{self.install_user_home}/{InstallWorld.INSTALL_SCAP_SCRIPT}"
        )
        targets_install = self._get_ssh_job_for(self.targets)
        targets_install.command([install_script_path, "-u", self.install_user])
        targets_install.progress(
            log.reporter("scap-install-to-targets", self.config["fancy_progress"])
        )
        _, failed = targets_install.run()
        if failed:
            self._abort(f"{failed} targets failed to install scap")

    def _get_ssh_job_for(self, hosts) -> ssh.Job:
        return ssh.Job(hosts, user=self.install_user, key=self.install_user_ssh_key)

    def _abort(self, message):
        self.logger.error(message)
        if self.arguments.batch:
            # Exit quietly to avoid the error bubbling up and affecting the caller (e.g. Puppet)
            exit(0)
        utils.abort("Install failed")
