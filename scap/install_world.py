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
import random
import re
import socket
import subprocess
import sys
from os.path import expanduser

import requests
from packaging.version import Version, InvalidVersion

if sys.version_info < (3, 9):
    from typing_extensions import Tuple
else:
    from typing import Tuple

from scap import cli, targets, utils, ssh, log


@cli.command(
    "install-world",
    help="Install scap version on targets",
    primary_deploy_server_only=True,
)
class InstallWorld(cli.Application):
    """
    Scap sub-command to install scap version on targets
    """

    # When changing the supported distros, also update the standalone list in install_local_version.sh
    SUPPORTED_DISTRIBUTIONS = ["bullseye", "bookworm", "trixie"]
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
        self.selected_masters = []
        self.selected_targets = []
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
    def main(self, *extra_args):
        # The lock ensures a scap installation cannot happen during a MediaWiki update
        with self.lock_mediawiki_staging(
            name="install-world",
            reason="Scap is being updated",
        ):
            self._initialize_from_config()
            self._set_version()
            self._select_masters()
            self._select_targets()

            total_install_hosts = len(self.selected_masters) + len(
                self.selected_targets
            )
            if total_install_hosts == 0:
                utils.abort("No hosts to install. Nothing to do")

            if not self.arguments.yes and not self.prompt_user_for_confirmation(
                f"""Scap version "{self.version}" will be installed on {total_install_hosts} host(s). Proceed?"""
            ):
                utils.abort("Canceled by user")

            self.announce(
                f"""Installing scap version "{self.version}" for {total_install_hosts} host(s)"""
            )

            download_only = self.deploy_master not in self.selected_masters
            self._install_local_scap(download_only)

            self._sync_masters_scap_installation()
            if len(self.selected_masters) > 0:
                # Under normal conditions, syncing scap should be enough to have a working scap on the other masters.
                # However, during the reimaging of deployment servers, the Python versions between different
                # masters may differ for a few days, leading to failed scap installations and errors during regular
                # deployments. Even though it's a rare occurrence, we install scap using the wheels to avoid this
                # problem (see https://phabricator.wikimedia.org/T371261)
                self._install_secondary_scap_masters()
            if len(self.selected_targets) > 0:
                self._install_scap_targets()

            self.announce(
                f"""Installation of scap version "{self.version}" completed for {total_install_hosts} hosts"""
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

    def _set_version(self):
        if self.arguments.version == "latest":
            #  Resolve "latest" into an actual version tag
            with utils.suppress_backtrace():
                # Not very Pythonic, but avoids having to install the extra package 'distro'
                distro = subprocess.check_output(
                    ["lsb_release", "-cs"], text=True
                ).strip()
            res = requests.get(
                f"https://docker-registry.wikimedia.org/v2/repos/releng/scap/{distro}/tags/list"
            )
            res.raise_for_status()
            versions = [
                version for version in res.json().get("tags") if version != "latest"
            ]
            versions.sort(key=Version)
            self.version = versions[-1]
        else:
            self.version = self.arguments.version

        try:
            Version(self.version)
        except InvalidVersion:
            utils.abort(f""""{self.version}" is not a valid version""")

    def _select_masters(self):
        self.selected_masters = self.get_master_list(
            limit_hosts=self.arguments.limit_hosts,
            exclude_hosts=self.arguments.exclude_hosts,
        )

    def _select_targets(self):
        selected_targets = targets.get(
            "scap_targets",
            self.config,
            self.arguments.limit_hosts,
            exclude_hosts=self.arguments.exclude_hosts,
        ).all
        self.selected_targets = [
            target for target in selected_targets if target not in self.masters
        ]

    def _install_local_scap(self, download_only: bool = False):
        action = "Downloading" if download_only else "Installing"
        self.logger.info(f"""{action} version "{self.version}" locally""")

        install_script_path = (
            f"{self.install_user_home}/{InstallWorld.INSTALL_SCAP_SCRIPT_PATH}"
        )
        cmd = [
            install_script_path,
            "--user",
            self.install_user,
            "--on-primary",
            "--tag",
            self.version,
            "--distros",
            ",".join(InstallWorld.SUPPORTED_DISTRIBUTIONS),
            "--skip-install" if download_only else "",
        ]

        with utils.suppress_backtrace():
            subprocess.run(cmd, check=True)

    def _sync_masters_scap_installation(self):
        masters_to_sync = list(self.masters)
        masters_to_sync.remove(self.deploy_master)
        if len(masters_to_sync) == 0:
            return

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
        masters_sync.progress(log.reporter("scap-sync-to-masters"))
        _, failed = masters_sync.run()
        if failed:
            self._abort(f"{failed} masters failed to sync scap installation")

    def _install_secondary_scap_masters(self):
        masters_to_install = list(self.selected_masters)
        if self.deploy_master in masters_to_install:
            masters_to_install.remove(self.deploy_master)
        if len(masters_to_install) == 0:
            return

        self.logger.info("Installing secondary scap masters")

        install_script_path = (
            f"{self.install_user_home}/{InstallWorld.INSTALL_SCAP_SCRIPT_PATH}"
        )
        masters_install = self._get_ssh_job_for(masters_to_install)
        masters_install.command(
            [
                install_script_path,
                "--user",
                self.install_user,
                "--on-secondary",
                "--tag",
                self.version,
            ]
        )
        masters_install.progress(log.reporter("scap-install-to-masters"))
        _, failed = masters_install.run()
        if failed:
            self._abort(f"{failed} masters failed to install scap")

    def _install_scap_targets(self):
        targets_by_master, targets_no_master = self._map_targets_to_master_by_dc()
        self._assign_targets_to_random_master(targets_by_master, targets_no_master)
        self._sync_targets(targets_by_master)
        self._install_targets()

    def _map_targets_to_master_by_dc(self) -> Tuple[dict, list]:
        def select_master(target):
            for master in self.masters:
                domain = re.search(r"^[^.]+.(.+)", master).group(1)
                if domain in target:
                    return master
            return None

        all_targets = {}
        for target in self.selected_targets:
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

            # Note that changes to the directory structure of the installation material need to be reflected in
            # the Puppet Scap bootstrapping script at:
            # https://gerrit.wikimedia.org/r/plugins/gitiles/operations/puppet/+/refs/heads/production/modules/scap/files/bootstrap-scap-target.sh

            base_rsync_command = [
                "/usr/bin/rsync",
                "--archive",
                "--compress",
                "--new-compress",
            ]
            rsync_wheels_call = base_rsync_command + [
                "--delay-updates",
                "--delete",
                "--delete-delay",
                f"{master}::{InstallWorld.SCAP_RSYNC_MODULE}/{InstallWorld.WHEELS_DIR}/$(lsb_release -cs)/{self.version}/",
                f"{self.install_user_home}/$(lsb_release -cs)",
            ]
            targets_sync = self._get_ssh_job_for(tgts)
            targets_sync.command(rsync_wheels_call)
            targets_sync.progress(log.reporter("scap-sync-wheels-to-targets"))
            _, failed = targets_sync.run()

            if not failed:
                rsync_install_script_call = base_rsync_command + [
                    f"{master}::{InstallWorld.SCAP_RSYNC_MODULE}/{InstallWorld.INSTALL_SCAP_SCRIPT_PATH}",
                    self.install_user_home + "/",
                ]
                targets_sync.command(rsync_install_script_call)
                targets_sync.progress(
                    log.reporter("scap-sync-install-script-to-targets")
                )
                _, failed = targets_sync.run()

            if failed:
                self._abort(
                    f"{failed} targets failed to sync scap installation material"
                )

    def _install_targets(self):
        self.logger.info(f"""Installing {len(self.selected_targets)} scap targets""")

        install_script_path = (
            f"{self.install_user_home}/{InstallWorld.INSTALL_SCAP_SCRIPT}"
        )
        targets_install = self._get_ssh_job_for(self.selected_targets)
        targets_install.command([install_script_path, "--user", self.install_user])
        targets_install.progress(log.reporter("scap-install-to-targets"))
        _, failed = targets_install.run()
        if failed:
            self._abort(f"{failed} targets failed to install scap")

    def _get_ssh_job_for(self, hosts) -> ssh.Job:
        return ssh.Job(hosts, user=self.install_user, key=self.install_user_ssh_key)

    def _abort(self, message):
        self.logger.error(message)
        utils.abort("Install failed")
