# -*- coding: utf-8 -*-
"""
    scap.deploy_promote
    ~~~~~~~~~~
    Scap command to promote a specified group of wikis to the specified wmf deployment branch (or
    the latest branch if none is specified)

    Example usage: scap deploy-promote group0 1.38.0-wmf.20

    The above command promotes all group0 wikis (testwiki and mediawiki.org)
    to version 1.38.0-wmf.20.

    The behavior associated to deploy-promote used to live in the `tools/release` repository as a
    shellscript. The file history is still available in that repo and can be viewed with:
        git log -- bin/deploy-promote

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
import re
import time
from functools import partial

import requests
from requests import HTTPError

from scap import cli, interaction, utils, config, git, train
from scap.runcmd import gitcmd
from scap.utils import BRANCH_RE_UNANCHORED

print = partial(print, flush=True)


@cli.command(
    "deploy-promote",
    help="Promote group of wikis to specific/latest wmf deployment branch",
    primary_deploy_server_only=True,
    require_tty_multiplexer=True,
)
class DeployPromote(cli.Application):
    """
    Scap sub-command to promote a specified group of wikis to a specific/latest wmf deployment branch
    """

    logger = None

    group = None
    promote_version = None
    announce_message = None
    commit_message = None

    @cli.argument(
        "group",
        help="existing group to which you'd like to deploy a new version (testwikis, group0, group1"
        ", or all).",
    )
    @cli.argument(
        "version",
        nargs="?",
        default=None,
        help="version to deploy (ex: 1.36.0-wmf.2). Defaults to the latest version found in"
        ' "<stage_dir>/<wikiversions_filename>". By default "<stage_dir>" is'
        " " + config.DEFAULT_CONFIG["stage_dir"][1],
    )
    @cli.argument("-y", "--yes", action="store_true", help="answer yes to all prompts")
    @cli.argument(
        "--train",
        action="store_true",
        help="First set all wikis to the version specified by --old-version, "
        "then set the new version on all train groups up to and including the target group",
    )
    @cli.argument("--old-version", help="The old train version to be used with --train")
    def main(self, *extra_args):
        self.logger = self.get_logger()

        if self.arguments.train and not self.arguments.old_version:
            utils.abort("--old-version must be used along with --train")

        self.group = self.arguments.group
        self._check_group()
        self._check_user_auth_sock()

        sorted_versions = self.active_wikiversions("stage")

        self.promote_version = self.arguments.version or sorted_versions[-1]

        prev_version = "/".join(
            utils.get_group_versions(
                self.arguments.group, self.config["stage_dir"], self.config["wmf_realm"]
            )
        )

        if not self.arguments.yes and not self._prompt_user_to_approve(prev_version):
            utils.abort("Canceled by user")

        os.umask(self.config["umask"])
        self._update_versions()

    def _check_group(self):
        group_file = "%s/dblists/%s.dblist" % (self.config["stage_dir"], self.group)
        if not os.path.isfile(group_file):
            utils.abort("""group "%s" does not exist""" % group_file)

    def _prompt_user_to_approve(self, prev_version) -> bool:
        if self.arguments.train:
            # FIXME: The generated message isn't pretty
            prompt_message = "Set and deploy these versions:\n"
            v = self.promote_version
            for group in train.GROUPS:
                prompt_message += f"{group}: {v}\n"
                if group == self.group:
                    v = self.arguments.old_version

            prompt_message += "?"
        else:
            prompt_message = "Promote %s from %s to %s" % (
                self.group,
                prev_version,
                self.promote_version,
            )
        return self.prompt_user_for_confirmation(prompt_message)

    def _update_versions(self):
        self._set_messages()

        if self.arguments.train:
            args = ["all", self.arguments.old_version]
            for group in train.GROUPS:
                args += [group, self.promote_version]
                if group == self.group:
                    break
        else:
            args = [self.group, self.promote_version]

        self.scap_check_call(["update-wikiversions", "--no-check"] + args)
        self._create_version_update_patch()
        self._sync_versions()

    def _create_version_update_patch(self):
        with utils.cd(self.config["stage_dir"]):
            if self._commit_files():
                self.logger.info("Pushing versions update patch")
                self._push_patch()
                self.logger.info("Running git pull")
                gitcmd("pull")
                git.tag("scap-prep-point", "HEAD", force=True)

    def _set_messages(self):
        """
        Craft commit message and scap announcement message
        """
        header = "%s to %s" % (self.group, self.promote_version)
        self.commit_message = header
        self.announce_message = header

        try:
            train_info = self.get_current_train_info()
            phabricator_task_id = train_info["task"]
        except Exception as e:
            complaint = f"Failed to automatically retrieve the train task: {e}"
            if not interaction.interactive():
                utils.abort(complaint)

            self.logger.error(complaint)
            phabricator_task_id = self.input_line(
                "Please enter the train blocker Phabricator task id for log messages: "
            )
            if not phabricator_task_id:
                utils.abort("No train blocker task supplied.  Canceling")

        self.commit_message += "\n\nBug: %s" % phabricator_task_id
        self.announce_message += "  refs %s" % phabricator_task_id

    def _commit_files(self) -> bool:
        """
        Returns True if a commit was created, False if not.
        """
        versions_file = utils.get_realm_specific_filename(
            "wikiversions.json", self.config["wmf_realm"]
        )
        files_to_commit = [
            file
            for file in [versions_file, "php"]
            if git.file_has_unstaged_changes(file)
        ]

        if not files_to_commit:
            return False

        gitcmd("add", *files_to_commit)
        gitcmd("commit", "-m", self.commit_message)
        return True

    def _push_patch(self):
        gitcmd(
            "push",
            "origin",
            "HEAD:%s" % self._get_git_push_dest(),
            env=self.get_gerrit_ssh_env(),
        )

        change_id = re.search(r"(?m)Change-Id:.+$", gitcmd("log", "-1")).group()
        gitcmd("reset", "--hard", "HEAD^")
        with self.reported_status("Waiting for jenkins to merge the patch", log=True):
            timeout = self.config["version_update_patch_timeout"]
            start = time.time()
            while not _commit_arrived_to_remote(change_id):
                if time.time() - start > timeout:
                    utils.abort(
                        f"Waited for {timeout} seconds but the patch was not merged"
                    )
                if interaction.have_terminal():
                    print(".", end="", flush=True)
                time.sleep(5)
        print()

    def _get_git_push_dest(self) -> str:
        branch = gitcmd("symbolic-ref", "--short", "HEAD").strip()
        return "refs/for/%s%%topic=%s,l=Code-Review+2" % (branch, self.promote_version)

    def _sync_versions(self):
        if self.group == "testwikis":
            self.logger.info("Running scap prep auto")
            self.scap_check_call(["prep", "auto"])
            self.logger.info("Running scap sync-world")
            self.scap_check_call(["sync-world", self.announce_message])
        else:
            self.logger.info("Running scap sync-wikiversions")
            self.scap_check_call(["sync-wikiversions", self.announce_message])

        # Group1 day is also the day we sync the php symlink
        if self.config["manage_mediawiki_php_symlink"] and self.group == "group1":
            self.logger.info("Running scap sync-file php")
            self.scap_check_call(["sync-file", "php", self.announce_message])

        self._check_versions()

    def _check_versions(self):
        check_url = self._get_check_url()

        polling_interval = 1  # seconds
        timeout = self._get_check_versions_timeout()

        deadline = time.time() + timeout

        while True:
            actual_version = self._get_special_version(check_url)
            self._notify_version_update_result(check_url, actual_version)

            if self.promote_version == actual_version:
                return

            if time.time() >= deadline:
                # Time ran out.
                utils.abort("Could not verify version update")

            time.sleep(polling_interval)

    # This is a method so that it can be patched during tests
    def _get_check_versions_timeout(self):
        return 10

    def _get_check_url(self) -> str:
        if self.group == "testwikis":
            check_domain = "test.wikipedia.org"
        elif self.group == "group0":
            check_domain = "www.mediawiki.org"
        elif self.group == "group1":
            check_domain = "en.wikinews.org"
        else:
            check_domain = "en.wikipedia.org"
        return "https://%s/wiki/Special:Version" % check_domain

    def _get_special_version(self, check_url) -> str:
        try:
            res = requests.get(check_url)
            res.raise_for_status()

            actual_version_match = re.search(
                r"(?i)MediaWiki (%s)" % BRANCH_RE_UNANCHORED.pattern, res.text
            )
            actual_version = (
                actual_version_match.group(1)
                if actual_version_match
                else "Version not found on checked page"
            )
        except HTTPError as e:
            actual_version = (
                f"Request to checked page failed with {e.response.status_code}"
            )
        except Exception as e:
            actual_version = f"{type(e).__name__} exception: {e}"

        return actual_version

    def _notify_version_update_result(self, check_url, actual_version):
        versions_match = self.promote_version == actual_version
        log = self.logger.info if versions_match else self.logger.error
        log(
            "==================================================\n"
            "Checking version on %s\n"
            "Expected: %s\n"
            "Actual:   %s\n"
            "Result:   %s\n"
            "==================================================",
            check_url,
            self.promote_version,
            actual_version,
            "SUCCESS" if versions_match else "FAIL",
        )


def _commit_arrived_to_remote(change_id) -> bool:
    gitcmd("fetch")
    return change_id in gitcmd("log", "HEAD..FETCH_HEAD")
