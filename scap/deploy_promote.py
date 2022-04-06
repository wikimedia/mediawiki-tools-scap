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
import json
import os
import re
import socket
import time
from functools import partial
from json import JSONDecodeError

import requests
from requests import RequestException, HTTPError

from scap import cli, utils, config
from scap.runcmd import gitcmd

print = partial(print, flush=True)


@cli.command(
    "deploy-promote",
    help="Promote group of wikis to specific/latest wmf deployment branch"
)
class DeployPromote(cli.Application):
    """
    Scap sub-command to promote a specified group of wikis to a specific/latest wmf deployment branch
    """

    logger = None

    group = None
    promote_version = None
    update_message = None

    @cli.argument(
        "group",
        help="existing group to which you'd like to deploy a new version (testwikis, group0, group1"
             ", or all)."
    )
    @cli.argument(
        "version",
        nargs="?",
        default=None,
        help="version to deploy (ex: 1.36.0-wmf.2). Defaults to the latest version found in"
             ' "<stage_dir>/<wikiversions_filename>". By default "<stage_dir>" is'
             " " + config.DEFAULT_CONFIG["stage_dir"][1],
    )
    @cli.argument(
        "-y", "--yes",
        action="store_true",
        help="answer yes to all prompts"
    )
    def main(self, *extra_args):
        self.logger = self.get_logger()

        self.group = self.arguments.group
        self._check_group()
        self._check_user_auth_sock()

        sorted_versions = self.active_wikiversions("stage")
        prev_version = sorted_versions[0]
        if self.arguments.version:
            self.promote_version = self.arguments.version
        else:
            if len(sorted_versions) < 2:
                utils.abort(
                    "Cannot determine version to promote to. Current active version: %s"
                    % prev_version
                )
            self.promote_version = sorted_versions[1]

        if not self.arguments.yes and not self._prompt_user_to_approve(prev_version):
            utils.abort("Canceled by user")

        self._update_versions()

    def _check_group(self):
        group_file = "%s/dblists/%s.dblist" % (self.config["stage_dir"], self.group)
        if not os.path.isfile(group_file):
            utils.abort("""group "%s" does not exist""" % group_file)

    def _get_train_task(self) -> str:
        """
        Returns the Phabricator id of the current train task.  If this
        information cannot be retrieved or decoded, or the response doesn't look
        like a Phabricator task id, an empty string is returned.
        """

        api_url = "https://train-blockers.toolforge.org/api.php"

        self.logger.info('Trying to retrieve Phabricator train task from "%s"' % api_url)
        try:
            proxy = {"https": "http://webproxy:8080"} if _on_real_deploy_server() else None
            train_task_json = requests.get(api_url, proxies=proxy).text
        except RequestException as e:
            self.logger.warning("Failed to retrieve Phabricator train task:\n%s" % str(e))
            return ""

        try:
            task = json.loads(train_task_json)["current"]["task_id"]
        except (JSONDecodeError, TypeError) as e:
            self.logger.warning("Invalid JSON received from %s:\n%s" % (api_url, e))
            return ""

        if not utils.is_phabricator_task_id(task):
            self.logger.warning(
                "Unexpected Phabricator train task format received from %s: %s" % (api_url, task)
            )
            return ""

        self.logger.info("Phabricator task id is %s" % task)
        return task

    def _prompt_user_to_approve(self, prev_version) -> bool:
        prompt_message = "Promote %s from %s to %s" % (
            self.group, prev_version, self.promote_version
        )
        return utils.prompt_user_for_confirmation(prompt_message)

    def _update_versions(self):
        self._set_update_message()

        self.scap_check_call(
            ["update-wikiversions", "--no-check", self.group, self.promote_version]
        )
        self._create_version_update_patch()
        self._sync_versions()

    def _create_version_update_patch(self):
        with utils.cd(self.config["stage_dir"]):
            if self._commit_files():
                self.logger.info("Pushing versions update patch")
                self._push_patch()
                self.logger.info("Running git pull")
                gitcmd("pull")

    def _set_update_message(self):
        phabricator_task_id = self._get_train_task()
        # Extra whitespace probably unnecessary; left in to mimic the behavior of the original
        # shellscript implementation of `deploy-promote`
        task_id_message = "  refs %s" % phabricator_task_id if phabricator_task_id else ""
        self.update_message = \
            "%s wikis to %s%s" % (self.group, self.promote_version, task_id_message)

    def _commit_files(self) -> bool:
        """
        Returns True if a commit was created, False if not.
        """
        versions_file = \
            utils.get_realm_specific_filename("wikiversions.json", self.config["wmf_realm"])
        files_to_commit = [file for file in [versions_file, "php"] if _file_updated(file)]

        if not files_to_commit:
            return False

        gitcmd("add", *files_to_commit)
        gitcmd("commit", "-m", self.update_message)
        return True

    def _push_patch(self):
        # Overriding "SSH_AUTH_SOCK" is required until we implement a long-term solution for
        # https://phabricator.wikimedia.org/T304557. At which point we can remove the override
        user_env = os.environ.copy()
        user_env["SSH_AUTH_SOCK"] = self.user_ssh_auth_sock
        gitcmd("push", "origin", "HEAD:%s" % self._get_git_push_dest(), env=user_env)

        change_id = re.search(r"(?m)Change-Id:.+$", gitcmd("log", "-1")).group()
        gitcmd("reset", "--hard", "HEAD^")
        self.logger.info("Waiting for jenkins to merge the patch")
        while not _commit_arrived_to_remote(change_id):
            print(".", end="")
            time.sleep(5)

    def _get_git_push_dest(self) -> str:
        branch = gitcmd("symbolic-ref", "--short", "HEAD").strip()
        return "refs/for/%s%%topic=%s,l=Code-Review+2" % (branch, self.promote_version)

    def _sync_versions(self):
        if self.group == "testwikis":
            self.logger.info("Running scap prep auto")
            self.scap_check_call(["prep", "auto"])
            self.logger.info("Running scap sync-world")
            self.scap_check_call(["sync-world", self.update_message])
        else:
            self.logger.info("Running scap sync-wikiversions")
            self.scap_check_call(["sync-wikiversions", self.update_message])

        # Group1 day is also the day we sync the php symlink
        if self.group == "group1":
            self.logger.info("Running scap sync-file php")
            self.scap_check_call(["sync-file", "php", self.update_message])

        self._check_versions()

    def _check_versions(self):
        if self.group == "testwikis":
            check_domain = "test.wikipedia.org"
        elif self.group == "group0":
            check_domain = "www.mediawiki.org"
        elif self.group == "group1":
            check_domain = "en.wikinews.org"
        else:
            check_domain = "en.wikipedia.org"
        check_url = "https://%s/wiki/Special:Version" % check_domain

        try:
            res = requests.get(check_url)
            res.raise_for_status()

            actual_version_match = \
                re.search(r'<meta name="generator" content="MediaWiki (.+)"/>', res.text)
            actual_version = \
                actual_version_match.group(1) if actual_version_match \
                else "Version not found on checked page"
        except RequestException as e:
            actual_version = \
                "Request to checked page failed" \
                + (" with %s" % e.response.status_code if isinstance(e, HTTPError) else "")
        self._notify_version_update_result(check_url, actual_version)

    def _notify_version_update_result(self, check_url, actual_version):
        self.logger.info(
            "==================================================\n"
            "Checking version on %s\n"
            "Expected: %s\n"
            "Actual:   %s\n"
            "Result:   %s\n"
            "==================================================",
            check_url,
            self.promote_version,
            actual_version,
            "SUCCESS" if self.promote_version == actual_version else "FAIL"
        )


def _on_real_deploy_server() -> bool:
    return socket.getfqdn().endswith(".wmnet")


def _file_updated(file) -> bool:
    git_status_output = gitcmd("status", "--porcelain", file)
    return bool(re.search(r'(?m)^ M', git_status_output))


def _commit_arrived_to_remote(change_id) -> bool:
    gitcmd("fetch")
    return change_id in gitcmd("log", "HEAD..FETCH_HEAD")
