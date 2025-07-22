# -*- coding: utf-8 -*-
"""
    scap.stage_train
    ~~~~~~~~
    Wrapper for all the manual steps performed before the deploy window on a Tuesday during a
    typical RelEng train deployment.

    Example usage: scap stage-train 1.38.0-wmf.20

    The above command:
    * Ensures you're in a tmux window
    * Ensures you've got a running ssh-agent
    * Checks for the new branch
    * Preps the new branch
    * Applies patches to the new branch
    * Deploys the new branch to testwikis

    The behavior associated to stage-train used to live in the `tools/release`. The file history is
    still available in that repo and can be viewed with:
        git log -- bin/stage-train

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
import sys

from scap import cli, utils

STAGE_SEQUENCE = [
    "prep",
    "patch",
    "testwikis",
]


@cli.command(
    "stage-train",
    help="Wraps manual steps required to stage a typical Tuesday deploy of the RelEng train"
    " deployment: Preps new version branch, applies patches and deploys to testwikis",
    primary_deploy_server_only=True,
    require_tty_multiplexer=True,
)
class StageTrain(cli.Application):
    """
    Scap sub-command to stage a typical Tuesday deploy of the RelEng train deployment
    """

    def __init__(self, exe_name):
        super().__init__(exe_name)

        self.logger = self.get_logger()

    @cli.argument(
        "version",
        help="Wikiversion needing staging.  Specify 'auto' to use the latest available wmf branch",
    )
    @cli.argument(
        "--start-from",
        choices=STAGE_SEQUENCE,
        default=STAGE_SEQUENCE[0],
        help="stage to start from",
    )
    @cli.argument("--dry-run", action="store_true", help="dry run")
    @cli.argument(
        "-y",
        "--yes",
        action="store_true",
        help="default to Yes for prompts",
    )
    def main(self, *extra_args):
        if self.arguments.version == "auto":
            self._setup_auto_mode()

        self._check_user_auth_sock()

        os.umask(self.config["umask"])

        stages = STAGE_SEQUENCE[STAGE_SEQUENCE.index(self.arguments.start_from) :]
        for i, stage in enumerate(stages, start=1):
            self.logger.info(
                "----------------------------\n" "%s. Starting: %s" % (i, stage)
            )
            getattr(self, "_" + stage)()
            self.logger.info("DONE!")

    # Stage implementations
    def _prep(self):
        self._run(["prep", self.arguments.version])

    def _patch(self):
        self._run(
            [
                "apply-patches",
                "--abort-git-am-on-fail",
                "--train",
                self.arguments.version,
            ]
        )

    def _testwikis(self):
        self._run(["deploy-promote", "--yes", "testwikis", self.arguments.version])

    # End stage implementations

    def _run(self, scap_cmd):
        scap_cmd_str = " ".join(scap_cmd)
        cmd_run_approved = self.arguments.yes or self.prompt_user_for_confirmation(
            "Run `%s` now?" % scap_cmd_str
        )

        if cmd_run_approved:
            if self.arguments.dry_run:
                self.logger.info("Skipping `%s`, dry run only..." % scap_cmd_str)
                return

            self.scap_check_call(scap_cmd)

    def _setup_auto_mode(self):
        self.logger.info("Initializing stage-train auto mode")
        self.logger.info("Retrieving train information...")
        gerrit_latest_version = utils.get_current_train_version_from_gerrit(
            self.config["gerrit_url"]
        )

        train_info = self.get_current_train_info()
        task = train_info["task"]
        status = train_info["status"]
        version = train_info["version"]

        if status not in ["open", "progress"]:
            self.logger.warning(
                "Phabricator task %s has status '%s'.  Cancelling operation.",
                task,
                status,
            )
            sys.exit(0)

        if version != gerrit_latest_version:
            utils.abort(
                "Phabricator task {} says the train version is '{}', but '{}' is the latest available in Gerrit.".format(
                    task, version, gerrit_latest_version
                )
            )

        self.arguments.version = version
        self.logger.info("Using version %s", version)
