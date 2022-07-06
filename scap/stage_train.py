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
    * Cleans up out-of-date branches

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

from scap import cli, utils

STAGE_SEQUENCE = [
    "prep",
    "patch",
    "testwikis",
    "clean",
]


@cli.command(
    "stage-train",
    help="Wraps manual steps required to stage a typical Tuesday deploy of the RelEng train"
         " deployment: Preps new version branch, applies patches and deploys to testwikis"
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
        help="Wikiversion needing staging.  Specify 'auto' to use the latest available wmf branch"
    )
    @cli.argument(
        "--start-from",
        choices=STAGE_SEQUENCE,
        default=STAGE_SEQUENCE[0],
        help="stage to start from"
    )
    @cli.argument(
        "--dry-run",
        action="store_true",
        help="dry run"
    )
    @cli.argument(
        "-y", "--yes",
        action="store_true",
        help="default to Yes for prompts, and don't check for tmux or screen running."
    )
    def main(self, *extra_args):
        def check_term_multplxr():
            if os.environ.get("TMUX") is None and os.environ.get("STY") is None:
                utils.abort(
                    "No tmux or screen process found. Try either:\n"
                    '\tTMUX: tmux new-session -s "🚂🌈"\n'
                    "\tSCREEN: screen -D -RR train"
                )

        if self.arguments.version == "auto":
            self.arguments.version = utils.get_current_train_version(self.config["gerrit_url"])
            self.logger.info("Using version %s", self.arguments.version)

        if not self.arguments.yes:
            check_term_multplxr()
        self._check_user_auth_sock()

        # Ensure that all files created by this operation are group writable.
        os.umask(0o002)

        stages = STAGE_SEQUENCE[STAGE_SEQUENCE.index(self.arguments.start_from):]
        for i, stage in enumerate(stages, start=1):
            self.logger.info(
                "----------------------------\n"
                "%s. Starting: %s" % (i, stage)
            )
            getattr(self, "_" + stage)()
            self.logger.info("DONE!")

    def _prep(self):
        self._run(["prep", self.arguments.version])

    def _patch(self):
        self._run(["apply-patches", "--abort-git-am-on-fail", "--train", self.arguments.version])

    def _testwikis(self):
        self._run(["deploy-promote", "--yes", "testwikis", self.arguments.version])

    def _clean(self):
        self._run(["clean", "auto"])

    def _run(self, scap_cmd):
        scap_cmd_str = " ".join(scap_cmd)
        cmd_run_approved = self.arguments.yes or utils.confirm("Run `%s` now?" % scap_cmd_str)

        if cmd_run_approved:
            if self.arguments.dry_run:
                self.logger.info("Skipping `%s`, dry run only..." % scap_cmd_str)
                return

            self.scap_check_call(scap_cmd)
