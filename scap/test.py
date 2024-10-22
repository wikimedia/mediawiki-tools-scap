# -*- coding: utf-8 -*-
"""
    scap.test
    ~~~~~~~~~~~~~~~~~

    For ultimate testing of progress reporters

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
import logging
import time
import random
from unittest.mock import patch

from scap.train import TrainInfo, GROUPS
from scap import cli, backport
from scap import log


@cli.command("test-backport")
class TestBackport(cli.Application):
    """Display a mock backport prompt"""

    def main(self, *extra_args):
        table = backport.make_table(
            [
                {
                    "_number": 123412,
                    "project": "mediawiki/core",
                    "branch": "1.42.0-wmf.2",
                    "subject": "Fix up for foo",
                    "mergeable": True,
                },
                {
                    "_number": 123808,
                    "project": "operations/mediawiki-config",
                    "branch": "master",
                    "subject": "Update InitialiseSettings.php",
                    "mergeable": False,
                },
            ],
            True,
        )
        print(table.get_string(sortby="Project"))


@cli.command("test-progress")
class TestProgress(cli.Application):
    """Display a mock progress bar"""

    @cli.argument("--steps", default="25", nargs="?")
    def main(self, *extra_args):
        steps = int(self.arguments.steps)
        stages = ["Testing Stage Number 1", "This is Stage 2", "Just Stage 3"]
        for stage in stages:
            reporter = log.ProgressReporter(name=stage, expect=steps)
            logger = logging.getLogger()
            reporter.start()
            for i in range(0, steps):
                reporter.add_in_flight()
                time.sleep(0.2)
                rand = random.randrange(0, 30, 2)
                if rand == 0:
                    reporter.add_failure()
                    logger.warning("Fail: %s of %s", i, steps)
                else:
                    reporter.add_success()
                    logger.info("Success: %s of %s", i, steps)

            reporter.finish()


@cli.command("test-train")
class TestTrain(cli.Application):
    """Display a mock of train stations"""

    @patch.object(TrainInfo, "get_train_version", return_value="v2")
    @cli.argument("--at", choices=GROUPS)
    def main(self, x, y, *extra_args):
        def groups(group, *_):
            # Train is at the starting point
            if self.arguments.at is None:
                return ["v1"]

            if GROUPS.index(self.arguments.at) >= GROUPS.index(group):
                return ["v2"]

            return ["v1"]

        with patch("scap.utils.get_group_versions", side_effect=groups):
            TrainInfo({"stage_dir": "", "wmf_realm": ""}).visualize()
