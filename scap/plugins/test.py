# -*- coding: utf-8 -*-
"""
    scap.plugins.test
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

import scap.cli as cli
import scap.log as log


@cli.command('test-progress')
class Test(cli.Application):
    """ Display a mock progress bar """
    @cli.argument('--steps', default='25', nargs='?')
    def main(self, *extra_args):
        steps = int(self.arguments.steps)
        stages = ['Testing Stage Number 1', 'This is Stage 2', 'Just Stage 3']
        for stage in stages:
            reporter = log.FancyProgressReporter(name=stage,
                                                 expect=steps)
            logger = logging.getLogger()
            reporter.start()
            for i in range(0, steps):
                rand = random.randrange(0, 30, 2)
                if rand == 0:
                    reporter.add_failure()
                    logger.warn('Fail: %s of %s', i, steps)
                else:
                    reporter.add_success()
                    logger.info('Success: %s of %s', i, steps)

                time.sleep(0.2)

            reporter.finish()
