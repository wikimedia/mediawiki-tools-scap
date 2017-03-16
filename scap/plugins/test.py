import logging
import time
import random

from scap import cli, log


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
                    logger.warn('Fail: %s of %s' % (i, steps))
                else:
                    reporter.add_success()
                    logger.info('Success: %s of %s' % (i, steps))

                time.sleep(0.2)

            reporter.finish()
