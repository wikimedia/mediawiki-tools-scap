import logging
import time

from scap import cli, log


@cli.command('test')
class Test(cli.Application):

    @cli.argument('--steps', default='150', nargs='?')
    def main(self, *extra_args):
        steps = int(self.arguments.steps)
        reporter = log.ProgressReporter(name='TestProgress',
                                        expect=steps)
        logger = logging.getLogger()
        reporter.start()
        for i in range(0, steps):
            reporter.add_success()
            logger.info('success: %s of %s' % (i, steps))
            time.sleep(0.2)

        reporter.finish()
