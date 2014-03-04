# -*- coding: utf-8 -*-
"""
    scap.main
    ~~~~~~~~~~
    Command wrappers for scap tasks

"""
import argparse
import os
import time

from . import cli
from . import log
from . import tasks
from . import utils


class MWVersionsInUse(cli.Application):
    """Get a list of the active MediaWiki versions."""

    @cli.argument('--withdb', action='store_true',
        help='Add `=wikidb` with some wiki using the version.')
    def main(self, *extra_args):
        versions = utils.wikiversions(self.config['deploy_dir'])

        if self.arguments.withdb:
            output = ['%s=%s' % (version, wikidb)
                    for version, wikidb in versions.items()]
        else:
            output = [str(version) for version in versions.keys()]

        print ' '.join(output)
        return 0

    def _process_arguments(self, args, extra_args):
        """Log warnings about unexpected arguments but don't exit."""
        if extra_args:
            self.logger.warning(
                'Unexpected argument(s) ignored: %s', extra_args)

        return args, extra_args


class SyncCommon(cli.Application):
    """Sync local MediaWiki deployment directory with deploy server state."""

    @cli.argument('servers', nargs=argparse.REMAINDER,
        help='Rsync server(s) to copy from')
    def main(self, *extra_args):
        tasks.sync_common(self.config, self.arguments.servers)
        return 0


class Scap(cli.Application):
    """Deploy MediaWiki to the cluster."""

    def __init__(self, exe_name):
        super(self.__class__, self).__init__(exe_name)
        self.start = time.time()

    def _process_arguments(self, args, extra_args):
        args.message = ' '.join(args.message) or '(no message)'
        return args, extra_args

    @cli.argument('message', nargs=argparse.REMAINDER,
        help='Log message for SAL')
    def main(self, *extra_args):
        assert 'SSH_AUTH_SOCK' in os.environ, \
            'scap requires SSH agent forwarding'

        tasks.scap(self.arguments.message, self.config)

        self.logger.info('Finished scap: %s (duration: %s)',
            self.arguments.message, self.human_duration)
        return 0

    @property
    def duration(self):
        """Get the elapsed duration in seconds."""
        return time.time() - self.start

    @property
    def human_duration(self):
        """Get the elapsed duration in human readable form."""
        return utils.human_duration(self.duration)

    def _handle_keyboard_interrupt(self, ex):
        self.logger.warning('scap aborted: %s (duration: %s)',
            self.arguments.message, self.human_duration)
        return 1

    def _handle_exception(self, ex):
        self.logger.debug('Unhandled error:', exc_info=True)
        self.logger.error('scap failed: %s %s (duration: %s)',
            type(ex).__name__, ex, self.human_duration)
        return 1

    def _before_exit(self, exit_status):
        if self.config:
            stats = log.Stats(
                self.config['statsd_host'], int(self.config['statsd_port']))
            stats.increment('scap.scap')
            stats.timing('scap.scap', self.duration * 1000)
