# -*- coding: utf-8 -*-
"""
    scap.main
    ~~~~~~~~~~
    Command wrappers for scap tasks

"""
import argparse
import logging
import os
import time

from . import cli
from . import config
from . import log
from . import tasks
from . import utils


def get_argparser(*args, **kwargs):
    """Get an argparse.ArgumentParser instance with common default arguments
    added."""
    parser = argparse.ArgumentParser(*args, **kwargs)
    parser.add_argument('-c', '--conf', dest='conf_file',
        type=argparse.FileType('r'),
        help='Path to configuration file')
    parser.add_argument('-D', '--define', dest='defines', action='append',
            type=lambda v: tuple(v.split(':')),
        help='Set a configuration value', metavar='<name>:<value>')
    return parser


def load_config(args):
    defines = None
    if args.defines:
        defines = dict(args.defines)
    cfg = config.load(args.conf_file, defines)
    return cfg


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


def scap():
    """Command wrapper for :func:`tasks.scap`

    :returns: Integer exit status suitable for use with ``sys.exit``
    """
    start = time.time()
    logger = logging.getLogger('scap')
    message = '(no message)'
    cfg = None
    try:
        parser = get_argparser(description='Deploy MediaWiki',
                epilog="Note: a ssh-agent is required to run this script.")
        parser.add_argument('message', nargs=argparse.REMAINDER,
            help='Log message for SAL')
        args = parser.parse_args()

        assert 'SSH_AUTH_SOCK' in os.environ, \
            'scap requires SSH agent forwarding'

        if args.message:
            message = ' '.join(args.message)

        cfg = load_config(args)

        tasks.scap(message, cfg)
        duration = time.time() - start

        logger.info('Finished scap: %s (duration: %s)',
            message, utils.human_duration(duration))
        return 0

    except SystemExit:
        # Triggered by sys.exit() calls
        raise

    except KeyboardInterrupt:
        # Handle ctrl-c from interactive user
        duration = time.time() - start
        logger.warning('scap aborted: %s (duration: %s)',
            message, utils.human_duration(duration))
        return 1

    except Exception as ex:
        # Handle all unhandled exceptions and errors
        duration = time.time() - start
        logger.debug('Unhandled error:', exc_info=True)
        logger.error('scap failed: %s %s (duration: %s)',
            type(ex).__name__, ex, utils.human_duration(duration))
        return 1

    finally:
        if cfg:
            stats = log.Stats(cfg['statsd_host'], int(cfg['statsd_port']))
            stats.increment('scap.scap')
            stats.timing('scap.scap', duration * 1000)
