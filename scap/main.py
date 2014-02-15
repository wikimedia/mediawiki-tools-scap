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

from . import log
from . import tasks
from . import utils


def sync_common():
    """Command wrapper for :func:`tasks.sync_common`

    :returns: Integer exit status suitable for use with ``sys.exit``
    """
    logger = logging.getLogger('sync_common')
    try:

        parser = argparse.ArgumentParser(description='Sync MW_COMMON')
        parser.add_argument('-c', '--conf',
            dest='conf_file', default='/usr/local/lib/mw-deployment-vars.sh',
            help='Path to configuration file')
        parser.add_argument('servers', nargs=argparse.REMAINDER,
            help='Rsync server(s) to copy from')
        args = parser.parse_args()
        args.cfg = None

        args.cfg = utils.get_config(args.conf_file)

        tasks.sync_common(args.cfg, args.servers)
        return 0

    except SystemExit:
        # Triggered by sys.exit() calls
        raise

    except KeyboardInterrupt:
        # Handle ctrl-c from interactive user
        logger.warning('sync_common aborted')
        return 1

    except Exception as ex:
        # Handle all unhandled exceptions and errors
        logger.debug('Unhandled error:', exc_info=True)
        logger.error('sync_common failed: <%s> %s', type(ex).__name__, ex)
        return 1


def scap():
    """Command wrapper for :func:`tasks.scap`

    :returns: Integer exit status suitable for use with ``sys.exit``
    """
    start = time.time()
    logger = logging.getLogger('scap')
    args = None
    try:
        assert 'SSH_AUTH_SOCK' in os.environ, \
            'scap requires SSH agent forwarding'
        parser = argparse.ArgumentParser(description='Deploy MediaWiki')
        parser.add_argument('--versions',
            type=lambda v: set(v.split()),
            help='Sync only specifed versions')
        parser.add_argument('-c', '--conf',
            dest='conf_file', default='/usr/local/lib/mw-deployment-vars.sh',
            help='Path to configuration file')
        parser.add_argument('message', nargs=argparse.REMAINDER,
            help='Log message for SAL')
        args = parser.parse_args()
        args.message = ' '.join(args.message) or '(no message)'
        args.cfg = None

        args.cfg = utils.get_config(args.conf_file)
        if args.versions and 'active' in args.versions:
            args.versions.remove('active')
            args.versions.update(utils.get_branches(
                '%(MW_COMMON)s/wikiversions.cdb' % args.cfg))

        tasks.scap(args)
        duration = time.time() - start

        logger.info('Finished scap: %s (duration: %s)',
            args.message, utils.human_duration(duration))
        return 0

    except SystemExit:
        # Triggered by sys.exit() calls
        raise

    except KeyboardInterrupt:
        # Handle ctrl-c from interactive user
        duration = time.time() - start
        msg = args.message if args else ''
        logger.warning('scap aborted: %s (duration: %s)',
            msg, utils.human_duration(duration))
        return 1

    except Exception as ex:
        # Handle all unhandled exceptions and errors
        duration = time.time() - start
        logger.debug('Unhandled error:', exc_info=True)
        logger.error('scap failed: %s %s (duration: %s)',
            type(ex).__name__, ex, utils.human_duration(duration))
        return 1

    finally:
        if args and args.cfg:
            stats = log.Stats(
                args.cfg['MW_STATSD_HOST'], args.cfg['MW_STATSD_PORT'])
            stats.increment('scap.scap')
            stats.timing('scap.scap', duration * 1000)
