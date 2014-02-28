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


def mwversionsinuse():
    """Get a list of the active MediaWiki versions.

    :returns: Integer exit status suitable for use with ``sys.exit``
    """
    logger = logging.getLogger('wikiversions')
    try:
        parser = argparse.ArgumentParser(
            description='Get a list of the active MW versions')
        parser.add_argument('-c', '--conf',
            dest='conf_file', default='/usr/local/lib/mw-deployment-vars.sh',
            help='Path to configuration file')
        parser.add_argument('--withdb', action='store_true',
            help='Add `=wikidb` with some wiki using the version.')
        args, unexpected = parser.parse_known_args()

        args.cfg = None
        args.cfg = utils.get_config(args.conf_file)

        if unexpected:
            logger.warning('Unexpected argument(s) ignored: %s', unexpected)

        versions = utils.wikiversions(args.cfg['MW_COMMON'])

        if args.withdb:
            output = ['%s=%s' % (version, wikidb)
                    for version, wikidb in versions.items()]
        else:
            output = [str(version) for version in versions.keys()]

        print ' '.join(output)
        return 0

    except SystemExit:
        # Triggered by sys.exit() calls
        raise

    except KeyboardInterrupt:
        # Handle ctrl-c from interactive user
        if logger:
            logger.warning('wikiversions aborted')
        return 1

    except Exception as ex:
        # Handle all unhandled exceptions and errors
        if logger:
            logger.debug('Unhandled error:', exc_info=True)
            logger.error('wikiversions failed: <%s> %s', type(ex).__name__, ex)
        return 1


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
        parser = argparse.ArgumentParser(description='Deploy MediaWiki',
                epilog="Note: a ssh-agent is required to run this script.")
        parser.add_argument('-c', '--conf',
            dest='conf_file', default='/usr/local/lib/mw-deployment-vars.sh',
            help='Path to configuration file')
        parser.add_argument('message', nargs=argparse.REMAINDER,
            help='Log message for SAL')
        args = parser.parse_args()

        assert 'SSH_AUTH_SOCK' in os.environ, \
            'scap requires SSH agent forwarding'

        args.message = ' '.join(args.message) or '(no message)'
        args.cfg = None

        args.cfg = utils.get_config(args.conf_file)

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
