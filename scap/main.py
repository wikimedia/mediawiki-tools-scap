# -*- coding: utf-8 -*-
"""
    scap.main
    ~~~~~~~~~~
    Command wrappers for scap tasks

"""
import argparse
import logging
import sys

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

    except:
        # Handle all unhandled exceptions and errors
        exctype, value = sys.exc_info()[:2]
        logger.error('sync_common failed: <%s> %s', exctype.__name__, value)
        return 1
