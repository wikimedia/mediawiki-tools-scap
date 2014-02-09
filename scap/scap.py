# -*- coding: utf-8 -*-
"""
    scap.scap
    ~~~~~~~~~
    This module contains the actual procedure for deploying MediaWiki.

"""
import argparse
import logging
import os
import random
import subprocess
import sys
import tempfile
import time

from . import log
from . import utils


logger = logging.getLogger('scap')


def parse_args():
    """Parse command-line arguments."""
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

    args.cfg = utils.get_config(args.conf_file)
    if args.versions and 'active' in args.versions:
        args.versions.remove('active')
        args.versions.update(utils.get_branches(
            '%(MW_COMMON)s/wikiversions.cdb' % args.cfg))
    return args


def main(args):
    """Core business logic of scap process."""
    env = {}
    if args.versions:
        env['MW_VERSIONS_SYNC'] = ' '.join(args.versions)

    with utils.lock('/var/lock/scap'):
        logger.info('started scap: %s', args.message)

        logger.debug('Checking syntax')
        utils.check_syntax('%(MW_COMMON_SOURCE)s/wmf-config' % args.cfg)
        utils.check_syntax('%(MW_COMMON_SOURCE)s/multiversion' % args.cfg)

        # Update the current machine so that serialization works. Push
        # wikiversions.dat changes so mwversionsinuse, set-group-write,
        # and mwscript work with the right version of the files.
        subprocess.check_call('/usr/local/bin/sync-common')

        # Update list of extension message files and regenerate the
        # localisation cache.
        subprocess.check_call('/usr/local/bin/mw-update-l10n')

        logger.debug('updating rsync proxies')
        utils.dsh('/usr/local/bin/scap-1', 'scap-proxies', env)

        scap_proxies = utils.read_dsh_hosts_file('scap-proxies')

        # Randomize the order of target machines.
        mw_install_hosts = utils.read_dsh_hosts_file('mediawiki-installation')
        random.shuffle(mw_install_hosts)
        with tempfile.NamedTemporaryFile(delete=False, prefix='scap') as tmp:
            try:
                tmp.write('\n'.join(mw_install_hosts))
                tmp.flush()

                logger.debug('copying code to Apaches')
                utils.dsh('/usr/local/bin/scap-1 %s' % ' '.join(scap_proxies),
                    tmp.name, env)

                logger.debug('rebuilding CDB files')
                utils.dsh('/usr/local/bin/scap-rebuild-cdbs', tmp.name, env)
            finally:
                os.remove(tmp.name)

        logger.debug('building wikiversions.cdb')
        subprocess.check_call('%(MW_COMMON_SOURCE)s/multiversion/'
                              'refreshWikiversionsCDB' % args.cfg)
        utils.dsh('sudo -u mwdeploy rsync -l %(MW_RSYNC_HOST)s::common/'
            'wikiversions.{dat,cdb} %(MW_COMMON)s' % args.cfg,
            'mediawiki-installation')


def scap():
    """Deploy MediaWiki code and configuration."""
    assert 'SSH_AUTH_SOCK' in os.environ, 'scap requires SSH agent forwarding'
    start = time.time()
    args = None
    try:
        args = parse_args()
        main(args)
        duration = time.time() - start

        logger.info('finished scap: %s (duration: %s)',
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

    except:
        # Handle all unhandled exceptions and errors
        duration = time.time() - start
        exctype, value = sys.exc_info()[:2]
        logger.error('scap failed: %s %s (duration: %s)',
            exctype.__name__, value, utils.human_duration(duration))
        return 1

    finally:
        if args:
            stats = log.Stats(
                args.cfg['MW_STATSD_HOST'], args.cfg['MW_STATSD_PORT'])
            stats.increment('scap.scap')
            stats.timing('scap.scap', duration * 1000)
