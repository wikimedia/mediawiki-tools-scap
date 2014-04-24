# -*- coding: utf-8 -*-
"""
    scap.main
    ~~~~~~~~~~
    Command wrappers for scap tasks

"""
import argparse
import multiprocessing
import os
import subprocess
import time

from . import cli
from . import log
from . import ssh
from . import tasks
from . import utils


class CompileWikiversions(cli.Application):
    """Compile wikiversions.json to wikiversions.cdb."""

    def main(self, *extra_args):
        self._run_as('mwdeploy')
        self._assert_current_user('mwdeploy')
        tasks.compile_wikiversions_cdb('deploy', self.config)
        return 0


class MWVersionsInUse(cli.Application):
    """Get a list of the active MediaWiki versions."""

    @cli.argument('--withdb', action='store_true',
        help='Add `=wikidb` with some wiki using the version.')
    def main(self, *extra_args):
        versions = self.active_wikiversions()

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


class PurgeL10nCache(cli.Application):
    """Purge the localization cache for an inactive MediaWiki version."""

    @cli.argument('-v', '--version', required=True,
        help='MediaWiki version (eg 1.23wmf16)')
    def main(self, *extra_args):
        if self.arguments.version.startswith('php-'):
            self.arguments.version = self.arguments.version[4:]

        if self.arguments.version in self.active_wikiversions():
            self.logger.error('Version %s is in use' % self.arguments.version)
            return 1

        tasks.purge_l10n_cache(self.arguments.version, self.config)
        self.announce('Purged l10n cache for %s' % self.arguments.version)
        return 0


class RebuildCdbs(cli.Application):
    """Rebuild localization cache CDB files from the JSON versions."""

    def main(self, *extra_args):
        self._run_as('mwdeploy')
        self._assert_current_user('mwdeploy')

        # Leave some of the cores free for apache processes
        use_cores = multiprocessing.cpu_count() / 2

        # Rebuild the CDB files from the JSON versions
        for version, wikidb in self.active_wikiversions().items():
            cache_dir = os.path.join(self.config['deploy_dir'],
                'php-%s' % version, 'cache', 'l10n')
            tasks.merge_cdb_updates(cache_dir, use_cores, True)


class SyncCommon(cli.Application):
    """Sync local MediaWiki deployment directory with deploy server state."""

    @cli.argument('servers', nargs=argparse.REMAINDER,
        help='Rsync server(s) to copy from')
    def main(self, *extra_args):
        tasks.sync_common(self.config, self.arguments.servers)
        return 0


class SyncWikiversions(cli.Application):
    """Rebuild and sync wikiversions.cdb to the cluster."""

    def _process_arguments(self, args, extra_args):
        args.message = ' '.join(args.message) or '(no message)'
        return args, extra_args

    @cli.argument('message', nargs='*', help='Log message for SAL')
    def main(self, *extra_args):
        self._assert_auth_sock()

        mw_install_hosts = utils.read_dsh_hosts_file('mediawiki-installation')
        tasks.sync_wikiversions(mw_install_hosts, self.config)

        self.announce(
            'rebuilt wikiversions.cdb and synchronized wikiversions files: %s',
            self.arguments.message)


class Scap(cli.Application):
    """Deploy MediaWiki to the cluster."""

    def __init__(self, exe_name):
        super(self.__class__, self).__init__(exe_name)
        self.start = time.time()

    def _process_arguments(self, args, extra_args):
        args.message = ' '.join(args.message) or '(no message)'
        return args, extra_args

    @cli.argument('-v', '--verbose', action='store_true',
        help='Verbose output')
    @cli.argument('message', nargs=argparse.REMAINDER,
        help='Log message for SAL')
    def main(self, *extra_args):
        """Core business logic of scap process.

        1. Validate php syntax of wmf-config and multiversion
        2. Sync deploy directory on localhost with staging area
        3. Compile wikiversions.json to cdb in deploy directory
        4. Update l10n files in staging area
        5. Ask scap proxies to sync with master server
        6. Ask apaches to sync with fastest rsync server
        7. Ask apaches to rebuild l10n CDB files
        8. Update wikiversions.cdb on localhost
        9. Ask apaches to sync wikiversions.cdb
        """
        self._assert_auth_sock()

        soft_errors = False
        with utils.lock(self.config['lock_file']):
            self.announce('Started scap: %s', self.arguments.message)

            tasks.check_php_syntax(
                '%(stage_dir)s/wmf-config' % self.config,
                '%(stage_dir)s/multiversion' % self.config)

            # Update the current machine so that serialization works. Push
            # wikiversions.json changes so mwversionsinuse, set-group-write,
            # and mwscript work with the right version of the files.
            tasks.sync_common(self.config)

            # Bug 63659: Compile deploy_dir/wikiversions.json to cdb
            subprocess.check_call('sudo -u mwdeploy -- '
                '/usr/local/bin/compile-wikiversions', shell=True)

            # Update list of extension message files and regenerate the
            # localisation cache.
            with log.Timer('mw-update-l10n', self.stats):
                for version, wikidb in self.active_wikiversions().items():
                    tasks.update_localization_cache(
                        version, wikidb, self.arguments.verbose, self.config)

            # Update rsync proxies
            scap_proxies = utils.read_dsh_hosts_file('scap-proxies')
            with log.Timer('sync-common to proxies', self.stats):
                update_proxies = ssh.Job(scap_proxies)
                update_proxies.command('/usr/local/bin/sync-common')
                update_proxies.progress('sync-common')
                succeeded, failed = update_proxies.run()
                if failed:
                    self.logger.warning(
                        '%d proxies had sync-common errors', failed)
                    soft_errors = True

            # Update apaches
            mw_install_hosts = utils.read_dsh_hosts_file(
                'mediawiki-installation')
            with log.Timer('update apaches', self.stats) as t:
                update_apaches = ssh.Job(mw_install_hosts)
                update_apaches.exclude_hosts(scap_proxies)
                update_apaches.shuffle()
                update_apaches.command(
                    ['/usr/local/bin/sync-common'] + scap_proxies)
                update_apaches.progress('sync-common')
                succeeded, failed = update_apaches.run()
                if failed:
                    self.logger.warning(
                        '%d apaches had sync-common errors', failed)
                    soft_errors = True
                t.mark('sync-common to apaches')

                rebuild_cdbs = ssh.Job(mw_install_hosts)
                rebuild_cdbs.command(
                    'sudo -u mwdeploy -n -- /usr/local/bin/scap-rebuild-cdbs')
                rebuild_cdbs.progress('scap-rebuild-cdbs')
                succeeded, failed = rebuild_cdbs.run()
                if failed:
                    self.logger.warning(
                        '%d hosts had scap-rebuild-cdbs errors', failed)
                    soft_errors = True
                t.mark('scap-rebuild-cdbs')

            # Update and sync wikiversions.cdb
            succeeded, failed = tasks.sync_wikiversions(
                mw_install_hosts, self.config)
            if failed:
                self.logger.warning(
                    '%d hosts had sync_wikiversions errors', failed)
                soft_errors = True

        self.announce('Finished scap: %s (duration: %s)',
            self.arguments.message, self.human_duration)
        # Return a non-zero status if soft errors were seen
        if soft_errors:
            return 1
        else:
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
        self.announce('scap aborted: %s (duration: %s)',
            self.arguments.message, self.human_duration)
        return 1

    def _handle_exception(self, ex):
        self.logger.debug('Unhandled error:', exc_info=True)
        self.announce('scap failed: %s %s (duration: %s)',
            type(ex).__name__, ex, self.human_duration)
        return 1

    def _before_exit(self, exit_status):
        if self.config:
            self.stats.increment('scap.scap')
            self.stats.timing('scap.scap', self.duration * 1000)
        return exit_status


class UpdateL10n(cli.Application):
    """Update localization files"""

    @cli.argument('-v', '--verbose', action='store_true',
        help='Verbose output')
    def main(self, *extra_args):
        for version, wikidb in self.active_wikiversions().items():
            tasks.update_localization_cache(
                version, wikidb, self.arguments.verbose, self.config)
