# -*- coding: utf-8 -*-
"""
    scap.main
    ~~~~~~~~~~
    Command wrappers for scap tasks

"""
import argparse
import errno
import multiprocessing
import os
import subprocess

from . import cli
from . import log
from . import ssh
from . import tasks
from . import utils


class AbstractSync(cli.Application):
    """Base class for applications that want to sync one or more files from
    the deployment server to the rest of the cluster."""

    needs_local_sync = False
    needs_two_phase_sync = False
    soft_errors = False

    def main(self, *extra_args):
        """Perform a sync operation to the cluster."""
        self._assert_auth_sock()

        with utils.lock(self.config['lock_file']):
            self._after_lock_aquire()

            if self.needs_local_sync:
                self._local_sync()

            self._before_cluster_sync()

            # Update proxies
            proxies = self._get_proxy_list()
            with log.Timer('sync-proxies', self.stats):
                update_proxies = ssh.Job(proxies)
                update_proxies.command(self._proxy_sync_command())
                update_proxies.progress('sync-proxies')
                succeeded, failed = update_proxies.run()
                if failed:
                    self.logger.warning('%d proxies had sync errors', failed)
                    self.soft_errors = True

            # Update apaches
            apaches = self._get_apache_list()
            with log.Timer('update apaches', self.stats) as t:
                update_apaches = ssh.Job(apaches)
                update_apaches.exclude_hosts(proxies)
                update_apaches.shuffle()
                update_apaches.command(self._apache_sync_command(proxies))
                update_apaches.progress('sync-common')
                succeeded, failed = update_apaches.run()
                if failed:
                    self.logger.warning(
                        '%d apaches had sync errors', failed)
                    self.soft_errors = True
                t.mark('sync to apaches')

                if self.needs_two_phase_sync:
                    self._phase_two_sync(apaches, t)

            self._after_cluster_sync()
        self._after_lock_release()
        if self.soft_errors:
            return 1
        else:
            return 0

    def _after_lock_aquire(self):
        pass

    def _local_sync(self):
        tasks.sync_common(self.config)

    def _before_cluster_sync(self):
        pass

    def _get_proxy_list(self):
        return utils.read_dsh_hosts_file('scap-proxies')

    def _proxy_sync_command(self):
        cmd = ['/usr/local/bin/sync-common']
        if self.verbose:
            cmd += ['--verbose']
        return cmd

    def _get_apache_list(self):
        return utils.read_dsh_hosts_file('mediawiki-installation')

    def _apache_sync_command(self, proxies):
        """
        :param proxies: List of proxy hostnames
        """
        return self._proxy_sync_command() + proxies

    def _phase_two_sync(self, apaches, timer):
        """
        :param apaches: List of apache hostnames
        :param timer: :class:`log.Timer` object timing the cluster sync
        """
        pass

    def _after_cluster_sync(self):
        pass

    def _after_lock_release(self):
        pass


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

    @cli.argument('--version', required=True,
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
        use_cores = max(multiprocessing.cpu_count() / 2, 1)

        # Rebuild the CDB files from the JSON versions
        for version, wikidb in self.active_wikiversions().items():
            cache_dir = os.path.join(self.config['deploy_dir'],
                'php-%s' % version, 'cache', 'l10n')
            tasks.merge_cdb_updates(cache_dir, use_cores, True)


class Scap(AbstractSync):
    """Deploy MediaWiki to the cluster."""

    needs_local_sync = True
    needs_two_phase_sync = True

    def _process_arguments(self, args, extra_args):
        args.message = ' '.join(args.message) or '(no message)'
        return args, extra_args

    @cli.argument('message', nargs=argparse.REMAINDER,
        help='Log message for SAL')
    def main(self, *extra_args):
        """Core business logic of scap process.

        #. Validate php syntax of wmf-config and multiversion
        #. Sync deploy directory on localhost with staging area
        #. Compile wikiversions.json to cdb in deploy directory
        #. Update l10n files in staging area
        #. Compute git version information
        #. Ask scap proxies to sync with master server
        #. Ask apaches to sync with fastest rsync server
        #. Ask apaches to rebuild l10n CDB files
        #. Update wikiversions.cdb on localhost
        #. Ask apaches to sync wikiversions.cdb
        """
        super(Scap, self).main(*extra_args)

    def _after_lock_aquire(self):
        self.announce('Started scap: %s', self.arguments.message)

        tasks.check_php_syntax(
            '%(stage_dir)s/wmf-config' % self.config,
            '%(stage_dir)s/multiversion' % self.config)

    def _before_cluster_sync(self):
        # Bug 63659: Compile deploy_dir/wikiversions.json to cdb
        subprocess.check_call('sudo -u mwdeploy -- '
            '/usr/local/bin/compile-wikiversions', shell=True)

        # Update list of extension message files and regenerate the
        # localisation cache.
        with log.Timer('mw-update-l10n', self.stats):
            for version, wikidb in self.active_wikiversions().items():
                tasks.update_localization_cache(
                    version, wikidb, self.verbose, self.config)

        # Compute git version information
        with log.Timer('cache_git_info', self.stats):
            for version, wikidb in self.active_wikiversions().items():
                tasks.cache_git_info(version, self.config)

    def _phase_two_sync(self, apaches, timer):
        rebuild_cdbs = ssh.Job(apaches)
        rebuild_cdbs.command(
            'sudo -u mwdeploy -n -- /usr/local/bin/scap-rebuild-cdbs')
        rebuild_cdbs.progress('scap-rebuild-cdbs')
        succeeded, failed = rebuild_cdbs.run()
        if failed:
            self.logger.warning(
                '%d hosts had scap-rebuild-cdbs errors', failed)
            self.soft_errors = True
        timer.mark('scap-rebuild-cdbs')

    def _after_cluster_sync(self):
        # Update and sync wikiversions.cdb
        succeeded, failed = tasks.sync_wikiversions(
            self._get_apache_list(), self.config)
        if failed:
            self.logger.warning(
                '%d hosts had sync_wikiversions errors', failed)
            self.soft_errors = True

    def _after_lock_release(self):
        self.announce('Finished scap: %s (duration: %s)',
            self.arguments.message, self.human_duration)

    def _handle_keyboard_interrupt(self, ex):
        self.announce('scap aborted: %s (duration: %s)',
            self.arguments.message, self.human_duration)
        return 1

    def _handle_exception(self, ex):
        self.logger.warn('Unhandled error:', exc_info=True)
        self.announce('scap failed: %s %s (duration: %s)',
            type(ex).__name__, ex, self.human_duration)
        return 1

    def _before_exit(self, exit_status):
        if self.config:
            self.stats.increment('scap.scap')
            self.stats.timing('scap.scap', self.duration * 1000)
        return exit_status


class SyncCommon(cli.Application):
    """Sync local MediaWiki deployment directory with deploy server state."""

    @cli.argument('-i', '--include', default=None, action='append',
        help='Rsync include pattern to limit transfer to.'
        'End directories with a trailing `/***`. Can be used multiple times.')
    @cli.argument('servers', nargs=argparse.REMAINDER,
        help='Rsync server(s) to copy from')
    def main(self, *extra_args):
        tasks.sync_common(
            self.config,
            include=self.arguments.include,
            sync_from=self.arguments.servers,
            verbose=self.verbose
        )
        return 0


class SyncDblist(AbstractSync):
    """Sync dblist files to the cluster."""

    def _process_arguments(self, args, extra_args):
        args.message = ' '.join(args.message) or '(no message)'
        return args, extra_args

    @cli.argument('message', nargs='*', help='Log message for SAL')
    def main(self, *extra_args):
        super(SyncDblist, self).main(*extra_args)

    def _proxy_sync_command(self):
        cmd = ['/usr/local/bin/sync-common', '--include', '*.dblist']
        if self.verbose:
            cmd += ['--verbose']
        return cmd

    def _after_lock_release(self):
        self.announce('Synchronized database lists: %s (duration: %s)',
            self.arguments.message, self.human_duration)
        self.stats.increment('deploy.sync-dblist')
        self.stats.increment('deploy.all')


class SyncDir(AbstractSync):
    """Sync a directory to the cluster."""

    def _process_arguments(self, args, extra_args):
        args.message = ' '.join(args.message) or '(no message)'
        return args, extra_args

    @cli.argument('dir', help='Directory to sync')
    @cli.argument('message', nargs='*', help='Log message for SAL')
    def main(self, *extra_args):
        super(SyncDir, self).main(*extra_args)

    def _before_cluster_sync(self):
        # assert file exists
        abspath = os.path.join(
            self.config['stage_dir'], self.arguments.dir)
        if not os.path.isdir(abspath):
            raise IOError(errno.ENOENT, 'Directory not found', abspath)

        relpath = os.path.relpath(abspath, self.config['stage_dir'])
        self.include = '%s/***' % relpath
        tasks.check_php_syntax(abspath)

    def _proxy_sync_command(self):
        cmd = ['/usr/local/bin/sync-common', '--include', self.include]
        if self.verbose:
            cmd += ['--verbose']
        return cmd

    def _after_lock_release(self):
        self.announce('Synchronized %s: %s (duration: %s)',
            self.arguments.dir, self.arguments.message, self.human_duration)
        self.stats.increment('deploy.sync-dir')
        self.stats.increment('deploy.all')


class SyncFile(AbstractSync):
    """Sync a specific file to the cluster."""

    def _process_arguments(self, args, extra_args):
        args.message = ' '.join(args.message) or '(no message)'
        return args, extra_args

    @cli.argument('file', help='File to sync')
    @cli.argument('message', nargs='*', help='Log message for SAL')
    def main(self, *extra_args):
        super(SyncFile, self).main(*extra_args)

    def _before_cluster_sync(self):
        # assert file exists
        abspath = os.path.join(
            self.config['stage_dir'], self.arguments.file)
        if not os.path.isfile(abspath):
            raise IOError(errno.ENOENT, 'File not found', abspath)

        self.include = os.path.relpath(abspath, self.config['stage_dir'])
        subprocess.check_call('/usr/bin/php -l %s' % abspath, shell=True)

    def _proxy_sync_command(self):
        cmd = ['/usr/local/bin/sync-common', '--include', self.include]
        if self.verbose:
            cmd += ['--verbose']
        return cmd

    def _after_lock_release(self):
        self.announce('Synchronized %s: %s (duration: %s)',
            self.arguments.file, self.arguments.message, self.human_duration)
        self.stats.increment('deploy.sync-file')
        self.stats.increment('deploy.all')


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


class UpdateL10n(cli.Application):
    """Update localization files"""

    def main(self, *extra_args):
        for version, wikidb in self.active_wikiversions().items():
            tasks.update_localization_cache(
                version, wikidb, self.verbose, self.config)
