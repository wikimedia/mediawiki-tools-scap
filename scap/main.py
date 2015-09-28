# -*- coding: utf-8 -*-
"""
    scap.main
    ~~~~~~~~~~
    Command wrappers for scap tasks

"""
import argparse
import errno
import multiprocessing
import netifaces
import os
import psutil
import subprocess

from . import cli
from . import log
from . import ssh
from . import tasks
from . import utils
from datetime import datetime


class AbstractSync(cli.Application):
    """Base class for applications that want to sync one or more files from
    the deployment server to the rest of the cluster."""

    soft_errors = False

    def _process_arguments(self, args, extra_args):
        if hasattr(args, 'message'):
            args.message = ' '.join(args.message) or '(no message)'
        return args, extra_args

    @cli.argument('message', nargs='*', help='Log message for SAL')
    def main(self, *extra_args):
        """Perform a sync operation to the cluster."""
        print utils.logo()
        self._assert_auth_sock()

        with utils.lock(self.config['lock_file']):
            self._before_cluster_sync()

            # Update proxies
            proxies = self._get_proxy_list()
            with log.Timer('sync-proxies', self.get_stats()):
                update_proxies = ssh.Job(proxies, user=self.config['ssh_user'])
                update_proxies.command(self._proxy_sync_command())
                update_proxies.progress('sync-proxies')
                succeeded, failed = update_proxies.run()
                if failed:
                    self.get_logger().warning(
                        '%d proxies had sync errors', failed)
                    self.soft_errors = True

            # Update apaches
            with log.Timer('sync-apaches', self.get_stats()):
                update_apaches = ssh.Job(self._get_target_list(),
                                         user=self.config['ssh_user'])
                update_apaches.exclude_hosts(proxies)
                update_apaches.shuffle()
                update_apaches.command(self._apache_sync_command(proxies))
                update_apaches.progress('sync-common')
                succeeded, failed = update_apaches.run()
                if failed:
                    self.get_logger().warning(
                        '%d apaches had sync errors', failed)
                    self.soft_errors = True

            self._after_cluster_sync()

        self._after_lock_release()
        if self.soft_errors:
            return 1
        else:
            return 0

    def _before_cluster_sync(self):
        pass

    def _get_proxy_list(self):
        """Get list of sync proxy hostnames that should be updated before the
        rest of the cluster."""
        return utils.read_hosts_file(self.config['dsh_proxies'])

    def _proxy_sync_command(self):
        """Synchronization command to run on the proxy hosts."""
        cmd = [self.get_script_path('sync-common'), '--no-update-l10n']
        if self.verbose:
            cmd.append('--verbose')
        return cmd

    def _get_target_list(self):
        """Get list of hostnames that should be updated from the proxies."""
        return utils.read_hosts_file(self.config['dsh_targets'])

    def _apache_sync_command(self, proxies):
        """Synchronization command to run on the apache hosts.

        :param proxies: List of proxy hostnames
        """
        return self._proxy_sync_command() + proxies

    def _after_cluster_sync(self):
        pass

    def _after_lock_release(self):
        pass


class CompileWikiversions(cli.Application):
    """Compile wikiversions.json to wikiversions.cdb."""

    def main(self, *extra_args):
        self._run_as('mwdeploy')
        self._assert_current_user('mwdeploy')
        tasks.compile_wikiversions('deploy', self.config)
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
            self.get_logger().warning(
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
            self.get_logger().error(
                'Version %s is in use' % self.arguments.version)
            return 1

        tasks.purge_l10n_cache(self.arguments.version, self.config)
        self.announce('Purged l10n cache for %s' % self.arguments.version)
        return 0


class RebuildCdbs(cli.Application):
    """Rebuild localization cache CDB files from the JSON versions."""

    @cli.argument('--no-progress', action='store_true', dest='mute',
        help='Do not show progress indicator.')
    def main(self, *extra_args):
        self._run_as('mwdeploy')
        self._assert_current_user('mwdeploy')

        # Leave some of the cores free for apache processes
        use_cores = max(multiprocessing.cpu_count() / 2, 1)

        # Rebuild the CDB files from the JSON versions
        for version, wikidb in self.active_wikiversions().items():
            cache_dir = os.path.join(self.config['deploy_dir'],
                'php-%s' % version, 'cache', 'l10n')
            tasks.merge_cdb_updates(
                cache_dir, use_cores, True, self.arguments.mute)


class Scap(AbstractSync):
    """Deploy MediaWiki to the cluster.

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
    #. Restart HHVM across the cluster
    """

    @cli.argument('-r', '--restart', action='store_true', dest='restart',
        help='Restart HHVM process on target hosts.')
    @cli.argument('message', nargs='*', help='Log message for SAL')
    def main(self, *extra_args):
        return super(Scap, self).main(*extra_args)

    def _before_cluster_sync(self):
        self.announce('Started scap: %s', self.arguments.message)

        # Validate php syntax of wmf-config and multiversion
        tasks.check_valid_syntax(
            '%(stage_dir)s/wmf-config' % self.config,
            '%(stage_dir)s/multiversion' % self.config)

        # Sync deploy directory on localhost with staging area
        tasks.sync_common(self.config)

        # Bug 63659: Compile deploy_dir/wikiversions.json to cdb
        utils.sudo_check_call('mwdeploy',
            self.get_script_path('compile-wikiversions'))

        # Update list of extension message files and regenerate the
        # localisation cache.
        with log.Timer('mw-update-l10n', self.get_stats()):
            for version, wikidb in self.active_wikiversions().items():
                tasks.update_localization_cache(
                    version, wikidb, self.verbose, self.config)

        # Compute git version information
        with log.Timer('cache_git_info', self.get_stats()):
            for version, wikidb in self.active_wikiversions().items():
                tasks.cache_git_info(version, self.config)

    def _after_cluster_sync(self):
        target_hosts = self._get_target_list()
        # Ask apaches to rebuild l10n CDB files
        with log.Timer('scap-rebuild-cdbs', self.get_stats()):
            rebuild_cdbs = ssh.Job(target_hosts, user=self.config['ssh_user'])
            rebuild_cdbs.shuffle()
            rebuild_cdbs.command('sudo -u mwdeploy -n -- %s' %
                                 self.get_script_path('scap-rebuild-cdbs'))
            rebuild_cdbs.progress('scap-rebuild-cdbs')
            succeeded, failed = rebuild_cdbs.run()
            if failed:
                self.get_logger().warning(
                    '%d hosts had scap-rebuild-cdbs errors', failed)
                self.soft_errors = True

        # Update and sync wikiversions.cdb
        succeeded, failed = tasks.sync_wikiversions(target_hosts, self.config)
        if failed:
            self.get_logger().warning(
                '%d hosts had sync_wikiversions errors', failed)
            self.soft_errors = True

        if self.arguments.restart:
            # Restart HHVM across the cluster
            succeeded, failed = tasks.restart_hhvm(
                target_hosts, self.config,
                # Use a batch size of 5% of the total target list
                len(target_hosts) // 20)
            if failed:
                self.get_logger().warning(
                    '%d hosts failed to restart HHVM', failed)
                self.soft_errors = True
            self.get_stats().increment('deploy.restart')

    def _after_lock_release(self):
        self.announce('Finished scap: %s (duration: %s)',
            self.arguments.message, utils.human_duration(self.get_duration()))
        self.get_stats().increment('deploy.scap')
        self.get_stats().increment('deploy.all')

    def _handle_keyboard_interrupt(self, ex):
        self.announce('scap aborted: %s (duration: %s)',
            self.arguments.message, utils.human_duration(self.get_duration()))
        return 1

    def _handle_exception(self, ex):
        self.get_logger().warn('Unhandled error:', exc_info=True)
        self.announce('scap failed: %s %s (duration: %s)',
            type(ex).__name__, ex, utils.human_duration(self.get_duration()))
        return 1

    def _before_exit(self, exit_status):
        if self.config:
            self.get_stats().timing('scap.scap', self.get_duration() * 1000)
        return exit_status


class SyncCommon(cli.Application):
    """Sync local MediaWiki deployment directory with deploy server state."""

    @cli.argument('--no-update-l10n', action='store_false', dest='update_l10n',
        help='Do not update l10n cache files.')
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
        if self.arguments.update_l10n:
            utils.sudo_check_call(
                'mwdeploy',
                self.get_script_path('scap-rebuild-cdbs') + ' --no-progress'
            )
        return 0


class SyncDblist(AbstractSync):
    """Sync dblist files to the cluster."""

    def _proxy_sync_command(self):
        cmd = [self.get_script_path('sync-common'),
               '--no-update-l10n', '--include', '*.dblist']
        if self.verbose:
            cmd.append('--verbose')
        return cmd

    def _after_lock_release(self):
        self.announce('Synchronized database lists: %s (duration: %s)',
            self.arguments.message, utils.human_duration(self.get_duration()))
        self.get_stats().increment('deploy.sync-dblist')
        self.get_stats().increment('deploy.all')


class SyncDir(AbstractSync):
    """Sync a directory to the cluster."""

    @cli.argument('dir', help='Directory to sync')
    @cli.argument('message', nargs='*', help='Log message for SAL')
    def main(self, *extra_args):
        return super(SyncDir, self).main(*extra_args)

    def _before_cluster_sync(self):
        # assert file exists
        abspath = os.path.join(
            self.config['stage_dir'], self.arguments.dir)
        if not os.path.isdir(abspath):
            raise IOError(errno.ENOENT, 'Directory not found', abspath)

        relpath = os.path.relpath(abspath, self.config['stage_dir'])
        self.include = '%s/***' % relpath
        tasks.check_valid_syntax(abspath)

    def _proxy_sync_command(self):
        cmd = [self.get_script_path('sync-common'), '--no-update-l10n']

        if '/' in self.include:
            parts = self.include.split('/')
            for i in range(1, len(parts)):
                # Include parent directories in sync command or the default
                # exclude will block them and by extension block the target
                # file.
                cmd.extend(['--include', '/'.join(parts[:i])])

        cmd.extend(['--include', self.include])
        if self.verbose:
            cmd.append('--verbose')
        return cmd

    def _after_lock_release(self):
        self.announce('Synchronized %s: %s (duration: %s)',
            self.arguments.dir, self.arguments.message,
            utils.human_duration(self.get_duration()))
        self.get_stats().increment('deploy.sync-dir')
        self.get_stats().increment('deploy.all')


class SyncDocroot(AbstractSync):

    def _proxy_sync_command(self):
        cmd = [
            self.get_script_path('sync-common'),
            '--no-update-l10n',
            '--include', 'docroot/***',
            '--include', 'w/***',
        ]
        if self.verbose:
            cmd.append('--verbose')
        return cmd

    def _after_lock_release(self):
        self.announce('Synchronized docroot and w: %s (duration: %s)',
            self.arguments.message, utils.human_duration(self.get_duration()))
        self.get_stats().increment('deploy.sync-docroot')
        self.get_stats().increment('deploy.all')


class SyncFile(AbstractSync):
    """Sync a specific file to the cluster."""

    @cli.argument('file', help='File to sync')
    @cli.argument('message', nargs='*', help='Log message for SAL')
    def main(self, *extra_args):
        return super(SyncFile, self).main(*extra_args)

    def _before_cluster_sync(self):
        # assert file exists
        abspath = os.path.join(
            self.config['stage_dir'], self.arguments.file)
        if not os.path.isfile(abspath):
            raise IOError(errno.ENOENT, 'File not found', abspath)
        # Warn when syncing a symlink.
        if os.path.islink(abspath):
            self.get_logger().warning(
                '%s: did you mean to sync a symbolic link?', abspath)

        self.include = os.path.relpath(abspath, self.config['stage_dir'])
        if abspath.endswith(('.php', '.inc', '.phtml', '.php5')):
            subprocess.check_call('/usr/bin/php -l %s' % abspath, shell=True)
            utils.check_php_opening_tag(abspath)
        elif abspath.endswith('.json'):
            utils.check_valid_json_file(abspath)

    def _proxy_sync_command(self):
        cmd = [self.get_script_path('sync-common'), '--no-update-l10n']

        if '/' in self.include:
            parts = self.include.split('/')
            for i in range(1, len(parts)):
                # Include parent directories in sync command or the default
                # exclude will block them and by extension block the target
                # file.
                cmd.extend(['--include', '/'.join(parts[:i])])

        cmd.extend(['--include', self.include])
        if self.verbose:
            cmd.append('--verbose')
        return cmd

    def _after_lock_release(self):
        self.announce('Synchronized %s: %s (duration: %s)',
            self.arguments.file, self.arguments.message,
            utils.human_duration(self.get_duration()))
        self.get_stats().increment('deploy.sync-file')
        self.get_stats().increment('deploy.all')


class SyncWikiversions(cli.Application):
    """Rebuild and sync wikiversions.cdb to the cluster."""

    def _process_arguments(self, args, extra_args):
        args.message = ' '.join(args.message) or '(no message)'
        return args, extra_args

    @cli.argument('message', nargs='*', help='Log message for SAL')
    def main(self, *extra_args):
        self._assert_auth_sock()

        # check for the presence of ExtensionMessages and l10n cache
        # for every branch of mediawiki that is referenced in wikiversions.json
        # to avoid syncing a branch that is lacking these critical files.
        for version, wikidb in self.active_wikiversions().items():
            ext_msg = os.path.join(self.config['stage_dir'],
                'wmf-config', 'ExtensionMessages-%s.php' % version)
            err_msg = 'ExtensionMessages not found in %s' % ext_msg
            utils.check_file_exists(ext_msg, err_msg)

            cache_file = os.path.join(self.config['stage_dir'],
                'php-%s' % version, 'cache', 'l10n', 'l10n_cache-en.cdb')
            err_msg = 'l10n cache missing for %s' % version
            utils.check_file_exists(cache_file, err_msg)

        mw_install_hosts = utils.read_hosts_file(
            self.config['dsh_targets'])
        tasks.sync_wikiversions(mw_install_hosts, self.config)

        self.announce(
            'rebuilt wikiversions.cdb and synchronized wikiversions files: %s',
            self.arguments.message)

        self.get_stats().increment('deploy.sync-wikiversions')
        self.get_stats().increment('deploy.all')


class UpdateL10n(cli.Application):
    """Update localization files"""

    def main(self, *extra_args):
        for version, wikidb in self.active_wikiversions().items():
            tasks.update_localization_cache(
                version, wikidb, self.verbose, self.config)


class RestartHHVM(cli.Application):
    """Restart the HHVM fcgi process on the local server

    #. Depool the server if registered with pybal
    #. Wait for pending requests to complete
    #. Restart HHVM process
    #. Re-pool the server if needed
    """

    def main(self, *extra_args):
        self._run_as('mwdeploy')
        self._assert_current_user('mwdeploy')

        try:
            hhvm_pid = utils.read_pid(self.config['hhvm_pid_file'])
        except IOError:
            self.get_logger().debug('HHVM pid not found', exc_info=True)
            return 0
        else:
            if not psutil.pid_exists(hhvm_pid):
                self.get_logger().debug('HHVM not running')
                return 0

        try:
            # Check for pybal interface
            have_pybal = netifaces.ifaddresses(self.config['pybal_interface'])
        except ValueError:
            self.get_logger().debug('Pybal interface not found', exc_info=True)
            have_pybal = False

        if have_pybal:
            # Depool by gracefully shutting down apache (SIGWINCH)
            try:
                apache_pid = utils.read_pid(self.config['apache_pid_file'])
            except IOError:
                self.get_logger().debug('Apache pid not found', exc_info=True)
                pass
            else:
                utils.sudo_check_call('root',
                    '/usr/sbin/apache2ctl graceful-stop')
                # Wait for Apache to stop hard after GracefulShutdownTimeout
                # seconds or when requests actually complete
                psutil.Process(apache_pid).wait()

        # Restart HHVM
        utils.sudo_check_call('root', '/sbin/restart hhvm')

        if have_pybal:
            utils.sudo_check_call('root',
                '/usr/sbin/service apache2 start')

        return 0


class HHVMGracefulAll(cli.Application):
    """Perform a rolling restart of HHVM across the cluster."""

    def _process_arguments(self, args, extra_args):
        if hasattr(args, 'message'):
            args.message = ' '.join(args.message) or '(no message)'
        return args, extra_args

    @cli.argument('message', nargs='*', help='Log message for SAL')
    def main(self, *extra_args):
        exit_code = 0
        self.announce('Restarting HHVM: %s', self.arguments.message)

        target_hosts = utils.read_hosts_file(self.config['dsh_targets'])
        succeeded, failed = tasks.restart_hhvm(
            target_hosts, self.config,
            # Use a batch size of 5% of the total target list
            len(target_hosts) // 20)
        if failed:
            self.get_logger().warning(
                '%d hosts failed to restart HHVM', failed)
            self.get_stats().increment('deploy.fail')
            exit_code = 1

        self.announce('Finished HHVM restart: %s (duration: %s)',
            self.arguments.message, utils.human_duration(self.get_duration()))
        self.get_stats().increment('deploy.restart')

        return exit_code


class DeployLocal(cli.Application):
    """Deploy service code via git"""
    STAGES = ['fetch', 'promote', 'check']
    EX_STAGES = ['rollback']

    rev = None
    cache_dir = None
    revs_dir = None
    rev_dir = None
    cur_link = None
    progress_flag = None
    done_flag = None
    user = None

    @cli.argument('stage', metavar='STAGE', choices=STAGES + EX_STAGES,
        help='Stage of the deployment to execute')
    def main(self, *extra_args):
        self.rev = self.config['git_rev']

        self.root_dir = os.path.normpath("{0}/{1}".format(
            self.config['git_deploy_dir'], self.config['git_repo']))

        # cache, revs, and current directory go under [repo]-cache and are
        # linked to [repo] as a final step
        root_deploy_dir = '{}-cache'.format(self.root_dir)
        deploy_dir = lambda subdir: os.path.join(root_deploy_dir, subdir)

        self.cache_dir = deploy_dir('cache')
        self.revs_dir = deploy_dir('revs')

        self.rev_dir = os.path.join(self.revs_dir, self.rev)

        self.cur_link = deploy_dir('current')
        self.progress_flag = deploy_dir('.in-progress')
        self.done_flag = deploy_dir('.done')

        self.user = self.config['git_repo_user']

        getattr(self, self.arguments.stage)()

    def fetch(self):
        """Fetch the specified revision of the remote repo.

        The given repo is cloned into the cache directory and a new working
        directory for the given revision is created under revs/{rev}.

        At the end of this stage, the .in-progress link is created to signal
        the possibility for future rollback.
        """

        repo = self.config['git_repo']
        server = self.config['git_server']
        has_submodules = self.config['git_submodules']

        # create deployment directories if they don't already exist
        for d in [self.cache_dir, self.revs_dir]:
            utils.mkdir_p(d)

        # only supports http from tin for the moment
        scheme = 'http'

        url = os.path.normpath('{0}/{1}'.format(server, repo))
        url = "{0}://{1}/.git".format(scheme, url)
        self.get_logger().debug('Fetching from: {}'.format(url))

        # clone/fetch from the repo to the cache directory
        tasks.git_fetch(self.cache_dir, url, user=self.user)

        # clone/fetch from the local cache directory to the revision directory
        tasks.git_fetch(self.rev_dir, self.cache_dir, user=self.user)

        # checkout the given revision
        tasks.git_checkout(self.rev_dir, self.rev,
                           submodules=has_submodules,
                           user=self.user)

        # link the .in-progress flag to the rev directory
        self._link_rev_dir(self.progress_flag)

    def promote(self):
        """Promote the current deployment.

        Switches the `current` symlink to the current revision directory and
        restarts the configured service.
        """

        service = self.config.get('service_name', None)

        self._link_rev_dir(self.cur_link)
        self._link_final_to_current()

        if service is not None:
            tasks.restart_service(service, user=self.config['git_repo_user'])

    def check(self):
        """Verifies whether the promotion was successful.

        Probes the configured service port to measure whether it successfully
        restarted.

        At the end of this stage, the .done link is created and the
        .in-progress link is removed.
        """

        port = self.config.get('service_port', None)

        if port is not None:
            tasks.check_port(int(port))

        # move .done flag and remove the .in-progress flag
        self._link_rev_dir(self.done_flag)
        self._remove_progress_link()

    def rollback(self):
        """Performs a rollback to the last deployed revision.

        The rollback stage expects an .in-progress symlink to points to the
        revision directory for the currently running deployment. If the link
        doesn't exist, it's assumed that the current deployment errored at an
        early enough stage where a rollback isn't necessary.

        It also looks for a .done symlink that points to the revision
        directory for the last successful deployment. If this link doesn't
        exist, a rollback isn't possible. If it does exist, the current
        revision directory is replaced with the target of the link and the
        promote stage is re-run.
        """

        logger = self.get_logger()

        if not os.path.exists(self.progress_flag):
            logger.info('No rollback necessary. Skipping')
            return 0

        if not os.path.exists(self.done_flag):
            raise RuntimeError('there is no previous revision to rollback to')

        rev_dir = os.path.realpath(self.done_flag)
        rev = os.path.basename(rev_dir)

        if not os.path.isdir(rev_dir):
            msg = 'rollback failed due to missing rev directory {}'
            raise RuntimeError(msg.format(rev_dir))

        logger.info('Rolling back to revision {}'.format(rev))
        self.rev = rev
        self.rev_dir = rev_dir
        self.promote()
        self._remove_progress_link()

    def _link_final_to_current(self):
        """Link the current checkout to final location at [repo]

        This should really only be needed the first time that scap03 is
        run. It links the [repo]-cache/current directory to [repo].
        """
        if (not os.path.islink(self.root_dir) and
           (os.path.isfile(self.root_dir) or os.path.isdir(self.root_dir))):
                os.rename(self.root_dir, '{}.trebuchet'.format(self.root_dir))

        tasks.move_symlink(self.cur_link, self.root_dir, user=self.user)

    def _link_rev_dir(self, symlink_path):
        tasks.move_symlink(self.rev_dir, symlink_path, user=self.user)

    def _remove_progress_link(self):
        tasks.remove_symlink(self.progress_flag, user=self.user)


class Deploy(cli.Application):
    """Sync new service code across cluster

    Uses local .scaprc as config for each host in cluster
    """

    MAX_BATCH_SIZE = 80
    # Stop executing on new hosts after failure
    MAX_FAILURES = 0

    DEPLOY_CONF = [
        'git_deploy_dir',
        'git_repo_user',
        'git_server',
        'git_scheme',
        'git_repo',
        'git_rev',
        'git_submodules',
        'service_name',
        'service_port',
    ]

    repo = None
    targets = []

    @cli.argument('-r', '--rev', default='HEAD', help='Revision to deploy')
    @cli.argument('-s', '--stages', choices=DeployLocal.STAGES,
                  help='Deployment stages to execute. Used only for testing.')
    @cli.argument('-l', '--limit-hosts', default='all',
                  help='Limit deploy to hosts matching expression')
    def main(self, *extra_args):
        logger = self.get_logger()
        self.repo = self.config['git_repo']
        deploy_dir = self.config['git_deploy_dir']
        cwd = os.getcwd()

        if self.arguments.stages:
            stages = self.arguments.stages.split(',')
        else:
            stages = DeployLocal.STAGES

        in_deploy_dir = os.path.commonprefix([cwd, deploy_dir]) == deploy_dir

        if not in_deploy_dir:
            raise RuntimeError(errno.EPERM,
                'Your path is not a part of the git deploy path', deploy_dir)

        if not utils.is_git_dir(cwd):
            raise RuntimeError(errno.EPERM,
                'Script must be run from deployment repository under {}'
                    .format(deploy_dir))

        self.targets = utils.get_target_hosts(
            self.arguments.limit_hosts,
            utils.read_hosts_file(self.config['dsh_targets'])
        )

        logger.info(
            'Deploy will run on the following targets: \n\t- {}'.format(
                '\n\t- '.join(self.targets)
            )
        )

        with utils.lock(self.config['lock_file']):
            with log.Timer('deploy_' + self.repo):
                timestamp = datetime.utcnow()
                tag = utils.git_next_deploy_tag(location=cwd)
                commit = utils.git_sha(location=cwd, rev=self.arguments.rev)
                user = utils.get_real_username()

                deploy_info = {
                    'tag': tag,
                    'commit': commit,
                    'user': user,
                    'timestamp': timestamp.isoformat(),
                }

                tasks.git_update_deploy_head(deploy_info, location=cwd)
                tasks.git_tag_repo(deploy_info, location=cwd)

                self.config['git_rev'] = commit

                # Run git update-server-info because git repo is a dumb
                # apache server
                tasks.git_update_server_info(self.config['git_submodules'])

                for stage in stages:
                    ret = self.execute_stage(stage)
                    if ret > 0:
                        self.execute_rollback(stage)
                        return ret

        return 0

    def execute_rollback(self, stage):
        prompt = "Stage '{}' failed. Perform rollback?".format(stage)

        if utils.ask(prompt, 'y') == 'y':
            return self.execute_stage('rollback')

        return 0

    def execute_stage(self, stage):
        logger = self.get_logger()
        deploy_local_cmd = [self.get_script_path('deploy-local')]
        batch_size = self._get_batch_size(stage)

        deploy_local_cmd.extend([
            "-D '{}:{}'".format(x, self.config.get(x))
            for x in self.DEPLOY_CONF
            if self.config.get(x) is not None
        ])

        # Handle JSON output from deploy-local
        deploy_local_cmd = deploy_local_cmd + ['-D log_json:True', '-v']

        deploy_stage_cmd = deploy_local_cmd + [stage]
        logger.debug('Running cmd {}'.format(deploy_stage_cmd))

        deploy_stage = ssh.Job(
            hosts=self.targets, user=self.config['ssh_user'])
        deploy_stage.output_handler = ssh.JSONOutputHandler
        deploy_stage.max_failure = self.MAX_FAILURES
        deploy_stage.command(deploy_stage_cmd)
        deploy_stage.progress('deploy_{}_{}'.format(self.repo, stage))

        succeeded, failed = deploy_stage.run(batch_size=batch_size)

        if failed:
            logger.warning('%d targets had deploy errors', failed)
            return 1

        return 0

    def _get_batch_size(self, stage):
        default = self.config.get('batch_size', self.MAX_BATCH_SIZE)
        size = int(self.config.get('{}_batch_size'.format(stage), default))
        return min(size, self.MAX_BATCH_SIZE)
