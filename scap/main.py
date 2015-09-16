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
            self.get_script_path('compile-wikiversions'), self.get_logger())

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
                self.get_script_path('scap-rebuild-cdbs') + ' --no-progress',
                self.get_logger()
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
                    '/usr/sbin/apache2ctl graceful-stop', self.get_logger())
                # Wait for Apache to stop hard after GracefulShutdownTimeout
                # seconds or when requests actually complete
                psutil.Process(apache_pid).wait()

        # Restart HHVM
        utils.sudo_check_call('root', '/sbin/restart hhvm', self.get_logger())

        if have_pybal:
            utils.sudo_check_call('root',
                '/usr/sbin/service apache2 start', self.get_logger())

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

    def main(self, *extra_args):
        repo = self.config['git_repo']
        repo_user = self.config['git_repo_user']
        server = self.config['git_server']
        rev = self.config['git_rev']
        has_submodules = self.config['git_submodules']

        # service_name/service_port only set for deploys requiring restart
        service_name = self.config.get('service_name', None)
        service_port = self.config.get('service_port', None)

        location = os.path.normpath("{0}/{1}".format(
            self.config['git_deploy_dir'], repo))

        # only supports http from tin for the moment
        scheme = 'http'

        url = os.path.normpath('{0}/{1}'.format(server, repo))
        url = "{0}://{1}/.git".format(scheme, url)
        self.get_logger().debug('Fetching from: {}'.format(url))

        tasks.git_fetch(location, url, repo_user)
        tasks.git_checkout(location, rev, has_submodules, repo_user)

        if service_name is not None:
            tasks.restart_service(service_name, user=repo_user)

        if service_port is not None:
            tasks.check_port(int(service_port))


class Deploy(cli.Application):
    """Sync new service code across cluster

    Uses local .scaprc as config for each host in cluster
    """

    @cli.argument('-r', '--rev', default='HEAD', help='Revision to deploy')
    @cli.argument('-l', '--limit-hosts', default='all',
                  help='Limit deploy to hosts matching expression')
    def main(self, *extra_args):
        logger = self.get_logger()
        repo = self.config['git_repo']
        deploy_dir = self.config['git_deploy_dir']
        cwd = os.getcwd()

        in_deploy_dir = os.path.commonprefix([cwd, deploy_dir]) == deploy_dir

        if not in_deploy_dir:
            raise RuntimeError(errno.EPERM,
                'Your path is not a part of the git deploy path', deploy_dir)

        if not utils.is_git_dir(cwd):
            raise RuntimeError(errno.EPERM,
                'Script must be run from deployment repository under {}'
                    .format(deploy_dir))

        targets = utils.get_target_hosts(
            self.arguments.limit_hosts,
            utils.read_hosts_file(self.config['dsh_targets'])
        )

        logger.info(
            'Deploy will run on the following targets: \n\t- {}'.format(
                '\n\t- '.join(targets)
            )
        )

        # batch_size not required, don't allow a batch_size > 80 if set
        batch_size = self.config.get('batch_size', 80)
        batch_size = 80 if batch_size >= 80 else batch_size

        with utils.lock(self.config['lock_file']):
            with log.Timer('deploy_' + repo):
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

                deploy_conf = [
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

                deploy_local_cmd = [self.get_script_path('deploy-local')]

                deploy_local_cmd.extend([
                    "-D '{}:{}'".format(x, self.config.get(x))
                    for x in deploy_conf
                    if self.config.get(x) is not None
                ])

                logger.debug('Running cmd {}'.format(deploy_local_cmd))
                deploy_rev = ssh.Job(
                    hosts=targets, user=self.config['ssh_user'])
                deploy_rev.command(deploy_local_cmd).progress('deploy_' + repo)
                succeeded, failed = deploy_rev.run(batch_size=batch_size)

                if failed:
                    logger.warning('%d targets had deploy errors', failed)
                    return 1

        return 0
