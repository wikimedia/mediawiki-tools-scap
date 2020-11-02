# -*- coding: utf-8 -*-
"""
    scap.main
    ~~~~~~~~~~
    Command wrappers for scap tasks

    Copyright © 2014-2017 Wikimedia Foundation and Contributors.

    This file is part of Scap.

    Scap is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, version 3.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""
from __future__ import absolute_import
from __future__ import print_function

import argparse
from concurrent.futures import ProcessPoolExecutor
import errno
import os
import pwd
import select
import socket
import sys
import time

from scap import ansi
from scap.sh import ErrorReturnCode
import scap.arg as arg
import scap.cli as cli
import scap.lint as lint
import scap.lock as lock
import scap.log as log
import scap.opcache_manager as opcache_manager
import scap.php_fpm as php_fpm
import scap.sh as sh
import scap.ssh as ssh
import scap.targets as targets
import scap.tasks as tasks
import scap.utils as utils
import scap.version as scapversion

try:
    from StringIO import StringIO
except ImportError:
    from io import StringIO


class AbstractSync(cli.Application):
    """Base class for applications that want to sync one or more files from
    the deployment server to the rest of the cluster."""

    soft_errors = False

    def __init__(self, exe_name):
        super(AbstractSync, self).__init__(exe_name)
        self.include = None
        self.om = None

    @cli.argument('--force', action='store_true',
                  help='Skip canary checks, '
                  'performs ungraceful php-fpm restarts')
    @cli.argument('message', nargs='*', help='Log message for SAL')
    def main(self, *extra_args):
        """Perform a sync operation to the cluster."""
        print(ansi.logo())
        self._assert_auth_sock()

        with lock.Lock(self.get_lock_file(), self.arguments.message):
            self._check_sync_flag()
            if not self.arguments.force:
                if self._can_run_check_fatals():
                    self.get_logger().info(
                        'Checking for new runtime errors locally')
                    self._check_fatals()
            else:
                self.get_logger().warning('check_fatals Skipped by --force')
            self._before_cluster_sync()
            self._sync_common()
            self._after_sync_common()
            self._sync_masters()

            full_target_list = self._get_target_list()

            if not self.arguments.force:
                canaries = [node for node in self._get_canary_list()
                            if node in full_target_list]
                with log.Timer(
                        'sync-check-canaries', self.get_stats()) as timer:
                    self.sync_canary(canaries)
                    timer.mark('Canaries Synced')
                    self._invalidate_opcache(canaries)
                    self.canary_checks(canaries, timer)
            else:
                self.get_logger().warning('Canaries Skipped by --force')

            # Update proxies
            proxies = [node for node in self._get_proxy_list()
                       if node in full_target_list]

            with log.Timer('sync-proxies', self.get_stats()):
                sync_cmd = self._apache_sync_command(self.get_master_list())
                # Proxies should always use the current host as their sync
                # origin server.
                sync_cmd.append(socket.getfqdn())
                update_proxies = ssh.Job(
                    proxies,
                    user=self.config['ssh_user'],
                    key=self.get_keyholder_key())
                update_proxies.command(sync_cmd)
                update_proxies.progress(
                    log.reporter(
                        'sync-proxies',
                        self.config['fancy_progress']))
                succeeded, failed = update_proxies.run()
                if failed:
                    self.get_logger().warning(
                        '%d proxies had sync errors', failed)
                    self.soft_errors = True

            # Update apaches
            with log.Timer('sync-apaches', self.get_stats()):
                update_apaches = ssh.Job(
                    full_target_list,
                    user=self.config['ssh_user'],
                    key=self.get_keyholder_key())
                update_apaches.exclude_hosts(proxies)
                update_apaches.exclude_hosts(self.get_master_list())
                if not self.arguments.force:
                    update_apaches.exclude_hosts(canaries)
                update_apaches.shuffle()
                if proxies:
                    targets = proxies
                else:
                    # scap pull will try to rsync from localhost if it doesn't
                    # get a list of deploy servers, so use the list of
                    # masters if there no proxies.
                    targets = self.get_master_list()
                update_apaches.command(self._apache_sync_command(targets))
                update_apaches.progress(
                    log.reporter(
                        'sync-apaches',
                        self.config['fancy_progress']))
                succeeded, failed = update_apaches.run()
                if failed:
                    self.get_logger().warning(
                        '%d apaches had sync errors', failed)
                    self.soft_errors = True

            self._after_cluster_sync()

        self._after_lock_release()
        if self.soft_errors:
            return 1
        return 0

    def increment_stat(self, stat, all_stat=True, value=1):
        """Increment a stat in deploy.*

        :param stat: String name of stat to increment
        :param all_stat: Whether to increment deploy.all as well
        :param value: How many to increment by, default of 1 is normal
        """
        self.get_stats().increment('deploy.%s' % stat, value)
        if all_stat:
            self.get_stats().increment('deploy.all', value)

    def get_keyholder_key(self):
        """
        Returns scap2-specific deploy key

        This way we can set a key in the default scap config without
        having all non-scap2 repos inherit that configuration.
        """
        key = self.config.get('mediawiki_keyholder_key', None)
        if key:
            return key

        return super(AbstractSync, self).get_keyholder_key()

    def _before_cluster_sync(self):
        pass

    def _after_sync_common(self):
        self._git_repo()

        # Compute git version information
        with log.Timer('cache_git_info', self.get_stats()):
            for version, wikidb in self.active_wikiversions().items():
                tasks.cache_git_info(version, self.config)

    def _can_run_check_fatals(self):
        """
        _check_fatals will not succeed if /srv/mediawiki/wikiversions.php
        hasn't been prepared yet. This will be the case if 'scap
        sync-world' has never been run.  Running 'scap sync-world'
        will fix everything up.
        """
        wikiversionsphp = os.path.join(
            self.config['deploy_dir'],
            utils.get_realm_specific_filename("wikiversions.php",
                                              self.config['wmf_realm']))
        return os.path.exists(wikiversionsphp)

    def _check_fatals(self):
        mwscript = sh.Command('mwscript')
        errbuf = StringIO()
        errmsg = 'Scap failed!: Call to mwscript eval.php {}: {}'
        try:
            mwscript("eval.php", "--wiki", "enwiki", _in="1",
                     _err=errbuf)
            errout = errbuf.getvalue().strip()
            if errout:
                self.announce(errmsg.format("stderr", "not empty"))
                raise RuntimeError(errmsg.format("stderr", errout))
        except ErrorReturnCode as e:
            self.announce(errmsg.format("returned", e.exit_code))
            if e.stdout:
                self.announce("stdout: {}".format(e.stdout))
            self.announce("stderr: {}".format(errbuf.getvalue().strip()))
            raise RuntimeError(errmsg.format('returned', e.exit_code))
        finally:
            errbuf.close()

    def _check_sync_flag(self):
        sync_flag = os.path.join(self.config['stage_dir'], 'sync.flag')
        if os.path.exists(sync_flag):
            stat = os.stat(sync_flag)
            owner = pwd.getpwuid(stat.st_uid).pw_name
            utils.get_logger().error(
                "%s's sync.flag is blocking deployments", owner)
            raise IOError(errno.EPERM, 'Blocked by sync.flag', sync_flag)

    def _get_proxy_list(self):
        """Get list of sync proxy hostnames that should be updated before the
        rest of the cluster."""
        return targets.get('dsh_proxies', self.config).all

    def _get_target_list(self):
        """Get list of hostnames that should be updated from the proxies."""
        return list(
            set(self.get_master_list()) |
            set(self._get_proxy_list()) |
            set(targets.get('dsh_targets', self.config).all)
        )

    def _get_api_canary_list(self):
        """Get list of MediaWiki api canaries."""
        return targets.get('dsh_api_canaries', self.config).all

    def _get_app_canary_list(self):
        """Get list of MediaWiki api canaries."""
        return targets.get('dsh_app_canaries', self.config).all

    def _get_canary_list(self):
        """Get list of MediaWiki canary hostnames."""
        return list(
            set(self._get_api_canary_list()) |
            set(self._get_app_canary_list())
        )

    def _sync_masters(self):
        """Sync the staging directory across all deploy master servers."""
        self.master_only_cmd('sync-masters', self._master_sync_command())
        self.master_only_cmd('sync-pull-masters', self._proxy_sync_command())

    def master_only_cmd(self, timer, cmd):
        """
        Run a command on all other master servers than the one we're on

        :param timer: String name to use in timer/logging
        :param cmd: List of command/parameters to be executed
        """

        masters = self.get_master_list()
        with log.Timer(timer, self.get_stats()):
            update_masters = ssh.Job(
                masters,
                user=self.config['ssh_user'],
                key=self.get_keyholder_key())
            update_masters.exclude_hosts([socket.getfqdn()])
            update_masters.command(cmd)
            update_masters.progress(
                log.reporter(timer, self.config['fancy_progress']))
            succeeded, failed = update_masters.run()
            if failed:
                self.get_logger().warning(
                    '%d masters had sync errors', failed)
                self.soft_errors = True

    def _master_sync_command(self):
        """Synchronization command to run on the master hosts."""
        cmd = [self.get_script_path(), 'pull-master']
        if self.verbose:
            cmd.append('--verbose')
        cmd.append(socket.getfqdn())
        return cmd

    def _proxy_sync_command(self):
        """Synchronization command to run on the proxy hosts."""
        cmd = [
            self.get_script_path(),
            'pull',
            '--no-php-restart',
            '--no-update-l10n'
        ]
        if self.verbose:
            cmd.append('--verbose')
        return cmd

    def _apache_sync_command(self, proxies):
        """
        Synchronization command to run on the apache hosts.

        :param proxies: List of proxy hostnames
        """
        return self._proxy_sync_command() + proxies

    def _sync_common(self):
        """Sync stage_dir to deploy_dir on the deployment host."""
        includes = None

        if self.include is not None:
            includes = []

            parts = self.include.split('/')
            for i in range(1, len(parts)):
                # Include parent directories in sync command or the default
                # exclude will block them and by extension block the target
                # file.
                includes.append('/'.join(parts[:i]))

            includes.append(self.include)
            includes.append('php-*/cache/gitinfo')

        tasks.sync_common(
            self.config,
            include=includes,
            verbose=self.verbose
        )

    def _git_repo(self):
        """Flatten deploy directory into shared git repo."""
        if self.config['scap3_mediawiki']:
            self.get_logger().info('Setting up deploy git directory')
            cmd = '{} deploy-mediawiki -v "{}"'.format(
                self.get_script_path(), self.arguments.message)
            utils.sudo_check_call('mwdeploy', cmd)

    def _after_cluster_sync(self):
        pass

    def _after_lock_release(self):
        pass

    def sync_canary(self, canaries=None):
        """
        Sync canary hosts

        :param canaries: Iterable of canary servers to sync
        """
        if not canaries:
            return

        sync_cmd = self._apache_sync_command(self.get_master_list())

        # Go ahead and attempt to restart php for canaries
        if '--no-php-restart' in sync_cmd:
            sync_cmd.remove('--no-php-restart')

        sync_cmd.append(socket.getfqdn())

        update_canaries = ssh.Job(
            canaries,
            user=self.config['ssh_user'],
            key=self.get_keyholder_key())
        update_canaries.command(sync_cmd)
        update_canaries.progress(
            log.reporter(
                'check-canaries',
                self.config['fancy_progress']
            )
        )

        succeeded, failed = update_canaries.run()
        if failed:
            self.get_logger().warning(
                '%d canaries had sync errors', failed)
            self.soft_errors = True

    def canary_checks(self, canaries=None, timer=None):
        """
        Run canary checks

        :param canaries: Iterable of canary servers to check
        :param timer: log.Timer
        :raises RuntimeError: on canary check failure
        """
        if not canaries:
            return

        # If more than 1/4 of the canaries failed, stop deployment
        max_failed_canaries = max(len(canaries) / 4, 1)

        swagger_url = self.config['mediawiki_canary_swagger_url']
        spec_path = self.config['mediawiki_canary_swagger_spec_path']

        succeeded, failed = tasks.endpoint_canary_checks(
            canaries,
            swagger_url,
            spec_path,
            cores=utils.cpus_for_jobs(),
        )

        if failed > max_failed_canaries:
            canary_fail_msg = (
                'Scap failed!: {}/{} canaries failed their endpoint checks'
                '({})'
            ).format(
                failed,
                len(canaries),
                swagger_url
            )
            self.announce(canary_fail_msg)
            raise RuntimeError(canary_fail_msg)

        time_since_sync = 0

        if timer:
            time_since_sync = timer.mark('Canary Endpoint Check Complete')

        # Needs some time for log errors to happen
        canary_wait_time = self.config['canary_wait_time']
        remaining_wait_time = canary_wait_time - time_since_sync

        # If the canary endpoint check took less than the wait time we
        # should wait longer
        if remaining_wait_time > 0:
            self.get_logger().info('Waiting for canary traffic...')
            time.sleep(remaining_wait_time)
        # Otherwise Canary endpoint check took more than the wait time
        # we should adjust the logstash canary delay
        else:
            canary_wait_time = time_since_sync

        logstash_canary_checks = {
            'service': self.config['canary_service'],
            'threshold': self.config['canary_threshold'],
            'logstash': self.config['logstash_host'],
            'delay': canary_wait_time,
            'cores': utils.cpus_for_jobs(),
        }

        succeeded, failed = tasks.logstash_canary_checks(
            canaries, **logstash_canary_checks)

        if failed > max_failed_canaries:
            canary_fail_msg = (
                'scap failed: average error rate on {}/{} '
                'canaries increased by 10x '
                '(rerun with --force to override this check, '
                'see {} for details)'.format(
                    failed,
                    len(canaries),
                    self.config['canary_dashboard_url']))

            self.announce(canary_fail_msg)
            raise RuntimeError(canary_fail_msg)

        # If some canaries failed, explain why we didn't raise a
        # RuntimeError - T173146
        if failed > 0:
            self.get_logger().info(
                'Canary error check failed for {} canaries, less than '
                'threshold to halt deployment ({}/{}), see {} for '
                'details. Continuing...'.format(
                    failed,
                    max_failed_canaries + 1,  # + 1 since we use > to compare
                    len(canaries),
                    self.config['canary_dashboard_url']))

    def _invalidate_opcache(self, target_hosts=None, filename=None):
        """Invalidate opcache"""
        php7_admin_port = self.config.get('php7-admin-port')
        if not php7_admin_port:
            return
        if self.om is None:
            self.om = opcache_manager.OpcacheManager(php7_admin_port)
        if target_hosts:
            failed = self.om.invalidate(target_hosts, filename)
        else:
            failed = self.om.invalidate_all(self.config, filename)
        for host, reason in failed.items():
            self.get_logger().warning(
                '%s failed to update opcache: %s', host, reason)

    def _restart_php(self):
        """
        On all dsh groups referenced by the mw_web_clusters config parameter:

        Check if php-fpm opcache is full, if so restart php-fpm.  If
        the php_fpm_always_restart config parameter is true, the
        opcache is treated as always full, so php-fpm will always
        restart.

        If the operator invoked scap with the --force flag, restart
        php-fpm unsafely (i.e., without depooling and repooling
        around the service restart).  T243009

        """

        # mw_web_clusters is expected to be a comma-separated string naming dsh
        # groups.
        # target_groups will be a list of objects representing representing
        # each group.
        target_groups = targets.DirectDshTargetList(
            'mw_web_clusters',
            self.config
        )
        pool = ProcessPoolExecutor(max_workers=5)

        php_fpm.INSTANCE = php_fpm.PHPRestart(
            self.config,
            ssh.Job(
                key=self.get_keyholder_key(),
                user=self.config['ssh_user']
            ),
            self.arguments.force,
            self.get_logger()
        )

        if php_fpm.INSTANCE.cmd:
            self.get_logger().info("Running '{}' on {} host(s)".format(
                php_fpm.INSTANCE.cmd, len(target_groups.all)))

            # Convert the list of group objects into a
            # list of lists of targets.
            group_hosts = []
            for group in target_groups.groups.values():
                group_hosts.append(group.targets)

            results = pool.map(php_fpm.restart_helper, group_hosts)
            for _, failed in results:
                if failed:
                    self.get_logger().warning(
                        '%d hosts had failures restarting php-fpm',
                        failed
                    )


@cli.command('security-check')
class SecurityPatchCheck(cli.Application):
    """
    Check if security patches are applied.

    class to check if patches in ``/srv/patches`` have been applied to the
    active wikiversions
    """

    def main(self, *extra_args):
        for version in self.active_wikiversions():
            tasks.check_patch_files(version, self.config)

        return 0


@cli.command('wikiversions-compile', help=argparse.SUPPRESS)
class CompileWikiversions(cli.Application):
    """Compile wikiversions.json to wikiversions.php."""

    @cli.argument('--staging', action='store_true',
                  help='Compile wikiversions in staging directory')
    def main(self, *extra_args):
        if self.arguments.staging:
            source_tree = 'stage'
        else:
            source_tree = 'deploy'
            self._run_as('mwdeploy')
            self._assert_current_user('mwdeploy')

        tasks.compile_wikiversions(source_tree, self.config)
        return 0


@cli.command('wikiversions-inuse')
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

        print(' '.join(output))
        return 0


@cli.command('cdb-rebuild', help=argparse.SUPPRESS)
class RebuildCdbs(cli.Application):
    """Rebuild localization cache CDB files from the JSON versions."""

    @cli.argument('--version', type=arg.is_version,
                  help='MediaWiki version (eg 1.27.0-wmf.7)')
    @cli.argument('--no-progress', action='store_true', dest='mute',
                  help='Do not show progress indicator.')
    @cli.argument('--staging', action='store_true',
                  help='Rebuild cdb files in staging directory')
    def main(self, *extra_args):
        user = 'mwdeploy'
        source_tree = 'deploy'
        root_dir = self.config['deploy_dir']

        if self.arguments.staging:
            user = 'l10nupdate'
            source_tree = 'stage'
            root_dir = self.config['stage_dir']

        self._run_as(user)
        self._assert_current_user(user)

        # Leave some of the cores free for apache processes
        use_cores = utils.cpus_for_jobs()

        versions = self.active_wikiversions(source_tree)

        if self.arguments.version:
            version = self.arguments.version
            if version.startswith('php-'):
                version = version[4:]

            # Assert version is active
            if version not in versions:
                raise IOError(
                    errno.ENOENT, 'Version not active', version)

            # Replace dict of active versions with the single version selected
            versions = {version: versions[version]}

        # Rebuild the CDB files from the JSON versions
        for version in versions.keys():
            cache_dir = os.path.join(
                root_dir, 'php-%s' % version, 'cache', 'l10n')
            tasks.merge_cdb_updates(
                cache_dir, use_cores, True, self.arguments.mute)


@cli.command('sync', help='Deploy MediaWiki to the cluster (formerly scap)')
class Scap(AbstractSync):
    """
    This is getting renamed to scap sync-world. Use that instead.
    """

    @cli.argument('--force', action='store_true', help='Skip canary checks')
    @cli.argument('-w', '--canary-wait-time', dest='canary_wait_time',
                  type=int,
                  help='Define how long new code will run on the '
                       'canary servers (default is 20s)',
                  metavar='<time in secs>')
    @cli.argument('message', nargs='*', help='Log message for SAL')
    def main(self, *extra_args):
        msg = ''.join([
            ansi.esc(ansi.FG_RED, ansi.BRIGHT),
            '[ERROR] ', ansi.reset(),
            ansi.esc(ansi.FG_BLUE), '"scap sync"', ansi.reset(),
            ' has been renamed to "scap sync-world".\n'
            'Use ', ansi.esc(ansi.FG_GREEN), '"scap sync-world"', ansi.reset(),
            ' or "scap sync-file" instead,\n',
            'depending on what you want to achieve.\n'
        ])
        sys.stderr.write(msg)
        return 1


@cli.command('sync-world', help='Deploy MediaWiki to the cluster')
class ScapWorld(AbstractSync):
    """
    Deploy MediaWiki to the cluster.

    #. Validate php syntax of wmf-config and multiversion
    #. Sync deploy directory on localhost with staging area
    #. Create/update git repo in staging area
    #. Compile wikiversions.json to php in deploy directory
    #. Update l10n files in staging area
    #. Compute git version information
    #. Commit all changes to local git repo in deploy directory
    #. Ask scap masters to sync with current master
    #. Ask scap proxies to sync with master server
    #. Ask apaches to sync with fastest rsync server
    #. Ask apaches to rebuild l10n CDB files
    #. Update wikiversions.php on localhost
    #. Ask apaches to sync wikiversions.php
    #. Run refreshMessageBlobs.php
    #. Rolling invalidation of all opcache for php 7.x
    """

    @cli.argument('--force', action='store_true',
                  help='Skip canary checks, '
                  'performs ungraceful php-fpm restarts')
    @cli.argument('-w', '--canary-wait-time', dest='canary_wait_time',
                  type=int,
                  help='Define how long new code will run on the '
                       'canary servers (default is 20s)',
                  metavar='<time in secs>')
    @cli.argument('message', nargs='*', help='Log message for SAL')
    def main(self, *extra_args):
        try:
            if any('canary_wait_time' in s for s in self.arguments.defines):
                raise ValueError('Canary wait time must be defined with '
                                 '-w or --canary-wait-time')
        except TypeError:
            pass

        wait = self.arguments.canary_wait_time
        if wait is not None:
            self.config['canary_wait_time'] = wait

        return super(ScapWorld, self).main(*extra_args)

    def _before_cluster_sync(self):
        self.announce('Started scap: %s', self.arguments.message)

        # Validate php syntax of wmf-config and multiversion
        lint.check_valid_syntax(
            ['%(stage_dir)s/wmf-config' % self.config,
             '%(stage_dir)s/multiversion' % self.config],
            utils.cpus_for_jobs())

    def _after_sync_common(self):
        super(ScapWorld, self)._after_sync_common()

        # Bug 63659: Compile deploy_dir/wikiversions.json to cdb
        cmd = '{} wikiversions-compile'.format(self.get_script_path())
        utils.sudo_check_call('mwdeploy', cmd)

        # Update list of extension message files and regenerate the
        # localisation cache.
        with log.Timer('l10n-update', self.get_stats()):
            for version, wikidb in self.active_wikiversions().items():
                tasks.update_localization_cache(
                    version, wikidb, self.verbose, self.config)

    def _after_cluster_sync(self):
        target_hosts = self._get_target_list()
        # Ask apaches to rebuild l10n CDB files
        with log.Timer('scap-cdb-rebuild', self.get_stats()):
            rebuild_cdbs = ssh.Job(
                target_hosts,
                user=self.config['ssh_user'],
                key=self.get_keyholder_key())
            rebuild_cdbs.shuffle()
            rebuild_cdbs.command(
                'sudo -u mwdeploy -n -- %s cdb-rebuild' %
                self.get_script_path())
            rebuild_cdbs.progress(
                log.reporter(
                    'scap-cdb-rebuild',
                    self.config['fancy_progress']))
            succeeded, failed = rebuild_cdbs.run()
            if failed:
                self.get_logger().warning(
                    '%d hosts had scap-cdb-rebuild errors', failed)
                self.soft_errors = True

        # Update and sync wikiversions.php
        succeeded, failed = tasks.sync_wikiversions(
            target_hosts, self.config, key=self.get_keyholder_key())
        if failed:
            self.get_logger().warning(
                '%d hosts had sync_wikiversions errors', failed)
            self.soft_errors = True

        tasks.clear_message_blobs()
        self._invalidate_opcache()
        self._restart_php()

    def _after_lock_release(self):
        self.announce(
            'Finished scap: %s (duration: %s)',
            self.arguments.message, utils.human_duration(self.get_duration()))
        self.increment_stat('scap')

    def _handle_exception(self, ex):
        self.get_logger().warning('Unhandled error:', exc_info=True)
        self.announce(
            'scap failed: %s %s (duration: %s)',
            type(ex).__name__, ex, utils.human_duration(self.get_duration()))
        return 1

    def _before_exit(self, exit_status):
        if self.config:
            self.get_stats().timing('scap.scap', self.get_duration() * 1000)
        return exit_status


@cli.command('pull-master', help=argparse.SUPPRESS)
class SyncMaster(cli.Application):
    """Sync local MediaWiki staging directory with deploy server state."""

    @cli.argument('master', help='Master rsync server to copy from')
    def main(self, *extra_args):
        tasks.sync_master(
            self.config,
            master=self.arguments.master,
            verbose=self.verbose
        )
        return 0


@cli.command('pull', help='Sync local MediaWiki deployment directory with '
                          'deploy server state (formerly sync-common)')
class SyncCommon(cli.Application):
    """Sync local MediaWiki deployment directory with deploy server state."""

    @cli.argument('--no-update-l10n', action='store_false', dest='update_l10n',
                  help='Do not update l10n cache files.')
    @cli.argument('-i', '--include', default=None, action='append',
                  help='Rsync include pattern to limit transfer to.'
                       ' End directories with a trailing `/***`.'
                       ' Can be used multiple times.')
    @cli.argument('--delete-excluded', action='store_true',
                  help='Also delete local files not found on the master.')
    @cli.argument('--no-php-restart', action='store_false', dest='php_restart',
                  help='Check to see if php needs a restart')
    @cli.argument('servers', nargs=argparse.REMAINDER,
                  help='Rsync server(s) to copy from')
    def main(self, *extra_args):
        rsync_args = [
            '--delete-excluded'] if self.arguments.delete_excluded else []
        tasks.sync_common(
            self.config,
            include=self.arguments.include,
            sync_from=self.arguments.servers,
            verbose=self.verbose,
            rsync_args=rsync_args
        )
        if self.arguments.update_l10n:
            with log.Timer('scap-cdb-rebuild', self.get_stats()):
                utils.sudo_check_call(
                    'mwdeploy',
                    self.get_script_path() + ' cdb-rebuild --no-progress'
                )
        # Invalidate opcache
        # TODO deduplicate this from AbstractSync._invalidate_opcache()
        php7_admin_port = self.config.get('php7-admin-port')
        if php7_admin_port:
            om = opcache_manager.OpcacheManager(php7_admin_port)
            failed = om.invalidate([socket.gethostname()], None)
            if failed:
                self.get_logger().warning(
                    'Opcache invalidation failed. '
                    'Consider performing it manually.'
                )

        if (self.arguments.php_restart):
            fpm = php_fpm.PHPRestart(self.config)
            self.get_logger().info('Checking if php-fpm restart needed')
            failed = fpm.restart_self()
            if failed:
                self.get_logger().warning('php-fpm restart failed!')

        return 0


@cli.command('sync-dir', help=argparse.SUPPRESS)
@cli.command('sync-file')
class SyncFile(AbstractSync):
    """Sync a specific file/directory to the cluster."""

    @cli.argument('--force', action='store_true', help='Skip canary checks')
    @cli.argument('file', help='File/directory to sync')
    @cli.argument('message', nargs='*', help='Log message for SAL')
    def main(self, *extra_args):
        return super(SyncFile, self).main(*extra_args)

    def _before_cluster_sync(self):
        # assert file exists
        abspath = os.path.join(
            self.config['stage_dir'], self.arguments.file)
        if not os.path.exists(abspath):
            raise IOError(errno.ENOENT, 'File/directory not found', abspath)

        relpath = os.path.relpath(abspath, self.config['stage_dir'])
        if os.path.isdir(abspath):
            relpath = '%s/***' % relpath
        self.include = relpath

        # Notify when syncing a symlink.
        if os.path.islink(abspath):
            symlink_dest = os.path.realpath(abspath)
            self.get_logger().info("%s: syncing symlink, not its target [%s]",
                                   abspath, symlink_dest)
        else:
            lint.check_valid_syntax(abspath, utils.cpus_for_jobs())

    def _after_cluster_sync(self):
        self._invalidate_opcache(None, self.arguments.file)
        self._restart_php()

    def _proxy_sync_command(self):
        cmd = [
            self.get_script_path(),
            'pull',
            '--no-update-l10n',
            '--no-php-restart'
        ]

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
        self.announce(
            'Synchronized %s: %s (duration: %s)',
            self.arguments.file, self.arguments.message,
            utils.human_duration(self.get_duration()))
        self.increment_stat('sync-file')


@cli.command('sync-l10n')
class SyncL10n(AbstractSync):
    """Sync l10n files for a given branch and rebuild cache files."""

    @cli.argument('--force', action='store_true', help='Skip canary checks')
    @cli.argument('version', type=arg.is_version,
                  help='MediaWiki version (eg 1.27.0-wmf.7)')
    @cli.argument('message', nargs='*', help='Log message for SAL')
    def main(self, *extra_args):
        return super(SyncL10n, self).main(*extra_args)

    def _before_cluster_sync(self):
        if self.arguments.version.startswith('php-'):
            self.arguments.version = self.arguments.version[4:]

        # Assert version is active
        if self.arguments.version not in self.active_wikiversions():
            raise IOError(
                errno.ENOENT, 'Version not active', self.arguments.version)

        # Assert l10n cache dir for version exists
        abspath = os.path.join(
            self.config['stage_dir'],
            'php-%s/cache/l10n' % self.arguments.version)
        if not os.path.isdir(abspath):
            raise IOError(errno.ENOENT, 'Directory not found', abspath)

        relpath = os.path.relpath(abspath, self.config['stage_dir'])
        self.include = '%s/***' % relpath

    def _proxy_sync_command(self):
        cmd = [
            self.get_script_path(),
            'pull',
            '--no-update-l10n',
            '--no-php-restart'
        ]

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

    def _after_cluster_sync(self):
        # Rebuild l10n CDB files
        target_hosts = self._get_target_list()
        with log.Timer('scap-cdb-rebuild', self.get_stats()):
            rebuild_cdbs = ssh.Job(
                target_hosts,
                user=self.config['ssh_user'],
                key=self.get_keyholder_key())
            rebuild_cdbs.shuffle()
            cdb_cmd = 'sudo -u mwdeploy -n -- {} cdb-rebuild --version {}'
            cdb_cmd = cdb_cmd.format(
                self.get_script_path(),
                self.arguments.version
            )
            rebuild_cdbs.command(cdb_cmd)
            rebuild_cdbs.progress(
                log.reporter(
                    'scap-cdb-rebuild',
                    self.config['fancy_progress']))
            succeeded, failed = rebuild_cdbs.run()
            if failed:
                self.get_logger().warning(
                    '%d hosts had scap-cdb-rebuild errors', failed)
                self.soft_errors = True
        tasks.clear_message_blobs()
        # Globally invalidate opcache. TODO: is this needed?
        self._invalidate_opcache(target_hosts)
        self._restart_php()

    def _after_lock_release(self):
        self.announce(
            'scap sync-l10n completed (%s) (duration: %s)',
            self.arguments.version, utils.human_duration(self.get_duration()))
        self.increment_stat('l10nupdate-sync')

    def _after_sync_common(self):
        self._git_repo()


@cli.command('sync-wikiversions')
class SyncWikiversions(AbstractSync):
    """Rebuild and sync wikiversions.php to the cluster."""

    def _after_sync_common(self):
        """
        Skip this step.

        It currently consists only of cache_git_info and this class should
        attempt to be fast where possible.
        """
        pass

    def _before_cluster_sync(self):
        """
        check for the presence of ExtensionMessages and l10n cache
        for every branch of mediawiki that is referenced in wikiversions.json
        to avoid syncing a branch that is lacking these critical files.
        """
        for version in self.active_wikiversions().keys():
            ext_msg = os.path.join(
                self.config['stage_dir'],
                'wmf-config', 'ExtensionMessages-%s.php' % version)
            err_msg = 'ExtensionMessages not found in %s' % ext_msg
            utils.check_file_exists(ext_msg, err_msg)

            cache_file = os.path.join(
                self.config['stage_dir'],
                'php-%s' % version, 'cache', 'l10n', 'l10n_cache-en.cdb')
            err_msg = 'l10n cache missing for %s' % version
            utils.check_file_exists(cache_file, err_msg)

        # Compile mediawiki-staging wikiversions
        tasks.compile_wikiversions('stage', self.config)
        self.include = 'wikiversions*.*'

    def _after_lock_release(self):
        self.announce(
            'rebuilt and synchronized wikiversions files: %s',
            self.arguments.message
        )

        self.increment_stat('sync-wikiversions')

    def _after_cluster_sync(self):
        self._invalidate_opcache(None, 'wikiversions.php')
        self._restart_php()


@cli.command('cdb-json-refresh', help=argparse.SUPPRESS)
class RefreshCdbJsonFiles(cli.Application):
    """
    Create JSON/MD5 files for all CDB files in a directory.

    This will put a JSON and MD5 file in /upstream for each CDB file.

    This can be combined with rsync and the scap-rebuild-cdbs to
    push out large CDB files with minimal traffic. CDB files change
    drastically with small key/value changes, where as JSON files do not, and
    thus they diff/rdiff much better.

    When pushing updates with rsync, this should be run before running rsync.
    The rsync command should exclude CDB files or at least use
    -ignore-existing. After the rsync is done, scap-rebuild-cdbs can be
    run on each server to apply the updates to the CDB files.
    """

    @cli.argument('-d', '--directory', required=True,
                  type=arg.is_dir,
                  help='Directory containing cdb files')
    @cli.argument('-t', '--threads', default=1, type=int,
                  help='Number of threads to use to build json/md5 files')
    def main(self, *extra_args):
        cdb_dir = os.path.realpath(self.arguments.directory)
        upstream_dir = os.path.join(cdb_dir, 'upstream')
        use_cores = self.arguments.threads

        if not os.path.isdir(cdb_dir):
            raise IOError(errno.ENOENT, 'Directory does not exist', cdb_dir)

        if use_cores < 1:
            use_cores = utils.cpus_for_jobs()

        if not os.path.isdir(upstream_dir):
            os.mkdir(upstream_dir)

        tasks.refresh_cdb_json_files(cdb_dir, use_cores, self.verbose)


@cli.command('version', help='Show the version number and exit')
class Version(cli.Application):
    def main(self, *extra_args):
        print(scapversion.__version__)
        return 0


@cli.command('lock', help='Temporarily lock deployment of this repository')
class LockManager(cli.Application):
    """
    Holds a lock open for a given repository.

    examples::

        lock 'Testing something, do not deploy'
    """

    @cli.argument('--all', action='store_true',
                  help='Lock ALL repositories from deployment. ' +
                       'With great power comes great responsibility')
    @cli.argument('--time', type=int, default=3600,
                  help='How long to lock deployments, in seconds')
    @cli.argument('message', nargs='*', help='Log message for SAL/lock file')
    def main(self, *extra_args):
        logger = self.get_logger()

        if self.arguments.message == '(no justification provided)':
            logger.fatal('Cannot lock repositories without a reason')
            return 1

        if self.arguments.all:
            lock_path = lock.GLOBAL_LOCK_FILE
            repo = 'ALL REPOSITORIES'
        else:
            lock_path = self.get_lock_file()
            repo = self.config['git_repo']

        got_lock = False
        with lock.Lock(lock_path, self.arguments.message, group_write=True):
            got_lock = True
            self.announce(
                'Locking from deployment [%s]: %s (planned duration: %s)',
                repo, self.arguments.message,
                utils.human_duration(self.arguments.time))

            logger.info('Press enter to abort early...')
            try:
                rlist, _, _ = select.select([sys.stdin], [], [],
                                            self.arguments.time)
                if rlist:
                    sys.stdin.readline()
            except KeyboardInterrupt:
                pass  # We don't care here

        if got_lock:
            self.announce(
                'Unlocked for deployment [%s]: %s (duration: %s)', repo,
                self.arguments.message,
                utils.human_duration(self.get_duration()))

        return 0


class PatchBase(cli.Application):
    """Common code for list-patches, test-patches, and apply-patches"""

    @cli.argument('--train', action='store',
                  help='train version number (e.g., "1.35.0.wmf30")',
                  metavar='TRAIN', required=True)
    def main(self, *extra_args):
        return self.real_main(*extra_args)

    def real_main(self, *extra_args):
        raise NotImplementedError()

    def codedir(self):
        """Return the full path to the train-specific staging directory"""
        code = self.config["stage_dir"]
        return os.path.abspath(
            os.path.join(code, 'php-{}'.format(self.arguments.train)))

    def patchdir(self):
        """Return the full path to the train-specific patches directory"""
        patch = self.config["patch_path"]
        return os.path.abspath(os.path.join(patch, self.arguments.train))

    def patch_files(self):
        """Return valid patches or exit with error"""
        patches = self.patchdir()
        if not os.path.exists(patches):
            sys.exit("Patch directory {} does not exist".format(patches))

        filenames = utils.find_regular_files(patches)
        invalid = self.invalid_patch_names(filenames)
        if invalid:
            for filename in invalid:
                sys.stderr.write(
                    'Bad filename for patch: {}\n'.format(filename))
            sys.exit(1)
        return filenames

    def invalid_patch_names(self, filenames):
        """Return any filenames that aren't correct for patch files"""
        return [x for x in filenames if not x.endswith('.patch')]

    def map_patches(self, func, codedir, patchdir, patchfiles):
        """Call func for every patch file, in the right directory

        func will probably run git apply or git am on the patch file. func
        gets patch filename and the directory it should be applied in as
        arguments.

        If patch is for core, call func with the core sub-directory of
        codedir. Similarly, call func with an extensions's sub-directory.
        The extension's name is parsed from the patch file name.

        """
        for patchfile in patchfiles:
            filename = os.path.join(patchdir, patchfile)
            if patchfile.startswith('core/'):
                func(filename, codedir)
            elif patchfile.startswith('extensions/'):
                # patchfile looks like extensions/GoatExt/01-T123.patch
                # Split on /, take the second element as the extension
                # name, to construct the path to the extension directory.
                extension = patchfile.split('/')[1]
                extdir = os.path.join(codedir, 'extensions', extension)
                func(filename, extdir)
            else:
                sys.exit('Cannot understand patch file {}'.format(patchfile))

    def git_apply_check(self, patchfile, dirname):
        """Check if a patch applies cleanly"""
        with utils.cd(dirname):
            return sh.git('apply', '--no-3way', '--check', patchfile)

    def git_am(self, patchfile, dirname):
        """Apply a patch"""
        with utils.cd(dirname):
            ret = sh.git('apply', '--no-3way', '--check', patchfile)
            if ret.exit_code != 0:
                return ret
            return sh.git('am', '--no-3way', patchfile)


@cli.command('list-patches', help='List pending security patches for train')
class ListPatches(PatchBase):
    @cli.argument('--train', action='store',
                  help='train version number (e.g., "1.35.0.wmf30")',
                  metavar='TRAIN', required=True)
    def main(self, *extra_args):
        for filename in self.patch_files():
            print(filename)
        return 0


@cli.command(
    'test-patches',
    help='Check that pending security patches for train can be applied')
class TestPatches(PatchBase):
    @cli.argument('--train', action='store',
                  help='train version number (e.g., "1.35.0.wmf30")',
                  metavar='TRAIN', required=True)
    def main(self, *extra_args):
        filenames = self.patch_files()
        code = self.codedir()
        patches = self.patchdir()
        self.map_patches(self.git_apply_check, code, patches, filenames)
        return 0


@cli.command('apply-patches', help='Apply security patches for train')
class ApplyPatches(PatchBase):
    @cli.argument('--train', action='store',
                  help='train version number (e.g., "1.35.0.wmf30")',
                  metavar='TRAIN', required=True)
    def main(self, *extra_args):
        filenames = self.patch_files()
        code = self.codedir()
        patches = self.patchdir()
        self.map_patches(self.git_am, code, patches, filenames)
        return 0
