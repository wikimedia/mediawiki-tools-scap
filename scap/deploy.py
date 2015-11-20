# -*- coding: utf-8 -*-
"""
    scap.deploy
    ~~~~~~~~~~~
    Command wrappers for deploy tasks

"""
import collections
import glob
import hashlib
import os
import requests
import shutil
import time
import yaml
import errno
from datetime import datetime

from . import checks
from . import nrpe
from . import template
from . import cli
from . import log
from . import ssh
from . import tasks
from . import utils

REV_DELIMITER = '_'


class DeployApplication(cli.Application):
    """Common base class for all deployments"""

    def _load_config(self):
        """Initializes commonly used attributes after the config is loaded."""

        super(DeployApplication, self)._load_config()

        # We don't want to enforce a directory structure where
        # config['git_deploy_dir'] has to be the root of both the deploy-host
        # and deploy-target of the application. The 'git_deploy_dir' is the
        # place on the targets in which your repo is placed and should have no
        # impact on the deployer. T116207
        self.root_dir = os.getcwd()
        self.scap_dir = os.path.join(self.root_dir, 'scap')
        self.log_dir = os.path.join(self.scap_dir, 'log')


class DeployLocal(DeployApplication):
    """Command that runs on target hosts. Responsible for fetching code from
    the git server, checking out the appropriate revisions, restarting services
    and running checks.
    """
    STAGES = ['config_deploy', 'fetch', 'promote']
    EX_STAGES = ['rollback']

    rev = None
    cache_dir = None
    revs_dir = None
    rev_dir = None
    cur_link = None
    progress_flag = None
    done_flag = None
    user = None

    @cli.argument('-g', '--group',
                  help='Group of which this local machine is a part')
    @cli.argument('stage', metavar='STAGE', choices=STAGES + EX_STAGES,
                  help='Stage of the deployment to execute')
    @cli.argument('-f', '--force', action='store_true',
                  help='force stage even when noop detected')
    def main(self, *extra_args):
        self.root_dir = os.path.join(self.config['git_deploy_dir'],
                                     self.config['git_repo'])

        self.rev = self.config['git_rev']

        # cache, revs, and current directory go under [repo]-cache and are
        # linked to [repo] as a final step
        root_deploy_dir = '{}-cache'.format(self.root_dir)

        def deploy_dir(subdir):
            return os.path.join(root_deploy_dir, subdir)

        self.cache_dir = deploy_dir('cache')
        self.revs_dir = deploy_dir('revs')
        self.tmp_dir = deploy_dir('tmp')
        self.cfg_digest = os.path.join(self.tmp_dir, '.config-digest')
        self.noop = False

        rev_dir = self.rev
        try:
            with open(self.cfg_digest, 'r') as f:
                rev_dir = REV_DELIMITER.join([f.read(), self.rev])
        except IOError:
            pass

        self.rev_dir = os.path.join(self.revs_dir, rev_dir)

        self.cur_link = deploy_dir('current')
        self.progress_flag = deploy_dir('.in-progress')
        self.done_flag = deploy_dir('.done')

        self.user = self.config['git_repo_user']

        # only supports http from tin for the moment
        scheme = 'http'
        repo = self.config['git_repo']
        server = self.config['git_server']

        url = os.path.normpath('{0}/{1}'.format(server, repo))

        self.server_url = "{0}://{1}".format(scheme, url)

        stage = self.arguments.stage
        group = self.arguments.group

        getattr(self, stage)()

        status = 0

        if not self.noop and self.config['perform_checks']:
            status = self._execute_checks(stage, group)

        # Perform final tasks after the last stage
        if status == 0 and self.STAGES[-1] == stage:
            self._finalize()

        return status

    def config_deploy(self):
        """Renders config files

        Grabs the current config yaml file from the deploy git server, and
        renders the final template inside the repo-cache's tmp directory
        """
        logger = self.get_logger()
        if not self.config['config_deploy']:
            return

        config_url = os.path.join(self.server_url, '.git', 'config-files',
                                  '{}.yaml'.format(self.rev))

        logger.debug('Get config yaml: {}'.format(config_url))
        r = requests.get(config_url)
        if r.status_code != requests.codes.ok:
            raise IOError(errno.ENOENT, 'Config file not found', config_url)

        config_files = yaml.load(r.text)
        overrides = config_files.get('override_vars', {})

        source_basepath = os.path.join(self.tmp_dir, self.rev, 'config-files')
        logger.debug('Source basepath: {}'.format(source_basepath))
        utils.mkdir_p(source_basepath)
        config_file_tree = {}

        for config_file in config_files['files']:
            name = config_file['name']
            tmpl = template.Template(
                name=name,
                loader={name: config_file['template']},
                var_file=config_file.get('remote_vars', None),
                overrides=overrides
            )

            filename = config_file['name']
            if filename.startswith('/'):
                filename = filename[1:]

            utils.mkdir_p(os.path.join(
                source_basepath, os.path.dirname(filename)))

            source = os.path.join(source_basepath, filename)
            logger.debug('Rendering config_file: {}'.format(source))

            with open(source, 'w') as f:
                output_file = tmpl.render()
                s = hashlib.sha1()
                s.update('blob {}\0'.format(len(output_file)))
                s.update(output_file)
                config_file_tree[source] = s.hexdigest()
                f.write(output_file)

        with open(self.cfg_digest, 'w') as f:
            s = hashlib.sha1()
            s.update(repr(config_file_tree))
            digest = s.hexdigest()
            if os.path.isdir(self.cur_link):
                current = os.path.basename(os.path.realpath(self.cur_link))
                if '_' in current:
                    if (digest == current.split(REV_DELIMITER)[0] and
                            not self.arguments.force):
                        # Even when a noop is detected, we still write the
                        # digest file: it is required to determine the
                        # rev_dir in all future steps
                        logger.info('Config already deployed '
                                    '(use --force to override)')
                        self.noop = True
            f.write(digest)

    def fetch(self):
        """Fetch the specified revision of the remote repo.

        The given repo is cloned into the cache directory and a new working
        directory for the given revision is created under revs/{rev}.

        At the end of this stage, the .in-progress link is created to signal
        the possibility for future rollback.
        """
        has_submodules = self.config['git_submodules']
        logger = self.get_logger()

        # create deployment directories if they don't already exist
        for d in [self.cache_dir, self.revs_dir]:
            utils.mkdir_p(d)

        git_remote = os.path.join(self.server_url, '.git')
        logger.debug('Fetching from: {}'.format(git_remote))

        # clone/fetch from the repo to the cache directory
        tasks.git_fetch(self.cache_dir, git_remote, user=self.user)

        # If the rev_dir already exists AND the currently checked-out HEAD is
        # already at the revision specified by ``self.rev`` then you can assume
        #
        # 1. If there is a config deploy, the config is inside the rev_dir
        # 2. The code represented by the SHA1 to be deployed is inside
        #    the rev_dir
        #
        # Set the noop flag and return
        if os.path.isdir(self.rev_dir) and not self.arguments.force:
            rev = utils.git_sha(self.rev_dir, 'HEAD')
            if rev == self.rev:
                logger.info('Revision directory already exists '
                            '(use --force to override)')
                self.noop = True
                return

        # clone/fetch from the local cache directory to the revision directory
        tasks.git_fetch(self.rev_dir, self.cache_dir, user=self.user)

        # checkout the given revision
        tasks.git_checkout(self.rev_dir, self.rev, user=self.user)

        if has_submodules:
            upstream_submodules = self.config['git_upstream_submodules']
            tasks.git_update_submodules(self.rev_dir, git_remote,
                                        use_upstream=upstream_submodules,
                                        user=self.user)

        # link the .in-progress flag to the rev directory
        self._link_rev_dir(self.progress_flag)

    def promote(self):
        """Promote the current deployment.

        Switches the `current` symlink to the current revision directory and
        restarts the configured service.

        Probes the configured service port to measure whether it successfully
        restarted.
        """

        service = self.config.get('service_name', None)
        logger = self.get_logger()

        if (os.path.realpath(self.cur_link) == self.rev_dir and
                not self.arguments.force):
            logger.info('{} is already live '
                        '(use --force to override)'.format(self.rev_dir))
            self.noop = True
            return

        self._link_rev_dir(self.cur_link)
        self._link_final_to_current()

        if self.config['config_deploy']:
            self._link_config_files()

        if service is not None:
            tasks.restart_service(service, user=self.config['git_repo_user'])

            port = self.config.get('service_port', None)

            if port is not None:
                timeout = float(self.config['service_timeout'])
                tasks.check_port(int(port), timeout=timeout)

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

        try:
            rev = rev.split(REV_DELIMITER)[1]
        except IndexError:
            # Don't blow up if there was no config deployed last time
            pass

        if not os.path.isdir(rev_dir):
            msg = 'rollback failed due to missing rev directory {}'
            raise RuntimeError(msg.format(rev_dir))

        logger.info('Rolling back to revision {}'.format(rev))
        self.rev = rev
        self.rev_dir = rev_dir

        # Config re-evaluation no longer necessary or desirable at this point
        self.config['config_deploy'] = False
        self.promote()
        self._finalize()

    @utils.log_context('checks')
    def _execute_checks(self, stage, group=None, logger=None):
        """Fetches and executes all checks configured for the given stage.

        Checks are retrieved from the remote deploy host and cached within
        tmp.
        """

        # Load NRPE checks
        if os.path.isdir(self.config['nrpe_dir']):
            nrpe.register(nrpe.load_directory(self.config['nrpe_dir']))

        checks_url = os.path.join(self.server_url, 'scap', 'checks.yaml')
        response = requests.get(checks_url)

        if response.status_code != requests.codes.ok:
            raise IOError(errno.ENOENT, 'Error downloading checks', checks_url)

        chks = checks.load(response.text)
        chks = [
            chk for chk in chks.values() if self._valid_chk(chk, stage, group)
        ]

        success, done = checks.execute(chks, logger=logger)
        failed = [job.check.name for job in done if job.isfailure()]

        if success:
            return 0
        else:
            return 1 if len(failed) else 2

    def _valid_chk(self, chk, stage, group):
        """Make sure a check is valid for our current group"""
        if group is not None:
            return chk.stage == stage and (chk.group == group or
                                           chk.group is None)
        else:
            return chk.stage == stage

    def _finalize(self):
        """Performs the final deploy actions.

        Moves the .done flag to the rev directory and removes the .in-progress
        flag.
        """

        self._link_rev_dir(self.done_flag)
        self._cleanup()

    def _link_final_to_current(self):
        """Link the current checkout to final location at [repo]

        This should really only be needed the first time that scap03 is
        run. It links the [repo]-cache/current directory to [repo].
        """
        timestamp = datetime.utcnow()
        date = timestamp.isoformat()

        # if the path is not a symlink, but it does exist, move it
        if (not os.path.islink(self.root_dir) and
                (os.path.isfile(self.root_dir) or
                 os.path.isdir(self.root_dir))):
            os.rename(self.root_dir, '{}.{}'.format(self.root_dir, date))

        tasks.move_symlink(self.cur_link, self.root_dir, user=self.user)

    def _link_config_files(self):
        """Moves rendered configs inside the current checkout, then links
        to final destination
        """
        logger = self.get_logger()

        config_base = os.path.join(self.cur_link, '.git', 'config-files')

        if os.path.isdir(config_base):
            shutil.rmtree(config_base)

        os.rename(
            os.path.join(self.tmp_dir, self.rev, 'config-files'),
            config_base
        )

        logger.debug('Linking config files at: {}'.format(config_base))

        for dir_path, _, conf_files in os.walk(config_base):
            for conf_file in conf_files:
                full_path = os.path.normpath(
                    '{}/{}'.format(dir_path, conf_file))

                rel_path = os.path.relpath(full_path, config_base)
                final_path = os.path.join('/', rel_path)
                tasks.move_symlink(full_path, final_path, user=self.user)

    def _link_rev_dir(self, symlink_path):
        tasks.move_symlink(self.rev_dir, symlink_path, user=self.user)

    def _cleanup(self):
        if not self.noop:
            self._remove_progress_link()
        self._remove_config_digest()
        self._clean_old_revs()

    def _remove_progress_link(self):
        tasks.remove_symlink(self.progress_flag, user=self.user)

    def _remove_config_digest(self):
        if os.path.exists(self.cfg_digest):
            os.unlink(self.cfg_digest)

    def _clean_old_revs(self):
        """If there are more than 5 directories in self.revs_dir, remove
        the oldest
        """
        logger = self.get_logger()
        with utils.cd(self.revs_dir):
            # get list of top-level directories in revs_dir
            dirs = os.walk('.').next()[1]

            if len(dirs) <= 5:
                return

            sorted_dirs = sorted(dirs, key=os.path.getctime, reverse=True)
            oldest = os.path.abspath(sorted_dirs.pop())

            # if *somehow* the oldest directory is the current deployment
            # directory, don't delete it
            if oldest == self.rev_dir:
                return

            logger.info('Cleaning old revision {}'.format(oldest))
            shutil.rmtree(oldest)


class Deploy(DeployApplication):
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
        'nrpe_dir',
        'git_upstream_submodules',
        'service_name',
        'service_port',
        'service_timeout',
        'config_deploy',
        'perform_checks',
    ]

    repo = None
    targets = []

    @cli.argument('-r', '--rev', default='HEAD', help='Revision to deploy')
    @cli.argument('-s', '--stages', choices=DeployLocal.STAGES,
                  help='Deployment stages to execute. Used only for testing.')
    @cli.argument('-l', '--limit-hosts', default='all',
                  help='Limit deploy to hosts matching expression')
    @cli.argument('-f', '--force', action='store_true',
                  help='force re-fetch and checkout')
    def main(self, *extra_args):
        logger = self.get_logger()

        self.repo = self.config['git_repo']

        cwd = os.getcwd()

        if self.arguments.stages:
            stages = self.arguments.stages.split(',')
        else:
            stages = DeployLocal.STAGES

        if not utils.is_git_dir(cwd):
            raise RuntimeError(errno.EPERM, 'Script must be run from git repo')

        self._build_deploy_groups()

        if not len(self.all_targets):
            logger.warn('No targets selected, check limits and dsh_targets')
            return 1

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

                self.config_deploy_setup(commit)

                self.config['git_rev'] = commit

                # Run git update-server-info because git repo is a dumb
                # apache server
                tasks.git_update_server_info(self.config['git_submodules'])

                return self._execute_for_groups(stages)
        return 0

    def _execute_for_groups(self, stages):
        logger = self.get_logger()

        for group, targets in self.deploy_groups.iteritems():
            if not len(targets):
                continue

            logger.info('\n== {0} ==\n:* {1}'.format(
                group.upper(),
                '\n:* '.join(targets)
            ))

            for stage in stages:
                ret = self.execute_stage_on_group(stage, group, targets)
                if ret > 0:
                    self.execute_rollback(stage, group, targets)
                    return ret

            prompt = '{} deploy successful. Continue?'.format(group)

            if not self._last_group(group) and utils.ask(prompt, 'y') != 'y':
                break

        return 0

    def _last_group(self, group):
        return group == next(reversed(self.deploy_groups))

    def config_deploy_setup(self, commit):
        """Generate environment-specific config file and variable template list

        Builds a yaml file that contains:
        #. A list of file objects containing template files to be deployed
        #. An object containing variables specified in the
        environment-specific `vars.yaml` file and inheriting from the
        `vars.yaml` file
        """
        logger = self.get_logger()

        if not self.config['config_deploy']:
            return

        logger.debug('Deploy config: True')

        cfg_file = utils.get_env_specific_filename(
            os.path.join(self.scap_dir, 'config-files.yaml'),
            self.arguments.environment
        )

        logger.debug('Config deploy file: {}'.format(cfg_file))
        if not os.path.isfile(cfg_file):
            return

        config_file_path = os.path.join(self.root_dir, '.git', 'config-files')
        utils.mkdir_p(config_file_path)
        tmp_cfg_file = os.path.join(config_file_path, '{}.yaml'.format(commit))
        tmp_cfg = {}

        with open(cfg_file, 'r') as cf:
            config_files = yaml.load(cf.read())

        tmp_cfg['files'] = []
        # Get an environment specific template
        for config_file in config_files:
            f = {}
            f['name'] = config_file
            template_name = config_files[config_file]['template']
            template = utils.get_env_specific_filename(
                os.path.join(self.scap_dir, 'templates', template_name),
                self.arguments.environment
            )
            with open(template, 'r') as tmp:
                f['template'] = tmp.read()

            # Remote var file is optional
            if config_files[config_file].get('remote_vars', None):
                f['remote_vars'] = config_files[config_file]['remote_vars']

            tmp_cfg['files'].append(f)

        tmp_cfg['override_vars'] = {}

        # Build vars to override remote
        default_vars = os.path.join(self.scap_dir, 'vars.yaml')
        vars_files = [
            default_vars,
            utils.get_env_specific_filename(
                default_vars,
                self.arguments.environment
            ),
        ]

        for vars_file in vars_files:
            try:
                with open(vars_file, 'r') as vf:
                    tmp_cfg['override_vars'].update(yaml.load(vf.read()))
            except IOError:
                pass  # don't worry if a vars.yaml doesn't exist

        with open(tmp_cfg_file, 'w') as tc:
            yaml.dump(tmp_cfg, tc)
            logger.debug('Wrote config deploy file: {}'.format(tmp_cfg_file))

    def _build_deploy_groups(self):
        """Build server groups based on configuration `server_groups` variable
        """
        groups = collections.OrderedDict()
        all_hosts = []
        server_groups = self.config.get('server_groups', None)

        if server_groups is None:
            server_groups = ['default']
        else:
            server_groups = server_groups.split(',')

        for group in server_groups:
            group = group.strip()
            dsh_key = '{}_dsh_targets'.format(group)
            limit = False
            if group == 'default':
                limit = True
                dsh_key = 'dsh_targets'

            dsh_file = self.config.get(dsh_key, None)

            if dsh_file is None:
                raise RuntimeError('Reading `{0}` file "{1}" failed'.format(
                                   dsh_key, dsh_file))

            env = self.arguments.environment
            if env and os.path.isdir(env):
                search_path = [os.path.join(self.scap_dir, env)]
            else:
                search_path = []

            search_path.extend([self.scap_dir, "/etc/dsh/group"])
            targets = utils.read_hosts_file(dsh_file, search_path)
            if limit and self.arguments.limit_hosts is not None:
                targets = utils.get_target_hosts(self.arguments.limit_hosts,
                                                 targets)

            targets = list(set(targets) - set(all_hosts))
            all_hosts += targets
            groups[group] = targets

        self.all_targets = all_hosts
        self.deploy_groups = groups

    def execute_rollback(self, stage, group, targets):
        prompt = "Stage '{}' failed on group '{}'. Perform rollback?".format(
            stage, group)

        if utils.ask(prompt, 'y') == 'y':
            return self.execute_stage_on_group('rollback', group, targets)

        return 0

    def execute_stage_on_group(self, stage, group, targets):
        logger = self.get_logger()
        deploy_local_cmd = [self.get_script_path('deploy-local')]
        batch_size = self._get_batch_size(stage)

        config = {
            key: self.config.get(key)
            for key in self.DEPLOY_CONF
            if self.config.get(key) is not None
        }

        # Handle JSON output from deploy-local
        config['log_json'] = True
        deploy_local_cmd.append('-v')

        # Be sure to skip checks if they aren't configured
        if not os.path.exists(os.path.join(self.scap_dir, 'checks.yaml')):
            config['perform_checks'] = False

        for key, value in config.iteritems():
            deploy_local_cmd.extend(['-D', '{}:{}'.format(key, value)])

        if self.arguments.force:
                deploy_local_cmd.append('--force')

        deploy_local_cmd += ['-g', group, stage]

        logger.debug('Running remote deploy cmd {}'.format(deploy_local_cmd))

        deploy_stage = ssh.Job(
            hosts=targets, user=self.config['ssh_user'])
        deploy_stage.output_handler = ssh.JSONOutputHandler
        deploy_stage.max_failure = self.MAX_FAILURES
        deploy_stage.command(deploy_local_cmd)
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

    def _setup_loggers(self):
        """Sets up additional logging to `scap/deploy.log`."""

        if not os.path.exists(self.log_dir):
            os.mkdir(self.log_dir)

        basename = utils.git_describe(self.root_dir).replace('/', '-')
        log_file = os.path.join(self.log_dir, '{}.log'.format(basename))
        log.setup_loggers(self.config,
                          self.arguments.loglevel,
                          handlers=[log.DeployLogHandler(log_file)])


class DeployLog(DeployApplication):
    """Tail/filter/output events from the deploy logs

    examples::

        deploy-log -v
        deploy-log 'host == scap-target-01'
        deploy-log 'msg ~ "some important (message|msg)"'
        deploy-log 'levelno >= WARNING host == scap-target-*'
    """

    DATE_FORMAT = '%H:%M:%S'
    DIR_SCAN_DELAY = 1
    FORMAT = '%(asctime)s [%(host)s] %(message)s'

    @cli.argument('expr', metavar='EXPR', nargs='?', default='',
                  help='Filter expression.')
    @cli.argument('-f', '--file', metavar='FILE', default=None,
                  help='Parse and filter an existing log file')
    @cli.argument('-l', '--latest', action='store_true',
                  help='Parse and filter the latest log file')
    def main(self, *extra_args):
        if self.arguments.latest:
            given_log = self._latest_log_file()
        else:
            given_log = self.arguments.file

        filter = log.Filter.loads(self.arguments.expr, filter=False)

        if not filter.isfiltering('levelno'):
            filter.append({'levelno': lambda v: v >= self.arguments.loglevel})

        formatter = log.AnsiColorFormatter(self.FORMAT, self.DATE_FORMAT)

        if not os.path.exists(self.log_dir):
            os.mkdir(self.log_dir)

        cur_log_path = given_log
        cur_log_file = open(given_log, 'r') if given_log else None
        last_scan = 0

        # How we do:
        #  1. read the next line from the current file
        #  2. if there's output, parse the line, match it, print it
        #  3. if there's no output, scan the log directory for a new file
        #  4. if a newer log file is found, open it instead
        #  5. repeat
        while True:
            line = None
            if cur_log_file:
                line = cur_log_file.readline()

            if line:
                try:
                    record = log.JSONFormatter.make_record(line)
                    if filter.filter(record):
                        print formatter.format(record)

                except (ValueError, TypeError):
                    pass
            else:
                if given_log:
                    # we were given a file and there's nothing more to read
                    break

                now = time.time()

                if (now - last_scan) > self.DIR_SCAN_DELAY:
                    last_scan = now
                    log_path = self._latest_log_file()

                    if log_path and log_path != cur_log_path:
                        print "-- Opening log file: '{}'".format(log_path)
                        cur_log_path = log_path

                        if cur_log_file:
                            cur_log_file.close()

                        cur_log_file = open(cur_log_path, 'r')
                else:
                    time.sleep(0.1)

        if cur_log_file:
            cur_log_file.close()

        return 0

    def _latest_log_file(self):
        log_glob = os.path.join(self.log_dir, '*.log')

        try:
            return max(glob.iglob(log_glob), key=os.path.getmtime)
        except ValueError:
            return None

    def _setup_loggers(self):
        pass
