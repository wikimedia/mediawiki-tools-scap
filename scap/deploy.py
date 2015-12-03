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
from . import context
from . import nrpe
from . import template
from . import cli
from . import log
from . import ssh
from . import tasks
from . import utils
from . import git

STAGES = ['config_deploy', 'fetch', 'promote']
EX_STAGES = ['rollback']


class DeployLocal(cli.Application):
    """Command that runs on target hosts. Responsible for fetching code from
    the git server, checking out the appropriate revisions, restarting services
    and running checks.
    """

    @cli.argument('-g', '--group',
                  help='Group of which this local machine is a part')
    @cli.argument('stage', metavar='STAGE', choices=STAGES + EX_STAGES,
                  help='Stage of the deployment to execute')
    @cli.argument('-f', '--force', action='store_true',
                  help='force stage even when noop detected')
    def main(self, *extra_args):
        self.rev = self.config['git_rev']
        self.user = self.config['git_repo_user']
        self.noop = False
        self.final_path = os.path.join(self.config['git_deploy_dir'],
                                       self.config['git_repo'])

        root = '{}-cache'.format(self.final_path)
        self.context = context.TargetContext(root, user=self.user)
        self.context.setup()

        # only supports http from tin for the moment
        url = os.path.normpath('{git_server}/{git_repo}'.format(**self.config))
        self.server_url = 'http://{0}'.format(url)

        stage = self.arguments.stage
        group = self.arguments.group

        getattr(self, stage)()

        status = 0

        if not self.noop and self.config['perform_checks']:
            status = self._execute_checks(stage, group)

        # Perform final tasks after the last stage
        if status == 0 and STAGES[-1] == stage:
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

        source_basepath = self.context.temp_path(self.rev, 'config-files')
        logger.debug('Source basepath: {}'.format(source_basepath))
        utils.mkdir_p(source_basepath)
        config_file_tree = {}

        for config_file in config_files['files']:
            filename = config_file['name']

            tmpl = template.Template(
                name=filename,
                loader={filename: config_file['template']},
                erb_syntax=config_file.get('erb_syntax', False),
                var_file=config_file.get('remote_vars', None),
                overrides=overrides
            )

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

        s = hashlib.sha1()
        s.update(repr(config_file_tree))
        digest = s.hexdigest()

        if digest == self.context.current_config_rev:
            if not self.arguments.force:
                logger.info('Config already deployed '
                            '(use --force to override)')
                self.noop = True

        # Even though this may be a noop, we still record a change in the
        # config rev as it's required by all future stages
        self.context.use_config_rev(digest)

    def fetch(self):
        """Fetch the specified revision of the remote repo.

        The given repo is cloned into the cache directory and a new working
        directory for the given revision is created under revs/{rev}.

        At the end of this stage, the .in-progress link is created to signal
        the possibility for future rollback.
        """
        has_submodules = self.config['git_submodules']
        logger = self.get_logger()
        rev_dir = self.context.rev_path(self.rev)

        git_remote = os.path.join(self.server_url, '.git')
        logger.debug('Fetching from: {}'.format(git_remote))

        # clone/fetch from the repo to the cache directory
        git.fetch(self.context.cache_dir, git_remote, user=self.user)

        # If the rev_dir already exists AND the currently checked-out HEAD is
        # already at the revision specified by ``self.rev`` then you can assume
        #
        # 1. If there is a config deploy, the config is inside the rev_dir
        # 2. The code represented by the SHA1 to be deployed is inside
        #    the rev_dir
        #
        # Set the noop flag and return
        if os.path.isdir(rev_dir) and not self.arguments.force:
            rev = git.sha(rev_dir, 'HEAD')
            if rev == self.rev:
                logger.info('Revision directory already exists '
                            '(use --force to override)')
                self.noop = True
                return

        # clone/fetch from the local cache directory to the revision directory
        git.fetch(rev_dir, self.context.cache_dir, user=self.user)

        # checkout the given revision
        git.checkout(rev_dir, self.rev, user=self.user)

        if has_submodules:
            upstream_submodules = self.config['git_upstream_submodules']
            git.update_submodules(rev_dir, git_remote,
                                  use_upstream=upstream_submodules,
                                  user=self.user)

        self.context.mark_rev_in_progress(self.rev)

    def promote(self, rev=None, rev_dir=None, config_deploy=True):
        """Promote the current deployment.

        Switches the `current` symlink to the current revision directory and
        restarts the configured service.

        Probes the configured service port to measure whether it successfully
        restarted.
        """

        if rev is None:
            rev = self.rev

        if rev_dir is None:
            rev_dir = self.context.rev_path(rev)

        config_deploy = config_deploy and self.config['config_deploy']

        service = self.config.get('service_name', None)
        logger = self.get_logger()

        if (self.context.current_rev_dir == rev_dir and
                not self.arguments.force):

            logger.info('{} is already live '
                        '(use --force to override)'.format(rev_dir))
            self.noop = True
            return

        if config_deploy:
            # Move the rendered config files from their temporary location
            # into a .git/config-files subdirectory of the rev directory
            config_dest = self.context.rev_path(rev, '.git', 'config-files')

            if os.path.isdir(config_dest):
                shutil.rmtree(config_dest)

            os.rename(self.context.temp_path(rev, 'config-files'),
                      config_dest)

            logger.debug('Linking config files at: {}'.format(config_dest))

            for dir_path, _, conf_files in os.walk(config_dest):
                for conf_file in conf_files:
                    full_path = os.path.normpath(
                        '{}/{}'.format(dir_path, conf_file))

                    rel_path = os.path.relpath(full_path, config_dest)
                    final_path = os.path.join('/', rel_path)
                    utils.move_symlink(full_path, final_path, user=self.user)

        self.context.mark_rev_current(rev)
        self.context.link_path_to_rev(self.final_path, rev, backup=True)

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

        rollback_from = self.context.rev_in_progress
        rollback_to = self.context.rev_done

        if not rollback_from:
            logger.info('No rollback necessary. Skipping')
            return 0

        if not rollback_to:
            raise RuntimeError('there is no previous revision to rollback to')

        logger.info('Rolling back from revision {} to {}'.format(rollback_from,
                                                                 rollback_to))

        rev_dir = self.context.done_rev_dir

        if not os.path.isdir(rev_dir):
            msg = 'rollback failed due to missing rev directory {}'
            raise RuntimeError(msg.format(rev_dir))

        # Promote the previous rev and skip config re-evaluation as it's no
        # longer necessary or desirable at this point
        self.promote(rollback_to, rev_dir, config_deploy=False)
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
        logger = self.get_logger()

        self.context.mark_rev_done(self.rev)
        self.context.cleanup()

        for rev_dir in self.context.find_old_rev_dirs():
            logger.info('Removing old revision {}'.format(rev_dir))
            shutil.rmtree(rev_dir)


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
    @cli.argument('-s', '--stages', choices=STAGES,
                  help='Deployment stages to execute. Used only for testing.')
    @cli.argument('-l', '--limit-hosts', default='all',
                  help='Limit deploy to hosts matching expression')
    @cli.argument('-f', '--force', action='store_true',
                  help='force re-fetch and checkout')
    def main(self, *extra_args):
        logger = self.get_logger()

        self.repo = self.config['git_repo']
        self.context.setup()

        if self.arguments.stages:
            stages = self.arguments.stages.split(',')
        else:
            stages = STAGES

        if not git.is_dir(self.context.root):
            raise RuntimeError(errno.EPERM, 'Script must be run from git repo')

        self._build_deploy_groups()

        if not len(self.all_targets):
            logger.warn('No targets selected, check limits and dsh_targets')
            return 1

        with utils.lock(self.config['lock_file']):
            with log.Timer('deploy_' + self.repo):
                timestamp = datetime.utcnow()
                tag = git.next_deploy_tag(location=self.context.root)
                commit = git.sha(location=self.context.root,
                                 rev=self.arguments.rev)

                deploy_info = {
                    'tag': tag,
                    'commit': commit,
                    'user': self.context.user,
                    'timestamp': timestamp.isoformat(),
                }

                git.update_deploy_head(deploy_info,
                                       location=self.context.root)
                git.tag_repo(deploy_info, location=self.context.root)

                self.config_deploy_setup(commit)

                self.config['git_rev'] = commit

                # Run git update-server-info because git repo is a dumb
                # apache server
                git.update_server_info(self.config['git_submodules'])

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

        cfg_file = self.context.env_specific_path('config-files.yaml')

        logger.debug('Config deploy file: {}'.format(cfg_file))

        if not cfg_file:
            return

        tmp_cfg_file = self.context.path('.git', 'config-files',
                                         '{}.yaml'.format(commit))
        utils.mkdir_p(os.path.basename(tmp_cfg_file))
        tmp_cfg = {}

        with open(cfg_file, 'r') as cf:
            config_files = yaml.load(cf.read())

        tmp_cfg['files'] = []
        # Get an environment specific template
        for config_file in config_files:
            f = {}
            f['name'] = config_file
            template_attrs = config_files[config_file]
            template_name = template_attrs['template']
            template = self.context.env_specific_path('templates',
                                                      template_name)

            with open(template, 'r') as tmp:
                f['template'] = tmp.read()

            # Remote var file is optional
            if template_attrs.get('remote_vars', None):
                f['remote_vars'] = template_attrs['remote_vars']

            f['erb_syntax'] = template_attrs.get('erb_syntax', False)

            tmp_cfg['files'].append(f)

        tmp_cfg['override_vars'] = {}

        # Build vars to override remote
        search = self.context.env_specific_paths('vars.yaml')
        for vars_file in search:
            with open(vars_file, 'r') as vf:
                tmp_cfg['override_vars'].update(yaml.load(vf.read()))

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

            search_path = self.context.env_specific_paths()
            search_path.append('/etc/dsh/group')
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
        if not os.path.exists(self.context.scap_path('checks.yaml')):
            config['perform_checks'] = False

        for key, value in config.iteritems():
            deploy_local_cmd.extend(['-D', '{}:{}'.format(key, value)])

        if self.arguments.force:
                deploy_local_cmd.append('--force')

        deploy_local_cmd += ['-g', group, stage]

        logger.debug('Running remote deploy cmd {}'.format(deploy_local_cmd))

        deploy_stage = ssh.Job(hosts=targets, user=self.config['ssh_user'])
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

    def _load_config(self):
        """Sets the host directory after the config has been loaded."""

        super(Deploy, self)._load_config()
        env = self.arguments.environment
        self.context = context.HostContext(os.getcwd(), environment=env)

    def _setup_loggers(self):
        """Sets up additional logging to `scap/deploy.log`."""

        basename = git.describe(self.context.root).replace('/', '-')
        log_file = self.context.log_path('{}.log'.format(basename))
        log.setup_loggers(self.config,
                          self.arguments.loglevel,
                          handlers=[log.DeployLogHandler(log_file)])


class DeployLog(cli.Application):
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
        ctx = context.HostContext(os.getcwd())
        ctx.setup()

        def latest_log_file():
            log_glob = ctx.log_path('*.log')

            try:
                return max(glob.iglob(log_glob), key=os.path.getmtime)
            except ValueError:
                return None

        if self.arguments.latest:
            given_log = latest_log_file()
        else:
            given_log = self.arguments.file

        filter = log.Filter.loads(self.arguments.expr, filter=False)

        if not filter.isfiltering('levelno'):
            filter.append({'levelno': lambda v: v >= self.arguments.loglevel})

        formatter = log.AnsiColorFormatter(self.FORMAT, self.DATE_FORMAT)

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
                    log_path = latest_log_file()

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

    def _setup_loggers(self):
        pass
