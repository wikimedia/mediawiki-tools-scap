# -*- coding: utf-8 -*-
"""
    scap.deploy
    ~~~~~~~~~~~
    Command wrappers for deploy tasks

"""
import argparse
import collections
import errno
import glob
import os
import requests
import shutil
import time
import yaml
import subprocess

from datetime import datetime

from . import checks
from . import config
from . import context
from . import nrpe
from . import template
from . import cli
from . import log
from . import ssh
from . import targets
from . import tasks
from . import utils
from . import git

RESTART = 'restart_service'
STAGES = ['fetch', 'config_deploy', 'promote', 'finalize']
EX_STAGES = [RESTART, 'rollback']


@cli.command('deploy-local', help=argparse.SUPPRESS)
class DeployLocal(cli.Application):
    """
    Command that runs on target hosts.

    Responsible for fetching code from the git server, checking out
    the appropriate revisions, restarting services and running checks.
    """

    def _load_config(self):

        super(DeployLocal, self)._load_config()

        # FIXME: this makes the assumption that the git_deploy_dir specified on
        # the target machines in /etc/scap.cfg is correct. Currently, we have
        # no deployments that don't use /srv/deployment as the deployment
        # directory; however, this may not always be the case.
        #
        # I think in order to work around this issue `scap deploy-local` will
        # need to be passed more information about the state of the world it's
        # intended to create, i.e.:
        #
        #     scap deploy-local http://deployment.eqiad.wmnet/[repo] \
        #         /srv/deployment/[repo]
        #
        # Might prove to be a better way to do this.
        self.final_path = os.path.join(self.config['git_deploy_dir'],
                                       self.arguments.repo)

        root = '{}-cache'.format(self.final_path)
        self.context = context.TargetContext(root)
        self.context.setup()

        overrides = self._get_config_overrides()
        if self.arguments.defines:
            overrides.update(dict(self.arguments.defines))

        config.override_config(self.config, overrides)

    @cli.argument('-g', '--group',
                  help='Group of which this local machine is a part')
    @cli.argument('stage', metavar='STAGE', choices=STAGES + EX_STAGES,
                  nargs='?', help='Stage of the deployment to execute')
    @cli.argument('-f', '--force', action='store_true',
                  help='force stage even when noop detected')
    @cli.argument('-r', '--repo',
                  help='repo that you are deploying')
    @cli.argument('--refresh-config', action='store_true',
                  help='fetch a new version of a config from the deploy '
                       'server rather than using the locally cached config')
    def main(self, *extra_args):
        self.rev = self.config['git_rev']
        self.noop = False
        # only supports http from tin for the moment
        url = os.path.normpath('{git_server}/{git_repo}'.format(**self.config))
        self.server_url = 'http://{0}'.format(url)

        self.stages = STAGES

        if self.arguments.stage:
            self.stages = [self.arguments.stage]

        group = self.arguments.group

        status = 0

        for stage in self.stages:
            self.noop = False

            getattr(self, stage)()

            if not self.noop and self.config['perform_checks']:
                status = self._execute_checks(stage, group)

            if not status == 0:
                break

        return status

    def config_deploy(self):
        """
        Render config files.

        Grabs the current config yaml file from the deploy git server, and
        renders the final template inside the repo-cache's tmp directory.
        """
        logger = self.get_logger()
        if not self.config['config_deploy']:
            logger.info('No config files to deploy, skipping...')
            return

        config_files = self.config.get('config_files')
        if not config_files:
            raise IOError(errno.ENOENT, 'No config_files found!')

        overrides = config_files.get('override_vars', {})

        source_basepath = self.context.rev_path(
            self.rev, '.git', 'config-files')
        logger.debug('Source basepath: {}'.format(source_basepath))
        utils.mkdir_p(source_basepath)

        for config_file in config_files['files']:
            filename = config_file['name']

            tmpl = template.Template(
                name=filename,
                loader={filename: config_file['template']},
                erb_syntax=config_file.get('erb_syntax', False),
                var_file=config_file.get('remote_vars', None),
                overrides=overrides
            )

            if self.rev in os.path.realpath(filename):
                self.noop = True

            if self.noop and not self.arguments.force:
                logger.info(
                    '{} is already linked to current rev'
                    '(use --force to override)'.format(filename))
                return 0

            # Checks should run if the --force argument is used
            self.noop = False

            if filename.startswith('/'):
                filename = filename[1:]

            utils.mkdir_p(os.path.join(
                source_basepath, os.path.dirname(filename)))

            source = os.path.join(source_basepath, filename)
            logger.info('Rendering config_file: {}'.format(source))

            with open(source, 'w+') as f:
                output_file = tmpl.render()
                f.write(output_file)

    def fetch(self):
        """
        Fetch the specified revision of the remote repo.

        The given repo is cloned into the cache directory and a new working
        directory for the given revision is created under revs/{rev}.

        At the end of this stage, the .in-progress link is created to signal
        the possibility for future rollback.
        """
        has_submodules = self.config['git_submodules']
        has_gitfat = self.config['git_fat']
        logger = self.get_logger()
        rev_dir = self.context.rev_path(self.rev)

        git_remote = os.path.join(self.server_url, '.git')
        logger.info('Fetch from: {}'.format(git_remote))

        # clone/fetch from the repo to the cache directory
        git.fetch(self.context.cache_dir, git_remote)

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
                logger.info(
                    'Revision directory already exists '
                    '(use --force to override)')
                self.noop = True
                return

        # clone/fetch from the local cache directory to the revision directory
        git.fetch(rev_dir, self.context.cache_dir)

        logger.info('Checkout rev: {}'.format(self.rev))

        # checkout the given revision
        git.checkout(rev_dir, self.rev)

        if has_submodules:
            upstream_submodules = self.config['git_upstream_submodules']
            logger.info('Update submodules')
            git.update_submodules(
                rev_dir, git_remote, use_upstream=upstream_submodules)

        if has_gitfat:
            if not git.fat_isinitialized(rev_dir):
                logger.info('Git fat initialize')
                git.fat_init(rev_dir)

            logger.info("Git fat pull '%s'", rev_dir)
            git.fat_pull(rev_dir)

        self.context.mark_rev_in_progress(self.rev)

    def promote(self, rev=None, rev_dir=None, config_deploy=True):
        """
        Promote the current deployment.

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

        logger = self.get_logger()

        if (self.context.current_rev_dir == rev_dir and
                not self.arguments.force):

            logger.info(
                '{} is already live (use --force to override)'.format(rev_dir))
            self.noop = True
            return

        if config_deploy:
            # Move the rendered config files from their temporary location
            # into a .git/config-files subdirectory of the rev directory
            config_dest = self.context.rev_path(rev, '.git', 'config-files')

            logger.info('Linking config files at: {}'.format(config_dest))

            for dir_path, _, conf_files in os.walk(config_dest):
                for conf_file in conf_files:
                    full_path = os.path.normpath(
                        '{}/{}'.format(dir_path, conf_file))

                    rel_path = os.path.relpath(full_path, config_dest)
                    final_path = os.path.join('/', rel_path)
                    utils.move_symlink(full_path, final_path)

        self.context.mark_rev_current(rev)
        self.context.link_path_to_rev(self.final_path, rev, backup=True)
        self.stages.append(RESTART)

    def finalize(self, rollback=False, rev=None):
        """
        Perform the final deploy actions.

        Moves the .done flag to the rev directory, removes the .in-progress
        flag, and cleans up old revision directories.
        """
        logger = self.get_logger()

        if rev is None:
            rev = self.rev

        if not rollback:
            self.context.mark_rev_done(rev)

        self.context.rm_in_progress()

        for rev_dir in self.context.find_old_rev_dirs():
            logger.info('Removing old revision {}'.format(rev_dir))
            shutil.rmtree(rev_dir)

    def restart_service(self):
        service = self.config.get('service_name', None)
        if not service:
            return

        tasks.restart_service(service)

        port = self.config.get('service_port', None)
        if not port:
            return

        service_timeout = self.config['service_timeout']
        tasks.check_port(int(port), timeout=service_timeout)

    def rollback(self):
        """
        Performs a rollback to the last deployed revision.

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

        logger.info(
            'Rolling back from revision {} to {}'.format(
                rollback_from, rollback_to))

        rev_dir = self.context.done_rev_dir

        if not os.path.isdir(rev_dir):
            msg = 'rollback failed due to missing rev directory {}'
            raise RuntimeError(msg.format(rev_dir))

        # Promote the previous rev and skip config re-evaluation as it's no
        # longer necessary or desirable at this point
        self.promote(rollback_to, rev_dir, config_deploy=False)
        self.finalize(rollback=True, rev=rollback_to)

    @utils.log_context('checks')
    def _execute_checks(self, stage, group=None, logger=None):
        """
        Fetch and executes all checks configured for the given stage.

        Checks are retrieved from the remote deploy host and cached within
        tmp.
        """

        # Load NRPE checks
        if os.path.isdir(self.config['nrpe_dir']):
            nrpe.register(nrpe.load_directory(self.config['nrpe_dir']))

        chks = checks.load(self.config)
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
        """Make sure a check is valid for our current group."""
        if group is not None:
            return chk.stage == stage and (chk.group == group or
                                           chk.group is None)
        else:
            return chk.stage == stage

    def _get_config_overrides(self):
        """
        Get config information locally or from the deployment host.

        If the local [repo]-cache/.config file does not exist, or
        --refresh-config has been passed explicitly on the command line, then
        the config is fetched from the deployment server. Otherwise the local
        config cache is used.

        This is useful for things like locally rebuilding config files when a
        remote var file has changed, and the version deployed from the
        deployment server has changed, but a particular target has not yet been
        updated.
        """
        if (self.arguments.refresh_config or
                not os.path.exists(self.context.local_config)):
            config = self._get_remote_overrides()
            with open(self.context.local_config, 'w') as cfg:
                cfg.write(yaml.dump(config, default_flow_style=False))

        with open(self.context.local_config) as cfg:
            return yaml.load(cfg.read())

    def _get_remote_overrides(self):
        """Grab remote config from git_server."""
        cfg_url = os.path.join(
            self.config['git_server'], self.arguments.repo,
            '.git', 'DEPLOY_HEAD')

        r = requests.get('{}://{}'.format(self.config['git_scheme'], cfg_url))
        if r.status_code != requests.codes.ok:
            raise IOError(errno.ENOENT, 'Config file not found', cfg_url)

        return yaml.load(r.text)


@cli.command('deploy', help='[SCAP 3] Sync new service code across cluster')
class Deploy(cli.Application):
    """
    Sync new service code across cluster.

    Uses local .scaprc as config for each host in cluster
    """

    STAGE_NAMES_OVERRIDES = {
        'promote': 'promote and restart_service',
    }

    MAX_BATCH_SIZE = 80
    # Stop executing on new hosts after failure
    MAX_FAILURES = 0

    DEPLOY_CONF = [
        'git_deploy_dir',
        'git_fat',
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
        'config_files',
        'perform_checks',
        'environment',
    ]

    repo = None
    targets = []

    @cli.argument('-r', '--rev', help='Revision to deploy')
    @cli.argument('-s', '--stages', choices=STAGES,
                  help='Deployment stages to execute. Used only for testing.')
    @cli.argument('-l', '--limit-hosts', default='all',
                  help='Limit deploy to hosts matching expression')
    @cli.argument('-f', '--force', action='store_true',
                  help='force re-fetch and checkout')
    @cli.argument('--service-restart', action='store_true',
                  help='Restart service')
    @cli.argument('-i', '--init', action='store_true',
                  help='Setup a repo for initial deployment')
    @cli.argument('message', nargs='*', help='Log message for SAL')
    def main(self, *extra_args):
        logger = self.get_logger()

        self.deploy_info = {}
        self.repo = self.config['git_repo']

        if self.arguments.stages:
            stages = self.arguments.stages.split(',')
        else:
            stages = STAGES

        if self.arguments.service_restart:
            stages = [RESTART]

        restart_only = False
        if len(stages) is 1 and stages[0] is RESTART:
            restart_only = True

        if not git.is_dir(self.context.root):
            raise RuntimeError(errno.EPERM, 'Script must be run from git repo')

        self._build_deploy_groups()

        if not len(self.all_targets):
            logger.warn('No targets selected, check limits and dsh_targets')
            return 1

        short_sha1 = git.info(self.context.root)['headSHA1'][:7]
        if not short_sha1:
            short_sha1 = 'UNKNOWN'

        deploy_name = 'deploy'
        if self.arguments.init:
            deploy_name = 'setup'
        elif restart_only:
            deploy_name = 'restart'
        display_name = '{} [{}@{}]'.format(deploy_name, self.repo, short_sha1)

        rev = self.arguments.rev
        if not rev:
            rev = self.config.get('git_rev', 'HEAD')

        with utils.lock(self.context.lock_path(), self.arguments.message):
            with log.Timer(display_name):
                timestamp = datetime.utcnow()
                tag = git.next_deploy_tag(location=self.context.root)
                commit = git.sha(location=self.context.root, rev=rev)
                self.get_logger().debug('Deploying Rev: {}'.format(commit))

                self.config_deploy_setup(commit)
                self.checks_setup()
                self.config['git_rev'] = commit

                self.deploy_info.update({
                    'tag': tag,
                    'commit': commit,
                    'user': utils.get_username(),
                    'timestamp': timestamp.isoformat(),
                })

                # Handle JSON output from deploy-local
                self.deploy_info['log_json'] = True

                self.get_logger().debug('Update DEPLOY_HEAD')

                self.deploy_info.update({
                    key: self.config.get(key)
                    for key in self.DEPLOY_CONF
                    if self.config.get(key) is not None
                })

                git.update_deploy_head(self.deploy_info, self.context.root)

                git.tag_repo(self.deploy_info, location=self.context.root)

                # Run git update-server-info because git repo is a dumb
                # apache server
                git.update_server_info(self.config['git_submodules'])

                if self.arguments.init:
                    return 0

                self.announce('Started %s: %s', display_name,
                              self.arguments.message)
                exec_result = self._execute_for_groups(stages)
                if not restart_only:
                    self.announce('Finished %s: %s (duration: %s)',
                                  display_name, self.arguments.message,
                                  utils.human_duration(self.get_duration()))
                return exec_result
        return 0

    def _execute_for_groups(self, stages):
        logger = self.get_logger()
        continue_all = False

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

            if not self._last_group(group) and not continue_all:
                prompt = '{} deploy successful. Continue?'.format(group)
                choices = '[y]es/[n]o/[c]ontinue all groups'
                answer = utils.ask(prompt, 'y', choices)

                if answer == 'c':
                    continue_all = True
                elif answer != 'y':
                    break

        return 0

    def _last_group(self, group):
        return group == next(reversed(self.deploy_groups))

    def config_deploy_setup(self, commit):
        """
        Generate environment-specific config file and variable template list.

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

        cfg_file = self.context.env_specific_path('config-files.y*ml')

        logger.debug('Config deploy file: {}'.format(cfg_file))

        if not cfg_file:
            return

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
            template = self.context.env_specific_path(
                'templates', template_name)

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
        for vars_file in reversed(search):
            with open(vars_file, 'r') as vf:
                tmp_cfg['override_vars'].update(yaml.load(vf.read()))

        self.config['config_files'] = tmp_cfg

    def checks_setup(self):
        """Build info to run checks."""
        checks_dict = collections.OrderedDict()
        checks_paths = self.context.env_specific_paths('checks.y*ml')

        # Reverse checks_paths so that paths are overwritten with more
        # environment-specific checks
        for check_path in reversed(checks_paths):
            with open(check_path) as f:
                checks = utils.ordered_load(f, yaml.SafeLoader)['checks']
                checks_dict.update(checks)

        if len(checks_dict.keys()) == 0:
            self.deploy_info['perform_checks'] = False
            return

        self.deploy_info['checks'] = checks_dict

    def _build_deploy_groups(self):
        """Build server groups based on configuration `server_groups` variable
        """
        target_obj = targets.get(
            'dsh_targets',
            self.config,
            self.arguments.limit_hosts,
            self.context.env_specific_paths()
        )

        self.all_targets = target_obj.all
        self.deploy_groups = target_obj.groups

    def execute_rollback(self, stage, group, targets):
        prompt = "Stage '{}' failed on group '{}'. Perform rollback?".format(
            stage, group)

        if utils.ask(prompt, 'y') == 'y':
            return self.execute_stage_on_group('rollback', group, targets)

        return 0

    def execute_stage_on_group(self, stage, group, targets):
        logger = self.get_logger()
        deploy_local_cmd = [self.get_script_path(), 'deploy-local']
        batch_size = self._get_batch_size(stage)

        deploy_local_cmd.append('-v')

        deploy_local_cmd += ['--repo', self.config['git_repo']]

        if self.arguments.force:
                deploy_local_cmd.append('--force')

        deploy_local_cmd += ['-g', group, stage]
        deploy_local_cmd.append('--refresh-config')

        logger.debug('Running remote deploy cmd {}'.format(deploy_local_cmd))

        deploy_stage = ssh.Job(hosts=targets, user=self.config['ssh_user'])
        deploy_stage.output_handler = ssh.JSONOutputHandler
        deploy_stage.max_failure = self.MAX_FAILURES
        deploy_stage.command(deploy_local_cmd)
        display_name = self._get_stage_name(stage)
        deploy_stage.progress(
            '{}: {} stage(s)'.format(self.repo, display_name))

        succeeded, failed = deploy_stage.run(batch_size=batch_size)

        if failed:
            logger.warning('%d targets had deploy errors', failed)
            return 1

        return 0

    def _get_stage_name(self, stage):
        """Map a stage name to a stage display name."""
        return self.STAGE_NAMES_OVERRIDES.get(stage, stage)

    def _get_batch_size(self, stage):
        default = self.config.get('batch_size', self.MAX_BATCH_SIZE)
        size = int(self.config.get('{}_batch_size'.format(stage), default))
        return min(size, self.MAX_BATCH_SIZE)

    def _load_config(self):
        """Set the host directory after the config has been loaded."""

        super(Deploy, self)._load_config()
        env = self.config['environment']
        self.context = context.HostContext(os.getcwd(), environment=env)
        self.context.setup()

    def _setup_loggers(self):
        """Set up additional logging to `scap/deploy.log`."""

        basename = git.describe(self.context.root).replace('/', '-')
        log_file = self.context.log_path('{}.log'.format(basename))
        log.setup_loggers(
            self.config, self.arguments.loglevel,
            handlers=[log.DeployLogHandler(log_file)])


@cli.command(
    'deploy-log',
    help='[SCAP 3] Tail/filter/output events from the deploy logs')
class DeployLog(cli.Application):
    """
    Tail/filter/output events from the deploy logs.

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
                        print(formatter.format(record))
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
                        print("-- Opening log file: '{}'".format(log_path))
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


@cli.command('deploy-mediawiki', help=argparse.SUPPRESS)
class DeployMediaWiki(cli.Application):
    """
    Deploy mediawiki via scap3.

    Ideally, this class and command will be fully merged with
    the `scap deploy` command by the end of the transition; however, there
    may also be unique reasons, particularly on the deployment masters, to
    keep this command
    """

    @cli.argument('message', nargs='*', help='Log message for git')
    def main(self, *extra_args):
        """Run deploy-mediawiki."""
        # Flatten local into git repo
        self.get_logger().info('scap deploy-mediawiki')
        git.default_ignore(self.config['deploy_dir'])

        git.add_all(self.config['deploy_dir'], message=self.arguments.message)

        scap = self.get_script_path()
        options = {
            'git_repo': self.config['deploy_dir'],
        }

        option_list = ['-D{}:{}'.format(x, y) for x, y in options.iteritems()]
        cmd = [scap, 'deploy', '-v']
        cmd += option_list
        cmd += ['--init']

        with utils.cd(self.config['deploy_dir']):
            subprocess.check_call(cmd)
