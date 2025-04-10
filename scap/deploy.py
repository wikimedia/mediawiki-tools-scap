# -*- coding: utf-8 -*-
"""
    scap.deploy
    ~~~~~~~~~~~
    Command wrappers for deploy tasks

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
import argparse
import collections
from datetime import datetime
import errno
import glob
import os
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.parse
import yaml

import requests

import scap.checks as checks
import scap.config as config
import scap.context as context
import scap.nrpe as nrpe
import scap.script as script
import scap.template as template
import scap.cli as cli
import scap.log as log
import scap.ssh as ssh
import scap.targets as targets
import scap.tasks as tasks
import scap.utils as utils
import scap.git as git

FINALIZE = "finalize"
# Default for a service is to be restarted
HANDLE_SERVICE = "handle_service"
ROLLBACK = "rollback"
CONFIG_DIFF = "config_diff"
PROMOTE = "promote"
CONFIG_DEPLOY = "config_deploy"
FETCH = "fetch"

STAGES = [FETCH, CONFIG_DEPLOY, PROMOTE]
EX_STAGES = [HANDLE_SERVICE, ROLLBACK, CONFIG_DIFF, FINALIZE]
ALL_STAGES = STAGES + EX_STAGES


class DeployGroupFailure(RuntimeError):
    """Signal that a particular deploy group failed"""

    pass


@cli.command("deploy-local", help=argparse.SUPPRESS)
class DeployLocal(cli.Application):
    """
    Command that runs on target hosts.

    Responsible for fetching code from the git server, checking out
    the appropriate revisions, restarting/disabling services and running checks.
    """

    def __init__(self, exe_name):
        super().__init__(exe_name)
        self.context = None
        self.final_path = None
        self.noop = False
        self.rev = None
        self.server_url = None
        self.stages = STAGES

    # This is called by scap.cli.Application.setup(), which is called
    # early in the application startup process.
    def _load_config(self, use_global_config=True):
        super()._load_config(use_global_config=use_global_config)

        if self.config["git_server"] is None:
            raise SystemExit("'git_server' is not set in scap configuration")

        self.server_url = urllib.parse.urlunparse(
            (
                self.config["git_scheme"],
                self.config["git_server"],
                self.arguments.repo,
                None,
                None,
                None,
            )
        )

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
        self.final_path = os.path.join(
            self.config["git_deploy_dir"], self.arguments.repo
        )

        root = "{}-cache".format(self.final_path)
        self.context = context.TargetContext(root)
        self.context.setup()

        overrides = self._get_config_overrides()
        if self.arguments.defines:
            overrides.update(self.cli_defines)

        config.override_config(self.config, overrides)

    @cli.argument("-g", "--group", help="Group of which this local machine is a part")
    @cli.argument(
        "stage",
        metavar="STAGE",
        choices=ALL_STAGES,
        nargs="?",
        help="Stage of the deployment to execute",
    )
    @cli.argument(
        "-f", "--force", action="store_true", help="force stage even when noop detected"
    )
    @cli.argument("-r", "--repo", help="repo that you are deploying", required=True)
    @cli.argument(
        "--refresh-config",
        action="store_true",
        help="fetch a new version of a config from the deploy "
        "server rather than using the locally cached config",
    )
    def main(self, *extra_args):
        self.rev = self.config["git_rev"]

        if self.arguments.stage:
            self.stages = [self.arguments.stage]

        group = self.arguments.group

        status = 0

        for stage in self.stages:
            self.noop = False

            if not self.noop and self.config["perform_checks"]:
                status = self._execute_checks("before", stage, group)

            if status != 0:
                break

            getattr(self, stage)()

            if not self.noop and self.config["perform_checks"]:
                status = self._execute_checks("after", stage, group)

            if status != 0:
                break

        return status

    def config_deploy(self):
        """
        Render config files.

        Grabs the current config yaml file from the deploy git server, and
        renders the config files templates inside the repo-cache's
        .git/config-files directory. During the promote stage, final links are
        created to the rev-specific config files.

        If the config file target path for a template is relative, however,
        the file is written directly at the target path and no link is
        created.
        """
        logger = self.get_logger()
        if not self.config["config_deploy"]:
            logger.info("config_deploy is not enabled in scap.cfg, skipping.")
            return 0

        config_files = self.config.get("config_files")
        if not config_files:
            raise IOError(errno.ENOENT, "No config_files found!")

        overrides = config_files.get("override_vars", {})

        source_basepath = self.context.rev_path(self.rev, ".git", "config-files")
        logger.debug("Source basepath: {}".format(source_basepath))
        utils.mkdir_p(source_basepath)

        for config_file in config_files["files"]:
            filename = config_file["name"]
            var_file = config_file.get("remote_vars", None)

            tmpl = template.Template(
                name=filename,
                loader={filename: config_file["template"]},
                erb_syntax=config_file.get("erb_syntax", False),
                var_file=var_file,
                output_format=config_file.get("output_format", None),
                overrides=overrides,
            )

            if filename.startswith("/"):
                # Absolute config files are written to .git/config-files and
                # linked from their final location in the promote stage
                filename = os.path.join(source_basepath, filename[1:])
            else:
                # Relative config files are written directly to their
                # destination under the rev directory and linking is skipped
                filename = self.context.rev_path(self.rev, filename)

            utils.mkdir_p(os.path.dirname(filename))
            logger.info("Rendering config_file: {} using {}".format(filename, var_file))
            updated = utils.write_file_if_needed(filename, tmpl.render())
            logger.info("%s: %s", filename, "[rendered]" if updated else "[unchanged]")

        return 0

    def config_diff(self):
        """
        Render config files from DEPLOY_HEAD and compare each file to the
        deployed version. This is called by scap deploy --dry-run
        """
        logger = self.get_logger()
        if not self.config["config_deploy"]:
            logger.info("config_deploy is not enabled in scap.cfg, skipping.")
            return

        config_files = self.config.get("config_files")
        if not config_files:
            raise IOError(errno.ENOENT, "No config_files found!")

        overrides = config_files.get("override_vars", {})

        for config_file in config_files["files"]:
            filename = config_file["name"]

            tmpl = template.Template(
                name=filename,
                loader={filename: config_file["template"]},
                erb_syntax=config_file.get("erb_syntax", False),
                var_file=config_file.get("remote_vars", None),
                output_format=config_file.get("output_format", None),
                overrides=overrides,
            )

            with tempfile.NamedTemporaryFile(mode="w", delete=False) as cfg:
                cfg.write(tmpl.render())

            logger.info("Diff for %s:", filename)

            try:
                diff_cmd = ["/usr/bin/diff", "-u", filename, cfg.name]
                subprocess.check_output(diff_cmd)
            except subprocess.CalledProcessError as result:
                logger.info(
                    "{type}: {output}",
                    extra={"type": "config_diff", "output": result.output},
                )
            else:
                logger.info("no differences")
            os.unlink(cfg.name)

    def fetch(self):
        """
        Fetch the specified revision of the remote repo.

        The given repo is cloned into the cache directory and a new working
        directory for the given revision is created under revs/{rev}.

        At the end of this stage, the .in-progress link is created to signal
        the possibility for future rollback.
        """
        logger = self.get_logger()

        has_submodules = self.config["git_submodules"]
        git_binary_manager = None
        if self.config["git_fat"]:
            logger.warning(
                "Using deprecated git_fat config, swap to git_binary_manager"
            )
            git_binary_manager = [git.FAT]
        elif self.config["git_binary_manager"]:
            git_binary_manager = config.multi_value(self.config["git_binary_manager"])

        rev_dir = self.context.rev_path(self.rev)

        git_remote = os.path.join(self.server_url, ".git")
        logger.info("Fetch from: {}".format(git_remote))

        fetch_config = {}
        original_origin = self.config.get("original_origin")
        if original_origin:
            fetch_config["lfs.url"] = f"{original_origin}/info/lfs"

        # clone/fetch from the repo to the cache directory
        git.fetch(self.context.cache_dir, git_remote, config=fetch_config)

        if has_submodules:
            upstream_submodules = self.config["git_upstream_submodules"]
            logger.info("Update submodules")
            git.update_submodules(
                location=self.context.cache_dir,
                git_remote=git_remote,
                use_upstream=upstream_submodules,
            )

        # If the rev_dir already exists AND the currently checked-out HEAD is
        # already at the revision specified by ``self.rev`` then you can assume
        #
        # 1. If there is a config deploy, the config is inside the rev_dir
        # 2. The code represented by the SHA1 to be deployed is inside
        #    the rev_dir
        #
        # Set the noop flag and return
        if os.path.isdir(rev_dir) and not self.arguments.force:
            rev = git.sha(rev_dir, "HEAD")
            if rev == self.rev:
                logger.info(
                    "Revision directory already exists " "(use --force to override)"
                )
                self.noop = True
                self.context.mark_rev_in_progress(self.rev)
                return

        if not os.path.isdir(rev_dir):
            # if lfs is enabled and this is the first time cloning this repo,
            # then we need to run `git lfs install`` before `git clone`
            if git_binary_manager and git.LFS in git_binary_manager:
                git.lfs_install()

        # clone/fetch from the local cache directory to the revision directory
        git.fetch(
            rev_dir,
            self.context.cache_dir,
            reference=self.context.cache_dir,
            dissociate=False,
            recurse_submodules=False,
            config=fetch_config,
        )

        logger.info("Checkout rev: {}".format(self.rev))

        # checkout the given revision
        git.checkout(rev_dir, self.rev)

        if has_submodules:
            git.update_submodules(
                location=rev_dir,
                git_remote=git_remote,
                use_upstream=upstream_submodules,
                reference=self.context.cache_dir,
            )

        if git_binary_manager:
            for manager in git_binary_manager:
                logger.info("Pulling large objects [using %s]", manager)
                if manager in [git.FAT, git.LFS]:
                    git.largefile_pull(rev_dir, manager, has_submodules)
                else:
                    logger.warning("Passed unrecognized binary manager %s", manager)

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

        config_deploy = config_deploy and self.config["config_deploy"]

        logger = self.get_logger()

        if self.context.current_rev_dir == rev_dir and not self.arguments.force:
            logger.info("{} is already live (use --force to override)".format(rev_dir))
            self.noop = True
            return

        if config_deploy:
            # Move the rendered config files from their temporary location
            # into a .git/config-files subdirectory of the rev directory
            config_dest = self.context.rev_path(rev, ".git", "config-files")

            logger.info("Linking config files at: {}".format(config_dest))

            for dir_path, _, conf_files in os.walk(config_dest):
                for conf_file in conf_files:
                    target = os.path.normpath(os.path.join(dir_path, conf_file))
                    rel_target = os.path.relpath(target, config_dest)
                    symlink_path = os.path.join("/", rel_target)
                    utils.update_symlink(target, symlink_path)

        self.context.mark_rev_current(rev)
        self.context.link_path_to_rev(self.final_path, rev, backup=True)
        self.stages.append(HANDLE_SERVICE)

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

        cache_revs = self.config.get("cache_revs", 5)

        for rev_dir in self.context.find_old_rev_dirs(cache_revs):
            logger.info("Removing old revision {}".format(rev_dir))
            shutil.rmtree(rev_dir)

    def handle_service(self):
        """
        Restart, reload or disable service(s) and check port based on configuration.
        """
        service = self.config.get("service_name", None)
        if not service:
            return

        tasks.handle_services(
            service,
            self.config.get("require_valid_service", False),
            os.path.exists(self.config["secondary_host_signal_file"]),
        )

        port = self.config.get("service_port", None)
        if not port:
            return

        service_timeout = self.config["service_timeout"]
        tasks.check_port(int(port), timeout=service_timeout)

    def rollback(self):
        """
        Performs a rollback to the last deployed revision.

        Rollback looks for a .done symlink that points to the revision
        directory for the last successful deployment. If this link doesn't
        exist, a rollback isn't possible. If it does exist, the current
        revision directory is replaced with the target of the link and the
        promote and finalize stages are re-run.
        """

        logger = self.get_logger()

        rollback_from = self.context.rev_in_progress
        rollback_to = self.context.rev_done

        if not rollback_to:
            raise RuntimeError("there is no previous revision to rollback to")

        logger.info(
            "Rolling back from revision {} to {}".format(rollback_from, rollback_to)
        )

        rev_dir = self.context.done_rev_dir

        if not os.path.isdir(rev_dir):
            msg = "rollback failed due to missing rev directory {}"
            raise RuntimeError(msg.format(rev_dir))

        # Promote the previous rev and skip config re-evaluation as it's no
        # longer necessary or desirable at this point
        self.promote(rollback_to, rev_dir, config_deploy=False)
        self.finalize(rollback=True, rev=rollback_to)

    @utils.log_context("checks")
    def _execute_checks(self, when, stage, group=None, logger=None):
        """
        Fetch and executes all checks configured for the given stage.

        Checks are retrieved from the remote deploy host and cached within
        tmp.
        """
        # Primarily for use in script checks
        check_environment = os.environ.copy()
        check_environment["SCAP_REVS_DIR"] = self.context.revs_dir
        check_environment["SCAP_FINAL_PATH"] = self.final_path
        check_environment["SCAP_REV_PATH"] = self.context.rev_path(self.rev)
        check_environment["SCAP_CURRENT_REV_DIR"] = self.context.current_rev_dir
        check_environment["SCAP_DONE_REV_DIR"] = self.context.done_rev_dir

        # Load NRPE checks
        if os.path.isdir(self.config["nrpe_dir"]):
            nrpe.register(nrpe.load_directory(self.config["nrpe_dir"]))

        # Load script checks
        scripts_dir = self.context.scripts_dir(self.rev)
        logger.debug("Registering scripts in directory '%s'" % scripts_dir)
        script.register_directory(scripts_dir, logger=logger)

        chks = checks.load(self.config, check_environment).values()

        for chk in chks:
            target_stage = getattr(chk, when)
            # See method `_check_applies` for an explanation about "restart_service"
            if target_stage not in ALL_STAGES + ["restart_service", None]:
                raise SystemExit(
                    f"""Unknown target stage "{target_stage}" specified in user-defined check"""
                )

        chks = [
            chk
            for chk in chks
            if DeployLocal._check_applies(chk, stage, group, when=when)
        ]

        for chk in chks:
            chk.validate()

        success, done = checks.execute(chks, logger=logger)
        failed = [job.check.name for job in done if job.isfailure()]

        if success:
            return 0
        return 1 if failed else 2

    @staticmethod
    def _check_applies(chk, stage, group, when):
        """Verify whether a check applies for the current group and stage."""
        if group is not None:
            if chk.group is not None and chk.group != group:
                return False

        # In https://gitlab.wikimedia.org/repos/releng/scap/-/commit/9e49f8658bdfa99bc9f0548026a516b386f0b633 the
        # deployment stage "restart_service" was renamed to "handle_service", but deployments can use stage names to
        # specify when a particular check is to be executed. The change broke the existing restbase deployment:
        # https://phabricator.wikimedia.org/T346354 and potentially others.
        # For backward compatibility, "restart_service" is treated here as a synonym for "handle_service"
        valid_stages = [stage]
        if stage == HANDLE_SERVICE:
            valid_stages.append("restart_service")

        return getattr(chk, when) in valid_stages

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
        if self.arguments.refresh_config or not os.path.exists(
            self.context.local_config
        ):
            config = self._get_remote_overrides()
            with open(self.context.local_config, "w") as cfg:
                cfg.write(yaml.dump(config, default_flow_style=False))

        with open(self.context.local_config) as cfg:
            # Note: Not using safe_load here because the config may
            # contain an OrderedDict that we want.
            return yaml.load(cfg.read(), yaml.loader.Loader)

    def _get_remote_overrides(self):
        """Grab remote config from git_server."""
        cfg_url = os.path.join(self.server_url, ".git", "DEPLOY_HEAD")
        r = requests.get(cfg_url)
        r.raise_for_status()

        # Note: Not using safe_load here because the config may
        # contain an OrderedDict that we want.
        return yaml.load(r.text, yaml.loader.Loader)


# T376995: Note "deploy --init" is handled in the code as an exception and can run on a non-primary server
@cli.command(
    "deploy",
    help="[SCAP 3] Sync new service code across cluster",
    primary_deploy_server_only=True,
    require_tty_multiplexer=True,
)
class Deploy(cli.Application):
    """
    Sync new service code across cluster.
    """

    MAX_BATCH_SIZE = 80
    # Stop executing on new hosts after failure
    MAX_FAILURES = 0

    DEPLOY_CONF = [
        "cache_revs",
        "config_deploy",
        "config_files",
        "environment",
        "git_deploy_dir",
        "git_fat",
        "git_binary_manager",
        "git_server",
        "git_scheme",
        "git_repo",
        "git_rev",
        "git_submodules",
        "git_upstream_submodules",
        "nrpe_dir",
        "perform_checks",
        "require_valid_service",
        "secondary_targets_disabled",
        "service_name",
        "service_port",
        "service_timeout",
    ]

    continue_all = False
    repo = None
    targets = []

    def __init__(self, exe_name):
        super().__init__(exe_name)
        self.all_targets = None
        self.context = None
        self.deploy_groups = None
        self.deploy_info = {}

    # This is called by scap.cli.Application.setup(), which is called
    # early in the application startup process.
    def _load_config(self, use_global_config=True):
        """Set the host directory after the config has been loaded."""

        cwd = os.getcwd()
        if not git.is_dir(cwd, top_level=True):
            raise SystemExit(
                """Deployment configuration not found.
For `scap deploy` to work, the current directory must be the top level of a git repo containing a scap/scap.cfg file."""
            )

        super()._load_config(use_global_config=use_global_config)

        os.umask(self.config["umask"])

        env = self.config["environment"]
        self.context = context.HostContext(cwd, environment=env)
        self.context.setup()

    def _setup_loggers(self):
        """Set up additional logging to `scap/deploy.log`."""

        basename = git.describe(self.context.root).replace("/", "-")
        log_file = self.context.log_path("{}.log".format(basename))
        log.setup_loggers(
            self.config,
            self.arguments.loglevel,
            handlers=[log.DeployLogHandler(log_file)],
        )

    @cli.argument("-r", "--rev", help="Specify the revision to deploy")
    @cli.argument(
        "-s",
        "--stages",
        choices=STAGES,
        help="Execute specific deployment stages (For testing)",
    )
    @cli.argument(
        "-l",
        "--limit-hosts",
        default="all",
        help="Limit actions to hosts matching expression",
    )
    @cli.argument(
        "-f",
        "--force",
        action="store_true",
        help="Force fetch and checkout even if nothing changed.",
    )
    @cli.argument(
        "--dry-run",
        action="store_true",
        dest="dry_run",
        help="Compile and deploy config files to a temp location "
        "and output a diff against the previously deployed "
        "config files.",
    )
    @cli.argument(
        "--service-restart",
        action="store_true",
        help="Skip deployment, just restart the service.",
    )
    @cli.argument(
        "-i", "--init", action="store_true", help="Setup a repo for initial deployment"
    )
    @cli.argument("message", nargs="*", help="Log message for SAL")
    def main(self, *extra_args):
        logger = self.get_logger()

        # NOTE: _load_config() runs before main() does, so some important
        # checks are contained there.

        repo = self.config.get("git_repo", None)

        if repo is None:
            sys.exit("Incomplete setup: git_repo must be defined in the configuration")

        self.key = self.get_keyholder_key()
        if self.key is None:
            sys.exit(
                "Unable to find keyholder key for ssh-user {}".format(
                    self.config["ssh_user"]
                )
            )

        if self.arguments.message == "(no justification provided)":
            if self.arguments.init:
                # T376996
                self.arguments.message = "scap deploy --init mode"
            else:
                new_message = self.input_line("Log message (press enter for none): ")
                if new_message:
                    self.arguments.message = new_message

        self.repo = repo

        if self.arguments.stages:
            stages = self.arguments.stages.split(",")
        else:
            stages = STAGES

        if self.arguments.service_restart:
            stages = [HANDLE_SERVICE]
            if not self.config.get("service_name"):
                raise RuntimeError(
                    "--service-restart flag requires a `service_name` in " "the config"
                )

        if self.arguments.dry_run:
            stages = ["config_diff"]

        restart_only = False
        if len(stages) == 1 and stages[0] == HANDLE_SERVICE:
            restart_only = True

        self._build_deploy_groups()

        if not self.all_targets:
            logger.warning("No targets selected, check limits and dsh_targets")
            return 1

        git_info = git.info(self.context.root)
        short_sha1 = git_info["headSHA1"][:7]
        if not short_sha1:
            short_sha1 = "UNKNOWN"

        deploy_name = "deploy"

        if self.arguments.init:
            deploy_name = "setup"
        elif restart_only:
            deploy_name = "restart"

        display_name = "{} [{}@{}]".format(deploy_name, self.repo, short_sha1)

        environment_name = self.config["environment"]

        if environment_name is not None:
            display_name = "{} ({})".format(display_name, environment_name)

        rev = self.arguments.rev

        # No revision passed on the command line, let's check the config
        if not rev:
            rev = self.config.get("git_rev")
            if rev:
                use_upstream = self.config.get("git_upstream_submodules", False)
                if rev.startswith("origin/") and not use_upstream:
                    logger.warning(
                        'You have set `git_rev` to "%s" without '
                        + "setting `git_upstream_submodules=True`. "
                        + "This could lead to unexpected behavior.",
                        rev,
                    )

        # No revision from the config or cli
        # AND we're not running a stage that we want to deploy new code
        if not rev and not self._needs_latest_sha1(stages):
            last_deploy_tag = git.last_deploy_tag(self.context.root)
            if last_deploy_tag is not None:
                # This is what we want to pass to rev-parse to get the sha1 of
                # the annotated commit, rather than the sha1 of the annotation.
                rev = "%s^{}" % last_deploy_tag

        # We must be trying to deploy the latest and greatest
        if not rev:
            rev = "HEAD"

        with self.lock(self.get_scap3_lock_file()):
            with self.Timer(description=display_name, name="deploy"):
                timestamp = datetime.utcnow()
                tag = git.next_deploy_tag(location=self.context.root)
                commit = git.sha(location=self.context.root, rev=rev)
                logger.info("Deploying Rev: {} = {}".format(rev, commit))

                self.config_deploy_setup(commit)
                self.checks_setup()
                self.config["git_rev"] = commit

                self.deploy_info.update(
                    {
                        "tag": tag,
                        "commit": commit,
                        "user": utils.get_username(),
                        "timestamp": timestamp.isoformat(),
                        "original_origin": git_info["remoteURL"],
                    }
                )

                # Handle JSON output from deploy-local
                self.deploy_info["log_json"] = True

                logger.debug("Update DEPLOY_HEAD")

                self.deploy_info.update(
                    {
                        key: self.config.get(key)
                        for key in self.DEPLOY_CONF
                        if self.config.get(key) is not None
                    }
                )

                git.update_deploy_head(self.deploy_info, self.context.root)

                git.tag_repo(self.deploy_info, location=self.context.root)

                # Remove old tags
                git.clean_tags(self.context.root, self.config["tags_to_keep"])

                # Run git update-server-info because git repo is a dumb
                # apache server
                git.update_server_info(self.config["git_submodules"])

                if self.arguments.init:
                    return 0

                self.announce("Started %s: %s", display_name, self.arguments.message)
                exec_result = self._execute_for_groups(stages)
                if not restart_only:
                    self.announce(
                        "Finished %s: %s (duration: %s)",
                        display_name,
                        self.arguments.message,
                        utils.human_duration(self.get_duration()),
                    )
                return exec_result
        return 0

    def _needs_latest_sha1(self, stages):
        """
        Determine whether we expect a new SHA1 to be tagged and deployed

        :stages: list of stages being run
        """

        # If we're only running a quick --dry-run or a --restart, we don't
        # want to update the SHA1
        return not (len(stages) == 1 and stages[0] in [CONFIG_DIFF, HANDLE_SERVICE])

    def _display_group_info(self, group, targets):
        self.get_logger().info(
            "\n== {0} ==\n:* {1}".format(group.upper(), "\n:* ".join(targets))
        )

    def _execute_for_groups(self, stages):
        logger = self.get_logger()
        self.continue_all = False
        attempted_groups = []

        try:
            for name, group in self.deploy_groups.items():
                attempted_groups.append(group)
                self._execute_for_group(stages, group, prompt_user=True)

        except DeployGroupFailure as failure:
            logger.error(str(failure))

            if self.prompt_user_for_confirmation(
                "Rollback all deployed groups?", default="y"
            ):
                # Rollback groups in reverse order
                for group in attempted_groups[::-1]:
                    self._execute_for_group([ROLLBACK], group, ignore_failure=True)

            return 1

        if self.arguments.dry_run:
            # don't finalize a dry-run
            return 0

        for group in attempted_groups:
            self._execute_for_group([FINALIZE], group, ignore_failure=True)

        return 0

    def _execute_for_group(
        self, stages, group, ignore_failure=False, prompt_user=False
    ):
        """
        Executes the given stages across targets of the given group's
        subgroups.

        :param stages: List of stages to execute
        :param group: The `targets.DeployGroup` for which to execute the
                      stages. Any target host that is unreachable via SSH will
                      be added to the group's list of excluded hosts.
        :param ignore_failure: Whether to keep on rolling past the
                               `failure_limit` threshold. Note that even with
                               this argument, SSH failure result in the target
                               being excluded from future stage execution.
        :param prompt_user: Whether to prompt the user after each subgroup.
        """

        logger = self.get_logger()

        failed = 0

        for label, targets in group.subgroups():
            self._display_group_info(label, targets)

            # Copy target list so we can dequeue targets without mutating
            # self.deploy_groups
            targets = list(targets)

            subgroup_failed = 0

            for stage in stages:
                executor = self.execute_stage_on_group(stage, group.name, targets)
                for host, status in executor:
                    if status > 0:
                        # Record failed host and remove it from targets
                        # for the remaining stages
                        subgroup_failed += 1
                        targets.remove(host)

                        if status == ssh.CONNECTION_FAILURE:
                            msg = (
                                "connection to %s failed and future "
                                "stages will not be attempted for this "
                                "target"
                            ) % host
                            logger.warning(msg)
                            group.exclude(host)

                # Stop executing stages if all targets have failed
                if not targets:
                    break

            failed += subgroup_failed

            if subgroup_failed > 0:
                logger.warning("%d targets failed" % subgroup_failed)

                if not ignore_failure and failed > group.failure_limit:
                    msg = "%d of %d %s targets failed, exceeding limit" % (
                        failed,
                        group.original_size,
                        label,
                    )
                    raise DeployGroupFailure(msg)

            logger.info("%s deploy successful" % label)
            if (
                prompt_user
                and not self.continue_all
                and not self._last_subgroup(group, label)
            ):
                prompt = "Continue?"
                choices = {"Yes": "y", "No": "n", "Continue all groups": "c"}
                answer = self.prompt_choices(prompt, choices, "y")

                if answer == "c":
                    self.continue_all = True
                elif answer == "n":
                    raise DeployGroupFailure("deployment aborted")

    def _last_subgroup(self, group, subgroup):
        last_group = next(reversed(self.deploy_groups))
        last_subgroup = list(self.deploy_groups[last_group].subgroups())[-1][0]

        return group.name == last_group and subgroup == last_subgroup

    def config_deploy_setup(self, commit):
        """
        Generate environment-specific config file and variable template list.

        Builds a yaml file that contains:
        1. A list of file objects containing template files to be deployed
        2. An object containing variables specified in the
        environment-specific `vars.yaml` file and inheriting from the
        `vars.yaml` file
        """
        logger = self.get_logger()

        if not self.config["config_deploy"]:
            return

        logger.debug("Prepare config deploy")

        cfg_file = self.context.env_specific_path("config-files.y*ml")

        logger.debug("Config deploy file: {}".format(cfg_file))

        if not cfg_file:
            return

        tmp_cfg = {}

        with open(cfg_file, "r") as cf:
            config_files = yaml.safe_load(cf.read())

        tmp_cfg["files"] = []
        # Get an environment specific template
        for config_file in config_files:
            f = {}
            f["name"] = config_file
            template_attrs = config_files[config_file]
            template_name = template_attrs["template"]

            template_output_format = template_attrs.get(
                "output_format", template.guess_format(config_file)
            )

            if (
                template_output_format is not None
                and template_output_format not in template.VALID_OUTPUT_FORMATS
            ):
                raise RuntimeError(
                    "Invalid output_format: '{}' for template '{}'".format(
                        template_output_format, template_name
                    )
                )

            f["output_format"] = template_output_format

            template_path = self.context.env_specific_path("templates", template_name)

            with open(template_path, "r") as tmp:
                f["template"] = tmp.read()

            # Remote var file is optional
            if template_attrs.get("remote_vars", None):
                f["remote_vars"] = template_attrs["remote_vars"]

            f["erb_syntax"] = template_attrs.get("erb_syntax", False)

            tmp_cfg["files"].append(f)

        tmp_cfg["override_vars"] = {}

        # Build vars to override remote
        search = self.context.env_specific_paths("vars.yaml")
        for vars_file in reversed(search):
            with open(vars_file, "r") as vf:
                tmp_cfg["override_vars"].update(yaml.safe_load(vf.read()))

        self.config["config_files"] = tmp_cfg

    def checks_setup(self):
        """Build info to run checks."""
        checks_dict = collections.OrderedDict()
        checks_paths = self.context.env_specific_paths("checks.y*ml")

        # Reverse checks_paths so that paths are overwritten with more
        # environment-specific checks
        for check_path in reversed(checks_paths):
            with open(check_path) as f:
                checks = utils.ordered_load(f, Loader=yaml.SafeLoader)["checks"]
                checks_dict.update(checks)

        if not checks_dict.keys():
            self.deploy_info["perform_checks"] = False
            return

        self.deploy_info["checks"] = checks_dict

    def _build_deploy_groups(self):
        """
        Build server groups based on configuration `server_groups` variable
        """
        target_obj = targets.get(
            "dsh_targets",
            self.config,
            self.arguments.limit_hosts,
            self.context.env_specific_paths(),
        )

        self.all_targets = target_obj.all
        self.deploy_groups = target_obj.groups

    def execute_stage_on_group(self, stage, group, targets):
        """
        Execute a deploy stage for the given group targets.

        :param stage: deploy stage.
        :param group: deploy group.
        :param targets: group targets.
        :yields: (host, status)
        """

        logger = self.get_logger()
        deploy_local_cmd = [self.get_script_path(remote=True), "deploy-local"]
        batch_size = self._get_batch_size(stage)

        deploy_local_cmd.append("-v")

        deploy_local_cmd += ["--repo", self.config["git_repo"]]

        if self.arguments.force:
            deploy_local_cmd.append("--force")

        deploy_local_cmd += ["-g", group, stage]
        deploy_local_cmd.append("--refresh-config")

        logger.debug("Running remote deploy cmd {}".format(deploy_local_cmd))

        deploy_stage = ssh.Job(
            hosts=targets,
            user=self.config["ssh_user"],
            key=self.key,
            verbose=self.verbose,
        )
        deploy_stage.output_handler = ssh.JSONOutputHandler
        deploy_stage.max_failure = self.MAX_FAILURES
        deploy_stage.command(deploy_local_cmd)
        display_name = self._get_stage_name(stage)
        progress_message = "{}: {} stage(s)".format(self.repo, display_name)
        deploy_stage.progress(log.reporter(progress_message))

        failed = 0

        for jobresult in deploy_stage.run_with_status(batch_size):
            if jobresult.status != 0:
                failed += 1
            yield jobresult.host, jobresult.status

        if failed:
            logger.warning("%d targets had deploy errors", failed)

    def _get_stage_name(self, stage):
        """Map a stage name to a stage display name."""
        if stage == PROMOTE and self.config.get("service_name") is not None:
            return "promote and handle_service"

        return stage

    def _get_batch_size(self, stage):
        default = self.config.get("batch_size", self.MAX_BATCH_SIZE)
        size = int(self.config.get("{}_batch_size".format(stage), default))
        return min(size, self.MAX_BATCH_SIZE)


@cli.command(
    "deploy-log", help="[SCAP 3] Tail/filter/output events from the deploy logs"
)
class DeployLog(cli.Application):
    """
    Tail/filter/output events from the deploy logs.

    examples::

        deploy-log -v
        deploy-log 'host == scap-target-01'
        deploy-log 'msg ~ "some important (message|msg)"'
        deploy-log 'levelno >= WARNING host == scap-target-*'
    """

    DATE_FORMAT = "%H:%M:%S"
    DIR_SCAN_DELAY = 1
    FORMAT = "%(asctime)s [%(host)s] %(message)s"

    @cli.argument(
        "expr", metavar="EXPR", nargs="?", default="", help="Filter expression."
    )
    @cli.argument(
        "-f",
        "--file",
        metavar="FILE",
        default=None,
        help="Parse and filter an existing log file",
    )
    @cli.argument(
        "-l",
        "--latest",
        action="store_true",
        help="Parse and filter the latest log file",
    )
    def main(self, *extra_args):
        ctx = context.HostContext(os.getcwd())
        ctx.setup()

        def latest_log_file():
            log_glob = ctx.log_path("*.log")

            try:
                return max(glob.iglob(log_glob), key=os.path.getmtime)
            except ValueError:
                return None

        if self.arguments.latest:
            given_log = latest_log_file()
        else:
            given_log = self.arguments.file

        logfilter = log.Filter.loads(self.arguments.expr, invert=False)

        if not logfilter.isfiltering("levelno"):
            logfilter.append({"levelno": lambda v: v >= self.arguments.loglevel})

        formatter = log.DiffLogFormatter(self.FORMAT, self.DATE_FORMAT)

        cur_log_path = given_log
        cur_log_file = open(given_log, "r") if given_log else None
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
                    if logfilter.filter(record):
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

                        cur_log_file = open(cur_log_path, "r")
                else:
                    time.sleep(0.1)

        if cur_log_file:
            cur_log_file.close()

        return 0

    def _setup_loggers(self):
        pass
