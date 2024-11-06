# -*- coding: utf-8 -*-
"""
    scap.cli
    ~~~~~~~~
    Classes and helpers for creating command line interfaces

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
from contextlib import contextmanager
import errno
import locale
import logging
import operator
import os
import re
import shlex
import socket
import subprocess
import sys
import time
from typing import Optional
from functools import reduce

import scap.version as version
import scap.ansi as ansi
import scap.arg as arg
import scap.config as config
import scap.interaction as interaction
import scap.lock as lock
import scap.log as log
import scap.targets as targets
import scap.utils as utils
from scap.ssh import SSH_WITH_KEY

PRIMARY_DEPLOY_SERVER_ONLY_COMMAND = "_primary_deploy_server_only"
REQUIRE_TTY_MULTIPLEXER_COMMAND = "_require_tty_multiplexer"


class Application(object):
    """Base class for creating command line applications."""

    program_name = None
    _logger = None
    _announce_logger = None
    _io = None
    _have_announced = False
    _stats = None
    arguments = None
    config = None
    # Dictionary representing configuration options that were
    # specified using -D on the command line.  Populated by _load_config().
    cli_defines = {}

    def __init__(self, exe_name):
        if self.program_name is None:
            self.program_name = os.path.basename(exe_name)
        self.exe_name = exe_name
        self.start = time.time()
        # Property and related code should be removed once a long-term solution for
        # https://phabricator.wikimedia.org/T304557 is implemented
        self.user_ssh_auth_sock = (
            os.environ["SSH_AUTH_SOCK"] if "SSH_AUTH_SOCK" in os.environ else None
        )

        try:
            locale.setlocale(locale.LC_ALL, "")
        except locale.Error:
            pass

    def get_logger(self):
        """Lazy getter for a logger instance."""
        if self._logger is None:
            self._logger = logging.getLogger(self.program_name)
        return self._logger

    def get_stats(self):
        if self._stats is None:
            self._stats = log.Stats(
                self.config["statsd_host"], int(self.config["statsd_port"])
            )
        return self._stats

    def get_mediawiki_staging_lock_file(self):
        """Get the path to the lock file corresponding to the MediaWiki staging directory"""
        stage_dir = self.config["stage_dir"]
        return os.path.join(
            "/var/lock", "scap." + stage_dir.strip("/").replace("/", "_") + ".lock"
        )

    def get_scap3_lock_file(self):
        """Get the path to scap.lock"""
        if self.config["lock_file"] is not None:
            return self.config["lock_file"]

        if "git_repo" not in self.config:
            raise LookupError(
                "git_repo must be defined in scap config for get_scap3_lock_file to work"
            )

        return "/var/lock/scap.%s.lock" % (self.config["git_repo"].replace("/", "_"))

    @property
    def verbose(self):
        return self.arguments.loglevel < logging.INFO

    def get_duration(self):
        """Get the elapsed duration in seconds."""
        return time.time() - self.start

    def get_script_path(self, remote=False):
        """Returns the path to the scap script."""

        scap = os.path.join(os.path.dirname(sys.argv[0]), "scap")

        # For development
        if remote:
            scap = os.environ.get("REMOTE_SCAP", scap)

        return scap

    def get_keyholder_key(self, ssh_user=None, key_name=None):
        """Get the private key for IdentityFile use in ssh."""

        if ssh_user is None:
            ssh_user = self.config["ssh_user"]
        if key_name is None:
            key_name = self.config.get("keyholder_key")

        key_dir = "/etc/keyholder.d"
        key_safe_name = re.sub(r"\W", "_", ssh_user)
        if key_name is None:
            key_name = key_safe_name
        key_path = os.path.join(key_dir, key_name)

        if os.path.exists(key_path):
            self.get_logger().debug("Using key: %s", key_path)
            return key_path

        self.get_logger().debug("Unable to find keyholder key for %s", key_safe_name)
        return None

    def get_master_list(self, limit_hosts=None, exclude_hosts=None):
        """Get list of deploy master hostnames that should be updated before
        the rest of the cluster."""
        return targets.get(
            "dsh_masters",
            self.config,
            limit_hosts=limit_hosts,
            exclude_hosts=exclude_hosts,
        ).all

    def announce(self, *args):
        """
        Announce a message to broadcast listeners.

        Emits a logging event to the 'scap.announce' logger which can be
        configured to broadcast messages via irc or another real-time
        notification channel.

        Announcements can be disabled by using '--no-log-message' in the
        command invocation (e.g. `scap sync-file --no-log-message foo.php`). In
        this case the log event will still be emitted to the normal logger as
        though `self.get_logger().info()` was used instead of
        `self.announce()`.
        """
        if self.arguments.no_log_message:
            # Do not log to the announce logger, but do log the event for the
            # console and other non-broadcast log collectors.
            self.get_logger().info(*args)
        else:
            if self._announce_logger is None:
                self._announce_logger = logging.getLogger("scap.announce")
            self._announce_logger.info(*args)
            self._have_announced = True

    def announce_final(self, *args):
        """
        Use this method to avoid announcing (to IRC, etc) that an operation
        has finished if there was no prior announcement that an operation
        had started.  This is useful is situations where self.announce()
        is called in an `except` or `finally` clause surrounding a larger
        operation.
        """
        if self._have_announced:
            self.announce(*args)
        else:
            self.get_logger().info(*args)

    def active_wikiversions(self, source_tree="deploy", return_type=list):
        """
         Get an ordered list or dictionary of active MediaWiki versions.

        :param source_tree: Source tree to read file from: 'deploy' or 'stage'

        :param return_type: One of list or dict.

        :returns: If return_type is list (the default), returns a list of
                  version strings (like "1.38.0-wmf.4") in ascending order.

                  If return_type is dict, returns a collections.OrderedDict of
                  {version:wikidb} values sorted by version number in ascending
                  order.  'wikidb' will be the alphabetically-first wikidb for 'version'.
                  This can be used by operations that need a db but don't care
                  which wiki's db is used.
        """
        return utils.get_active_wikiversions(
            self.config[source_tree + "_dir"], self.config["wmf_realm"], return_type
        )

    @property
    def message_argument(self) -> str:
        """
        The given `message` :func:`argument` if one was defined, otherwise it
        returns "(no justification provided)".
        """
        if hasattr(self, "arguments") and hasattr(self.arguments, "message"):
            return self.arguments.message
        return "(no justification provided)"

    # Interaction stuff
    def get_io(self):
        if self._io is None:
            self._io = interaction.GetIO()
        return self._io

    def output_line(self, line: str, sensitive: bool = False):
        return self.get_io().output_line(line, sensitive)

    def input_line(self, prompt: str) -> str:
        return self.get_io().input_line(prompt)

    def prompt_for_approval_or_exit(self, prompt_message, exit_message):
        """Exits successfully with a message if the user does not approve."""
        approval = self.get_io().prompt_user_for_confirmation(prompt_message)
        if not approval:
            self.announce_final(exit_message)
            sys.exit(os.EX_OK)

    def prompt_choices(self, question: str, choices, default=None) -> str:
        return self.get_io().prompt_choices(question, choices, default)

    def prompt_user_for_confirmation(self, prompt_message, default="n") -> bool:
        return self.get_io().prompt_user_for_confirmation(prompt_message, default)

    def report_status(self, status: Optional[str] = None, log=False):
        if log and status:
            self.get_logger().info(status)
        self.get_io().report_status(status)

    @contextmanager
    def reported_status(self, status: str, log=False):
        """
        Set status to `status`, do work, then clear the job status.
        """
        self.report_status(status, log)
        try:
            yield
        finally:
            self.report_status()

    # End interaction stuff

    def _process_arguments(self, args, extra_args):
        """
        Validate and process command line arguments.

        Default behavior is to abort the application with an error if any
        unparsed arguments were found.

        :returns: Tuple of (args, extra_args) after processing
        """
        if extra_args:
            self._argparser.error("extra arguments found: %s" % " ".join(extra_args))

        if hasattr(args, "message"):
            args.message = " ".join(args.message) or "(no justification provided)"

        return args, extra_args

    def setup(self, use_global_config=True):
        # Let each application handle `extra_args`
        self.arguments, self.extra_arguments = self._process_arguments(
            self.arguments, self.extra_arguments
        )

        self._load_config(use_global_config=use_global_config)
        self._setup_loggers()
        self._setup_environ()

    def _load_config(self, use_global_config=True):
        """Load configuration."""

        self.cli_defines = {}

        if self.arguments.defines:
            res = []
            for string in self.arguments.defines:
                pair = string.split(":", 1)
                if len(pair) != 2:
                    raise SystemExit(
                        "Invalid configuration setting: {}\nSettings must be in 'key:value' format".format(
                            string
                        )
                    )
                res.append(pair)
            self.cli_defines = dict(res)

        self.config = config.load(
            cfg_file=self.arguments.conf_file,
            environment=self.arguments.environment,
            overrides=self.cli_defines,
            use_global_config=use_global_config,
            use_local_config=(not self.arguments.no_local_config),
        )
        # self.cli_defines remains available for other code (such as
        # deploy.py) to use.

    def _setup_loggers(self):
        """Setup logging."""
        log.setup_loggers(self.config, self.arguments.loglevel)

    def _setup_environ(self):
        """Setup shell environment."""
        auth_sock = self.config.get("ssh_auth_sock")
        php_version = self.config.get("php_version")
        if php_version is not None:
            os.environ["PHP"] = php_version
        if auth_sock is not None and self.arguments.shared_authsock:
            os.environ["SSH_AUTH_SOCK"] = auth_sock

    def main(self, *extra_args):
        """
        Main business logic of the application.

        Parsed command line arguments are available in self.arguments. Global
        configuration is available in self.config. Unparsed command line
        arguments are passed as positional arguments.

        :returns: exit status
        """
        raise NotImplementedError()

    def handle_keyboard_interrupt(self):
        """
        Handle ctrl-c from interactive user.

        :returns: exit status
        """
        # Only announce the abort if any previous call to
        # self.announce() has generated a broadcast announcement
        # (T329228)
        if self._announce_logger:
            self.announce(
                "{} aborted: {} (duration: {})".format(
                    self.program_name,
                    self.message_argument,
                    utils.human_duration(self.get_duration()),
                )
            )
        return 130

    def format_passthrough_args(self) -> list:
        """
        Returns a list with user-supplied cli config args in cli format. Makes it easy to pass along
        config args to locally spawned `scap` processes.

        For example, if the user passed config options `canary_dashboard_url` and `log_json` on the
        command line, then the return value would be:

          ["-D", "canary_dashboard_url:https://somewhere.over.the.rainbow", "-D", "log_json:false"]
        """

        cmd_line_args = [
            ["-D", "%s:%s" % (cli_arg, self.cli_defines.get(cli_arg))]
            for cli_arg in self.cli_defines.keys()
        ]
        # Flatten and return cli args
        return reduce(operator.concat, cmd_line_args, [])

    @utils.log_context("cli.Application.scap_call")
    def scap_call(
        self,
        scap_cmd: list,
        user=None,
        env=None,
        passthrough_arguments=True,
        logger=None,
        **kwargs
    ) -> subprocess.CompletedProcess:
        """
        Call scap subcommand

        :returns: the completed process
        """

        args = [self.get_script_path()] + scap_cmd

        if passthrough_arguments:
            args += self.format_passthrough_args()

        # sudo if necessary
        if user and user != utils.get_username():
            args = ["sudo", "-u", user, "-n", "--"] + args

        if env is None:
            env = {}

        with utils.suppress_backtrace():
            if logger is not None:
                logger.debug("Running {}".format(" ".join(args)))

            return subprocess.run(
                args,
                env={**env, **self.get_gerrit_ssh_env()},
                text=True,
                **kwargs,
            )

    def scap_check_call(self, scap_cmd: list, **kwargs) -> subprocess.CompletedProcess:
        """
        Call scap subcommand and check for a non-zero exit status

        :returns: the completed process
        :raises CalledProcessError: if the subcommand fails
        """

        return self.scap_call(scap_cmd, check=True, **kwargs)

    def scap_check_output(self, scap_cmd: list, **kwargs) -> str:
        """
        Call scap subcommand and return output as text

        :returns: the output of `scap_cmd` as text
        :raises CalledProcessError: if the subcommand fails
        """
        return self.scap_call(
            scap_cmd,
            check=True,
            stdout=subprocess.PIPE,
            **kwargs,
        ).stdout.strip()

    def get_gerrit_ssh_env(self) -> dict:
        """
        An environment to interact with Gerrit over ssh

        When `gerrit_push_user` scap configuration is set, set the
        `GIT_SSH_COMMAND` environment variable to have git ssh to use that user
        instead of the username of the person that invoked scap.

        Else if a user-supplied ssh agent socket was provided, override the one
        set by scap which does not have the invoking user key.

        Eventually we will always require `gerrit_push_user` to be set.

        See https://phabricator.wikimedia.org/T304557

        :returns: environment suitable for sshing to Gerrit
        """

        # Copy from the user environment
        gerrit_env = os.environ.copy()

        if self.config["gerrit_push_user"]:
            key_file = self.get_keyholder_key(
                ssh_user=self.config["gerrit_push_user"],
            )
            ssh_command = SSH_WITH_KEY(
                user=self.config["gerrit_push_user"], key=key_file
            )
            # Do not redirect ssh stdin from /dev/null since git sends data
            # this way.
            ssh_command.remove("-n")

            # With python 3.8 we can use shlex.join(ssh_command)
            gerrit_env["GIT_SSH_COMMAND"] = " ".join(
                shlex.quote(arg) for arg in ssh_command
            )

        elif self.user_ssh_auth_sock:
            gerrit_env["SSH_AUTH_SOCK"] = self.user_ssh_auth_sock

        return gerrit_env

    def _handle_exception(self, ex):
        """
        Handle unhandled exceptions and errors.

        :returns: exit status
        """
        logger = self.get_logger()
        exception_type = type(ex).__name__
        backtrace = True

        # Retrieve scap version

        scap_version = version.__version__
        message = "%s failed: <%s> %s (scap version: %s)"

        if isinstance(ex, lock.LockFailedError) or getattr(
            ex, "_scap_no_backtrace", False
        ):
            backtrace = False

        if backtrace:
            logger.warning("Unhandled error:", exc_info=True)

        logger.error(message, self.program_name, exception_type, ex, scap_version)
        return 70

    def _before_exit(self, exit_status):
        """
        Do any final cleanup or processing before the application exits.

        Called after :meth:`main` and before `sys.exit` even when an exception
        occurs.

        :returns: exit status
        """
        if utils.should_colorize_output():
            try:
                sys.stdout.write(ansi.reset())
                sys.stdout.flush()
            except Exception:
                pass

        return exit_status

    def _run_as(self, user):
        """Ensure that this program is run as the given user."""
        if utils.get_username() != user:
            self.get_logger().info("%s must be run as user %s", self.program_name, user)
            # Replace the current process with a sudo call to the same script
            os.execvp("sudo", ["sudo", "-u", user, "-n", "--"] + sys.argv)

    def _assert_current_user(self, user):
        """Assert that this program is run as the given user."""
        if utils.get_username() != user:
            raise RuntimeError("%s must be run as user %s" % (self.program_name, user))

    def _assert_auth_sock(self):
        """Assert that SSH_AUTH_SOCK is present in the environment."""
        if "SSH_AUTH_SOCK" not in os.environ:
            with utils.suppress_backtrace():
                raise RuntimeError(
                    "%s requires SSH agent forwarding" % self.program_name
                )

    def _check_user_auth_sock(self):
        """
        Similar to `_assert_auth_sock` method above.

        Returns immediately when "gerrit_push_user" has been configured, it is
        assumed the identity is provided in the ssh authentication socket /
        keyholder.

        Unlike `_assert_auth_sock`, this method:
         * Checks the original SSH_AUTH_SOCK provided in the environment
           by the user, before scap potentially overrides it
         * Is meant to be called during operations where a user-provided
           ssh-agent is required (e.g. deploy-promote). It is not a global
           requirement of scap

        """
        if self.config["gerrit_push_user"]:
            return

        if not self.user_ssh_auth_sock:
            utils.abort(
                "You need to provide access to your own ssh-agent\n"
                "try: eval $(ssh-agent) && ssh-add"
            )

    def get_current_train_info(self):
        return utils.get_current_train_info(
            self.config["train_blockers_url"], self.config["web_proxy"]
        )

    @contextmanager
    def lock(self, lock_file, **kwargs):
        """
        Acquire a lock for the main work of this application.
        """
        if "name" not in kwargs:
            kwargs["name"] = self.program_name

        if "reason" not in kwargs:
            kwargs["reason"] = self.message_argument

        with lock.Lock(lock_file, **kwargs):
            yield

    @contextmanager
    def lock_mediawiki_staging(self, **kwargs):
        """
        Acquire a lock for the main work of this MediaWiki staging related application.
        """
        with self.lock(self.get_mediawiki_staging_lock_file(), **kwargs):
            yield

    @contextmanager
    def lock_mediawiki_staging_and_announce(self, **kwargs):
        """
        Acquire a lock for the main work of this application and announce its
        start and finish.
        """
        with self.lock_mediawiki_staging(**kwargs):
            start = time.time()
            self.announce(
                "Started scap %s: %s", self.program_name, self.message_argument
            )
            yield
            self.announce(
                "Finished scap %s: %s (duration: %s)",
                self.program_name,
                self.message_argument,
                utils.human_duration(time.time() - start),
            )

    # NOTE: The 'scap' directory is excluded by scap pull (aka
    # scap.tasks.sync_common()), and excluded when building mediawiki
    # container images
    # xref https://gitlab.wikimedia.org/repos/releng/release/-/blob/00aa9bd253c358fc7b4fb88498e4f9c806212f57/make-container-image/build-images.py#L95
    def scap_history_dbfile(self):
        return os.path.join(self.config["stage_dir"], "scap/log/history.db")

    def spiderpig_dir(self):
        return os.path.join(self.config["stage_dir"], "scap/spiderpig")

    def spiderpig_jobrunner_lockfile(self):
        return os.path.join(self.spiderpig_dir(), "jobrunner.lock")

    def spiderpig_dbfile(self):
        return os.path.join(self.spiderpig_dir(), "spiderpig.db")

    def spiderpig_joblogdir(self):
        return os.path.join(self.spiderpig_dir(), "jobs")

    def spiderpig_jwt_secret_file(self):
        return os.path.join(self.spiderpig_dir(), "spiderpig-jwt.key")

    def timed(self, fn, *args, **kwargs):
        """
        Call a function wrapped in a log.Timer.
        """
        with log.Timer(fn.__name__, self.get_stats()):
            fn(*args, **kwargs)

    def Timer(self, label, logger=None):
        return log.Timer(
            label,
            stats=self.get_stats(),
            logger=logger,
        )

    @staticmethod
    def factory(argv=None):
        parser = arg.build_parser()
        args, extra_args = parser.parse_known_args(argv)
        if not hasattr(args, "which"):
            sys.exit("MUST provide subcommand, run with --help for a list")
        app = args.which(args.command)
        app._argparser = parser
        app.arguments = args
        app.extra_arguments = extra_args
        return app

    @classmethod
    def run(cls):
        """
        Construct and run an application.

        Calls ``sys.exit`` with exit status returned by application by
        default. Setting ``exit`` to ``False`` will instead return the class
        instance and it's exit status. This would generally only be done when
        testing.

        :param cls: Class to create and run
        :param argv: Command line arguments
        :returns: Tuple of class instance and exit status when not exiting
        """
        # Bootstrap the logging system
        logging.basicConfig(
            level=logging.INFO,
            format=log.CONSOLE_LOG_FORMAT,
            datefmt=log.TIMESTAMP_FORMAT,
            stream=sys.stdout,
        )

        # Silence noisy loggers early
        logging.getLogger("urllib3").setLevel(logging.WARNING)
        logging.getLogger("scap.sh").setLevel(logging.WARNING)

        # Setup instance for logger access
        app = cls("scap")

        exit_status = 0
        try:
            app = Application.factory()

            if os.geteuid() == 0:
                raise OSError(errno.EPERM, "Scap should not be run as root")

            app.setup()

            if getattr(app.main, PRIMARY_DEPLOY_SERVER_ONLY_COMMAND, False):
                hostname = socket.gethostname()
                if not any(
                    hostname.startswith(prefix) for prefix in ["deploy", "releases"]
                ):
                    utils.abort(
                        "This scap command can only be used on a deploy server or the releases Jenkins instance."
                    )
                if hasattr(app, "config") and app.config["block_deployments"]:
                    # T376995: Provide an exception for "scap deploy --init". Necessary to boostrap fresh, non-primary
                    # deployment servers
                    if app.arguments.command != "deploy" or not app.arguments.init:
                        utils.abort(
                            "This scap command is disabled on this host. If you really need to run it, you can override by"
                            """ passing "-Dblock_deployments:False" to the call"""
                        )

            if (
                app.config.get("require_terminal_multiplexer")
                and getattr(app.main, REQUIRE_TTY_MULTIPLEXER_COMMAND, False)
                and interaction.have_terminal()
            ):
                if os.environ.get("TMUX") is None and os.environ.get("STY") is None:
                    utils.abort(
                        "No tmux or screen process found. Try either:\n"
                        '\tTMUX: tmux new-session -s "🚂🌈"\n'
                        "\tSCREEN: screen -D -RR train"
                    )

            if "subcommand" in app.arguments and app.arguments.subcommand:
                method = app.arguments.subcommand
                exit_status = method(app, app.extra_arguments)
            else:
                exit_status = app.main(app.extra_arguments)

        except KeyboardInterrupt:
            # Handle ctrl-c from interactive user
            exit_status = app.handle_keyboard_interrupt()

        except Exception as ex:
            # Handle all unhandled exceptions and errors
            exit_status = app._handle_exception(ex)

        finally:
            exit_status = app._before_exit(exit_status)

        # Flush logger before exiting
        logging.shutdown()

        # Make a beep
        if exit_status != 0:
            sys.stdout.write("\a")
            sys.stdout.flush()

        # Exit
        sys.exit(exit_status)


def argument(*args, **kwargs):
    """
    argument(option_flags..[,action='store'][,nargs=1]\
       [,const=None][,default][,type=str][,choices][,required=False][,help]\
       [,dest])

    Decorator used to declare a command line argument on an
    :class:`Application`

    Maps a command line argument to the decorated class or method.

    :param str option_flags: One or more option flags associated with this
                             argument, e.g. '-a', '--arg'
    :param action: the action associated with this argument. e.g. 'store_true'
    :param default: The default value for this argument if not specified by the
                    user.
    :param str help: Short description of this argument, displayed in
                    ``--help`` text.
    :param list choices: List possible values for this argument. If specified,
                        argument values will be validated for you and
                        ``--help`` will list the possible choices for the user.
    :param bool required: True if your argument is required.
    :param type type: The type of value accepted by your argument, e.g. int or
                      file.
    :type type: type
    :param nargs: The number of values accepted by this argument.
    :type nargs: int, str
    """

    def wrapper(func):
        arguments = getattr(func, arg.ATTR_ARGUMENTS, [])
        arguments.append(dict(_flags=args, **kwargs))
        setattr(func, arg.ATTR_ARGUMENTS, arguments)
        return func

    return wrapper


COMMAND_REGISTRY = {}


def command(*args, **kwargs):
    """
    command(command_name, help="help text",[subcommands=False])

    Map a `scap` sub-command to the decorated class.

    :param str command_name: The name of the sub-command
    :param str help: A summary of your command to be displayed in `--help` text
    :type help: str or None
    :raises ValueError: if there is already a command named `command_name`

    **Usage Example**::

        import scap.cli

        @cli.command('hello', help='prints "hello world" and exits')
        class HelloCommand(cli.Application):
            @cli.argument('--goodbye', action='store_true',
                          help='Say goodbye instead.')
            def main(extra_args):
                if self.arguments.goodbye:
                    print('Goodbye, cruel world.')
                else:
                    print('Hello, world.')

    """

    def wrapper(cls):
        global COMMAND_REGISTRY
        name = args[0]
        if name in COMMAND_REGISTRY:
            err = 'Duplicate: A command named "%s" already exists.' % name
            raise ValueError(err)
        has_subcommands = kwargs.pop("subcommands", False)
        if has_subcommands:
            setattr(cls, arg.ATTR_SUBPARSER, True)

        cmd = dict(name=name, cls=cls, args=args, kwargs=kwargs)
        COMMAND_REGISTRY[name] = cmd

        primary_deploy_server_only = kwargs.get("primary_deploy_server_only", False)
        setattr(
            cls.main, PRIMARY_DEPLOY_SERVER_ONLY_COMMAND, primary_deploy_server_only
        )
        kwargs.pop("primary_deploy_server_only", None)

        require_tty_multiplexer = kwargs.get("require_tty_multiplexer", False)
        setattr(cls.main, REQUIRE_TTY_MULTIPLEXER_COMMAND, require_tty_multiplexer)
        kwargs.pop("require_tty_multiplexer", None)

        return cls

    return wrapper


def subcommand(name=None):
    """
    subcommand(command_name)

    Define an argparse subcommand by decorating a method on your
    cli.Application subclass.

    Use this decorator when you want to have a subcommand on a method other
    than 'main'

    In order for this to have any affect, your cli.Application must be
    decorated with subcommands=True (see example below).

    **Usage Example**::

        import scap.cli

        @cli.command('hello', subcommands=True,
                     help='prints "hello world" and exits',)
        class HelloCommand(cli.Application):
            @cli.subcommand('world')
            def world_subcommand(extra_args):
                print('hello world')

    """

    def wrapper(func):
        subcommand_name = func.__name__ if name is None else name
        setattr(func, arg.ATTR_SUBCOMMAND, subcommand_name)
        setattr(func, arg.ATTR_ARGUMENTS, [])
        return func

    return wrapper
