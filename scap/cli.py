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
from __future__ import absolute_import

import errno
import logging
import os
import re
import sys
import time
import scap.plugins

from scap.terminal import TERM
import scap.arg as arg
import scap.config as config
import scap.lock as lock
import scap.log as log
import scap.targets as targets
import scap.utils as utils


class Application(object):
    """Base class for creating command line applications."""

    program_name = None
    _logger = None
    _announce_logger = None
    _stats = None
    arguments = None
    config = None

    def __init__(self, exe_name):
        if self.program_name is None:
            self.program_name = os.path.basename(exe_name)
        self.exe_name = exe_name
        self.start = time.time()

    def get_logger(self):
        """Lazy getter for a logger instance."""
        if self._logger is None:
            self._logger = logging.getLogger(self.program_name)
        return self._logger

    def get_stats(self):
        if self._stats is None:
            self._stats = log.Stats(
                self.config['statsd_host'], int(self.config['statsd_port']))
        return self._stats

    def get_lock_file(self):
        """Get the path to scap.lock"""
        if self.config['lock_file'] is not None:
            return self.config['lock_file']
        else:
            try:
                return '/var/lock/scap.%s.lock' % (
                    self.config['git_repo'].replace('/', '_'))
            except KeyError:
                # `scap sync*` can run from anywhere on the file system and
                # doesn't actually use the value of `git_repo`. In contrast,
                # `scap deploy` will fail almost instantly without a `git_repo`
                # set. If we're attempting to create a lock file, and there is
                # no git_repo, then it's likely for a sync* command and the
                # correct git_repo is operations/mediawiki-config.
                return '/var/lock/scap.operations_mediawiki-config.lock'

    @property
    def verbose(self):
        return self.arguments.loglevel < logging.INFO

    def get_duration(self):
        """Get the elapsed duration in seconds."""
        return time.time() - self.start

    def get_script_path(self):
        """Qualify the path to the scap script."""
        return os.path.join(os.path.dirname(sys.argv[0]), 'scap')

    def get_keyholder_key(self):
        """Get the public key for IdentityFile use in ssh."""
        key_dir = '/etc/keyholder.d'
        key_safe_name = re.sub(r'\W', '_', self.config['ssh_user'])
        key_name = self.config.get('keyholder_key')
        if key_name is None:
            key_name = key_safe_name
        key_path = os.path.join(key_dir, '{}.pub'.format(key_name))

        if os.path.exists(key_path):
            self.get_logger().debug('Using key: %s', key_path)
            return key_path

        self.get_logger().debug(
            'Unable to find keyholder key for %s', key_safe_name)
        return None

    def get_master_list(self):
        """Get list of deploy master hostnames that should be updated before
        the rest of the cluster."""
        return targets.get('dsh_masters', self.config).all

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
        env_check = True if 'DOLOGMSGNOLOG' in os.environ else False
        if self.arguments.no_log_message or env_check:
            if env_check:
                self.get_logger().warning(
                    'DOLOGMSGNOLOG has been deprecated, use --no-log-message')
            # Do not log to the announce logger, but do log the event for the
            # console and other non-broadcast log collectors.
            self.get_logger().info(*args)
        else:
            if self._announce_logger is None:
                self._announce_logger = logging.getLogger('scap.announce')
            self._announce_logger.info(*args)

    def active_wikiversions(self, source_tree='deploy'):
        """
        Get an ordered collection of active MediaWiki versions.

        :param source_tree: Source tree to read file from: 'deploy' or 'stage'
        :returns: collections.OrderedDict of {version:wikidb} values sorted by
                  version number in ascending order
        """
        return utils.get_active_wikiversions(
            self.config[source_tree + '_dir'],
            self.config['wmf_realm'], self.config['datacenter'])

    def _process_arguments(self, args, extra_args):
        """
        Validate and process command line arguments.

        Default behavior is to abort the application with an error if any
        unparsed arguments were found.

        :returns: Tuple of (args, extra_args) after processing
        """
        if extra_args:
            self._argparser.error('extra arguments found: %s' %
                                  ' '.join(extra_args))

        if hasattr(args, 'message'):
            args.message = (' '.join(args.message) or
                            '(no justification provided)')

        return args, extra_args

    def _load_config(self):
        """Load configuration."""
        defines = None
        if self.arguments.defines:
            defines = dict(self.arguments.defines)
        self.config = config.load(
            cfg_file=self.arguments.conf_file,
            environment=self.arguments.environment,
            overrides=defines
        )

    def _setup_loggers(self):
        """Setup logging."""
        log.setup_loggers(self.config, self.arguments.loglevel)

    def _setup_environ(self):
        """Setup shell environment."""
        auth_sock = self.config.get('ssh_auth_sock')
        php_version = self.config.get('php_version')
        if php_version is not None:
            os.environ['PHP'] = php_version
        if auth_sock is not None and self.arguments.shared_authsock:
            os.environ['SSH_AUTH_SOCK'] = auth_sock

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
        self.announce('{} aborted: {} (duration: {})'.format(
                      self.program_name,
                      self.arguments.message,
                      utils.human_duration(self.get_duration())))
        return 130

    def _handle_exception(self, ex):
        """
        Handle unhandled exceptions and errors.

        :returns: exit status
        """
        logger = self.get_logger()
        exception_type = type(ex).__name__
        backtrace = True
        message = '%s failed: <%s> %s'

        if isinstance(ex, lock.LockFailedError):
            backtrace = False

        if backtrace:
            logger.warning('Unhandled error:', exc_info=True)

        logger.error(message, self.program_name, exception_type, ex)
        return 70

    def _before_exit(self, exit_status):
        """
        Do any final cleanup or processing before the application exits.

        Called after :meth:`main` and before `sys.exit` even when an exception
        occurs.

        :returns: exit status
        """
        try:
            TERM.reset_colors()
            TERM.close()
        except Exception:
            pass

        return exit_status

    def _run_as(self, user):
        """Ensure that this program is run as the given user."""
        if utils.get_username() != user:
            self.get_logger().info(
                '%s must be run as user %s', self.program_name, user)
            # Replace the current process with a sudo call to the same script
            os.execvp('sudo', ['sudo', '-u', user, '-n', '--'] + sys.argv)

    def _assert_current_user(self, user):
        """Assert that this program is run as the given user."""
        if utils.get_username() != user:
            raise RuntimeError(
                '%s must be run as user %s' % (self.program_name, user))

    def _assert_auth_sock(self):
        """Assert that SSH_AUTH_SOCK is present in the environment."""
        if 'SSH_AUTH_SOCK' not in os.environ:
            raise RuntimeError(
                '%s requires SSH agent forwarding' % self.program_name)

    @staticmethod
    def factory():
        parser = arg.build_parser()
        args, extra_args = parser.parse_known_args()
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
            datefmt='%H:%M:%S',
            stream=sys.stdout)

        # Silence noisy loggers early
        logging.getLogger('urllib3').setLevel(logging.WARNING)
        logging.getLogger('scap.sh').setLevel(logging.WARNING)

        # Setup instance for logger access
        app = cls('scap')

        exit_status = 0
        try:
            app = Application.factory()

            if os.geteuid() == 0:
                raise OSError(errno.EPERM, 'Scap should not be run as root')

            # Let each application handle `extra_args`
            app.arguments, app.extra_arguments = app._process_arguments(
                app.arguments,
                app.extra_arguments)

            app._load_config()
            app._setup_loggers()
            app._setup_environ()
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
            sys.stdout.write('\a')
            sys.stdout.flush()

        # Exit
        sys.exit(exit_status)


def argument(*args, **kwargs):
    """Decorator used to declare a command line argument on an
    :class:`Application`

    .. decorator:: argument(option_flags..[,action='store'][,nargs=1]\
       [,const=None][,default][,type=str][,choices][,required=False][,help]\
       [,dest])

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


def all_commands():
    """
    return a list of all commands that have been registered with the
    command() decorator.
    """
    global COMMAND_REGISTRY
    # prevent plugins from overwriting built-in commands by first copying the
    # COMMAND_REGISTRY and then verifying that none of the registered plugins
    # write to any of the keys used by built-in commands.
    builtin_commands = COMMAND_REGISTRY.copy()
    COMMAND_REGISTRY.clear()
    all_commands = builtin_commands.copy()

    scap.plugins.load_plugins()

    for key in COMMAND_REGISTRY.keys():
        if key in builtin_commands:
            logger = logging.getLogger()
            msg = 'Plugin (%s) attempted to overwrite builtin command: %s' % (
                COMMAND_REGISTRY[key], key)
            logger.warning(msg)
        else:
            all_commands[key] = COMMAND_REGISTRY[key]

    COMMAND_REGISTRY = builtin_commands
    return all_commands


def command(*args, **kwargs):
    """
    .. decorator:: command(command_name, help="help text",[subcommands=False])

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
        has_subcommands = kwargs.pop('subcommands', False)
        if has_subcommands:
            setattr(cls, arg.ATTR_SUBPARSER, True)

        cmd = dict(name=name, cls=cls, args=args, kwargs=kwargs)
        COMMAND_REGISTRY[name] = cmd
        return cls
    return wrapper


def subcommand(name=None):
    """
    .. decorator:: subcommand(command_name)

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
