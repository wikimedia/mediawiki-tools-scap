# -*- coding: utf-8 -*-
"""
    scap.cli
    ~~~~~~~~
    Classes and helpers for creating command line interfaces

"""
import logging
import os
import sys
import time
import scap.plugins

from . import arg
from . import config
from . import log
from . import utils


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

    @property
    def verbose(self):
        return self.arguments.loglevel < logging.INFO

    def get_duration(self):
        """Get the elapsed duration in seconds."""
        return time.time() - self.start

    def get_script_path(self, script_name='scap'):
        """Qualify the path to a scap script."""
        return os.path.join(self.config['bin_dir'], script_name)

    def announce(self, *args):
        """
        Announce a message to broadcast listeners.

        Emits a logging event to the 'scap.announce' logger which can be
        configured to broadcast messages via irc or another real-time
        notification channel.

        Announcements can be disabled by setting 'DOLOGMSGNOLOG' in the calling
        shell environment (e.g. `DOLOGMSGNOLOG=1 scap sync-file foo.php`). In
        this case the log event will still be emitted to the normal logger as
        though `self.get_logger().info()` was used instead of
        `self.announce()`.
        """
        if 'DOLOGMSGNOLOG' in os.environ:
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
            args.message = ' '.join(args.message) or '(no message)'

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

    def _handle_system_exit(self, ex):
        """
        Handle a SystemExit error.

        :returns: exit status
        """
        raise

    def _handle_keyboard_interrupt(self, ex):
        """
        Handle ctrl-c from interactive user.

        :returns: exit status
        """
        self.get_logger().warning('%s aborted', self.program_name)
        return 130

    def _handle_exception(self, ex):
        """
        Handle unhandled exceptions and errors.

        :returns: exit status
        """
        self.get_logger().warn('Unhandled error:', exc_info=True)
        self.get_logger().error(
            '%s failed: <%s> %s', self.program_name, type(ex).__name__, ex)
        return 70

    def _before_exit(self, exit_status):
        """
        Do any final cleanup or processing before the application exits.

        Called after :meth:`main` and before `sys.exit` even when an exception
        occurs.

        :returns: exit status
        """
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
    def factory(argv):
        parser = arg.build_parser()
        args, extra_args = parser.parse_known_args()
        app = args.which(args.command)
        app._argparser = parser
        app.arguments = args
        app.extra_arguments = extra_args

        return app

    @classmethod
    def run(cls, argv=sys.argv):
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
            datefmt='%H:%M:%S')

        # Silence this noisy logger early
        logging.getLogger('urllib3').setLevel(logging.WARNING)

        # Setup instance for logger access
        app = cls('scap')

        exit_status = 0
        try:
            if os.geteuid() == 0:
                raise SystemExit('Scap should not be run as root')

            if len(argv) == 2 and (argv[1] == '--version' or argv[1] == '-V'):
                show_version()

            app = Application.factory(argv)

            # Let each application handle `extra_args`
            app.arguments, app.extra_arguments = app._process_arguments(
                app.arguments,
                app.extra_arguments)

            if app.arguments.show_version is True:
                show_version()

            app._load_config()
            app._setup_loggers()
            app._setup_environ()
            if "subcommand" in app.arguments and app.arguments.subcommand:
                method = app.arguments.subcommand
                exit_status = method(app, app.extra_arguments)
            else:
                exit_status = app.main(app.extra_arguments)

        except SystemExit as ex:
            # Triggered by sys.exit() calls
            exit_status = app._handle_system_exit(ex)

        except KeyboardInterrupt as ex:
            # Handle ctrl-c from interactive user
            exit_status = app._handle_keyboard_interrupt(ex)

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


def show_version():
    print(scap.__version__)
    sys.exit(0)


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


command_registry = {}


def all_commands():
    """
    return a list of all commands that have been registered with the
    command() decorator.
    """
    global command_registry
    # prevent plugins from overwriting built-in commands by first copying the
    # command_registry and then verifying that none of the registered plugins
    # write to any of the keys used by built-in commands.
    builtin_commands = command_registry.copy()
    command_registry.clear()
    all_commands = builtin_commands.copy()

    scap.plugins.load_plugins()

    for key in command_registry.keys():
        if key in builtin_commands:
            logger = logging.getLogger()
            msg = 'Plugin (%s) attempted to overwrite builtin command: %s' % (
                command_registry[key], key)
            logger.warning(msg)
        else:
            all_commands[key] = command_registry[key]

    command_registry = builtin_commands
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
        global command_registry
        name = args[0]
        if name in command_registry:
            err = 'Duplicate: A command named "%s" already exists.' % name
            raise ValueError(err)
        has_subcommands = kwargs.pop('subcommands', False)
        if has_subcommands:
            setattr(cls, arg.ATTR_SUBPARSER, True)

        cmd = dict(name=name, cls=cls, args=args, kwargs=kwargs)
        command_registry[name] = cmd
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
