# -*- coding: utf-8 -*-
"""
    scap.cli
    ~~~~~~~~
    Classes and helpers for creating command line interfaces

"""
import argparse
import collections
import distutils.version
import json
import logging
import os
import sys
import time

from . import config
from . import log
from . import utils


ATTR_ARGUMENTS = '_app_arguments'


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

    def get_script_path(self, script_name):
        """Qualify the path to a scap script."""
        return os.path.join(self.config['bin_dir'], script_name)

    def announce(self, *args):
        """Announce a message to broadcast listeners.

        Emits a logging event to the 'scap.announce' logger which can be
        configured to broadcast messages via irc or another real-time
        notification channel.

        Announcements can be disabled by setting 'DOLOGMSGNOLOG' in the calling
        shell environment (e.g. `DOLOGMSGNOLOG=1 sync-file foo.php`). In this
        case the log event will still be emitted to the normal logger as
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
        """Get an ordered collection of active MediaWiki versions.

        :param source_tree: Source tree to read file from: 'deploy' or 'stage'
        :returns: collections.OrderedDict of {version:wikidb} values sorted by
                  version number in ascending order
        """
        directory = self.config[source_tree + '_dir']
        path = utils.get_realm_specific_filename(
            os.path.join(directory, 'wikiversions.json'),
            self.config['wmf_realm'], self.config['datacenter'])

        with open(path) as f:
            wikiversions = json.load(f)

        versions = {}
        for wikidb, version in wikiversions.items():
            version = version[4:]  # trim 'php-' from version
            if version not in versions:
                versions[version] = wikidb

        # Convert to list of (version, db) tuples sorted by version number
        # and then convert that list to an OrderedDict
        sorted_versions = collections.OrderedDict(sorted(versions.iteritems(),
            key=lambda v: distutils.version.LooseVersion(v[0])))

        return sorted_versions

    def _parse_arguments(self, argv):
        """Parse command line arguments.

        :returns: Tuple of parsed argument Namespace and list of any extra
                  arguments that were not parsed.
        """
        self._argparser = self._build_argparser()
        return self._argparser.parse_known_args(argv)

    def _build_argparser(self):
        parser = argparse.ArgumentParser(description=self.__doc__)

        # Look for arguments that were added to our main method
        local_args = getattr(self.main, ATTR_ARGUMENTS, [])

        # List is built from the bottom up so reverse it to make the parser
        # arguments read in the same order as they were declared.
        for argspec in reversed(local_args):
            flags = argspec.pop('_flags')
            parser.add_argument(*flags, **argspec)

        parser.add_argument('-c', '--conf', dest='conf_file',
            type=argparse.FileType('r'),
            help='Path to configuration file')
        parser.add_argument('-D', '--define', dest='defines',
            action='append', type=lambda v: tuple(v.split(':')),
            help='Set a configuration value', metavar='<name>:<value>')
        parser.add_argument('-v', '--verbose', action='store_const',
            const=logging.DEBUG, default=logging.INFO, dest='loglevel',
            help='Verbose output')
        parser.add_argument('--no-shared-authsock', dest='shared_authsock',
            action='store_false',
            help='Ignore any shared ssh-auth configuration')
        parser.add_argument('-e', '--environment', default=None,
              help='environment in which to execute scap')

        return parser

    def _process_arguments(self, args, extra_args):
        """Validate and process command line arguments.

        Default behavior is to abort the application with an error if any
        unparsed arguments were found.

        :returns: Tuple of (args, extra_args) after processing
        """
        if extra_args:
            self._argparser.error('extra arguments found: %s' %
                ' '.join(extra_args))
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
        """Main business logic of the application.

        Parsed command line arguments are available in self.arguments. Global
        configuration is available in self.config. Unparsed command line
        arguments are passed as positional arguments.

        :returns: exit status
        """
        raise NotImplementedError()

    def _handle_system_exit(self, ex):
        """Handle a SystemExit error.

        :returns: exit status
        """
        raise

    def _handle_keyboard_interrupt(self, ex):
        """Handle ctrl-c from interactive user.

        :returns: exit status
        """
        self.get_logger().warning('%s aborted', self.program_name)
        return 130

    def _handle_exception(self, ex):
        """Handle unhandled exceptions and errors.

        :returns: exit status
        """
        self.get_logger().warn('Unhandled error:', exc_info=True)
        self.get_logger().error('%s failed: <%s> %s',
            self.program_name, type(ex).__name__, ex)
        return 70

    def _before_exit(self, exit_status):
        """Do any final cleanup or processing before the application exits.

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

    @classmethod
    def run(cls, argv=sys.argv, exit=True):
        """Construct and run an application.

        Calls ``sys.exit`` with exit status returned by application by
        default. Setting ``exit`` to ``False`` will instead return the class
        instance and it's exit status. This would generally only be done when
        testing.

        :param cls: Class to create and run
        :param argv: Command line arguments
        :param exit: Call sys.exit after execution
        :returns: Tuple of class instance and exit status when not exiting
        """
        # Bootstrap the logging system
        logging.basicConfig(
            level=logging.INFO,
            format=log.CONSOLE_LOG_FORMAT,
            datefmt='%H:%M:%S')

        argv = list(argv)
        app = cls(argv.pop(0))
        exit_status = 0
        try:
            args, extra_args = app._parse_arguments(argv)
            args, extra_args = app._process_arguments(args, extra_args)
            app.arguments = args
            app._load_config()
            app._setup_loggers()
            app._setup_environ()
            exit_status = app.main(extra_args)

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

        if exit:
            # Flush logger before exiting
            logging.shutdown()

            sys.exit(exit_status)
        else:
            return (app, exit_status)


def argument(*args, **kwargs):
    """Decorator used to declare a command line argument on a
    :class:`Application`'s ``main`` method.

    Use with the same signature as ``ArgumentParser.add_argument``.
    """
    def wrapper(func):
        arguments = getattr(func, ATTR_ARGUMENTS, [])
        arguments.append(dict(_flags=args, **kwargs))
        setattr(func, ATTR_ARGUMENTS, arguments)
        return func
    return wrapper