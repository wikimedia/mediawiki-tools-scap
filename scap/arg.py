# -*- coding: utf-8 -*-
"""
    scap.args
    ~~~~~~~~
    Helpers for creating a fancy argparser

"""
import argparse
import logging
import sys

import scap
import scap.plugins

ATTR_SUBPARSER = '_app_subparser'
ATTR_ARGUMENTS = '_app_arguments'


class _ScapAutoCompleteAction(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        print parser.completions

        # Autocompletion is all we want, exit here
        sys.exit(0)


class ScapArgParser(argparse.ArgumentParser):
    """Scap argparse subclass

    Created to allow for easier, scripted, autocompletion
    """
    def __init__(self, *args, **kwargs):
        self._autocomplete_options = []
        return super(ScapArgParser, self).__init__(*args, **kwargs)

    def add_argument(self, *args, **kwargs):
        for arg in args:
            if not arg.startswith('-') or arg == '--_autocomplete':
                # skip positional arguments
                continue
            self.add_completion(arg)

        super(ScapArgParser, self).add_argument(*args, **kwargs)

    def add_completion(self, arg):
        self._autocomplete_options.append(arg)

    @property
    def completions(self):
        return '\n'.join(self._autocomplete_options)


class ScapHelpFormatter(argparse.HelpFormatter):
    """Formatter that respects argparse.SUPPRESS for subparser actions."""

    def _format_action(self, action):
        if not action.help == argparse.SUPPRESS:
            return super(ScapHelpFormatter, self)._format_action(action)


def build_parser(script=None):
    """Build an argument parser for all ``cli.Application``'s."""
    parser = ScapArgParser(formatter_class=ScapHelpFormatter)

    desc = 'If you\'re attempting a full scap, try: `scap sync \'message\'`'

    # Path used during transition to subcommands only
    if script:
        parser = build_subargparser(script, parser)
        parser.set_defaults(which=script)
        return parser

    # Add a hidden --list-options arg for completion purposes
    parser.add_argument(
        '--_autocomplete', action=_ScapAutoCompleteAction,
        nargs=0, help=argparse.SUPPRESS)

    subparsers = parser.add_subparsers(
        title='command', metavar='<command>',
        parser_class=ScapArgParser, description=desc)

    cmds = scap.cli.all_commands()

    for cmd in sorted(cmds.values(), key=lambda x: x['name']):
        parser.add_completion(cmd['name'])
        build_subparser(cmd['cls'], subparsers)

    return parser


def add_base_arguments(parser):
    """
    Add standard arguments to argparser.

    These arguments should be present on all subparsers.

    ..info::
        The other option with these commands would be to make them
        into top-level flags for scap; however, that ends up feeling
        clunky with commands like:

            scap --verbose sync

        Or

            scap -e beta deploy --repo mockbase/deploy

    """
    parser.add_argument(
        '-c', '--conf', dest='conf_file',
        type=argparse.FileType('r'),
        help='Path to configuration file')
    parser.add_argument(
        '--no-shared-authsock', dest='shared_authsock',
        action='store_false',
        help='Ignore any shared ssh-auth configuration')
    parser.add_argument(
        '-D', '--define', dest='defines',
        action='append',
        type=lambda v: tuple(v.split(':')),
        help='Set a configuration value',
        metavar='<name>:<value>')
    parser.add_argument(
        '-v', '--verbose', action='store_const',
        const=logging.DEBUG, default=logging.INFO,
        dest='loglevel', help='Verbose output')
    parser.add_argument(
        '-e', '--environment', default=None,
        help='environment in which to execute scap')

    return parser


def build_subparser(cls, subparser):
    """Append subparsers to ``cli.Application``'s agparser using decorators."""
    local_subparser = getattr(cls, ATTR_SUBPARSER)

    doc = cls.__doc__.splitlines() if cls.__doc__ else [None]
    desc = doc[0]
    default_help = ''

    if desc:
        default_help = desc.splitlines()[0]

    if len(doc) > 1:
        epilog = "\n".join(doc[1::]).strip()
        formatter = argparse.RawDescriptionHelpFormatter
    else:
        epilog = None
        formatter = argparse.HelpFormatter

    flags = local_subparser.pop('_flags')
    kwargs = dict(
        help=default_help,
        description=desc,
        epilog=epilog,
        formatter_class=formatter)

    kwargs.update(local_subparser)
    sub = subparser.add_parser(*flags, **kwargs)
    sub.set_defaults(which=cls)
    build_subargparser(cls, sub)
    local_subparser['_flags'] = flags


def build_subargparser(app, parser):
    """Add arguments for subparsers."""
    main_func = getattr(app, 'main')
    local_args = getattr(main_func, ATTR_ARGUMENTS, [])

    # List is built from the bottom up so reverse it to make the parser
    # arguments read in the same order as they were declared.
    for argspec in reversed(local_args):
        flags = argspec.pop('_flags')
        parser.add_argument(*flags, **argspec)

    # Add a hidden --list-options arg for completion purposes
    parser.add_argument(
        '--_autocomplete', action=_ScapAutoCompleteAction,
        nargs=0, help=argparse.SUPPRESS)

    parser = add_base_arguments(parser)
    return parser
