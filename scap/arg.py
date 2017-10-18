# -*- coding: utf-8 -*-
"""
    scap.arg
    ~~~~~~~~
    Helpers for creating a fancy argparser. Most of the externally useful API
    for command line arg parsing is found in scap.cli

    :Author: Tyler Cipriani <thcipriani@wikimedia.org>
    :Author: Mukunda Modell <mmodell@wikimedia.org>

    :Copyright: Wikimedia Foundation, Inc.
    :License: GPL v3.0

    Parts of the shell argument completion code in this file is derived from
    `python-selfcompletion`_.

    .. _python-selfcompletion: https://github.com/dbarnett/python-selfcompletion # noqa

    :Author: David Barnett <davidbarnett2@gmail.com>
    :License: BSD

    .. seealso::
       * :func:`scap.cli.command`
       * :func:`scap.cli.argument`

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
import inspect
import logging
import os
import re

import scap
import scap.plugins

ATTR_SUBPARSER = '_app_subparser'
ATTR_ARGUMENTS = '_app_arguments'
ATTR_SUBCOMMAND = '_app_subcmd_name'


class _ScapAutoCompleteAction(argparse.Action):

    def __init__(self,
                 option_strings,
                 dest='_completion',
                 metavar='CMD_LINE',
                 default=argparse.SUPPRESS,
                 help=argparse.SUPPRESS):
        super(_ScapAutoCompleteAction, self).__init__(
            option_strings=option_strings,
            dest=dest,
            metavar=metavar,
            default=default,
            nargs=None,
            type=str,
            help=help)

    def __call__(self, parser, namespace, values, option_string=None):
        comp_words = re.split(r'\s+', values.lstrip())
        wordcount = 0
        for valid_word in parser.get_valid_next_words(comp_words):
            print(valid_word)
            wordcount += 1

        if wordcount > 0:
            parser.exit(0)
        else:
            parser.exit(2)


def is_dir(string):
    ''' represents a cli argument which accepts only a valid directory name '''
    if not os.path.isdir(string):
        message = "Argument '%s' is not a valid directory path."
        raise argparse.ArgumentTypeError(message % string)
    return os.path.normpath(string)


def is_version(string):
    ''' represents a cli argument which accepts a mediawiki branch version '''
    if string.startswith('php-'):
        string = string[4:]
    return string.strip('/')


class ScapArgParser(argparse.ArgumentParser):
    """Scap argparse subclass

    Created to allow for easier, scripted, autocompletion
    """

    def __init__(self, *args, **kwargs):
        self._autocomplete_options = []
        if 'conflict_handler' not in kwargs:
            kwargs['conflict_handler'] = 'resolve'
        super(ScapArgParser, self).__init__(*args, **kwargs)
        if '_COMPLETION' in os.environ:
            completion_action = _ScapAutoCompleteAction(['--_completion'])
            self._add_action(completion_action)

    def add_arguments(self, local_args):
        for argspec in reversed(local_args):
            flags = argspec.pop('_flags')
            self.add_argument(*flags, **argspec)
            argspec['_flags'] = flags

    def get_valid_next_words(self, words, more_valid_words=None):
        """ get completion words for cli auto-complete """
        if len(words) < 1:
            words.append('')
        valid_words = set()
        if more_valid_words:
            valid_words.update(more_valid_words)
        types = []
        word_is_optarg = False
        positionals_valid = True
        i = -1
        for w in words:
            i += 1
            if w in self._option_string_actions:
                # handle args which come after an --option
                action = self._option_string_actions[w]
                action_nargs = (1 if action.nargs is None else action.nargs)
                if action_nargs == 1 and action.choices:
                    valid_words = set()
                    valid_words.update([c + ' ' for c in action.choices])
                if action_nargs == i + 1:
                    word_is_optarg = True
                    types.append(action.type)
                    break
                if i == 0 and action.nargs == '?':
                    positionals_valid = False
                    types.append(action.type)
                    if action.choices:
                        valid_words.update([c + ' ' for c in action.choices])
                    break
        if not word_is_optarg:
            if '--' not in words[:-1]:
                for a in self._actions:
                    valid_words.update([o + ' ' for o in a.option_strings])
            if positionals_valid:
                positionals = self._get_positional_actions()
                for action in positionals:
                    if action.default == '==SUPPRESS==':
                        continue
                    choices = action.choices
                    if hasattr(action, 'add_parser'):
                        # recurse into subparsers
                        subwords = []
                        subwords.extend(words)
                        for word in words:
                            subwords = subwords[1:]
                            if word in choices:
                                subparser = choices[word]
                                valid_words = {w for w in valid_words
                                               if w.startswith('-')}
                                return subparser.get_valid_next_words(
                                    subwords, valid_words)
                        last = None
                        for word in reversed(words):
                            if word and not word.startswith('-'):
                                last = word
                                break
                        if last in choices:
                            return choices[last].get_valid_next_words(words)

                    if action.type is not None:
                        types.append(action.type)
                    if choices:
                        valid_words.update([c + ' ' for c in choices])
                    elif isinstance(action.default, str):
                        valid_words.add("'%s'" % action.default)
                    elif isinstance(action.default, list):
                        for item in action.default:
                            valid_words.add("'%s'" % item)
                    if action.nargs in (None, 1):
                        break

        if int in types:
            if re.match(r'\d*$', words[-1]):
                valid_words.update(['%s%d' % (words[-1], j)
                                    for j in xrange(10)])

        for word in words:
            valid_words.discard(word.strip())

        if is_dir in types:
            valid_words.add('__dirs__')

        if is_version in types:
            valid_words.add('__versions__')

        for t in types:
            cls = type(t)
            if t is argparse.FileType or \
               cls is argparse.FileType or \
               t is file or cls is file:
                valid_words.add('__files__')

        if words[-1].strip():
            valid_words = {w for w in valid_words if w.startswith(words[-1])}
        valid_words.discard('--_completion ')
        valid_words.discard('-')
        return valid_words


class ScapHelpFormatter(argparse.HelpFormatter):
    """Formatter that respects argparse.SUPPRESS for subparser actions."""

    def _format_action(self, action):
        if not action.help == argparse.SUPPRESS:
            return super(ScapHelpFormatter, self)._format_action(action)


def build_parser():
    """Build an argument parser for all ``cli.Application``'s."""
    parser = ScapArgParser(formatter_class=ScapHelpFormatter)

    global_parser = get_global_parser()

    desc = 'Available scap commands are listed below. \
    For help with a particular command, run `scap <command> -h`'

    subparsers = parser.add_subparsers(
        title='scap commands', metavar='<command>',
        parser_class=ScapArgParser, description=desc)

    cmds = scap.cli.all_commands()

    for cmd in sorted(cmds.values(), key=lambda x: x['name']):
        build_subparser(cmd, subparsers, global_parser)

    return parser


def get_global_parser():
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
    parser = ScapArgParser(formatter_class=ScapHelpFormatter, add_help=False)

    # global args are grouped into a separate help section
    title = 'global arguments'
    desc = "Although these arguments can be passed to all scap (sub-)commands,\
         \nnot all commands are affected by every global argument."
    group = parser.add_argument_group(title, desc)

    default_loglevel = os.getenv('SCAP_LOG_LEVEL', logging.INFO)

    group.add_argument(
        '-c', '--conf', dest='conf_file',
        type=argparse.FileType('r'),
        help='Path to configuration file')
    group.add_argument(
        '--no-shared-authsock', dest='shared_authsock',
        action='store_false',
        help='Ignore any shared ssh-auth configuration')
    group.add_argument(
        '-D', '--define', dest='defines',
        action='append',
        type=lambda v: tuple(v.split(':')),
        help='Set a configuration value',
        metavar='<name>:<value>')
    group.add_argument(
        '-v', '--verbose', action='store_const',
        const=logging.DEBUG, default=default_loglevel,
        dest='loglevel', help='Verbose output')
    group.add_argument(
        '-e', '--environment', default=None,
        help='environment in which to execute scap')

    return parser


def extract_help_from_object(obj):
    doc = inspect.getdoc(obj) or ""

    lines = doc.strip().splitlines()
    if len(lines) > 1:
        return dict(help=lines[0], description=lines[0],
                    epilog="\n".join(lines[1::]).strip(),
                    formatter_class=argparse.RawDescriptionHelpFormatter)
    return dict(help=doc, description=doc, epilog=None,
                formatter_class=argparse.HelpFormatter)


def build_subparser(cmd, parser, global_parser):
    """Append subparsers to ``cli.Application``'s agparser using decorators."""

    cls = cmd['cls']
    kwargs = extract_help_from_object(cls)
    kwargs.update(cmd['kwargs'])

    has_subparsers = getattr(cls, ATTR_SUBPARSER, False)
    kwargs['parents'] = [global_parser]
    sub = parser.add_parser(*cmd['args'], **kwargs)
    sub.set_defaults(which=cls, command=cmd['name'])

    if has_subparsers:
        class_level_args = getattr(cls, ATTR_ARGUMENTS, [])

        if class_level_args:
            class_parser = ScapArgParser(formatter_class=ScapHelpFormatter)
            class_parser.add_arguments(class_level_args)
            kwargs['parents'] = [class_parser, global_parser]
        else:
            kwargs['parents'] = [global_parser]

        kwargs['epilog'] = ''
        subsubparsers = sub.add_subparsers(
            title="%s sub-commands" % cmd['name'],
            metavar='<sub-command>',
            parser_class=ScapArgParser,
            dest='subcommand',
            description=kwargs['help'])
        cls_methods = inspect.getmembers(cls, inspect.ismethod)

        for method_name, method in cls_methods:
            local_args = getattr(method, ATTR_ARGUMENTS, None)
            subcmd_name = getattr(method, ATTR_SUBCOMMAND, method_name)
            if local_args is not None:
                kwargs.update(extract_help_from_object(method))
                kwargs['add_help'] = False
                subsubparser = subsubparsers.add_parser(subcmd_name, **kwargs)
                subsubparser.set_defaults(subcommand=method)
                if local_args:
                    subsubparser.add_arguments(local_args)
    else:
        method = getattr(cls, 'main')
        local_args = getattr(method, ATTR_ARGUMENTS, [])
        sub.add_arguments(local_args)
