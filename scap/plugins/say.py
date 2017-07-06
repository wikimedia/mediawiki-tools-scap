# -*- coding: utf-8 -*-
"""
    scap.plugins.say
    ~~~~~~~~~~~~~~~~

    Infinitely superior to cowsay.

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
import os
import random
import sys
import textwrap

import scap.cli as cli
import scap.utils as utils


def fortune():
    """Get a random fortune."""
    return random.choice([
        'S.C.A.P.: silencing communist American perpetrators',
        'S.C.A.P.: spreading communist American propaganda',
        'S.C.A.P.: someone can always pontificate',
        'S.C.A.P.: scap can absolutely pollute',
        'S.C.A.P.: sync cars and planes',
        'S.C.A.P.: scatter crap around production',
        'S.C.A.P.: succulent cacti are plentiful',
        'S.C.A.P.: sorcerer\'s cats are powerful',
        'S.C.A.P.: synthesizing carbs and protein',
        'S.C.A.P.: spiritually crippling, although pragmatic',
        'S.C.A.P.: smart container abstraction package',
        'S.C.A.P.: say command as a plugin',
        'S.C.A.P.: staunch conservatives are pissed',
        'S.C.A.P.: still crashes after patching',
        'S.C.A.P.: say command == ascii pig',
        'S.C.A.P.: shell commands are perplexing',
        'S.C.A.P.: salt, cfengine, ansible & puppet',
        'S.C.A.P.: someone called about a party?',
        'S.C.A.P.: sync commits, all python',
        'S.C.A.P.: small containers ate production',
        'S.C.A.P.: sheep comandeering a powerboat',
        'S.C.A.P.: synchronize, corrupt and push',
    ])


def scap_say(words=None, eyes=None, width=None):
    """Make the scap pig say stuff."""
    if not words:
        words = fortune()

    if not width:
        width = min([50, os.environ.get('COLUMNS', 50)])

    txt_width = width - 5
    box_width = width - 2

    if len(words) > txt_width:
        words = textwrap.wrap(words, txt_width)
    else:
        words = [words]

    lines = [' {:-^{width}}\n/{:^{width}}\\'.format('', '', width=box_width)]
    lines += ['|{:^{width}}|'.format(word, width=box_width) for word in words]

    lines.append('\{:^{width}}/\n {:-^{width}}'.format('', '',
                                                       width=box_width))
    lines.append('{:^10}'.format('\\'))
    lines.append('{:^11}'.format('\\'))
    lines.append('{:^13}'.format('\\'))
    lines.append(utils.logo(eyes=eyes))
    return '\n'.join(lines)


@cli.command('fortune', help=argparse.SUPPRESS)
class Fortune(cli.Application):
    """Scap propaganda of a middling order."""
    def main(self, *extra_args):
        print(fortune())


@cli.command('say', help=argparse.SUPPRESS)
class Say(cli.Application):
    """Scap propaganda of the lowest order."""

    @cli.argument('-W', '--width', type=int,
                  help='Column width for message box')
    @cli.argument('-e', '--eyes', type=lambda s: unicode(s, 'utf8'),
                  help='Eyes')
    @cli.argument('propaganda', nargs='*', help='Message to print')
    def main(self, *extra_args):
        msg = ' '.join(self.arguments.propaganda)

        if not msg:
            msg = sys.stdin.read()

        msg = msg.replace('\n', ' ').strip()
        print(
            scap_say(
                msg,
                eyes=self.arguments.eyes,
                width=self.arguments.width
            )
        )
        return 0
