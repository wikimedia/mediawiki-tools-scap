# -*- coding: utf-8 -*-
"""
    scap.plugins.say
    ~~~~~~~~~~~~~~~~

    Infinitely superior to cowsay.
"""
from __future__ import print_function

import argparse
import os
import random
import sys
import textwrap

from scap import cli
from scap import utils


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
