# -*- coding: utf-8 -*-
"""
    scap.say
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
import argparse
import os
import random
import sys
import textwrap

from scap import ansi
from scap import cli


def fortune():
    """Get a random fortune."""
    return random.choice(
        [
            "S.C.A.P.: silencing communist American perpetrators",
            "S.C.A.P.: spreading communist American propaganda",
            "S.C.A.P.: someone can always pontificate",
            "S.C.A.P.: scap can absolutely pollute",
            "S.C.A.P.: sync cars and planes",
            "S.C.A.P.: scatter crap around production",
            "S.C.A.P.: succulent cacti are plentiful",
            "S.C.A.P.: sorcerer's cats are powerful",
            "S.C.A.P.: synthesizing carbs and protein",
            "S.C.A.P.: spiritually crippling, although pragmatic",
            "S.C.A.P.: smart container abstraction package",
            "S.C.A.P.: say command as a pidgin",
            "S.C.A.P.: staunch conservatives are pissed",
            "S.C.A.P.: still crashes after patching",
            "S.C.A.P.: say command == ascii pig",
            "S.C.A.P.: shell commands are perplexing",
            "S.C.A.P.: salt, cfengine, ansible & puppet",
            "S.C.A.P.: someone called about a party?",
            "S.C.A.P.: sync commits, all python",
            "S.C.A.P.: small containers ate production",
            "S.C.A.P.: sheep comandeering a powerboat",
            "S.C.A.P.: synchronize, corrupt and push",
            "S.C.A.P.: ship, crash, apply patch",
            "S.C.A.P.: ship code and pray",
            "S.C.A.P.: some chaos and pandemonium",
            "S.C.A.P.: scaring children away promptly",
            "S.C.A.P.: soilent contents are people",
            "S.C.A.P.: say carl and pause",
            "S.C.A.P.: stupid captions annotate pictures",
            "S.C.A.P.: sh -c awk | perl",
            "S.C.A.P.: sulphur, carbon, arsenic, phosphorus",
            "S.C.A.P.: syntax: conjunction, article, pronoun",
            "S.C.A.P.: some cats and puppies",
            "S.C.A.P.: seriously cursed auxiliary program",
            "S.C.A.P.: someone copying and pasting - xkcd.com/2565",
        ]
    )


def scap_say(words=None, eyes=None, width=None, nowrap=False, color=True):
    """Make the scap pig say stuff."""
    if not words:
        words = fortune()

    if not width and not nowrap:
        width = min([50, int(os.environ.get("COLUMNS", 50))])

    if nowrap:
        width = max(list(map(len, words))) + 2

    txt_width = width - 5
    box_width = width - 2

    if not nowrap:
        word_string = " ".join(words)
        if len(word_string) > txt_width:
            words = textwrap.wrap(word_string, txt_width)

    lines = [" {:-^{width}}\n/{:^{width}}\\".format("", "", width=box_width)]
    if nowrap:
        lines += ["|{:<{width}}|".format(word, width=box_width) for word in words]
    else:
        lines += ["|{:^{width}}|".format(word, width=box_width) for word in words]

    lines.append(
        r"\{:^{width}}/{newline} {:-^{width}}".format(
            "", "", newline="\n", width=box_width
        )
    )

    lines.append("{:^10}".format("\\"))
    lines.append("{:^11}".format("\\"))
    lines.append("{:^13}".format("\\"))
    lines.append(ansi.logo(eyes=eyes, color=color))
    return "\n".join(lines)


@cli.command("fortune", help=argparse.SUPPRESS)
class Fortune(cli.Application):
    """Scap propaganda of a middling order."""

    def main(self, *extra_args):
        print(fortune())

        # If we don't return 0, this will ring the term bell!
        return 0


@cli.command("say", help=argparse.SUPPRESS)
class Say(cli.Application):
    """Scap propaganda of the lowest order."""

    @cli.argument("-W", "--width", type=int, help="Column width for message box")
    @cli.argument("-e", "--eyes", type=str, help="Eyes")
    @cli.argument("-n", "--no-wrap", action="store_true", help="No Wordwrap")
    @cli.argument("-c", "--color", action="store_true", help="Color logo")
    @cli.argument("propaganda", nargs="*", help="Message to print")
    def main(self, *extra_args):
        msg = self.arguments.propaganda

        if not msg:
            msg = sys.stdin.readlines()

        msg = [x.rstrip("\n") for x in msg]

        print(
            scap_say(
                msg,
                eyes=self.arguments.eyes,
                width=self.arguments.width,
                nowrap=self.arguments.no_wrap,
                color=self.arguments.color,
            )
        )
        return 0
