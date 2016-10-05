# -*- coding: utf-8 -*-
"""
    scap.plugins.say
    ~~~~~~~~~~~~~~~~

    Infinitely superior to cowsay.
"""

from scap import cli, utils


@cli.command('say')
class Say(cli.Application):
    """Scap propogranda of the lowest order."""

    @cli.argument('-w', '--width', type=int,
                  help='Column width for message box')
    @cli.argument('message', nargs='*', help='message to print')
    def main(self, *extra_args):
        print utils.scap_say(
            ' '.join(self.arguments.message), width=self.arguments.width)
        return 0
