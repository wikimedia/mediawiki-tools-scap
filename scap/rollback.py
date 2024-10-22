# -*- coding: utf-8 -*-
"""
    scap.rollback
    ~~~~~~~~~~~
    Checkout previously prepared working copies of MediaWiki repos and perform
    an immediate sync to restore production to a known working state.

    Copyright © 2014-2022 Wikimedia Foundation and Contributors.

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

import subprocess

from scap import cli, prep


@cli.command(
    "rollback",
    help="Restore production to a previous working state",
    primary_deploy_server_only=True,
)
class Rollback(cli.Application):
    """Checkout previously prepared working copies of MediaWiki repos and perform an immediate sync to restore production to a known working state."""

    def main(self, *extra_args):
        """
        Prompt the user to select a previously prepared and synced state using
        `scap prep auto --history` and then perform `scap sync-world`.
        """

        try:
            self.scap_check_call(["prep", "auto", "--history"])
        except subprocess.CalledProcessError as e:
            if e.returncode == prep.HISTORY_ABORT_STATUS:
                return
            raise e

        self.scap_check_call(["sync-world"])
