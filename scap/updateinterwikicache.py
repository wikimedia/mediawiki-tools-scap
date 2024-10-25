# -*- coding: utf-8 -*-
"""For updating the interwiki cache."""
import errno
import os
import subprocess

import scap.cli as cli
import scap.lint as lint
import scap.utils as utils


@cli.command("update-interwiki-cache")
class UpdateInterwikiCache(cli.Application):
    """Scap sub-command to update the interwiki cache."""

    @cli.argument(
        "--beta",
        action="store_true",
        help="Update the Beta Cluster interwiki cache file",
    )
    def main(self, *extra_args):
        """Update the latest interwiki cache."""
        interwiki_file = (
            "interwiki-labs.php" if self.arguments.beta else "interwiki.php"
        )
        interwiki_file = os.path.join(
            self.config["stage_dir"], "wmf-config", interwiki_file
        )

        if not os.path.exists(interwiki_file):
            raise IOError(errno.ENOENT, "File/directory not found", interwiki_file)

        with utils.temp_to_permanent_file(interwiki_file) as f:
            with utils.suppress_backtrace():
                subprocess.check_call(
                    [
                        "/usr/local/bin/mwscript",
                        "extensions/WikimediaMaintenance/dumpInterwiki.php",
                    ],
                    stdout=f,
                )

        # This shouldn't be needed, but let's be safe
        lint.check_valid_syntax(interwiki_file)
        self.get_logger().info(
            "wmf-config/interwiki.php updated; please commit it to Gerrit and deploy it"
        )
        self.get_logger().info(
            "See <https://wikitech.wikimedia.org/wiki/Update_the_interwiki_cache>."
        )
