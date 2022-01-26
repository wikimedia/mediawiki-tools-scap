# -*- coding: utf-8 -*-
"""For updating + syncing the interwiki cache."""
import errno
import os
import subprocess

import scap.cli as cli
import scap.lint as lint


@cli.command("update-interwiki-cache")
class UpdateInterwikiCache(cli.Application):
    """Scap sub-command to update the interwiki cache."""

    @cli.argument(
        "--beta", action="store_true", help="Update the Beta Cluster interwiki cache file"
    )
    def main(self, *extra_args):
        """Update the latest interwiki cache."""
        interwiki_file = "interwiki-labs.php" if self.arguments.beta else "interwiki.php"
        interwiki_file = os.path.join(self.config["stage_dir"], "wmf-config", interwiki_file)

        if not os.path.exists(interwiki_file):
            raise IOError(errno.ENOENT, "File/directory not found", interwiki_file)

        with open(interwiki_file, "w") as outfile:
            subprocess.check_call(
                [
                    "/usr/local/bin/mwscript",
                    "extensions/WikimediaMaintenance/dumpInterwiki.php",
                ],
                stdout=outfile,
            )

        # This shouldn't happen, but let's be safe
        lint.check_valid_syntax(interwiki_file)
