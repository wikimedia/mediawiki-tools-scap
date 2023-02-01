# -*- coding: utf-8 -*-
"""Updates wiki versions json files and symlink pointers."""

import json
import os

import scap.cli as cli
import scap.utils as utils


@cli.command("update-wikiversions")
class UpdateWikiversions(cli.Application):
    """Scap subcommand for updating staging dir wikiversions.json to a new version."""

    @cli.argument("dblist", help="The dblist file to use as input for migrating.")
    @cli.argument("branch", help="The name of the branch to migrate to.")
    @cli.argument("--no-check", help="Don't check that the branch is already checked out",
                  action='store_false', dest='check')
    def main(self, *extra_args):
        """Update the json file, maybe update the branch symlink."""
        self.update_wikiversions_json()
        self.update_branch_pointer()

    def update_wikiversions_json(self):
        """Change all the requested dblist entries to the new version."""
        json_path = utils.get_realm_specific_filename(
            os.path.join(self.config["stage_dir"], "wikiversions.json"),
            self.config["wmf_realm"],
        )

        db_list_name = os.path.basename(os.path.splitext(self.arguments.dblist)[0])
        dblist = utils.expand_dblist(self.config["stage_dir"], db_list_name)

        new_dir = "php-%s" % self.arguments.branch

        if self.arguments.check and not os.path.isdir(os.path.join(self.config["stage_dir"], new_dir)):
            raise SystemExit(
                "Train branch %s has not been checked out yet.\n"
                "Try running 'scap prep %s' first, or run update-wikiversions with --no-check."
                % (self.arguments.branch, self.arguments.branch)
            )

        if os.path.exists(json_path):
            with open(json_path) as json_in:
                version_rows = json.load(json_in)
        else:
            if db_list_name != "all":
                raise RuntimeError(
                    'No %s file and not invoked with "all."' % json_path
                    + "Cowardly refusing to act."
                )
            self.get_logger().info(
                "%s not found -- rebuilding from scratch!" % json_path
            )
            version_rows = {}

        inserted = 0
        migrated = 0

        for dbname in dblist:
            if dbname in version_rows:
                inserted += 1
            else:
                migrated += 1
            version_rows[dbname] = new_dir

        tmp = json_path + ".tmp"

        try:
            with open(tmp, "w") as json_out:
                json.dump(
                    version_rows,
                    json_out,
                    ensure_ascii=False,
                    indent=4,
                    separators=(",", ": "),
                    sort_keys=True,
                )
                json_out.write("\n")
            os.rename(tmp, json_path)
        except BaseException:
            if os.path.exists(tmp):
                os.remove(tmp)
            raise

        self.get_logger().info(
            "Updated %s: %s inserted, %s migrated." % (json_path, inserted, migrated)
        )

    def update_branch_pointer(self):
        """Swap the php symlink over to the new version as well, if needed."""
        cur_version = self.active_wikiversions("stage")[-1]  # Get the most recent active version

        real_path = os.path.join(self.config["stage_dir"], "php-%s" % cur_version)
        symlink = os.path.join(self.config["stage_dir"], "php")
        if os.path.realpath(symlink) != real_path:
            utils.move_symlink(real_path, symlink)
            self.get_logger().info("Symlink updated")
