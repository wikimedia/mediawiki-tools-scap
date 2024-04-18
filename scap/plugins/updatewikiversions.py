# -*- coding: utf-8 -*-
"""Updates wiki versions json files and symlink pointers."""

import collections
import json
import os

import scap.cli as cli
import scap.utils as utils


@cli.command("update-wikiversions", affected_by_blocked_deployments=True)
class UpdateWikiversions(cli.Application):
    """Scap subcommand for updating staging dir wikiversions.json to a new version."""

    @cli.argument(
        "--no-check",
        help="Don't check that the branch is already checked out",
        action="store_false",
        dest="check",
    )
    @cli.argument(
        "pairs",
        nargs="*",
        help="A sequence of one or more DBLIST and VERSION pairs."
        " DBLIST names a group of wikis to migrate and VERSION names a train"
        " branch to set the group to.  If more than one DBLIST/VERSION pair"
        " is supplied, they will be updated in the order specified.",
        metavar="DBLIST VERSION",
    )
    def main(self, *extra_args):
        """Update the json file, maybe update the branch symlink."""

        if len(self.arguments.pairs) == 0:
            utils.abort("At least one DBLIST VERSION pair must be supplied")

        if len(self.arguments.pairs) % 2 != 0:
            utils.abort(f"""Missing branch after '{" ".join(self.arguments.pairs)}'""")

        self.updates = collections.OrderedDict()
        while self.arguments.pairs:
            dblist = self._clean_dblist_name(self.arguments.pairs.pop(0))
            branch = self.arguments.pairs.pop(0)
            self.updates[dblist] = branch

        self.update_wikiversions_json()
        if self.config["manage_mediawiki_php_symlink"]:
            self.update_branch_pointer()

    def _clean_dblist_name(self, name: str) -> str:
        return os.path.basename(os.path.splitext(name)[0])

    def update_wikiversions_json(self):
        """Change all the requested dblist entries to the new version(s)."""

        dblists = {}

        for dblist, version in self.updates.items():
            if self.arguments.check and not os.path.isdir(
                os.path.join(self.config["stage_dir"], "php-%s" % version)
            ):
                raise SystemExit(
                    "Train branch %s has not been checked out yet.\n"
                    "Try running 'scap prep %s' first, or run update-wikiversions with --no-check."
                    % (version, version)
                )

            dblists[dblist] = utils.expand_dblist(self.config["stage_dir"], dblist)

        # Inputs have been validated at this point. Begin making changes.

        json_path = utils.get_realm_specific_filename(
            os.path.join(self.config["stage_dir"], "wikiversions.json"),
            self.config["wmf_realm"],
        )

        if os.path.exists(json_path):
            with open(json_path) as json_in:
                version_rows = json.load(json_in)
        else:
            if "all" not in self.updates:
                raise SystemExit(
                    'No %s file and not invoked with "all".' % json_path
                    + " Cowardly refusing to act."
                )

            self.get_logger().info(
                "%s not found -- rebuilding from scratch!" % json_path
            )
            version_rows = {}

        # For later stats
        old_version_rows = version_rows.copy()

        # Perform the stats
        for dblist, version in self.updates.items():
            new_dir = "php-%s" % version
            for dbname in dblists[dblist]:
                version_rows[dbname] = new_dir

        # Safely write a fresh wikiversions.json file
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

        # Compute and report stats
        inserted = 0
        migrated = 0

        for dbname, new_version in version_rows.items():
            try:
                if old_version_rows[dbname] != new_version:
                    migrated += 1
            except KeyError:
                inserted += 1

        self.get_logger().info(
            "Updated %s: %s migrated, %s inserted." % (json_path, migrated, inserted)
        )

    def update_branch_pointer(self):
        """Swap the php symlink over to the new version as well, if needed."""
        cur_version = self.active_wikiversions("stage")[
            -1
        ]  # Get the most recent active version

        real_path = os.path.join(self.config["stage_dir"], "php-%s" % cur_version)
        symlink = os.path.join(self.config["stage_dir"], "php")
        if os.path.realpath(symlink) != real_path:
            utils.move_symlink(real_path, symlink)
            self.get_logger().info("Symlink updated")
