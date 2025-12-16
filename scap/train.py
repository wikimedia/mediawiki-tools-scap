# -*- coding: utf-8 -*-
import prettytable
import re

from scap import cli, utils

GROUPS = ["testwikis", "group0", "group1", "group2"]


def fetch_remote_train_info(config) -> "TrainInfo":
    """
    Return a TrainInfo based on information from remote systems (Gerrit,
    trian-blockers) and the local on-disk wikiversions from the staging
    directory.

    The TrainInfo may contain warnings that should be shown to the user in
    interactive contexts.
    """
    version = utils.get_current_train_version_from_gerrit(config["gerrit_url"])

    info = TrainInfo(config, version)

    try:
        tb = utils.get_current_train_info(
            config["train_blockers_url"],
            config["web_proxy"],
        )
        info.task = tb["task"]
        info.task_url = config["phorge_url"] + "/" + tb["task"]
        info.task_status = tb["status"]
        info.task_version = tb["version"]
        info.task_release_date = tb["date"]

    except Exception as e:
        info.warnings.append(f"Failed to retrieve train blocker info: {e}")

    info.validate()

    return info


class TrainInfo:
    def __init__(self, config, train_version):
        self.config = config
        self.warnings = []
        self.train_version = train_version
        # self.groups maps a group name to the list of versions that the group is running.
        self.groups = {}
        self.train_is_at = None
        self.task = None
        self.task_status = None
        self.task_url = None
        self.task_version = None
        self.task_release_date = None
        self.old_version = None
        # self.group_wikis maps a group name to the list of wikidbs in that group.
        self.group_wikis = {}
        # self.wiki_versions maps a wikidb to its version.
        self.wiki_versions = {}

        self.update()

    def update(self):
        """
        Updates this TrainInfo instance with the latest on-disk information
        (versions for each group, wikiversions, old version). Note that
        old_version will be None if there are no old on-disk wikiversions.
        """

        # FIXME: Verify that versions are ascending as we advance through groups.
        # Warn if there is an unusual arrangement.
        for group in GROUPS:
            versions = utils.get_group_versions_for_train(
                group, self.config["stage_dir"], self.config["wmf_realm"]
            )
            self.groups[group] = versions

            if versions == [self.train_version]:
                self.train_is_at = group

        self.group_wikis = {
            group: utils.get_group_wikidbs(
                group, self.config["stage_dir"], self.config["wmf_realm"]
            )
            for group in self.groups
        }
        self.wiki_versions = utils.read_wikiversions(
            self.config["stage_dir"],
            self.config["wmf_realm"],
            trim_version=True,
        )

        versions = utils.get_wikiversions_ondisk(
            self.config["stage_dir"], trainBranchesOnly=True
        )
        if self.train_version in versions:
            versions.remove(self.train_version)

        self.old_version = versions[-1] if versions else None

        return self

    def validate(self):
        if self.task_status not in ["open", "progress"]:
            self.warnings.append(
                f"Phabricator train task {self.task} has already been closed "
                f"with status {self.task_status}."
            )

    def visualize(self, show_positions=False):
        # Copied from https://www.asciiart.eu/vehicles/trains
        train_image = """
____
|DD|_____T_
|_ |XXXXXX|<
  @-@-@-oo\
"""
        train_image = re.sub("XXXXXX", self.train_version[-6:], train_image)

        train_is_at = self.train_is_at

        stops = ["START"] + GROUPS

        # Dividers are not printed if border=False
        table = prettytable.PrettyTable(border=True, header=False)

        table.add_row(
            [
                (
                    train_image
                    if (
                        stop == train_is_at or (stop == "START" and train_is_at is None)
                    )
                    else ""
                )
                for stop in stops
            ],
            divider=True,
        )
        table.add_row(stops)
        table.add_row(
            ["/".join(self.groups[stop]) if stop != "START" else "" for stop in stops]
        )
        if show_positions:
            table.add_row([f"[{n}]" for n in range(0, len(stops))])

        # Must be set after adding the rows, otherwise it has no effect.
        table.align = "l"
        table.vrules = prettytable.NONE
        table.hrules = prettytable.NONE
        # ===== looks like a train track!
        table.horizontal_char = "="

        print(table)


@cli.command(
    "train",
    help="Advance or rollback the train",
    primary_deploy_server_only=True,
    require_tty_multiplexer=True,
)
class Train(cli.Application):
    """
    Advance or rollback the train
    """

    @cli.argument("--forward", action="store_true", help="Advance the train")
    @cli.argument(
        "--backward", "--rollback", action="store_true", help="Rollback the train"
    )
    @cli.argument(
        "--pause-after-testserver-sync",
        action="store_true",
        help="Pause after syncing testservers and prompt the user to confirm to continue syncing",
    )
    def main(self, *extra_args):
        if self.arguments.forward and self.arguments.backward:
            raise SystemExit("Please choose one of --forward or --backward")

        info = fetch_remote_train_info(self.config)
        io = self.get_io()
        if info.warnings and io is not None:
            if not io.prompt_user_for_confirmation(
                "WARNING:\n\n" + "\n".join(info.warnings) + "\n\nContinue?",
            ):
                utils.abort("Canceled due to warnings.")

        train_version = info.train_version
        self.old_version = info.old_version
        train_is_at = info.train_is_at
        info.visualize()
        print()

        stops = ["START"] + GROUPS
        current_pos = 0 if train_is_at is None else stops.index(train_is_at)
        next_pos = current_pos + 1
        previous_pos = current_pos - 1

        if self.arguments.forward:
            target_pos = next_pos

            if target_pos >= len(stops):
                print(f"The train has already reached {stops[-1]}")
                return
        elif self.arguments.backward:
            target_pos = previous_pos

            if target_pos <= 0:
                print("The train is already rolled back all the way.")
                return

            if not self.old_version:
                raise SystemExit(
                    "No other train branches are checked out.  Nothing to roll back to"
                )
        else:
            choices = {stop: str(index) for index, stop in enumerate(stops)}
            choices["Cancel"] = "c"
            default = str(next_pos) if next_pos < len(stops) else "c"
            target_pos = self.prompt_choices(
                f"What station do you want the {train_version} train to be at?",
                choices,
                default,
            )

            if target_pos == "c":
                print("Cancelled")
                return

            try:
                target_pos = int(target_pos)
                if target_pos < 0 or target_pos >= len(stops):
                    raise ValueError()
            except ValueError:
                raise SystemExit(f"'{target_pos}' is not a valid choice.")

        # At this point target_pos points to an entry in 'stops' indicating
        # the user's choice.

        if target_pos == 0:  # START
            if not self.old_version:
                raise SystemExit(
                    "No other train branches are checked out.  Nothing to roll back to"
                )
            self._deploy_promote("all", self.old_version)
        else:
            self._deploy_promote(stops[target_pos], train_version)

        # Re-read train info and show final visualization
        info.update().visualize()

    def _deploy_promote(self, group, version):
        args = []

        if group != "all" and self.old_version:
            args = ["--train", "--old-version", self.old_version]

        if self.arguments.pause_after_testserver_sync:
            args.insert(0, "--pause-after-testserver-sync")

        self.scap_check_call(["deploy-promote"] + args + [group, version])
