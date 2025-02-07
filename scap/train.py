# -*- coding: utf-8 -*-
import prettytable
import re

from scap import cli, interaction, utils, tasks

GROUPS = ["testwikis", "group0", "group1", "group2"]


class TrainInfo:
    def __init__(self, config, io, train_version=None):
        self.config = config
        self.io = io
        self.train_version = (
            train_version if train_version else self.get_train_version()
        )
        self.groups = dict()
        self.train_is_at = None

        # FIXME: Verify that versions are ascending as we advance through groups.
        # Warn if there is an unusual arrangement.
        for group in GROUPS:
            versions = utils.get_group_versions(
                group, config["stage_dir"], config["wmf_realm"]
            )
            self.groups[group] = versions

            if versions == [self.train_version]:
                self.train_is_at = group

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
                train_image
                if (stop == train_is_at or (stop == "START" and train_is_at is None))
                else ""
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

    def get_train_version(self) -> str:
        """
        Returns the version of the current train.  Validation is performed
        first to ensure that a good version will be returned.
        """
        gerrit_latest_version = utils.get_current_train_version_from_gerrit(
            self.config["gerrit_url"]
        )

        try:
            train_info = utils.get_current_train_info(
                self.config["train_blockers_url"], self.config["web_proxy"]
            )
        except Exception as e:
            complaint = f"Failed to automatically retrieve train information: {e}"
            if not interaction.interactive():
                utils.abort(complaint)

            self.io.output_line(complaint)
            version = self.io.input_line(
                "Please enter the train version (e.g. 1.23.4-wmf.5): "
            )
            if not version:
                utils.abort("No train version supplied.  Canceling")
            return version

        task = train_info["task"]
        status = train_info["status"]
        version = train_info["version"]

        if status not in ["open", "progress"]:
            check_status = self.io.prompt_user_for_confirmation(
                f"Train task {task} has status '{status}'. Continue anyway?"
            )
            if not check_status:
                utils.abort(f"Train task {task} has status '{status}'.")

        if version != gerrit_latest_version:
            utils.abort(
                "Phabricator task {} says the train version is '{}', but '{}' is the latest available in Gerrit.".format(
                    task, version, gerrit_latest_version
                )
            )

        return version

    def get_prior_version(self):
        """
        Returns the latest on-disk train version that's not the current train version, or None
        if nothing found.
        """
        versions = tasks.get_wikiversions_ondisk(self.config["stage_dir"])
        if self.train_version in versions:
            versions.remove(self.train_version)

        if not versions:
            return None

        return versions[-1]


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
    def main(self, *extra_args):
        if self.arguments.forward and self.arguments.backward:
            raise SystemExit("Please choose one of --forward or --backward")

        info = TrainInfo(self.config, self.get_io())
        train_version = info.train_version
        self.old_version = info.get_prior_version()

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
        TrainInfo(self.config, self.get_io(), train_version).visualize()

    def _deploy_promote(self, group, version):
        args = []
        if group != "all" and self.old_version:
            args = ["--train", "--old-version", self.old_version]

        self.scap_check_call(["deploy-promote"] + args + [group, version])
