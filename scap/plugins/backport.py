# -*- coding: utf-8 -*-

"""Scap plugin for listing, applying, and rolling back backports."""
from prettytable import PrettyTable

import time
from scap import cli, git
from scap.plugins.gerrit import GerritSession


def make_table(backports):
    table = PrettyTable()
    table.field_names = ["Change Number", "Project", "Branch", "Subject"]

    for change in backports:
        table.add_row([change['_number'], change['project'], change['branch'], change['subject']])

    return table


@cli.command("backport", help="list eligible backports")
class Backport(cli.Application):
    """Doing things with backports."""

    gerrit = None
    config_branch = None

    @cli.argument(
        "--list",
        help='list the available backports and prompts for change numbers/URLs to backport',
        action="store_true"
    )
    @cli.argument("change_numbers", nargs="*", help="Change numbers/URLs to backport")
    def main(self, *extra_args):
        self.gerrit = GerritSession(url=self.config['gerrit_url'])
        self.config_branch = self.config["operations_mediawiki_config_branch"]
        change_numbers = [self.change_number(n) for n in self.arguments.change_numbers]
        versions = self.active_wikiversions("stage")

        if self.arguments.list:
            self.list_backports(versions)
            change_numbers = input("Enter the change numbers (separated by a space) you wish to backport: ")
            change_numbers = change_numbers.split()

        if not change_numbers:
            self.get_logger().warn("No change url supplied to backport!")
            return 1

        print("Backport function not yet implemented!")
        self.validate_changes(change_numbers, versions)
        self.confirm_changes(change_numbers)
        self.approve_changes(change_numbers)
        self.wait_for_changes_to_be_merged(change_numbers)
        # TODO: trigger pipeline job? done automatically?
        # TODO: listen for promote on deployment-charts
        # TODO: +2 deployment-charts
        # TODO: unpack image into /srv/mediawiki/staging
        # TODO: run helmfile apply
        # TODO: scap sync
        return 0

    def list_backports(self, versions):
        if len(versions) <= 0:
            self.get_logger().warn("No active wikiversions!")
            raise SystemExit(1)

        backports = self.get_backports(versions)

        if len(backports) <= 0:
            self.get_logger().info("No open backports.")
            raise SystemExit()

        backports_table = make_table(backports)
        print(backports_table.get_string(sortby="Project"))

    def get_backports(self, versions):
        query = ("status:open AND (" + " OR ".join(["branch:wmf/{}".format(v) for v in versions]) +
                 " OR (project:operations/mediawiki-config AND branch:" +
                 self.config_branch + "))")
        return self.gerrit.changes().get(params={"q": query})

    def approve_changes(self, change_numbers):
        """Approves the given changes by voting Code-Review+2"""

        self.get_logger().info('Approving %s change(s)' % len(change_numbers))

        for change in change_numbers:
            self.gerrit.change(change).revision('review').post({
                "message": "Approved via scap backport",
                "labels": {
                    "Code-Review": 2,
                },
            })
            self.get_logger().info('Change %s approved' % change)

    def change_number(self, number_or_url):
        if number_or_url.isnumeric():
            return number_or_url

        # Assume the non-numeric string is a URL and attempt to parse it
        number = self.gerrit.change_number_from_url(number_or_url)

        if number is None:
            self.get_logger().warn("'%s' is not a valid change number or URL" % number_or_url)
            raise SystemExit(1)

        return number

    def validate_changes(self, change_numbers, versions):
        mediawiki_location = self.config['stage_dir']
        config_submodules = ["core"] + git.list_submodules(mediawiki_location, "--recursive")

        for change_number in change_numbers:
            detail = self.gerrit.change_detail(change_number).get()
            project = detail.project.replace("mediawiki/", "")
            branch = detail.branch.replace("wmf/", "")
            status = detail.status

            if status != "NEW":
                self.get_logger().warn("Change '%s' is not open!" % change_number)
                raise SystemExit(1)
            if project == "operations/mediawiki-config" and branch == self.config_branch:
                self.get_logger().info("Change '%s' valid for backport" % change_number)
            elif branch not in versions:
                self.get_logger().warn("Change '%s' branch '%s' not valid for any deployed wikiversion. Deployed wikiversions: %s" % (change_number, branch, list(versions)))
                raise SystemExit(1)
            elif project not in config_submodules + git.list_submodules(mediawiki_location + '/' + 'php-' + branch, "--recursive"):
                self.get_logger().warn("Change '%s' project '%s' not valid for any production project/submodule" % (change_number, project))
                raise SystemExit(1)

    def confirm_changes(self, change_numbers):
        approval = input("Backport the changes %s? (y/N) " % change_numbers)
        if approval.lower() != "y":
            raise SystemExit("Backport cancelled.")

    def wait_for_changes_to_be_merged(self, change_numbers):
        interval = 5

        self.get_logger().info('Waiting for changes to be merged')

        finished = False

        while not finished:
            finished = True  # optimism
            for number in change_numbers:
                info = self.gerrit.change(number).get()
                status = info['status']
                mergeable = getattr(info, 'mergeable', None)  # Want this to be True. We can probably stop polling immediately if this becomes false
                print("Change {} status: {}, mergeable: {}".format(number, status, mergeable))
                if status != 'MERGED':
                    finished = False

            if not finished:
                time.sleep(interval)

        self.get_logger().info('All changes have been merged')
