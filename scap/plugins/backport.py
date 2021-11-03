# -*- coding: utf-8 -*-

"""Scap plugin for listing, applying, and rolling back backports."""
from prettytable import PrettyTable

from scap import cli
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

    @cli.argument(
        "--list",
        help='list the available backports and prompts for change numbers/URLs to backport',
        action="store_true"
    )
    @cli.argument("change_numbers", nargs="*", help="Change numbers/URLs to backport")
    def main(self, *extra_args):
        self.gerrit = GerritSession(url=self.config['gerrit_url'])
        change_numbers = [self.change_number(n) for n in self.arguments.change_numbers]

        if self.arguments.list:
            versions = self.active_wikiversions("stage").keys()

            if len(versions) > 0:
                backports = self.get_backports(versions)

                if len(backports) > 0:
                    backports_table = make_table(backports)
                    print(backports_table.get_string(sortby="Project"))

                    change_numbers = input("Enter the change numbers (separated by a space) you wish to backport: ")
                    change_numbers = change_numbers.split()
                else:
                    print("No open backports")
                    return 0
            else:
                print("No active wikiversions!")
                return 0

        if change_numbers:
            print("Backport function not yet implemented!")
            # TODO: validate branch & repository
            self.approve_changes(change_numbers)
            # TODO: trigger pipeline job? done automatically?
            # TODO: listen for promote on deployment-charts
            # TODO: +2 deployment-charts
            # TODO: unpack image into /srv/mediawiki/staging
            # TODO: run helmfile apply
            # TODO: scap sync
        else:
            print("No change url supplied to backport")

        return 0

    def get_backports(self, versions):
        query = "status:open(" + "+OR+".join(["branch:wmf/{}".format(v) for v in versions]) + ")"
        return self.gerrit.changes().get(q=query)

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
            raise SystemExit("'%s' is not a valid change number or URL" % number_or_url)

        return number
