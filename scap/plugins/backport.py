# -*- coding: utf-8 -*-

"""Scap plugin for listing, applying, and rolling back backports."""
import json
import requests

from scap import cli
from prettytable import PrettyTable


def get_backports(versions):
    query = "status:open(" + "+OR+".join(["branch:wmf/{}".format(v) for v in versions]) + ")"
    response = requests.get('https://gerrit.wikimedia.org/r/changes/?q=' + query)

    response.raise_for_status()

    return json.loads(response.text[4:] if response.text.startswith(')]}\'') else response.text)


def make_table(backports):
    table = PrettyTable()
    table.field_names = ["Change Number", "Project", "Branch", "Subject"]

    for change in backports:
        table.add_row([change['_number'], change['project'], change['branch'], change['subject']])

    return table


@cli.command("backport", help="list eligible backports")
class Backport(cli.Application):
    """Doing things with backports."""

    @cli.argument(
        "--list",
        help='list the available backports and prompts for change numbers to backport',
        action="store_true"
    )
    @cli.argument("change_numbers", nargs="*", help="Change numbers to backport")
    def main(self, *extra_args):
        change_numbers = self.arguments.change_numbers

        if self.arguments.list:
            versions = self.active_wikiversions("stage").keys()

            if len(versions) > 0:
                backports = get_backports(versions)

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
            # TODO:
            # validate branch & repository
            # +2 changes
            # trigger pipeline job? done automatically?
            # listen for promote on deployment-charts
            # +2 deployment-charts
            # unpack image into /srv/mediawiki/staging
            # run helmfile apply
            # scap sync
        else:
            print("No change url supplied to backport")

        return 0
