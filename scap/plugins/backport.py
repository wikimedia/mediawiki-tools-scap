# -*- coding: utf-8 -*-

"""Scap plugin for listing, applying, and rolling back backports."""
import subprocess
import time

from prettytable import PrettyTable
from scap import cli, git, utils
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
            change_numbers = [self.change_number(n) for n in change_numbers.split()]

        if not change_numbers:
            self.get_logger().warn("No change url supplied to backport!")
            return 1

        change_details = list(map(lambda number: self.gerrit.change_detail(number).get(), change_numbers))
        print("Backport function not yet implemented!")
        self.validate_changes(change_details, versions)
        self.check_dependencies(change_details, change_numbers)
        self.confirm_changes(change_numbers)
        self.approve_changes(change_numbers)
        self.wait_for_changes_to_be_merged(change_numbers)
        # TODO T295495 what are the commit hashes of the merged changes
        # TODO option to sync files/order of sync?
        with utils.suppress_backtrace():
            subprocess.check_call([self.get_script_path(), "sync-world",
                                   "Backport for %s" % change_numbers])

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
        query = ("status:open AND ("
                 + " OR ".join(["branch:wmf/{}".format(v) for v in versions])
                 + " OR (project:operations/mediawiki-config AND branch:"
                 + self.config_branch + "))")
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
            return int(number_or_url)

        # Assume the non-numeric string is a URL and attempt to parse it
        number = self.gerrit.change_number_from_url(number_or_url)

        if number is None:
            self.get_logger().warn("'%s' is not a valid change number or URL" % number_or_url)
            raise SystemExit(1)

        return int(number)

    def validate_changes(self, change_details, versions):
        mediawiki_location = self.config['stage_dir']
        config_submodules = ["core"] + git.list_submodules(mediawiki_location, "--recursive")

        self.get_logger().info("Checking whether changes are in a branch and version deployed to production...")
        for detail in change_details:
            change_number = detail['_number']
            project = detail.project.replace("mediawiki/", "")
            branch = detail.branch.replace("wmf/", "")
            status = detail.status

            if status != "NEW":
                self.get_logger().warn("Change '%s' is not open!" % change_number)
                raise SystemExit(1)
            if project == "operations/mediawiki-config" and branch == self.config_branch:
                self.get_logger().info("Change '%s' valid for backport" % change_number)
            elif branch not in versions:
                self.get_logger().warn(
                    "Change '%s' branch '%s' not valid for any deployed wikiversion. Deployed wikiversions: %s" %
                    (change_number, branch, list(versions)))
                raise SystemExit(1)
            elif project not in config_submodules + git.list_submodules(mediawiki_location + '/' + 'php-'
                                                                        + branch, "--recursive"):
                self.get_logger().warn("Change '%s' project '%s' not valid for any production project/submodule" %
                                       (change_number, project))
                raise SystemExit(1)

    def check_dependencies(self, change_details, change_numbers):
        self.get_logger().info("Checking for relation chains and Depends-Ons...")
        for detail in change_details:
            change_number = detail['_number']
            project_branch_id = detail['id']
            deps = self.gerrit.submitted_together(change_number).get().changes
            deps += self.get_depends_ons(project_branch_id, change_number)

            if len(deps) > 0:
                deps_numbers = list(map(lambda dep: dep["_number"], deps))
                unscheduled_dependencies = set(deps_numbers) - set(change_numbers)

                if len(unscheduled_dependencies) > 0:
                    raise SystemExit("The change '%s' cannot be merged because it has dependencies '%s' "
                                     "which are not scheduled for backport." % (change_number, unscheduled_dependencies))

    def get_depends_ons(self, project_branch_id, change_number):
        depends_ons = self.gerrit.depends_ons(project_branch_id).get()
        deps = []

        if bool(depends_ons.cycle) is True:
            raise SystemExit("The change '%s' cannot be merged because a dependency cycle was detected." % change_number)

        for change_id in depends_ons.depends_on:
            if change_id not in deps:
                change_detail = self.gerrit.change_detail(change_id).get()
                deps.append(change_detail)
                deps += self.get_depends_ons(change_detail['id'], change_detail['_number'])

        return deps

    def confirm_changes(self, change_numbers):
        approval = input("Backport the changes %s? (y/N) " % change_numbers)
        if approval.lower() != "y":
            raise SystemExit("Backport cancelled.")

    def wait_for_changes_to_be_merged(self, change_numbers):
        interval = 5

        self.get_logger().info('Waiting for changes to be merged. '
                               'This may take some time if there are long running tests.')

        finished = False

        while not finished:
            finished = True  # optimism
            for number in change_numbers:
                detail = self.gerrit.change_detail(number).get()
                status = detail['status']
                verified = detail['labels']['Verified']
                rejected = getattr(verified, 'rejected', None)
                mergeable = getattr(detail, 'mergeable', None)
                print("Change {} status: {}, mergeable: {}".format(number, status, mergeable))

                if status != 'MERGED':
                    if not mergeable:
                        raise SystemExit("Gerrit could not merge the change '%s' as is and could require a rebase"
                                         % number)

                    if rejected:
                        all_verified = getattr(verified, 'all', [])
                        jenkins_rejected = [v for v in all_verified if v.username == 'jenkins-bot' and v.value == -1]
                        if len(jenkins_rejected) > 0:
                            raise SystemExit("The change '%s' failed build tests and could not be merged" % number)

                    finished = False

            if not finished:
                time.sleep(interval)

        self.get_logger().info('All changes have been merged')
