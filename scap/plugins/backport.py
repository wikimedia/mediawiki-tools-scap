# -*- coding: utf-8 -*-

"""Scap plugin for listing, applying, and rolling back backports."""
import subprocess
import time
import urllib.parse

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
    mediawiki_location = None
    versions = None
    interval = None
    answer_yes = False

    @cli.argument(
        "--list",
        help='list the available backports and prompts for change numbers/URLs to backport',
        action="store_true"
    )
    @cli.argument(
        "--yes",
        help='Skip all non-warning prompts.',
        action="store_true"
    )
    @cli.argument(
        "--stop-before-sync",
        help='Stage backports without syncing. Useful for running tests',
        action="store_true"
    )
    @cli.argument("change_numbers", nargs="*", help="Change numbers/URLs to backport")
    def main(self, *extra_args):
        self.interval = 5
        self.gerrit = GerritSession(url=self.config['gerrit_url'])
        self.config_branch = self.config["operations_mediawiki_config_branch"]
        self.mediawiki_location = self.config["stage_dir"]
        self.versions = self.active_wikiversions("stage")
        self.answer_yes = self.arguments.yes
        change_numbers = [self.change_number(n) for n in self.arguments.change_numbers]

        self._assert_auth_sock()
        self.check_ssh_auth()

        if self.arguments.list:
            self.list_backports()
            change_numbers = input("Enter the change numbers (separated by a space) you wish to backport: ")
            change_numbers = [self.change_number(n) for n in change_numbers.split()]

        if not change_numbers:
            self.get_logger().warn("No change url supplied to backport!")
            return 1

        change_details = list(map(lambda number: self.gerrit.change_detail(number).get(), change_numbers))

        self.validate_changes(change_details)
        self.check_dependencies(change_details, change_numbers)
        if not self.answer_yes:
            self.prompt_for_approval_or_exit("Backport the changes %s? (y/N): " % change_numbers, "Backport cancelled.")
        self.approve_changes(change_details)
        self.wait_for_changes_to_be_merged(change_numbers)
        self.confirm_commits_to_sync(change_details)
        self.scap_check_call(["prep", "auto"])

        if self.arguments.stop_before_sync:
            return 0

        self.scap_check_call(["sync-world", "Backport for %s" % ",".join(
            ["[[gerrit:%d]] %s" % (change["_number"], change["subject"]) for change in change_details])])

        return 0

    def gerrit_ssh(self, gerrit_arguments):
        gerrit_hostname = urllib.parse.urlparse(self.config['gerrit_url']).hostname

        with utils.suppress_backtrace():
            subprocess.check_call(['ssh', '-p', '29418', gerrit_hostname, 'gerrit'] +
                                  gerrit_arguments, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def check_ssh_auth(self):
        try:
            self.gerrit_ssh(['version'])
        except subprocess.CalledProcessError as e:
            self.get_logger().error("SSH to gerrit failed. "
                                    "Please check your ssh configuration.")
            raise SystemExit(e)

    def list_backports(self):
        if len(self.versions) <= 0:
            self.get_logger().warn("No active wikiversions!")
            raise SystemExit(1)

        backports = self.get_backports()

        if len(backports) <= 0:
            self.get_logger().info("No open backports.")
            raise SystemExit()

        backports_table = make_table(backports)
        print(backports_table.get_string(sortby="Project"))

    def get_backports(self):
        query = ("status:open AND ("
                 + " OR ".join(["branch:wmf/{}".format(v) for v in self.versions])
                 + " OR (project:operations/mediawiki-config AND branch:"
                 + self.config_branch + "))")
        return self.gerrit.changes().get(params={"q": query})

    def approve_changes(self, change_details):
        """Approves the given changes by voting Code-Review+2"""

        self.get_logger().info('Approving %s change(s)' % len(change_details))
        for detail in change_details:
            self.gerrit_ssh(['review', '--code-review', '+2', '-m', '"Approved via scap backport"',
                             '%s' % detail['current_revision']])
            self.get_logger().info('Change %s approved' % detail['_number'])

    def change_number(self, number_or_url):
        if number_or_url.isnumeric():
            return int(number_or_url)

        # Assume the non-numeric string is a URL and attempt to parse it
        number = self.gerrit.change_number_from_url(number_or_url)

        if number is None:
            self.get_logger().warn("'%s' is not a valid change number or URL" % number_or_url)
            raise SystemExit(1)

        return int(number)

    def validate_changes(self, change_details):
        core_submodules = ["core"] + git.list_submodules(self.mediawiki_location, "--recursive")

        self.get_logger().info("Checking whether changes are in a branch and version deployed to production...")
        for detail in change_details:
            change_number = detail['_number']
            project = detail.project.replace("mediawiki/", "")
            branch = detail.branch.replace("wmf/", "")
            status = detail.status

            if status == "ABANDONED":
                self.get_logger().warn("Change '%s' has been abandoned!" % change_number)
                raise SystemExit(1)
            if project == "operations/mediawiki-config" and branch == self.config_branch:
                self.get_logger().info("Change '%s' valid for backport" % change_number)
            elif branch not in self.versions:
                self.get_logger().warn(
                    "Change '%s' branch '%s' not valid for any deployed wikiversion. Deployed wikiversions: %s" %
                    (change_number, branch, list(self.versions)))
                raise SystemExit(1)
            elif project not in core_submodules + git.list_submodules(self.mediawiki_location + '/' + 'php-'
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

    def prompt_for_approval_or_exit(self, prompt_message, exit_message):
        approval = input(prompt_message)
        if approval.lower() != "y":
            raise SystemExit(exit_message)

    def wait_for_changes_to_be_merged(self, change_numbers):
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
                time.sleep(self.interval)

        self.get_logger().info('All changes have been merged')

    def fetch_git_changes(self, location):
        with utils.suppress_backtrace():
            subprocess.check_call(["git", "-C", location, "fetch"])

    def grep_for_git_commit(self, directory, branch, search_string):
        with utils.suppress_backtrace():
            return subprocess.check_output(["git", "-C", directory, "rev-list", branch, "--grep", search_string],
                                           text=True).strip("\n")

    def collect_commit_fingerprints(self, change_details):
        """
        Returns commit fingerprints for backported changes for each production branch
        including merge commits and submodule update commits

        :param change_details:  list[object]: The change details for each backport

        :returns: dict[str, set]: Dict with string directory as key and set of string
                                  fingerprints for each active production branch
        """
        repo_commits = {
            self.mediawiki_location: set()
        }

        self.get_logger().info('Fetching new changes...')
        self.fetch_git_changes(self.mediawiki_location)

        for version in self.versions:
            repo_commits["%s/php-%s" % (self.mediawiki_location, version)] = set()
            self.fetch_git_changes("%s/php-%s" % (self.mediawiki_location, version))

        for detail in change_details:
            change_id = detail["change_id"]
            project = detail.project
            branch = detail.branch

            if project == "operations/mediawiki-config":
                repo_location = self.mediawiki_location
            else:
                repo_location = "%s/php-%s" % (self.mediawiki_location, branch.replace("wmf/", ""))

            self.get_logger().info('Collecting commit for %s...' % change_id)
            # The submodule update commit will have the same change-id as the original commit to
            # the submodule repo, so it can be searched for in the core repo using the change-id
            commit = self.grep_for_git_commit(repo_location, "origin/%s" % branch, change_id)

            # just to be safe in case submodule update commit has not landed yet
            while not commit:
                time.sleep(self.interval)
                self.fetch_git_changes(repo_location)
                commit = self.grep_for_git_commit(repo_location, "origin/%s" % branch, change_id)

            repo_commits[repo_location].add(commit)

            self.get_logger().info('Collecting merge commit for %s if it exists...' % change_id)
            with utils.suppress_backtrace():
                # The merge commit is the latest descendant in the chain between the original commit and upstream.
                # It appears last in the list.
                ancestors = subprocess.check_output(["git",
                                                     "-C", repo_location, "rev-list", "%s..@{u}" % commit,
                                                     "--ancestry-path", "--merges"], text=True).splitlines()
            if ancestors:
                merge_commit = ancestors[-1]
                repo_commits[repo_location].add(merge_commit)
                self.get_logger().info('Found merge commit %s' % merge_commit)
            else:
                self.get_logger().info('No merge commit found.')

        return repo_commits

    def confirm_commits_to_sync(self, change_details):
        self.get_logger().info('Collecting commits to deploy...')
        repo_commits = self.collect_commit_fingerprints(change_details)

        for repo, commits in repo_commits.items():
            with utils.suppress_backtrace():
                # use the --left-only git option to list commits that are in upstream but not present on
                # the local repo.
                # This means local-only commits will be ignored.
                new_commits = set(filter(None, subprocess.check_output(["git", "-C", repo, "rev-list", "--left-only",
                                                                        "@{upstream}...HEAD"], text=True).splitlines()))

            extra_commits = new_commits.difference(commits)

            if extra_commits:
                self.get_logger().warn('The following are unexpected commits pulled from origin for %s:' % repo)
                with utils.suppress_backtrace():
                    subprocess.check_call(["git", "-C", repo, "show", "-s"] + list(extra_commits))

                if self.answer_yes:
                    check_diff = 'y'
                else:
                    check_diff = input('Would you like to see the diff? (y/N): ')
                if check_diff.lower() == 'y':
                    with utils.suppress_backtrace():
                        subprocess.check_call(["git", "--no-pager", "-C", repo, "show"] + list(extra_commits))

                self.prompt_for_approval_or_exit('There were unexpected commits pulled from origin for %s. '
                                                 'Continue with backport? (y/N): ' % repo, "Backport cancelled.")

            self.get_logger().info('Printing git status for %s for your reference...' % repo)
            with utils.suppress_backtrace():
                subprocess.check_call(["git", "-C", repo, "status"])

        if not self.answer_yes:
            self.prompt_for_approval_or_exit('All live versions will be synced. Continue? (y/N): ',
                                             "Sync cancelled.")
