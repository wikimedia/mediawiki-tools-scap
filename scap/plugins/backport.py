# -*- coding: utf-8 -*-

"""Scap plugin for listing, applying, and rolling back backports."""
import getpass
import hashlib
import platform
import re
import subprocess
import time
import urllib.parse
from datetime import datetime

from prettytable import PrettyTable
from random import randint
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
    backport_or_revert = None

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
    @cli.argument(
        "--revert",
        help='revert a backport',
        action="store_const",
        const="revert",
        default="backport"
    )
    @cli.argument("change_numbers", nargs="*", help="Change numbers/URLs to backport or revert")
    def main(self, *extra_args):
        self.interval = 5
        # rename for clarity
        self.backport_or_revert = self.arguments.revert
        self.gerrit = GerritSession(url=self.config['gerrit_url'])
        self.config_branch = self.config["operations_mediawiki_config_branch"]
        self.mediawiki_location = self.config["stage_dir"]
        self.versions = self.active_wikiversions("stage")
        change_numbers = [self.change_number(n) for n in self.arguments.change_numbers]

        self._assert_auth_sock()
        self.check_ssh_auth()

        if self.arguments.list:
            self.list_backports()
            change_numbers = input("Enter the change numbers (separated by a space) you wish to %s: "
                                   % self.backport_or_revert)
            change_numbers = [self.change_number(n) for n in change_numbers.split()]

        if not change_numbers:
            self.get_logger().warn("No change number or url supplied!")
            return 1

        change_details = list(map(lambda number: self.gerrit.change_detail(number).get(), change_numbers))

        if self.backport_or_revert == "revert":
            self.do_revert(change_details)
        else:
            self.do_backport(change_numbers, change_details)

        return 0

    def do_revert(self, change_details):
        self.validate_reverts(change_details)

        arguments = ["backport"]
        if self.arguments.yes:
            arguments.append("--yes")
        if self.arguments.stop_before_sync:
            arguments.append("--stop-before-sync")

        reverts = self.create_reverts(change_details)

        if len(reverts) > 0:
            self.scap_check_call(arguments + reverts)

    def do_backport(self, change_numbers, change_details):
        self.validate_backports(change_details)
        self.check_dependencies(change_details, change_numbers)
        if not self.arguments.yes:
            utils.prompt_for_approval_or_exit("Backport the changes %s? (y/N): " % change_numbers,
                                              "Backport cancelled.")
        self.approve_changes(change_details)
        self.wait_for_changes_to_be_merged(change_numbers)
        self.confirm_commits_to_sync(change_details)

        self.scap_check_call(["prep", "auto"])

        if self.arguments.stop_before_sync:
            return 0

        self.sync_world(change_details)

    def sync_world(self, change_details):
        sync_arguments = ["Backport for %s" % ","
                          .join(["[[gerrit:%d]] %s"
                                 % (change["_number"], change["subject"]) for change in change_details])]

        if not self.arguments.yes:
            sync_arguments.insert(0, "--pause-after-testserver-sync")

        self.scap_check_call(["sync-world"] + sync_arguments)

    def gerrit_ssh(self, gerrit_arguments):
        gerrit_hostname = urllib.parse.urlparse(self.config['gerrit_url']).hostname

        with utils.suppress_backtrace():
            subprocess.check_call(['ssh', '-p', '29418', gerrit_hostname, 'gerrit'] +
                                  gerrit_arguments, env=self.get_gerrit_ssh_env(),
                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

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
            self.get_logger().info("No available %s." % self.backport_or_revert)
            raise SystemExit()

        backports_table = make_table(backports)
        print(backports_table.get_string(sortby="Project"))

    def get_backports(self):
        params = {}

        if self.backport_or_revert == "revert":
            status = "merged"
            params["n"] = 10
        else:
            status = "open"

        params["query"] = ("status:" + status + " AND ("
                           + " OR ".join(["branch:wmf/{}".format(v) for v in self.versions])
                           + " OR (project:operations/mediawiki-config AND branch:"
                           + self.config_branch + "))")

        return self.gerrit.changes().get(params=params)

    def reset_workspace(self):
        self.get_logger().info("Running scap prep to reset the workspace.")
        self.scap_check_call(['prep', 'auto'])

    def push_and_collect_change_number(self, repo_location, project, branch):
        """Pushes to gerrit and parses the response to return the change number"""
        change_number = None
        with utils.suppress_backtrace():
            push_response = subprocess.check_output(["git", "-C", repo_location, "push", "--porcelain", "origin",
                                                     "HEAD:refs/for/%s" % branch], text=True, stderr=subprocess.STDOUT,
                                                    env=self.get_gerrit_ssh_env())

        # the change number is included in the remote url
        # ex: https://gerrit.wikimedia.org/r/c/project/+/change_no
        pattern = r"%s/\+/(\d+)" % re.escape(project)
        pattern_match = re.search(pattern, push_response)
        if pattern_match:
            change_number = pattern_match.group(1)
        return change_number

    def generate_change_id(self, commit_msg):
        random_no = randint(10000, 99999)
        user = getpass.getuser()
        datestr = datetime.now().strftime("%a %d %b %Y %I:%M:%S %p %Z")
        hostname = platform.node()
        encoded_str = ("%s\n%s\n%s\n%s\n%s" % (user, datestr, hostname, commit_msg, random_no)).encode("utf-8")

        return hashlib.sha1(encoded_str).hexdigest()

    def create_reverts(self, change_details):
        """Creates a revert on gerrit

        Returns a list of change numbers
        """
        revert_numbers = []
        self.get_logger().info('Reverting %s change(s)' % len(change_details))

        for detail in change_details:
            revision = self.gerrit.change_revision_commit(detail['id']).get()
            commit = revision['commit']
            project = detail.project.replace("mediawiki/", "")
            branch = detail.branch

            if project == "operations/mediawiki-config":
                repo_location = self.mediawiki_location
            elif project == "core":
                repo_location = "%s/php-%s" % (self.mediawiki_location, branch.replace("wmf/", ""))
            else:
                repo_location = "%s/php-%s/%s" % (self.mediawiki_location, branch.replace("wmf/", ""), project)

            # handle security patches by resetting. They will be re-applied by scap prep
            with utils.suppress_backtrace():
                subprocess.check_call(["git", "-C", repo_location, "checkout", branch])
                subprocess.check_call(["git", "-C", repo_location, "reset", "--hard", "@{u}"])
                subprocess.check_call(["git", "-C", repo_location, "revert", "--no-edit", commit])
                commit_msg = subprocess.check_output(["git", "-C", repo_location, "show", "--pretty=format:%s", "-s",
                                                      "HEAD"], text=True)

            change_id = self.generate_change_id(commit_msg)
            with utils.suppress_backtrace():
                commit_msg = subprocess.check_output(["git", "-c", "trailer.ifexists=doNothing", "interpret-trailers",
                                                      "--trailer", "Change-Id: I%s" % change_id],
                                                     input=bytes(commit_msg, encoding='utf-8'))
                subprocess.check_call(["git", "-C", repo_location, "commit", "--amend", "-m", commit_msg])

            revert_number = self.push_and_collect_change_number(repo_location, project, branch)
            if revert_number is None:
                self.get_logger().warn("Could not find change number for revert of %s. Push to gerrit may have failed."
                                       % detail['_number'])
                self.reset_workspace()
                raise SystemExit(1)

            revert_numbers.append(revert_number)
            self.get_logger().info('Change %s created' % revert_number)

        self.reset_workspace()
        return revert_numbers

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

    def validate_change(self, change_detail):
        core_submodules = ["core"] + git.list_submodules(self.mediawiki_location, "--recursive")
        change_number = change_detail['_number']
        project = change_detail.project.replace("mediawiki/", "")
        branch = change_detail.branch.replace("wmf/", "")
        status = change_detail.status

        if status == "ABANDONED":
            self.get_logger().warn("Change '%s' has been abandoned!" % change_number)
            raise SystemExit(1)
        if project == "operations/mediawiki-config" and branch == self.config_branch:
            pass
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

        self.get_logger().info("Change '%s' valid for %s" % (change_number, self.backport_or_revert))

    def validate_backports(self, change_details):
        self.get_logger().info("Checking whether changes are in a branch and version deployed to production...")
        for detail in change_details:
            self.validate_change(detail)

    def validate_reverts(self, change_details):
        self.get_logger().info("Checking whether changes are in a branch and version deployed to production...")
        for detail in change_details:
            if detail['status'] != "MERGED":
                raise SystemExit("Change '%s' has not yet been merged and cannot be reverted." % detail['_number'])
            self.validate_change(detail)

    def check_dependencies(self, change_details, change_numbers):
        self.get_logger().info("Checking for relation chains and Depends-Ons...")
        for detail in change_details:
            change_number = detail['_number']
            project_branch_id = detail['id']
            deps_numbers = list(map(lambda change: change['_number'],
                                    self.gerrit.submitted_together(change_number).get().changes))
            deps_numbers += self.get_depends_ons(project_branch_id, change_number)

            if len(deps_numbers) > 0:
                unscheduled_dependencies = set(deps_numbers) - set(change_numbers)

                if len(unscheduled_dependencies) > 0:
                    raise SystemExit("The change '%s' cannot be merged because it has dependencies '%s' "
                                     "which are not scheduled for backport." % (
                                      change_number, unscheduled_dependencies))

    def get_depends_ons(self, project_branch_id, change_number):
        depends_ons = self.gerrit.depends_ons(project_branch_id).get()
        deps = []

        if bool(depends_ons.cycle) is True:
            raise SystemExit(
                "The change '%s' cannot be merged because a dependency cycle was detected." % change_number)

        for change_info in depends_ons.depends_on_found:
            change_id = change_info['change_id']
            if change_id not in deps:
                change_number = change_info['_number']
                deps.append(change_number)
                deps += self.get_depends_ons(change_info['id'], change_number)

        return deps

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
                # The "mergeable" field will only exist if Gerrit's config has
                # change.mergeabilityComputationBehavior set to API_REF_UPDATED_AND_CHANGE_REINDEX.
                mergeable = getattr(detail, 'mergeable', None)
                print("Change {} status: {}, mergeable: {}".format(number, status, mergeable))

                if status != 'MERGED':
                    # Specifically checking for false, since mergeable could be None
                    if mergeable is False:
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
            # the submodule repo, so it can be searched for in the core repo using the change-id.
            # Depends-on commits can also include the change-id, so make sure to prefix with 'Change-Id:'.
            commit = self.grep_for_git_commit(repo_location, "origin/%s" % branch, "Change-Id: %s" % change_id)

            # just to be safe in case submodule update commit has not landed yet
            while not commit:
                time.sleep(self.interval)
                self.fetch_git_changes(repo_location)
                commit = self.grep_for_git_commit(repo_location, "origin/%s" % branch, "Change-Id: %s" % change_id)

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

                if self.arguments.yes:
                    check_diff = 'y'
                else:
                    check_diff = input('Would you like to see the diff? (y/N): ')
                if check_diff.lower() == 'y':
                    with utils.suppress_backtrace():
                        subprocess.check_call(["git", "--no-pager", "-C", repo, "show"] + list(extra_commits))

                utils.prompt_for_approval_or_exit('There were unexpected commits pulled from origin for %s. '
                                                  'Continue with backport? (y/N): ' % repo, "Backport cancelled.")

            self.get_logger().info('Printing git status for %s for your reference...' % repo)
            with utils.suppress_backtrace():
                subprocess.check_call(["git", "-C", repo, "status"])
