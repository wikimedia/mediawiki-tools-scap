# -*- coding: utf-8 -*-

"""Scap plugin for listing, applying, and rolling back backports."""
import hashlib
import platform
import re
import socket
import subprocess
import time
import urllib.parse
from datetime import datetime

from prettytable import PrettyTable
from random import randint
from scap import cli, git, log, ssh, utils
from scap.plugins.gerrit import GerritSession


def make_table(backports, display_mergable):
    table = PrettyTable()
    field_names = ["#", "Project", "Branch", "Subject"]

    if display_mergable:
        field_names.append("Mergeable")
        for change in backports:
            table.add_row([change['_number'], change['project'].replace("mediawiki/", ""), change['branch'],
                           change['subject'], change['mergeable']])
    else:
        for change in backports:
            table.add_row([change['_number'], change['project'].replace("mediawiki/", ""), change['branch'],
                           change['subject']])

    table.field_names = field_names
    table.max_width["Subject"] = 100
    table.align["#"] = "r"
    table.align["Subject"] = "l"
    return table


@cli.command("backport", help="List, apply, or revert backports")
class Backport(cli.Application):
    """
    Merge, pull, and sync the specified commits

    Scap backport will +2 the specified Gerrit commits, wait for them to be
    merged, pull them down into the staging directory, sync to test servers,
    prompt for confirmation to proceed, then sync to all servers.
    """
    OPERATIONS_CONFIG = "operations/mediawiki-config"
    gerrit = None
    config_branch = None
    mediawiki_location = None
    versions = None
    interval = None
    backport_or_revert = None
    deploy_user = None
    base_repos = None
    git_submodules = {}

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
        action="store_true"
    )
    @cli.argument("change_numbers", nargs="*", help="Change numbers/URLs to backport or revert")
    def main(self, *extra_args):
        self.deploy_user = utils.get_real_username() + "@" + socket.gethostname()
        self.interval = 5
        self.backport_or_revert = "revert" if self.arguments.revert else "backport"
        self.gerrit = GerritSession(url=self.config['gerrit_url'])
        self.config_branch = self.config["operations_mediawiki_config_branch"]
        self.mediawiki_location = self.config["stage_dir"]
        self.versions = self.active_wikiversions("stage")
        self.base_repos = git.list_submodules(self.mediawiki_location, "--recursive") + ["core"]
        change_numbers = [self._change_number(n) for n in self.arguments.change_numbers]

        self._assert_auth_sock()
        self._check_ssh_auth()

        if self.arguments.list:
            self._list_backports()
            change_numbers = input("Enter the change numbers (separated by a space) you wish to %s: "
                                   % self.backport_or_revert)
            change_numbers = [self._change_number(n) for n in change_numbers.split()]

        if not change_numbers:
            self.get_logger().warn("No change number or url supplied!")
            return 1

        change_details = list(map(lambda number: self.gerrit.change_detail(number).get(), change_numbers))

        if self.arguments.revert:
            self._do_revert(change_details)
        else:
            self._do_backport(change_numbers, change_details)

        return 0

    def _do_revert(self, change_details):
        self._validate_reverts(change_details)

        arguments = ["backport"]
        if self.arguments.yes:
            arguments.append("--yes")
        if self.arguments.stop_before_sync:
            arguments.append("--stop-before-sync")

        reverts = self._create_reverts(change_details)

        if len(reverts) > 0:
            self.scap_check_call(arguments + reverts)

    def _do_backport(self, change_numbers, change_details):
        self._validate_backports(change_details, change_numbers)
        if not self.arguments.yes:
            table = make_table(change_details, False)
            self.prompt_for_approval_or_exit("The following changes are scheduled for backport:\n%s\n"
                                             "Backport the changes?" % table.get_string(),
                                             "Backport cancelled.")
        self._approve_changes(change_details)
        self._wait_for_changes_to_be_merged(change_numbers)
        self._confirm_commits_to_sync(change_details)

        self.scap_check_call(["prep", "auto"])

        if self._beta_only_config_changes(change_details):
            self.get_logger().info("Skipping sync since all commits were beta/labs-only changes. Operation completed.")
            return 0

        if self.arguments.stop_before_sync:
            return 0

        self._sync_world(change_details)

    def _sync_world(self, change_details):
        sync_arguments = [self._build_sal(change_details)]
        notify_users = set(map(lambda change: "--notify-user=" + change['owner'].username, change_details))

        if not self.arguments.yes:
            sync_arguments = list(notify_users) + sync_arguments
            sync_arguments.insert(0, "--pause-after-testserver-sync")

        self.scap_check_call(["sync-world"] + sync_arguments)

    def _extract_bug_ids_from_gerrit_change_details(self, change) -> list:
        """Returns a list of Phabricator task id strings"""
        commit_msg = change["revisions"][change["current_revision"]]["commit_with_footers"]
        footers = commit_msg.split('\n\n')[-1]
        return re.findall(r'Bug: (T\d+)\n', footers)

    def _build_sal(self, change_details) -> str:
        """Build a Server Admin Log entry"""
        return "Backport for {}".format(", ".join(map(self._build_sal_1, change_details)))

    # This code was inspired by https://gerrit.wikimedia.org/r/plugins/gitiles/labs/tools/deploy-commands/+/refs/heads/master/deploy_commands/bacc.py#10
    def _build_sal_1(self, change) -> str:
        bug_ids = self._extract_bug_ids_from_gerrit_change_details(change)

        if not bug_ids:
            bug_str = ''
        else:
            bug_str = ' (' + ' '.join(bug_ids) + ')'

        return '[[gerrit:{}|{}{}]]'.format(change["_number"], change["subject"], bug_str)

    def _gerrit_ssh(self, gerrit_arguments):
        gerrit_hostname = urllib.parse.urlparse(self.config['gerrit_url']).hostname
        key_file = self.get_keyholder_key(
            ssh_user=self.config["gerrit_push_user"],
        )
        ssh_command = ssh.SSH_WITH_KEY(
            user=self.config["gerrit_push_user"],
            key=key_file,
            port='29418'
        ) + [gerrit_hostname, 'gerrit'] + gerrit_arguments

        with utils.suppress_backtrace():
            subprocess.check_call(ssh_command , env=self.get_gerrit_ssh_env(),
                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _check_ssh_auth(self):
        try:
            self._gerrit_ssh(['version'])
        except subprocess.CalledProcessError as e:
            self.get_logger().error("SSH to gerrit failed. "
                                    "Please check your ssh configuration.")
            raise SystemExit(e)

    def _list_backports(self):
        if len(self.versions) <= 0:
            self.get_logger().warn("No active wikiversions!")
            raise SystemExit(1)

        backports = self._get_backports()

        if len(backports) <= 0:
            self.get_logger().info("No available %s." % self.backport_or_revert)
            raise SystemExit()

        backports_table = make_table(backports, not self.arguments.revert)
        print(backports_table.get_string(sortby="Project"))

    def _get_backports(self):
        params = {}

        if self.arguments.revert:
            status = "merged"
            params["n"] = 10
        else:
            status = "open"

        params["query"] = ("status:" + status + " -is:wip" + " AND ("
                           + " OR ".join(["branch:wmf/{}".format(v) for v in self.versions])
                           + " OR (project:" + self.OPERATIONS_CONFIG + " AND branch:"
                           + self.config_branch + "))")

        return self.gerrit.changes().get(params=params)

    def _reset_workspace(self):
        self.get_logger().info("Running scap prep to reset the workspace.")
        self.scap_check_call(['prep', 'auto'])

    def _push_and_collect_change_number(self, repo_location, project, branch):
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

    def _generate_change_id(self, commit_msg):
        random_no = randint(10000, 99999)
        user = self.deploy_user
        datestr = datetime.now().strftime("%a %d %b %Y %I:%M:%S %p %Z")
        hostname = platform.node()
        encoded_str = ("%s\n%s\n%s\n%s\n%s" % (user, datestr, hostname, commit_msg, random_no)).encode("utf-8")

        return "I" + hashlib.sha1(encoded_str).hexdigest()

    def _create_revert_message(self, revert_id, commit, commit_msg):
        reason = None
        default_reason = "Reverted by %s via scap backport" % self.deploy_user

        if not self.arguments.yes:
            reason = input("Please supply a reason for revert (default: %s): " % default_reason)

        if reason:
            reason_msg = "\nReason for revert: %s: %s\n" % (self.deploy_user, reason)
        else:
            reason_msg = "\nReason for revert: %s\n" % default_reason

        revert_msg = commit_msg + "\nThis reverts commit %s\n" % commit + reason_msg

        # Adds the change-id trailer line to the git commit message
        # This should make sure not to clobber any other existing trailer lines that are part of the commit message
        with utils.suppress_backtrace():
            revert_msg = subprocess.check_output(
                ["git", "-c", "trailer.ifexists=doNothing", "interpret-trailers",
                 "--trailer", "Change-Id: %s" % revert_id],
                input=revert_msg, text=True)

        return revert_msg

    def _create_reverts(self, change_details):
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

            if project == self.OPERATIONS_CONFIG:
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
                                                      "HEAD"], text=True) + "\n"

            revert_id = self._generate_change_id(commit_msg)
            commit_msg = self._create_revert_message(revert_id, commit, commit_msg)

            with utils.suppress_backtrace():
                subprocess.check_call(["git", "-C", repo_location, "commit", "--amend", "-m", commit_msg])

            revert_number = self._push_and_collect_change_number(repo_location, project, branch)
            if revert_number is None:
                self.get_logger().warn("Could not find change number for revert of %s. Push to gerrit may have failed."
                                       % detail['_number'])
                self._reset_workspace()
                raise SystemExit(1)

            revert_numbers.append(revert_number)
            self.get_logger().info('Change %s created' % revert_number)
            self._gerrit_ssh(['review', '-m', '"%s created a revert of this change as %s"'
                             % (self.deploy_user, revert_id), '%s' % detail['current_revision']])

        self._reset_workspace()
        return revert_numbers

    def _approve_changes(self, change_details):
        """Approves the given changes by voting Code-Review+2"""

        self.get_logger().info('Voting on %s change(s)' % len(change_details))
        for detail in change_details:
            change_number = detail['_number']
            if detail['status'] == 'MERGED':
                self.get_logger().info('Change %s was already merged', change_number)
                continue

            self._gerrit_ssh(['review', '--code-review', '+2', '-m',
                              '"Approved by %s using scap backport"' % self.deploy_user,
                              '%s' % detail['current_revision']])
            self.get_logger().info('Change %s approved', change_number)

    def _change_number(self, number_or_url):
        if number_or_url.isnumeric():
            return int(number_or_url)

        # Assume the non-numeric string is a URL and attempt to parse it
        number = self.gerrit.change_number_from_url(number_or_url)

        if number is None:
            self.get_logger().warn("'%s' is not a valid change number or URL" % number_or_url)
            raise SystemExit(1)

        return int(number)

    def _is_project_suitable(self, change_number, project, branch):
        if branch not in self.git_submodules:
            self.git_submodules[branch] = git.list_submodules(self.mediawiki_location + "/php-" +
                                                              branch.replace("wmf/", ""), "--recursive")
        if project not in self.base_repos + self.git_submodules[branch]:
            self.get_logger().warn("Change '%s', project '%s', branch '%s' not valid for any production "
                                   "project/submodule" % (change_number, project, branch))
            return False
        return True

    def _are_branches_suitable(self, change_number, project, branches):
        if project == self.OPERATIONS_CONFIG and self.config_branch in branches:
            return True
        elif project is not self.OPERATIONS_CONFIG:
            included_in_production_branches = set("wmf/{}".format(v) for v in self.versions).intersection(branches)
            for branch in included_in_production_branches:
                if self._is_project_suitable(change_number, project, branch):
                    return True

        self.get_logger().info(
            "Change '%s', project '%s', branches '%s' not found in any deployed wikiversion. Deployed wikiversions: %s"
            % (change_number, project, branches, list(self.versions)))
        return False

    def _is_status_suitable(self, change_detail):
        change_number = change_detail['_number']
        if change_detail.status == "ABANDONED":
            self.get_logger().warn("Change '%s' has been abandoned!" % change_number)
            return False
        if change_detail.work_in_progress:
            self.get_logger().warn("Change '%s' is a work in progress and not ready for merge!" % change_number)
            return False
        return True

    def _validate_change(self, change_detail):
        change_number = change_detail['_number']
        project = change_detail.project.replace("mediawiki/", "")
        branch = change_detail.branch

        if not self._is_status_suitable(change_detail):
            raise SystemExit(1)

        if not self._are_branches_suitable(change_number, project, [branch]):
            if not self.arguments.yes:
                self.prompt_for_approval_or_exit("Continue with %s?" % self.backport_or_revert.capitalize(),
                                                 "%s Cancelled" % self.backport_or_revert)

    def _validate_backports(self, change_details, change_numbers):
        self.get_logger().info("Checking whether changes are in a branch and version deployed to production...")
        for detail in change_details:
            self._validate_change(detail)
            self._validate_dependencies(detail, change_numbers)
            self.get_logger().info("Change '%s' validated for %s" % (detail['_number'], self.backport_or_revert))

    def _validate_reverts(self, change_details):
        self.get_logger().info("Checking whether changes are in a branch and version deployed to production...")
        for detail in change_details:
            if detail['status'] != "MERGED":
                raise SystemExit("Change '%s' has not yet been merged and cannot be reverted." % detail['_number'])
            self._validate_change(detail)

    def _validate_dependencies(self, change_detail, change_numbers):
        """Checks if all dependencies are merged or scheduled to be merged."""
        self.get_logger().info("Checking for relation chains and Depends-Ons...")
        change_number = change_detail['_number']
        project_branch_id = change_detail['id']

        deps = dict(map(lambda change: (change['_number'], change),
                        self.gerrit.submitted_together(change_number).get().changes))
        deps.update(self._get_depends_ons(project_branch_id, change_number))

        unscheduled_dependencies = set(deps.keys()) - set(change_numbers)
        unmet_dependencies = []
        unsuitable_dependencies = []

        for dep_number in unscheduled_dependencies:
            dep_project = deps[dep_number]['project'].replace("mediawiki/", "")
            included_info = self.gerrit.change_in(dep_number).get()
            branches = included_info.branches
            if len(branches) == 0:
                unmet_dependencies.append(dep_number)
            elif not self._are_branches_suitable(dep_number, dep_project, branches):
                unsuitable_dependencies.append(dep_number)

        if len(unmet_dependencies) > 0:
            raise SystemExit("Change '%s' cannot be merged without merging its dependencies '%s', which are not "
                             "merged or scheduled for backport" % (change_number, unmet_dependencies))

        if len(unsuitable_dependencies) > 0:
            self.get_logger().warn("The change '%s' has dependencies '%s' which are not scheduled for backport "
                                   "or included in any mediawiki production branch." %
                                   (change_number, unsuitable_dependencies))
            self.prompt_for_approval_or_exit("Continue with %s?" % self.backport_or_revert.capitalize(),
                                             "%s Cancelled" % self.backport_or_revert)

    def _get_depends_ons(self, project_branch_changeid, change_number):
        # There can be multiple changes with the same change_id, so make sure to use 'id',
        # https://gerrit-review.googlesource.com/Documentation/rest-api-changes.html#change-info
        # which combines project, branch, and change_id to request dependency information
        # for the correct change.
        depends_ons = self.gerrit.depends_ons(project_branch_changeid).get()
        project_branch = project_branch_changeid.rsplit('~', 1)[0]
        deps = {}

        if bool(depends_ons.cycle) is True:
            raise SystemExit(
                "The change '%s' cannot be merged because a dependency cycle was detected." % change_number)

        depends_ons_changes = self._filter_depends_ons(change_number, project_branch, depends_ons.depends_on_found)
        for change_info in depends_ons_changes:
            dep_change_number = change_info['_number']
            dep_project_branch_changeid = change_info['id']
            if change_number not in deps:
                self.get_logger().info("Dependency %s found. Checking for dependencies of %s..." %
                                       (dep_change_number, dep_change_number))
                deps[dep_change_number] = change_info
                deps.update(self._get_depends_ons(dep_project_branch_changeid, dep_change_number))

        return deps

    def _filter_depends_ons(self, change_number, project_branch, depends_ons):
        # filter out duplicate change ids
        # the correct dependency should share project and branch with the original change
        depends_ons_by_changeid = {}
        filtered_depends_ons = []
        for change in depends_ons:
            depends_ons_by_changeid.setdefault(change['change_id'], []).append(change)
        for change_id, changes in depends_ons_by_changeid.items():
            if len(changes) > 1:
                same_project_branch_deps = list((change for change in changes if project_branch in change['id']))
                if len(same_project_branch_deps) != 1:
                    raise SystemExit("Could not determine dependency for %s from those sharing change ids: %s."
                                     % (change_number, list((change['_number'] for change in changes))))
                filtered_depends_ons.extend(same_project_branch_deps)
            else:
                filtered_depends_ons.extend(changes)
        return filtered_depends_ons

    def _wait_for_changes_to_be_merged(self, change_numbers):
        self.get_logger().info('Waiting for changes to be merged. '
                               'This may take some time if there are long running tests.')

        finished = False
        reporter = log.reporter("awaiting-backport-merges")
        reporter.expect(len(change_numbers))
        reporter.start()
        changes = set(change_numbers)
        changes_merged = set()

        try:
            while not finished:
                finished = True  # optimism
                for number in changes.difference(changes_merged):
                    detail = self.gerrit.change_detail(number).get()
                    status = detail['status']
                    verified = detail['labels']['Verified']
                    rejected = getattr(verified, 'rejected', None)
                    # The "mergeable" field will only exist if Gerrit's config has
                    # change.mergeabilityComputationBehavior set to API_REF_UPDATED_AND_CHANGE_REINDEX.
                    mergeable = getattr(detail, 'mergeable', None)

                    if status == 'MERGED':
                        changes_merged.add(number)
                        reporter.add_success()
                    else:
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
                    reporter.refresh()
                    time.sleep(self.interval)

        finally:
            reporter.finish()

        self.get_logger().info('All changes have been merged')

    def _fetch_git_changes(self, location):
        with utils.suppress_backtrace():
            subprocess.check_call(["git", "-C", location, "fetch"])

    def _grep_for_git_commit(self, directory, branch, search_string):
        with utils.suppress_backtrace():
            return subprocess.check_output(["git", "-C", directory, "rev-list", branch, "--regexp-ignore-case",
                                            "--grep", search_string], text=True).strip("\n")

    def _collect_commit_fingerprints(self, change_details):
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
        self._fetch_git_changes(self.mediawiki_location)

        for version in self.versions:
            repo_commits["%s/php-%s" % (self.mediawiki_location, version)] = set()
            self._fetch_git_changes("%s/php-%s" % (self.mediawiki_location, version))

        for detail in change_details:
            change_id = detail["change_id"]
            project = detail.project
            branch = detail.branch

            if project == self.OPERATIONS_CONFIG:
                repo_location = self.mediawiki_location
            else:
                repo_location = "%s/php-%s" % (self.mediawiki_location, branch.replace("wmf/", ""))

            self.get_logger().info('Collecting commit for %s...' % change_id)
            # The submodule update commit will have the same change-id as the original commit to
            # the submodule repo, so it can be searched for in the core repo using the change-id.
            # Depends-on commits can also include the change-id, so make sure to prefix with 'Change-Id:'.
            commit = self._grep_for_git_commit(repo_location, "origin/%s" % branch, "Change-Id: %s" % change_id)

            if commit is None:
                self.get_logger().error("Could not find commit for change %s" % change_id)
                self._reset_workspace()
                raise SystemExit(1)

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

    def _confirm_commits_to_sync(self, change_details):
        self.get_logger().info('Collecting commits to deploy...')
        repo_commits = self._collect_commit_fingerprints(change_details)

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
                    check_diff = True
                else:
                    check_diff = utils.prompt_user_for_confirmation("Would you like to see the diff?")
                if check_diff:
                    with utils.suppress_backtrace():
                        subprocess.check_call(["git", "--no-pager", "-C", repo, "show"] + list(extra_commits))

                self.prompt_for_approval_or_exit('There were unexpected commits pulled from origin for %s. '
                                                 'Continue with backport?' % repo, "Backport cancelled.")

            self.get_logger().info('Printing git status for %s for your reference...' % repo)
            with utils.suppress_backtrace():
                subprocess.check_call(["git", "-C", repo, "status"])

    def _get_file_list(self, details):
        """
        Returns the list of files modified by the change associated with 'details'.
        """
        return [filename for filename in self.gerrit.change_files(details["_number"]).get().keys()
                if filename != "/COMMIT_MSG"]

    def _count_beta_only_config_files(self, details):
        beta_only_config_files = self.config["beta_only_config_files"].split()
        num_beta_files = 0
        num_other_files = 0

        for file in self._get_file_list(details):
            if file in beta_only_config_files:
                num_beta_files += 1
            else:
                num_other_files += 1

        return (num_beta_files, num_other_files)

    def _beta_only_config_changes(self, change_details) -> bool:
        """
        Returns True if the changes being backported consist exclusively of beta/labs-only
        configuration changes.
        """
        for details in change_details:
            if details["project"] != self.OPERATIONS_CONFIG:
                return False

            (num_beta_files, num_other_files) = self._count_beta_only_config_files(details)

            if num_other_files > 0 or num_beta_files == 0:
                return False

        return True
