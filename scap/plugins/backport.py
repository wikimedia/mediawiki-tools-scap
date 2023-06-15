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


class GitRepos:
    """
    Contains base list of deployed git repos and a cache of submodules
    with functions to check whether a project/branch is currently deployable
    """
    OPERATIONS_CONFIG = None
    config_branch = None
    versions = None
    mediawiki_location = None
    base_repos = []
    submodules = {}
    logger = None

    def __init__(self, logger, operations_config, config_branch, versions, mediawiki_location, base_repos):
        self.OPERATIONS_CONFIG = operations_config
        self.config_branch = config_branch
        self.versions = versions
        self.mediawiki_location = mediawiki_location
        self.base_repos = base_repos
        self.logger = logger

    def get(self, branch):
        """Returns the submodules for a branch.
        Gets and adds them to the dictionary if they haven't been recorded yet
        """
        res = self.submodules.get(branch)
        if res:
            return res
        res = git.list_submodules(self.mediawiki_location + "/php-" + branch, "--recursive")
        self.submodules[branch] = res
        return res

    def non_config_is_in_production(self, project, branches, change_number):
        """Non-config projects only.
           Checks if any of the included in branches of the project for the change are deployed to production.
           The associated change_number is used for logging purposes.
        """
        included_in_production_branches = set("wmf/{}".format(v) for v in self.versions).intersection(branches)
        for branch in included_in_production_branches:
            if project in self.base_repos + self.get(branch.replace("wmf/", "")):
                return True

            self.logger.info("Change '%s', project '%s', branch '%s' not valid for any production "
                             "project/submodule" % (change_number, project, branch))
        return False

    def are_any_branches_deployable(self, change_number, project, branches):
        """Checks if any of the supplied project/branches are deployed to production.
           The associated change_number is used only for logging purposes.
        """
        if project == self.OPERATIONS_CONFIG and self.config_branch in branches:
            return True
        elif project is not self.OPERATIONS_CONFIG:
            if self.non_config_is_in_production(project, branches, change_number):
                return True

        self.logger.warning(
            "Change '%s', project '%s', branches '%s' not found in any deployed wikiversion. Deployed wikiversions: %s"
            % (change_number, project, branches, list(self.versions)))
        return False


class InvalidChangeException(SystemExit):
    """Exception for changes which are determined to be invalid for backport"""


class GerritChanges:
    """
    Manages gerrit changes to be backported
    """
    gerrit = None
    changes = None
    change_numbers = None
    git_repos = None

    def __init__(self, logger, gerrit, git_repos, change_numbers):
        self.logger = logger
        self.git_repos = git_repos
        self.gerrit = gerrit
        self.change_numbers = change_numbers
        self.changes = dict(map(lambda number: (number, GerritChange(logger, gerrit, number)), change_numbers))

    def __len__(self):
        return len(self.change_numbers)

    def is_change_scheduled(self, change_number):
        """Returns whether the change is in the list of changes requested for backport"""
        return change_number in self.change_numbers

    def _select_ideal_dependency(self, change, deps):
        """Attempts to select the appropriate dependency from a list of those sharing the same change Id"""
        merged_count = 0
        for dep in deps:
            if change.get('branch') == dep.get('branch'):
                return dep
            if self.is_change_scheduled(dep.number):
                return dep
            if dep.is_merged:
                merged_count = merged_count + 1
        if merged_count == len(deps):
            return None
        raise InvalidChangeException("Could not determine dependency for %s from those sharing change ids: %s. To "
                                     "backport your change, please include the correct dependency's change number in "
                                     "the scap backport arguments"
                                     % (change.number, list((dep.number for dep in deps))))

    def validate_and_get_ambiguous_dependencies(self, change):
        """
        Ensures all dependencies for a change are met or provided for backport by the user
        Returns a list of dependencies that are not in a production project/branch
        """
        non_prod_dependencies = []
        unmet_dependencies = []
        changes_by_change_id = {}
        for dep in change.dependencies.values():
            changes_by_change_id.setdefault(dep.get('change_id'), []).append(dep)
        for changes in changes_by_change_id.values():
            if len(changes) > 1:
                dep = self._select_ideal_dependency(change, changes)
                if dep is None:
                    continue
            else:
                dep = changes[0]

            if not self.is_change_scheduled(dep.number):
                branches = dep.included_in_branches()
                project = dep.get('project').replace("mediawiki/", "")
                if len(branches) == 0:
                    unmet_dependencies.append(dep.number)
                elif not self.git_repos.are_any_branches_deployable(change.number, project, branches):
                    non_prod_dependencies.append(dep.number)
            non_prod_dependencies.extend(self.validate_and_get_ambiguous_dependencies(dep))

        if len(unmet_dependencies) > 0:
            raise InvalidChangeException("Change '%s' has dependencies '%s', which are not merged or scheduled for "
                                         "backport" % (change.number, unmet_dependencies))
        return non_prod_dependencies


class GerritChange:
    """
    Stores and manages a gerrit change
    """
    gerrit = None
    number = None
    details = None
    dependencies = None
    logger = None

    def __init__(self, logger, gerrit, number, details=None):
        self.logger = logger
        self.gerrit = gerrit
        self.number = number
        self.dependencies = {}
        if details is not None:
            self.details = details
        else:
            self.update_details()
        self.check_status()

    def get(self, key):
        return self.details.get(key)

    def is_merged(self):
        return self.details.status == 'MERGED'

    def check_status(self):
        if self.get('status') == "ABANDONED":
            raise InvalidChangeException("Change '%s' has been abandoned!" % self.number)
        if self.get('work_in_progress'):
            raise InvalidChangeException("Change '%s' is a work in progress and not ready for merge!" % self.number)

    def included_in_branches(self):
        included_info = self.gerrit.change_in(self.number).get()
        return included_info.branches

    def update_details(self, get_all_revisions=False):
        if get_all_revisions:
            self.details = self.gerrit.change_detail(self.number, 'all').get()
        else:
            self.details = self.gerrit.change_detail(self.number).get()

    def update_dependencies(self):
        self.dependencies = {}
        self._update_relations()
        self._update_depends_ons()

    def _record_dependency(self, change):
        if self.dependencies.get(change['_number']) is None:
            gerrit_change = GerritChange(self.logger, self.gerrit, change['_number'], change)
            gerrit_change._update_depends_ons()
            self.dependencies[gerrit_change.number] = gerrit_change

    def _update_relations(self):
        relations = self.gerrit.submitted_together(self.number).get().changes
        if len(relations) > 1:
            # remove self from list
            relations.pop(0)
            for change in relations:
                self.logger.info("Related change %s found for %s", change['_number'], self.number)
                self._record_dependency(change)

    def _update_depends_ons(self):
        depends_ons = self.gerrit.depends_ons(self.get('id')).get()

        if bool(depends_ons.cycle) is True:
            raise InvalidChangeException(
                "A dependency cycle was detected for change %s." % self.number)

        for change in depends_ons.depends_on_found:
            self.logger.info("Dependency %s found for %s", change['_number'], self.number)
            self._record_dependency(change)


@cli.command("backport", help="List, apply, or revert backports", affected_by_blocked_deployments=True)
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
    git_submodules = None
    backports = None

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
        self.git_submodules = GitRepos(self.get_logger(), self.OPERATIONS_CONFIG, self.config_branch, self.versions,
                                       self.mediawiki_location, self.base_repos)

        self._assert_auth_sock()
        self._check_ssh_auth()

        if self.arguments.list:
            self._list_available_backports()
            change_numbers = input("Enter the change numbers (separated by a space) you wish to %s: "
                                   % self.backport_or_revert)
            change_numbers = [self._change_number(n) for n in change_numbers.split()]

        if not change_numbers:
            self.get_logger().warning("No change number or url supplied!")
            return 1

        self.backports = GerritChanges(self.get_logger(), self.gerrit, self.git_submodules, change_numbers)

        if self.arguments.revert:
            self._do_revert()
        else:
            self._do_backport()

        return 0

    def _do_revert(self):
        self._validate_reverts()

        arguments = ["backport"]
        if self.arguments.yes:
            arguments.append("--yes")
        if self.arguments.stop_before_sync:
            arguments.append("--stop-before-sync")

        reverts = self._create_reverts()

        if len(reverts) > 0:
            self.scap_check_call(arguments + reverts)

    def _do_backport(self):
        self._validate_backports()
        if not self.arguments.yes:
            table = make_table(list(change.details for change in self.backports.changes.values()), False)
            self.prompt_for_approval_or_exit("The following changes are scheduled for backport:\n%s\n"
                                             "Backport the changes?" % table.get_string(),
                                             "Backport cancelled.")
        self._approve_changes(self.backports.changes.values())
        self._wait_for_changes_to_be_merged()
        self._confirm_commits_to_sync()

        self.scap_check_call(["prep", "auto"])

        if self._beta_only_config_changes():
            self.get_logger().info("Skipping sync since all commits were beta/labs-only changes. Operation completed.")
            return 0

        if self.arguments.stop_before_sync:
            return 0

        self._sync_world()

    def _sync_world(self):
        sync_arguments = [self._build_sal()]
        notify_users = set(map(lambda change: "--notify-user=" + change.get('owner').username,
                               self.backports.changes.values()))

        if not self.arguments.yes:
            sync_arguments = list(notify_users) + sync_arguments
            sync_arguments.insert(0, "--pause-after-testserver-sync")

        self.scap_check_call(["sync-world"] + sync_arguments)

    def _extract_bug_ids_from_gerrit_change_details(self, details) -> list:
        """Returns a list of Phabricator task id strings"""
        commit_msg = details["revisions"][details["current_revision"]]["commit_with_footers"]
        footers = commit_msg.split('\n\n')[-1]
        return re.findall(r'Bug: (T\d+)\n', footers)

    def _build_sal(self) -> str:
        """Build a Server Admin Log entry"""
        return "Backport for {}".format(", ".join(map(self._build_sal_1, self.backports.changes.values())))

    # This code was inspired by https://gerrit.wikimedia.org/r/plugins/gitiles/labs/tools/deploy-commands/+/refs/heads/master/deploy_commands/bacc.py#10
    def _build_sal_1(self, change) -> str:
        bug_ids = self._extract_bug_ids_from_gerrit_change_details(change.details)

        if not bug_ids:
            bug_str = ''
        else:
            bug_str = ' (' + ' '.join(bug_ids) + ')'

        return '[[gerrit:{}|{}{}]]'.format(change.number, change.get("subject"), bug_str)

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

    def _list_available_backports(self):
        if len(self.versions) <= 0:
            self.get_logger().warning("No active wikiversions!")
            raise SystemExit(1)

        backports = self._get_available_backports()

        if len(backports) <= 0:
            self.get_logger().info("No available %s." % self.backport_or_revert)
            raise SystemExit()

        backports_table = make_table(backports, not self.arguments.revert)
        print(backports_table.get_string(sortby="Project"))

    def _get_available_backports(self):
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

    def _create_reverts(self):
        """Creates a revert on gerrit

        Returns a list of change numbers
        """
        revert_numbers = []
        self.get_logger().info('Reverting %s change(s)' % len(self.backports))

        for change in self.backports.changes.values():
            revision = self.gerrit.change_revision_commit(change.get('id')).get()
            commit = revision['commit']
            project = change.get('project').replace("mediawiki/", "")
            branch = change.get('branch')

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
                self.get_logger().warning(
                    "Could not find change number for revert of %s. Push to gerrit may have failed."
                    % change.number)
                self._reset_workspace()
                raise SystemExit(1)

            revert_numbers.append(revert_number)
            self.get_logger().info('Change %s created' % revert_number)
            self._gerrit_ssh(['review', '-m', '"%s created a revert of this change as %s"'
                             % (self.deploy_user, revert_id), '%s' % change.get('current_revision')])

        self._reset_workspace()
        return revert_numbers

    def _approve_changes(self, changes):
        """Approves the given changes by voting Code-Review+2"""

        self.get_logger().info('Voting on %s change(s)' % len(changes))
        for change in changes:
            if change.is_merged():
                self.get_logger().info('Change %s was already merged', change.number)
                continue

            self._gerrit_ssh(['review', '--code-review', '+2', '-m',
                              '"Approved by %s using scap backport"' % self.deploy_user,
                              '%s' % change.get('current_revision')])
            self.get_logger().info('Change %s approved', change.number)

    def _change_number(self, number_or_url):
        if number_or_url.isnumeric():
            return int(number_or_url)

        # Assume the non-numeric string is a URL and attempt to parse it
        number = self.gerrit.change_number_from_url(number_or_url)

        if number is None:
            self.get_logger().warning("'%s' is not a valid change number or URL" % number_or_url)
            raise SystemExit(1)

        return int(number)

    def _confirm_change(self, change):
        project = change.get('project').replace("mediawiki/", "")
        branch = change.get('branch')

        if not self.git_submodules.are_any_branches_deployable(change.number, project, [branch]):
            if not self.arguments.yes:
                self.prompt_for_approval_or_exit("Continue with %s?" % self.backport_or_revert.capitalize(),
                                                 "%s Cancelled" % self.backport_or_revert)

    def _validate_backports(self):
        self.get_logger().info("Checking whether changes are in a branch and version deployed to production...")
        for change_number, change in self.backports.changes.items():
            self._confirm_change(change)
            change.update_dependencies()
            ambiguous_dependencies = self.backports.validate_and_get_ambiguous_dependencies(change)

            if len(ambiguous_dependencies) > 0:
                self.get_logger().warning("The change '%s' has dependencies '%s' which are not scheduled for backport "
                                          "or included in any mediawiki production branch." %
                                          (change_number, ambiguous_dependencies))
                self.prompt_for_approval_or_exit("Continue with %s?" % self.backport_or_revert.capitalize(),
                                                 "%s Cancelled" % self.backport_or_revert)
            self.get_logger().info("Change '%s' validated for %s" % (change_number, self.backport_or_revert))

    def _validate_reverts(self):
        self.get_logger().info("Checking whether changes are in a branch and version deployed to production...")
        for change_number, change in self.backports.changes.items():
            if not change.is_merged():
                raise InvalidChangeException("Change '%s' has not yet been merged and cannot be reverted."
                                             % change_number)
            self._confirm_change(change)

    def _wait_for_changes_to_be_merged(self):
        self.get_logger().info('Waiting for changes to be merged. '
                               'This may take some time if there are long running tests.')

        finished = False
        reporter = log.reporter("awaiting-backport-merges")
        reporter.expect(len(self.backports))
        reporter.start()
        changes = set(self.backports.change_numbers)
        changes_merged = set()

        try:
            while not finished:
                finished = True  # optimism
                for number in changes.difference(changes_merged):
                    change = self.backports.changes[number]
                    old_revision = change.get('current_revision')
                    change.update_details(True)
                    new_revision = change.get('current_revision')
                    status = change.get('status')
                    verified = change.get('labels')['Verified']
                    rejected = getattr(verified, 'rejected', None)
                    # The "mergeable" field will only exist if Gerrit's config has
                    # change.mergeabilityComputationBehavior set to API_REF_UPDATED_AND_CHANGE_REINDEX.
                    mergeable = change.get('mergeable')

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

                        if old_revision != new_revision:
                            old_number = change.get('revisions')[old_revision]['_number']
                            new_number = change.get('revisions')[new_revision]['_number']
                            self.prompt_for_approval_or_exit(
                                "Change %s has been updated from patchset %s to patchset %s. Re-approve change and "
                                "continue with %s(s)? " % (change.number, old_number, new_number,
                                                           self.backport_or_revert),
                                "%s Cancelled" % self.backport_or_revert)
                            self._approve_changes([change])

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

    def _collect_commit_fingerprints(self):
        """
        Returns commit fingerprints for backported changes for each production branch
        including merge commits and submodule update commits

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

        for change in self.backports.changes.values():
            change_id = change.get("change_id")
            project = change.get('project')
            branch = change.get('branch')

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

    def _confirm_commits_to_sync(self):
        self.get_logger().info('Collecting commits to deploy...')
        repo_commits = self._collect_commit_fingerprints()

        for repo, commits in repo_commits.items():
            with utils.suppress_backtrace():
                # use the --left-only git option to list commits that are in upstream but not present on
                # the local repo.
                # This means local-only commits will be ignored.
                new_commits = set(filter(None, subprocess.check_output(["git", "-C", repo, "rev-list", "--left-only",
                                                                        "@{upstream}...HEAD"], text=True).splitlines()))

            extra_commits = new_commits.difference(commits)

            if extra_commits:
                self.get_logger().warning('The following are unexpected commits pulled from origin for %s:' % repo)
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

    def _get_file_list(self, change_number):
        """
        Returns the list of files modified by the change associated with change number.
        """
        return [filename for filename in self.gerrit.change_files(change_number).get().keys()
                if filename != "/COMMIT_MSG"]

    def _count_beta_only_config_files(self, change_number):
        beta_only_config_files = self.config["beta_only_config_files"].split()
        num_beta_files = 0
        num_other_files = 0

        for file in self._get_file_list(change_number):
            if file in beta_only_config_files:
                num_beta_files += 1
            else:
                num_other_files += 1

        return num_beta_files, num_other_files

    def _beta_only_config_changes(self) -> bool:
        """
        Returns True if the changes being backported consist exclusively of beta/labs-only
        configuration changes.
        """
        for change in self.backports.changes.values():
            if change.get("project") != self.OPERATIONS_CONFIG:
                return False

            (num_beta_files, num_other_files) = self._count_beta_only_config_files(change.number)

            if num_other_files > 0 or num_beta_files == 0:
                return False

        return True
