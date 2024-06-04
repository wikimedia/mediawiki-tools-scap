# -*- coding: utf-8 -*-

"""Scap plugin for listing, applying, and rolling back backports."""
import hashlib
import os
import platform
import re
import requests.exceptions
import socket
import subprocess
import time
import urllib.parse
from datetime import datetime
from collections import defaultdict

from prettytable import PrettyTable, SINGLE_BORDER
from random import randint
from scap import cli, git, log, ssh, utils, lock, interaction
from scap.plugins.gerrit import GerritSession


def make_table(backports, display_mergable):
    table = PrettyTable()
    table.set_style(SINGLE_BORDER)
    field_names = ["#", "Project", "Branch", "Subject"]

    if display_mergable:
        field_names.append("Mergeable")
        for change in backports:
            table.add_row(
                [
                    change["_number"],
                    change["project"].replace("mediawiki/", ""),
                    change["branch"],
                    change["subject"],
                    change["mergeable"],
                ]
            )
    else:
        for change in backports:
            table.add_row(
                [
                    change["_number"],
                    change["project"].replace("mediawiki/", ""),
                    change["branch"],
                    change["subject"],
                ]
            )

    table.field_names = field_names
    table.max_width["Subject"] = 100
    table.align["#"] = "r"
    table.align["Subject"] = "l"
    return table


class GitRepos:
    """
    Contains base list of deployed git repos and a cache of submodules
    with functions to check whether a project/branch is currently deployable
    Attributes:
        versions (list) : A list of strings, each representing a MediaWiki version that is in deployment . Example: ['1.42.0-wmf.18']
    """

    OPERATIONS_CONFIG = None
    MEDIAWIKI_CORE = None
    config_branch = None
    versions = None
    mediawiki_location = None
    config_repos = {}
    core_repos = {}
    gerrit = None

    def __init__(
        self,
        mediawiki_core,
        operations_config,
        config_branch,
        versions,
        mediawiki_location,
        gerrit,
    ):
        self.MEDIAWIKI_CORE = mediawiki_core
        self.OPERATIONS_CONFIG = operations_config
        self.config_branch = config_branch
        self.versions = versions
        self.mediawiki_location = mediawiki_location
        self.gerrit = gerrit
        self.config_repos = self._get_submodules_paths(self.mediawiki_location)
        self.config_repos[self.OPERATIONS_CONFIG] = self.mediawiki_location

    def _get_submodules_paths(self, location):
        """Returns a dictionary of submodule project keys and path values for the given location"""
        submodule_paths = {}
        paths_urls = git.list_submodules_paths_urls(location, "--recursive")
        for path_url in paths_urls:
            path, url = path_url.split(" ")
            project = self.gerrit.submodule_project_from_url(url)
            if project is not None:
                submodule_paths[project] = path
        return submodule_paths

    def _get_core_repos_for_version(self, version):
        """Returns a dict of the core repos for a version with project keys and path values.
        Gets and adds them to the dictionary if they haven't been recorded yet
        """
        res = self.core_repos.get(version)
        if res:
            return res
        core_path = self.mediawiki_location + "/php-" + version
        self.core_repos[version] = self._get_submodules_paths(core_path)
        self.core_repos[version][self.MEDIAWIKI_CORE] = core_path
        return self.core_repos[version]

    def any_branch_of_mediawiki_project_is_deployable(self, project, branches):
        """mediawiki & extensions projects only.
        Checks if any of the branches a project is in are deployable to production mediawiki
        """
        for branch in branches:
            version = branch.replace("wmf/", "")
            if os.path.isdir(f"{self.mediawiki_location}/php-{version}"):
                if project in self._get_core_repos_for_version(version):
                    return True
        return False

    def any_branch_of_mediawiki_project_is_in_production(self, project, branches):
        """mediawiki & extensions projects only.
        Checks if any of the branches a project is in are deployed to production mediawiki
        """
        included_in_production_branches = set(
            [f"wmf/{v}" for v in self.versions]
        ).intersection(branches)
        for branch in included_in_production_branches:
            version = branch.replace("wmf/", "")
            if project in self._get_core_repos_for_version(version):
                return True
        return False

    def are_any_branches_deployable(self, project, branches):
        """Checks if any of the branches a 'change' is included in (as a commit)
        are deployable to production and whether the change's project exists in any
        of the deployable branches.
        """
        return self.is_in_production_config(
            project, branches
        ) or self.any_branch_of_mediawiki_project_is_deployable(project, branches)

    def are_any_branches_in_production(self, project, branches):
        """Checks if any of the branches a 'change' is included in (as a commit)
        are currently deployed to production and whether the change's project exists in any
        of the deployed branches.
        """
        return self.is_in_production_config(
            project, branches
        ) or self.any_branch_of_mediawiki_project_is_in_production(project, branches)

    def is_in_production_config(self, project, branches):
        return self.config_branch in branches and project in self.config_repos

    def get_repo_location(self, project, branch, use_submodule_directory=False):
        """Gets the location of the repo for the project and version defined by the branch.
        If use_submodule_directory is True, then the submodule directory is returned,
        otherwise, the parent project's location will be returned.

        Returns the repo location
        """
        if project in self.config_repos:
            if use_submodule_directory:
                repo_location = self.config_repos[project]
            else:
                repo_location = self.mediawiki_location
        else:
            core_repos = self._get_core_repos_for_version(branch.replace("wmf/", ""))
            if use_submodule_directory:
                repo_location = core_repos[project]
            else:
                repo_location = core_repos[self.MEDIAWIKI_CORE]
        return repo_location


class InvalidChangeException(SystemExit):
    """Exception for changes which are determined to be invalid for backport"""


class GerritChanges:
    """
    Manages gerrit changes to be backported
    """

    gerrit = None
    changes = None
    change_numbers = None

    def __init__(self, logger, gerrit, change_numbers):
        self.logger = logger
        self.gerrit = gerrit
        self.change_numbers = change_numbers
        self.changes = dict(
            map(
                lambda number: (number, GerritChange(gerrit, number)),
                change_numbers,
            )
        )

    def __len__(self):
        return len(self.change_numbers)


class GerritChange:
    """
    Stores and manages a gerrit change
    """

    gerrit = None
    number = None
    details = None
    depends_ons = None
    depends_on_cycle = None
    needed_by_chain = None

    def __init__(self, gerrit, number, details=None, needed_by_chain=None):
        self.gerrit = gerrit
        self.number = number
        if details is not None:
            self.details = details
        else:
            try:
                self.update_details()
            except requests.exceptions.HTTPError as e:
                if e.response.status_code == 404:
                    raise InvalidChangeException(f"Change '{self.number}' not found")
                raise
        # The changes which depend on this change
        if needed_by_chain is not None:
            self.needed_by_chain = needed_by_chain + [str(number)]
        else:
            self.needed_by_chain = [str(number)]
        # The changes this change depends on
        depends_ons = self.gerrit.depends_ons(self.get("id")).get()
        self.depends_on_cycle = depends_ons.cycle
        self.depends_ons = depends_ons.depends_on_found

        self._validate()

    def get(self, key):
        return self.details.get(key)

    def is_merged(self):
        return self.details.status == "MERGED"

    def update_details(self, get_all_revisions=False):
        revisionid = "all" if get_all_revisions else "current"
        self.details = self.gerrit.change_detail(self.number, revisionid).get()

    def _validate(self):
        def formatted_dep_chain():
            chain = " -> ".join(self.needed_by_chain)
            return f" Change is pulled by the following dependency chain: {chain}"

        if self.get("status") == "ABANDONED":
            raise InvalidChangeException(
                f"Change '{self.number}' has been abandoned!{formatted_dep_chain()}"
            )
        if self.get("work_in_progress"):
            raise InvalidChangeException(
                f"Change '{self.number}' is a work in progress and not ready for merge!{formatted_dep_chain()}"
            )
        if bool(self.depends_on_cycle):
            raise InvalidChangeException(
                f"A dependency cycle was detected for change {self.number}!{formatted_dep_chain()}"
            )


def _get_submodule_urls(repo):
    return git.list_submodules(repo, "--recursive")


@cli.command(
    "backport",
    help="List, apply, or revert backports",
    primary_deploy_server_only=True,
)
class Backport(cli.Application):
    """
    Merge, pull, and sync the specified commits

    Scap backport will +2 the specified Gerrit commits, wait for them to be
    merged, pull them down into the staging directory, sync to test servers,
    prompt for confirmation to proceed, then sync to all servers.
    """

    allowed_attempts = None
    backports = None
    backport_or_revert = None
    config_branch = None
    deploy_user = None
    gerrit = None
    git_repos = None
    interval = None
    mediawiki_location = None
    OPERATIONS_CONFIG = "operations/mediawiki-config"
    MEDIAWIKI_CORE = "mediawiki/core"
    versions = None

    @cli.argument(
        "--list",
        help="list the available backports and prompts for change numbers/URLs to backport",
        action="store_true",
    )
    @cli.argument("--yes", help="Skip all non-warning prompts.", action="store_true")
    @cli.argument(
        "--stop-before-sync",
        help="Stage backports without syncing. Useful for running tests",
        action="store_true",
    )
    @cli.argument("--revert", help="revert a backport", action="store_true")
    @cli.argument(
        "change_numbers", nargs="*", help="Change numbers/URLs to backport or revert"
    )
    def main(self, *extra_args):
        self.deploy_user = utils.get_real_username() + "@" + socket.gethostname()
        self.allowed_attempts = 2
        self.interval = 5
        self.backport_or_revert = "revert" if self.arguments.revert else "backport"
        self.gerrit = GerritSession(url=self.config["gerrit_url"])
        self.config_branch = self.config["operations_mediawiki_config_branch"]
        self.mediawiki_location = self.config["stage_dir"]
        self.versions = self.active_wikiversions("stage")

        if len(self.versions) <= 0:
            self.get_logger().warning("No active wikiversions!")
            raise SystemExit(1)

        change_numbers = [self._change_number(n) for n in self.arguments.change_numbers]
        self.git_repos = GitRepos(
            self.MEDIAWIKI_CORE,
            self.OPERATIONS_CONFIG,
            self.config_branch,
            self.versions,
            self.mediawiki_location,
            self.gerrit,
        )

        self._assert_auth_sock()
        self._check_ssh_auth()

        if self.arguments.list:
            self._list_available_backports()
            change_numbers = input(
                "Enter the change numbers (separated by a space) you wish to %s: "
                % self.backport_or_revert
            )
            change_numbers = [self._change_number(n) for n in change_numbers.split()]

        if not change_numbers:
            self.get_logger().warning("No change number or url supplied!")
            return 1

        self.backports = GerritChanges(self.get_logger(), self.gerrit, change_numbers)

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
        # Note that the default lock file returned by `self.get_lock_file()` cannot be used here or backport would
        # deadlock itself when calling "sync-world" further down the flow.
        # Also, a backport revert calls backport recursively, deferring to the lock here instead of itself locking
        # '/var/lock/scap.backport.lock'. Otherwise, deadlock again
        with lock.Lock(
            "/var/lock/scap.backport.lock", name="backport", reason=self._build_sal()
        ):
            self._validate_backports()
            if not self.arguments.yes:
                table = make_table(
                    list(change.details for change in self.backports.changes.values()),
                    False,
                )
                self._prompt_for_approval_or_exit(
                    "The following changes are scheduled for backport:\n%s\n"
                    "Backport the changes?" % table.get_string()
                )
            self._approve_changes(self.backports.changes.values())
            self._wait_for_changes_to_be_merged()
            num_commits = self._confirm_commits_to_sync()
            if num_commits == 0:
                self.get_logger().info(
                    "Skipping sync since there are no deployable commits."
                )
                return 0

            self.scap_check_call(["prep", "auto"])

            if self._beta_only_config_changes():
                self.get_logger().info(
                    "Skipping sync since all commits were beta/labs-only changes. Operation completed."
                )
                return 0

            if self.arguments.stop_before_sync:
                self.get_logger().info(
                    "Skipping sync since --stop-before-sync was specified"
                )
                return 0

            self._sync_world()

    def _sync_world(self):
        sync_arguments = [self._build_sal()]
        notify_users = set(
            map(
                lambda change: "--notify-user=" + change.get("owner").username,
                self.backports.changes.values(),
            )
        )

        if not self.arguments.yes:
            sync_arguments = list(notify_users) + sync_arguments
            sync_arguments.insert(0, "--pause-after-testserver-sync")

        self.scap_check_call(["sync-world"] + sync_arguments)

    def _extract_bug_ids_from_gerrit_change_details(self, details) -> list:
        """Returns a list of Phabricator task id strings"""
        commit_msg = details["revisions"][details["current_revision"]][
            "commit_with_footers"
        ]
        footers = commit_msg.split("\n\n")[-1]
        return re.findall(r"Bug: (T\d+)\n", footers)

    def _build_sal(self) -> str:
        """Build a Server Admin Log entry"""
        return "Backport for {}".format(
            ", ".join(map(self._build_sal_1, self.backports.changes.values()))
        )

    # This code was inspired by https://gerrit.wikimedia.org/r/plugins/gitiles/labs/tools/deploy-commands/+/refs/heads/master/deploy_commands/bacc.py#10
    def _build_sal_1(self, change) -> str:
        bug_ids = self._extract_bug_ids_from_gerrit_change_details(change.details)

        if not bug_ids:
            bug_str = ""
        else:
            bug_str = " (" + " ".join(bug_ids) + ")"

        return "[[gerrit:{}|{}{}]]".format(
            change.number, change.get("subject"), bug_str
        )

    def _gerrit_ssh(self, gerrit_arguments):
        gerrit_hostname = urllib.parse.urlparse(self.config["gerrit_url"]).hostname
        key_file = self.get_keyholder_key(
            ssh_user=self.config["gerrit_push_user"],
        )
        ssh_command = (
            ssh.SSH_WITH_KEY(
                user=self.config["gerrit_push_user"], key=key_file, port="29418"
            )
            + [gerrit_hostname, "gerrit"]
            + gerrit_arguments
        )

        with utils.suppress_backtrace():
            subprocess.check_call(
                ssh_command,
                env=self.get_gerrit_ssh_env(),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

    def _check_ssh_auth(self):
        try:
            self._gerrit_ssh(["version"])
        except subprocess.CalledProcessError as e:
            self.get_logger().error(
                "SSH to gerrit failed. " "Please check your ssh configuration."
            )
            raise SystemExit(e)

    def _list_available_backports(self):
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

        params["query"] = (
            "status:"
            + status
            + " -is:wip"
            + " AND ("
            + " OR ".join(["branch:wmf/{}".format(v) for v in self.versions])
            + " OR (project:"
            + self.OPERATIONS_CONFIG
            + " AND branch:"
            + self.config_branch
            + "))"
        )

        return self.gerrit.changes().get(params=params)

    def _reset_workspace(self):
        self.get_logger().info("Running scap prep to reset the workspace.")
        self.scap_check_call(["prep", "auto"])

    def _push_and_collect_change_number(self, repo_location, project, branch):
        """Pushes to gerrit and parses the response to return the change number"""
        change_number = None
        with utils.suppress_backtrace():
            push_response = subprocess.check_output(
                [
                    "git",
                    "-C",
                    repo_location,
                    "push",
                    "--porcelain",
                    "origin",
                    "HEAD:refs/for/%s" % branch,
                ],
                text=True,
                stderr=subprocess.STDOUT,
                env=self.get_gerrit_ssh_env(),
            )

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
        encoded_str = (
            "%s\n%s\n%s\n%s\n%s" % (user, datestr, hostname, commit_msg, random_no)
        ).encode("utf-8")

        return "I" + hashlib.sha1(encoded_str).hexdigest()

    def _create_revert_message(self, revert_id, commit, commit_msg):
        reason = None
        default_reason = "Reverted by %s via scap backport" % self.deploy_user

        if not self.arguments.yes:
            reason = input(
                "Please supply a reason for revert (default: %s): " % default_reason
            )

        if reason:
            reason_msg = "\nReason for revert: %s: %s\n" % (self.deploy_user, reason)
        else:
            reason_msg = "\nReason for revert: %s\n" % default_reason

        revert_msg = commit_msg + "\nThis reverts commit %s\n" % commit + reason_msg

        # Adds the change-id trailer line to the git commit message
        # This should make sure not to clobber any other existing trailer lines that are part of the commit message
        with utils.suppress_backtrace():
            revert_msg = subprocess.check_output(
                [
                    "git",
                    "-c",
                    "trailer.ifexists=doNothing",
                    "interpret-trailers",
                    "--trailer",
                    "Change-Id: %s" % revert_id,
                ],
                input=revert_msg,
                text=True,
            )

        return revert_msg

    def _create_reverts(self):
        """Creates a revert on gerrit

        Returns a list of change numbers
        """
        revert_numbers = []
        self.get_logger().info("Reverting %s change(s)" % len(self.backports))

        for change in self.backports.changes.values():
            revision = self.gerrit.change_revision_commit(change.get("id")).get()
            commit = revision["commit"]
            project = change.get("project")
            branch = change.get("branch")

            repo_location = self.git_repos.get_repo_location(project, branch, True)

            # handle security patches by resetting. They will be re-applied by scap prep
            with utils.suppress_backtrace():
                subprocess.check_call(["git", "-C", repo_location, "checkout", branch])
                subprocess.check_call(
                    ["git", "-C", repo_location, "reset", "--hard", "@{u}"]
                )
                subprocess.check_call(
                    ["git", "-C", repo_location, "revert", "--no-edit", commit]
                )
                commit_msg = (
                    subprocess.check_output(
                        [
                            "git",
                            "-C",
                            repo_location,
                            "show",
                            "--pretty=format:%s",
                            "-s",
                            "HEAD",
                        ],
                        text=True,
                    )
                    + "\n"
                )

            revert_id = self._generate_change_id(commit_msg)
            commit_msg = self._create_revert_message(revert_id, commit, commit_msg)

            with utils.suppress_backtrace():
                subprocess.check_call(
                    ["git", "-C", repo_location, "commit", "--amend", "-m", commit_msg]
                )

            revert_number = self._push_and_collect_change_number(
                repo_location, project, branch
            )
            if revert_number is None:
                self.get_logger().warning(
                    "Could not find change number for revert of %s. Push to gerrit may have failed."
                    % change.number
                )
                self._reset_workspace()
                raise SystemExit(1)

            revert_numbers.append(revert_number)
            self.get_logger().info("Change %s created" % revert_number)
            self._gerrit_ssh(
                [
                    "review",
                    "-m",
                    '"%s created a revert of this change as %s"'
                    % (self.deploy_user, revert_id),
                    "%s" % change.get("current_revision"),
                ]
            )

        self._reset_workspace()
        return revert_numbers

    def _approve_changes(self, changes):
        """Approves the given changes by voting Code-Review+2"""

        self.get_logger().info("Voting on %s change(s)" % len(changes))
        for change in changes:
            if change.is_merged():
                self.get_logger().info("Change %s was already merged", change.number)
                continue

            self._gerrit_ssh(
                [
                    "review",
                    "--code-review",
                    "+2",
                    "-m",
                    '"Approved by %s using scap backport"' % self.deploy_user,
                    "%s" % change.get("current_revision"),
                ]
            )
            self.get_logger().info("Change %s approved", change.number)

    def _change_number(self, number_or_url):
        if number_or_url.isnumeric():
            return int(number_or_url)

        # Assume the non-numeric string is a URL and attempt to parse it
        number = self.gerrit.change_number_from_url(number_or_url)

        if number is None:
            self.get_logger().warning(
                "'%s' is not a valid change number or URL" % number_or_url
            )
            raise SystemExit(1)

        return int(number)

    def _confirm_change(self, change_details):
        """
        In case the backport is not in a production branch, get confirmation from operator
        """

        project = change_details["project"]
        branch = change_details["branch"]
        included_in_branches = set(
            self.gerrit.change_in(change_details["id"]).get().branches
        )
        included_in_branches.add(branch)

        if not self.git_repos.are_any_branches_in_production(
            project, included_in_branches
        ):
            self.get_logger().warning(
                "Change '%s', project '%s', branch '%s' not found in any deployed wikiversion. Deployed wikiversions: %s"
                % (change_details["_number"], project, branch, list(self.versions))
            )
            if not self.arguments.yes:
                self._prompt_for_approval_or_exit(
                    "Continue with %s?" % self.backport_or_revert,
                )

    def _validate_backports(self):
        self.get_logger().info(
            "Checking whether requested changes are in a branch deployed to production and their dependencies"
            " valid..."
        )

        for change_number, change in self.backports.changes.items():
            unmet_dependencies = []
            project = change.details["project"]
            branch = change.details["branch"]
            if not self.git_repos.are_any_branches_deployable(project, [branch]):
                raise InvalidChangeException(
                    f"Change '{change.number}, branch {branch} is not deployable to production"
                )

            self._confirm_change(change.details)
            self._validate_dependencies(change, unmet_dependencies)
            if len(unmet_dependencies) > 0:
                raise InvalidChangeException(
                    f"Change '{change.number}' has dependencies '{unmet_dependencies}', which are not merged or"
                    " scheduled for backport"
                )

            self.get_logger().info(
                "Change '%s' validated for %s"
                % (change_number, self.backport_or_revert)
            )

    def _validate_dependencies(self, change, unmet_dependencies):
        self._validate_relations(change, unmet_dependencies)
        self._validate_depends_ons(change, unmet_dependencies)

    def _validate_relations(self, change, unmet_dependencies):
        relations = self.gerrit.submitted_together(change.number).get().changes
        if len(relations) > 1:
            # remove self from list
            relations.pop(0)
            for rel in relations:
                self.get_logger().info(
                    f"Related change {rel['_number']} found for {change.number}"
                )
                self._validate_chain_change(
                    rel, change.needed_by_chain, unmet_dependencies
                )

    def _validate_depends_ons(self, change, unmet_dependencies):
        def is_relevant_dep(dep_changeinfo):
            # Case where the dependency project is the configuration repo or one of its submodules and the branch its
            # production branch (e.g. "master"). Other branches in those repos are not relevant during backport
            if dep_changeinfo["project"] in self.git_repos.config_repos:
                return dep_changeinfo["branch"] == self.config_branch
            # Case where the dependant is the configuration repo and the dependency a MW repo. In this situation there
            # is not enough information to determine which MW dep(s) is/are intended when there are several of them. We
            # verify in case of a non-prod branch and continue
            if self.git_repos.is_in_production_config(
                change.get("project"), [change.get("branch")]
            ):
                self._confirm_change(dep_changeinfo)
                return True
            # Case where branches match. In this case we know this is the intended dependency
            return change.get("branch") == dep_changeinfo["branch"]

        def get_link(dep_changeinfo):
            return f"\t* {self.gerrit.url}/c/{dep_changeinfo['project']}/+/{dep_changeinfo['_number']}".replace(
                "//c", "/c"
            )

        relevant_deps = [
            dep_changeinfo
            for dep_changeinfo in change.depends_ons
            if is_relevant_dep(dep_changeinfo)
        ]
        if (
            len(change.depends_ons) > 0
            and len(relevant_deps) == 0
            and not self.arguments.yes
        ):
            found_deps_links = "\n".join(
                [get_link(dep_changeinfo) for dep_changeinfo in change.depends_ons]
            )
            self.get_logger().warning(
                f"Change {change.number} specified 'Depends-On' but found dependencies are neither configuration"
                f" changes nor do they belong to the same branch. Found dependencies are:\n{found_deps_links}"
            )
            self._prompt_for_approval_or_exit(
                f"Ignore dependencies and continue with {self.backport_or_revert}?",
            )

        for dep_changeinfo in relevant_deps:
            self.get_logger().info(
                f"Dependency {dep_changeinfo['_number']} found for {change.number}"
            )
            self._validate_chain_change(
                dep_changeinfo, change.needed_by_chain, unmet_dependencies
            )

    def _validate_chain_change(
        self, dep_changeinfo, needed_by_chain, unmet_dependencies
    ):
        gerrit_change = GerritChange(
            self.gerrit, dep_changeinfo["_number"], dep_changeinfo, needed_by_chain
        )

        # The change must be either merged or already scheduled by the user
        if (
            not gerrit_change.is_merged()
            and gerrit_change.number not in self.backports.change_numbers
        ):
            unmet_dependencies.append(dep_changeinfo["_number"])
        self._validate_depends_ons(gerrit_change, unmet_dependencies)

    def _validate_reverts(self):
        self.get_logger().info(
            "Checking whether changes are in a branch and version deployed to production..."
        )
        for change_number, change in self.backports.changes.items():
            if not change.is_merged():
                raise InvalidChangeException(
                    "Change '%s' has not yet been merged and cannot be reverted."
                    % change_number
                )
            self._confirm_change(change.details)

    def _wait_for_changes_to_be_merged(self):
        self.get_logger().info(
            "Waiting for changes to be merged. "
            "This may take some time if there are long running tests."
        )

        finished = False
        reporter = log.reporter("awaiting-backport-merges")
        reporter.expect(len(self.backports))
        reporter.start()
        changes = set(self.backports.change_numbers)
        attempts = {change_number: 0 for change_number in changes}
        changes_merged = set()

        try:
            while not finished:
                finished = True  # optimism
                for number in changes.difference(changes_merged):
                    change = self.backports.changes[number]
                    old_revision_number = change.get("current_revision_number")
                    change.update_details(True)
                    new_revision_number = change.get("current_revision_number")
                    status = change.get("status")
                    verified = change.get("labels")["Verified"]
                    code_review = change.get("labels")["Code-Review"]
                    rejected = getattr(verified, "rejected", None)
                    vetoed = getattr(code_review, "rejected", None)
                    # The "mergeable" field will only exist if Gerrit's config has
                    # change.mergeabilityComputationBehavior set to API_REF_UPDATED_AND_CHANGE_REINDEX.
                    mergeable = change.get("mergeable")

                    if status == "MERGED":
                        changes_merged.add(number)
                        reporter.add_success()
                    else:
                        # Specifically checking for false, since mergeable could be None
                        if mergeable is False:
                            attempts[number] += 1

                            if attempts[number] >= self.allowed_attempts:
                                raise SystemExit(
                                    "Gerrit could not merge the change '%s' as is and could require a "
                                    "rebase" % number
                                )

                            self.get_logger().info(
                                "Change %s is not currently mergeable, but may be being rebased. "
                                "Attempt %s of %s"
                                % (number, attempts[number], self.allowed_attempts)
                            )

                        if vetoed:
                            raise SystemExit(
                                "The change '%s' has been rejected (Code-Review -2) by '%s'"
                                % (number, vetoed["name"])
                            )

                        if rejected:
                            all_verified = getattr(verified, "all", [])
                            jenkins_rejected = [
                                v
                                for v in all_verified
                                if v.username == "jenkins-bot" and v.value == -1
                            ]
                            if len(jenkins_rejected) > 0:
                                raise SystemExit(
                                    "The change '%s' failed build tests and could not be merged"
                                    % number
                                )

                        if old_revision_number != new_revision_number:
                            self.prompt_for_approval_or_exit(
                                "Change %s has been updated from patchset %s to patchset %s. Re-approve change and "
                                "continue with %s(s)? "
                                % (
                                    change.number,
                                    old_revision_number,
                                    new_revision_number,
                                    self.backport_or_revert,
                                ),
                            )
                            self._approve_changes([change])

                        finished = False

                if not finished:
                    reporter.refresh()
                    time.sleep(self.interval)

        finally:
            reporter.finish()

        self.get_logger().info("All changes have been merged")

    def _fetch_git_changes(self, location):
        with utils.suppress_backtrace():
            subprocess.check_call(["git", "-C", location, "fetch"])

    def _grep_for_git_commit(self, directory, branch, search_string):
        with utils.suppress_backtrace():
            return subprocess.check_output(
                [
                    "git",
                    "-C",
                    directory,
                    "rev-list",
                    branch,
                    "--regexp-ignore-case",
                    "--grep",
                    search_string,
                ],
                text=True,
            ).strip("\n")

    def _collect_commit_fingerprints(self):
        """
        Prepares a data structure to be used by the caller to check for extra
        (those beyond the ones specified by the user) commits being pulled.

        The data structure is a dictionary with a key for
        /srv/mediawiki-staging, a key for each live
        /srv/mediawiki-staging/php-<version> directory, and a key for any
        staged-but-not-live /srv/mediawiki-staging/php-<version> directory
        referenced by a backport.

        The value for each key is a set of commit hashes corresponding to a
        change being backported (or its corresponding submodule update commit)
        and its merge commit (if any).

        """
        repo_commits = defaultdict(set)
        repo_commits[self.mediawiki_location] = set()

        self.get_logger().info("Fetching new changes...")
        self._fetch_git_changes(self.mediawiki_location)

        for version in self.versions:
            repo_commits["%s/php-%s" % (self.mediawiki_location, version)] = set()
            self._fetch_git_changes("%s/php-%s" % (self.mediawiki_location, version))

        for change in self.backports.changes.values():
            change_id = change.get("change_id")
            project = change.get("project")
            branch = change.get("branch")

            repo_location = self.git_repos.get_repo_location(project, branch)
            # In case the version is not an active version, the git changes wouldn't have been
            # fetched for the repo_location. This can be checked by verifying if the repo_location
            # is present in repo_commits.
            if repo_location not in repo_commits:
                self._fetch_git_changes(repo_location)
            self.get_logger().info("Collecting commit for %s..." % change_id)
            # The submodule update commit will have the same change-id as the original commit to
            # the submodule repo, so it can be searched for in the core repo using the change-id.
            # Depends-on commits can also include the change-id, so make sure to prefix with 'Change-Id:'.
            commit = self._grep_for_git_commit(
                repo_location, "origin/%s" % branch, "Change-Id: %s" % change_id
            )

            if commit is None:
                self.get_logger().error(
                    "Could not find commit for change %s" % change_id
                )
                raise SystemExit(1)

            repo_commits[repo_location].add(commit)

            self.get_logger().info(
                "Collecting merge commit for %s if it exists..." % change_id
            )
            with utils.suppress_backtrace():
                # The merge commit is the latest descendant in the chain between the original commit and upstream.
                # It appears last in the list.
                ancestors = subprocess.check_output(
                    [
                        "git",
                        "-C",
                        repo_location,
                        "rev-list",
                        "%s..@{u}" % commit,
                        "--ancestry-path",
                        "--merges",
                    ],
                    text=True,
                ).splitlines()
            if ancestors:
                merge_commit = ancestors[-1]
                repo_commits[repo_location].add(merge_commit)
                self.get_logger().info("Found merge commit %s" % merge_commit)
            else:
                self.get_logger().info("No merge commit found.")

        return repo_commits

    def _confirm_commits_to_sync(self):
        self.get_logger().info("Collecting commits to deploy...")
        repo_commits = self._collect_commit_fingerprints()

        for repo, commits in repo_commits.items():
            with utils.suppress_backtrace():
                # use the --left-only git option to list commits that are in upstream but not present on
                # the local repo.
                # This means local-only commits will be ignored.
                new_commits = set(
                    filter(
                        None,
                        subprocess.check_output(
                            [
                                "git",
                                "-C",
                                repo,
                                "rev-list",
                                "--left-only",
                                "@{upstream}...HEAD",
                            ],
                            text=True,
                        ).splitlines(),
                    )
                )

            extra_commits = new_commits.difference(commits)

            if extra_commits:
                self.get_logger().warning(
                    "The following are unexpected commits pulled from origin for %s:"
                    % repo
                )
                with utils.suppress_backtrace():
                    subprocess.check_call(
                        ["git", "-C", repo, "show", "-s"] + list(extra_commits)
                    )

                if self.arguments.yes:
                    check_diff = True
                else:
                    check_diff = interaction.prompt_user_for_confirmation(
                        "Would you like to see the diff?", default="y"
                    )
                if check_diff:
                    with utils.suppress_backtrace():
                        subprocess.check_call(
                            ["git", "--no-pager", "-C", repo, "show"]
                            + list(extra_commits)
                        )

                self.get_logger().warning(
                    "There were unexpected commits pulled from origin for %s." % repo
                )
                self._prompt_for_approval_or_exit(
                    "Continue with deployment (all patches will be deployed)?",
                )
        return len(repo_commits)

    def _get_file_list(self, change_number):
        """
        Returns the list of files modified by the change associated with change number.
        """
        return [
            filename
            for filename in self.gerrit.change_files(change_number).get().keys()
            if filename != "/COMMIT_MSG"
        ]

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

            (num_beta_files, num_other_files) = self._count_beta_only_config_files(
                change.number
            )

            if num_other_files > 0 or num_beta_files == 0:
                return False

        return True

    def _prompt_for_approval_or_exit(self, prompt):
        self.prompt_for_approval_or_exit(
            prompt,
            "%s cancelled" % self.backport_or_revert.capitalize(),
        )
