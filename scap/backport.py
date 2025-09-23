# -*- coding: utf-8 -*-

"""Scap command for listing, applying, and rolling back backports."""
import hashlib
import os
import platform
import re
from typing import Dict, List
import requests.exceptions
import socket
import subprocess
import time
from datetime import datetime
from collections import defaultdict

from prettytable import PrettyTable, SINGLE_BORDER
from random import randint
from scap import cli, git, log, utils
from scap.gerrit import GerritSession


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

    def targets_deployable_mediawiki_code(self, change: "GerritChange") -> bool:
        """
        Returns True if the change's branch/project is deployable to production mediawiki.

        Deployable means:
        1. There is a corresponding php-<version> directory in the staging
           directory, implying that it has been checked out by scap prep.
        2. The project is mediawiki/core itself, or the project exists in the submodules of
           mediawiki/core on that branch.
        """
        project = change.get("project")
        branch = change.get("branch")

        if not branch.startswith("wmf/"):
            return False

        version = branch.replace("wmf/", "")

        try:
            return project in self._get_core_repos_for_version(version)
        except FileNotFoundError:
            return False

    def targets_any_mediawiki_code(self, change: "GerritChange") -> bool:
        """
        Returns True if the change's project is mediawiki/core itself,
        or the project exists in the submodules of mediawiki/core on
        some deployable branch.

        This differs from `targets_deployable_mediawiki_code`.  This method identifies
        any project that contains MediaWiki code, regardless of whether it's currently
        deployable to a specific branch.  In practice, the two checks usually yield the
        same result.  The only time they differ is when a submodule exists in some deployable
        versions but not others.

        1. The MediaWiki core repository (mediawiki/core), or
        2. An extension or skin that exists as a submodule in any of the active
        """
        project = change.get("project")
        return project == self.MEDIAWIKI_CORE or any(
            project in self._get_core_repos_for_version(v)
            for v in utils.get_wikiversions_ondisk(self.mediawiki_location)
        )

    def targets_live_code(self, change: "GerritChange") -> bool:
        """mediawiki & extensions projects only.

        Returns True if the branch of `change` is live (i.e., mentioned in wikiversions.json)
        and `change` is deployable (see targets_deployable_code for the definition of deployable).
        """
        branch = change.get("branch")

        if not branch.startswith("wmf/"):
            return False

        version = branch.replace("wmf/", "")

        return version in self.versions and self.targets_deployable_mediawiki_code(
            change
        )

    def change_is_deployable(self, change: "GerritChange") -> bool:
        """
        Returns True if `change` is a deployable MediaWiki config or code change.

        Deployable means:
        1. The project is a top level mediawiki code or config repo, or
           exists as a submodule of a top level repo on the change's branch.
        2. For code changes, there is a corresponding php-<version> directory
           in the staging directory, implying that it has been checked out by scap prep.

        This is the very first validation that is performed on changes
        supplied to scap backport.
        """
        return self.targets_prod_config(
            change
        ) or self.targets_deployable_mediawiki_code(change)

    def change_targets_production(self, change: "GerritChange") -> bool:
        """
        Returns True if change's branch is currently deployed to production and the change's
        project exists as a submodule of the top level repo on that branch.

        For code changes, this means that the branch/version is mentioned in wikiversions.json
        """
        return self.targets_prod_config(change) or self.targets_live_code(change)

    def targets_prod_config(self, change: "GerritChange") -> bool:
        project = change.get("project")
        branch = change.get("branch")
        return self.config_branch == branch and project in self.config_repos

    def get_repo_location(self, change: "GerritChange", use_submodule_directory=False):
        """Gets the location of the repo for the project and version defined by the change.
        If use_submodule_directory is True, then the submodule directory is returned,
        otherwise, the parent project's location will be returned.

        Returns the repo location
        """
        project = change.get("project")
        branch = change.get("branch")

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

    gerrit: GerritSession = None
    # This is a dict of GerritChange objects, keyed by change number
    changes: Dict[int, "GerritChange"] = None
    change_numbers = None

    def __init__(self, logger, gerrit: GerritSession, change_numbers):
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
    depends_ons: List["GerritChange"] = None
    depends_on_cycle = None
    needed_by_chain = None

    def __init__(
        self, gerrit: GerritSession, number, details=None, needed_by_chain=None
    ):
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
        self.depends_ons = [
            GerritChange(
                self.gerrit, dep_info["_number"], dep_info, self.needed_by_chain
            )
            for dep_info in depends_ons.depends_on_found
        ]

        self._validate()

    def get(self, key):
        return self.details.get(key)

    def __str__(self):
        return f"Change '{self.number}', project: '{self.get('project')}', branch: '{self.get('branch')}'"

    def is_merged(self):
        return self.details.status == "MERGED"

    def get_link(self):
        """Generate a formatted link for this change"""
        return f"\t* {self.gerrit.url}/c/{self.get('project')}/+/{self.number}".replace(
            "//c", "/c"
        )

    def update_details(self, get_all_revisions=False):
        revisionid = "all" if get_all_revisions else "current"
        self.details = self.gerrit.change_detail(self.number, revisionid).get()

    def _format_dependency_chain(self) -> str:
        """Format the dependency chain for error messages."""
        chain = " -> ".join(self.needed_by_chain)
        return f" Change is pulled by the following dependency chain: {chain}"

    def _validate(self):
        if self.get("work_in_progress"):
            raise InvalidChangeException(
                f"Change '{self.number}' is a work in progress and not ready for merge!{self._format_dependency_chain()}"
            )
        if bool(self.depends_on_cycle):
            raise InvalidChangeException(
                f"A dependency cycle was detected for change {self.number}!{self._format_dependency_chain()}"
            )

    def check_abandoned_for_trail(self, trail: "DependencyTrail"):
        """Check if this change is abandoned and add error to trail if so."""
        if self.get("status") == "ABANDONED":
            trail.add_error(
                f"Change '{self.number}' has been abandoned!{self._format_dependency_chain()}"
            )


class DependencyTrail:
    """
    Represents a dependency trail from a root change to all its relevant dependencies.
    Stores information about errors and warnings for T371611.
    """

    def __init__(self, root_change: GerritChange):
        self.root_change = root_change
        self.relevant_dependencies = []  # List of GerritChange objects
        self.dependency_chains = (
            {}
        )  # Maps dep_change_id -> list of changes in chain from root
        self.errors = []  # List of error messages
        self.warnings = []  # List of warning messages

    def add_relevant_dependency(
        self,
        dep_change: GerritChange,
        current_change: GerritChange,
        chain_from_root: List[GerritChange],
    ):
        """
        Add a relevant dependency with the chain from root change to this dependency.
        Avoids adding the same dependency multiple times.
        """
        # Check if this dependency is already in the trail to avoid duplicates
        if not any(
            dep.number == dep_change.number for dep in self.relevant_dependencies
        ):
            self.relevant_dependencies.append(dep_change)

        # Store the dependency chain from root to this dependency
        dep_chain = chain_from_root + [current_change]
        dep_change_id = dep_change.get("id")

        if dep_change_id not in self.dependency_chains:
            self.dependency_chains[dep_change_id] = dep_chain.copy()

    def add_error(self, error_message: str):
        """Add an error message to this trail."""
        self.errors.append(error_message)

    def add_warning(self, warning_message: str):
        """Add a warning message to this trail."""
        self.warnings.append(warning_message)

    def has_errors(self) -> bool:
        """Check if this trail has any errors."""
        return len(self.errors) > 0

    def has_warnings(self) -> bool:
        """Check if this trail has any warnings."""
        return len(self.warnings) > 0

    def get_all_changes(self) -> List[GerritChange]:
        """Get all changes in this trail (root + dependencies)."""
        return [self.root_change] + self.relevant_dependencies


@cli.command(
    "backport",
    help="List, apply, or revert backports",
    primary_deploy_server_only=True,
    require_tty_multiplexer=True,
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

        self.git_repos = GitRepos(
            self.MEDIAWIKI_CORE,
            self.OPERATIONS_CONFIG,
            self.config_branch,
            self.versions,
            self.mediawiki_location,
            self.gerrit,
        )

        os.umask(self.config["umask"])

        self._assert_auth_sock()
        self.gerritssh.assert_authorized()
        if self.arguments.revert:
            self._assert_gerrit_push_config()

        if self.arguments.list:
            self._list_available_backports()
            change_numbers = self.input_line(
                "Enter the change numbers (separated by a space) you wish to %s: "
                % self.backport_or_revert
            )
            change_numbers = [self._change_number(n) for n in change_numbers.split()]
        else:
            change_numbers = [
                self._change_number(n) for n in self.arguments.change_numbers
            ]

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
        # Note that the default lock file returned by `self.get_mediawiki_staging_lock_file()` cannot be used
        # here or backport would deadlock itself when calling "sync-world" further down the flow.
        # Also, a backport revert calls backport recursively, deferring to the lock here instead of itself locking
        # '/var/lock/scap.backport.lock'. Otherwise, deadlock again
        with self.lock("/var/lock/scap.backport.lock", reason=self._build_sal()):
            # Validate root changes for deployability first
            self.validate_root_changes()

            dependency_trails = self._calculate_relevant_dependencies()
            self._validate_dependency_trails(dependency_trails)

            changes_to_backport = list(self.backports.changes.values())

            if not self.arguments.yes:
                table = make_table(
                    list(change.details for change in changes_to_backport),
                    False,
                )
                # Keep message in sync with the code in web/src/components/Interaction.vue#onMounted
                self._prompt_for_approval_or_exit(
                    "The following changes are scheduled for backport:\n%s\n"
                    "Backport the changes?" % table.get_string()
                )
            self._approve_changes(changes_to_backport)
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

    def _assert_gerrit_push_config(self):
        for setting in ["gerrit_push_user", "gerrit_push_url"]:
            if not self.config[setting]:
                raise SystemExit(
                    f"scap backport --revert requires '{setting}' to be set in scap configuration"
                )

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
            reason = self.input_line(
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

            repo_location = self.git_repos.get_repo_location(change, True)

            with utils.suppress_backtrace():
                # Make sure we're working with the same repository state as Gerrit.
                # This will discard any local changes such as security patches.
                # The staging directory will be restored to a deployable state at
                # the end of this function.
                subprocess.check_call(
                    ["git", "-C", repo_location, "fetch", "origin", branch]
                )
                subprocess.check_call(
                    [
                        "git",
                        "-C",
                        repo_location,
                        "checkout",
                        "--force",
                        "-B",
                        branch,
                        f"origin/{branch}",
                    ]
                )

                # Create the revert commit
                with git.with_env_vars_set_for_user():
                    subprocess.check_call(
                        ["git", "-C", repo_location, "revert", "--no-edit", commit]
                    )
                # Collect the generated commit message subject
                commit_msg = (
                    subprocess.check_output(
                        [
                            "git",
                            "-C",
                            repo_location,
                            "show",
                            "--pretty=format:%s",
                            "--no-patch",
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

            revert_number = self.gerritssh.push_and_collect_change_number(
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
            # Add a note to the original change about the revert
            self.gerritssh.review(
                change.get("current_revision"),
                f"{self.deploy_user} created a revert of this change as {revert_id}",
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

            self.gerritssh.review(
                change.get("current_revision"),
                f"Approved by {self.deploy_user} using scap backport",
                "+2",
            )
            self.get_logger().info("Change %s approved", change.number)

    def _change_number(self, number_or_url: str) -> int:
        number = self.gerrit.change_number_from_url(number_or_url)

        if number is None:
            self.get_logger().warning(
                "'%s' is not a valid change number or URL" % number_or_url
            )
            raise SystemExit(1)

        return int(number)

    def validate_root_changes(self):
        """
        Validate root changes for deployability, dependency cycles, production targeting, and abandonment.
        Raises InvalidChangeException if any change fails validation.
        """
        self.get_logger().info(
            "Checking whether requested changes are valid for backport..."
        )

        # Check all validation criteria for root changes
        for change_number, change in self.backports.changes.items():
            # Check deployability
            if not self.git_repos.change_is_deployable(change):
                raise InvalidChangeException(
                    f"{change} is not deployable to production"
                )

            # Check for dependency cycles
            if bool(change.depends_on_cycle):
                raise InvalidChangeException(
                    f"Dependency cycle detected at change {change}"
                )

            # Check if change is abandoned
            if change.get("status") == "ABANDONED":
                raise InvalidChangeException(
                    f"Change '{change.number}' has been abandoned!{change._format_dependency_chain()}"
                )

            # Warn about non-production changes (but don't fail)
            if not self.git_repos.change_targets_production(change):
                warning_msg = f"{change} not found in any live wikiversion. Live wikiversions: {list(self.versions)}"
                if self.arguments.yes:
                    self.get_logger().warning(warning_msg)
                else:
                    self._prompt_for_approval_or_exit(
                        f"{warning_msg}\nContinue with {self.backport_or_revert} anyway?"
                    )

    def _calculate_relevant_dependencies(self) -> List[DependencyTrail]:
        """
        Calculates and returns relevant dependency trails for all root changes.

        Note: Root change validation (cycles, abandonment, production targeting) is
        handled in validate_root_changes() and called before this method.
        """
        dependency_trails = []

        for change_number, change in self.backports.changes.items():
            trail = DependencyTrail(change)

            self._analyze_dependencies_recursive(change, trail, [], set())

            dependency_trails.append(trail)

            self.get_logger().info(
                f"Root change '{change_number}' processed with {len(trail.relevant_dependencies)} relevant dependencies"
            )

        return dependency_trails

    def _analyze_dependencies_recursive(
        self,
        current_change: GerritChange,
        trail: DependencyTrail,
        chain_from_root: List[GerritChange],
        visited: set,
    ):
        if current_change.number in visited:
            return
        visited.add(current_change.number)

        # Process Depends-On relationships
        depends_on_count = len(current_change.depends_ons)
        relevant_depends_on_count = 0

        for dep_change in current_change.depends_ons:
            was_relevant = self._process_dependency(
                current_change,
                dep_change,
                trail,
                chain_from_root,
                "Depends-On",
            )
            if was_relevant:
                relevant_depends_on_count += 1

            # Recursively analyze this dependency's dependencies
            dep_chain = chain_from_root + [current_change]
            self._analyze_dependencies_recursive(dep_change, trail, dep_chain, visited)

        # Warn if change has Depends-On relationships but none are relevant
        if depends_on_count > 0 and relevant_depends_on_count == 0:
            dep_numbers = [str(dep.number) for dep in current_change.depends_ons]
            trail.add_warning(
                f"Change '{current_change.number}' has {depends_on_count} Depends-On "
                f"relationship(s) ({', '.join(dep_numbers)}) but none were deemed relevant "
                f"by the dependency analysis rules. This may be unexpected."
            )

        # Process git relation dependencies
        relations = self.gerrit.submitted_together(current_change.number).get().changes
        if len(relations) > 1:
            # remove self from list
            relations.pop(0)
            for rel in relations:
                rel_change = GerritChange(
                    self.gerrit, rel["_number"], rel, current_change.needed_by_chain
                )

                self._process_dependency(
                    current_change,
                    rel_change,
                    trail,
                    chain_from_root,
                    "Related",
                )
                # Recursively analyze this relation's dependencies
                dep_chain = chain_from_root + [current_change]
                self._analyze_dependencies_recursive(
                    rel_change, trail, dep_chain, visited
                )

    def _process_dependency(
        self,
        current_change: GerritChange,
        dep_change: GerritChange,
        trail: DependencyTrail,
        chain_from_root: List[GerritChange],
        dependency_type: str,
    ):
        """
        Process a single dependency by checking if it is relevant and adding it to the trail.

        Applies the T365146 relevance rules to determine if dep_change should be included
        in the backport. The relevance check considers other changes with the same Change-Id
        (sibling changes) to make decisions about master vs branch targeting.

        If the dependency is relevant, adds it to the trail and performs additional
        validation (abandoned status, merge status).

        Args:
            current_change: The change that has this dependency
            dep_change: The dependency to evaluate
            trail: The dependency trail to update if relevant
            chain_from_root: The chain of changes from root to current_change
            dependency_type: Description of relationship ("Depends-On" or "Related")

        Returns:
            True if the dependency was deemed relevant and added to trail, False otherwise.
        """
        sibling_dependencies = self._get_sibling_dependencies(
            current_change, dep_change
        )
        relevant = self.is_relevant_dep(
            current_change, dep_change, sibling_dependencies
        )

        if relevant:
            trail.add_relevant_dependency(dep_change, current_change, chain_from_root)
            dep_change.check_abandoned_for_trail(trail)
            self._backport_vote(current_change, dep_change, trail)

        msg = "Relevant" if relevant else "NOT Relevant"
        self.get_logger().info(
            f"{current_change} {dependency_type} {dep_change} [{msg}]"
        )

        return relevant

    def _get_included_branches(self, dep_change: GerritChange) -> set:
        """
        Get the set of branches where a change is included using Gerrit's change_in API.

        Args:
            dep_change: The change to check branch inclusion for

        Returns:
            Set of branch names where the change is included
        """
        # Get the branches this change is included in using Gerrit's change_in API
        change_in_response = self.gerrit.change_in(dep_change.get("id")).get()

        # Extract branch names from the response
        included_branches = set()
        if hasattr(change_in_response, "branches"):
            for branch in change_in_response.branches:
                # Gerrit API returns branch names as strings in refs/heads/ format
                branch_name = branch.replace("refs/heads/", "")
                included_branches.add(branch_name)

        return included_branches

    def _validate_master_dependency_in_deployable_branches(
        self,
        dep_change: GerritChange,
        trail: DependencyTrail,
        current_change: GerritChange,
    ):
        """
        Validate that a master branch MediaWiki code dependency is present in all deployable
        train branches.

        For dependencies targeting MediaWiki code projects on the master branch, we need to ensure
        the commit is actually present in ALL currently deployable train branches, not just merged to master.
        This is because a commit could be merged to master but not yet deployed to wmf branches.

        Args:
            dep_change: The dependency change targeting master branch MW code
            trail: The dependency trail to add errors to if validation fails
            current_change: The change that depends on dep_change
        """
        included_branches = self._get_included_branches(dep_change)

        # Get all currently deployable train branch names
        deployable_branches = git.get_deployable_branches(self.mediawiki_location)

        # Check if the dependency is missing from any deployable branch
        missing_branches = deployable_branches - included_branches
        if missing_branches:
            missing_list = ", ".join(sorted(missing_branches))
            project = dep_change.get("project")
            trail.add_error(
                f"Change '{current_change.number}' has dependency '{dep_change.number}' "
                f"targeting the master branch of MediaWiki code project '{project}', but the "
                f"commit is not present in deployable branch(es): {missing_list}. "
                f"Master dependencies must be deployed to all active branches."
            )

    def _validate_master_dependency_in_target_branch(
        self,
        dep_change: GerritChange,
        trail: DependencyTrail,
        current_change: GerritChange,
    ):
        """
        Validate that a master branch MediaWiki code dependency is present in the target branch
        of current_change.

        Args:
            dep_change: The dependency change targeting master branch MW code
            trail: The dependency trail to add errors to if validation fails
            current_change: The change that depends on dep_change
        """
        target_branch = current_change.get("branch")
        included_branches = self._get_included_branches(dep_change)

        # Check if the dependency is missing from the target branch
        if target_branch not in included_branches:
            project = dep_change.get("project")
            trail.add_error(
                f"Change '{current_change.number}' has dependency '{dep_change.number}' "
                f"targeting the master branch of MediaWiki code project '{project}', but the "
                f"commit is not present in target branch '{target_branch}'. "
                f"Master dependencies must be cherry-picked to the target branch."
            )

    def _backport_vote(
        self,
        current_change: GerritChange,
        dep_change: GerritChange,
        trail: DependencyTrail,
    ):
        """
        Validate a relevant dependency according to the rules
        from https://phabricator.wikimedia.org/T362987#vote, which
        have been translated here as follows:

        Given a `Change` targeting branch `Br`, its relevant dependency `Dep`:

        1: If `Change` and `Dep` belong to a MW code repo, then:
        1a: If `Dep` doesn't target master, verify that it has been merged or
        passed as root change in the command line, otherwise add a trail
        error.

        1b: If `Dep` targets master, verify that it is included-in `Br` (which
        implies being merged).  Otherwise add a trail error.

        2: If `Dep` belongs to the MW configuration repo, then verify that
        `Dep` verify has been merged or passed as root change in the command
        line, otherwise add a trail error.

        3: If `Change` belongs to the MW configuration repo and `Dep` belongs
        to a MW code repo, then:
        3a: If `Dep` doesn't target master, verify that it has been merged or
        passed as root change in the command line, otherwise add a trail
        error.  (same as 1a)

        3b: If `Dep` targets master, verify that it is included-in all
        deployable branches (which implies being merged), Otherwise add a
        trail error.

        Args:
            current_change: The change that has this dependency
            dep_change: The relevant dependency to validate
            trail: The dependency trail to add errors to if validation fails
        """
        # Apply T362987 rules for master MW code dependencies
        # 1: If `Change` and `Dep` belong to a MW code repo, then:
        if self.git_repos.targets_any_mediawiki_code(
            current_change
        ) and self.git_repos.targets_any_mediawiki_code(dep_change):
            # 1b: If `Dep` targets master, verify that it is included-in `Br` (which
            # implies being merged). Otherwise add a trail error.
            if dep_change.get("branch") == "master":
                self._validate_master_dependency_in_target_branch(
                    dep_change, trail, current_change
                )
            # 1a: If `Dep` doesn't target master, verify that it has been merged or
            # passed as root change in the command line, otherwise add a trail error.
            else:
                self._validate_dependency_merged_or_scheduled(
                    current_change, dep_change, trail
                )

        # 2: If `Dep` belongs to the MW configuration repo, then verify that `Dep`
        # has been merged or passed as root change in the command line, otherwise add a trail error.
        elif self.git_repos.targets_prod_config(dep_change):
            self._validate_dependency_merged_or_scheduled(
                current_change, dep_change, trail
            )

        # 3: If `Change` belongs to the MW configuration repo and `Dep`
        # belongs to a MW code repo, then:
        elif self.git_repos.targets_prod_config(
            current_change
        ) and self.git_repos.targets_any_mediawiki_code(dep_change):
            # 3b: If `Dep` targets master, verify that it is included-in all deployable branches
            # (which implies being merged), Otherwise add a trail error.
            if dep_change.get("branch") == "master":
                self._validate_master_dependency_in_deployable_branches(
                    dep_change, trail, current_change
                )
            # 3a: If `Dep` doesn't target master, verify that it has been merged or passed
            # as root change in the command line, otherwise add a trail error. (same as 1a)
            else:
                self._validate_dependency_merged_or_scheduled(
                    current_change, dep_change, trail
                )

        # This should never happen
        else:
            raise RuntimeError(
                f"Internal error: Relevant dependency '{dep_change.number}' from change "
                f"'{current_change.number}' was not handled by any validation rule. "
                f"This indicates a bug in the dependency validation logic."
            )

    def _validate_dependency_merged_or_scheduled(
        self,
        current_change: GerritChange,
        dep_change: GerritChange,
        trail: DependencyTrail,
    ):
        """
        Validate that a dependency is either merged or scheduled for backport.
        If not, adds an error to the trail.

        Args:
            current_change: The change that has this dependency
            dep_change: The dependency change to validate
            trail: The dependency trail to add errors to if validation fails
        """
        if (
            not dep_change.is_merged()
            and dep_change.number not in self.backports.change_numbers
        ):
            trail.add_error(
                f"Change '{current_change.number}' has dependency '{dep_change.number}', "
                f"which is not merged or scheduled for backport"
            )

    def _validate_dependency_trails(self, dependency_trails: List[DependencyTrail]):
        """
        Validate dependency trails for errors and warnings.

        Checks for errors and warnings in dependency analysis, reporting them to the user
        and prompting for confirmation if needed. Raises InvalidChangeException if any
        errors are found.
        """
        # Collect all errors from all trails before reporting
        all_errors = []
        for trail in dependency_trails:
            if trail.has_errors():
                for error in trail.errors:
                    all_errors.append(f"Error for {trail.root_change}: {error}")

        # If any errors were found, report them all at once
        if all_errors:
            error_messages = "\n".join(all_errors)
            raise InvalidChangeException(
                f"Errors found in dependency trails:\n{error_messages}"
            )

        # After checking for errors, check for warnings and prompt for confirmation if any exist
        all_warnings = []
        for trail in dependency_trails:
            if trail.has_warnings():
                for warning in trail.warnings:
                    all_warnings.append(f"Warning for {trail.root_change}: {warning}")

        if all_warnings and not self.arguments.yes:
            warning_messages = "\n".join(all_warnings)
            self._prompt_for_approval_or_exit(
                f"Warnings found:\n{warning_messages}\nContinue with {self.backport_or_revert} anyway?"
            )

    def _get_sibling_dependencies(
        self, change: GerritChange, dep_change: GerritChange
    ) -> List[GerritChange]:
        """
        Get sibling dependencies - all changes with the same Change-Id as dep_change.

        Searches through change.depends_ons to find all changes that share the same
        Change-Id as dep_change, which represents sibling changes across different branches.

        Args:
            change: The change whose dependencies to search through
            dep_change: The change to find siblings for

        Returns:
            List of changes with the same Change-Id as dep_change, always including dep_change itself.
        """
        dep_change_id = dep_change.get("change_id")
        siblings = []

        # Search through all dependencies of the current change to find siblings
        for dependency in change.depends_ons:
            if dependency.get("change_id") == dep_change_id:
                siblings.append(dependency)

        # Ensure dep_change itself is included in the sibling list
        if not any(s.number == dep_change.number for s in siblings):
            siblings.append(dep_change)

        return siblings

    # Implement the dependency relevance rules defined in https://phabricator.wikimedia.org/T365146
    def is_relevant_dep(
        self,
        change: GerritChange,
        dep_change: GerritChange,
        sibling_dependencies: List[GerritChange],
    ) -> bool:
        """
        Determine if `dep_change` is a relevant dependency for `change` according to T365146 rules.

        Given a root change or relevant dependency "Change" targeting branch "Br", one of its
        dependencies "Dep" as specified by Depends-On is also relevant in the following cases:
        1. Change and Dep are MW code & Br is deployable --> Dep will be relevant if it targets Br,
           or if it targets master but there is no sibling dependency targeting Br.
        2. If Dep targets production config --> Dep will be relevant
        3. If change targets_prod_config or change is MW code targeting master, then
           Dep will be relevant if it's a deployable change, or if it targets master but there are no siblings
           targeting deployable branches
        """
        change_branch = change.get("branch")
        dep_branch = dep_change.get("branch")

        # Check repo types once
        change_is_prod_config = self.git_repos.targets_prod_config(change)
        change_is_deployable_code = self.git_repos.targets_deployable_mediawiki_code(
            change
        )
        change_is_any_mw_code = self.git_repos.targets_any_mediawiki_code(change)
        dep_is_prod_config = self.git_repos.targets_prod_config(dep_change)
        dep_is_any_mw_code = self.git_repos.targets_any_mediawiki_code(dep_change)

        # If dep is not in a MW code or config repo, it's not relevant
        if not dep_is_any_mw_code and not dep_is_prod_config:
            return False

        # Case 2: If Dep targets production config --> Dep will be relevant
        if dep_is_prod_config:
            return True

        # Case 1: Change and Dep are MW code & change branch is deployable
        if change_is_deployable_code and dep_is_any_mw_code:
            # Dep is relevant if it targets the same branch
            if dep_branch == change_branch:
                return True
            # Or if it targets master and there's no sibling targeting the same branch
            if dep_branch == "master":
                return not self._has_sibling_targeting_branch(
                    dep_change, change_branch, sibling_dependencies
                )
            return False

        # Case 3: If change targets_prod_config or change is MW code targeting master
        if change_is_prod_config or (
            change_is_any_mw_code and change_branch == "master"
        ):
            # Dep is relevant if it's a deployable change
            if self.git_repos.change_is_deployable(dep_change):
                return True
            # Or if it targets master and there are no siblings targeting deployable branches
            if dep_branch == "master":
                return not self._has_sibling_targeting_deployable_branches(
                    dep_change, sibling_dependencies
                )
            return False

        # Fallback: not relevant
        return False

    def _has_sibling_targeting_branch(
        self,
        dep_change: GerritChange,
        target_branch: str,
        sibling_dependencies: List[GerritChange],
    ) -> bool:
        """Check if there's a sibling dependency (same change-id) targeting the specified branch."""
        dep_change_id = dep_change.get("change_id")
        for sibling in sibling_dependencies:
            if (
                sibling.get("change_id") == dep_change_id
                and sibling.number != dep_change.number
                and sibling.get("branch") == target_branch
            ):
                return True
        return False

    def _has_sibling_targeting_deployable_branches(
        self, dep_change: GerritChange, sibling_dependencies: List[GerritChange]
    ) -> bool:
        """Check if there's a sibling dependency targeting any deployable branch."""
        dep_change_id = dep_change.get("change_id")
        for sibling in sibling_dependencies:
            if (
                sibling.get("change_id") == dep_change_id
                and sibling.number != dep_change.number
                and self.git_repos.change_is_deployable(sibling)
            ):
                return True
        return False

    def _validate_reverts(self):
        # First validate that all changes are deployable, not abandoned, no cycles, etc.
        self.validate_root_changes()

        for change_number, change in self.backports.changes.items():
            if not change.is_merged():
                raise InvalidChangeException(
                    "Change '%s' has not yet been merged and cannot be reverted."
                    % change_number
                )

    def _wait_for_changes_to_be_merged(self):
        self.report_status(
            "Waiting for changes to be merged. "
            "This may take some time if there are long running tests.",
            log=True,
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
                            self._prompt_for_approval_or_exit(
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

        self.report_status("All changes have been merged", log=True)

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
            branch = change.get("branch")

            repo_location = self.git_repos.get_repo_location(change)
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
                    check_diff = self.prompt_user_for_confirmation(
                        f"There were unexpected commits pulled from origin for {repo}.\nWould you like to see the diff?",
                        default="y",
                    )
                if check_diff:
                    with utils.suppress_backtrace():
                        subprocess.check_call(
                            [
                                "git",
                                "--no-pager",
                                "-C",
                                repo,
                                "show",
                                "--submodule=diff",
                            ]
                            + list(extra_commits)
                        )

                self._prompt_for_approval_or_exit(
                    f"There were unexpected commits pulled from origin for {repo}.\n"
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
