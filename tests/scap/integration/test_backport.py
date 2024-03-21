import contextlib
import logging
import os
import re
import subprocess
import sys
import time
import unittest
from datetime import datetime

import pexpect

import scap.cli
import scap.git
import scap.utils
from scap.plugins.gerrit import GerritSession
from scap.runcmd import gitcmd

""" These tests are expected to be run in the train-dev test environment:
    python3 -m pytest -s -o log_cli=1 --log-cli-level info /workspace/scap/tests/scap/integration/test_backport.py
"""


def announce(message):
    logging.info("\n** %s **\n" % message)


class BackportsTestHelper:
    homedir = "/home/debian"
    workdir = homedir + "/testdir"
    mwconfig_dir = workdir + "/mediawiki-config"
    mwdir = workdir + "/mediawiki"
    mwcore_dir = mwdir + "/core"
    mwgrowthexperiments_dir = mwdir + "/extensions/GrowthExperiments"
    mwvisualeditor_dir = mwdir + "/extensions/VisualEditor"
    mwstaging_dir = "/srv/mediawiki-staging"
    mwphp_dir = None
    mwbranch = None
    oldmwversion = None
    gerrit = None
    gerrit_url = "http://gerrit.traindev:8080"
    gerrit_domain = "gerrit.traindev"

    def setup(self):
        self.gerrit = GerritSession(url=self.gerrit_url, use_auth=True)

        if not os.path.exists(self.homedir + "/new-version"):
            logging.info("Bootstrapping with ~/train first")
            subprocess.check_call(self.homedir + "/train")

        announce("Running scap prep auto to clean up /srv/mediawiki-staging")
        subprocess.check_call(["scap", "prep", "auto"])
        scap.utils.mkdir_p(self.workdir)

        with open(self.homedir + "/new-version", "r") as f:
            version = f.read().strip()
            self.mwbranch = "wmf/" + version
            self.mwphp_dir = "php-" + version

        with open(self.homedir + "/old-version", "r") as f:
            self.oldmwversion = f.read().rstrip()

        self.setup_mediawiki_config_repo()
        self.setup_core_repo()
        self.setup_visualeditor_repo()

        self.setup_core_gitmodules()

    def setup_mediawiki_config_repo(self, branch="train-dev"):
        self.git_clone(
            branch, self.gerrit_url + "/operations/mediawiki-config", self.mwconfig_dir
        )

    def setup_core_repo(self, branch=None):
        self.git_clone(
            branch if branch else self.mwbranch,
            self.gerrit_url + "/mediawiki/core",
            self.mwcore_dir,
        )

    def setup_growthexperiments_repo(self):
        self.git_clone(
            self.mwbranch,
            self.gerrit_url + "/mediawiki/extensions/GrowthExperiments",
            self.mwgrowthexperiments_dir,
        )

    def setup_visualeditor_repo(self):
        self.git_clone(
            self.mwbranch,
            self.gerrit_url + "/mediawiki/extensions/VisualEditor",
            self.mwvisualeditor_dir,
        )
        self.git_command(self.mwvisualeditor_dir, ["submodule", "update", "--init"])
        self.install_commit_hook(self.mwvisualeditor_dir + "/.git/modules/lib/ve")

    def setup_gitmodule(self, dir, submodule_config, submodule_dir):
        self.git_command(
            dir,
            [
                "config",
                "-f",
                ".gitmodules",
                submodule_config,
                self.gerrit_url + submodule_dir,
            ],
        )
        if subprocess.check_output(
            ["git", "-C", dir, "status", "--porcelain", ".gitmodules"]
        ):
            announce("Tweaking %s in %s/.gitmodules" % (submodule_config, dir))
            self.git_commit(
                dir,
                "scap backport testing: set URL for %s to %s"
                % (submodule_dir, self.gerrit_domain),
                ".gitmodules",
            )
            change_url = self.push_and_collect_url(dir, self.mwbranch)
            self.git_command(dir, ["reset", "--hard", "HEAD^"])
            self.scap_backport([change_url])
            self.git_command(dir, ["pull"])
            announce(".gitmodules tweak successful")

    def setup_core_gitmodules(self):
        self.setup_gitmodule(
            self.mwcore_dir,
            "submodule.extensions/GrowthExperiments.url",
            "/mediawiki/extensions/GrowthExperiments",
        )
        self.setup_gitmodule(
            self.mwcore_dir,
            "submodule.extensions/VisualEditor.url",
            "/mediawiki/extensions/VisualEditor",
        )
        self.setup_gitmodule(
            self.mwvisualeditor_dir,
            "submodule.lib/ve.url",
            "/VisualEditor/VisualEditor",
        )

    def install_commit_hook(self, path):
        hookdir = path + "/hooks"
        if not os.path.isdir(hookdir):
            os.makedirs(hookdir)
        subprocess.check_call(
            [
                "scp",
                "-p",
                "-P",
                "29418",
                "traindev@%s:hooks/commit-msg" % self.gerrit_domain,
                hookdir,
            ]
        )

    def git_clone(self, branch, repo, path):
        """clones a repo"""
        announce("Updating checkout of %s (%s) in %s" % (repo, branch, path))
        if not os.path.isdir(path):
            subprocess.check_call(["git", "clone", "-b", branch, repo, path])
            self.install_commit_hook(path + "/.git/")

        self.git_command(path, ["fetch", "-q", "origin", branch])
        self.git_command(path, ["reset", "-q", "--hard", "FETCH_HEAD"])

    def push_and_collect_urls(self, repo, branch) -> list:
        """Returns a list of change urls after pushing to gerrit"""
        git_response = subprocess.check_output(
            ["git", "-C", repo, "push", "origin", "HEAD:refs/for/%s" % branch],
            text=True,
            stderr=subprocess.STDOUT,
        )
        return re.findall(r"remote:\s+(%s/c/\S+)" % self.gerrit_url, git_response)

    def push_and_collect_url(self, repo, branch):
        """returns the change url after pushing to gerrit"""

        change_urls = self.push_and_collect_urls(repo, branch)

        if len(change_urls) != 1:
            raise Exception(f"Expected one change_url but got {change_urls}")

        change_url = change_urls[0]
        logging.info("change url is " + change_url)
        return change_url

    def get_change_detail(self, change_url) -> dict:
        """Returns a dictionary representing the specified change"""
        change_number = self.gerrit.change_number_from_url(change_url)
        return self.gerrit.change_detail(change_number).get()

    def approve_change(self, change_url):
        """approve the change"""
        change_number = self.gerrit.change_number_from_url(change_url)
        current_rev = self.gerrit.change_detail(change_number).get()["current_revision"]
        logging.info("Approving change %s" % change_number)
        subprocess.check_call(
            [
                "ssh",
                "-p",
                "29418",
                self.gerrit_domain,
                "gerrit review",
                current_rev,
                "--code-review",
                "+2",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def approve_and_wait_for_merge(self, change_url):
        """approves the change and returns after change has merged"""
        self.approve_change(change_url)
        logging.info("Waiting for %s to merge...", change_url)

        while True:
            detail = self.get_change_detail(change_url)
            status = detail.get("status")
            mergeable = detail.get("mergeable")

            if status == "MERGED":
                logging.info("%s has merged", change_url)
                return

            if mergeable is False:
                raise RuntimeError(
                    f"{change_url} is not mergeable (probably due to conflict)"
                )

            time.sleep(2)

    def scap_backport(self, args_list):
        """Run scap backport and perform a standard interaction with it"""
        child = self._start_scap_backport(args_list)
        try:
            self._scap_backport_interact(child, args_list)
        finally:
            child.terminate(force=True)

    def _pexpect_spawn(self, cmd, *args) -> pexpect.spawn:
        return pexpect.spawn(
            cmd, *args, logfile=sys.stdout, encoding="utf-8", timeout=120
        )

    def _start_scap_backport(self, args_list) -> pexpect.spawn:
        cmd = ["scap", "backport", "--stop-before-sync"] + args_list
        logging.info("Running %s" % " ".join(cmd))

        return self._pexpect_spawn(cmd[0], cmd[1:])

    def _scap_backport_interact(self, child: pexpect.spawn, args_list=[]):
        if "--revert" in args_list:
            child.expect_exact("Please supply a reason for revert")
            child.sendline("")  # Accept the default revert reason

            # scap backport --revert performs an initial scap prep auto to reset
            # the workspace so we watch for that here to avoid confusing the
            # next stage.
            child.expect_exact("Started scap prep auto")
            child.expect_exact("Finished scap prep auto")

        while True:
            index = child.expect_exact(
                ["Backport the changes? [y/N]:", "Finished scap prep auto"]
            )
            if index == 0:
                child.sendline("y")
            elif index == 1:
                break
            else:
                raise Exception("This should never happen")

        logging.info("Waiting for scap backport to terminate")
        child.expect_exact("Skipping sync since --stop-before-sync was specified")
        child.wait()
        if child.exitstatus != 0:
            raise RuntimeError(
                " ".join(["scap", "backport"] + args_list)
                + f" exited with status {child.exitstatus}"
            )

    def write_to_file(self, filepath, message: str, append=True):
        """
        If 'append' is True, write 'message' to the end of the
        specified file.  If 'append' is False, write 'message' at
        the top of the file.

        'message' will have a newline appended to it.
        """

        message += "\n"

        if append:
            with open(filepath, "a") as f:
                f.write(message)
        else:
            with open(filepath, "r") as f:
                contents = f.read()
            with open(filepath, "w") as f:
                f.write(message + contents)

    def git_command(self, repo, command):
        subprocess.check_call(["git", "-C", repo] + command)

    def git_commit(self, repo, commit_message, filepath):
        command = ["commit", "-m", commit_message, filepath]
        self.git_command(repo, command)

    def get_change_id(self, repo) -> str:
        """
        Return the Change-Id of the HEAD commit in the specified git repo directory.
        """
        return (
            re.search(r"(?m)Change-Id:.+$", gitcmd("-C", repo, "log", "-1"))
            .group()
            .split(" ")[1]
        )

    def change_and_push_file(
        self, repo, branch, file, text, commit_message, append=True
    ):
        self.write_to_file(file, text, append)
        self.git_commit(repo, commit_message, file)
        return self.push_and_collect_url(repo, branch)

    def setup_config_change(self, mode, branch="train-dev", context=""):
        """
        Pushes a commit to the operations/config repo on Gerrit.  The local commit
        is discarded.

        If mode is "already merged", the change is +2'd and we wait for it to merge.

        Returns the URL of the change.
        """
        # Freshen up
        self.setup_mediawiki_config_repo()

        readme_path = self.mwconfig_dir + "/README"
        text = (
            f"\nAdded by setup_config_change({mode}, {context}) on "
            + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        )
        change_url = self.change_and_push_file(
            self.mwconfig_dir,
            branch,
            readme_path,
            text,
            f"setup_config_change({context})",
        )
        self.git_command(self.mwconfig_dir, ["reset", "--hard", "HEAD~1"])

        if mode == "already_merged":
            logging.info(
                "Merging the change before scap backport to verify that this is okay"
            )
            self.approve_and_wait_for_merge(change_url)

        return change_url

    @contextlib.contextmanager
    def wikiversions_all_temporarily_set_to_old(self):
        wikiversions_filename = "/srv/mediawiki-staging/wikiversions-dev.json"

        with open(wikiversions_filename) as f:
            saved_wikiversions_dev = f.read()

        subprocess.check_call(
            [
                "scap",
                "update-wikiversions",
                "--no-update-php-symlink",
                "all",
                self.oldmwversion,
            ]
        )

        try:
            yield
        finally:
            with scap.utils.temp_to_permanent_file(wikiversions_filename) as f:
                f.write(saved_wikiversions_dev)

    def make_mediawiki_core_developers_md_change(self, who, where, extra_text=None):
        """
        Makes a change to mediawiki/core's DEVELOPERS.md file.
        A line is written to either the top or the bottom of the file
        depending on the value of "where" (which must be "top" or "bottom").
        The change is committed and pushed and the change URL is returned.
        The commit is discarded from the local checkout before returning.
        """
        assert where in ["top", "bottom"]

        developers_path = self.mwcore_dir + "/DEVELOPERS.md"
        text = f"Added by {who} on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        if extra_text:
            text += "\n" + extra_text

        change_url = self.change_and_push_file(
            self.mwcore_dir,
            self.mwbranch,
            developers_path,
            text,
            f"{who}: add a line to the {where} of DEVELOPERS.md",
            False if where == "top" else True,
        )

        self.git_command(self.mwcore_dir, ["reset", "--hard", "HEAD~1"])

        return change_url

    # T317795
    def backport_non_live_mediawiki_core_change(self):
        change_url = self.make_mediawiki_core_developers_md_change(
            "backport_non_live_mediawiki_core_change",
            "bottom",
        )
        with self.wikiversions_all_temporarily_set_to_old():
            child = self._start_scap_backport([change_url])
            try:
                child.expect_exact(
                    "not found in any deployed wikiversion. Deployed wikiversions:"
                )
                child.expect_exact("Continue with Backport? [y/N]:")
                child.sendline("y")
                self._scap_backport_interact(child)
            finally:
                child.terminate(force=True)

    def merge_commit_backport(self):
        """makes and backports a change that should result in a merge commit"""
        change_url1 = self.make_mediawiki_core_developers_md_change(
            "merge_commit_backport",
            "top",
        )
        change_url2 = self.make_mediawiki_core_developers_md_change(
            "merge_commit_backport",
            "bottom",
            "This is expected to result in a merge commit",
        )

        self.scap_backport([change_url1])
        self.scap_backport([change_url2])

    # There are two types of dependency chains that can be set up.  One occurs when you push
    # multiple commits at once.  We call this a relation chain.  The other type is established
    # using the Depends-On footer in commit messages.  We call this the depends-on chain.  The
    # chains are set up differently but scap backport must behave the same way for either type.

    def setup_dependency_chain(
        self, *, style=None, use_different_branches=False
    ) -> list:
        """
        Constructs a chain of 3 commits in one of two ways depending on
        'style', which must be "relation" or "Depends-On".  The chain is pushed to
        Gerrit and a list of change URLs is returned.

        The first commit in the chain is first created on the master branch (and
        pushed to Gerrit) and then cherry-picked into the train branch (wmf/*),
        simulating a typical backport workflow.  This helps exercise T324275/T323277.

        If `use_different_branches` is True then the last commit in the chain will
        use a different branch from the two first commits

        """
        allowed_styles = ["relation", "Depends-On"]

        if style not in allowed_styles:
            raise ValueError(f"style must be one of {allowed_styles}")

        # Freshen
        self.setup_core_repo(branch="master")

        files = ["COPYING", "CREDITS", "FAQ"]
        change_urls = []

        def maybe_push_commit(branch=None):
            if style == "Depends-On":
                change_urls.append(
                    self.push_and_collect_url(
                        self.mwcore_dir, branch if branch else self.mwbranch
                    )
                )
                self.git_command(self.mwcore_dir, ["reset", "--hard", "HEAD~1"])

        # Prepare the first commit in the master branch first.
        file = files[0]
        path = os.path.join(self.mwcore_dir, file)

        text = "Added by setup_dependency_chain (1) on " + datetime.now().strftime(
            "%Y-%m-%d %H:%M:%S"
        )
        commit_msg = f"{file}: setup_dependency_chain (1)"
        self.change_and_push_file(self.mwcore_dir, "master", path, text, commit_msg)
        commit = subprocess.check_output(
            ["git", "-C", self.mwcore_dir, "rev-parse", "HEAD"], text=True
        ).strip()
        change_id = self.get_change_id(self.mwcore_dir)

        current_branch = None
        if use_different_branches:
            # Deployed branches in ascending order. When this var gets used below, we end up with the following
            # arrangement of commits. Assuming latest branch is 1.40.0-wmf.10:
            # * [0]COPYING patch: 1.40.0-wmf.9
            # * [1]CREDITS patch: 1.40.0-wmf.9
            # * [2]FAQ patch:     1.40.0-wmf.10
            deployed_branches = [
                "wmf/" + ver
                for ver in scap.utils.get_active_wikiversions(self.mwconfig_dir, "dev")
            ]
            current_branch = deployed_branches[0]

        # Now cherry pick that commit as the first in a chain for the train branch.
        self.setup_core_repo(current_branch)
        self.git_command(self.mwcore_dir, ["cherry-pick", "-x", "-Xtheirs", commit])
        maybe_push_commit(current_branch)

        # Then stack up two more commits on top
        for i, file in enumerate(files[1:]):
            if use_different_branches:
                current_branch = deployed_branches[i]
                self.setup_core_repo(current_branch)

            path = os.path.join(self.mwcore_dir, file)
            text = "Added by setup_dependency_chain (2) on " + datetime.now().strftime(
                "%Y-%m-%d %H:%M:%S"
            )
            commit_msg = f"{file}: setup_dependency_chain (2)"

            if style == "Depends-On":
                commit_msg += f"\n\nDepends-On: {change_id}"

            self.write_to_file(path, text, append=True)
            self.git_commit(self.mwcore_dir, commit_msg, path)
            change_id = self.get_change_id(self.mwcore_dir)
            maybe_push_commit(current_branch)

        if style == "relation":
            change_urls = self.push_and_collect_urls(self.mwcore_dir, self.mwbranch)

        return change_urls

    def dependency_chain_backport_fails(self, change_urls) -> int:
        """
        Attempts to backport one or more commits with unmerged dependencies (of whatever type).

        Returns the exit status of scap backport
        """

        child = self._start_scap_backport(change_urls)
        child.expect(
            r"Change '\d+' has dependencies '\[.*\]', which are not merged or scheduled for backport"
        )
        child.wait()
        return child.exitstatus

    def depends_with_nonprod_branch(
        self, root_change_in_prod_branch=False
    ) -> pexpect.spawn:
        """
        Using the configuration repository (`operations/mediawiki-config`), creates two changes in a Depends-On
        relationship using a non-production branch. If root_change_in_prod_branch is True, then the change with the
        "Depends-On" will use the production branch instead
        """

        # Make a configuration change on non-prod branch. In the case
        # of train-dev (where these tests run), the prod branch is "train-dev"
        # "master" also exists.
        self.setup_mediawiki_config_repo(branch="master")

        readme_path = self.mwconfig_dir + "/README"
        text = "Added by depends_with_nonprod_branch on " + datetime.now().strftime(
            "%Y-%m-%d %H:%M:%S"
        )
        change_url = self.change_and_push_file(
            self.mwconfig_dir,
            "master",
            readme_path,
            text,
            "A sample config change made on a non-prod branch",
        )
        change_id = self.get_change_id(self.mwconfig_dir)
        self.git_command(self.mwconfig_dir, ["reset", "--hard", "HEAD~1"])

        # And deploy it, answering 'y' to the confirmation of the weird situation.
        child = self._start_scap_backport([change_url])
        try:
            child.expect_exact(
                "not found in any deployed wikiversion. Deployed wikiversions:"
            )
            child.expect_exact("Continue with Backport? [y/N]:")
            child.sendline("y")
            self._scap_backport_interact(child)
        finally:
            child.terminate(force=True)

        # Now create a new patch which "Depends-On" the first one. By default, it uses the same non-prod branch
        self.setup_mediawiki_config_repo()
        text = "Added by depends_with_nonprod_branch on " + datetime.now().strftime(
            "%Y-%m-%d %H:%M:%S"
        )
        if root_change_in_prod_branch:
            branch = "train-dev"
        else:
            branch = "master"
        change_url = self.change_and_push_file(
            self.mwconfig_dir,
            branch,
            readme_path,
            text,
            "Depends-On references commit in non-prod branch\n\nDepends-On: %s"
            % change_id,
            False,
        )
        self.git_command(self.mwconfig_dir, ["reset", "--hard", "HEAD~1"])

        return self._start_scap_backport([change_url])

    def setup_extension_change(self):
        """Creates an extension change and pushes to Gerrit.  This will result in
        a submodule update commit.  The local commit is discarded.

        Returns the change url.

        FIXME: it might be interesting to also test for submodule _and_ merge commit
        """
        # Freshen up
        self.setup_growthexperiments_repo()

        copying_path = self.mwgrowthexperiments_dir + "/COPYING"
        text = "\nAdded by setup_extension_change on " + datetime.now().strftime(
            "%Y-%m-%d %H:%M:%S"
        )
        change_url = self.change_and_push_file(
            self.mwgrowthexperiments_dir,
            self.mwbranch,
            copying_path,
            text,
            "setup_extension_change: add line to bottom of COPYING",
        )
        self.git_command(self.mwgrowthexperiments_dir, ["reset", "--hard", "HEAD~1"])
        return change_url

    def cleanup_unusual_submodule_change(self):
        """Avoids merge conflicts in future test runs by updating the submodule link"""
        ve_dir = self.mwvisualeditor_dir + "/lib/ve"

        announce("cleanup_unusual_submodule_change started")

        announce("Updating checkout of VisualEditor/VisualEditor")
        self.git_command(ve_dir, ["checkout", self.mwbranch])
        self.git_command(ve_dir, ["pull"])
        self.git_command(self.mwvisualeditor_dir, ["checkout", self.mwbranch])

        if scap.git.file_has_unstaged_changes(
            "lib/ve", location=self.mwvisualeditor_dir
        ):
            announce(
                "Committing and backporting .gitmodules update in extensions/VisualEditor"
            )
            self.git_commit(
                self.mwvisualeditor_dir, "update ve submodule link", "lib/ve"
            )
            self.scap_backport(
                [self.push_and_collect_url(self.mwvisualeditor_dir, self.mwbranch)]
            )
        announce("cleanup_unusual_submodule_change completed")

    def setup_unusual_submodule_path_change(self):
        """Creates a change in a submodule that has an unusual path and pushes to gerrit.
        Updates the submodule link with another commit. The local commit to the submodule is discarded.

        Returns the change url.
        """
        self.setup_visualeditor_repo()
        self.cleanup_unusual_submodule_change()
        ve_dir = self.mwvisualeditor_dir + "/lib/ve"

        readme_path = ve_dir + "/README.md"
        text = (
            "\nAdded by setup_unusual_submodule_path_change on "
            + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        )
        change_url = self.change_and_push_file(
            ve_dir,
            self.mwbranch,
            readme_path,
            text,
            "setup_unusual_submodule_path_change: add line to bottom of README.md",
        )

        self.git_command(ve_dir, ["reset", "--hard", "HEAD~1"])
        return change_url

    def growthexperiments_extension_revert(self, change_url):
        """reverts a change

        Returns the number of commits different between the local deploy branch and origin
        """
        self.scap_backport(["--revert", change_url])
        # confirm only one commit was pushed to gerrit (no submission of security patch)
        # workdir does not contain patches; check must be done in /srv/mediawiki-staging
        return subprocess.check_output(
            [
                "git",
                "-C",
                self.mwstaging_dir + "/" + self.mwphp_dir,
                "rev-list",
                "@{u}..HEAD",
                "--count",
            ],
            text=True,
        ).strip()

    def backport_with_extra_commits(self):
        """backports a change when there are unexpected new commits merged to the branch"""
        # create, push, and merge an operations/mediawiki-config commit
        self.setup_config_change(
            "already_merged", context="backport_with_extra_commits"
        )

        change_url = self.setup_extension_change()
        return self._pexpect_spawn("scap backport --stop-before-sync %s" % change_url)

    def reject_change(self, change_url):
        """reject the change based on the change url.
        Returns the change number.
        """
        change_number = self.gerrit.change_number_from_url(change_url)
        current_rev = self.gerrit.change_detail(change_number).get()["current_revision"]
        logging.info("Rejecting change %s" % change_number)
        subprocess.check_output(
            [
                "ssh",
                "-p",
                "29418",
                self.gerrit_domain,
                "gerrit review",
                current_rev,
                "--code-review",
                "-2",
            ],
            stderr=subprocess.DEVNULL,
        )
        return change_number


class TestBackports(unittest.TestCase):
    """tests the backports"""

    backports_test_helper = None

    @classmethod
    def setup_class(cls):
        cls.backports_test_helper = BackportsTestHelper()
        cls.backports_test_helper.setup()

    def test_config_changes(self):
        for mode in ["normal", "already_merged"]:
            announce("Config change test: %s" % mode)
            change_url = self.backports_test_helper.setup_config_change(
                mode, context=f"test_config_changes({mode})"
            )
            self.backports_test_helper.scap_backport([change_url])

    def test_merge_commits(self):
        announce(
            "Testing mediawiki/core changes (two independent commits resulting in a merge commit)"
        )
        self.backports_test_helper.merge_commit_backport()

    def test_extension_and_revert_with_security_patch(self):
        announce("Testing mediawiki/extensions/GrowthExperiments change")
        change_url = self.backports_test_helper.setup_extension_change()
        self.backports_test_helper.scap_backport([change_url])
        announce(
            "Testing mediawiki/extensions/GrowthExperiments backport --revert change"
        )
        # assert the security patch only exists locally
        self.assertEqual(
            self.backports_test_helper.growthexperiments_extension_revert(change_url),
            "1",
        )

    def test_unusual_submodule_path_and_revert(self):
        """Tests recognition of a submodule with a project name that differs from the submodule name"""
        announce("Testing mediawiki/extensions/VisualEditor/lib/ve change")
        change_url = self.backports_test_helper.setup_unusual_submodule_path_change()
        self.backports_test_helper.scap_backport([change_url])
        self.backports_test_helper.cleanup_unusual_submodule_change()
        self.backports_test_helper.scap_backport(["--revert", change_url])
        self.backports_test_helper.cleanup_unusual_submodule_change()

    def _test_dependencies(self, dep_type, change_urls):
        if len(change_urls) != 3:
            raise Exception(f"Expected to get 3 change_urls but got {len(change_urls)}")

        announce("Testing that backport with unmerged unspecified dependencies fails")

        bad_combos = [
            [change_urls[1]],
            [change_urls[2]],
            [change_urls[1], change_urls[2]],
        ]

        for urls in bad_combos:
            exit_status = self.backports_test_helper.dependency_chain_backport_fails(
                urls
            )
            self.assertNotEqual(exit_status, 0)

        announce(f"Testing fully-specified {dep_type} chains can be backported")

        good_combos = [
            [change_urls[0], change_urls[1]],
            # This verifies that specifying an already-merged dependency is ok.
            [change_urls[1], change_urls[2]],
        ]

        for urls in good_combos:
            self.backports_test_helper.scap_backport(urls)

    def test_relation_chain(self):
        change_urls = self.backports_test_helper.setup_dependency_chain(
            style="relation"
        )
        self._test_dependencies("relation", change_urls)

    def test_depends_on_chain(self):
        change_urls = self.backports_test_helper.setup_dependency_chain(
            style="Depends-On"
        )
        self._test_dependencies("Depends-On", change_urls)

    def test_non_production_branch(self):
        announce(
            "Testing chain of dependencies in a non-production branch is allowed pending user approval"
        )
        child = self.backports_test_helper.depends_with_nonprod_branch()
        try:
            child.expect_exact(
                "not found in any deployed wikiversion. Deployed wikiversions:"
            )
            child.expect_exact("Continue with Backport? [y/N]:")
            child.sendline("n")
        finally:
            child.terminate(force=True)

    def test_depends_on_different_branch(self):
        announce(
            "Testing Depends-On a different branch makes the dependency be ignored"
        )

        child = self.backports_test_helper.depends_with_nonprod_branch(
            root_change_in_prod_branch=True
        )
        child.expect(
            r"Change.*specified 'Depends-On'.*Ignore dependencies and continue with Backport?"
        )
        child.sendline("y")
        self.backports_test_helper._scap_backport_interact(child)

    # Verifies T345304
    def test_depends_on_abandoned(self):
        def abandon_change(change):
            self.backports_test_helper.gerrit.post(
                f"{self.backports_test_helper.gerrit_url}/a/changes/{change}/abandon"
            )

        announce(
            "Testing abandoned Depends-On changes. Different branches should be ignored"
        )

        change_urls = self.backports_test_helper.setup_dependency_chain(
            style="Depends-On", use_different_branches=True
        )

        # First change and middle change are in the same branch and middle change Depends-On first change. Abandoning
        # the first change and trying to backport the middle one should make the backport fail
        abandoned_change_number = change_urls[0].split("/")[-1]
        abandon_change(abandoned_change_number)
        backported_change_number = change_urls[1].split("/")[-1]
        child = self.backports_test_helper._start_scap_backport([change_urls[1]])
        child.expect_exact(
            f"Change '{abandoned_change_number}' has been abandoned! Change is pulled by the following dependency"
            f" chain: {backported_change_number} -> {abandoned_change_number}"
        )
        child.wait()

        # Now we abandon the middle change. Last change Depends-On middle change but they belong to different branches,
        # so backporting the last change should succeed when the user confirms the operation
        abandoned_change_number = change_urls[1].split("/")[-1]
        abandon_change(abandoned_change_number)
        backported_change_number = change_urls[2].split("/")[-1]
        child = self.backports_test_helper._start_scap_backport([change_urls[2]])
        child.expect(
            rf"Change {backported_change_number} specified 'Depends-On'.*Ignore dependencies and continue with"
            " Backport?"
        )
        child.sendline("y")
        self.backports_test_helper._scap_backport_interact(child)

    def test_extra_commit_confirmation(self):
        announce(
            "Testing whether confirmation is requested if extra commits are pulled"
        )
        child = self.backports_test_helper.backport_with_extra_commits()
        try:
            child.expect_exact("Backport the changes? [y/N]: ")
            child.sendline("y")

            child.expect_exact(
                "The following are unexpected commits pulled from origin for /srv/mediawiki-staging"
            )
            child.expect_exact("Would you like to see the diff? [Y/n]: ")
            child.sendline("y")

            child.expect_exact(
                "There were unexpected commits pulled from origin for /srv/mediawiki-staging."
            )
            child.expect_exact(
                "Continue with deployment (all patches will be deployed)? [y/N]: "
            )

            # Go ahead and "deploy" the extra commit to avoid confusing other tests by leaving
            # a merged-but-unpulled commit
            child.sendline("y")

            self.backports_test_helper._scap_backport_interact(child)
        finally:
            child.terminate(force=True)

    def test_concurrency(self):
        announce("Testing behavior during concurrent backports")

        change_url = self.backports_test_helper.setup_config_change(
            "normal",
            context=f"\nAdded by test_concurrency on {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}. Pass 1",
        )
        patch_id = re.sub(".*/", "", change_url)
        # Start backport process that should immediately grab the "backport" lock
        backport_first = self.backports_test_helper._start_scap_backport([change_url])

        change_url = self.backports_test_helper.setup_config_change(
            "normal",
            context=f"\nAdded by test_concurrency on {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}. Pass 2",
        )
        # A new attempt to backport should fail to grab the lock indicating the patch id of the blocker
        backport_second = self.backports_test_helper._start_scap_backport([change_url])
        try:
            backport_second.expect(rf"backport is locked by.*{patch_id}", timeout=3)
        finally:
            backport_first.terminate(force=True)
            backport_second.terminate(force=True)

    def test_backport_of_rejected_change(self):
        """Test that the backport of a rejected change is aborted with an appropriate error message."""
        announce(
            "Testing operations/mediawiki-config, --backporting of rejected change"
        )
        change_url = self.backports_test_helper.setup_config_change(
            "normal",
            context=f"\nAdded by test_backport_of_rejected_change on {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}.",
        )
        child = self.backports_test_helper._start_scap_backport([change_url])
        change_number = self.backports_test_helper.reject_change(change_url)

        try:
            child.expect_exact("Backport the changes? [y/N]: ")
            child.sendline("y")
            child.expect_exact(
                f"The change '{change_number}' has been rejected (Code-Review -2) by 'TrainConductor"
            )
        finally:
            child.terminate(force=True)
        announce(
            f"Finished testing operations/mediawiki-config backport --rejected change (Code-Review -2) for {change_number}"
        )

    # T317795
    def test_backport_to_checked_out_but_not_live_branch(self):
        self.backports_test_helper.backport_non_live_mediawiki_core_change()
