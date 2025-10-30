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
from scap.gerrit import GerritSession
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
        # Any operations that require auth should use ssh.
        self.gerrit = GerritSession(url=self.gerrit_url, use_auth=False)

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
        self.git_command(
            self.mwvisualeditor_dir,
            ["submodule", "update", "--init", "--checkout", "--force"],
        )
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
        self.git_command(
            path, ["checkout", "-q", "--force", "-B", branch, f"origin/{branch}"]
        )
        # Ensure local branch exactly matches remote to prevent implicit merges
        self.git_command(path, ["reset", "--hard", f"origin/{branch}"])

    def set_push_url(self, repo):
        url = scap.git.remote_get_url(repo, push=True)

        if not url.startswith(self.gerrit_url):
            return

        rest = url[len(self.gerrit_url) :]
        push_url = f"ssh://traindev@{self.gerrit_domain}:29418{rest}"
        scap.git.remote_set_url(repo, push_url, push=True)

    def push_and_collect_urls(self, repo, branch) -> list:
        """Returns a list of change urls after pushing to gerrit"""
        self.set_push_url(repo)
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

    def abandon_change(self, change_url):
        """Abandon the specified change"""
        change_number = self.gerrit.change_number_from_url(change_url)
        current_rev = self.gerrit.change_detail(change_number).get()["current_revision"]
        logging.info("Abandoning change %s" % change_number)
        subprocess.check_call(
            [
                "ssh",
                "-p",
                "29418",
                self.gerrit_domain,
                "gerrit review",
                current_rev,
                "--abandon",
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

    def get_change_id(self, repo, branch=None) -> str:
        """
        Return the Change-Id of the HEAD commit in the specified git repo directory.
        """
        if branch is not None:
            args = [repo, "log", branch, "-1"]
        else:
            args = [repo, "log", "-1"]
        return (
            re.search(r"(?m)Change-Id:.+$", gitcmd("-C", *args)).group().split(" ")[1]
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
                "-Dmanage_mediawiki_php_symlink:False",
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

        # Ensure we start with a clean, synchronized repository state
        self.setup_core_repo()

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
                    "not found in any live wikiversion. Live wikiversions:"
                )
                child.expect_exact("Continue with backport anyway? [y/N]:")
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
        self.scap_backport([change_url1])

        change_url2 = self.make_mediawiki_core_developers_md_change(
            "merge_commit_backport",
            "bottom",
            "This is expected to result in a merge commit",
        )
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
            if len(deployed_branches) != 2:
                raise Exception(
                    f"Expected 2 deployed branches, got {len(deployed_branches)}.  Please run ~/train and then restart tests"
                )
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
            r"Change '\d+' has dependency '\d+', which is not merged or scheduled for backport"
        )
        child.wait()
        return child.exitstatus

    def master_dependency_backport_fails(self, change_urls) -> int:
        """
        Attempts to backport one or more commits with master dependencies that
        are not present in all deployable branches.

        Returns the exit status of scap backport
        """

        child = self._start_scap_backport(change_urls)
        child.expect(r"Master dependencies must be deployed to all active branches")
        child.wait()
        return child.exitstatus

    def setup_depends_on_master_included_in_production(self):
        """
        T388025: Set up a scenario where a master branch change is included in production
        branches (simulating normal train process), and a config change depends
        on it.

        Returns a list of change URLs: [wmf_branch_change_url, config_change_url].
        """

        # Create and merge a change to mediawiki/core master branch
        self.setup_core_repo(branch="master")

        filepath = self.mwcore_dir + "/README.md"
        text = (
            "Added by setup_depends_on_master_included_in_production on "
            + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        )
        commit_msg = "T388025 test: master change to be included in production branches"

        master_change_url = self.change_and_push_file(
            self.mwcore_dir, "master", filepath, text, commit_msg
        )
        master_change_id = self.get_change_id(self.mwcore_dir)
        self.approve_and_wait_for_merge(master_change_url)

        self.git_command(self.mwcore_dir, ["checkout", "master"])
        self.git_command(self.mwcore_dir, ["pull", "origin", "master"])
        master_commit = subprocess.check_output(
            ["git", "-C", self.mwcore_dir, "rev-parse", "HEAD"], text=True
        ).strip()

        # Simulate the master change being included in production branches
        # This simulates what happens during the normal train process
        self.setup_core_repo(branch=self.mwbranch)

        # Cherry-pick the master commit into the production branch
        # Use the same approach as setup_dependency_chain()
        self.git_command(
            self.mwcore_dir, ["cherry-pick", "-x", "-Xtheirs", master_commit]
        )

        wmf_change_url = self.push_and_collect_url(self.mwcore_dir, self.mwbranch)
        self.approve_and_wait_for_merge(wmf_change_url)
        self.git_command(self.mwcore_dir, ["reset", "--hard", "HEAD~1"])

        # Create config change that depends on the master change
        # This mimics the actual scenario from T388025 where config changes
        # have Depends-On pointing to the master change

        # Freshen up the config repo
        self.setup_mediawiki_config_repo()

        readme_path = self.mwconfig_dir + "/README"
        text = "\nAdded by T388025 test config change on " + datetime.now().strftime(
            "%Y-%m-%d %H:%M:%S"
        )
        commit_msg = "T388025 test config change\n\n" f"Depends-On: {master_change_id}"

        config_change_url = self.change_and_push_file(
            self.mwconfig_dir,
            "train-dev",  # Use the config branch
            readme_path,
            text,
            commit_msg,
        )
        self.git_command(self.mwconfig_dir, ["reset", "--hard", "HEAD~1"])

        return [wmf_change_url, config_change_url]

    def setup_depends_on_master_not_deployed_to_all_branches(self):
        """
        Set up a scenario where a master branch MediaWiki code change exists
        but is NOT present in all deployable branches, and a config change
        depends on it. This should cause the master dependency validation
        to fail with an error about missing branches.

        Returns a list of change URLs: [master_change_url, config_change_url].
        """

        # Create and merge a change to mediawiki/core master branch
        self.setup_core_repo(branch="master")

        filepath = self.mwcore_dir + "/README.md"
        text = (
            "Added by setup_depends_on_master_not_deployed_to_all_branches on "
            + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        )
        commit_msg = "Test: master change NOT deployed to all branches"

        master_change_url = self.change_and_push_file(
            self.mwcore_dir, "master", filepath, text, commit_msg
        )
        master_change_id = self.get_change_id(self.mwcore_dir)
        self.approve_and_wait_for_merge(master_change_url)

        # Importantly: We DO NOT cherry-pick this master change into production branches
        # This simulates a scenario where a master change was merged but hasn't been
        # deployed to wmf branches yet (e.g., waiting for the next train)

        # Create config change that depends on the master change
        self.setup_mediawiki_config_repo()

        readme_path = self.mwconfig_dir + "/README"
        text = (
            "\nAdded by master not deployed test config change on "
            + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        )
        commit_msg = (
            "Test config change with undeployed master dependency\n\n"
            f"Depends-On: {master_change_id}"
        )

        config_change_url = self.change_and_push_file(
            self.mwconfig_dir,
            "train-dev",  # Use the config branch
            readme_path,
            text,
            commit_msg,
        )
        self.git_command(self.mwconfig_dir, ["reset", "--hard", "HEAD~1"])

        return [master_change_url, config_change_url]

    def setup_cross_repo_same_changeid_dependency(self):
        """
        T387798: Set up a scenario where:
        1. A master branch change exists with Change-Id X
        2. The same Change-Id X is also on a wmf branch (different change number)
        3. A config change depends on Change-Id X
        4. Both the wmf branch change and config change are deployed together

        This reproduces the scenario where scap warns about the master branch
        version not being deployed, even though the wmf version is being deployed.

        Returns a list of change URLs: [master_change_url, wmf_change_url, config_change_url].
        """

        # 1. Create a change on master branch first
        self.setup_core_repo(branch="master")

        filepath = self.mwcore_dir + "/README.md"
        text = (
            "Added by setup_cross_repo_same_changeid_dependency master on "
            + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        )
        commit_msg = "T387798 test: master change with shared Change-Id"

        master_change_url = self.change_and_push_file(
            self.mwcore_dir, "master", filepath, text, commit_msg
        )
        shared_change_id = self.get_change_id(self.mwcore_dir)
        # Reset local checkout after pushing
        self.git_command(self.mwcore_dir, ["reset", "--hard", "HEAD~1"])

        # 2. Create a different change on wmf branch but with the SAME Change-Id
        # This simulates the scenario where the same logical change exists
        # on both master and wmf branches (common in MediaWiki workflow)
        self.setup_core_repo(branch=self.mwbranch)

        # Make a different file change but use the same Change-Id
        filepath2 = self.mwcore_dir + "/COPYING"
        text2 = (
            "Added by setup_cross_repo_same_changeid_dependency wmf on "
            + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        )
        commit_msg2 = f"T387798 test: wmf change with shared Change-Id\n\nChange-Id: {shared_change_id}"

        wmf_change_url = self.change_and_push_file(
            self.mwcore_dir, self.mwbranch, filepath2, text2, commit_msg2
        )
        # Reset local checkout after pushing
        self.git_command(self.mwcore_dir, ["reset", "--hard", "HEAD~1"])

        # 3. Create config change that depends on the shared Change-Id
        self.setup_mediawiki_config_repo()

        readme_path = self.mwconfig_dir + "/README"
        text3 = "\nAdded by T387798 test config change on " + datetime.now().strftime(
            "%Y-%m-%d %H:%M:%S"
        )
        commit_msg3 = "T387798 test config change\n\n" f"Depends-On: {shared_change_id}"

        config_change_url = self.change_and_push_file(
            self.mwconfig_dir,
            "train-dev",
            readme_path,
            text3,
            commit_msg3,
        )
        # Reset local checkout after pushing
        self.git_command(self.mwconfig_dir, ["reset", "--hard", "HEAD~1"])

        # Return all three changes: master, wmf, and config
        # In T387798 scenario, we deploy wmf + config together
        # Master should not cause warnings since wmf version is being deployed
        return [master_change_url, wmf_change_url, config_change_url]

    def setup_depends_on_relation_chain(self) -> list:
        """
        Constructs a chain of 3 commits where each commit beyond the first has
        a Depends-On footer pointing to the prior commit, but all commits are
        pushed together as a single relation chain stack (not individually).
        This tests the combination of Depends-On footers with relation chain structure.

        All commits use the same train branch to ensure they can be pushed as a relation chain.

        Returns a list of change URLs.
        """
        # Freshen - use train branch for all commits
        self.setup_core_repo(branch=self.mwbranch)

        files = ["COPYING", "CREDITS", "FAQ"]

        # Create the first commit
        file = files[0]
        path = os.path.join(self.mwcore_dir, file)
        text = (
            "Added by setup_depends_on_relation_chain (1) on "
            + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        )
        commit_msg = f"{file}: setup_depends_on_relation_chain (1)"
        self.write_to_file(path, text, append=True)
        self.git_commit(self.mwcore_dir, commit_msg, path)
        change_id = self.get_change_id(self.mwcore_dir)

        # Create the second commit with Depends-On pointing to the first
        file = files[1]
        path = os.path.join(self.mwcore_dir, file)
        text = (
            "Added by setup_depends_on_relation_chain (2) on "
            + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        )
        commit_msg = (
            f"{file}: setup_depends_on_relation_chain (2)\n\nDepends-On: {change_id}"
        )
        self.write_to_file(path, text, append=True)
        self.git_commit(self.mwcore_dir, commit_msg, path)
        change_id = self.get_change_id(self.mwcore_dir)

        # Create the third commit with Depends-On pointing to the second
        file = files[2]
        path = os.path.join(self.mwcore_dir, file)
        text = (
            "Added by setup_depends_on_relation_chain (3) on "
            + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        )
        commit_msg = (
            f"{file}: setup_depends_on_relation_chain (3)\n\nDepends-On: {change_id}"
        )
        self.write_to_file(path, text, append=True)
        self.git_commit(self.mwcore_dir, commit_msg, path)

        # Push all 3 commits together as a relation chain (not individually)
        change_urls = self.push_and_collect_urls(self.mwcore_dir, self.mwbranch)

        # Reset local commits after pushing
        self.git_command(self.mwcore_dir, ["reset", "--hard", "HEAD~3"])

        return change_urls

    def setup_depends_on_self_cycle(self):
        """
        T408675: Create a single commit whose Depends-On footer references its own Change-Id,
        forming a self-cycle. Returns the change URL.
        """
        # Use MediaWiki core on the train branch
        self.setup_core_repo(branch=self.mwbranch)

        # Create initial commit to obtain a Change-Id
        readme_path = self.mwcore_dir + "/README.md"
        text = "Added by setup_depends_on_self_cycle on " + datetime.now().strftime(
            "%Y-%m-%d %H:%M:%S"
        )
        base_msg = "T408675: self-cycle test"

        # Commit once to get a Change-Id from the commit-msg hook
        self.write_to_file(readme_path, text, append=True)
        self.git_commit(self.mwcore_dir, base_msg, readme_path)
        change_id = self.get_change_id(self.mwcore_dir)

        # Amend the commit message to depend on itself and preserve the Change-Id
        amended_msg = f"{base_msg}\n\nDepends-On: {change_id}\n\nChange-Id: {change_id}"
        self.git_command(self.mwcore_dir, ["commit", "--amend", "-m", amended_msg])

        # Push and return the change URL; clean local state afterwards
        change_url = self.push_and_collect_url(self.mwcore_dir, self.mwbranch)
        self.git_command(self.mwcore_dir, ["reset", "--hard", "HEAD~1"])
        return change_url

    def setup_depends_on_three_commit_cycle(self) -> list:
        """
        T408675: Create three changes A, B, and C such that A→B→C→B forms a cycle.
        This is accomplished by:
          1) Create and push A
          2) Create and push B with Depends-On: A (capture Change-Id B)
          3) Create and push C with Depends-On: B (capture Change-Id C)
          4) Create a new patchset for B with Depends-On: A,C (preserving Change-Id B)

        Returns [change_url_a, change_url_b, change_url_c]
        """
        self.setup_core_repo(branch=self.mwbranch)

        path = self.mwcore_dir + "/FAQ"

        # 1) Create and push A
        text_a = (
            "Added by setup_depends_on_three_commit_cycle A on "
            + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        )
        msg_a = "T408675: three commit cycle A"
        self.write_to_file(path, text_a, append=True)
        self.git_commit(self.mwcore_dir, msg_a, path)
        change_id_a = self.get_change_id(self.mwcore_dir)
        change_url_a = self.push_and_collect_url(self.mwcore_dir, self.mwbranch)
        self.git_command(self.mwcore_dir, ["reset", "--hard", "HEAD~1"])

        # 2) Create and push B with Depends-On: A
        text_b = (
            "Added by setup_depends_on_three_commit_cycle B on "
            + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        )
        msg_b = f"T408675: three commit cycle B\n\nDepends-On: {change_id_a}"
        self.write_to_file(path, text_b, append=True)
        self.git_commit(self.mwcore_dir, msg_b, path)
        change_id_b = self.get_change_id(self.mwcore_dir)
        change_url_b = self.push_and_collect_url(self.mwcore_dir, self.mwbranch)
        self.git_command(self.mwcore_dir, ["reset", "--hard", "HEAD~1"])

        # 3) Create and push C with Depends-On: B
        text_c = (
            "Added by setup_depends_on_three_commit_cycle C on "
            + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        )
        msg_c = f"T408675: three commit cycle C\n\nDepends-On: {change_id_b}"
        self.write_to_file(path, text_c, append=True)
        self.git_commit(self.mwcore_dir, msg_c, path)
        change_id_c = self.get_change_id(self.mwcore_dir)
        change_url_c = self.push_and_collect_url(self.mwcore_dir, self.mwbranch)
        self.git_command(self.mwcore_dir, ["reset", "--hard", "HEAD~1"])

        # 4) Upload a new patchset for B with Depends-On: A,C (preserving Change-Id B)
        # This creates the cycle: A→B→C→B
        text_b_ps2 = (
            "Added by setup_depends_on_three_commit_cycle B-PS2 on "
            + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        )
        msg_b_ps2 = f"T408675: three commit cycle B PS2\n\nDepends-On: {change_id_a}\nDepends-On: {change_id_c}\n\nChange-Id: {change_id_b}"
        self.write_to_file(path, text_b_ps2, append=True)
        self.git_commit(self.mwcore_dir, msg_b_ps2, path)
        _ = self.push_and_collect_url(self.mwcore_dir, self.mwbranch)
        self.git_command(self.mwcore_dir, ["reset", "--hard", "HEAD~1"])

        return [change_url_a, change_url_b, change_url_c]

    def depends_with_nonprod_branch(self) -> pexpect.spawn:
        """
        Using the configuration repository (`operations/mediawiki-config`), creates two changes in a Depends-On
        relationship using a non-production branch as a dependency. The root change is in a production branch.
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

        # merge
        self.approve_change(change_url)

        # Now create a new patch which "Depends-On" the first one. By default, it uses the same non-prod branch
        self.setup_mediawiki_config_repo()
        text = "Added by depends_with_nonprod_branch on " + datetime.now().strftime(
            "%Y-%m-%d %H:%M:%S"
        )
        change_url = self.change_and_push_file(
            self.mwconfig_dir,
            "train-dev",
            readme_path,
            text,
            "Depends-On references commit in non-prod branch\n\nDepends-On: %s"
            % change_id,
            False,
        )
        self.git_command(self.mwconfig_dir, ["reset", "--hard", "HEAD~1"])

        return self._start_scap_backport([change_url])

    def setup_rule_1a_mw_code_to_non_master_mw_code(self):
        """
        T362987 Rule 1a: Set up scenario where MW code change depends on non-master MW code.
        This should do no extra validation.

        Returns a list of change URLs: [dependency_change_url, dependent_change_url].
        """
        # Create first change in mediawiki/core on wmf branch
        self.setup_core_repo(branch=self.mwbranch)

        filepath = self.mwcore_dir + "/README.md"
        text = (
            "Added by setup_rule_1a_mw_code_to_non_master_mw_code (1) on "
            + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        )
        commit_msg = "T362987 Rule 1a test: MW code change (dependency)"

        dep_change_url = self.change_and_push_file(
            self.mwcore_dir, self.mwbranch, filepath, text, commit_msg
        )
        dep_change_id = self.get_change_id(self.mwcore_dir)
        self.approve_and_wait_for_merge(dep_change_url)
        self.git_command(self.mwcore_dir, ["reset", "--hard", "HEAD~1"])

        # Create second change in mediawiki/extensions/VisualEditor that depends on first
        self.setup_visualeditor_repo()

        filepath2 = self.mwvisualeditor_dir + "/README.md"
        text2 = (
            "Added by setup_rule_1a_mw_code_to_non_master_mw_code (2) on "
            + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        )
        commit_msg2 = f"T362987 Rule 1a test: MW extension change depending on non-master MW code\n\nDepends-On: {dep_change_id}"

        dependent_change_url = self.change_and_push_file(
            self.mwvisualeditor_dir, self.mwbranch, filepath2, text2, commit_msg2
        )
        self.git_command(self.mwvisualeditor_dir, ["reset", "--hard", "HEAD~1"])

        return [dep_change_url, dependent_change_url]

    def setup_rule_1b_mw_code_to_master_success(self):
        """
        T362987 Rule 1b success: Set up scenario where MW code change depends on master MW code
        that IS present in the target branch.

        Returns a list of change URLs: [master_change_url, wmf_change_url, dependent_change_url].
        """
        # Create master change in mediawiki/core
        self.setup_core_repo(branch="master")

        filepath = self.mwcore_dir + "/README.md"
        text = (
            "Added by setup_rule_1b_mw_code_to_master_success master on "
            + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        )
        commit_msg = "T362987 Rule 1b test: master MW code change"

        master_change_url = self.change_and_push_file(
            self.mwcore_dir, "master", filepath, text, commit_msg
        )
        master_change_id = self.get_change_id(self.mwcore_dir)
        self.approve_and_wait_for_merge(master_change_url)

        # Get the master commit and cherry-pick it to wmf branch
        self.git_command(self.mwcore_dir, ["checkout", "master"])
        self.git_command(self.mwcore_dir, ["pull", "origin", "master"])
        master_commit = subprocess.check_output(
            ["git", "-C", self.mwcore_dir, "rev-parse", "HEAD"], text=True
        ).strip()

        # Cherry-pick to wmf branch (simulating normal train process)
        self.setup_core_repo(branch=self.mwbranch)
        self.git_command(
            self.mwcore_dir, ["cherry-pick", "-x", "-Xtheirs", master_commit]
        )

        wmf_change_url = self.push_and_collect_url(self.mwcore_dir, self.mwbranch)
        self.approve_and_wait_for_merge(wmf_change_url)
        self.git_command(self.mwcore_dir, ["reset", "--hard", "HEAD~1"])

        # Run scap prep auto to ensure the cherry-pick is fully integrated
        # Ideally this would be replaced with something that just updates the
        # relevant repo in the staging dircectory.  Scap prep auto is too heavyweight.
        subprocess.check_call(["scap", "prep", "auto"])

        # Create extension change that depends on the master change
        self.setup_visualeditor_repo()

        filepath2 = self.mwvisualeditor_dir + "/README.md"
        text2 = (
            "Added by setup_rule_1b_mw_code_to_master_success extension on "
            + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        )
        commit_msg2 = f"T362987 Rule 1b test: extension change depending on master MW code\n\nDepends-On: {master_change_id}"

        dependent_change_url = self.change_and_push_file(
            self.mwvisualeditor_dir, self.mwbranch, filepath2, text2, commit_msg2
        )
        self.git_command(self.mwvisualeditor_dir, ["reset", "--hard", "HEAD~1"])

        return [master_change_url, wmf_change_url, dependent_change_url]

    def setup_rule_1b_mw_code_to_master_failure(self):
        """
        T362987 Rule 1b failure: Set up scenario where MW code change depends on master MW code
        that is NOT present in the target branch.

        Returns a list of change URLs: [master_change_url, dependent_change_url].
        """
        # Create master change in mediawiki/core but don't cherry-pick it
        self.setup_core_repo(branch="master")

        filepath = self.mwcore_dir + "/README.md"
        text = (
            "Added by setup_rule_1b_mw_code_to_master_failure master on "
            + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        )
        commit_msg = "T362987 Rule 1b test: master MW code change NOT in target branch"

        master_change_url = self.change_and_push_file(
            self.mwcore_dir, "master", filepath, text, commit_msg
        )
        master_change_id = self.get_change_id(self.mwcore_dir)
        self.approve_and_wait_for_merge(master_change_url)
        self.git_command(self.mwcore_dir, ["reset", "--hard", "HEAD~1"])

        # Importantly: Do NOT cherry-pick to wmf branch - this causes the validation failure

        # Create extension change that depends on the master change
        self.setup_visualeditor_repo()

        filepath2 = self.mwvisualeditor_dir + "/README.md"
        text2 = (
            "Added by setup_rule_1b_mw_code_to_master_failure extension on "
            + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        )
        commit_msg2 = f"T362987 Rule 1b test: extension change depending on undeployed master MW code\n\nDepends-On: {master_change_id}"

        dependent_change_url = self.change_and_push_file(
            self.mwvisualeditor_dir, self.mwbranch, filepath2, text2, commit_msg2
        )
        self.git_command(self.mwvisualeditor_dir, ["reset", "--hard", "HEAD~1"])

        return [master_change_url, dependent_change_url]

    def setup_rule_2_depends_on_config(self):
        """
        T362987 Rule 2: Set up scenario where MW code change depends on config repo.
        This should do no extra validation.

        Returns a list of change URLs: [config_change_url, mw_change_url].
        """
        # Create config change
        self.setup_mediawiki_config_repo()

        readme_path = self.mwconfig_dir + "/README"
        text = "Added by setup_rule_2_depends_on_config on " + datetime.now().strftime(
            "%Y-%m-%d %H:%M:%S"
        )
        commit_msg = "T362987 Rule 2 test: config change"

        config_change_url = self.change_and_push_file(
            self.mwconfig_dir, "train-dev", readme_path, text, commit_msg
        )
        config_change_id = self.get_change_id(self.mwconfig_dir)
        self.approve_and_wait_for_merge(config_change_url)
        self.git_command(self.mwconfig_dir, ["reset", "--hard", "HEAD~1"])

        # Create MW code change that depends on config
        self.setup_core_repo(branch=self.mwbranch)

        filepath = self.mwcore_dir + "/README.md"
        text2 = (
            "Added by setup_rule_2_depends_on_config MW change on "
            + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        )
        commit_msg2 = f"T362987 Rule 2 test: MW code change depending on config\n\nDepends-On: {config_change_id}"

        mw_change_url = self.change_and_push_file(
            self.mwcore_dir, self.mwbranch, filepath, text2, commit_msg2
        )
        self.git_command(self.mwcore_dir, ["reset", "--hard", "HEAD~1"])

        return [config_change_url, mw_change_url]

    def setup_rule_3a_config_to_non_master_mw_code(self):
        """
        T362987 Rule 3a: Set up scenario where config change depends on non-master MW code.
        This should do no extra validation.

        Returns a list of change URLs: [mw_change_url, config_change_url].
        """
        # Create MW code change on wmf branch
        self.setup_core_repo(branch=self.mwbranch)

        filepath = self.mwcore_dir + "/README.md"
        text = (
            "Added by setup_rule_3a_config_to_non_master_mw_code MW change on "
            + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        )
        commit_msg = "T362987 Rule 3a test: MW code change on wmf branch"

        mw_change_url = self.change_and_push_file(
            self.mwcore_dir, self.mwbranch, filepath, text, commit_msg
        )
        mw_change_id = self.get_change_id(self.mwcore_dir)
        self.approve_and_wait_for_merge(mw_change_url)
        self.git_command(self.mwcore_dir, ["reset", "--hard", "HEAD~1"])

        # Create config change that depends on MW code
        self.setup_mediawiki_config_repo()

        readme_path = self.mwconfig_dir + "/README"
        text2 = (
            "Added by setup_rule_3a_config_to_non_master_mw_code config on "
            + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        )
        commit_msg2 = f"T362987 Rule 3a test: config change depending on non-master MW code\n\nDepends-On: {mw_change_id}"

        config_change_url = self.change_and_push_file(
            self.mwconfig_dir, "train-dev", readme_path, text2, commit_msg2
        )
        self.git_command(self.mwconfig_dir, ["reset", "--hard", "HEAD~1"])

        return [mw_change_url, config_change_url]

    def rule_1b_target_branch_backport_fails(self, change_url) -> int:
        """
        Attempts to backport a MW code change that depends on master MW code
        not present in the target branch.

        Returns the exit status of scap backport.
        """
        child = self._start_scap_backport([change_url])
        child.expect(r"Master dependencies must be cherry-picked to the target branch")
        child.wait()
        return child.exitstatus

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

    def update_gitmodules_for_unusual_submodule_change(self):
        """
        Update the commit pointer for VisualEditor/VisualEditor in its parent submodule,
        mediawiki/extensions/VisualEditor.
        """
        announce(
            "Committing and backporting .gitmodules update in extensions/VisualEditor"
        )

        self.git_command(self.mwvisualeditor_dir, ["fetch", "origin", self.mwbranch])
        self.git_command(
            self.mwvisualeditor_dir,
            ["checkout", "--force", "-B", self.mwbranch, f"origin/{self.mwbranch}"],
        )

        self.git_commit(self.mwvisualeditor_dir, "update ve submodule link", "lib/ve")
        self.scap_backport(
            [self.push_and_collect_url(self.mwvisualeditor_dir, self.mwbranch)]
        )

    def setup_unusual_submodule_path_change(self):
        """
        Creates a change in the VisualEditor/VisualEditor submodule (which has
        the unusual path extensions/VisualEditor/lib/ve), and pushes to gerrit.

        Returns the change url.
        """
        ve_dir = self.mwvisualeditor_dir + "/lib/ve"

        # Make sure the checkout is up-to-date before creating a change
        announce(f"Resetting {ve_dir} to latest {self.mwbranch}")

        self.git_command(ve_dir, ["fetch", "origin", self.mwbranch])
        self.git_command(
            ve_dir,
            ["checkout", "--force", "-B", self.mwbranch, f"origin/{self.mwbranch}"],
        )

        readme_path = ve_dir + "/README.md"
        announce(f"Creating a change to {readme_path}")
        text = (
            "\nAdded by setup_unusual_submodule_path_change on "
            + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        )
        return self.change_and_push_file(
            ve_dir,
            self.mwbranch,
            readme_path,
            text,
            "setup_unusual_submodule_path_change: add line to bottom of README.md",
        )

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
        self.backports_test_helper.setup_visualeditor_repo()
        change_url = self.backports_test_helper.setup_unusual_submodule_path_change()
        self.backports_test_helper.scap_backport([change_url])
        self.backports_test_helper.update_gitmodules_for_unusual_submodule_change()
        # Cleanup
        self.backports_test_helper.setup_visualeditor_repo()

        self.backports_test_helper.scap_backport(["--revert", change_url])

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

    def test_depends_on_self_cycle_detected(self):
        announce("Testing Depends-On self-cycle detection (T408675)")

        change_url = self.backports_test_helper.setup_depends_on_self_cycle()
        child = self.backports_test_helper._start_scap_backport([change_url])
        child.expect(r"A dependency cycle was detected for change \d+!")
        child.wait()
        self.assertNotEqual(child.exitstatus, 0)

    def test_depends_on_three_commit_cycle_detected(self):
        announce("Testing Depends-On three-commit cycle detection (T408675)")

        (
            change_url_a,
            change_url_b,
            change_url_c,
        ) = self.backports_test_helper.setup_depends_on_three_commit_cycle()
        # Try to backport C, which will traverse C→B→C forming a cycle
        child = self.backports_test_helper._start_scap_backport([change_url_c])
        child.expect(r"A dependency cycle was detected for change \d+!")
        child.wait()
        self.assertNotEqual(child.exitstatus, 0)

    def test_depends_on_different_branch(self):
        announce(
            "Testing: Depends-On to different branch triggers warning for irrelevant dependencies"
        )

        child = self.backports_test_helper.depends_with_nonprod_branch()
        # T365146 says "If a change in the dependency trail specifies a Depends-On clause
        # but no relevant dependencies can be determined for it, scap should prompt with a
        # confirmation warning to let the operator know about the anomalous situation"

        # First, expect the warning message about irrelevant dependencies
        child.expect_exact("Warnings found:")
        child.expect("but none were deemed relevant by the dependency analysis rules")
        child.expect_exact("Continue with backport anyway? [y/N]:")
        child.sendline("y")

        # After handling the warning, proceed to the normal backport confirmation
        child.expect(r"Backport the changes\? \[y/N\]:")
        child.sendline("y")
        self.backports_test_helper._scap_backport_interact(child)

    # Verifies T345304
    def test_depends_on_abandoned(self):
        announce(
            "Testing abandoned Depends-On changes. Different branches should be ignored"
        )

        change_urls = self.backports_test_helper.setup_dependency_chain(
            style="Depends-On", use_different_branches=True
        )

        # First change and middle change are in the same branch and middle change Depends-On first change. Abandoning
        # the first change and trying to backport the middle one should make the backport fail
        abandoned_change_number = change_urls[0].split("/")[-1]
        self.backports_test_helper.abandon_change(change_urls[0])
        backported_change_number = change_urls[1].split("/")[-1]
        child = self.backports_test_helper._start_scap_backport([change_urls[1]])
        child.expect_exact(
            f"Change '{abandoned_change_number}' has been abandoned! Change is pulled by the following dependency"
            f" chain: {backported_change_number} -> {abandoned_change_number}"
        )
        child.wait()

        # Now we abandon the middle change. Last change Depends-On middle change but they belong to different branches,
        # With T365146, cross-branch dependencies are automatically [NOT Relevant] and ignored,
        # but the abandoned change is still detected in the dependency chain and causes failure
        middle_change_number = change_urls[1].split("/")[-1]
        self.backports_test_helper.abandon_change(change_urls[1])
        backported_change_number = change_urls[2].split("/")[-1]
        child = self.backports_test_helper._start_scap_backport([change_urls[2]])
        # With T365146, the system detects the abandoned change in the dependency chain and fails
        child.expect_exact(
            f"Change '{abandoned_change_number}' has been abandoned! Change is pulled by the following dependency"
            f" chain: {backported_change_number} -> {middle_change_number} -> {abandoned_change_number}"
        )
        child.wait()

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

    def test_nonexistent_change_number(self):
        child = self.backports_test_helper._start_scap_backport(["0"])
        child.expect_exact("Change '0' not found")
        child.wait()
        self.assertNotEqual(child.exitstatus, 0)

    # T388025
    def test_depends_on_master_included_in_production(self):
        """
        Test scenario where a master branch code change is included in production
        branches via cherry-pick, and a config change depends on it. Scap should
        proceed normally since the dependency is satisfied through the cherry-picked
        change in the production branch.
        """
        announce(
            "Testing master change included via cherry-pick allows normal backport"
        )

        change_urls = (
            self.backports_test_helper.setup_depends_on_master_included_in_production()
        )

        self.backports_test_helper.scap_backport(change_urls)

    def test_depends_on_master_not_deployed_to_all_branches(self):
        """
        Test scenario where a master branch MediaWiki code change exists but
        is NOT present in all deployable branches, and a config change depends
        on it. This should cause scap backport to fail with an error about
        the master dependency not being deployed to all active branches.
        """
        announce(
            "Testing master change not deployed to all branches causes backport failure"
        )

        change_urls = (
            self.backports_test_helper.setup_depends_on_master_not_deployed_to_all_branches()
        )

        # This backport should fail because the master dependency is not present
        # in all deployable branches
        child = self.backports_test_helper._start_scap_backport(
            [change_urls[1]]  # Only try to backport the config change
        )
        child.expect(r"Master dependencies must be deployed to all active branches")
        child.wait()

        # Verify that the failure was due to the master dependency validation
        # The exit status should be non-zero indicating failure
        self.assertNotEqual(child.exitstatus, 0)

    # T387798
    def test_depends_on_cross_repo_same_changeid(self):
        """
        Test scenario where a config change depends on a Change-Id that exists
        in both master and wmf branches, and both the wmf branch change and
        config change are being deployed simultaneously. Scap should not warn
        about the master branch version when the wmf version is also being
        deployed in the same command.
        """
        announce(
            "Testing cross-repo dependency with same Change-Id in multiple branches"
        )

        change_urls = (
            self.backports_test_helper.setup_cross_repo_same_changeid_dependency()
        )

        # Deploy only the wmf branch change and config change (not the master change)
        # This reproduces T387798: config depends on Change-Id that exists in both
        # master and wmf, but we're only deploying the wmf version + config
        # Should proceed without warnings about master branch version
        wmf_and_config_changes = [change_urls[1], change_urls[2]]  # Skip master change
        self.backports_test_helper.scap_backport(wmf_and_config_changes)

    def test_depends_on_chain_all_commits(self):
        """
        Test scenario with a chain of 3 commits where each commit beyond the first
        has a Depends-On pointing to the prior commit. All commits are pushed as a
        single relation chain stack and then backported to ensure scap doesn't get
        confused by the combination of Depends-On footers and relation chain structure.
        """
        announce("Testing Depends-On chain pushed as relation chain stack")

        # Create a chain of 3 commits with both Depends-On relationships AND pushed as relation chain
        change_urls = self.backports_test_helper.setup_depends_on_relation_chain()

        # Backport all 3 commits together to test that scap correctly handles
        # the combination of Depends-On footers and relation chain structure
        announce(
            f"Backporting all 3 commits in the Depends-On relation chain: {change_urls}"
        )
        self.backports_test_helper.scap_backport(change_urls)

    def test_rule_1a_mw_code_depends_on_non_master_mw_code(self):
        """Test T362987 Rule 1a: MW code change depending on non-master MW code should do no extra validation."""
        announce("Testing Rule 1a: MW code → non-master MW code (no extra validation)")

        change_urls = (
            self.backports_test_helper.setup_rule_1a_mw_code_to_non_master_mw_code()
        )

        # Should succeed without any extra validation
        self.backports_test_helper.scap_backport(change_urls)

    def test_rule_1b_mw_code_depends_on_master_success(self):
        """Test T362987 Rule 1b success: MW code depending on master MW code present in target branch."""
        announce(
            "Testing Rule 1b success: MW code → master MW code (target branch validation passes)"
        )

        change_urls = (
            self.backports_test_helper.setup_rule_1b_mw_code_to_master_success()
        )

        # Only backport the dependent extension change (not the master changes)
        # The master dependency should be detected and validation should pass because it's in target branch
        self.backports_test_helper.scap_backport([change_urls[2]])

    def test_rule_1b_mw_code_depends_on_master_failure(self):
        """Test T362987 Rule 1b failure: MW code depending on master MW code missing from target branch."""
        announce(
            "Testing Rule 1b failure: MW code → master MW code (target branch validation fails)"
        )

        change_urls = (
            self.backports_test_helper.setup_rule_1b_mw_code_to_master_failure()
        )

        # Should fail because master dependency is not present in target branch
        child = self.backports_test_helper._start_scap_backport([change_urls[1]])
        child.expect(r"Master dependencies must be cherry-picked to the target branch")
        child.wait()
        self.assertNotEqual(child.exitstatus, 0)

    def test_rule_2_depends_on_config_repo(self):
        """Test T362987 Rule 2: Dependencies on config repo should do no extra validation."""
        announce("Testing Rule 2: Dependencies on config repo (no extra validation)")

        change_urls = self.backports_test_helper.setup_rule_2_depends_on_config()

        # Should succeed without any extra validation
        self.backports_test_helper.scap_backport(change_urls)

    def test_rule_3a_config_depends_on_non_master_mw_code(self):
        """Test T362987 Rule 3a: Config change depending on non-master MW code should do no extra validation."""
        announce("Testing Rule 3a: Config → non-master MW code (no extra validation)")

        change_urls = (
            self.backports_test_helper.setup_rule_3a_config_to_non_master_mw_code()
        )

        # Should succeed without any extra validation
        self.backports_test_helper.scap_backport(change_urls)
