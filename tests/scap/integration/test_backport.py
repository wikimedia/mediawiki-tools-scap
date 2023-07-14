import logging
import os
import re
import subprocess
import sys
import time
from datetime import datetime
import pexpect
import unittest

import scap.cli
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
    mwbranch = None
    gerrit = None
    gerrit_url = "http://gerrit.traindev:8080"
    gerrit_domain = "gerrit.traindev"

    def setup(self):
        self.gerrit = GerritSession(url=self.gerrit_url)

        if not os.path.exists(self.homedir + "/new-version"):
            logging.info("Bootstrapping with ~/train first")
            subprocess.check_call(self.homedir + "/train")

        announce("Running scap prep auto to clean up /srv/mediawiki-staging")
        subprocess.check_call(["scap", "prep", "auto"])
        scap.utils.mkdir_p(self.workdir)

        with open(self.homedir + "/new-version", "r") as f:
            self.mwbranch = "wmf/" + f.read().rstrip()

        self.setup_mediawiki_config_repo()
        self.setup_core_repo()
        self.setup_growthexperiments_repo()

        self.setup_core_gitmodules()

    def setup_mediawiki_config_repo(self, branch="train-dev"):
        self.git_clone(branch, self.gerrit_url + "/operations/mediawiki-config", self.mwconfig_dir)

    def setup_core_repo(self, branch=None):
        self.git_clone(branch if branch else self.mwbranch, self.gerrit_url + "/mediawiki/core", self.mwcore_dir)

    def setup_growthexperiments_repo(self):
        self.git_clone(self.mwbranch, self.gerrit_url + "/mediawiki/extensions/GrowthExperiments",
                       self.mwgrowthexperiments_dir)

    def setup_core_gitmodules(self):
        self.git_command(self.mwcore_dir, ["config", "-f", ".gitmodules", "submodule.extensions/GrowthExperiments.url",
                                           self.gerrit_url + "/mediawiki/extensions/GrowthExperiments"])
        if subprocess.check_output(["git", "-C", self.mwcore_dir, "status", "--porcelain", ".gitmodules"]):
            announce("Tweaking submodule.extensions/GrowthExperiments.url in %s/.gitmodules" % self.mwcore_dir)
            self.git_commit(self.mwcore_dir, "scap backport testing: set URL for extensions/GrowthExperiments to %s"
                            % self.gerrit_domain, ".gitmodules")
            change_url = self.push_and_collect_url(self.mwcore_dir, self.mwbranch)
            self.git_command(self.mwcore_dir, ["reset", "--hard", "HEAD^"])
            self.scap_backport([change_url])
            self.git_command(self.mwcore_dir, ["pull"])
            announce(".gitmodules tweak successful")

    def git_clone(self, branch, repo, path):
        """ clones a repo """
        announce("Updating checkout of %s (%s) in %s" % (repo, branch, path))
        if not os.path.isdir(path):
            subprocess.check_call(["git", "clone", "-b", branch, repo, path])
            # install gerrit commit-msg hook
            subprocess.check_call(["scp", "-p", "-P", "29418", "traindev@%s:hooks/commit-msg" % self.gerrit_domain,
                                  "%s/.git/hooks/" % path])

        self.git_command(path, ["fetch", "-q", "origin", branch])
        self.git_command(path, ["reset", "-q", "--hard", "FETCH_HEAD"])

    def push_and_collect_urls(self, repo, branch) -> list:
        """ Returns a list of change urls after pushing to gerrit """
        git_response = subprocess.check_output(["git", "-C", repo, "push", "origin", "HEAD:refs/for/%s" % branch],
                                               text=True, stderr=subprocess.STDOUT)
        return re.findall(r'remote:\s+(%s/c/\S+)' % self.gerrit_url,
                          git_response)

    def push_and_collect_url(self, repo, branch):
        """ returns the change url after pushing to gerrit """

        change_urls = self.push_and_collect_urls(repo, branch)

        if len(change_urls) != 1:
            raise Exception(f"Expected one change_url but got {change_urls}")

        change_url = change_urls[0]
        logging.info("change url is " + change_url)
        return change_url

    def get_change_detail(self, change_url) -> dict:
        """ Returns a dictionary representing the specified change """
        change_number = self.gerrit.change_number_from_url(change_url)
        return self.gerrit.change_detail(change_number).get()

    def approve_change(self, change_url):
        """ approve the change """
        change_number = self.gerrit.change_number_from_url(change_url)
        current_rev = self.gerrit.change_detail(change_number).get()['current_revision']
        logging.info("Approving change %s" % change_number)
        subprocess.check_call(["ssh", "-p", "29418", self.gerrit_domain, "gerrit review", current_rev, "--code-review",
                               "+2"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def approve_and_wait_for_merge(self, change_url):
        """ approves the change and returns after change has merged  """
        self.approve_change(change_url)
        logging.info("Waiting for %s to merge...", change_url)

        while True:
            detail = self.get_change_detail(change_url)
            status = detail.get('status')
            mergeable = detail.get('mergeable')

            if status == "MERGED":
                logging.info("%s has merged", change_url)
                return

            if mergeable is False:
                raise RuntimeError(f"{change_url} is not mergeable (probably due to conflict)")

            time.sleep(2)

    def scap_backport(self, args_list):
        """ Run scap backport and perform a standard interaction with it """
        child = self._start_scap_backport(args_list)
        self._scap_backport_interact(child, args_list)

    def _pexpect_spawn(self, cmd, *args) -> pexpect.spawn:
        return pexpect.spawn(cmd, *args, logfile=sys.stdout, encoding='utf-8', timeout=120)

    def _start_scap_backport(self, args_list) -> pexpect.spawn:
        cmd = ["scap", "backport", "--stop-before-sync"] + args_list
        logging.info("Running %s" % ' '.join(cmd))

        return self._pexpect_spawn(cmd[0], cmd[1:])

    def _scap_backport_interact(self, child: pexpect.spawn, args_list=[]):
        if "--revert" in args_list:
            child.expect_exact('Please supply a reason for revert')
            child.sendline('')  # Accept the default revert reason

            # scap backport --revert performs an initial scap prep auto to reset
            # the workspace so we watch for that here to avoid confusing the
            # next stage.
            child.expect_exact('Started scap prep auto')
            child.expect_exact('Finished scap prep auto')

        while True:
            index = child.expect_exact(['Backport the changes? (y/n):',
                                        'Finished scap prep auto'])
            if index == 0:
                child.sendline('y')
            elif index == 1:
                break
            else:
                raise Exception("This should never happen")

        logging.info("Waiting for scap backport to terminate")
        child.wait()
        if child.exitstatus != 0:
            raise RuntimeError(' '.join(["scap", "backport"] + args_list) + f" exited with status {child.exitstatus}")

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
            with open(filepath, 'r') as f:
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
        return re.search(r"(?m)Change-Id:.+$", gitcmd("-C", repo, "log", "-1")).group().split(" ")[1]

    def change_and_push_file(self, repo, branch, file, text, commit_message, append=True):
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
        text = f"\nAdded by setup_config_change({mode}, {context}) on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        change_url = self.change_and_push_file(self.mwconfig_dir, branch, readme_path, text, f"setup_config_change({context})")
        self.git_command(self.mwconfig_dir, ["reset", "--hard", "HEAD~1"])

        if mode == "already_merged":
            logging.info("Merging the change before scap backport to verify that this is okay")
            self.approve_and_wait_for_merge(change_url)

        return change_url

    def merge_commit_backport(self):
        """ makes and backports a change that should result in a merge commit """
        developers_path = self.mwcore_dir + "/DEVELOPERS.md"
        text = "Added by merge_commit_backport (1) on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        change_url1 = self.change_and_push_file(self.mwcore_dir, self.mwbranch, developers_path, text,
                                                "merge_commit_backport (1): add a line to top of DEVELOPERS.md", False)

        self.git_command(self.mwcore_dir, ["reset", "--hard", "HEAD~1"])
        text = "\nAdded by merge_commit_backport (2) on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        change_url2 = self.change_and_push_file(self.mwcore_dir, self.mwbranch, developers_path, text,
                                                "merge_commit_backport (2): add line to bottom of DEVELOPERS.md\n "
                                                "This is expected to result in a merge commit")

        self.scap_backport([change_url1])
        self.scap_backport([change_url2])

    # There are two types of dependency chains that can be set up.  One occurs when you push
    # multiple commits at once.  We call this a relation chain.  The other type is established
    # using the Depends-On footer in commit messages.  We call this the depends-on chain.  The
    # chains are set up differently but scap backport must behave the same way for either type.

    def setup_dependency_chain(self, *, style=None) -> list:
        """
        Constructs a chain of 3 commits in one of two ways depending on
        'style', which must be "relation" or "Depends-On".  The chain is pushed to
        Gerrit and a list of change URLs is returned.

        The first commit in the chain is first created on the master branch (and
        pushed to Gerrit) and then cherry-picked into the train branch (wmf/*),
        simulating a typical backport workflow.  This helps exercise T324275/T323277.

        """
        allowed_styles = ["relation", "Depends-On"]

        if style not in allowed_styles:
            raise ValueError(f"style must be one of {allowed_styles}")

        # Freshen
        self.setup_core_repo(branch="master")

        files = ["COPYING", "CREDITS", "FAQ"]
        change_urls = []

        def maybe_push_commit():
            if style == "Depends-On":
                change_urls.append(self.push_and_collect_url(self.mwcore_dir, self.mwbranch))
                self.git_command(self.mwcore_dir, ["reset", "--hard", "HEAD~1"])

        # Prepare the first commit in the master branch first.
        file = files[0]
        path = os.path.join(self.mwcore_dir, file)

        text = "Added by setup_dependency_chain (1) on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        commit_msg = f"{file}: setup_dependency_chain (1)"
        self.change_and_push_file(self.mwcore_dir, "master", path, text, commit_msg)
        commit = subprocess.check_output(["git", "-C", self.mwcore_dir, "rev-parse", "HEAD"], text=True).strip()
        change_id = self.get_change_id(self.mwcore_dir)

        # Now cherry pick that commit as the first in a chain for the train branch.
        self.setup_core_repo()
        self.git_command(self.mwcore_dir, ["cherry-pick", "-x", "-Xtheirs", commit])
        maybe_push_commit()

        # Then stack up two more commits on top
        for file in files[1:]:
            path = os.path.join(self.mwcore_dir, file)
            text = "Added by setup_dependency_chain (2) on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            commit_msg = f"{file}: setup_dependency_chain (2)"

            if style == "Depends-On":
                commit_msg += f"\n\nDepends-On: {change_id}"

            self.write_to_file(path, text, append=True)
            self.git_commit(self.mwcore_dir, commit_msg, path)
            change_id = self.get_change_id(self.mwcore_dir)
            maybe_push_commit()

        if style == "relation":
            change_urls = self.push_and_collect_urls(self.mwcore_dir, self.mwbranch)

        return change_urls

    def dependency_chain_backport_fails(self, change_urls) -> int:
        """
        Attempts to backport one or more commits with unmerged dependencies (of whatever type).

        Returns the exit status of scap backport
        """

        child = self._start_scap_backport(change_urls)
        child.expect(r"Change '\d+' has dependencies '\[.*\]', which are not merged or scheduled for backport")
        child.wait()
        return child.exitstatus

    def depends_on_backport_wrong_branch(self) -> pexpect.spawn:
        """ Backports with a dependency in a non-production branch"""

        # Make a configuration change on the wrong branch.  In the case
        # of train-dev (where these tests run), the right branch is "train-dev"
        # and "master" is the wrong one.
        self.setup_mediawiki_config_repo(branch="master")

        readme_path = self.mwconfig_dir + "/README"
        text = "Added by depends_on_backport_wrong_branch on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        change_url = self.change_and_push_file(self.mwconfig_dir, "master", readme_path, text, "A sample config change made on the wrong branch")
        change_id = self.get_change_id(self.mwconfig_dir)
        self.git_command(self.mwconfig_dir, ["reset", "--hard", "HEAD~1"])

        # And deploy it, answering 'y' to the confirmation of the weird situation.
        child = self._start_scap_backport([change_url])
        child.expect_exact(' not found in any deployed wikiversion. Deployed wikiversions: ')
        child.expect_exact('Continue with Backport? (y/n):')
        child.sendline('y')
        self._scap_backport_interact(child)

        # Switch back to the right branch, train-dev, and make a configuration change
        # which has a Depends-On pointing to the "wrong branch" change.
        self.setup_mediawiki_config_repo()
        text = "Added by depends_on_backport_wrong_branch on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        change_url = self.change_and_push_file(self.mwconfig_dir, "train-dev", readme_path, text,
                                               "Depends-On references commit in wrong branch\n\nDepends-On: %s" % change_id, False)
        self.git_command(self.mwconfig_dir, ["reset", "--hard", "HEAD~1"])

        return self._start_scap_backport([change_url])

    def setup_extension_change(self):
        """ Creates an extension change and pushes to Gerrit.  This will result in
            a submodule update commit.  The local commit is discarded.

            Returns the change url.

            FIXME: it might be interesting to also test for submodule _and_ merge commit
        """
        # Freshen up
        self.setup_growthexperiments_repo()

        copying_path = self.mwgrowthexperiments_dir + "/COPYING"
        text = "\nAdded by setup_extension_change on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        change_url = self.change_and_push_file(self.mwgrowthexperiments_dir, self.mwbranch, copying_path, text,
                                               "setup_extension_change: add line to bottom of COPYING")
        self.git_command(self.mwconfig_dir, ["reset", "--hard", "HEAD~1"])
        return change_url

    def extension_revert(self, change_url):
        """ reverts a change """
        self.scap_backport(["--revert", change_url])
        return subprocess.check_output(["git", "-C", self.mwgrowthexperiments_dir, "rev-list", "@{u}..HEAD", "--count"],
                                       text=True).strip()

    def backport_with_extra_commits(self):
        """backports a change when there are unexpected new commits merged to the branch"""
        # create, push, and merge an operations/mediawiki-config commit
        self.setup_config_change("already_merged", context="backport_with_extra_commits")

        change_url = self.setup_extension_change()
        return self._pexpect_spawn("scap backport --stop-before-sync %s" % change_url)


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
            change_url = self.backports_test_helper.setup_config_change(mode, context=f"test_config_changes({mode})")
            self.backports_test_helper.scap_backport([change_url])

    def test_merge_commits(self):
        announce("Testing mediawiki/core changes (two independent commits resulting in a merge commit)")
        self.backports_test_helper.merge_commit_backport()

    def test_extension_and_revert(self):
        announce("Testing mediawiki/extensions/GrowthExperiments change")
        change_url = self.backports_test_helper.setup_extension_change()
        self.backports_test_helper.scap_backport([change_url])
        announce("Testing mediawiki/extensions/GrowthExperiments backport --revert change")
        self.assertEqual(self.backports_test_helper.extension_revert(change_url), '1')

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
            exit_status = self.backports_test_helper.dependency_chain_backport_fails(urls)
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
        change_urls = self.backports_test_helper.setup_dependency_chain(style="relation")
        self._test_dependencies("relation", change_urls)

    def test_depends_on_chain(self):
        change_urls = self.backports_test_helper.setup_dependency_chain(style="Depends-On")
        self._test_dependencies("Depends-On", change_urls)

    def test_depends_on_non_production_branch(self):
        announce("Testing warning occurs when Depends-On commit isn't in a production branch")
        child = self.backports_test_helper.depends_on_backport_wrong_branch()
        child.expect_exact('which are not scheduled for backport or included in any mediawiki production branch')
        child.expect_exact('Continue with Backport? (y/n):')
        child.sendline('n')

    def test_extra_commit_confirmation(self):
        announce("Testing whether confirmation is requested if extra commits are pulled")
        child = self.backports_test_helper.backport_with_extra_commits()
        child.expect_exact('Backport the changes? (y/n): ')
        child.sendline('y')

        child.expect_exact('The following are unexpected commits pulled from origin for /srv/mediawiki-staging')
        child.expect_exact('Would you like to see the diff? (y/n): ')
        child.sendline('y')

        child.expect_exact('There were unexpected commits pulled from origin for /srv/mediawiki-staging.')
        child.expect_exact('Continue with backport? (y/n): ')

        # Go ahead and "deploy" the extra commit to avoid confusing other tests by leaving
        # a merged-but-unpulled commit
        child.sendline('y')

        self.backports_test_helper._scap_backport_interact(child)
