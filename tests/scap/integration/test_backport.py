import logging
import os
import re
import subprocess
import time
from datetime import datetime
import pexpect
import unittest

import scap.cli
import scap.utils
from scap.plugins.gerrit import GerritSession
from scap.runcmd import gitcmd


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

        self.git_clone("train-dev", self.gerrit_url + "/operations/mediawiki-config", self.mwconfig_dir)
        self.git_clone(self.mwbranch, self.gerrit_url + "/mediawiki/core", self.mwcore_dir)
        self.git_clone(self.mwbranch, self.gerrit_url + "/mediawiki/extensions/GrowthExperiments",
                       self.mwgrowthexperiments_dir)

        self.setup_core_gitmodules()

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

    def push_and_collect_url(self, repo, branch):
        """ returns the change url after pushing to gerrit """
        git_response = subprocess.check_output(["git", "-C", repo, "push", "origin", "HEAD:refs/for/%s" % branch],
                                               text=True, stderr=subprocess.STDOUT)
        change_url = re.findall(r'remote:\s+(%s/c/\S+)' % self.gerrit_url,
                                git_response)[0]
        logging.info("change url is " + change_url)
        return change_url

    def get_change_status(self, change_url):
        """ gets the status of a change """
        change_number = self.gerrit.change_number_from_url(change_url)
        change = self.gerrit.change_detail(change_number).get()
        return change['status']

    def change_has_merged(self, change_url):
        """ whether the change has merged """
        if self.get_change_status(change_url) == "MERGED":
            return True
        return False

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

        while not self.change_has_merged(change_url):
            time.sleep(2)
        logging.info("%s has merged", change_url)

    def scap_backport(self, args_list):
        logging.info("Running scap backport --yes --stop-before-sync %s" % ' '.join(args_list))
        subprocess.check_call(["scap", "backport", "-Dlog_test:True", "--yes", "--stop-before-sync"] + args_list)

    def write_to_file(self, filepath, message, append=True):
        if append:
            with open(filepath, "a") as f:
                f.writelines(message)
        else:
            with open(filepath, 'r') as f:
                contents = f.read()
            with open(filepath, "w") as f:
                f.writelines(message + contents)

    def git_command(self, repo, command):
        subprocess.check_call(["git", "-C"] + [repo] + command)

    def git_commit(self, repo, commit_message, filepath):
        command = ["commit", "-m", commit_message, filepath]
        self.git_command(repo, command)

    def get_change_id(self, repo):
        return re.search(r"(?m)Change-Id:.+$", gitcmd("-C", repo, "log", "-1")).group().split(" ")[1]

    def change_and_push_file(self, repo, branch, file, text, commit_message, append=True):
        self.write_to_file(file, text, append)
        self.git_commit(repo, commit_message, file)
        return self.push_and_collect_url(repo, branch)

    def config_backport(self, mode, branch="train-dev"):
        """ makes a change to the operations/config repo and backports """
        readme_path = self.mwconfig_dir + "/README"
        text = "\nscap backport test config change: " + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        change_url = self.change_and_push_file(self.mwconfig_dir, branch, readme_path, text, "config backport test")

        if mode == "already_merged":
            logging.info("Merging the change before scap backport to verify that this is okay")
            self.approve_and_wait_for_merge(change_url)

        self.scap_backport([change_url])

    def merge_commit_backport(self):
        """ makes and backports a change that should result in a merge commit """
        developers_path = self.mwcore_dir + "/DEVELOPERS.md"
        text = "Added by scap backport on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S") + "\n"
        change_url1 = self.change_and_push_file(self.mwcore_dir, self.mwbranch, developers_path, text,
                                                "scap backport testing: add a line to top of DEVELOPERS.md", False)

        self.git_command(self.mwcore_dir, ["reset", "--hard", "HEAD~1"])
        text = "\nAdded by scap backport on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        change_url2 = self.change_and_push_file(self.mwcore_dir, self.mwbranch, developers_path, text,
                                                "scap backport testing: add line to bottom of DEVELOPERS.md\n "
                                                "This is expected to result in a merge commit")

        self.scap_backport([change_url1])
        self.scap_backport([change_url2])

    def setup_relation_chains(self):
        """ creates relation chain changesets and returns change urls """
        credits_path = self.mwcore_dir + "/CREDITS"
        text = "Added by scap backport on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S") + "\n"
        change_url1 = self.change_and_push_file(self.mwcore_dir, self.mwbranch, credits_path, text,
                                                "scap backport testing: add line to top of CREDITS", False)

        text = "\nAdded by scap backport on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        change_url2 = self.change_and_push_file(self.mwcore_dir, self.mwbranch, credits_path, text,
                                                "scap backport testing: add line to bottom of CREDITS\n"
                                                "This is expected to result in a relation chain")

        return [change_url1, change_url2]

    def relation_chain_backport_fails(self, change_url):
        """ attempts to backport stored relation chain commits out of order """
        self.scap_backport([change_url])

    def relation_chain_backport_succeeds(self, change_urls_list):
        """ backports stored relation chain urls """
        self.scap_backport(change_urls_list)

    def setup_depends_ons(self):
        """ creates Depends-On changesets and returns change urls """
        faq_path = self.mwcore_dir + "/FAQ"
        text = "Added by scap backport on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S") + "\n"
        change_url1 = self.change_and_push_file(self.mwcore_dir, self.mwbranch, faq_path, text,
                                                "scap backport testing: add line to top of FAQ", False)
        change_id = self.get_change_id(self.mwcore_dir)

        self.git_command(self.mwcore_dir, ["reset", "--hard", "HEAD~1"])

        text = "\nAdded by scap backport on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        change_url2 = self.change_and_push_file(self.mwcore_dir, self.mwbranch, faq_path, text,
                                                "scap backport testing: add Depends-On\n\nDepends-On: %s" % change_id)
        return [change_url1, change_url2]

    def depends_on_backport_fails(self, change_url):
        """ attempts to backport a change which depends on an unmerged change """
        self.scap_backport([change_url])

    def depends_on_backport_succeeds(self, change_urls_list):
        """ backports depends-on changes """
        announce("Attempting simultaneous backport of commit and dependency")
        self.scap_backport(change_urls_list)
        change_id = self.get_change_id(self.mwcore_dir)

        announce("Attempting backport of commit with already merged dependency")
        faq_path = self.mwcore_dir + "/FAQ"
        text = "\nAdded by scap backport on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        change_url1 = self.change_and_push_file(self.mwcore_dir, self.mwbranch, faq_path, text,
                                                "scap backport testing: add Depends-On part 2\n\nDepends-On: %s"
                                                % change_id)
        self.scap_backport([change_url1])

        readme_path = self.mwcore_dir + "/README.md"
        announce("Attempting backport of dependency that shares a change_id")
        text = "Added by scap backport on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S") + "\n"
        change_url1 = self.change_and_push_file(self.mwcore_dir, self.mwbranch, readme_path, text,
                                                "scap backport testing: add line to top of README.md", False)
        commit = subprocess.check_output(["git", "-C", self.mwcore_dir, "rev-parse", "HEAD"], text=True).strip()
        change_id = self.get_change_id(self.mwcore_dir)
        self.git_command(self.mwcore_dir, ["reset", "--hard", "HEAD~1"])
        self.scap_backport([change_url1])

        # cherry-pick change to another branch
        self.git_command(self.mwcore_dir, ["cherry-pick", commit])
        self.push_and_collect_url(self.mwcore_dir, "master")
        self.git_command(self.mwcore_dir, ["reset", "--hard", "HEAD~1"])

        # make a new change depending on cherry-pick
        text = "\nAdded by scap backport on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        change_url3 = self.change_and_push_file(self.mwcore_dir, self.mwbranch, readme_path, text,
                                                "scap backport testing: add Depends-On\n\nDepends-On: %s" % change_id)
        self.scap_backport([change_url1, change_url3])

    def depends_on_backport_wrong_branch(self):
        """ Backports with a dependency in a non-production branch"""
        self.git_command(self.mwconfig_dir, ["checkout", "master"])
        self.git_command(self.mwconfig_dir, ["pull", "origin", "master", "--rebase"])
        readme_path = self.mwconfig_dir + "/README"
        text = "Added by scap backport on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S") + "\n"
        change_url = self.change_and_push_file(self.mwconfig_dir, "master", readme_path, text, "Depends-On testing 3")
        change_id = self.get_change_id(self.mwconfig_dir)
        self.scap_backport([change_url])

        self.git_command(self.mwconfig_dir, ["reset", "--hard", "HEAD~1"])
        self.git_command(self.mwconfig_dir, ["checkout", "train-dev"])
        text = "Added by scap backport on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S") + "\n"
        change_url = self.change_and_push_file(self.mwconfig_dir, "train-dev", readme_path, text,
                                               "Depends-On testing 3\n\nDepends-On: %s" % change_id, False)

        return pexpect.spawn("scap backport --yes --stop-before-sync %s" % change_url)

    def extension_backport(self):
        """ backports an extension change with submodule update commit and returns the change url
            FIXME: it might be interesting to also test for submodule _and_ merge commit
        """
        copying_path = self.mwgrowthexperiments_dir + "/COPYING"
        text = "\nAdded by scap backport on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        change_url = self.change_and_push_file(self.mwgrowthexperiments_dir, self.mwbranch, copying_path, text,
                                               "scap backport testing: add line to bottom of COPYING")
        self.scap_backport([change_url])
        return change_url

    def extension_revert(self, change_url):
        """ reverts a change """
        self.scap_backport(["--revert", change_url])
        return subprocess.check_output(["git", "-C", self.mwgrowthexperiments_dir, "rev-list", "@{u}..HEAD", "--count"],
                                       text=True).strip()


class TestBackports(unittest.TestCase):
    """tests the backports"""
    backports_test_helper = None

    @classmethod
    def setup_class(cls):
        cls.backports_test_helper = BackportsTestHelper()
        cls.backports_test_helper.setup()

    def test_config_changes(self):
        announce("Config change test: %s" % "normal")
        self.backports_test_helper.config_backport("normal")
        announce("Config change test: %s" % "already merged")
        self.backports_test_helper.config_backport("already_merged")

    def test_merge_commits(self):
        announce("Testing mediawiki/core changes (two independent commits resulting in a merge commit)")
        self.backports_test_helper.merge_commit_backport()

    def test_extension_and_revert(self):
        announce("Testing mediawiki/extensions/GrowthExperiments change")
        change_url = self.backports_test_helper.extension_backport()
        announce("Testing mediawiki/extensions/GrowthExperiments backport --revert change")
        self.assertEqual(self.backports_test_helper.extension_revert(change_url), '1')

    def test_relation_chains(self):
        change_urls = self.backports_test_helper.setup_relation_chains()
        announce("Testing the latest commit in a relation chain cannot be backported when relation changes haven't "
                 "been merged")
        with self.assertRaises(Exception) as context:
            self.backports_test_helper.relation_chain_backport_fails(change_urls[1])
            self.assertTrue("not merged or scheduled for backport" in context.exception)

        announce("Testing relation chains can be backported")
        self.backports_test_helper.relation_chain_backport_succeeds(change_urls)

    def test_depends_ons(self):
        change_urls = self.backports_test_helper.setup_depends_ons()
        announce("Testing commit with Depends-On label cannot be backported when Depends-On hasn't been merged")
        with self.assertRaises(Exception) as context:
            self.backports_test_helper.depends_on_backport_fails(change_urls[1])
            self.assertTrue("not merged or scheduled for backport" in context.exception)

        announce("Testing commits with Depends-On label can be backported")
        self.backports_test_helper.depends_on_backport_succeeds(change_urls)

        announce("Testing warning occurs when Depends-On commit isn't in a production branch")
        child = self.backports_test_helper.depends_on_backport_wrong_branch()
        child.expect_exact('Continue with Backport? (y/n): ')
        child.sendline('n')
        self.assertTrue("included in any mediawiki production branch" in child.before.decode("utf-8"))
