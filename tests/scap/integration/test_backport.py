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
    depends_on_change_urls = None
    relation_change_urls = None
    extension_change_url = None

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
        announce("Config change test: %s" % mode)
        readme_path = self.mwconfig_dir + "/README"
        text = "\nscap backport test config change: " + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        change_url = self.change_and_push_file(self.mwconfig_dir, branch, readme_path, text, "config backport test")

        if mode == "already_merged":
            logging.info("Merging the change before scap backport to verify that this is okay")
            self.approve_and_wait_for_merge(change_url)

        self.scap_backport([change_url])

    def merge_commit_backport(self):
        """ makes and backports a change that should result in a merge commit """
        announce("Testing mediawiki/core changes (two independent commits resulting in a merge commit)")
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
        """ creates relation chain changesets and stores change urls """
        credits_path = self.mwcore_dir + "/CREDITS"
        text = "Added by scap backport on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S") + "\n"
        change_url1 = self.change_and_push_file(self.mwcore_dir, self.mwbranch, credits_path, text,
                                                "scap backport testing: add line to top of CREDITS", False)

        text = "\nAdded by scap backport on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        change_url2 = self.change_and_push_file(self.mwcore_dir, self.mwbranch, credits_path, text,
                                                "scap backport testing: add line to bottom of CREDITS\n"
                                                "This is expected to result in a relation chain")

        self.relation_change_urls = [change_url1, change_url2]

    def relation_chain_backport_fails(self):
        """ attempts to backport stored relation chain commits out of order """
        announce("Testing the latest commit in a relation chain cannot be backported alone")
        self.scap_backport([self.relation_change_urls[1]])

    def relation_chain_backport_succeeds(self):
        """ backports stored relation chain urls """
        announce("Testing relation chains can be backported")
        self.scap_backport(self.relation_change_urls)

    def setup_depends_ons(self):
        """ creates Depends-On changesets and stores change urls """
        faq_path = self.mwcore_dir + "/FAQ"
        text = "Added by scap backport on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S") + "\n"
        change_url1 = self.change_and_push_file(self.mwcore_dir, self.mwbranch, faq_path, text,
                                                "scap backport testing: add line to top of FAQ", False)
        change_id = self.get_change_id(self.mwcore_dir)

        self.git_command(self.mwcore_dir, ["reset", "--hard", "HEAD~1"])

        text = "\nAdded by scap backport on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        change_url2 = self.change_and_push_file(self.mwcore_dir, self.mwbranch, faq_path, text,
                                                "scap backport testing: add Depends-On\n\nDepends-On: %s" % change_id)
        self.depends_on_change_urls = [change_url1, change_url2]

    def depends_on_backport_fails(self):
        """ attempts to backport a change which depends on an unmerged change """
        announce("Testing commit with Depends-On cannot be backported alone")
        self.scap_backport([self.depends_on_change_urls[1]])

    def depends_on_backport_succeeds(self):
        """ backports depends-on changes """
        announce("Testing simultaneous merge of commit and dependency")
        self.scap_backport(self.depends_on_change_urls)
        change_id = self.get_change_id(self.mwcore_dir)

        announce("Testing merge of commit with already merged dependency")
        faq_path = self.mwcore_dir + "/FAQ"
        text = "\nAdded by scap backport on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        change_url1 = self.change_and_push_file(self.mwcore_dir, self.mwbranch, faq_path, text,
                                                "scap backport testing: add Depends-On part 2\n\nDepends-On: %s"
                                                % change_id)
        self.scap_backport([change_url1])

        readme_path = self.mwcore_dir + "/README.md"
        announce("Testing dependency that shares a change_id")
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
        """ backports an extension change with submodule update commit and stores the change url
            FIXME: it might be interesting to also test for submodule _and_ merge commit
        """
        announce("Testing mediawiki/extensions/GrowthExperiments change")
        copying_path = self.mwgrowthexperiments_dir + "/COPYING"
        text = "\nAdded by scap backport on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        change_url = self.change_and_push_file(self.mwgrowthexperiments_dir, self.mwbranch, copying_path, text,
                                               "scap backport testing: add line to bottom of COPYING")
        self.scap_backport([change_url])
        self.extension_change_url = change_url

    def extension_revert(self):
        """ reverts a change """
        announce("mediawiki/extensions/GrowthExperiments revert change")
        self.scap_backport(["--revert", self.extension_change_url])
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
        self.backports_test_helper.config_backport("normal")
        self.backports_test_helper.config_backport("already_merged")

    def test_merge_commits(self):
        self.backports_test_helper.merge_commit_backport()

    def test_extension(self):
        self.backports_test_helper.extension_backport()

    def test_relation_chains(self):
        self.backports_test_helper.setup_relation_chains()
        with self.assertRaises(Exception) as context:
            self.backports_test_helper.relation_chain_backport_fails()
            self.assertTrue("not merged or scheduled for backport" in context.exception)

        self.backports_test_helper.relation_chain_backport_succeeds()

    def test_depends_ons(self):
        self.backports_test_helper.setup_depends_ons()
        with self.assertRaises(Exception) as context:
            self.backports_test_helper.depends_on_backport_fails()
            self.assertTrue("not merged or scheduled for backport" in context.exception)

        self.backports_test_helper.depends_on_backport_succeeds()

        child = self.backports_test_helper.depends_on_backport_wrong_branch()
        child.expect_exact('Continue with Backport? (y/n): ')
        child.sendline('n')
        self.assertTrue("included in any mediawiki production branch" in child.before.decode("utf-8"))

    def test_revert(self):
        self.assertEqual(self.backports_test_helper.extension_revert(), '1')
