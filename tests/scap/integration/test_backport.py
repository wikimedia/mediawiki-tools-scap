import os
import re
import subprocess
import time
from datetime import datetime
import unittest

import scap.utils
from scap.plugins.gerrit import GerritSession


def announce(message):
    print("\n** %s **\n" % message)
    

class BackportsTestHelper:
    homedir = "/home/debian"
    workdir = homedir + "/testdir"
    mwconfig_dir = workdir + "/mediawiki-config"
    mwdir = workdir + "/mediawiki"
    mwcore_dir = mwdir + "/core"
    mwgrowthexperiments_dir = mwdir + "/extensions/GrowthExperiments"
    mwbranch = None
    mwgrowth_change_url = None
    gerrit = None
    gerrit_url = "http://gerrit.traindev:8080"
    gerrit_domain = "gerrit.traindev"
    open_change_urls = None

    def setup(self):
        announce("Running scap prep auto to clean up /srv/mediawiki-staging")
        subprocess.check_call(["scap", "prep", "auto"])

        self.gerrit = GerritSession(url=self.gerrit_url)

        if not os.path.exists(self.homedir + "/new-version"):
            print("Bootstrapping with ~/train first")
            subprocess.check_call(self.homedir + "/train")

        scap.utils.mkdir_p(self.workdir)

        with open(self.homedir + "/new-version", "r") as f:
            self.mwbranch = "wmf/" + f.read().rstrip()

        self.git_clone("train-dev", self.gerrit_url + "/operations/mediawiki-config", self.mwconfig_dir)
        self.git_clone(self.mwbranch, self.gerrit_url + "/mediawiki/core", self.mwcore_dir)
        self.git_clone(self.mwbranch, self.gerrit_url + "/mediawiki/extensions/GrowthExperiments",
                       self.mwgrowthexperiments_dir)

        self.setup_core_gitmodules()

    def setup_core_gitmodules(self):
        with scap.utils.cd(self.mwcore_dir):
            subprocess.check_call(["git", "config", "-f", ".gitmodules", "submodule.extensions/GrowthExperiments.url",
                                   self.gerrit_url + "/mediawiki/extensions/GrowthExperiments"])
            if subprocess.check_output(["git", "status", "--porcelain", ".gitmodules"]):
                announce("Tweaking submodule.extensions/GrowthExperiments.url in %s/.gitmodules" % self.mwcore_dir)
                subprocess.check_call(["git", "commit", "-m",
                                       "scap backport testing: set URL for extensions/GrowthExperiments to %s"
                                       % self.gerrit_domain, ".gitmodules"])
                change_url = self.push_and_collect_url(self.mwbranch)
                subprocess.check_call(["git", "reset", "--hard", "HEAD^"])
                self.scap_backport(change_url)
                subprocess.check_call(["git", "pull"])
                announce(".gitmodules tweak successful")

    def git_clone(self, branch, repo, path):
        """ clones a repo """
        announce("Updating checkout of %s (%s) in %s" % (repo, branch, path))
        if not os.path.isdir(path):
            subprocess.check_call(["git", "clone", "-b", branch, repo, path])
            # install gerrit commit-msg hook
            subprocess.check_call(["scp", "-p", "-P", "29418", "traindev@%s:hooks/commit-msg" % self.gerrit_domain,
                                  "%s/.git/hooks/" % path])

        subprocess.check_call(["git", "-C", path, "fetch", "-q", "origin", branch])
        subprocess.check_call(["git", "-C", path, "reset", "-q", "--hard", "FETCH_HEAD"])

    def push_and_collect_url(self, path, branch):
        """ returns the change url after pushing to gerrit """
        git_response = subprocess.check_output(["git", "-C", path, "push", "origin", "HEAD:refs/for/%s" % branch],
                                               text=True, stderr=subprocess.STDOUT)
        change_url = re.findall(r'remote:\s+(%s/c/\S+)' % self.gerrit_url,
                                git_response)[0]
        print("change url is " + change_url)
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
        print("Approving change %s" % change_number)
        subprocess.check_call(["ssh", "-p", "29418", self.gerrit_domain, "gerrit review", current_rev, "--code-review",
                               "+2"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def approve_and_wait_for_merge(self, change_url):
        """ as named  """
        self.approve_change(change_url)
        print("Waiting for %s to merge...", change_url)

        while not self.change_has_merged(change_url):
            time.sleep(2)
        print("%s has merged", change_url)

    def scap_backport(self, change_urls_list):
        print("Running scap backport --yes --stop-before-sync %s" % ' '.join(change_urls_list))
        subprocess.check_call(["scap", "backport", "--yes", "--stop-before-sync"] + change_urls_list)

    def write_to_file(self, mode, filepath, message):
        with open(filepath, mode) as f:
            f.writelines(message)

    def git_command(self, repo, command):
        subprocess.check_call(["git", "-C"] + [repo] + command)

    def git_commit(self, repo, commit_message, filepath):
        command = ["commit", "-m", commit_message, filepath]
        self.git_command(repo, command)

    def config_change_test(self, mode):
        """test that a config change can be backported"""
        announce("Config change test: %s" % mode)
        readme_path = self.mwconfig_dir + "/README"
        text = "scap backport test config change: " + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.write_to_file("a", readme_path, text)
        self.git_commit(self.mwconfig_dir, text, readme_path)
        change_url = self.push_and_collect_url(self.mwconfig_dir, "train-dev")

        if mode == "already_merged":
            print("Merging the change before scap backport to verify that this is okay")
            self.approve_and_wait_for_merge(change_url)

        self.scap_backport([change_url])

    def core_change_test(self):
        """tests a merge commit"""
        announce("Testing mediawiki/core changes (two independent commits resulting in a merge commit)")
        developers_path = self.mwcore_dir + "/DEVELOPERS.md"
        text = "Added by scap backport on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S") + "\n"

        with open(developers_path, 'r') as f:
            contents = f.read()
        self.write_to_file("w", developers_path, text + contents)

        self.git_commit(self.mwcore_dir, "scap backport testing: add a line to top of DEVELOPERS.md", developers_path)
        change_url1 = self.push_and_collect_url(self.mwcore_dir, self.mwbranch)
        print("change_url1 is %s" % change_url1)

        self.git_command(self.mwcore_dir, ["reset", "--hard", "HEAD~1"])
        text = "Added by scap backport on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.write_to_file("a", developers_path, text)
        self.git_commit(self.mwcore_dir, "scap backport testing: add line to bottom of DEVELOPERS.md\n "
                                         "This is expected to result in a merge commit", developers_path)
        change_url2 = self.push_and_collect_url(self.mwcore_dir, self.mwbranch)
        print("change_url2 is %s" % change_url2)

        self.scap_backport([change_url1])
        self.scap_backport([change_url2])

    def extension_test(self):
        """Tests for a submodule update commit
            FIXME: it might be interesting to also test for submodule _and_ merge commit
        """
        announce("Testing mediawiki/extensions/GrowthExperiments change")
        copying_path = self.mwgrowthexperiments_dir + "/COPYING"
        text = "Added by scap backport on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.write_to_file("a", copying_path, text)
        self.git_commit(self.mwgrowthexperiments_dir, "scap backport testing: add line to bottom of COPYING",
                        copying_path)
        change_url = self.push_and_collect_url(self.mwgrowthexperiments_dir, self.mwbranch)
        print("change_url is %s" % change_url)
        self.scap_backport([change_url])

    def relation_chain_test_fails(self):
        """Tests backporting relation chains out of order"""
        announce("Testing the latest commit in a relation chain cannot be backported alone")
        readme_path = self.mwcore_dir + "/README.md"
        text = "Added by scap backport on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.write_to_file("a", readme_path, text)
        self.git_commit(self.mwcore_dir, "scap backport testing: add line to to of README.md", readme_path)
        change_url1 = self.push_and_collect_url(self.mwcore_dir, self.mwbranch)
        print("change_url1 is %s" % change_url1)

        text = "Added by scap backport on " + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(readme_path, 'r') as f:
            contents = f.read()
        self.write_to_file("w", readme_path, text + contents)
        self.git_commit(self.mwcore_dir, "scap backport testing: add line to bottom of README.md\n"
                                         "This is expected to result in a relation chain", readme_path)
        change_url2 = self.push_and_collect_url(self.mwcore_dir, self.mwbranch)
        print("change_url2 is %s" % change_url2)

        self.open_change_urls = [change_url1, change_url2]
        self.scap_backport([change_url2])

    def relation_chain_test_succeeds(self):
        """Tests backporting relation chains"""
        announce("Testing relation chains can be backported")
        self.scap_backport(self.open_change_urls)
        self.open_change_urls = None


class TestBackports(unittest.TestCase):
    """tests the backports"""
    backports_test_helper = None

    @classmethod
    def setup_class(cls):
        cls.backports_test_helper = BackportsTestHelper()
        cls.backports_test_helper.setup()

    def test_config_changes(self):
        self.backports_test_helper.config_change_test("normal")
        self.backports_test_helper.config_change_test("already_merged")

    def test_core_changes(self):
        self.backports_test_helper.core_change_test()

    def test_extension_commit(self):
        self.backports_test_helper.extension_test()

    def test_relation_chains(self):
        with self.assertRaises(Exception) as context:
            self.backports_test_helper.relation_chain_test_fails()
            self.assertTrue("not merged or scheduled for backport" in context.exception)

        self.backports_test_helper.relation_chain_test_succeeds()

