#!/usr/bin/env python2

from __future__ import absolute_import

import unittest
import tempfile
import os.path
import subprocess

import scap.git as git
import scap.sh as sh

tempdir = tempfile.mkdtemp(suffix='-scap-test-repo')
touch = sh.Command('touch')


class GitTest(unittest.TestCase):

    def setUp(self):
        git.init(tempdir)
        git.default_ignore(tempdir)
        with sh.pushd(tempdir):
            touch('testfile')
            git.add_all(tempdir, 'first commit')

    def tearDown(self):
        if tempdir.startswith('/tmp') and os.path.isdir(tempdir):
            subprocess.check_call('rm -rf %s' % tempdir, shell=True)

    def test_git_version(self):
        VERSION = git.version()
        self.assertTrue(len(VERSION) > 1)
        self.assertGreaterEqual(VERSION[0], 2, 'git version too old. %s'
                                % repr(VERSION))

    def test_git_init(self):

        self.assertTrue(os.path.exists(tempdir), 'Failed to create temp dir')
        self.assertTrue(git.is_dir(tempdir), 'Failed to initialize git repo')
        desc = git.describe(tempdir)
        self.assertGreater(len(desc), 1, "git.describe value unexpected: %s"
                           % desc)
        info = git.info(tempdir)
        self.assertIsNotNone(info)
        self.assertIn('head', info, 'missing git.info[head]')

    def test_git_fat(self):
        if sh.which('git-fat') is None:
            return
        git.fat_init(tempdir)
        self.assertTrue(git.fat_isinitialized(tempdir),
                        'git fat was not initialized')

    def test_git_default_ignore(self):
        git.default_ignore(tempdir)
        gif = os.path.join(tempdir, '.gitignore')
        self.assertTrue(os.path.isfile(gif), 'failed create .gitignore')
        git.remove_all_ignores(tempdir)
        self.assertFalse(os.path.isfile(gif), 'failed remove_all_ignores')

    def test_resolve_gitdir(self):
        git_dir = git.resolve_gitdir(tempdir)
        self.assertEqual(git_dir, "%s/.git" % tempdir)

    def test_git_update_server_info(self):
        git.update_server_info(has_submodules=False, location=tempdir)

        git.update_server_info(has_submodules=True, location=tempdir)

    def test_clean_tags(self):
        with sh.pushd(tempdir):
            for i in range(10):
                nexttag = git.next_deploy_tag(tempdir)
                git.git.tag(nexttag)

            git.clean_tags(tempdir, 2)
            tags = git.git.tag('--list').splitlines()
            self.assertIsNotNone(tags, 'After clean_tags(2), no tags remain')
            self.assertEquals(len(tags), 2,
                              'There should be 2 tags rmaining, found %s'
                              % repr(tags))

    def test_git_gc(self):
        git.garbage_collect(tempdir)
