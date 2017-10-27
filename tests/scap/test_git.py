#!/usr/bin/env python2

from __future__ import absolute_import

import unittest
import tempfile
import os.path
import subprocess

import scap.git as git


tempdir = tempfile.mkdtemp(suffix='-scap-test-repo')


class GitTest(unittest.TestCase):

    def setUp(self):
        git.init(tempdir)

    def tearDown(self):
        if tempdir.startswith('/tmp') and os.path.isdir(tempdir):
            subprocess.check_call('rm -rf %s' % tempdir, shell=True)

    def test_git_init(self):
        self.assertTrue(os.path.exists(tempdir), 'Failed to create temp dir')
        self.assertTrue(git.is_dir(tempdir), 'Failed to initialize git repo')

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
