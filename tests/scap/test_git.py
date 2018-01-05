#!/usr/bin/env python2

from __future__ import absolute_import

import unittest
import tempfile
import os.path
import subprocess

from scap import git
from scap import sh

TEMPDIR = tempfile.mkdtemp(suffix='-scap-test-repo')
_TOUCH = sh.Command('touch')


class GitTest(unittest.TestCase):

    def setUp(self):
        git.init(TEMPDIR)
        git.default_ignore(TEMPDIR)
        with sh.pushd(TEMPDIR):
            _TOUCH('testfile')
            git.add_all(TEMPDIR, 'first commit')

    def tearDown(self):
        if TEMPDIR.startswith('/tmp') and os.path.isdir(TEMPDIR):
            subprocess.check_call('rm -rf %s' % TEMPDIR, shell=True)

    def test_git_version(self):
        version = git.version()
        assert len(version) > 1
        assert version[0] >= 2, 'git version too old. %s' % repr(version)

    def test_git_init(self):
        assert os.path.exists(TEMPDIR), 'Failed to create temp dir'
        assert git.is_dir(TEMPDIR), 'Failed to initialize git repo'
        desc = git.describe(TEMPDIR)
        assert len(desc) > 1, "git.describe value unexpected: %s" % desc
        info = git.info(TEMPDIR)
        assert info is not None
        assert 'head' in info, 'missing git.info[head]'

    def test_git_fat(self):
        if sh.which('git-fat') is None:
            return
        git.fat_init(TEMPDIR)
        assert git.fat_isinitialized(TEMPDIR), 'git fat was not initialized'

    def test_git_default_ignore(self):
        git.default_ignore(TEMPDIR)
        gif = os.path.join(TEMPDIR, '.gitignore')
        assert os.path.isfile(gif), 'failed create .gitignore'
        git.remove_all_ignores(TEMPDIR)
        assert not os.path.isfile(gif), 'failed remove_all_ignores'

    def test_resolve_gitdir(self):
        git_dir = git.resolve_gitdir(TEMPDIR)
        assert git_dir == "%s/.git" % TEMPDIR

    def test_git_update_server_info(self):
        git.update_server_info(has_submodules=False, location=TEMPDIR)

        git.update_server_info(has_submodules=True, location=TEMPDIR)

    def test_clean_tags(self):
        with sh.pushd(TEMPDIR):
            for _ in range(10):
                nexttag = git.next_deploy_tag(TEMPDIR)
                git.git.tag(nexttag)

            git.clean_tags(TEMPDIR, 2)
            tags = git.git.tag('--list').splitlines()
            assert tags is not None, 'After clean_tags(2), no tags remain'
            assert len(tags) == 2, 'There should only be 2 tags rmaining'

    def test_git_gc(self):
        git.garbage_collect(TEMPDIR)
