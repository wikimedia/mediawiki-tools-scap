import unittest
import tempfile
import os.path
import subprocess

from scap import git
import scap

TEMPDIR = tempfile.mkdtemp(suffix="-scap-test-repo")


class GitTest(unittest.TestCase):
    def setUp(self):
        git.init(TEMPDIR)
        git.default_ignore(TEMPDIR)
        scap.runcmd.touch("testfile", cwd=TEMPDIR)
        git.add_all(TEMPDIR, "first commit")
        scap.runcmd.touch("testfile2", cwd=TEMPDIR)
        git.add_all(TEMPDIR, "second commit")

    def tearDown(self):
        if TEMPDIR.startswith("/tmp") and os.path.isdir(TEMPDIR):
            subprocess.check_call("rm -rf %s" % TEMPDIR, shell=True)

    def test_git_version(self):
        version = git.version()
        assert len(version) > 1
        assert version[0] >= 2, "git version too old. %s" % repr(version)

    def test_is_dir_bad_dir(self):
        assert git.is_dir("/this/cannot/possibly/exist!") is False
        with tempfile.TemporaryDirectory() as tmpdir:
            assert git.is_dir(tmpdir) is False

    def test_is_dir_top_level(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            git.init(tmpdir)
            subdir = os.path.join(tmpdir, "sub")
            os.mkdir(subdir)
            assert git.is_dir(tmpdir) is True
            assert git.is_dir(subdir) is True
            assert git.is_dir(subdir, top_level=True) is False

    def test_git_init(self):
        assert os.path.exists(TEMPDIR), "Failed to create temp dir"
        assert git.is_dir(TEMPDIR), "Failed to initialize git repo"
        desc = git.describe(TEMPDIR)
        assert len(desc) > 1, "git.describe value unexpected: %s" % desc
        info = git.info(TEMPDIR)
        assert info is not None
        assert "head" in info, "missing git.info[head]"

    def test_git_default_ignore(self):
        git.default_ignore(TEMPDIR)
        gif = os.path.join(TEMPDIR, ".gitignore")
        assert os.path.isfile(gif), "failed create .gitignore"
        git.remove_all_ignores(TEMPDIR)
        assert not os.path.isfile(gif), "failed remove_all_ignores"

    def test_resolve_gitdir(self):
        git_dir = git.resolve_gitdir(TEMPDIR)
        assert git_dir == "%s/.git" % TEMPDIR

    def test_git_update_server_info(self):
        git.update_server_info(has_submodules=False, location=TEMPDIR)

        git.update_server_info(has_submodules=True, location=TEMPDIR)

    def test_tag(self):
        git.tag("testing-git-tag", "HEAD", TEMPDIR)
        git.tag("testing-git-tag", "HEAD~1", TEMPDIR, force=True)

    def test_tag_repo(self):
        deploy_info = {
            "user": "scap-test-suite",
            "timestamp": "Right now",
            "tag": "testing-tag_repo",
            "commit": "HEAD",
        }
        git.tag_repo(deploy_info, TEMPDIR)

    def test_clean_tags(self):
        for _ in range(10):
            nexttag = git.next_deploy_tag(TEMPDIR)
            scap.runcmd.gitcmd("tag", nexttag, cwd=TEMPDIR)

        git.clean_tags(TEMPDIR, 2)
        tags = scap.runcmd.gitcmd("tag", "--list", cwd=TEMPDIR).splitlines()
        assert tags is not None, "After clean_tags(2), no tags remain"
        assert len(tags) == 2, "There should only be 2 tags rmaining"

    def test_git_gc(self):
        git.garbage_collect(TEMPDIR)

    def test_parse_submodules(self):
        gitmodules_path = os.path.join(TEMPDIR, ".gitmodules")

        try:
            with open(gitmodules_path, "w") as f:
                f.write('[submodule "sub"]\n\tpath= sub\n\turl = /tmp\n')
            out = git.parse_submodules(TEMPDIR)
            assert out
            assert "sub" in out
            assert "url" in out["sub"]
            assert out["sub"]["url"] == "/tmp"

            # Same thing, but with one less tab character.
            with open(gitmodules_path, "w") as f:
                f.write('[submodule "sub"]\npath = sub\n\turl = /tmp\n')
            out = git.parse_submodules(TEMPDIR)
            assert out
            assert "sub" in out
            assert "url" in out["sub"]
            assert out["sub"]["url"] == "/tmp"
        finally:
            if os.path.exists(gitmodules_path):
                os.unlink(gitmodules_path)

    def test_remote_set_and_get_url(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            git.init(tmpdir)
            # Test the codepath where the remote doesn't exist yet
            git.remote_set_url(tmpdir, "https://gerrit.wikimedia.org/pull1")
            assert git.remote_get_url(tmpdir) == "https://gerrit.wikimedia.org/pull1"
            assert (
                git.remote_get_url(tmpdir, push=True)
                == "https://gerrit.wikimedia.org/pull1"
            )

            # Test the codepath where the remote already exists
            git.remote_set_url(tmpdir, "https://gerrit.wikimedia.org/pull2")
            assert git.remote_get_url(tmpdir) == "https://gerrit.wikimedia.org/pull2"
            assert (
                git.remote_get_url(tmpdir, push=True)
                == "https://gerrit.wikimedia.org/pull2"
            )

            git.remote_set_url(tmpdir, "https://gerrit.wikimedia.org/push", push=True)
            assert git.remote_get_url(tmpdir) == "https://gerrit.wikimedia.org/pull2"
            assert (
                git.remote_get_url(tmpdir, push=True)
                == "https://gerrit.wikimedia.org/push"
            )

    def test_set_env_vars_for_user(self):
        # Start with a blank environment for testing
        with unittest.mock.patch.dict(os.environ, clear=True):
            git_env_vars = [
                "GIT_COMMITTER_EMAIL",
                "GIT_AUTHOR_EMAIL",
                "GIT_COMMITTER_NAME",
                "GIT_AUTHOR_NAME",
            ]

            with git.with_env_vars_set_for_user():
                for var in git_env_vars:
                    assert os.environ.get(var)

            # Verify that with_env_vars_set_for_user() restores the environment on exit.
            for var in git_env_vars:
                assert os.environ.get(var) is None

            with tempfile.TemporaryDirectory() as fakehome:
                os.environ["HOME"] = fakehome

                with open(os.path.join(fakehome, ".gitconfig"), "w") as f:
                    f.write(
                        """
[user]
    name = Scap Tests
    email = scap@wikimedia.org
"""
                    )
                with git.with_env_vars_set_for_user():
                    # Verify that with_env_vars_set_for_user() does not set any
                    # environment variables when it is able to retrieve user.name and user.email from
                    # configuration.
                    for var in git_env_vars:
                        assert os.environ.get(var) is None
