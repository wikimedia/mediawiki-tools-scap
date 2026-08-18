import json
import unittest
import tempfile
import os.path
import subprocess

from scap import git
import scap

TEMPDIR = tempfile.mkdtemp(suffix="-scap-test-repo")


def add_submodule(repo, submodule, path):
    """Add 'submodule' to 'repo' at 'path' and commit the result."""
    scap.runcmd.gitcmd(
        # git refuses a submodule on a local path without this
        "-c",
        "protocol.file.allow=always",
        "submodule",
        "add",
        submodule,
        path,
        cwd=repo,
    )
    git.add_all(repo, "add submodule %s" % path)


def fake_remote_branch(repo, branch, commit):
    """Create the remote-tracking ref that a fetch from origin would create."""
    scap.runcmd.gitcmd(
        "update-ref", "refs/remotes/origin/%s" % branch, commit, cwd=repo
    )


def clone(source, dest, recursive=False):
    """Clone a repo that is on a local path, like scap prep does."""
    args = ["-c", "protocol.file.allow=always", "clone", "--quiet", source, dest]
    if recursive:
        args.append("--recursive")
    scap.runcmd.gitcmd(*args)


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

    def test_lfs_install_passes_cwd(self):
        with unittest.mock.patch("scap.git.gitcmd") as gitcmd:
            git.lfs_install("--local", "--skip-repo", cwd="/some/repo")

        gitcmd.assert_called_once_with(
            "lfs", "install", "--local", "--skip-repo", cwd="/some/repo"
        )

    @unittest.skipUnless(scap.runcmd.which("git-lfs"), "git-lfs is not installed")
    def test_lfs_install_local(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            repo = os.path.join(tmpdir, "repo")
            home = os.path.join(tmpdir, "home")
            os.mkdir(home)
            git.init(repo)

            # --skip-repo must leave an existing hook alone
            hook = os.path.join(repo, ".git", "hooks", "pre-push")
            with open(hook, "w") as f:
                f.write("#!/bin/sh\nexit 0\n")

            # HOME tells git and git-lfs where the user config is
            with unittest.mock.patch.dict(os.environ, {"HOME": home}):
                git.lfs_install_local(repo)

            config = scap.runcmd.gitcmd("config", "--local", "--list", cwd=repo)
            assert "filter.lfs.process=git-lfs filter-process" in config
            assert "filter.lfs.required=true" in config

            with open(hook) as f:
                assert f.read() == "#!/bin/sh\nexit 0\n"

            user_config = os.path.join(home, ".gitconfig")
            assert not os.path.exists(user_config), "user config must not be written"

    @unittest.skipUnless(scap.runcmd.which("git-lfs"), "git-lfs is not installed")
    def test_lfs_install_local_submodules(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            sub = os.path.join(tmpdir, "sub")
            parent = os.path.join(tmpdir, "parent")
            home = os.path.join(tmpdir, "home")
            os.mkdir(home)

            git.init(sub)
            scap.runcmd.touch("subfile", cwd=sub)
            git.add_all(sub, "sub commit")

            git.init(parent)
            # git refuses a submodule on a local path without this
            scap.runcmd.gitcmd(
                "-c",
                "protocol.file.allow=always",
                "submodule",
                "add",
                sub,
                "sub",
                cwd=parent,
            )
            git.add_all(parent, "add submodule")

            with unittest.mock.patch.dict(os.environ, {"HOME": home}):
                git.lfs_install_local_submodules(parent)

            # The config of the parent repo does not apply to the submodule
            config = scap.runcmd.gitcmd(
                "config", "--local", "--list", cwd=os.path.join(parent, "sub")
            )
            assert "filter.lfs.process=git-lfs filter-process" in config

            user_config = os.path.join(home, ".gitconfig")
            assert not os.path.exists(user_config), "user config must not be written"

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

    def test_submodule_refs(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            deep = os.path.join(tmpdir, "deep")
            sub = os.path.join(tmpdir, "sub")
            parent = os.path.join(tmpdir, "parent")

            for repo in [deep, sub, parent]:
                git.init(repo)
                scap.runcmd.touch("afile", cwd=repo)
                git.add_all(repo, "first commit")

            add_submodule(sub, deep, "lib/deep")
            sub_ref = git.sha(sub, "HEAD")
            add_submodule(parent, sub, "sub")

            # 'sub/lib/deep' is not checked out yet
            assert git.submodule_refs(parent, git.sha(parent, "HEAD")) == {
                "sub": sub_ref
            }

            scap.runcmd.gitcmd(
                "-c",
                "protocol.file.allow=always",
                "submodule",
                "update",
                "--init",
                "--recursive",
                cwd=parent,
            )

            assert git.submodule_refs(parent, git.sha(parent, "HEAD")) == {
                "sub": sub_ref,
                "sub/lib/deep": git.sha(deep, "HEAD"),
            }

            # A repo without submodules
            assert git.submodule_refs(deep, git.sha(deep, "HEAD")) == {}

    def test_collect_info_reports_the_public_submodule_ref(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            sub = os.path.join(tmpdir, "sub")
            parent = os.path.join(tmpdir, "parent")

            for repo in [sub, parent]:
                git.init(repo)
                scap.runcmd.touch("afile", cwd=repo)
                git.add_all(repo, "first commit")

            add_submodule(parent, sub, "sub")
            subdir = os.path.join(parent, "sub")
            public_ref = git.sha(subdir, "HEAD")

            # A local commit in the submodule, like a security patch
            scap.runcmd.touch("patched", cwd=subdir)
            git.add_all(subdir, "security patch")
            assert git.sha(subdir, "HEAD") != public_ref

            git_infos = git.collect_info(parent)

            assert [git_info["@directory"] for git_info in git_infos] == [
                parent,
                subdir,
            ]
            # The submodule is reported at the commit that the parent records,
            # not at the local commit
            assert git_infos[1]["headSHA1"] == public_ref
            for git_info in git_infos:
                assert git_info["branch"] == git.get_branch(parent)

    def test_collect_info_reports_a_branch_that_has_the_commit(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            on_branch = os.path.join(tmpdir, "on-branch")
            no_branch = os.path.join(tmpdir, "no-branch")
            parent = os.path.join(tmpdir, "parent")

            for repo in [on_branch, no_branch, parent]:
                git.init(repo)
                scap.runcmd.touch("afile", cwd=repo)
                git.add_all(repo, "first commit")

            add_submodule(parent, on_branch, "on-branch")
            add_submodule(parent, no_branch, "no-branch")

            # The parent is on a branch of the train, like a MediaWiki version
            branch = "wmf/1.0"
            scap.runcmd.gitcmd("branch", "-m", branch, cwd=parent)
            fake_remote_branch(parent, branch, git.sha(parent, "HEAD"))

            on_branch_dir = os.path.join(parent, "on-branch")
            no_branch_dir = os.path.join(parent, "no-branch")

            # One submodule has that branch too.  The other one is a library
            # that only ever has its own branches.
            fake_remote_branch(on_branch_dir, branch, git.sha(on_branch_dir, "HEAD"))

            git_infos = {
                git_info["@directory"]: git_info
                for git_info in git.collect_info(parent)
            }

            assert git_infos[parent]["branch"] == branch
            assert git_infos[on_branch_dir]["branch"] == branch
            assert "branch" not in git_infos[no_branch_dir]

    def test_collect_info_with_the_master_branch_layout(self):
        # On the master branch of mediawiki/core, extensions, skins and vendor
        # are separate checkouts and not submodules.  See prep._master_stuff.
        with tempfile.TemporaryDirectory() as tmpdir:

            def upstream(name):
                repo = os.path.join(tmpdir, name)
                git.init(repo)
                scap.runcmd.touch("afile", cwd=repo)
                git.add_all(repo, "first commit")
                return repo

            library = upstream("ve")
            extension = upstream("Foo")
            extensions = upstream("extensions")
            skin = upstream("Bar")
            skins = upstream("skins")
            vendor = upstream("vendor")
            core = upstream("core")

            add_submodule(extension, library, "lib/ve")
            add_submodule(extensions, extension, "Foo")
            add_submodule(skins, skin, "Bar")

            # What scap prep does for the master branch
            branch_dir = os.path.join(tmpdir, "php-master")
            clone(core, branch_dir)
            clone(extensions, os.path.join(branch_dir, "extensions"), recursive=True)
            clone(skins, os.path.join(branch_dir, "skins"), recursive=True)
            clone(vendor, os.path.join(branch_dir, "vendor"))
            with open(os.path.join(branch_dir, ".gitignore"), "a") as f:
                f.write("/extensions\n/skins\n/vendor\n")
            git.add_all(branch_dir, "scap prep auto setup")

            git_infos = git.collect_info(branch_dir)

            def path_of(git_info):
                return os.path.relpath(git_info["@directory"], branch_dir)

            assert [path_of(git_info) for git_info in git_infos] == [
                ".",
                "extensions",
                "extensions/Foo",
                "extensions/Foo/lib/ve",
                "skins",
                "skins/Bar",
                "vendor",
            ]

    def test_cache_git_info(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            ext = os.path.join(tmpdir, "Echo")
            branch_dir = os.path.join(tmpdir, "php-1.45.0-wmf.1")

            for repo in [ext, branch_dir]:
                git.init(repo)
                scap.runcmd.touch("afile", cwd=repo)
                git.add_all(repo, "first commit")

            os.mkdir(os.path.join(branch_dir, "cache"))
            os.mkdir(os.path.join(branch_dir, "skins"))
            add_submodule(branch_dir, ext, "extensions/Echo")
            # A directory that is not a git checkout
            os.mkdir(os.path.join(branch_dir, "extensions", "NotACheckout"))

            git_infos = git.cache_git_info(branch_dir)

            assert [git_info["@directory"] for git_info in git_infos] == [
                branch_dir,
                os.path.join(branch_dir, "extensions", "Echo"),
            ]

            cache_dir = os.path.join(branch_dir, "cache", "gitinfo")
            assert sorted(os.listdir(cache_dir)) == [
                "info-extensions-Echo.json",
                "info.json",
            ]
            with open(os.path.join(cache_dir, "info-extensions-Echo.json")) as f:
                assert json.load(f) == git_infos[1]

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
