# -*- coding: utf-8 -*-
"""
    scap.git
    ~~~~~~~~
    Helpers for git operations and interacting with .git directories

"""
from __future__ import absolute_import

from datetime import datetime
import errno
import re
import os
import socket
import subprocess
import yaml


from scap.runcmd import gitcmd, FailedCommand
import scap.utils as utils
import scap


def version():
    try:
        v = gitcmd("version")
    except (FailedCommand, KeyError):
        return (1, 9, 0)

    version_numbers = v.split(" ")[2]
    return tuple(int(n) for n in version_numbers.split(".")[:4] if n.isdigit())


# All tags created by scap use this prefix
TAG_PREFIX = "scap/sync"
GIT_VERSION = version()

# Key is the pattern for .gitignore, value is a test for that pattern.
DEFAULT_IGNORE = {"*~", "*.swp", "*/cache/l10n/*.cdb", "scap/log/*"}


LFS = "git-lfs"
FAT = "git-fat"


def info_filename(directory, install_path, cache_path):
    """Compute the path for a git_info cache file related to a given
    directory.

    >>> info_filename('foo', 'foo', '')
    'info.json'
    >>> info_filename('foo/bar/baz', 'foo', 'xyzzy')
    'xyzzy/info-bar-baz.json'
    """
    path = directory
    if path.startswith(install_path):
        path = path[len(install_path) :]
    return os.path.join(cache_path, "info%s.json" % path.replace("/", "-"))


def sha(location, rev):
    """Returns SHA1 for things like HEAD or HEAD~~"""
    ensure_dir(location)
    return gitcmd("rev-parse", "--verify", rev, cwd=location).strip()


def describe(location):
    """Returns a convenient label for the current state of the git repo."""
    ensure_dir(location)
    return gitcmd("describe", "--always", cwd=location).strip()


def init(location):
    if not os.path.exists(location):
        utils.mkdir_p(location)

    if not os.path.isdir(location):
        raise IOError(errno.ENOENT, "Location is not a directory", location)

    return gitcmd("init", cwd=location).strip()


def fat_init(location):
    """Initializes the given directory for git-fat use."""

    gitcmd("fat", "init", cwd=location)


def fat_isinitialized(location):
    """Returns whether git-fat has been initialized for the given directory."""

    try:
        gitcmd("config", "--local", "--get", "filter.fat.smudge", cwd=location)
        return True
    except FailedCommand:
        return False


def largefile_pull(location, implementor):
    """Syncs all git-fat or git-lfs objects for the given repo directory.

    :param location: Repository to work in
    :param implementor: What implementation to pull with (git-lfs, git-fat)
    """
    if implementor == LFS:
        gitcmd("lfs", "pull", cwd=location)
    elif implementor == FAT:
        fat_init(location)
        gitcmd("fat", "pull", cwd=location)
    else:
        raise ValueError("Must be passed one of lfs or fat")


def lfs_install(*args):
    """Run git-lfs-install with provided arguments.

    If no args are provided, defaults to `git lfs install --global`
    """
    if not args:
        args = ["--global"]
    lfsargs = ["install"] + list(args)
    # run `git lfs install $args`
    gitcmd("lfs", *lfsargs)


def info(directory):
    """Compute git version information for a given directory that is
    compatible with MediaWiki's GitInfo class.

    :param directory: Directory to scan for git information
    :returns: Dict of information about current repository state
    """
    git_dir = resolve_gitdir(directory)

    head_file = os.path.join(git_dir, "HEAD")
    with open(head_file, "r") as headfile:
        head = headfile.read().strip()
    if head.startswith("ref: "):
        head = head[5:]

    if head.startswith("refs/heads/"):
        branch = head[11:]
    elif head.startswith("refs/tags/"):
        branch = head[10:]
    else:
        branch = head

    head_sha1 = get_disclosable_head(directory, branch)
    if head_sha1:
        commit_date = gitcmd(
            "show", "-s", "--format=%ct", head_sha1, cwd=directory
        ).strip()
    else:
        commit_date = ""

    # Requires git v1.7.5+
    try:
        remote_url = gitcmd("ls-remote", "--get-url", cwd=directory).strip()

    except FailedCommand:
        remote_url = ""
        utils.get_logger().info("Unable to find remote URL for %s", git_dir)

    return {
        "@directory": directory,
        "head": head,
        "headSHA1": head_sha1,
        "headCommitDate": commit_date,
        "branch": branch,
        "remoteURL": remote_url,
    }


def remove_all_ignores(location):
    """
    Remove .gitignore files under a location.
    """
    scap.runcmd.delete_file_in_tree(location, ".gitignore")


def default_ignore(location):
    """Create a default .gitignore file."""
    remove_all_ignores(location)
    ignore = "\n".join(DEFAULT_IGNORE)
    filename = os.path.join(location, ".gitignore")
    with open(filename, "w+") as gitignore:
        gitignore.write(ignore)


def clean_tags(location, max_tags):
    """Make sure there aren't more than max_tags."""

    ensure_dir(location)

    tags = gitcmd(
        "for-each-ref",
        "--sort=taggerdate",
        "--format=%(refname)",
        "refs/tags",
        cwd=location,
    )
    tags = tags.splitlines()

    old_tags = []
    while len(tags) > max_tags:
        tag = tags.pop(0)
        if tag.startswith("refs/tags/"):
            tag = tag[10:]

        # Don't delete tags that aren't ours
        if not tag.startswith(TAG_PREFIX):
            continue

        old_tags.append(tag)

    # if there aren't any old tags, bail early
    if not old_tags:
        return

    gitcmd("tag", "-d", *old_tags, cwd=location)


def garbage_collect(location):
    """Clean up a repo."""

    ensure_dir(location)
    gitcmd("gc", "--quiet", "--auto", cwd=location)


def add_all(location, message="Update"):
    """Add everything to repo at location as user."""

    # Initialize repo if it isn't already
    if not is_dir(location):
        gitcmd("init", cwd=location)

    gitcmd("add", "--all", cwd=location)

    # None of these values can be unset or empty strings because we use
    # them as git envvars below. Unset values and empty strings will
    # cause git to shout about ident errors.
    host = socket.getfqdn() or "localhost"
    euid = utils.get_username() or "unknown"
    ruid = utils.get_real_username() or "unknown"
    ename = utils.get_user_fullname() or "Unknown User"
    rname = utils.get_real_user_fullname() or "Unknown User"

    os.environ["GIT_COMMITTER_EMAIL"] = "{}@{}".format(euid, host)
    os.environ["GIT_AUTHOR_EMAIL"] = "{}@{}".format(ruid, host)

    os.environ["GIT_COMMITTER_NAME"] = ename
    os.environ["GIT_AUTHOR_NAME"] = rname

    try:
        gitcmd("commit", "--quiet", "-m", message, cwd=location)
    except FailedCommand:
        pass  # ignore errors


def last_deploy_tag(location):
    """Finds the last tag to use for this deployment"""
    ensure_dir(location)

    tags = gitcmd(
        "tag", "--list", os.path.join(TAG_PREFIX, "*"), cwd=location
    ).splitlines()
    tags = sorted(tags, reverse=True)
    if tags:
        return tags[0]

    return None


def next_deploy_tag(location):
    """Calculates the scap/sync/{date}/{n} tag to use for this deployment"""
    ensure_dir(location)
    timestamp = datetime.utcnow()
    date = timestamp.strftime("%F")
    args = ["--list"]
    tag_fmt = os.path.join(TAG_PREFIX, "{}", "*")
    args.append(tag_fmt.format(date))
    seq = len(gitcmd("tag", *args, cwd=location).splitlines()) + 1
    tag_fmt = os.path.join(TAG_PREFIX, "{0}", "{1:04d}")
    return tag_fmt.format(date, seq)


def ensure_dir(location):
    """Ensure that we're in a git directory. If not, explode"""
    if location is None or not is_dir(location):
        raise IOError(errno.ENOENT, "Location is not a git repo", location)


def is_dir(path):
    """Checks if path is a git, doesn't count submodule directories"""
    git_path = os.path.join(
        os.path.abspath(os.path.expandvars(os.path.expanduser(path))), ".git"
    )
    return (
        os.path.isdir(git_path)
        and os.path.isdir(os.path.join(git_path, "objects"))
        and os.path.isdir(os.path.join(git_path, "refs"))
        and os.path.isfile(os.path.join(git_path, "HEAD"))
    )


def remote_exists(location, remote):
    """Check if remote exists in location"""
    ensure_dir(location)
    with utils.cd(location):
        argv = ["git", "config", "--local", "--get", "remote.{}.url".format(remote)]
        return subprocess.call(argv) == 0


def remote_set(location, repo, remote="origin"):
    """set the remote at location to repo"""
    ensure_dir(location)
    if remote_exists(location, remote):
        gitcmd("remote", "set-url", remote, repo, cwd=location)
    else:
        gitcmd("remote", "add", remote, repo, cwd=location)


def fetch(
    location,
    repo,
    reference=None,
    dissociate=True,
    recurse_submodules=False,
    shallow=False,
    bare=False,
    config=None,
    branch=None,
):
    """Fetch a git repo to a location"""
    if config is None:
        config = {}

    if is_dir(location):
        remote_set(location, repo)
        cmd = append_jobs_arg(["--tags"])
        if recurse_submodules:
            cmd.append("--recurse-submodules")
        else:
            cmd.append("--no-recurse-submodules")
        if branch:
            cmd.append("origin")
            cmd.append(branch)
        gitcmd("fetch", *cmd, cwd=location)
        for name, value in config.items():
            gitcmd("config", name, value, cwd=location)
    else:
        cmd = append_jobs_arg([])
        if shallow:
            cmd.append("--depth")
            cmd.append("1")
        if reference is not None and GIT_VERSION[0] > 1:
            ensure_dir(reference)
            cmd.append("--reference")
            cmd.append(reference)
            if dissociate:
                cmd.append("--dissociate")
        if recurse_submodules:
            cmd.append("--recurse-submodules")
            if shallow:
                cmd.append("--shallow-submodules")
        if bare:
            cmd.append("--bare")
        if branch:
            cmd.append("-b")
            cmd.append(branch)
        if config:
            for name, value in config.items():
                cmd.append("--config")
                cmd.append("{}={}".format(name, value))
        cmd.append(repo)
        cmd.append(location)
        gitcmd("clone", *cmd)


def append_jobs_arg(cmd):
    if GIT_VERSION[0] > 2 or (GIT_VERSION[0] == 2 and GIT_VERSION[1] > 9):
        cmd.append("--jobs")
        cmd.append(str(utils.cpus_for_jobs()))
    return cmd


def checkout(location, rev):
    """Checkout a git repo sha at a location"""
    ensure_dir(location)

    logger = utils.get_logger()

    logger.debug("Checking out rev: %s at location: %s", rev, location)
    gitcmd("checkout", "--force", "--quiet", rev, cwd=location)


def sync_submodules(location):
    """Sync git submodules on target machines"""

    ensure_dir(location)

    logger = utils.get_logger()

    logger.debug("Syncing out submodules")
    gitcmd("submodule", "sync", "--recursive", cwd=location)


def update_submodules(location, git_remote=None, use_upstream=False, reference=None, checkout=False, force=False):
    """Update git submodules on target machines"""

    if not use_upstream and git_remote is None:
        raise ValueError("Must set git_remote if not using upstream")

    ensure_dir(location)

    logger = utils.get_logger()

    with utils.cd(location):
        logger.debug("Fetch submodules")
        if not use_upstream:
            logger.debug("Remapping submodule %s to %s", location, git_remote)
            remap_submodules(location, git_remote)
        else:
            logger.debug("Using upstream submodules")

        cmd = ["update", "--init", "--recursive"]
        cmd = append_jobs_arg(cmd)

        if checkout:
            cmd.append("--checkout")
        if force:
            cmd.append("--force")

        if reference is not None and GIT_VERSION[0] > 1:
            logger.debug("Using --reference repository: %s", reference)
            ensure_dir(reference)
            cmd.append("--reference")
            cmd.append(reference)

        gitcmd("submodule", *cmd, cwd=location)


@utils.log_context("git_update_server_info")
def update_server_info(has_submodules=False, location=os.getcwd(), logger=None):
    """runs git update-server-info and tags submodules"""

    logger.debug("Update server info")
    gitcmd("update-server-info", cwd=location)

    if has_submodules:
        gitcmd(
            "submodule",
            "foreach",
            "--recursive",
            "git update-server-info",
            cwd=location,
        )


def update_deploy_head(deploy_info, location):
    """updates .git/DEPLOY_HEAD file

    :param deploy_info: current deploy info to write to file as YAML
    :param (optional) location: git directory location (default cwd)
    """
    logger = utils.get_logger()
    ensure_dir(location)

    with utils.cd(location):
        deploy_file = os.path.join(location, ".git", "DEPLOY_HEAD")
        logger.debug("Creating %s", deploy_file)
        with open(deploy_file, "w+") as deployfile:
            # Note that part of deploy_info may be an OrderedDict (xref
            # deploy.py:checks_setup) which requires non-safe yaml load in
            # deploy.py:_get_remote_overrides and _get_config_overrides.  If
            # we settle on Python 3.7 as the minimum supported version, we
            # could just use regular dicts which are defined to retain their
            # insertion order.
            deployfile.write(yaml.dump(deploy_info, default_flow_style=False))
            deployfile.close()


def tag_repo(deploy_info, location=os.getcwd()):
    """creates new tag in deploy repo"""

    ensure_dir(location)
    with utils.cd(location):
        gitcmd(
            "tag",
            "-fa",
            "-muser {}".format(deploy_info["user"]),
            "-mtimestamp {}".format(deploy_info["timestamp"]),
            "--",
            deploy_info["tag"],
            deploy_info["commit"],
        )


def resolve_gitdir(directory):
    """Find the .git directory for a given path.

    This will resolve the gitlink
    if path/.git is a gitlink to a bare repo e.g. a file with one line,
    like this:

    gitdir:/path/to/somewhere/else
    """
    if directory.endswith(".git"):
        git_dir = directory
        directory = directory[:-3]
    else:
        git_dir = os.path.join(directory, ".git")

    if not os.path.exists(git_dir):
        raise IOError(errno.ENOENT, ".git not found", directory)

    if os.path.isfile(git_dir):
        # submodules
        with open(git_dir, "r") as gitdir:
            git_ref = gitdir.read().strip()
        if not git_ref.startswith("gitdir: "):
            raise IOError(errno.EINVAL, "Unexpected .git contents", git_dir)
        git_ref = git_ref[8:]
        if git_ref[0] != "/":
            git_ref = os.path.abspath(os.path.join(directory, git_ref))
        git_dir = git_ref

    return git_dir


def remap_submodules(location, server):
    """Remap all submodules to deployment server

    This function supports remapping submodules available on the deployment
    server. Since the remote is a non-bare repo (as of git 1.7.8) all
    submodules should be available over http under the remote server
    checkout's git dir:
    [server]/[repo]/.git/modules/[submodule_path]

    :param location: String path to local git checkout containing a
                     `.gitmodules` file
    :param server: String path to remote, non-bare, repo gitdir
    """
    ensure_dir(location)

    logger = utils.get_logger()

    gitmodule = os.path.join(location, ".gitmodules")
    if not os.path.isfile(gitmodule):
        logger.warning("Unable to rewrite_submodules: No .gitmodules in %s", location)
        return

    logger.info("Updating .gitmodule: %s", os.path.dirname(gitmodule))

    # ensure we're working with a non-modified .gitmodules file
    gitcmd("checkout", ".gitmodules", cwd=location)

    # get .gitmodule info
    modules = gitcmd("config", "--list", "--file", ".gitmodules", cwd=location)

    submodules = {}
    for line in modules.split("\n"):
        if not line.startswith("submodule."):
            continue

        module_conf = line.split("=")
        module_name = module_conf[0].strip()

        if module_name.endswith(".path"):
            name = module_name[len("submodule.") : -len(".path")]
            submodules[name] = module_conf[1].strip()

    with open(gitmodule, "w") as module:
        for submodule_name, submodule_path in submodules.items():
            # Since we're using a non-bare http remote, map the submodule
            # to the submodule path under $GIT_DIR/modules subdirectory of
            # the superproject (git documentation: https://git.io/v4W9F).
            remote_path = "{}/modules/{}".format(server, submodule_name)
            module.write('[submodule "{}"]\n'.format(submodule_name))
            module.write("\tpath = {}\n".format(submodule_path))
            module.write("\turl = {}\n".format(remote_path))

    sync_submodules(location)


def get_disclosable_head(repo_directory, remote_thing):
    """
    Get the SHA1 of the most recent commit that can be publicly disclosed.
    If a commit only exists locally, it is considered private. This function
    will try to get the tip of the remote tracking branch, and fall back to
    the common ancestor of HEAD and the remote version of the local branch
    we're ostensibly tracking.

    :param repo_directory: Directory to look into
    :param remote_thing: If you're not actively tracking a remote branch, you
                         need to provide something remote for this function to
                         look for a common ancestor with. Otherwise, this
                         function has no way of knowing what common tree
                         you could possibly care about. This could be a branch,
                         a tag, or a plain sha1
    :returns: str
    """
    with open(os.devnull, "wb") as dev_null:
        try:
            return subprocess.check_output(
                ("/usr/bin/git", "rev-list", "-1", "@{upstream}"),
                cwd=repo_directory,
                stderr=dev_null,
            ).decode().strip()
        except subprocess.CalledProcessError:
            try:
                remote = subprocess.check_output(
                    ("/usr/bin/git", "remote"), cwd=repo_directory, stderr=dev_null
                ).decode().strip()

                # If the branch is not a SHA1, combine with remote name
                if not re.match("[a-f0-9]{40}", remote_thing):
                    remote_thing = "%s/%s" % (remote, remote_thing)
                # If the branch is a SHA1, count on remote HEAD being a
                # symbolic-ref for the actual remote
                else:
                    remote_thing = remote
                return subprocess.check_output(
                    ("/usr/bin/git", "merge-base", "HEAD", remote_thing),
                    cwd=repo_directory,
                    stderr=dev_null,
                ).decode().strip()
            except subprocess.CalledProcessError:
                utils.get_logger().info(
                    "Unable to find remote tracking branch/tag for %s", repo_directory
                )
                return ""


def list_submodules(repo, args):
    """List all of the submodules of a given respository"""
    ensure_dir(repo)
    submodules = []

    res = gitcmd("submodule", "status", args, cwd=repo)

    for line in res.splitlines():
        submodules.append(line.split()[1])
    return submodules


def reflog(repo, fmt="oneline", branch=None):
    """
    Fetch reflog as list
    """
    cmd = ["-C", repo, "log", "--walk-reflogs", "--format={}".format(fmt)]

    if branch is not None:
        cmd.append(branch)

    return gitcmd(*cmd, cwd=repo).splitlines()
