# -*- coding: utf-8 -*-
"""
    scap.git
    ~~~~~~~~
    Helpers for git operations and interacting with .git directories

"""
import collections
import errno
import os
import re
import socket
import subprocess
from datetime import datetime

import yaml

import scap
import scap.utils as utils
from scap.runcmd import gitcmd, FailedCommand


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


def largefile_pull(location, implementor, submodules=False):
    """Syncs all git-fat or git-lfs objects for the given repo directory.

    :param location: Repository to work in
    :param implementor: What implementation to pull with (git-lfs, git-fat)
    """
    if implementor == LFS:
        gitcmd("lfs", "pull", cwd=location)
        if submodules:
            gitcmd("submodule", "foreach", "--recursive", "git lfs pull", cwd=location)
    elif implementor == FAT:
        fat_init(location)
        gitcmd("fat", "pull", cwd=location)
    else:
        raise ValueError("Must be passed one of lfs or fat")


def lfs_install(*args):
    """Run git-lfs-install with provided arguments."""
    lfsargs = ["install"] + list(args)
    # run `git lfs install $args`
    gitcmd("lfs", *lfsargs)


def info(directory, remote="origin"):
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

    # This information is used by https://<site>/wiki/Special:Version
    # to construct a link to a commit in Gerrit (gitiles), so it must
    # not refer to a local commit (i.e., a patch).  Use git merge-base
    # to find the nearest public commit.
    try:
        head_sha1 = gitcmd(
            "merge-base", "HEAD", "{}".format(remote), cwd=directory
        ).strip()
    except Exception:
        # The git merge-base command won't work if origin doesn't have
        # a HEAD reference (which points to the default branch).
        head_sha1 = sha(directory, "HEAD")

    commit_date = gitcmd("show", "-s", "--format=%ct", head_sha1, cwd=directory).strip()

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
    set_env_vars_for_user()

    try:
        gitcmd("commit", "--quiet", "-m", message, cwd=location)
    except FailedCommand:
        pass  # ignore errors


def set_env_vars_for_user():
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
    """Calculates the scap/sync/YYYY-MM-DD/{n} tag to use for this deployment"""
    ensure_dir(location)
    timestamp = datetime.utcnow()
    date = timestamp.strftime("%F")  # YYYY-MM-DD format.

    last_seq = 0

    todays_tags = gitcmd(
        "tag",
        "--list",
        "--sort=-version:refname",
        f"{TAG_PREFIX}/{date}/*",
        cwd=location,
    ).splitlines()
    if todays_tags:
        last_tag = todays_tags[0]
        m = re.search(r"/(\d{4})$", last_tag)
        if not m:
            raise ValueError(
                f"The most recent deployment tag '{last_tag}' does not have the expected format."
            )
        last_seq = int(m[1])

    seq = last_seq + 1

    return f"{TAG_PREFIX}/{date}/{seq:04d}"


def ensure_dir(location):
    """Ensure that we're in a git directory. If not, explode"""
    if location is None or not is_dir(location):
        raise IOError(errno.ENOENT, "Location is not a git repo", location)


def is_dir(path) -> bool:
    """
    Returns True if 'path' is a git checkout directory, False otherwise.
    """

    path = os.path.abspath(os.path.expandvars(os.path.expanduser(path)))
    try:
        gitcmd("rev-parse", "--is-inside-work-tree", cwd=path)
        return True
    except FailedCommand:
        return False


def remote_exists(location, remote):
    """Check if remote exists in location"""
    ensure_dir(location)
    argv = [
        "git",
        "-C",
        location,
        "config",
        "--local",
        "--get",
        "remote.{}.url".format(remote),
    ]
    return (
        subprocess.call(argv, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0
    )


def remote_set_url(location, url, remote="origin", push=False):
    """
    In the git repo at 'location', set the url of the specified remote.
    """
    ensure_dir(location)
    if remote_exists(location, remote):
        cmd = ["remote", "set-url"]
        if push:
            cmd.append("--push")
        cmd.append(remote)
        cmd.append(url)
        gitcmd(*cmd, cwd=location)
    else:
        gitcmd("remote", "add", remote, url, cwd=location)


def remote_get_url(location, remote="origin", push=False) -> str:
    ensure_dir(location)
    cmd = ["remote", "get-url"]
    if push:
        cmd.append("--push")
    cmd.append(remote)

    return gitcmd(*cmd, cwd=location).strip()


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
        remote_set_url(location, repo)
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

    logger.debug("git submodule sync")
    gitcmd("submodule", "sync", "--recursive", cwd=location)


def update_submodules(
    location,
    git_remote=None,
    use_upstream=False,
    reference=None,
    checkout=False,
    force=False,
    lfs_smudge=False,
):
    """Update git submodules on target machines"""

    if not use_upstream and git_remote is None:
        raise ValueError("Must set git_remote if not using upstream")

    ensure_dir(location)

    logger = utils.get_logger()

    sync_submodules(location)

    original_submodules_info = None

    logger.debug("Fetch submodules")
    if not use_upstream:
        logger.debug("Remapping submodule %s to %s", location, git_remote)
        original_submodules_info = remap_submodules(location, git_remote)
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

    env = os.environ.copy()
    if not lfs_smudge:
        env["GIT_LFS_SKIP_SMUDGE"] = "1"

    gitcmd("submodule", *cmd, cwd=location, env=env)

    if original_submodules_info:
        for submodule_name, submodule_info in original_submodules_info.items():
            lfs_url = f"{submodule_info['url']}/info/lfs"
            logger.debug(f"Setting lfs.url of {submodule_name} to {lfs_url}")
            gitcmd(
                "config", "lfs.url", lfs_url, cwd=os.path.join(location, submodule_name)
            )


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
            "-a",
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


def parse_submodules(location) -> dict:
    """
    Reads .gitmodules and returns a dictionary of information about each
    defined submodule.

    The key is the name of the submodule, and the value is a dictionary
    with "path" (typically the same as the submodule name) and "url" keys.
    """
    submodules = collections.defaultdict(dict)

    for line in gitcmd(
        "config", "--list", "--file", ".gitmodules", cwd=location
    ).splitlines():
        m = re.match(r"submodule\.(.*)\.(path|url)=(.*)$", line)
        if m:
            name, setting, value = m.groups()
            submodules[name][setting] = value

    return submodules


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

    submodules_info = parse_submodules(location)

    with utils.temp_to_permanent_file(gitmodule) as module:
        for submodule_name, info in submodules_info.items():
            # Since we're using a non-bare http remote, map the submodule
            # to the submodule path under $GIT_DIR/modules subdirectory of
            # the superproject (git documentation: https://git.io/v4W9F).
            remote_path = "{}/modules/{}".format(server, submodule_name)
            module.write('[submodule "{}"]\n'.format(submodule_name))
            module.write("\tpath = {}\n".format(info["path"]))
            module.write("\turl = {}\n".format(remote_path))

    sync_submodules(location)

    return submodules_info


def list_submodules_paths_urls(repo, args):
    """Return a list of the paths and URLs of the submodules of the given repository, separated by a space"""
    ensure_dir(repo)

    return gitcmd(
        "submodule",
        "-q",
        "foreach",
        'echo -n "$PWD " && git remote get-url origin',
        args,
        cwd=repo,
    ).splitlines()


def reflog(repo, fmt="oneline", branch=None):
    """
    Fetch reflog as list
    """
    cmd = ["-C", repo, "log", "--walk-reflogs", "--format={}".format(fmt)]

    if branch is not None:
        cmd.append(branch)

    return gitcmd(*cmd, cwd=repo).splitlines()


# FIXME: reference and ref together are confusing.
def clone_or_update_repo(dir, repo, branch, logger, reference=None, ref=None):
    """
    Clone or update the checkout of 'repo' in 'dir', using the specified
    branch.   Note that existing repos are hard-reset to the match the
    state of the origin.

    Submodules are handled as well.

    Returns the sha1 of the checked out branch.
    """

    operation = "Update"
    if not os.path.isdir(dir) or utils.dir_is_empty(dir):
        operation = "Clone"

    logger.info("{} {} ({} branch) in {}".format(operation, repo, branch, dir))

    config = {"core.sharedRepository": "group"}

    if operation == "Clone":
        fetch(dir, repo, branch=branch, reference=reference, config=config)

    logger.debug("Fetching from origin")
    fetch(dir, repo, branch=branch, config=config)
    changes_fetched = gitcmd("log", "HEAD..@{upstream}", cwd=dir)
    if changes_fetched:
        logger.info("Changes pulled down since last fetch:\n%s", changes_fetched)

    if ref is None:
        ref = "origin/{}".format(branch)

    gitcmd("checkout", "--force", "-B", branch, ref, cwd=dir)
    head = sha(dir, "HEAD")
    logger.info("{} checked out at commit {}".format(repo, head))

    logger.debug("Updating submodules")
    update_submodules(dir, use_upstream=True, checkout=True, force=True)

    return head


def file_has_unstaged_changes(file, location=None) -> bool:
    """
    Untracked files are also considered
    """

    location = location or os.getcwd()

    ensure_dir(location)
    git_status_output = gitcmd("status", "--porcelain", file, cwd=location)
    return bool(re.search(r"(?m)^( M|\?\?)", git_status_output))
