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

import scap.utils as utils


# All tags created by scap use this prefix
TAG_PREFIX = 'scap/sync'

# Key is the pattern for .gitignore, value is a test for that pattern.
DEFAULT_IGNORE = {
    '*~',
    '*.swp',
    '*/cache/l10n/*.cdb',
    'scap/log/*',
}


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
        path = path[len(install_path):]
    return os.path.join(cache_path, 'info%s.json' % path.replace('/', '-'))


def sha(location, rev):
    """Returns SHA1 for things like HEAD or HEAD~~"""
    ensure_dir(location)
    with utils.cd(location):
        cmd = '/usr/bin/git rev-parse --verify {}'.format(rev)
        return subprocess.check_output(cmd, shell=True).strip()


def describe(location):
    """Returns a convenient label for the current state of the git repo."""
    ensure_dir(location)
    with utils.cd(location):
        cmd = '/usr/bin/git describe --always'
        return subprocess.check_output(cmd, shell=True).strip()


def fat_init(location):
    """Initializes the given directory for git-fat use."""

    with utils.cd(location):
        subprocess.check_call('/usr/bin/git fat init', shell=True)


def fat_isinitialized(location):
    """Returns whether git-fat has been initialized for the given directory."""

    with utils.cd(location):
        with open(os.devnull, 'w') as devnull:
            try:
                cmd = '/usr/bin/git config --local --get filter.fat.smudge'
                subprocess.check_call(cmd, stdout=devnull, shell=True)
                return True
            except subprocess.CalledProcessError as cpe:
                if cpe.returncode == 1:
                    return False
                raise cpe


def fat_pull(location):
    """Syncs all git-fat objects for the given repo directory."""

    with utils.cd(location):
        subprocess.check_call('/usr/bin/git fat pull', shell=True)


def info(directory):
    """Compute git version information for a given directory that is
    compatible with MediaWiki's GitInfo class.

    :param directory: Directory to scan for git information
    :returns: Dict of information about current repository state
    """
    git_dir = os.path.join(directory, '.git')
    if not os.path.exists(git_dir):
        raise IOError(errno.ENOENT, '.git not found', directory)

    if os.path.isfile(git_dir):
        # submodules
        with open(git_dir, 'r') as gitdir:
            git_ref = gitdir.read().strip()

        if not git_ref.startswith('gitdir: '):
            raise IOError(errno.EINVAL, 'Unexpected .git contents', git_dir)
        git_ref = git_ref[8:]
        if git_ref[0] != '/':
            git_ref = os.path.abspath(os.path.join(directory, git_ref))
        git_dir = git_ref

    head_file = os.path.join(git_dir, 'HEAD')
    with open(head_file, 'r') as headfile:
        head = headfile.read().strip()
    if head.startswith('ref: '):
        head = head[5:]

    if head.startswith('refs/heads/'):
        branch = head[11:]
    elif head.startswith('refs/tags/'):
        branch = head[10:]
    else:
        branch = head

    head_sha1 = get_disclosable_head(directory, branch)
    if head_sha1:
        commit_date = subprocess.check_output(
            ('/usr/bin/git', 'show', '-s', '--format=%ct', head_sha1),
            cwd=git_dir).strip()
    else:
        commit_date = ''

    # Requires git v1.7.5+
    try:
        remote_url = subprocess.check_output(
            ('/usr/bin/git', 'ls-remote', '--get-url'),
            cwd=git_dir).strip()
    except subprocess.CalledProcessError:
        remote_url = ''
        utils.get_logger().info("Unable to find remote URL for %s", git_dir)

    return {
        '@directory': directory,
        'head': head,
        'headSHA1': head_sha1,
        'headCommitDate': commit_date,
        'branch': branch,
        'remoteURL': remote_url,
    }


def remove_all_ignores(location):
    """
    Remove .gitignore files under a location.
    """
    cmd = (
        '/usr/bin/find',
        '.',
        '-name',
        '.gitignore',
        '-delete')

    subprocess.check_call(cmd, cwd=location)


def default_ignore(location):
    """Create a default .gitignore file."""
    remove_all_ignores(location)
    ignore = '\n'.join(DEFAULT_IGNORE)
    with utils.cd(location):
        with open('.gitignore', 'w+') as gitignore:
            gitignore.write(ignore)


def clean_tags(location, max_tags):
    """Make sure there aren't more than max_tags."""
    git = '/usr/bin/git'
    ensure_dir(location)
    with utils.cd(location):
        cmd = [
            git,
            'for-each-ref',
            '--sort=taggerdate',
            '--format=%(refname)',
            'refs/tags'
        ]

        tags = subprocess.check_output(cmd).splitlines()
        old_tags = []
        while len(tags) > max_tags:
            tag = tags.pop(0)
            if tag.startswith('refs/tags/'):
                tag = tag[10:]

            # Don't delete tags that aren't ours
            if not tag.startswith(TAG_PREFIX):
                continue

            old_tags.append(tag)

        # if there aren't any old tags, bail early
        if len(old_tags) == 0:
            return

        cmd = [git, 'tag', '-d']
        cmd += old_tags
        subprocess.check_call(cmd)


def garbage_collect(location):
    """Clean up a repo."""
    git = '/usr/bin/git'
    ensure_dir(location)
    with utils.cd(location):
        cmd = [git, 'gc', '--quiet', '--auto']
        subprocess.check_call(cmd)


def add_all(location, message='Update'):
    """Add everything to repo at location as user."""
    git = '/usr/bin/git'
    with utils.cd(location):
        # Initialize repo if it isn't already
        if not is_dir(location):
            cmd = [git, 'init']
            subprocess.check_call(cmd)

        cmd = [git, 'add', '--all']
        subprocess.check_call(cmd)

        # None of these values can be unset or empty strings because we use
        # them as git envvars below. Unset values and empty strings will
        # cause git to shout about ident errors.
        host = socket.getfqdn() or 'localhost'
        euid = utils.get_username() or 'unknown'
        ruid = utils.get_real_username() or 'unknown'
        ename = utils.get_user_fullname() or 'Unknown User'
        rname = utils.get_real_user_fullname() or 'Unknown User'

        os.environ['GIT_COMMITTER_EMAIL'] = '{}@{}'.format(euid, host)
        os.environ['GIT_AUTHOR_EMAIL'] = '{}@{}'.format(ruid, host)

        os.environ['GIT_COMMITTER_NAME'] = ename
        os.environ['GIT_AUTHOR_NAME'] = rname

        cmd = [git, 'commit', '--quiet', '-m', message]

        # Soft errors if nothing new to commit
        subprocess.call(cmd)


def next_deploy_tag(location):
    """Calculates the scap/sync/{date}/{n} tag to use for this deployment"""
    ensure_dir(location)
    with utils.cd(location):
        timestamp = datetime.utcnow()
        date = timestamp.strftime('%F')
        cmd = ['/usr/bin/git', 'tag', '--list']
        tag_fmt = os.path.join(TAG_PREFIX, '{}', '*')
        cmd.append(tag_fmt.format(date))
        seq = len(subprocess.check_output(cmd).splitlines()) + 1
        tag_fmt = os.path.join(TAG_PREFIX, '{0}', '{1:04d}')
        return tag_fmt.format(date, seq)


def ensure_dir(location):
    """Ensure that we're in a git directory. If not, explode"""
    if location is None or not is_dir(location):
        raise IOError(errno.ENOENT, 'Location is not a git repo', location)


def is_dir(path):
    """Checks if path is a git, doesn't count submodule directories"""
    git_path = os.path.join(
        os.path.abspath(os.path.expandvars(os.path.expanduser(path))),
        '.git'
    )
    return (
        os.path.isdir(git_path) and
        os.path.isdir(os.path.join(git_path, 'objects')) and
        os.path.isdir(os.path.join(git_path, 'refs')) and
        os.path.isfile(os.path.join(git_path, 'HEAD')))


def remote_exists(location, remote):
    """Check if remote exists in location"""
    ensure_dir(location)
    with utils.cd(location):
        cmd = '/usr/bin/git config --local --get remote.{}.url'.format(remote)
        return subprocess.call(cmd, shell=True) == 0


def remote_set(location, repo, remote='origin'):
    """set the remote at location to repo"""
    ensure_dir(location)
    with utils.cd(location):
        if remote_exists(location, remote):
            cmd = '/usr/bin/git remote rm {}'.format(remote)
            subprocess.check_call(cmd, shell=True)

        cmd = '/usr/bin/git remote add {} {}'.format(remote, repo)
        subprocess.check_call(cmd, shell=True)


def fetch(location, repo, reference=None, dissociate=True):
    """Fetch a git repo to a location
    """
    if is_dir(location):
        remote_set(location, repo)
        with utils.cd(location):
            cmd = ['/usr/bin/git', 'fetch']
            subprocess.check_call(cmd)
    else:
        cmd = ['/usr/bin/git', 'clone', repo, location]
        if reference is not None:
            ensure_dir(reference)
            cmd.append('--reference')
            cmd.append(reference)
            if dissociate:
                cmd.append('--dissociate')
        subprocess.check_call(cmd)


def checkout(location, rev):
    """Checkout a git repo sha at a location
    """
    ensure_dir(location)

    logger = utils.get_logger()

    with utils.cd(location):
        logger.debug('Checking out rev: %s at location: %s', rev, location)
        cmd = '/usr/bin/git checkout --force --quiet {}'.format(rev)
        subprocess.check_call(cmd, shell=True)


def sync_submodules(location):
    """Sync git submodules on target machines"""

    ensure_dir(location)

    logger = utils.get_logger()

    with utils.cd(location):
        logger.debug('Syncing out submodules')
        cmd = '/usr/bin/git submodule sync --recursive'
        subprocess.check_call(cmd, shell=True)


def update_submodules(location, git_remote=None, use_upstream=False):
    """Update git submodules on target machines"""

    if not use_upstream and git_remote is None:
        raise ValueError("Must set git_remote if not using upstream")

    ensure_dir(location)

    logger = utils.get_logger()

    with utils.cd(location):
        logger.debug('Checking out submodules')
        if not use_upstream:
            remap_submodules(location, git_remote)
        cmd = '/usr/bin/git submodule update --init --recursive'
        subprocess.check_call(cmd, shell=True)


@utils.log_context('git_update_server_info')
def update_server_info(has_submodules=False, location=os.getcwd(),
                       logger=None):
    """runs git update-server-info and tags submodules"""

    with utils.cd(location):
        cmd = '/usr/bin/git update-server-info'
        subprocess.check_call(cmd, shell=True)
        logger.debug('Update server info')

        if has_submodules:
            cmd = "/usr/bin/git submodule foreach --recursive '{}'".format(cmd)
            subprocess.check_call(cmd, shell=True)


def update_deploy_head(deploy_info, location):
    """updates .git/DEPLOY_HEAD file

    :param deploy_info: current deploy info to write to file as YAML
    :param (optional) location: git directory location (default cwd)
    """
    logger = utils.get_logger()
    ensure_dir(location)

    with utils.cd(location):
        deploy_file = os.path.join(location, '.git', 'DEPLOY_HEAD')
        logger.debug('Creating %s', deploy_file)
        with open(deploy_file, 'w+') as deployfile:
            deployfile.write(yaml.dump(deploy_info, default_flow_style=False))
            deployfile.close()


def tag_repo(deploy_info, location=os.getcwd()):
    """creates new tag in deploy repo"""

    ensure_dir(location)
    with utils.cd(location):
        cmd = """
        /usr/bin/git tag -fa \\
                -m 'user {0}' \\
                -m 'timestamp {1}' -- \\
                {2} {3}
        """.format(
            deploy_info['user'],
            deploy_info['timestamp'],
            deploy_info['tag'],
            deploy_info['commit']
        )
        subprocess.check_call(cmd, shell=True)


def remap_submodules(location, server):
    """Remap all submodules to new server (tin)

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

    with utils.cd(location):
        gitmodule = os.path.join(os.getcwd(), '.gitmodules')
        if not os.path.isfile(gitmodule):
            return

        logger.info('Updating .gitmodule: %s', os.path.dirname(gitmodule))

        # ensure we're working with a non-modified .gitmodules file
        subprocess.check_call(['/usr/bin/git', 'checkout', '.gitmodules'])

        # get .gitmodule info
        modules = subprocess.check_output([
            '/usr/bin/git', 'config', '--list', '--file', '.gitmodules'])

        submodules = {}
        for line in modules.split('\n'):
            if not line.startswith('submodule.'):
                continue

            module_conf = line.split('=')
            module_name = module_conf[0].strip()

            if module_name.endswith('.path'):
                name = module_name[len('submodule.'):-len('.path')]
                submodules[name] = module_conf[1].strip()

        with open(gitmodule, 'w') as module:
            for submodule_name, submodule_path in submodules.iteritems():
                # Since we're using a non-bare http remote, map the submodule
                # to the submodule path under $GIT_DIR/modules subdirectory of
                # the superproject (git documentation: https://git.io/v4W9F).
                remote_path = '{}/modules/{}'.format(server, submodule_name)
                module.write('[submodule "{}"]\n'.format(submodule_name))
                module.write('\tpath = {}\n'.format(submodule_path))
                module.write('\turl = {}\n'.format(remote_path))


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
    with open(os.devnull, 'wb') as dev_null:
        try:
            return subprocess.check_output(
                ('/usr/bin/git', 'rev-list', '-1', '@{upstream}'),
                cwd=repo_directory, stderr=dev_null).strip()
        except subprocess.CalledProcessError:
            try:
                if not re.match('[a-f0-9]{40}', remote_thing):
                    remote = subprocess.check_output(
                        ('/usr/bin/git', 'remote'),
                        cwd=repo_directory, stderr=dev_null).strip()
                    remote_thing = '%s/%s' % (remote, remote_thing)
                return subprocess.check_output(
                    ('/usr/bin/git', 'merge-base', 'HEAD', remote_thing),
                    cwd=repo_directory, stderr=dev_null).strip()
            except subprocess.CalledProcessError:
                utils.get_logger().info(
                    'Unable to find remote tracking branch/tag for %s',
                    repo_directory)
                return ''


def list_submodules(repo):
    """List all of the submodules of a given respository"""
    ensure_dir(repo)
    submodules = []
    res = subprocess.check_output(
        ('/usr/bin/git', 'submodule', 'status'), cwd=repo)
    for line in res.splitlines():
        submodules.append(re.sub(r'-[a-f0-9]{40} ', '', line))
    return submodules


def reflog(repo, fmt='oneline', branch=None):
    """
    Fetch reflog as list
    """
    cmd = [
        '/usr/bin/git',
        '-C', repo,
        'log', '--walk-reflogs',
        '--format={}'.format(fmt)]

    if branch is not None:
        cmd.append(branch)

    return subprocess.check_output(cmd).splitlines()
