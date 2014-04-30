# -*- coding: utf-8 -*-
"""
    scap.utils
    ~~~~~~~~~~
    Contains misc utility functions.

"""
import contextlib
import errno
import fcntl
import hashlib
import logging
import os
import pwd
import random
import re
import socket
import struct
import subprocess


def read_dsh_hosts_file(path):
    """Reads hosts from a file into a list.

    Blank lines and comments are ignored.
    """
    try:
        with open(os.path.join('/etc/dsh/group', path)) as hosts_file:
            return re.findall(r'^[\w\.\-]+', hosts_file.read(), re.MULTILINE)
    except IOError, e:
        raise IOError(e.errno, e.strerror, path)


class LockFailedError(Exception):
    """Signal that a locking attempt failed."""
    pass


@contextlib.contextmanager
def lock(filename):
    """Context manager. Acquires a file lock on entry, releases on exit.

    :param filename: File to lock
    :raises: LockFailedError on failure
    """
    lock_fd = None
    try:
        lock_fd = open(filename, 'w+')
        fcntl.lockf(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except IOError as e:
        raise LockFailedError('Failed to lock %s: %s' % (filename, e))
    else:
        yield
    finally:
        if lock_fd:
            fcntl.lockf(lock_fd, fcntl.LOCK_UN)
            lock_fd.close()


def human_duration(elapsed):
    """Format an elapsed seconds count as human readable duration.

    >>> human_duration(1)
    '00m 01s'
    >>> human_duration(65)
    '01m 05s'
    >>> human_duration(60*30+11)
    '30m 11s'
    """
    return '%02dm %02ds' % divmod(elapsed, 60)


def find_nearest_host(hosts, port=22, timeout=1):
    """Given a collection of hosts, find the one that is the fewest
    number of hops away.

    >>> # Begin test fixture
    >>> import socket
    >>> fixture_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    >>> fixture_socket.bind(('127.0.0.1', 0))
    >>> fixture_socket.listen(1)
    >>> fixture_port = fixture_socket.getsockname()[1]
    >>> # End test fixture
    >>> find_nearest_host(['127.0.0.1'], port=fixture_port)
    '127.0.0.1'

    :param hosts: Hosts to check
    :param port: Port to try to connect on (default: 22)
    :param timeout: Timeout in seconds (default: 1)
    """
    host_map = {}
    for host in hosts:
        try:
            host_map[host] = socket.getaddrinfo(host, port)[0]
        except socket.gaierror:
            continue

    for ttl in range(1, 30):
        if not host_map:
            break
        for host, info in random.sample(host_map.items(), len(host_map)):
            family, type, proto, _, addr = info
            s = socket.socket(family, type, proto)
            s.setsockopt(socket.IPPROTO_IP, socket.IP_TTL,
                         struct.pack('I', ttl))
            s.settimeout(timeout)
            try:
                s.connect(addr)
            except socket.error as e:
                if e.errno != errno.EHOSTUNREACH:
                    del host_map[host]
                continue
            except socket.timeout:
                continue
            else:
                return host
            finally:
                s.close()


def get_realm_specific_filename(filename, realm, datacenter):
    """Find the most specific file for the given realm and datacenter.

    The extension is separated from the filename and then recombined with the
    realm and datacenter:
    - base-realm-datacenter.ext
    - base-realm.ext
    - base-datacenter.ext
    """
    base, ext = os.path.splitext(filename)

    if ext == '':
        return filename

    parts = {
        'base': base,
        'realm': realm,
        'datacenter': datacenter,
        'ext': ext,
    }

    possible = (
        '%(base)s-%(realm)s-%(datacenter)%(ext)s',
        '%(base)s-%(realm)s%(ext)s',
        '%(base)s-%(datacenter)%(ext)s',
    )

    for new_filename in (p % parts for p in possible):
        if os.path.isfile(new_filename):
            return new_filename

    # If all else fails, return the original filename
    return filename


def get_username():
    """Get the username of the effective user."""
    return pwd.getpwuid(os.getuid())[0]


def get_real_username():
    """Get the username of the real user."""
    try:
        # Get the username of the user owning the terminal (ie the user
        # that is running scap even if they are sudo-ing something)
        return os.getlogin()
    except OSError:
        # When running under Jenkins there is no terminal so os.getlogin()
        # blows up. Use the username matching the effective user id
        # instead.
        return get_username()


def md5_file(path):
    """Compute the md5 checksum of a file's contents.

    :param path: Path to file
    :returns: hexdigest of md5 checksum
    """
    crc = hashlib.md5()
    with open(path, 'rb') as f:
        # Digest file in 1M chunks just in case it's huge
        for block in iter(lambda: f.read(1048576), b''):
            crc.update(block)
    return crc.hexdigest()


def sudo_check_call(user, cmd, logger=None):
    """Run a command as a specific user. Reports stdout/stderr of process
    to logger during execution.

    :param user: User to run command as
    :param cmd: Command to execute
    :param logger: Logger to send process output to
    :raises: subprocess.CalledProcessError on non-zero process exit
    """
    if logger is None:
        logger = logging.getLogger('sudo_check_call')

    proc = subprocess.Popen('sudo -u %s -- %s' % (user, cmd),
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True)

    while proc.poll() is None:
        line = proc.stdout.readline().strip()
        if line:
            logger.debug(line)

    if proc.returncode:
        raise subprocess.CalledProcessError(proc.returncode, cmd)


def iterate_subdirectories(root):
    for name in os.listdir(root):
        subdir = os.path.join(root, name)
        if os.path.isdir(subdir):
            yield subdir


def git_info_filename(directory, install_path, cache_path):
    """Compute the path for a git_info cache file related to a given
    directory.

    >>> git_info_filename('foo', 'foo', '')
    'info.json'
    >>> git_info_filename('foo/bar/baz', 'foo', 'xyzzy')
    'xyzzy/info-bar-baz.json'
    """
    path = directory
    if path.startswith(install_path):
        path = path[len(install_path):]
    return os.path.join(cache_path, 'info%s.json' % path.replace('/', '-'))


def git_info(directory):
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
        with open(git_dir, 'r') as f:
            git_ref = f.read().strip()

        if not git_ref.startswith('gitdir: '):
            raise IOError(errno.EINVAL, 'Unexpected .git contents', git_dir)
        git_ref = git_ref[8:]
        if git_ref[0] != '/':
            git_ref = os.path.abspath(os.path.join(directory, git_ref))
        git_dir = git_ref

    head_file = os.path.join(git_dir, 'HEAD')
    with open(head_file, 'r') as f:
        head = f.read().strip()
    if head.startswith('ref: '):
        head = head[5:]

    if re.match(r'^[0-9a-f]{40}$', head, re.IGNORECASE):
        # Working copy is a detached head, so we can't check for upstream
        # intersection easily.
        head_sha1 = head
    else:
        # Find first commit shared with the upstream branch. This keeps us
        # from leaking information about locally committed changes such as
        # security patches.
        head_sha1 = subprocess.check_output(
            ('/usr/bin/git', 'rev-list', '-1', '@{upstream}'),
            cwd=git_dir).strip()

    commit_date = subprocess.check_output(
        ('/usr/bin/git', 'show', '-s', '--format=%ct', head_sha1),
        cwd=git_dir).strip()

    if head.startswith('refs/heads/'):
        branch = head[11:]
    else:
        branch = head

    # Requires git v1.7.5+
    remote_url = subprocess.check_output(
        ('/usr/bin/git', 'ls-remote', '--get-url'),
        cwd=git_dir).strip()

    return {
        '@directory': directory,
        'head': head,
        'headSHA1': head_sha1,
        'headCommitDate': commit_date,
        'branch': branch,
        'remoteURL': remote_url,
    }
