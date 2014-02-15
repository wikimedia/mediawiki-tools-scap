# -*- coding: utf-8 -*-
"""
    scap.utils
    ~~~~~~~~~~
    Contains misc utility functions.

"""
import contextlib
import fcntl
import imp
import os
import pipes
import re
import struct
import subprocess


def shell_map(mapping):
    """Convert a map to a string of space-separated KEY=VALUE pairs.

    >>> shell_map({'a':'b'})
    'a=b'
    >>> shell_map({'a':'1', 'b':'2 or 3'})
    "a=1 b='2 or 3'"
    """
    return ' '.join('%s=%s' % (k, pipes.quote(v)) for k, v in mapping.items())


def read_dsh_hosts_file(path):
    """Reads hosts from a file into a list.

    Blank lines and comments are ignored.
    """
    try:
        with open(os.path.join('/etc/dsh/group', path)) as hosts_file:
            return re.findall(r'^[\w\.]+', hosts_file.read(), re.MULTILINE)
    except IOError, e:
        raise IOError(e.errno, e.strerror, path)


def get_config(cfg_file='/usr/local/lib/mw-deployment-vars.sh'):
    """Load environment variables from mw-deployment-vars.sh."""
    try:
        dep_env = imp.load_source('__env', cfg_file)
    except IOError, e:
        raise IOError(e.errno, e.strerror, cfg_file)
    else:
        return {k: v for k, v in dep_env.__dict__.items()
                if k.startswith('MW_')}


@contextlib.contextmanager
def lock(filename):
    """Context manager. Acquires a file lock on entry, releases on exit."""
    with open(filename, 'w+') as lock_fd:
        fcntl.lockf(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            yield
        finally:
            fcntl.lockf(lock_fd, fcntl.LOCK_UN)


def cdb_items(buf):
    """Iterates over CDB key/value pairs."""
    table_start, = struct.unpack_from('<L', buf)
    offset = 2048
    while offset < table_start:
        lengths = struct.unpack_from('<LL', buf, offset)
        offset += 8
        yield struct.unpack_from('%ds %ds' % lengths, buf, offset)
        offset += sum(lengths)


def get_branches(wikiversions_cdb_path):
    """Get the set of active branches from a wikiversions.cdb file."""
    with open(wikiversions_cdb_path, 'rb') as cdb_file:
        cdb = cdb_file.read()
        return {v.replace('php-', '') for k, v in cdb_items(cdb)
                if k.startswith('ver:')}


def dsh(command, group, exports=None):
    """Run a command on multiple hosts via DSH."""
    if exports:
        command = '%s %s' % (shell_map(exports), command)
    group_file = os.path.join('/etc/dsh/group', group)
    return subprocess.call(['/usr/bin/dsh', '-F40', '-cM', '-f',
                            group_file, '-o', '-oSetupTimeout=10', '--',
                            command.strip()])


def sudo_args(command, user=None, exports=None):
    """Build an argument list for running a command under sudo

    >>> sudo_args('hello')
    ['sudo', 'hello']
    >>> sudo_args(['hello'])
    ['sudo', 'hello']
    >>> sudo_args(['hello'] + 'foo bar baz'.split())
    ['sudo', 'hello', 'foo', 'bar', 'baz']
    >>> sudo_args(['scap-2', 'tin'], 'wmdeploy', {'FOO':'bar baz'})
    ['sudo', '-u', 'wmdeploy', "FOO='bar baz'", 'scap-2', 'tin']

    :param command: Command to execute
    :type command: str or list
    :param user: User to execute as
    :type user: str
    :param exports: Environment variables to export to the command
    :type exports: dict
    :returns: List of arguments suitable for use with subprocess methods
    """
    args = ['sudo']
    if user is not None:
        args.extend(['-u', user])
    if exports is not None:
        args.extend('%s=%s' % (k, pipes.quote(v)) for k, v in exports.items())
    if isinstance(command, basestring):
        func = args.append
        command = command.strip()
    else:
        func = args.extend
    func(command)
    return args


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
