# -*- coding: utf-8 -*-
"""
    scap.utils
    ~~~~~~~~~~
    Contains misc utility functions.

"""
import contextlib
import errno
import fcntl
import imp
import os
import pipes
import random
import re
import shlex
import socket
import struct
import subprocess


def shell_map(mapping):
    """Convert a dict to a list of KEY=VALUE pairs.

    >>> shell_map({'a':'b'})
    ['a=b']
    >>> shell_map({'a':'1', 'b':'2 or 3'})
    ['a=1', "b='2 or 3'"]
    """
    return ['%s=%s' % (k, pipes.quote(v)) for k, v in mapping.items()]


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
        command = '%s %s' % (' '.join(shell_map(exports)), command)
    group_file = os.path.join('/etc/dsh/group', group)
    return subprocess.call(['/usr/bin/dsh', '-F40', '-cM', '-f',
                            group_file, '-o', '-oSetupTimeout=10', '--',
                            command.strip()])


def build_command(command, exports=None):
    """Build an argument list for running a command.

    If the command is a string rather than a sequence it will be split using
    ``shlex.split`` as recommended by the ``subprocess`` documentation. If
    a dict of environment exports is provided it will be converted into
    VAR=VALUE statements and prepended to the command.

    >>> build_command('hello')
    ['hello']
    >>> build_command(['hello'])
    ['hello']
    >>> build_command(['hello'] + 'foo bar baz'.split())
    ['hello', 'foo', 'bar', 'baz']
    >>> build_command('hello --who world')
    ['hello', '--who', 'world']
    >>> build_command('echo "hello world"')
    ['echo', 'hello world']
    >>> build_command(['scap-2', 'tin'], {'FOO':'bar baz'})
    ["FOO='bar baz'", 'scap-2', 'tin']

    :param command: Command to execute
    :type command: str or sequence
    :param exports: Environment variables to export to the command
    :type exports: dict
    :returns: List of arguments suitable for use with subprocess methods
    """
    args = []
    if exports:
        args.extend(shell_map(exports))
    try:
        command = shlex.split(command)
    except AttributeError:
        # Command is already a sequence
        pass
    args.extend(command)
    return args


def sudo_args(command, user=None, exports=None):
    """Build an argument list for running a command under sudo.

    The command and optional exports will be processed by
    :func:`build_command` and then used as the object of a ``sudo``
    invocation.

    >>> sudo_args('hello')
    ['sudo', 'hello']
    >>> sudo_args(['hello'])
    ['sudo', 'hello']
    >>> sudo_args(['hello'] + 'foo bar baz'.split())
    ['sudo', 'hello', 'foo', 'bar', 'baz']
    >>> sudo_args(['scap-2', 'tin'], 'wmdeploy', {'FOO':'bar baz'})
    ['sudo', '-u', 'wmdeploy', "FOO='bar baz'", 'scap-2', 'tin']

    :param command: Command to execute
    :type command: str or sequence
    :param user: User to execute as
    :type user: str
    :param exports: Environment variables to export to the command
    :type exports: dict
    :returns: List of arguments suitable for use with subprocess methods

    .. seealso:: :func:`build_command`
    """
    args = ['sudo']
    if user is not None:
        args.extend(['-u', user])
    args.extend(build_command(command, exports))
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


def find_nearest_host(hosts, port=22, timeout=1):
    """Given a collection of hosts, find the one that is the fewest
    number of hops away.

    >>> find_nearest_host(['localhost'])
    'localhost'

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
