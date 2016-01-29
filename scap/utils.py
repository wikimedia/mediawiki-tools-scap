# -*- coding: utf-8 -*-
"""
    scap.utils
    ~~~~~~~~~~
    Contains misc utility functions.

"""
import collections
import contextlib
import distutils.version
import errno
import fcntl
import glob
import hashlib
import inspect
import json
import logging
import os
import pwd
import random
import socket
import struct
import subprocess
import sys
import tempfile

from . import ansi
from functools import wraps


def eintr_retry(func, *args):
    """Retry a system call if it is interrupted by EINTR.

    Extracted from stdlib's subprocess (where it is called `_eintr_retry_call`
    -- the leading underscore indicating it is not part of the module's API).
    This is not needed on Python >= 3.5, thanks to PEP 0475.

    See <https://www.python.org/dev/peps/pep-0475/>."""
    while True:
        try:
            return func(*args)
        except (OSError, IOError) as e:
            if e.errno == errno.EINTR:
                continue
            raise


class LockFailedError(Exception):
    """Signal that a locking attempt failed."""
    pass


def ask(question, default):
    """Provides a y/n prompt if the controlling terminal is interactive.

    :param question: Prompt message to display
    :param default: Default answer to use in the case of a non-interactive
                    terminal
    :returns: str User input or default value
    """

    if not sys.stdout.isatty():
        return default

    ans = raw_input('{} [{}]: '.format(question, default)).strip()
    return ans.lower() if ans else default


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


def get_env_specific_filename(path, env=None):
    """Find a file specific to the environment in which scap is running"""
    if env is None:
        return path

    base = os.path.dirname(path)
    filename = os.path.basename(path)

    if base.endswith('/templates'):
        base = os.path.dirname(base)
        filename = os.path.join('templates', filename)

    env_filename = os.path.join(base, 'environments', env, filename)

    if os.path.isfile(env_filename):
        return env_filename

    return path


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


def iterate_subdirectories(root):
    """Generator over the child directories of a given directory."""
    for name in os.listdir(root):
        subdir = os.path.join(root, name)
        if os.path.isdir(subdir):
            yield subdir


logger_stack = []


@contextlib.contextmanager
def context_logger(context_name, *args):
    """
    context_logger is a context manager that maintains nested logger
    contexts. Each time you enter a with block using this context manager,
    a named logger is set up as a child of the current logger.
    When exiting the with block, the logger gets popped off the stack and
    the parent logger takes it's place as the 'current' logging context.

    The easiest way to use this is to decorate a function with log_context,
    For Example::

        @log_context('name')
        def my_func(some, args, logger=None):
            logger.debug('something')

    """
    if len(logger_stack) < 1:
        logger_stack.append(logging.getLogger())

    parent = logger_stack[-1]

    logger = parent.getChild(context_name)
    logger_stack.append(logger)
    try:
        yield logger
    finally:
        logger_stack.pop()


def log_context(context_name):
    """Decorator to wrap the a function in a new context_logger,
       the logger is passed to the function via a kwarg named 'logger'"""
    def arg_wrapper(func):
        @wraps(func)
        def context_wrapper(*args, **kwargs):
            argspec = inspect.getargspec(func)

            # Check if logger was passed as a positional argument
            try:
                l = args[argspec.args.index('logger')]
            except IndexError:
                l = None

            # Check if logger was passed as a keyword argument
            if l is None:
                l = kwargs.get('logger', None)

            if l is not None:
                return func(*args, **kwargs)

            with context_logger(context_name) as logger:
                kwargs['logger'] = logger
                return func(*args, **kwargs)
        return context_wrapper
    return arg_wrapper


def get_logger():
    if len(logger_stack) > 0:
        return logger_stack[-1]
    else:
        return logging.getLogger()


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
            if os.path.exists(filename):
                os.unlink(filename)


@contextlib.contextmanager
def cd(dirname):
    """Context manager. Cds to dirname, moves back to previous dir on context exit

    :param dirname: directory into which it should change
    """
    old_path = os.getcwd()
    try:
        os.chdir(dirname)
        yield
    finally:
        os.chdir(old_path)


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


@log_context('sudo_check_call')
def sudo_check_call(user, cmd, logger=None):
    """Run a command as a specific user. Reports stdout/stderr of process
    to logger during execution.

    :param user: User to run command as
    :param cmd: Command to execute
    :param logger: Logger to send process output to
    :raises: subprocess.CalledProcessError on non-zero process exit
    """

    proc = subprocess.Popen('sudo -u %s -n -- %s' % (user, cmd),
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            shell=True)

    fullOut = []
    while proc.poll() is None:
        line = proc.stdout.readline().strip()
        if line:
            logger.debug(line)
            fullOut.append(line)

    # Consumes rest of stdout
    leftover = proc.stdout.readlines()
    map(logger.debug, leftover)

    if proc.returncode:
        logger.error("Last output:\n" + '\n'.join(fullOut + leftover))
        raise subprocess.CalledProcessError(proc.returncode, cmd)


def check_valid_json_file(path):
    if not path.endswith('.json'):
        return
    with open(path) as f:
        try:
            json.load(f)
        except ValueError:
            raise ValueError(
                '%s is an invalid JSON file'
                % path
            )


def check_file_exists(path, message=False):
    if not os.path.isfile(path):
        raise IOError(
            errno.ENOENT,
            message or 'Error: %s is not a file.' % path,
            path
        )


def check_dir_exists(path, message=False):
    if not os.path.isdir(path):
        raise IOError(
            errno.ENOTDIR,
            message or 'Error: %s is not a directory.' % path,
            path
        )


def check_php_opening_tag(path):
    """Checks a PHP file to make sure nothing is before the opening <?php
    except for shebangs.

    :param path: Location of file
    :raises: ValueError on invalid file
    """
    if not path.endswith(('.php', '.inc', '.phtml', '.php5')):
        return
    with open(path) as f:
        text = f.read()

        # Empty files are ok
        if len(text) < 1:
            return

        # Best case scenario to begin with the php open tag
        if text.lower().startswith('<?php'):
            return

        # Also reasonable to start with a doctype declaration
        if text.startswith('<!DOCTYPE'):
            return

        # If the first line is a shebang and the
        # second has <?php, that's ok
        lines = text.splitlines()

        if (
            len(lines) > 1 and
            lines[0].startswith('#!') and
            lines[1].lower().startswith('<?php')
        ):
            return

        # None of the return conditions matched, the file must contain <?php
        # but with some content preceeding it.
        raise ValueError(
            '%s has content before opening <?php tag'
            % path
        )


def logo(color=True, **colors):
    """Get the scap logo::

               ___ ____
             ⎛   ⎛ ,----
              \  //==--'
         _//| .·//==--'    ____________________________
        _OO≣=-  ︶ ᴹw ⎞_§ ______  ___\ ___\ ,\__ \/ __ \
       (∞)_, )  (     |  ______/__  \/ /__ / /_/ / /_/ /
         ¨--¨|| |- (  / _______\____/\___/ \__^_/  .__/
             ««_/  «_/ jgs/bd808               /_/

    Ascii art derived from original work by Joan Stark [#]_ and the `speed`
    figlet font [#]_.

    :param color: Color logo using ANSI escapes
    :param colors: Alternate colors
    :returns: str

    .. [#] http://www.oocities.org/spunk1111/farm.htm#pig
    .. [#] http://www.jave.de/figlet/fonts/details/speed.html
    """
    pallet = {
        'pig': ansi.reset() + ansi.esc(ansi.FG_MAGENTA, ansi.BRIGHT),
        'nose': ansi.reset() + ansi.esc(ansi.FG_MAGENTA, ansi.BRIGHT),
        'mouth': ansi.reset() + ansi.esc(ansi.FG_MAGENTA, ansi.BRIGHT),
        'goggles': ansi.reset() + ansi.esc(ansi.FG_YELLOW),
        'brand': ansi.reset(),
        'hoof': ansi.reset() + ansi.esc(ansi.FG_BLUE),
        'wing': ansi.reset() + ansi.esc(ansi.FG_CYAN),
        'speed': ansi.reset() + ansi.esc(ansi.FG_WHITE),
        'text': ansi.reset() + ansi.esc(ansi.FG_GREEN),
        'signature': ansi.reset() + ansi.esc(ansi.FG_BLUE),
        'reset': ansi.reset(),
    }
    pallet.update(colors)

    if not color:
        for key in pallet.keys():
            pallet[key] = ''

    return ''.join(line % pallet for line in [
        '''           %(wing)s___%(reset)s %(wing)s____%(reset)s\n''',
        '''         %(wing)s⎛   ⎛ ,----%(reset)s\n''',
        '''          %(wing)s\  //==--'%(reset)s\n''',
        '''     %(pig)s_//|,.·%(wing)s//==--'%(reset)s    ''',
        '''%(speed)s______%(text)s____''',
        '''%(speed)s_%(text)s____''',
        '''%(speed)s___%(text)s____''',
        '''%(speed)s__%(text)s____%(reset)s\n''',

        '''    %(pig)s_%(goggles)sOO≣=-%(pig)s ''',
        ''' %(wing)s︶%(pig)s %(brand)sᴹw%(pig)s ⎞_§%(reset)s ''',
        '''%(speed)s______%(text)s  ___\ ___\ ,\__ \/ __ \%(reset)s\n''',
        '''   %(pig)s(%(nose)s∞%(pig)s)%(mouth)s_,''',
        '''%(pig)s )  (     |%(reset)s''',
        '''  %(speed)s______%(text)s/__  \/ /__ / /_/ / /_/ /%(reset)s\n''',
        '''     %(pig)s¨--¨|| |- (  /%(reset)s''',
        ''' %(speed)s______%(text)s\____/ \___/ \__^_/  .__/%(reset)s\n''',
        '''         %(hoof)s««%(pig)s_/%(reset)s''',
        '''  %(hoof)s«%(pig)s_/%(reset)s''',
        ''' %(signature)sjgs/bd808%(reset)s''',
        '''                %(text)s/_/%(reset)s\n''',
    ])


@contextlib.contextmanager
def sudo_temp_dir(owner, prefix):
    """Create a temporary directory and delete it after the block.

    :param owner: Directory owner
    :param prefix: Temp directory prefix
    :returns: Full path to temporary directory
    """

    while True:
        dirname = os.path.join(
            tempfile.gettempdir(),
            prefix + str(random.SystemRandom().randint(0, 0xffffffff)))
        if not os.path.exists(dirname):
            break
    # Yes, there is a small race condition here in theory. In practice it
    # should be pretty hard to hit due to scap's global concurrency lock.
    sudo_check_call(owner, 'mkdir "%s"' % dirname)

    try:
        yield dirname
    finally:
        sudo_check_call(owner, 'find "%s" -maxdepth 1 -delete' % dirname)


def read_pid(path):
    """Read a PID from a file"""
    try:
        return int(open(path).read().strip())
    except IOError as e:
        raise IOError(e.errno, e.strerror, path)


def mkdir_p(path, user=get_real_username(), logger=None):
    sudo_check_call(user, "mkdir -p '{}'".format(path))


def move_symlink(source, dest, user=get_real_username()):
    if os.path.realpath(dest) == source:
        return

    dest_dir = os.path.dirname(dest)
    rsource = os.path.relpath(source, dest_dir)
    rdest = os.path.relpath(dest, dest_dir)

    # Make link target's parent directory if it doesn't exist
    mkdir_p(dest_dir, user=user)

    with cd(dest_dir):
        sudo_check_call(user, "ln -sf '{}' '{}'".format(rsource, rdest))


def remove_symlink(path, user=get_real_username()):
    sudo_check_call(user, "rm '{}'".format(path))


def get_active_wikiversions(directory, realm, datacenter):
    """Get an ordered collection of active MediaWiki versions.

    :returns: collections.OrderedDict of {version:wikidb} values sorted by
                version number in ascending order
    """
    path = get_realm_specific_filename(
        os.path.join(directory, 'wikiversions.json'), realm, datacenter)

    with open(path) as f:
        wikiversions = json.load(f)

    versions = {}
    for wikidb, version in wikiversions.items():
        version = version[4:]  # trim 'php-' from version
        if version not in versions:
            versions[version] = wikidb

    # Convert to list of (version, db) tuples sorted by version number
    # and then convert that list to an OrderedDict
    sorted_versions = collections.OrderedDict(
        sorted(versions.iteritems(),
               key=lambda v: distutils.version.LooseVersion(v[0])))

    return sorted_versions


def uniqify(seq, idfun=None):
    '''
    return a copy of list with duplicate values removed, preserving order
    :param seq: a list of items with possible duplicate values
    :returns: list of values from seq, minus any duplicates
    '''
    # order preserving
    if idfun is None:
        def idfun(x):
            return x
    seen = {}
    result = []
    for item in seq:
        marker = idfun(item)
        if marker in seen:
            continue
        seen[marker] = 1
        result.append(item)
    return result


def find_upwards(name, starting_point=os.getcwd()):
    """
    Search the specified directory, and all parent directories, for a given
    filename, returning the first matching path that is found.

    :param name: the relative path name to search for
    :param starting_point: the directory to start searching from.
                           Each parent directory will be searched until a
                           match is found.
    :return: if a match is found, returns the full path to the matching
             file, None otherwise.
    """
    current = os.path.abspath(starting_point)
    while True:
        if not os.path.exists(current):
            return None

        search = join_path(current, name)
        if os.path.exists(search):
            return search

        parent = os.path.dirname(current)
        if parent == current or parent == "/":
            return None
        current = parent


def find_in_path(name, paths=[os.getcwd()], match_func=os.path.exists):
    """
     Search a list of directories for a given file

     :param name: the relative path name to search for
     :param paths: a list of paths to be searched
     :param match_func: a function which is called to test each candidate
                        defaults to testing for path existence only.
     """
    for path in paths:
        candidate = join_path(path, name)
        if match_func(candidate):
            return candidate
    return None


def find_dir(name, paths=[os.getcwd()]):
    """
    like find_in_path for directories only

    :param name: the relative path name to search for
    :param paths: a list of paths to be searched
    """
    return find_in_path(name, match_func=os.path.isdir)


def find_file(name, paths=[os.getcwd()]):
    """
    like find_in_path for files only

    :param name: the relative path name to search for
    :param paths: a list of paths to be searched
    """
    return find_in_path(name, paths=paths, match_func=os.path.isfile)


def join_path(*fragments):
    """
    Join several path fragments into a complete, normalized path string.
    Strips leading and trailing slashes from path fragments to avoid an
    unfortunate feature of `os.path.join()` which is described in the
    python documentation for `os.path` as follows:

      "If any component is an absolute path, all previous components are
      thrown away, and joining continues."

      - https://docs.python.org/2/library/os.path.html#os.path.join
    """
    path = []
    for p in fragments:
        if len(path) > 0:
            p = p.strip('\t\r\n/')
        if len(p) > 0:
            path.append(p)

    path_str = os.path.join(*path)
    return os.path.normpath(path_str)


def get_patches(sub_dirs, root_dir):
    """ Find all patches under each subdirectory
    :param sub_dirs list of sub directories under which to search
    :param root_dir base path under which subdirectories reside
    :return dictionary of patches, keyed by sub_dir
    """
    patches = {}
    for sub_dir in sub_dirs:
        for patch_file in sorted(
                glob.glob(os.path.join(root_dir, sub_dir, '*.patch')),
                reverse=True):

            with open(patch_file, 'r') as f:
                patches.setdefault(sub_dir, []).append(f.read())

    return patches
