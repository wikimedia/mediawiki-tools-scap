# -*- coding: utf-8 -*-
"""
    scap.utils
    ~~~~~~~~~~
    Contains misc utility functions.

"""
from __future__ import absolute_import
from __future__ import print_function

import collections
import contextlib
import distutils.version
import errno
from functools import wraps
import hashlib
import glob
import inspect
import json
from json import JSONEncoder
import logging
import math
import multiprocessing
import os
import pwd
import random
import re
import socket
import struct
import subprocess
import sys
import tempfile
import yaml

import pygments
import pygments.lexers
import pygments.formatters
import scap.ansi as ansi


BRANCH_RE = re.compile(
    r'(?P<major>\d{1}).'
    r'(?P<minor>\d{1,2}).'
    r'(?P<patch>\d{1,2})'
    r'-wmf.(?P<prerelease>\d{1,2})'
)


def isclose(a, b, rel_tol=1e-9, abs_tol=0.0):
    """
    Return True if a is close in value to b. False otherwise.

    :param a: one of the values to be tested
    :param b: the other value to be tested
    :param rel_tol=1e-9: The relative tolerance -- the amount of error
                         allowed, relative to the absolute value of the
                         larger input values.
    :param abs_tol=0.0: The minimum absolute tolerance level -- useful
                        for comparisons to zero.

    Copyright: Christopher H. Barker
    Original License: Apache License 2.0
    <https://github.com/PythonCHB/close_pep>
    """
    if a == b:  # short-circuit exact equality
        return True

    if rel_tol < 0.0 or abs_tol < 0.0:
        raise ValueError('error tolerances must be non-negative')

    # use cmath so it will work with complex ot float
    if math.isinf(abs(a)) or math.isinf(abs(b)):
        # This includes the case of two infinities of opposite sign, or
        # one infinity and one finite number. Two infinities of opposite sign
        # would otherwise have an infinite relative tolerance.
        return False
    diff = abs(b - a)

    return (
        ((diff <= abs(rel_tol * b)) or (diff <= abs(rel_tol * a))) or
        (diff <= abs_tol))


def eintr_retry(func, *args):
    """
    Retry a system call if it is interrupted by EINTR.

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


def ask(question, default, choices=None):
    """
    Provide a y/n prompt if the controlling terminal is interactive.

    :param question: Prompt message to display
    :param default: Default answer to use in the case of a non-interactive
                    terminal
    :param choices: Optional choices to present after the question. Defaults
                    to the default answer
    :returns: str User input or default value
    """

    if not sys.stdout.isatty():
        return default

    if choices is None:
        choices = '[{}]'.format(default)

    ans = raw_input('{} {}: '.format(question, choices)).strip()
    return ans.lower() if ans else default


def confirm(question='Continue?', default=False, on_fulfilled=None,
            on_rejected=None):
    """
    Ask for confirmation from the user if possible, otherwise return default
    when stdin is not attached to a terminal.

    The confirmation is fullfilled when the user types an affirming response
    which can be either 'y' or 'yes', otherwise the default choice is assumed

    The confirmation is rejected when default=False and the user types anything
    other than affirmative.

    :param question: prompt text to show to the user
    :param default: boolean default choice, True [Y/n] or False [y/N]. This is
                    the value that is returned when a tty is not attached to
                    stdin or the user presses enter without typing a response.
    :param on_fullfilled: optional callback function which is called before
                          returning True
    :param on_rejected: optional, either a callback function or an exception
                        to be raised when we fail to get confirmation. This
                        can be used to let the user bail out of a workflow or
                        to bail when execution is not attached to a terminal.

    """
    yes = ['y', 'yes']
    no = ['n', 'no']

    if default:
        choices = '[Y/n]'
    else:
        choices = '[y/N]'

    # in case stdin is not a tty or the user accepts the default answer, then
    # the result will be default.
    result = default

    if sys.stdout.isatty():
        ans = raw_input('{} {}: '.format(question, choices)).strip().lower()
        if ans in yes:
            result = True
        elif ans in no:
            result = False

    if result:
        # yes
        if callable(on_fulfilled):
            on_fulfilled()
    else:
        # no
        if isinstance(on_rejected, Exception):
            raise on_rejected
        elif callable(on_rejected):
            on_rejected()

    return result


def find_nearest_host(hosts, port=22, timeout=1):
    """
    Given a collection of hosts, find the one that is the fewest
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
            s.setsockopt(
                socket.IPPROTO_IP, socket.IP_TTL, struct.pack('I', ttl))
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


def get_real_user_fullname():
    """Return the first entry in GECOS field for real user."""
    return get_user_fullname(get_real_username())


def get_user_fullname(name=None):
    """Return the first entry in GECOS field for name."""
    if name is None:
        name = get_username()

    return pwd.getpwnam(name).pw_gecos.split(',')[0]


def get_env_specific_filename(path, env=None):
    """Find a file specific to the environment in which scap is running."""
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
    """
    Find the most specific file for the given realm and datacenter.

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


def get_username(user=None):
    """Get the username of the effective user."""
    if user is None:
        user = os.getuid()
    return pwd.getpwuid(user)[0]


def human_duration(elapsed):
    """
    Format an elapsed seconds count as human readable duration.

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


LOGGER_STACK = []


@contextlib.contextmanager
def context_logger(context_name, *args):
    """
    Context manager that maintains nested logger contexts.

    Each time you enter a with block using this context manager,
    a named logger is set up as a child of the current logger.
    When exiting the with block, the logger gets popped off the stack and
    the parent logger takes it's place as the 'current' logging context.

    The easiest way to use this is to decorate a function with log_context,
    For Example::

        @log_context('name')
        def my_func(some, args, logger=None):
            logger.debug('something')

    """
    if len(LOGGER_STACK) < 1:
        LOGGER_STACK.append(logging.getLogger())

    parent = LOGGER_STACK[-1]

    logger = parent.getChild(context_name)
    LOGGER_STACK.append(logger)
    try:
        yield logger
    finally:
        LOGGER_STACK.pop()


def log_context(context_name):
    """
    Decorator to wrap the a function in a new context_logger.

    The logger is passed to the function via a kwarg named 'logger'.
    """
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
    if LOGGER_STACK:
        return LOGGER_STACK[-1]
    return logging.getLogger()


@contextlib.contextmanager
def cd(dirname):
    """
    Context manager. Cds to dirname.

    It moves back to previous dir on context exit.
    :param dirname: directory into which it should change
    """
    old_path = os.getcwd()
    try:
        os.chdir(dirname)
        yield
    finally:
        os.chdir(old_path)


def md5_file(path):
    """
    Compute the md5 checksum of a file's contents.

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
    """
    Run a command as a specific user.

    Reports stdout/stderr of process to logger during execution.

    :param user: User to run command as
    :param cmd: Command to execute
    :param logger: Logger to send process output to
    :raises: subprocess.CalledProcessError on non-zero process exit
    """

    # Only sudo when necessary
    if user == get_username():
        fullCmd = cmd
    else:
        fullCmd = 'sudo -u %s -n -- %s' % (user, cmd)

    proc = subprocess.Popen(
        fullCmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True)

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
            raise ValueError('%s is an invalid JSON file' % path)


def check_file_exists(path, message=False):
    if path is None or not os.path.isfile(path):
        raise IOError(
            errno.ENOENT,
            message or 'Error: %s is not a file.' % path,
            path
        )


def check_dir_exists(path, message=False):
    if path is None or not os.path.isdir(path):
        raise IOError(
            errno.ENOTDIR,
            message or 'Error: %s is not a directory.' % path,
            path
        )


def check_php_opening_tag(path):
    """
    Check a PHP file to make sure nothing is before the opening <?php.

    Except for shebangs.

    :param path: Location of file
    :raises: ValueError on invalid file
    """
    if not path.endswith(('.php', '.inc')):
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

        if (len(lines) > 1 and
                lines[0].startswith('#!') and
                lines[1].lower().startswith('<?php')):
            return

        # None of the return conditions matched, the file must contain <?php
        # but with some content preceeding it.
        raise ValueError(
            '%s has content before opening <?php tag' % path)


def logo(eyes=None, color=True, **colors):
    """
    Get the scap logo.

    Scappy the scap pig::

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

    if not eyes:
        eyes = 'OO'

    eyes = eyes[:2]

    pallet = {
        'pig': ansi.reset() + ansi.esc(ansi.FG_MAGENTA, ansi.BRIGHT),
        'nose': ansi.reset() + ansi.esc(ansi.FG_MAGENTA, ansi.BRIGHT),
        'mouth': ansi.reset() + ansi.esc(ansi.FG_MAGENTA, ansi.BRIGHT),
        'goggles': ansi.reset() + ansi.esc(ansi.FG_YELLOW),
        'eyes': eyes.encode('utf-8'),
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

        '''    %(pig)s_%(goggles)s%(eyes)s≣=-%(pig)s ''',
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
    """
    Create a temporary directory and delete it after the block.

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


def is_initsystem(to_test):
    """
    Check if the init system on is the one to test.

    :param to_test: init system to test
    :returns: boolean
    """
    if to_test == 'systemd':
        return os.path.isdir('/run/systemd/system')
    elif to_test == 'upstart':
        return os.path.isfile('/sbin/initctl')
    else:
        raise NotImplementedError("Only systemd and upstart are supported")


def is_service_running(service):
    """
    Check if a service is running.

    :param service: Service name
    """
    if is_initsystem('systemd'):
        service_name = '{}.service'.format(service)
        systemctl_exit_code = subprocess.call(
            ['/bin/systemctl', '--quiet', 'is-active', service_name]
        )

        return systemctl_exit_code == 0
    elif is_initsystem('upstart'):
        status = subprocess.check_output(
            ['/sbin/status', service]).rstrip().split(' ')
        if not status[1].startswith('start/running'):
            return False
        return True
    else:
        raise NotImplementedError('Only  upstart or systemd are supported')


def systemd_service_exists(service):
    """
    Systemd service unit exists
    """
    state_cmd = ['/bin/systemctl', 'show', '--property', 'LoadState', service]
    try:
        loaded_state = subprocess.check_output(state_cmd).strip()
    except subprocess.CalledProcessError:
        return False

    # Newer versions of systemctl have the --value flag, which means you don't
    # have to split output on '='. That'd sure be nice, but this throws an
    # error on older systemctl versions.
    if '=' not in loaded_state:
        return False

    # Should contain something like LoadState=loaded
    state = loaded_state.split('=')[1]

    # not-found does not, in fact, exit non-zero as one might expect
    # - <3 systemd
    return state not in ['masked', 'not-found']


def upstart_service_exists(service):
    """
    Upstart service exists
    """
    return os.path.exists(
        os.path.join('/etc/init/', '{}.conf'.format(service)))


def sysv_service_exists(service):
    """
    Determine if a sysvinit script exists for a service.
    """
    return os.path.exists(
        os.path.join('/etc/init.d', service))


def service_exists(service):
    """
    Determine if service exists.
    """
    sysv_exists = sysv_service_exists(service)

    if is_initsystem('upstart'):
        upstart_exists = upstart_service_exists(service)

    if is_initsystem('systemd'):
        systemd_exists = systemd_service_exists(service)
        # Return early for systemd systems because we want to obey the "masked"
        # property of systemd which isn't taken into account in sysv_exists or
        # upstart_exists
        return systemd_exists

    return sysv_exists or upstart_exists


def mkdir_p(path):
    """
    Create directory path.

    :param path: The directory path to be created.
    """
    if not os.path.exists(path):
        os.makedirs(path)


def move_symlink(source, dest):
    if os.path.realpath(dest) == source:
        return

    dest_dir = os.path.dirname(dest)
    rsource = os.path.relpath(source, dest_dir)
    rdest = os.path.relpath(dest, dest_dir)

    # Make link's parent directory if it doesn't exist
    mkdir_p(dest_dir)

    with cd(dest_dir):
        if os.path.lexists(rdest):
            os.unlink(rdest)

        os.symlink(rsource, rdest)


def get_active_wikiversions(directory, realm, datacenter):
    """
    Get an ordered collection of active MediaWiki versions.

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
        if path:
            p = p.strip('\t\r\n/')
        if p:
            path.append(p)

    path_str = os.path.join(*path)
    return os.path.normpath(path_str)


def get_patches(sub_dirs, root_dir):
    """
    Find all patches under each subdirectory.

    :param sub_dirs list of sub directories under which to search
    :param root_dir base path under which subdirectories reside
    :return dictionary of patches, keyed by sub_dir
    """
    patches = {}
    for sub_dir in sub_dirs:
        sorted_patches = sorted(
            glob.glob(os.path.join(root_dir, sub_dir, '*.patch')),
            reverse=True
        )
        for patch_file in sorted_patches:
            with open(patch_file, 'r') as f:
                patches.setdefault(sub_dir, []).append(f.read())

    return patches


def ordered_load(stream, Loader=yaml.Loader,
                 object_pairs_hook=collections.OrderedDict):
    """
    Load yaml files and keeping order.

    From stackoverflow.com/questions/5121931

    :param stream the file object to read
    :param loader yaml.Load or its subclasses
    :object_pairs_hook type of return
    :return OrderedDict object with the same order of the yaml file"""
    class OrderedLoader(Loader):
        pass

    def construct_mapping(loader, node):
        loader.flatten_mapping(node)
        return object_pairs_hook(loader.construct_pairs(node))

    OrderedLoader.add_constructor(
        yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
        construct_mapping)
    return yaml.load(stream, OrderedLoader)


class VarDumpJSONEncoder(JSONEncoder):
    ''' encode python objects to json '''
    def default(self, o):
        if hasattr(o, '__dump__'):
            return o.__dump__()
        if hasattr(o, '__dict__'):
            return o.__dict__
        try:
            return JSONEncoder.default(self, o)
        except (TypeError, ValueError):
            return "Unserializable"


def var_dump(*args, **kwargs):
    ''' dump an object to the console as pretty-printed json'''

    lexer = pygments.lexers.JsonLexer()
    formatter = pygments.formatters.TerminalFormatter()
    encoder = VarDumpJSONEncoder(indent=2)

    def dump(obj):
        try:
            json_str = encoder.encode(obj)
            output = pygments.highlight(json_str, lexer, formatter)
            print(output)
        except Exception as e:
            print(e)
            print(obj)

    for arg in args:
        dump(arg)
    if kwargs:
        dump(kwargs.items())


def cpus_for_jobs():
    """Get how many CPUs we can use for farming jobs out"""
    return max(multiprocessing.cpu_count() - 2, 1)
