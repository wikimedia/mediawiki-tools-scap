# -*- coding: utf-8 -*-
"""
    scap.utils
    ~~~~~~~~~~
    Contains misc utility functions.

"""
import argparse
import base64
import collections
import contextlib
import errno
import fcntl
from functools import wraps
import glob
import hashlib
import inspect
import json
from json import JSONEncoder
import logging
import math
import multiprocessing
import os
import packaging.version
import pwd
import random
import re
import requests
import socket
import struct
import subprocess
import sys
import tempfile
import yaml

import pygments
import pygments.lexers
import pygments.formatters


BRANCH_RE_UNANCHORED = re.compile(
    r"(?P<major>\d{1})."
    r"(?P<minor>\d{1,2})."
    r"(?P<patch>\d{1,2})"
    r"-wmf.(?P<prerelease>\d{1,2})"
)
# Anchored to the end of string.  Use BRANCH_RE.match() or
# BRANCH_RE.search() according to your needs.
BRANCH_RE = re.compile(rf"{BRANCH_RE_UNANCHORED.pattern}$")
BRANCH_CUT_PRETEST_BRANCH = "branch_cut_pretest"


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
        raise ValueError("error tolerances must be non-negative")

    # use cmath so it will work with complex ot float
    if math.isinf(abs(a)) or math.isinf(abs(b)):
        # This includes the case of two infinities of opposite sign, or
        # one infinity and one finite number. Two infinities of opposite sign
        # would otherwise have an infinite relative tolerance.
        return False
    diff = abs(b - a)

    return ((diff <= abs(rel_tol * b)) or (diff <= abs(rel_tol * a))) or (
        diff <= abs_tol
    )


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
            host_map[host] = socket.getaddrinfo(host, port, proto=socket.IPPROTO_TCP)
        except socket.gaierror:
            continue

    for ttl in range(1, 30):
        if not host_map:
            break
        for host, infos in random.sample(host_map.items(), len(host_map)):
            for info in infos:
                family, sock_type, proto, _, addr = info

                if family == socket.AF_INET:
                    setsockopt_level = socket.IPPROTO_IP
                    setsockopt_option = socket.IP_TTL
                elif family == socket.AF_INET6:
                    setsockopt_level = socket.IPPROTO_IPV6
                    setsockopt_option = socket.IPV6_UNICAST_HOPS
                else:
                    # Unsupported address family
                    continue

                s = socket.socket(family, sock_type, proto)
                # Set the TTL (aka hop limit in IPv6)
                s.setsockopt(setsockopt_level, setsockopt_option, struct.pack("I", ttl))
                s.settimeout(timeout)
                try:
                    s.connect(addr)
                except socket.timeout:
                    continue
                except socket.error as e:
                    # EHOSTUNREACH will occur if the TTL is too low.
                    # ECONNREFUSED might happen if the host is only listening
                    # on IPv4 or only IPv6 but we're tried the other address family.
                    if e.errno != errno.EHOSTUNREACH and e.errno != errno.ECONNREFUSED:
                        # Something unexpected.  Discard the host
                        del host_map[host]
                    continue
                else:
                    return host
                finally:
                    s.close()


def get_real_username():
    """Get the username of the user that initiated the current operation."""
    spiderpig_user = os.environ.get("SPIDERPIG_REAL_USER")
    if spiderpig_user:
        return spiderpig_user

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

    return pwd.getpwnam(name).pw_gecos.split(",")[0]


def get_env_specific_filename(path, env=None):
    """Find a file specific to the environment in which scap is running."""
    if env is None:
        return path

    base = os.path.dirname(path)
    filename = os.path.basename(path)

    if base.endswith("/templates"):
        base = os.path.dirname(base)
        filename = os.path.join("templates", filename)

    env_filename = os.path.join(base, "environments", env, filename)

    if os.path.isfile(env_filename):
        return env_filename

    return path


def get_realm_specific_filename(filename, realm):
    """
    If a realm-specific version of 'filename' exists, return it,
    otherwise return 'filename'.  To construct the realm-specific
    filename, "-REALM" is inserted before the file extension.  For
    example, "wikiversions.json" becomes "wikiversions-REALM.json".
    """
    base, ext = os.path.splitext(filename)

    # FIXME: Why should an extensionless file not undergo the same treatment?
    if ext == "":
        return filename

    realm_specific = "%s-%s%s" % (base, realm, ext)
    if os.path.isfile(realm_specific):
        return realm_specific
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
    return "%02dm %02ds" % divmod(elapsed, 60)


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
            if hasattr(inspect, "getfullargspec"):
                argspec = inspect.getfullargspec(func)
            else:
                argspec = inspect.getargspec(func)

            # Check if logger was passed as a positional argument
            try:
                logger = args[argspec.args.index("logger")]
            except IndexError:
                logger = None
            except ValueError:
                logger = None

            # Check if logger was passed as a keyword argument
            if logger is None:
                logger = kwargs.get("logger", None)

            if logger is not None:
                return func(*args, **kwargs)

            with context_logger(context_name) as logger:
                kwargs["logger"] = logger
                return func(*args, **kwargs)

        return context_wrapper

    return arg_wrapper


def get_logger():
    if LOGGER_STACK:
        return LOGGER_STACK[-1]
    return logging.getLogger()


@contextlib.contextmanager
def suppress_backtrace():
    """
    Context manager that sets the "don't backtrace" flag on any exception
    that occurs within context.

    Can be overridden by setting the environment variable `SCAP_BACKTRACE`.

    Example:
       def my_function():
           with suppress_backtrace():
              some_function_that_may_reasonably_fail()
    """
    try:
        yield
    except Exception as e:
        # This value is read by _handle_exception in cli.py and main.py.
        e._scap_no_backtrace = os.environ.get("SCAP_BACKTRACE", None) is None
        raise


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
    with open(path, "rb") as f:
        # Digest file in 1M chunks just in case it's huge
        for block in iter(lambda: f.read(1048576), b""):
            crc.update(block)
    return crc.hexdigest()


def make_sudo_check_call_env(env):
    """
    Returns a string of environment variables formatted for the shell

    sudo 1.8.21 adds support for adding a list of variables to
    --preserve-env, that should replace this function in future
    """
    envvars = {}
    for key in env:
        val = os.environ.get(key)
        if val:
            envvars[key] = val
    return " ".join(['{}="{}"'.format(k, v) for k, v in envvars.items()])


@log_context("sudo_check_call")
def sudo_check_call(user, cmd, logger=None, logLevel=logging.DEBUG, app=None):
    """
    Run a command as a specific user.

    Reports stdout/stderr of process to logger during execution.

    Returns a string containing stdout/stderr output from the subprocess.

    :param user: User to run command as
    :param cmd: Command to execute
    :param logger: Logger to send process output to
    :param app: Application calling the function, required if the command is `scap`
    :raises: subprocess.CalledProcessError on non-zero process exit
    :raises: ValueError if the command is `scap` and app was not specified
    """

    # If command is `scap`, pass cli config args through
    cmd_basename = os.path.basename(cmd.split()[0])
    if "scap" == cmd_basename:
        if app is None:
            raise ValueError(
                'When calling "scap" locally, the "app" parameter is required'
            )

        cmd += " " + " ".join(app.format_passthrough_args())

    # Only sudo when necessary
    if user == get_username():
        fullCmd = cmd
    else:
        cmd_env = make_sudo_check_call_env(["PHP"])
        fullCmd = "sudo -u %s -n %s -- %s" % (user, cmd_env, cmd)

    logger.debug("sudo_check_call running {}".format(fullCmd))
    proc = subprocess.Popen(
        fullCmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        shell=True,
        text=True,
    )

    fullOut = []
    for line in proc.stdout:
        line = line.strip()
        logger.log(logLevel, line)
        fullOut.append(line)
    # stdout has been read entirely by this point.
    proc.wait()

    # Change fullOut from a list of strings to one big string.
    fullOut = "\n".join(fullOut)

    if proc.returncode:
        logger.error("Last output:\n%s", fullOut)
        raise subprocess.CalledProcessError(proc.returncode, cmd)

    return fullOut


def check_file_exists(path, message=False):
    if path is None or not os.path.isfile(path):
        raise IOError(errno.ENOENT, message or "Error: %s is not a file." % path, path)


def check_dir_exists(path, message=False):
    if path is None or not os.path.isdir(path):
        raise IOError(
            errno.ENOTDIR, message or "Error: %s is not a directory." % path, path
        )


def dir_is_empty(path):
    """
    Returns True if 'path' is an empty directory, otherwise returns False.
    Raises an error if 'path' does not name a directory.
    """
    return len(os.listdir(path)) == 0


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
            prefix + str(random.SystemRandom().randint(0, 0xFFFFFFFF)),
        )
        if not os.path.exists(dirname):
            break
    # Yes, there is a small race condition here in theory. In practice it
    # should be pretty hard to hit due to scap's global concurrency lock.
    sudo_check_call(owner, 'mkdir "%s"' % dirname)

    try:
        yield dirname
    finally:
        sudo_check_call(owner, 'find "%s" -maxdepth 1 -delete' % dirname)


def service_exists(service):
    """
    Determine if a systemd service unit exists.
    """
    state_cmd = ["/bin/systemctl", "show", "--property", "LoadState", service]
    try:
        loaded_state = subprocess.check_output(state_cmd).decode().strip()
    except subprocess.CalledProcessError:
        return False

    # Newer versions of systemctl have the --value flag, which means you don't
    # have to split output on '='. That'd sure be nice, but this throws an
    # error on older systemctl versions.
    if "=" not in loaded_state:
        return False

    # Should contain something like LoadState=loaded
    state = loaded_state.split("=")[1]

    # not-found does not, in fact, exit non-zero as one might expect
    # - <3 systemd
    return state not in ["masked", "not-found"]


def mkdir_p(path):
    """
    Create directory path.

    :param path: The directory path to be created.
    """
    if not os.path.exists(path):
        os.makedirs(path)


def update_symlink(target, linkpath):
    if os.path.realpath(linkpath) == target:
        # The symlink already points to the desired target.
        return

    link_dir = os.path.dirname(linkpath)
    rtarget = os.path.relpath(target, link_dir)
    rlinkpath = os.path.relpath(linkpath, link_dir)

    # Make link's parent directory if it doesn't exist
    mkdir_p(link_dir)

    with cd(link_dir):
        # WARNING: This is not atomic.  There will be a short period of time
        # between when the link has been removed and when it is recreated.
        # This could be improved by creating a temp-named symlink in link_dir
        # and renaming it.

        if os.path.lexists(rlinkpath):
            os.unlink(rlinkpath)

        os.symlink(rtarget, rlinkpath)


def read_wikiversions(directory, realm) -> dict:
    """
    Return a dictionary representing the contents of the realm-specific wikiversions.json
    file in the specified directory.

    Keys are wikidbs, values are "php-<version>"
    """
    path = get_realm_specific_filename(
        os.path.join(directory, "wikiversions.json"), realm
    )

    with open(path) as f:
        wikiversions = json.load(f)

    force_version = os.environ.get("FORCE_MW_VERSION")
    if force_version:
        # Make sure FORCE_MW_VERSION has a decent value
        version_argument_parser(force_version, allow_auto=False)

        force_version = f"php-{force_version}"
        for key in wikiversions:
            wikiversions[key] = force_version

    return wikiversions


def get_active_wikiversions(directory, realm, return_type=list):
    """
    Get an ordered collection of active MediaWiki versions.

    :returns: If 'return_type' is list (the default), returns a list of
              versions, sorted in ascending order.

              If 'return_type' is dict (the default), returns a
              collections.OrderedDict of {version:wikidb} values sorted by
              version number in ascending order.  'wikidb' will be the
              alphabetically-first wikidb for 'version'.  This can be used by
              operations that need a db but don't care which wiki's db is
              used.
    """
    if return_type not in [list, dict]:
        raise ValueError(f"Unexpected return_type: {return_type}")

    wikiversions = read_wikiversions(directory, realm)

    versions = {}
    # Process keys in sorted order to ensure that we always use the same
    # representative wikidb for a given version.
    for wikidb in sorted(wikiversions.keys()):
        version = wikiversions[wikidb][4:]  # trim 'php-' from version
        if version not in versions:
            versions[version] = wikidb

    # Convert to list of (version, representative-db) tuples sorted by version
    # number and then convert that list to an OrderedDict
    sorted_versions = collections.OrderedDict(
        sorted(versions.items(), key=lambda v: parse_wmf_version(v[0]))
    )

    if return_type == dict:
        return sorted_versions
    else:
        return list(sorted_versions.keys())


def get_wikiversions_ondisk(directory) -> list:
    """
    Returns a list of the train branch versions that are currently
    checked out in DIRECTORY.  The list is sorted in ascending order.

    :returns: list like::
        ['1.41.0-wmf.1', '1.41.0-wmf.2']

    """

    def is_wikiversion(name):
        return BRANCH_RE.match(name[len("php-") :]) or name == "php-master"

    versions = [
        d[len("php-") :]
        for d in os.listdir(directory)
        if is_wikiversion(d) and os.path.isdir(os.path.join(directory, d))
    ]

    return sorted(versions, key=parse_wmf_version)


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
        if parent in (current, "/"):
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
            p = p.strip("\t\r\n/")
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
            glob.glob(os.path.join(root_dir, sub_dir, "*.patch")), reverse=True
        )
        for patch_file in sorted_patches:
            with open(patch_file, "r") as f:
                patches.setdefault(sub_dir, []).append(f.read())

    return patches


def ordered_load(
    stream, Loader=yaml.SafeLoader, object_pairs_hook=collections.OrderedDict
):
    """
    Load yaml files and keeping order.

    From stackoverflow.com/questions/5121931

    :param stream the file object to read
    :param loader yaml.SafeLoader or its subclasses
    :object_pairs_hook type of return
    :return OrderedDict object with the same order of the yaml file"""

    class OrderedLoader(Loader):
        pass

    def construct_mapping(loader, node):
        loader.flatten_mapping(node)
        return object_pairs_hook(loader.construct_pairs(node))

    OrderedLoader.add_constructor(
        yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, construct_mapping
    )
    return yaml.load(stream, OrderedLoader)


class VarDumpJSONEncoder(JSONEncoder):
    """encode python objects to json"""

    def default(self, o):
        if hasattr(o, "__dump__"):
            return o.__dump__()
        if hasattr(o, "__dict__"):
            return o.__dict__
        try:
            return JSONEncoder.default(self, o)
        except (TypeError, ValueError):
            return "Unserializable"


def var_dump(*args, **kwargs):
    """dump an object to the console as pretty-printed json"""

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


def find_regular_files(dirname):
    """Return sorted list of all regular files under a directory"""
    return list(sorted(_listfiles(dirname)))


def _listfiles(dirname):
    """Generate each pathname for each regular file under dirname"""
    prefix = dirname + "/"
    for parent, _, filenames in os.walk(dirname):
        for filename in filenames:
            pathname = os.path.join(parent, filename)
            if os.path.isfile(pathname):
                yield pathname[len(prefix) :]


def read_first_line_from_file(filename) -> str:
    """
    Reads and returns the first line of the specified file.
    Whitespace is stripped.
    """
    with open(filename) as f:
        return f.readline().strip()


def write_file_if_needed(filename, data: str) -> bool:
    """Write 'data' to 'filename' if 'filename' doesn't already have that data in it

    When updating is needed, the file is replaced atomically by writing
    to a temp file and then renaming to the final name.

    Note, the file is written in text mode.

    Returns a boolean indicating whether or not the file was updated.
    """

    if os.path.exists(filename):
        with open(filename) as f:
            if f.read() == data:
                # Already set.  Bail out.
                return False

    # Do the deed.
    with temp_to_permanent_file(filename) as f:
        f.write(data)

    return True


@contextlib.contextmanager
def temp_to_permanent_file(final_filename, mode="w"):
    """
    temp_to_permanent_file yields (by default) a text stream on a temporary file
    that is open for writing.  If the context body completes without
    exception, the temp file is renamed to `final_filename`,
    atomically replacing any existing file of that name.  If an exception
    is raised during the exception of the body, the temp file is deleted
    and `final_filename` remains unaffected.

    Specify mode="wb" for a binary stream.

    Example:

    with temp_to_permanent_file("/tmp/important") as f:
        f.write("Important information")

    """

    # Create the temp file in the same directory as the final filename
    # so that os.rename() can atomically replace the destination file
    # (if one exists)
    with tempfile.NamedTemporaryFile(
        mode, dir=os.path.dirname(final_filename), delete=False
    ) as tmp:
        try:
            yield tmp
        except BaseException as e:
            os.unlink(tmp.name)
            raise e

    # Reach here on success
    os.chmod(tmp.name, 0o666 & ~get_umask())
    # This is atomic
    os.rename(tmp.name, final_filename)


@contextlib.contextmanager
def open_with_lock(path, mode="r", *args, **kwargs):
    """
    Opens the given file and acquires an advisory lock using the open file
    object. If the mode is read-only ('r' or 'rb'), the lock is acquired as
    shared, and otherwise acquired as exclusive.
    """
    lock_cmd = fcntl.LOCK_SH if mode in {"r", "rb"} else fcntl.LOCK_EX

    with open(path, mode, *args, **kwargs) as f:
        try:
            fcntl.lockf(f, lock_cmd)
            yield f
        finally:
            fcntl.lockf(f, fcntl.LOCK_UN)


def is_phabricator_task_id(string: str) -> bool:
    """Returns true if 'string' has the format of a phabricator task id"""
    return re.match(r"T\d+$", string) is not None


def get_umask_linux():
    if not os.path.exists("/proc/self/status"):
        return None

    # Requires Linux 4.7.  xref man umask(2)
    with open("/proc/self/status") as f:
        for line in f.readlines():
            m = re.match(r"Umask:\s*([0-7]+)", line)
            if m:
                return int(m[1], 8)

    return None


def get_umask() -> int:
    res = get_umask_linux()
    if res is not None:
        return res

    # Fall back to POSIX where, sadly you cannot retrieve the umask
    # without setting it to something, and this affects the whole process,
    # not just the current thread.
    res = os.umask(0)
    os.umask(res)
    return res


@contextlib.contextmanager
def empty_file_mask():
    orig_umask = os.umask(0)
    yield
    os.umask(orig_umask)


def abort(message):
    raise SystemExit("Aborting: %s" % message)


def list_intersection(list1, list2):
    """Returns a list containing the intersection (items in common) of list1 and list2"""
    return list(set(list1).intersection(set(list2)))


def list_union(list1, list2):
    """Returns a list containing the union of list1 and list2"""
    return list(set(list1).union(set(list2)))


def parse_rsync_stats(string: str) -> dict:
    """
    Scans the string looking for text like the following and
    returns a dictionary with the extracted integer fields.

    Note that if no such matching text is found an empty dictionary
    will be returned.

    Number of files: 184,935 (reg: 171,187, dir: 13,596, link: 152)
    Number of created files: 0
    Number of deleted files: 0
    Number of regular files transferred: 1
    Total file size: 8,756,954,367 bytes
    Total transferred file size: 815,772 bytes
    Literal data: 0 bytes
    Matched data: 815,772 bytes
    File list size: 4,744,396
    File list generation time: 0.517 seconds
    File list transfer time: 0.000 seconds
    Total bytes sent: 5,603
    Total bytes received: 4,744,454
    """

    # Keys are header names expected from rsync --stats output.
    # Values are the names of the keys in 'res' that will be used.
    integer_fields = {
        "Number of files": "files",
        "Number of created files": "files_created",
        "Number of deleted files": "files_deleted",
        "Number of regular files transferred": "regular_files_transferred",
        "Total file size": "total_file_size",
        "Total transferred file size": "total_transferred_file_size",
        "Literal data": "literal_data",
        "Matched data": "matched_data",
        "File list size": "file_list_size",
        "Total bytes sent": "total_bytes_sent",
        "Total bytes received": "total_bytes_received",
    }

    res = {}

    for header, key in integer_fields.items():
        m = re.search(header + r": ([\d,]+)", string, re.MULTILINE)
        if m:
            res[key] = int(m.group(1).replace(",", ""))

    return res


def parse_wmf_version(version: str) -> packaging.version.Version:
    """
    Parses a string like "1.29.0-wmf.4" and returns a packaging.version.Version
    object representing the version.  These objects can be compared using
    <, <=, >, >=, ==.  The special case version of "master" will be treated as
    a very large version number.  "branch_cut_pretest" and "next" are second-to-highest.
    """
    # Ensure that the supplied version string is acceptable
    version_argument_parser(version, allow_auto=False)

    if version == "master":
        return packaging.version.Version(str(sys.maxsize))
    if version in [BRANCH_CUT_PRETEST_BRANCH, "next"]:
        return packaging.version.Version(str(sys.maxsize - 1))

    # Strip all non-digit, non-dot characters from the version string, then
    # parse it.
    return packaging.version.Version(re.sub(r"[^.\d]", "", version))


def version_argument_parser(ver: str, allow_auto=True) -> str:
    """Validate a mediawiki version number argument"""

    candidates = ["master", BRANCH_CUT_PRETEST_BRANCH, "next"]
    if allow_auto:
        candidates.append("auto")

    if ver in candidates:
        return ver

    if BRANCH_RE.match(ver):
        return ver

    raise argparse.ArgumentTypeError("Branch '%s' does not match required format" % ver)


def valid_version(version: str) -> bool:
    """
    Returns True if `version` is a valid version.  In this context
    valid means suitable for naming a <staging dir>/php-<version> directory.
    This means that "auto" is not an acceptable version here.
    """
    try:
        version_argument_parser(version, allow_auto=False)
        return True
    except argparse.ArgumentTypeError:
        return False


def on_real_deploy_server() -> bool:
    return socket.getfqdn().endswith(".wmnet")


def get_current_train_version_from_gerrit(gerrit_url) -> str:
    """Returns a string like '1.39.0-wmf.19'"""

    url = os.path.join(gerrit_url, "mediawiki/core")

    # output will be something like '3137081c2ab92df3bc9c97956b00fb3017d7b511\trefs/heads/wmf/1.39.0-wmf.19'
    output = subprocess.check_output(
        ["git", "ls-remote", "--sort=version:refname", url, "refs/heads/wmf/*"],
        text=True,
    )
    valid_versions = [line for line in output.splitlines() if BRANCH_RE.search(line)]
    res = re.sub(r"^.*wmf/(.*)$", "\\1", valid_versions[-1])

    return res


def get_current_train_info(api_url, proxy=None) -> dict:
    """
    Returns a dictionary containing information about this week's train
    """

    current = get_train_blockers_info(api_url, proxy)["current"]

    version = current["version"]
    task = current["task_id"]
    status = current["status"]

    if not is_phabricator_task_id(task):
        raise ValueError(
            "{} returned invalid Phabricator task id '{}'".format(api_url, task)
        )

    if not re.match(BRANCH_RE, version):
        raise ValueError("{} returned invalid version '{}'".format(api_url, version))

    return {
        "version": version,
        "task": task,
        "status": status,
    }


def get_train_blockers_info(api_url, proxy=None) -> dict:
    """
    Returns a dictionary with details about the current and upcoming train blocker tasks
    """

    # Support absolute file:// URLs for testing (particularly by train-dev).
    if api_url.startswith("file:///"):
        with open(api_url[len("file://") :]) as f:
            return {"current": json.loads(f.read())}
    else:
        proxies = {"http": proxy, "https": proxy} if proxy else None
        resp = requests.get(api_url, proxies=proxies)
        resp.raise_for_status()

        return resp.json()


def expand_dblist(stage_dir, db_list_name: str) -> list:
    script = os.path.join(stage_dir, "multiversion", "bin", "expanddblist")
    return subprocess.check_output([script, db_list_name], text=True).splitlines()


def get_group_versions(group, directory, realm) -> list:
    """
    Returns a list of versions used by 'group', in ascending version order.
    """
    dblist = expand_dblist(directory, group)

    versions = set()

    for wikidb, version in read_wikiversions(directory, realm).items():
        version = re.sub("^php-", "", version)
        if wikidb in dblist:
            versions.add(version)

    return sorted(versions, key=parse_wmf_version)


def select_latest_patches(patch_base_dir):
    """
    Find the latest /srv/patches/<version> directory. Useful to e.g. populate the patches dir for a new version by
    carrying over the most recent patches

    Returns None if unavailable.
    """

    candidates = [
        name for name in os.listdir(patch_base_dir) if re.match(BRANCH_RE, name)
    ]

    if not candidates:
        return None

    latest_patches_vers = sorted(candidates, key=parse_wmf_version)[-1]

    return os.path.join(patch_base_dir, latest_patches_vers)


def pluralize(word: str, quantity) -> str:
    """
    If 'quantity' represents a quantity of one, returns 'word',
    otherwise, returns the pluralized version of 'word'.

    'quantity' can be an int, or an object that works with len().
    """
    if not (isinstance(quantity, int)):
        quantity = len(quantity)

    if quantity == 1:
        return word

    for suffix in ["s", "sh", "ch", "x", "z"]:
        if word.endswith(suffix):
            return word + "es"
    return word + "s"


def should_colorize_output() -> bool:
    return sys.stderr.isatty() or "FORCE_COLOR" in os.environ


def string_to_base64_string(input: str) -> str:
    """
    Returns a string representing the base64 encoding of the input string.
    """
    return base64.b64encode(input.encode("utf-8")).decode("utf-8")
