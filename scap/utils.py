# -*- coding: utf-8 -*-
"""
    scap.utils
    ~~~~~~~~~~
    Contains misc utility functions.

"""
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


BRANCH_RE = re.compile(
    r"(?P<major>\d{1})."
    r"(?P<minor>\d{1,2})."
    r"(?P<patch>\d{1,2})"
    r"-wmf.(?P<prerelease>\d{1,2})"
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
        choices = "[{}]".format(default)

    ans = input("{} {}: ".format(question, choices)).strip()
    return ans.lower() if ans else default


def confirm(question="Continue?", default=False, on_fulfilled=None, on_rejected=None):
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
    yes = ["y", "yes"]
    no = ["n", "no"]

    if default:
        choices = "[Y/n]"
    else:
        choices = "[y/N]"

    # in case stdin is not a tty or the user accepts the default answer, then
    # the result will be default.
    result = default

    if sys.stdout.isatty():
        ans = input("{} {}: ".format(question, choices)).strip().lower()
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
        if callable(on_rejected):
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
                except socket.error as e:
                    # EHOSTUNREACH will occur if the TTL is too low.
                    # ECONNREFUSED might happen if the host is only listening
                    # on IPv4 or only IPv6 but we're tried the other address family.
                    if e.errno != errno.EHOSTUNREACH and e.errno != errno.ECONNREFUSED:
                        # Something unexpected.  Discard the host
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

    Example:
       def my_function():
           with suppress_backtrace():
              some_function_that_may_reasonably_fail()
    """
    try:
        yield
    except Exception as e:
        # This value is read by _handle_exception in cli.py and main.py.
        e._scap_no_backtrace = True
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
            raise ValueError("When calling \"scap\" locally, the \"app\" parameter is required")

        cmd += " " + " ".join(app.format_passthrough_args())

    # Only sudo when necessary
    if user == get_username():
        fullCmd = cmd
    else:
        cmd_env = make_sudo_check_call_env(["PHP"])
        fullCmd = "sudo -u %s -n %s -- %s" % (user, cmd_env, cmd)

    logger.debug("sudo_check_call running {}".format(fullCmd))
    # We're using universal_newlines=True to put the stdout pipe into
    # text mode for simpler processing.  We don't actually care about
    # the universal newline behavior.  Python 3.7 makes this clearer
    # by providing an argument named 'text' as an alias for
    # universal_newlines.
    proc = subprocess.Popen(
        fullCmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True,
        universal_newlines=True,
    )

    fullOut = []
    for line in proc.stdout:
        line = line.strip()
        logger.log(logLevel, line)
        fullOut.append(line)
    # stdout has been read entirely by this point.
    proc.wait()

    if proc.returncode:
        logger.error("Last output:\n" + "\n".join(fullOut))
        raise subprocess.CalledProcessError(proc.returncode, cmd)


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


def is_initsystem(to_test):
    """
    Check if the init system on is the one to test.

    :param to_test: init system to test
    :returns: boolean
    """
    if to_test == "systemd":
        return os.path.isdir("/run/systemd/system")
    if to_test == "upstart":
        return os.path.isfile("/sbin/initctl")
    raise NotImplementedError("Only systemd and upstart are supported")


def is_service_running(service):
    """
    Check if a service is running.

    :param service: Service name
    """
    if is_initsystem("systemd"):
        service_name = "{}.service".format(service)
        systemctl_exit_code = subprocess.call(
            ["/bin/systemctl", "--quiet", "is-active", service_name]
        )

        return systemctl_exit_code == 0
    if is_initsystem("upstart"):
        status = subprocess.check_output(["/sbin/status", service]).decode().rstrip().split(" ")
        if not status[1].startswith("start/running"):
            return False
        return True
    raise NotImplementedError("Only  upstart or systemd are supported")


def systemd_service_exists(service):
    """
    Systemd service unit exists
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


def upstart_service_exists(service):
    """
    Upstart service exists
    """
    return os.path.exists(os.path.join("/etc/init/", "{}.conf".format(service)))


def sysv_service_exists(service):
    """
    Determine if a sysvinit script exists for a service.
    """
    return os.path.exists(os.path.join("/etc/init.d", service))


def service_exists(service):
    """
    Determine if service exists.
    """
    sysv_exists = sysv_service_exists(service)

    if is_initsystem("upstart"):
        upstart_exists = upstart_service_exists(service)

    if is_initsystem("systemd"):
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


def get_active_wikiversions(directory, realm, return_type=list):
    """
    Get an ordered collection of active MediaWiki versions.

    :returns: If 'return_type' is list (the default), returns a list of
              versions, sorted in ascending order.

              If 'return_type' is dict (the default), returns a
              collections.OrderedDict of {version:wikidb} values sorted by
              version number in ascending order.  'wikidb' will be the
              first-seen wikidb for 'version'.  This can be used by
              operations that need a db but don't care which wiki's db is
              used.
    """
    path = get_realm_specific_filename(
        os.path.join(directory, "wikiversions.json"), realm
    )

    with open(path) as f:
        wikiversions = json.load(f)

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

    if return_type == list:
        return list(sorted_versions.keys())

    raise ValueError("Unexpected return_type: {}".format(return_type))


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
    """ encode python objects to json """

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
    """ dump an object to the console as pretty-printed json"""

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


def write_file_if_needed(filename, data: str):
    """Write 'data' to 'filename' if 'filename' doesn't already have that data in it

    Note, the file is written in text mode.
    """

    if os.path.exists(filename):
        with open(filename) as f:
            if f.read() == data:
                # Already set.  Bail out.
                return

    # Do the deed.
    with tempfile.NamedTemporaryFile("w", dir=os.path.dirname(filename)) as f:
        f.write(data)

        if os.path.exists(filename):
            os.unlink(filename)

        os.chmod(f.name, 0o664)
        os.link(f.name, filename)


@contextlib.contextmanager
def temp_to_permanent_file(final_filename):
    """
    temp_to_permanent_file yields a text stream on a temporary file
    that is open for writing.  If the context body completes without
    exception, the temp file is renamed to `final_filename`,
    atomically replacing any existing file of that name.  If an exception
    is raised during the exception of the body, the temp file is deleted
    and `final_filename` remains unaffected.

    Example:

    with temp_to_permanent_file("/tmp/important") as f:
        f.write("Important information")

    """

    # Create the temp file in the same directory as the final filename
    # so that os.rename() can atomically replace the destination file
    # (if one exists)
    with tempfile.NamedTemporaryFile("w", dir=os.path.dirname(final_filename), delete=False) as tmp:
        try:
            yield tmp
        except BaseException as e:
            os.unlink(tmp.name)
            raise e

    # Reach here on success
    os.chmod(tmp.name, 0o644)
    # This is atomic
    os.rename(tmp.name, final_filename)


def prompt_user_for_confirmation(prompt_message) -> bool:
    """
    Prompts user with `prompt_message` and expects yes/no answer.
    """
    while True:
        answer = input(prompt_message + " (y/n): ")
        if re.match(r"(?i)(n|no)$", answer):
            return False
        if re.match(r"(?i)(y|yes)$", answer):
            return True


@contextlib.contextmanager
def open_with_lock(path, mode='r', *args, **kwargs):
    """
    Opens the given file and acquires an advisory lock using the open file
    object. If the mode is read-only ('r' or 'rb'), the lock is acquired as
    shared, and otherwise acquired as exclusive.
    """
    lock_cmd = fcntl.LOCK_SH if mode in {'r', 'rb'} else fcntl.LOCK_EX

    with open(path, mode, *args, **kwargs) as f:
        try:
            fcntl.lockf(f, lock_cmd)
            yield f
        finally:
            fcntl.lockf(f, fcntl.LOCK_UN)


def is_phabricator_task_id(string: str) -> bool:
    """ Returns true if 'string' has the format of a phabricator task id """
    return re.match(r"T\d+$", string) is not None


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
    a very large version number.
    """
    if version == "master":
        return packaging.version.Version(str(sys.maxsize))

    # Strip all non-digit, non-dot characters from the version string, then
    # parse it.
    return packaging.version.Version(re.sub(r"[^.\d]", "", version))


def on_real_deploy_server() -> bool:
    return socket.getfqdn().endswith(".wmnet")


def get_current_train_version_from_gerrit(gerrit_url) -> str:
    """Returns a string like '1.39.0-wmf.19'"""

    url = os.path.join(gerrit_url, "mediawiki/core")

    # output will be something like '3137081c2ab92df3bc9c97956b00fb3017d7b511\trefs/heads/wmf/1.39.0-wmf.19'
    output = subprocess.check_output(["git", "ls-remote", "--sort=version:refname", url, "refs/heads/wmf/*"],
                                     universal_newlines=True)
    res = re.sub(r"^.*wmf/(.*)$", "\\1", output.splitlines()[-1])

    return res


def get_current_train_info(api_url, proxy=None) -> dict:
    """
    Returns a dictionary containing information about this week's train
    """

    # Support absolute file:// URLs for testing (particularly by train-dev).
    if api_url.startswith("file:///"):
        with open(api_url[len("file://"):]) as f:
            current = json.loads(f.read())
    else:
        proxies = {"http": proxy, "https": proxy} if proxy else None
        resp = requests.get(api_url, proxies=proxies)
        resp.raise_for_status()

        current = resp.json()["current"]

    version = current["version"]
    task = current["task_id"]
    status = current["status"]

    if not is_phabricator_task_id(task):
        raise ValueError("{} returned invalid Phabricator task id '{}'".format(
            api_url, task))

    if not re.match(BRANCH_RE, version):
        raise ValueError("{} returned invalid version '{}'".format(
            api_url, version))

    return {
        "version": version,
        "task": task,
        "status": status,
    }


def subprocess_check_run_quietly_if_ok(cmd, dir, logfile, logger, shell=False):
    try:
        with open(logfile, "a") as logstream:
            log_file_position = logstream.tell()
            logger.debug("Running {} in {}".format(cmd, dir))
            subprocess.run(cmd, shell=shell, check=True, cwd=dir,
                           stdout=logstream,
                           stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError as e:
        # Print the error message, which contains the command that was executed and its
        # exit status.
        logger.error(e)
        logger.error("Stdout/stderr follows:")
        with open(logfile) as logstream:
            logstream.seek(log_file_position)
            logger.error(logstream.read())
        raise
