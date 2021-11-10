# This module contains functions that run external commands, one function for
# each command. Only the specific ways in which Scap needs to run the commands
# is supported. All function check the exit code of the command, and raise
# FailedCommand, if not zero.
#
# The functions can take the cwd keyword argument to specify the directory in
# which the command should be invoked.

import logging
import os
import subprocess


class FailedCommand(Exception):
    """Exception for when a command fails (exits non-zero)

    Exception attributes exitcode, stdout, and stderr hold the command's exit
    code, captured standard output, and captured standard error.
    """

    def __init__(self, command, exitcode, stdout, stderr):
        Exception.__init__(
            self,
            "Command '{command}' failed with exit code {exitcode}; stderr:\n{stderr}".format(
                command=command, exitcode=exitcode, stderr=stderr
            ),
        )
        self.exitcode = exitcode
        self.stdout = stdout
        self.stderr = stderr


def _runcmd(argv, **kwargs):
    """Run an external command, return its stdout

    Raises FailedCommand if command exit code is not zero.

    Set the keyword argument _want_stderr to True (not just a value that
    is considered true) to return the command's stderr instead of its
    stdout.

    Other keyword arguments are passed to subprocess.Popen, except that
    stdout, stderr, and stdin are overridden to capture output and make
    stdin come from /dev/null.
    """

    want_stderr = kwargs.pop("_want_stderr", False) is True

    # Set keyword arguments to capture stdout and stderr.
    kwargs["stdout"] = subprocess.PIPE
    kwargs["stderr"] = subprocess.PIPE

    # Open /dev/null so stdin can be redirected to come from there. This way,
    # if a command is accidentally invoked in a way that it reads from stdin,
    # it won't get stuck.
    with open("/dev/null", "rb") as devnull:
        kwargs["stdin"] = devnull

        # Invoke the commmand.
        logging.debug(
            "Running {argv!r} with {kwargs!r}".format(argv=argv, kwargs=kwargs)
        )
        p = subprocess.Popen(argv, **kwargs)

    # Wait for command to finish.
    (stdout, stderr) = p.communicate()

    logging.debug("Command exited with code %s", p.returncode)

    # Check if command failed.
    if p.returncode != 0:
        raise FailedCommand(" ".join(argv), p.returncode, stdout, stderr)

    # All good, return captured stdout or stderr.
    if want_stderr:
        return stderr
    return stdout


def gitcmd(subcommand, *args, **kwargs):
    """Run a git subcommand, return its stdout

    Return the output of git as a Unicode string.
    """
    return _runcmd(["git", subcommand] + list(args), **kwargs).decode("UTF8")


def delete_file_in_tree(dirname, basename):
    """Delete every file in a directory tree, if its basename is as given"""
    _runcmd(["find", dirname, "-name", basename, "-delete"])


def touch(filename, **kwargs):
    """Create a file, or update its modification time."""
    _runcmd(["touch", filename], **kwargs)


def which(name):
    """Does a named program exist on $PATH?

    Return the path to the command, or None.
    """
    dirs = os.environ["PATH"].split(":")
    for dirname in dirs:
        path = os.path.join(dirname, name)
        if os.access(path, os.X_OK) and os.path.isfile(path):
            return path


def mwscript(phpfile, *args, **kwargs):
    """Run the mwcript command on a PHP file, return its stderr"""
    return _runcmd(["mwscript", phpfile] + list(args), _want_stderr=True, **kwargs)
