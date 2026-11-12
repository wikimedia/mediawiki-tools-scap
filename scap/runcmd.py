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
        if isinstance(command, str):
            pass
        elif isinstance(command, list):
            command = " ".join(command)

        Exception.__init__(
            self,
            "Command '{command}' failed with exit code {exitcode};\nstdout:\n{stdout}\nstderr:\n{stderr}".format(
                command=command, exitcode=exitcode, stdout=stdout, stderr=stderr
            ),
        )
        self.exitcode = exitcode
        self.stdout = stdout
        self.stderr = stderr


def _runcmd(argv, **kwargs) -> str:
    """Run an external command, return its stdout

    Raises FailedCommand if command exit code is not zero.

    Set the keyword argument _want_stderr to True (not just a value that
    is considered true) to return the command's stderr instead of its
    stdout.

    Set the keyword argument _stream to True (not just a value that is
    considered true) to not capture stdout/stderr or change stdin.
    The function returns an empty string in this case.

    Other keyword arguments are passed to subprocess.Popen, except that
    stdout, stderr, and stdin are always set by this function.
    """

    want_stderr = kwargs.pop("_want_stderr", False) is True
    stream = kwargs.pop("_stream", False) is True

    if stream:
        # None makes Popen pass on the streams of scap.
        kwargs["stdout"] = None
        kwargs["stderr"] = None
        kwargs["stdin"] = None
    else:
        # Set keyword arguments to capture stdout and stderr.
        kwargs["stdout"] = subprocess.PIPE
        kwargs["stderr"] = subprocess.PIPE

        # Open /dev/null so stdin can be redirected to come from there. This
        # way, if a command is accidentally invoked in a way that it reads
        # from stdin, it won't get stuck.
        kwargs["stdin"] = subprocess.DEVNULL

    # Enable text mode
    kwargs["text"] = True

    # Invoke the commmand.
    logging.debug("Running {argv!r} with {kwargs!r}".format(argv=argv, kwargs=kwargs))
    p = subprocess.Popen(argv, **kwargs)

    # Wait for command to finish.
    (stdout, stderr) = p.communicate()

    # Check if command failed.
    if p.returncode != 0:
        logging.debug("Command exited with code %s", p.returncode)
        raise FailedCommand(argv, p.returncode, stdout or "", stderr or "")

    # All good, return captured stdout or stderr.
    if stream:
        return ""
    if want_stderr:
        return stderr
    return stdout


def gitcmd(subcommand, *args, **kwargs) -> str:
    """Run a git subcommand, return its stdout

    Return the output of git as a Unicode string.
    """

    # gc.autoDetach=false prevents git from sneakily running in the background
    # after exit, possibly causing a lock error on a later git command.
    # (T438236).
    return _runcmd(
        ["git", "-c", "gc.autoDetach=false", subcommand] + list(args), **kwargs
    )


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
