# -*- coding: utf-8 -*-
"""
    scap.ssh
    ~~~~~~~~
    This module provides functions for running commands on remote hosts
    via SSH.

"""
import errno
import os
import random
import select
import shlex
import socket
import subprocess

import scap.log as log
import scap.utils as utils
import scap.cmd as cmd


CONNECTION_FAILURE = 255
DEFAULT_BATCH_SIZE = 80
SSH = cmd.Command(
    '/usr/bin/ssh', '-oBatchMode=yes',
    '-oSetupTimeout=10',
    '-F/dev/null', cmd.arg('user', '-oUser={}'))


class OutputHandler:
    """
    Standard handler for SSH command output from hosts.

    Simply stores output as a string for future handling.
    """

    host = None
    output = ''

    def __init__(self, host):
        self.host = host

    def accept(self, output):
        self.output += output


class JSONOutputHandler(OutputHandler):
    """
    Deserialize and log structured JSON output from hosts.

    Any non-structured output is stored for future handling.
    """

    def __init__(self, host):
        self.host = host
        self._logger = utils.get_logger().getChild('target').getChild(host)
        self._partial = ''

    def accept(self, output):
        """
        Extract and deserializes line-wise JSON from the given output.

        Any non-JSON is stored in self.output.
        """
        for line in self.lines(output):
            if line.startswith('{'):
                try:
                    record = log.JSONFormatter.make_record(line)
                except (ValueError, TypeError):
                    self.output += line + "\n"
                    record = None

                if record is not None:
                    # qualify the record name according to our prefix
                    record.name = self._logger.name + '.' + record.name

                    # amend the record with the host name
                    record.host = self.host

                    # propagate the log record
                    self._logger.handle(record)
            else:
                self.output += line + "\n"

    def lines(self, output):
        """
        Generate each line of the given output.

        Reconstructs partial lines using the leftovers from previous calls.
        """
        while True:
            pos = output.find("\n")
            if pos < 0:
                self._partial += output
                break

            yield self._partial + output[0:pos]
            output = output[pos + 1:]
            self._partial = ''


class Job(object):
    """Execute a job on a group of remote hosts via ssh."""
    @utils.log_context('ssh.job')
    def __init__(self, hosts=None, command=None, user=None, logger=None):
        self.hosts(hosts or [])
        self._command = command
        self._reporter = None
        self._user = user
        self.max_failure = len(self._hosts)
        self._logger = logger
        self.output_handler = OutputHandler

    def get_logger(self):
        """Lazy getter for a logger instance."""
        return self._logger

    def hosts(self, hosts):
        """Set hosts to run command on."""
        self._hosts = list(hosts)
        return self

    def shuffle(self):
        """Randomize order of target hosts."""
        random.shuffle(self._hosts)
        return self

    def exclude_hosts(self, exclude):
        exclude = [socket.getfqdn(h) for h in exclude]
        self.hosts(
            [h for h in self._hosts if socket.getfqdn(h) not in exclude])

    def command(self, command):
        """Set command to run."""
        self._command = command
        return self

    def progress(self, reporter):
        """Set the reporter used when reporting progress."""
        self._reporter = reporter
        return self

    def run(self, batch_size=DEFAULT_BATCH_SIZE):
        """
        Run the job, report progress, and return success/failed counts.

        :returns: (ok, failed) counts of successful/failed hosts
        :raises: RuntimeError if command has not been set
        """
        ok = 0
        failed = 0

        for host, status in self.run_with_status(batch_size):
            if status == 0:
                ok += 1
            else:
                failed += 1

        return ok, failed

    def run_with_status(self, batch_size=DEFAULT_BATCH_SIZE):
        """
        Run the job, report progress, and yield host/status as execution
        completes.

        :yields: (host, status)
        :raises: RuntimeError if command has not been set
        """
        if not self._command:
            raise RuntimeError('Command must be provided')

        if not self._reporter:
            self._reporter = log.reporter(self._command)

        if self._hosts:
            self._reporter.expect(len(self._hosts))
            self._reporter.start()

            for host, status, ohandler in cluster_ssh(
                    self._hosts, self._command,
                    self._user, batch_size,
                    self.max_failure,
                    self.output_handler):

                if status == 0:
                    self._reporter.add_success()
                else:
                    self.get_logger().warning(
                        '%s on %s returned [%d]: %s',
                        self._command, host, status, ohandler.output)
                    self._reporter.add_failure()

                yield host, status

            self._reporter.finish()
        else:
            self.get_logger().warning(
                'Job %s called with an empty host list.', self._command)


def cluster_ssh(
        hosts, command, user=None, limit=DEFAULT_BATCH_SIZE,
        max_fail=None, output_handler=None):
    """Run a command via SSH on multiple hosts concurrently."""
    hosts = set(hosts)
    # Ensure a minimum batch size of 1
    limit = max(limit, 1)

    max_failure = len(hosts) if max_fail is None else max_fail

    try:
        command = shlex.split(command)
    except AttributeError:
        pass

    failures = 0
    procs = {}
    output_handlers = {}
    poll = select.epoll()
    try:
        while hosts or procs:
            if hosts and len(procs) < limit:
                host = hosts.pop()
                ssh_command = SSH(host, command, user=user)
                proc = subprocess.Popen(
                    ssh_command,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    preexec_fn=os.setsid)

                procs[proc.pid] = (proc, host)
                poll.register(proc.stdout, select.EPOLLIN)
                output_handlers[proc.stdout.fileno()] = output_handler(host)

            elif procs:
                try:
                    pid, status = utils.eintr_retry(os.waitpid, -1, os.WNOHANG)
                except OSError as e:
                    # We lost track of our children somehow. So grab any child
                    # process from procs (they're all dead anyway) and pretend
                    # it exited normally.
                    # See https://bugs.python.org/issue1731717
                    if e.errno == errno.ECHILD:
                        pid = next(iter(procs))
                        status = 0
                    else:
                        raise

                for fd, event in utils.eintr_retry(poll.poll, 0.01):
                    output = utils.eintr_retry(os.read, fd, 1048576)
                    output_handlers[fd].accept(output)

                if pid:
                    status = -(status & 255) or (status >> 8)
                    if status != 0:
                        failures = failures + 1
                    proc, host = procs.pop(pid)
                    poll.unregister(proc.stdout)
                    ohandler = output_handlers.pop(proc.stdout.fileno())
                    if failures > max_failure:
                        hosts = []
                    yield host, status, ohandler
    finally:
        poll.close()
        for pid, (proc, host) in procs.items():
            proc.kill()
