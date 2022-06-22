# -*- coding: utf-8 -*-
"""
    scap.ssh
    ~~~~~~~~
    This module provides functions for running commands on remote hosts
    via SSH.

    Copyright © 2014-2017 Wikimedia Foundation and Contributors.

    This file is part of Scap.

    Scap is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, version 3.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""
from __future__ import absolute_import

import errno
import os
import random
import select
import shlex
import socket
import statistics
import subprocess
import time

import scap.log as log
import scap.utils as utils
import scap.cmd as cmd

CONNECTION_FAILURE = 255
DEFAULT_BATCH_SIZE = 80
SSH = cmd.Command(
    "/usr/bin/ssh",
    "-n",
    "-oBatchMode=yes",
    "-oSetupTimeout=10",
    "-F/dev/null",
    cmd.arg("verbose", "-v"),
    cmd.arg("user", "-oUser={}"),
)
SSH_WITH_KEY = cmd.Command(
    "/usr/bin/ssh",
    "-n",
    "-oBatchMode=yes",
    "-oSetupTimeout=10",
    "-oIdentitiesOnly=yes",
    "-F/dev/null",
    cmd.arg("verbose", "-v"),
    cmd.arg("user", "-oUser={}"),
    cmd.arg("key", "-oIdentityFile={}"),
)


class OutputHandler(object):
    """
    Standard handler for SSH command output from hosts.

    Simply stores output as a string for future handling.
    """

    host = None
    output = ""

    def __init__(self, host):
        self.host = host

    def accept(self, output):
        if isinstance(output, bytes):
            output = output.decode()
        self.output += output


class JSONOutputHandler(OutputHandler):
    """
    Deserialize and log structured JSON output from hosts.

    Any non-structured output is stored for future handling.
    """

    def __init__(self, host):
        super().__init__(host)
        self._logger = utils.get_logger().getChild("target").getChild(host)
        self._partial = ""

    def accept(self, output):
        """
        Extract and deserializes line-wise JSON from the given output.

        Any non-JSON is stored in self.output.
        """
        for line in self.lines(output):
            if line.startswith("{"):
                try:
                    record = log.JSONFormatter.make_record(line)
                except (ValueError, TypeError):
                    self.output += line + "\n"
                    record = None

                if record is not None:
                    # qualify the record name according to our prefix
                    record.name = self._logger.name + "." + record.name

                    # amend the record with the host name
                    record.host = self.host

                    # propagate the log record
                    self._logger.handle(record)

                    # store the output in case of error
                    self.output += record.getMessage() + "\n"
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
            output = output[pos + 1 :]
            self._partial = ""


class Job(object):
    """Execute a job on a group of remote hosts via ssh."""

    @utils.log_context("ssh.job")
    def __init__(
        self, hosts=None, command=None, user=None, logger=None, key=None, verbose=False
    ):
        self.hosts(hosts or [])
        self._command = command
        self._reporter = None
        self._user = user
        self._key = key
        self.max_failure = len(self._hosts)
        self._logger = logger
        self.output_handler = OutputHandler
        self.verbose = verbose

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
        self.hosts([h for h in self._hosts if socket.getfqdn(h) not in exclude])

    def command(self, command):
        """Set command to run."""
        self._command = command
        return self

    def progress(self, reporter):
        """Set the reporter used when reporting progress."""
        self._reporter = reporter
        return self

    def run(self, batch_size=DEFAULT_BATCH_SIZE, return_jobresults=False):
        """
        Run the job, report progress, and return success/failed counts
        or a JobResults object if return_jobresults is truthy.

        :returns: (ok, failed) counts of successful/failed hosts
                  or a JobResults object if return_jobresults is truthy.
        :raises: RuntimeError if command has not been set
        """
        results = JobResults()

        for jobresult in self.run_with_status(batch_size):
            results.add(jobresult)

        if return_jobresults:
            return results

        return results.num_ok, results.num_failed

    def run_with_status(self, batch_size=DEFAULT_BATCH_SIZE):
        """
        Run the job, report progress, and yield JobResult objects as execution
        completes.

        :yields: JobResult
        :raises: RuntimeError if command has not been set
        """
        if not self._command:
            raise RuntimeError("Command must be provided")

        if not self._reporter:
            self._reporter = log.reporter(self._command)

        if self._hosts:
            self._reporter.expect(len(self._hosts))
            self._reporter.start()

            for jobresult in cluster_ssh(
                self._hosts,
                self._command,
                self._user,
                self._key,
                batch_size,
                self.max_failure,
                self.output_handler,
                self.verbose,
                self._reporter,
            ):

                host = jobresult.host
                status = jobresult.status
                output = jobresult.output
                if status == 0:
                    self._reporter.add_success()
                else:
                    self.get_logger().warning(
                        "%s (ran as %s@%s) returned [%d]: %s",
                        self._command,
                        self._user,
                        host,
                        status,
                        output,
                    )
                    self._reporter.add_failure()

                yield jobresult

            self._reporter.finish()
        else:
            self.get_logger().warning(
                "Job %s called with an empty host list.", self._command
            )


class JobResult(object):
    def __init__(self, proc: subprocess.Popen, host: str):
        self.starttime = time.time()
        self.proc = proc
        self.host = host
        self.duration = None
        self.status = None
        self.output = None

    def finish(self, status: int, output: str):
        self.duration = time.time() - self.starttime
        self.status = status
        self.output = output


class JobResults(object):
    def __init__(self):
        self.results = []
        self.num_ok = 0
        self.num_failed = 0

    def add(self, jobresult: JobResult):
        self.results.append(jobresult)
        if jobresult.status == 0:
            self.num_ok += 1
        else:
            self.num_failed += 1

    def average_duration(self):
        if len(self.results) == 0:
            return 0

        return statistics.mean([jr.duration for jr in self.results])

    def median_duration(self):
        if len(self.results) == 0:
            return 0

        return statistics.median([jr.duration for jr in self.results])

    def __iter__(self):
        return iter(self.results)


def cluster_ssh(
    hosts,
    command,
    user=None,
    key=None,
    limit=DEFAULT_BATCH_SIZE,
    max_fail=None,
    output_handler=None,
    verbose=False,
    reporter=None,
):
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
    # key is pid, value is a JobResult
    procs = {}
    # key is file descriptor, value is an OutputHandler (or subclass) object
    output_handlers = {}
    poll = select.epoll()
    try:
        while hosts or procs:
            if hosts and len(procs) < limit:
                host = hosts.pop()

                if key:
                    ssh_cmd = SSH_WITH_KEY(
                        host, command, user=user, key=key, verbose=verbose
                    )
                else:
                    ssh_cmd = SSH(host, command, user=user, verbose=verbose)

                proc = subprocess.Popen(
                    ssh_cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    preexec_fn=os.setsid,
                )

                procs[proc.pid] = JobResult(proc, host)
                poll.register(proc.stdout, select.EPOLLIN)
                output_handlers[proc.stdout.fileno()] = output_handler(host)

                if reporter:
                    reporter.add_in_flight()

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
                    output_handlers[fd].accept(output.decode("UTF-8"))

                if pid:
                    status = -(status & 255) or (status >> 8)
                    if status != 0:
                        failures = failures + 1
                    jobresult = procs.pop(pid)
                    proc = jobresult.proc
                    poll.unregister(proc.stdout)
                    ohandler = output_handlers.pop(proc.stdout.fileno())
                    jobresult.finish(status, ohandler.output)
                    if failures > max_failure:
                        hosts = []
                    yield jobresult
    finally:
        poll.close()
        for pid, jobresult in procs.items():
            jobresult.proc.kill()
