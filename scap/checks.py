# -*- coding: utf-8 -*-
"""
    scap.checks
    ~~~~~~~~~~~
    Deployment checks.

    Definitions are typically loaded from YAML of the following format:

        checks:
          some_unique_check_name:
            type: command
            command: /usr/local/bin/my_special_check
            stage: promote

          some_other_check_name:
            type: nrpe
            command: some_parsed_nrpe_command_name
            stage: promote

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
import collections
import os
import select
import shlex
import subprocess
import time

from . import utils


_types = {}


class CheckInvalid(AssertionError):
    pass


def checktype(type):
    """
    Class decorator for registering a new check type.

    :param type: type name
    """
    def decorator(klass):
        register_type(type, klass)
        return klass

    return decorator


def execute(checks, logger, concurrency=1):
    """
    Execute the given checks in parallel.

    :param checks: iterable of `checks.Check` objects
    :param logger: `logging.Logger` to send messages to
    :param concurrency: level of concurrency

    :returns: tuple of the aggregate check success and list of executed checks
    :rtype: (bool, list)
    """

    epoll = select.epoll()
    todo = checks[::-1]
    doing = {}

    done = []
    success = 0

    def handle_failure(job, msg, kill=True):
        logger.warning(msg)
        handle_done(job)

        if kill:
            try:
                job.kill()
            except OSError:
                pass

    def handle_done(job):
        epoll.unregister(job.fd)
        if job.fd in doing:
            del doing[job.fd]

    try:
        while todo or doing:
            # Schedule new jobs up to the concurrency level
            while todo and len(doing) < concurrency:
                check = todo.pop()
                logger.info("Executing check '{}'".format(check.name))

                job = check.run()
                doing[job.fd] = job
                epoll.register(job.fd, select.EPOLLIN)

            # Poll for stdout events
            for (fd, event) in epoll.poll(0.01):
                job = doing[fd]

                # Handle job completion
                if job.poll() is not None:
                    job.wait()

                    if job.isfailure():
                        msg = "Check '{}' failed: {}"
                        msg = msg.format(job.check.name, job.output)
                        handle_failure(job, msg, kill=False)
                    else:
                        handle_done(job)
                        success += 1

                    done.append(job)

            # Enforce timeout on running jobs
            for job in doing.values():
                if job.timedout():
                    msg = "Check '{}' exceeded {}s timeout"
                    msg = msg.format(job.check.name, job.check.timeout)
                    handle_failure(job, msg)

    finally:
        for job in doing.values():
            msg = "Error running check '{}'".format(job.check.name)
            handle_failure(job, msg)

        epoll.close()

    return (len(done) == len(checks) == success, done)


def load(cfg):
    """
    Load checks from the given config dict.

    :param cfg: config dict
    """

    checks = collections.OrderedDict()
    if cfg and cfg.get('checks', None):
        for name, options in cfg['checks'].iteritems():
            check_type = options.get('type', 'command')

            if not options:
                check_type = 'override'

            if check_type not in _types:
                msg = "unknown check type '{}'".format(check_type)
                raise CheckInvalid(msg)

            checks[name] = _types[check_type](name=name, **options)

    return checks


def register_type(type, factory):
    """
    Register a new check type and factory.

    :param type: type name
    :param factory: callable type factory
    """

    _types[type] = factory


@checktype('command')
class Check(object):
    """
    Represent a loaded 'command' check.

    :param name: check name
    :param stage: stage after which to run the check
    :param group: deploy group for which to run the check
    :param timeout: maximum time allowed for check execution, in seconds
    :param command: check command to run
    """

    def __init__(self, name, stage, group=None, timeout=30.0, command='',
                 **opts):
        self.name = name
        self.stage = stage
        self.group = group
        self.timeout = timeout
        self.command = command
        self.options = opts

        self.validate()

    def run(self):
        """Return a running :class:`CheckJob`."""

        return CheckJob(self)

    def validate(self):
        """Validate check properties."""

        if not self.command:
            raise CheckInvalid("missing 'command'")


class CheckJob(object):
    """
    Represent and control a running check.

    A :class:`CheckJob` begins execution immediately and should be controlled
    within some kind of poll loop, typically `checks.execute`.

    """

    def __init__(self, check):
        """Inititalizes a new CheckJob and begins execution."""

        self.check = check
        self.proc = subprocess.Popen(
            shlex.split(check.command),
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        self.fd = self.proc.stdout.fileno()
        self.stream = self.proc.stdout
        self.output = ''

        self.started = time.time()
        self.ended = None

    def duration(self):
        """Return the current or final job duration."""

        ended = time.time() if self.ended is None else self.ended
        return ended - self.started

    def isfailure(self):
        """Return whether this check failed."""

        return self.proc.returncode != 0

    def kill(self):
        """Kill the executing process."""

        self.proc.kill()

    def poll(self):
        """
        Read output and polls the process for exit status.

        If the process has exited, an (approximate) end time is recorded. This
        method is non-blocking typically called within an event loop like
        `checks.execute`.
        """

        self.output += utils.eintr_retry(os.read, self.fd, 1048576)
        result = utils.eintr_retry(self.proc.poll)

        if result is not None:
            self.ended = time.time()

        return result

    def timedout(self):
        """Whether the job duration has exceeded the job timeout."""

        return self.duration() > self.check.timeout

    def wait(self):
        """
        Block for the last stdout/stderr read of the check process.

        To ensure the least amount of blocking, this method should only be
        called within an event loop once `poll` has signaled that the process
        has exited.
        """

        for output in self.proc.communicate():
            if output is not None:
                self.output += output


@checktype('override')
class OverrideCheck(object):
    """Represent a loaded 'override' check."""

    def __init__(self, name):
        """Initialize override check."""
        self.name = name

    @property
    def stage(self):
        utils.get_logger().info(
            'Check %s is empty and will not be run',
            self.name
        )
        return None
