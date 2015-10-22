# -*- coding: utf-8 -*-
"""
    scap.checks
    ~~~~~~~~
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
"""
import os
import select
import shlex
import subprocess
import time
import yaml

_types = {}


class CheckInvalid(AssertionError):
    pass


def checktype(type):
    """Class decorator for registering a new check type.

    :param type: type name
    """

    def decorator(klass):
        register_type(type, klass)
        return klass

    return decorator


def execute(checks, logger, concurrency=2, timeout=30):
    """Executes the given checks in parallel.

    :param checks: iterable of `checks.Check` objects
    :param logger: `logging.Logger` to send messages to
    :param concurrency: level of concurrency
    :param timeout: maximum time allowed for check execution

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
                if job.duration() >= timeout:
                    msg = "Check '{}' exceeded {}s timeout"
                    msg = msg.format(job.check.name, timeout)
                    handle_failure(job, msg)

    finally:
        for job in doing.values():
            msg = "Error running check '{}'".format(job.check.name)
            handle_failure(job, msg)

        epoll.close()

    return (len(done) == len(checks) == success, done)


def load(string):
    """Load checks from the given YAML.

    :param string: YAML check configuration
    """

    checks = {}
    definitions = yaml.load(string)['checks']

    for name in definitions:
        options = definitions[name]

        check_type = options.get('type', 'command')

        if check_type not in _types:
            raise CheckInvalid("unknown check type '{}'".format(check_type))

        checks[name] = _types[check_type](name=name, **options)

    return checks


def register_type(type, factory):
    """Register a new check type and factory.

    :param type: type name
    :param factory: callable type factory
    """

    _types[type] = factory


@checktype('command')
class Check(object):
    """Represents a loaded 'command' check."""

    def __init__(self, name, stage, group=None, command='', **opts):
        self.name = name
        self.stage = stage
        self.group = group
        self.command = command
        self.options = opts

        self.validate()

    def run(self):
        """Returns a running :class:`CheckJob`."""

        return CheckJob(self)

    def validate(self):
        """Validates check properties."""

        if not self.command:
            raise CheckInvalid("missing 'command'")


class CheckJob(object):
    """Represents and controls a running check.

    A :class:`CheckJob` begins execution immediately and should be controlled
    within some kind of poll loop, typically `checks.execute`.

    """

    def __init__(self, check):
        """Inititalizes a new CheckJob and begins execution."""

        self.check = check
        self.proc = subprocess.Popen(shlex.split(check.command),
                                     stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT)
        self.fd = self.proc.stdout.fileno()
        self.stream = self.proc.stdout
        self.output = ''

        self.started = time.time()
        self.ended = None

    def duration(self):
        """Returns the current or final job duration."""

        ended = time.time() if self.ended is None else self.ended
        return ended - self.started

    def isfailure(self):
        """Returns whether this check failed."""

        return self.proc.returncode != 0

    def kill(self):
        """Kills the executing process."""

        self.proc.kill()

    def poll(self):
        """Reads output and polls the process for exit status.

        If the process has exited, an (approximate) end time is recorded. This
        method is non-blocking typically called within an event loop like
        `checks.execute`.
        """

        self.output += os.read(self.fd, 1048576)
        result = self.proc.poll()

        if result is not None:
            self.ended = time.time()

        return result

    def wait(self):
        """Blocks for the last stdout/stderr read of the check process.

        To ensure the least amount of blocking, this method should only be
        called within an event loop once `poll` has signaled that the process
        has exited.
        """

        for output in self.proc.communicate():
            if output is not None:
                self.output += output
