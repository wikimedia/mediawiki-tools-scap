# -*- coding: utf-8 -*-
"""
    scap.log
    ~~~~~~~~
    Helpers for routing and formatting log data.

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
import contextlib
import fnmatch
from functools import partial
import itertools
import json
import logging
import logging.handlers
import math
import operator
import os
import queue
import re
import shlex
import socket
import sys
import threading
import time
import traceback

import pygments

try:
    from pygments.formatters import TerminalFormatter
    from pygments.lexers.diff import DiffLexer
except ImportError:
    DiffLexer = None

import scap.utils as utils
import scap.interaction as interaction
import scap.spiderpig.io

# Format string for log messages. Interpolates LogRecord attributes.
# See <http://docs.python.org/2/library/logging.html#logrecord-attributes>
# for attribute names you can include here.
CONSOLE_LOG_FORMAT = "%(asctime)s %(levelname)-8s - %(message)s"
TIMESTAMP_FORMAT = "%H:%M:%S"


class AnsiColorFormatter(logging.Formatter):
    """Colorize output according to logging level."""

    colors = {
        "CRITICAL": "41;37",  # white on red
        "ERROR": "31",  # red
        "WARNING": "33",  # yellow
        "INFO": "32",  # green
        "DEBUG": "36",  # cyan
    }

    def __init__(self, fmt=None, datefmt=None, colors=None, colorize="auto"):
        """
        :param fmt: Message format string
        :param datefmt: Time format string
        :param colors: Dict of {'levelname': ANSI SGR parameters}
        :param colorize: Set to true to colorize messages based on log level.
                         Default 'auto' enable color when stderr is a tty or
                         'FORCE_COLOR' is found in the environment.

        .. seealso:: https://en.wikipedia.org/wiki/ANSI_escape_code
        """
        super().__init__(fmt, datefmt)

        if colors:
            self.colors.update(colors)

        if colorize == "auto":
            self.colorize = utils.should_colorize_output()
        else:
            self.colorize = colorize

    def format(self, record):
        msg = super().format(record)
        if not self.colorize:
            return msg

        color = self.colors.get(record.levelname, "0")
        return "\x1b[%sm%s\x1b[0m" % (color, msg)


class DiffLogFormatter(AnsiColorFormatter):
    lex = None
    formatter = None

    def __init__(self, fmt=None, datefmt=None, colors=None, colorize="auto"):
        if DiffLexer:
            self.lex = DiffLexer()
            self.formatter = TerminalFormatter()
        super().__init__(fmt, datefmt, colors, colorize)

    def format(self, record):
        if getattr(record, "type", None) == "config_diff":
            if self.lex:
                return pygments.highlight(record.output, self.lex, self.formatter)
            return record.output
        return super().format(record)


class IRCSocketHandler(logging.Handler):
    """
    Log handler for logmsgbot on #wikimedia-operation.

    Sends log events to a tcpircbot server for relay to an IRC channel.
    """

    def __init__(self, host, port, timeout=1.0):
        """
        :param host: tcpircbot host
        :type host: str
        :param port: tcpircbot listening port
        :type port: int
        :param timeout: timeout for sending message
        :type timeout: float
        """
        super().__init__()
        self.addr = (host, port)
        self.level = logging.INFO
        self.timeout = timeout

    def emit(self, record):
        message = "!log %s@%s %s" % (
            utils.get_real_username(),
            socket.gethostname(),
            record.getMessage(),
        )
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.timeout)
            sock.connect(self.addr)
            sock.sendall(message.encode("utf-8"))
            sock.close()
        except (socket.timeout, socket.error, socket.gaierror):
            self.handleError(record)


class JSONFormatter(logging.Formatter):
    """
    Serialize logging output as JSON.

    Can be used to maintain logged event structure between the deployment host
    and remote targets.
    """

    DEFAULTS = [
        ("name", ""),
        ("levelno", logging.INFO),
        ("filename", None),
        ("lineno", None),
        ("msg", ""),
        ("args", ()),
        ("exc_info", None),
        ("funcName", None),
    ]

    # LogRecord fields that we omit because we can't reliably serialize them
    UNSERIALIZABLE = {"exc_info"}

    # Native fields that we propagate as is
    PRESERVED = {"created", "msecs", "relativeCreated", "exc_text"}

    # Extrapolate efficient sets of mapped and built-in LogRecord attributes
    FIELDS = ({k for k, _ in DEFAULTS} | PRESERVED) - UNSERIALIZABLE
    NATIVE = set(logging.LogRecord(*[v for _, v in DEFAULTS]).__dict__.keys())

    @staticmethod
    def make_record(data):
        fields = json.loads(data)
        args = []
        for k, v in JSONFormatter.DEFAULTS:
            val = fields.get(k, v)

            if k == "args":
                val = tuple(val)

            args.append(val)

        record = logging.LogRecord(*args)

        for k in fields:
            if k in JSONFormatter.PRESERVED or k not in JSONFormatter.NATIVE:
                record.__dict__[k] = fields[k]

        return record

    def format(self, record):
        rec = record.__dict__
        fields = {k: rec[k] for k in rec if self._isvalid(k)}

        # We can't serialize `exc_info` so preserve it as a formatted string
        if rec.get("exc_info", None):
            fields["exc_text"] = self.formatException(rec["exc_info"])

        def serialize_obj(obj):
            if hasattr(obj, "getvalue"):
                return obj.getvalue()
            if hasattr(obj, "__dict__"):
                return obj.__dict__
            return None

        return json.dumps(fields, default=serialize_obj, skipkeys=True)

    def _isvalid(self, field):
        return field in self.FIELDS or field not in self.NATIVE


class LogstashFormatter(logging.Formatter):
    """Format log messages for logstash."""

    def __init__(self, fmt=None, datefmt="%Y-%m-%dT%H:%M:%SZ", log_type="scap"):
        """
        :param fmt: Message format string (not used)
        :param datefmt: Time format string
        :param type: Logstash event type
        """
        super().__init__(fmt, datefmt)
        self.type = log_type
        self.host = socket.gethostname()
        self.script = sys.argv[0]
        self.user = utils.get_real_username()

    def format(self, record):
        """Format a record as a logstash v1 JSON string."""
        fields = record.__dict__.copy()

        # Ensure message is populated
        if "message" not in fields:
            try:
                if "args" in fields and fields["args"]:
                    fields["message"] = fields["msg"] % fields["args"]
                else:
                    fields["message"] = fields["msg"]
            except TypeError as typee:
                # This sometimes happens if the fields['msg'] has a
                # '%<something>' in it some place.
                #
                # Re-raising and dumping the message and fields seems like it
                # may be more helpful in tracking down the root cause of an
                # error than the output, "not enough arguments for format
                # string"
                raise TypeError(
                    "error: ({}); "
                    "format string: ({}); "
                    "arguments: ({})".format(str(typee), fields["msg"], fields["args"])
                )

        # Format exception
        if "exc_info" in fields and fields["exc_info"]:
            fields["exception"] = self.formatException(fields["exc_info"])

        # Remove fields
        ignore_fields = [
            "args",
            "asctime",
            "created",
            "exc_info",
            "exc_text",
            "filename",
            "levelno",
            "module",
            "msecs",
            "msg",
            "processName",
            "relativeCreated",
            "thread",
            "threadName",
            "stack_info",
        ]
        for field in ignore_fields:
            fields.pop(field, None)

        logger = fields.pop("name", "unnamed")

        logstash_record = {
            "ecs": {"version": "1.7.0"},
            "@timestamp": self.formatTime(record, self.datefmt),
            "host": {
                "name": self.host,
            },
            "labels": {"channel": logger},
            "log": {
                "level": fields.pop("levelname", None),
                "logger": logger,
                "origin": {
                    "file": {
                        "name": fields.pop("pathname", None),
                        "line": fields.pop("lineno", None),
                    },
                    "function": fields.pop("funcName", None),
                },
            },
            "process": {
                "executable": self.script,
                "pid": fields.pop("process", None),
            },
            "service": {
                "type": self.type,
            },
        }
        logstash_record.update(fields)

        effective_user = {
            "name": self.user,
        }

        user = logstash_record.get("user")

        if user:
            assert isinstance(user, dict)
            # NOTE: This modifies a caller-supplied dict
            user["effective"] = effective_user
        else:
            logstash_record["user"] = effective_user

        return json.dumps(logstash_record, default=str)

    def formatException(self, exc_info):
        """Format the given exception as a dict."""
        ex_type, ex_value, ex_traceback = exc_info
        return {
            "class": ex_type.__name__,
            "message": "%s" % ex_value,
            "stacktrace": [
                {"file": fname, "line": line, "function": func, "text": text}
                for fname, line, func, text in traceback.extract_tb(ex_traceback)
            ],
        }


class SyslogFormatter(LogstashFormatter):
    def format(self, record):
        # Add the 'cee cookie' to signal json-in-syslog
        # The cookie is recognized by rsyslog for json parsing:
        # https://www.rsyslog.com/doc/v8-stable/configuration/modules/mmjsonparse.html
        return "scap: @cee: " + super().format(record)


def reporter(name, mute=False):
    """
    Instantiate progress reporter

    :name: - Name of operation being monitored
    """
    if mute:
        return MuteReporter()

    if interaction.spiderpig_mode():
        return SpiderpigProgressReporter(name)

    if not sys.stdout.isatty():
        return RateLimitedProgressReporter(name)

    return ProgressReporter(name)


class ProgressReporter(object):
    """
    Track and display progress of a process.

    Report on the status of a multi-step process by displaying the completion
    percentage and success, failure and remaining task counts on a single
    output line.
    """

    def __init__(self, name, expect=0, fd=sys.stderr, spinner=None):
        """
        :param name: Name of operation being monitored
        :param expect: Number of results to expect
        :param fd: File handle to write status messages to
        :param spinner: Cyclical iterator that returns progress spinner.
        """
        if spinner is None:
            spinner = itertools.cycle(["-", "\\", "|", "/"])

        self._name = name
        self._expect = expect
        self._done = 0
        self._ok = 0
        self._failed = 0
        self._in_flight = None
        self._fd = fd
        self._spinner = spinner
        self._finished = False

    @property
    def ok(self):
        return self._ok

    @property
    def failed(self):
        return self._failed

    @property
    def remaining(self):
        if self._in_flight is not None:
            return self._expect - self._done - self._in_flight
        else:
            return self._expect - self._done

    @property
    def done(self):
        return self._done

    @property
    def percent_complete(self) -> int:
        return math.floor(100.0 * (float(self._done) / max(self._expect, 1)))

    def expect(self, count):
        """Set expected result count."""
        self._expect = count

    def refresh(self):
        """Refresh/redraw progress output."""
        self._progress()

    def start(self):
        """Start tracking progress."""
        self._progress()

    def finish(self):
        """Finish tracking progress."""
        self._finished = True
        self._progress()
        if sys.stdout.isatty():
            self._fd.write("\n")

    def add_in_flight(self):
        if self._in_flight is None:
            self._in_flight = 1
        else:
            self._in_flight += 1
        self._progress()

    def add_success(self):
        """Record a sucessful task completion."""
        self._done += 1
        self._ok += 1
        if self._in_flight is not None:
            self._in_flight -= 1
        self._progress()

    def set_success(self, value):
        """Sets the number of done/successful jobs to the specified value."""
        if self._in_flight is not None:
            raise Exception("set_success cannot be used along with add_in_flight")
        self._done = self._ok = value
        self._progress()

    def add_failure(self):
        """Record a failed task completion."""
        self._done += 1
        self._failed += 1
        if self._in_flight is not None:
            self._in_flight -= 1
        self._progress()

    def _progress(self):
        show_spinner = not self._finished

        if sys.stdout.isatty():
            fmt = "%-80s\r"
        else:
            fmt = "%-80s\n"
            show_spinner = False

        output = "%s %s: %3.0f%% (%sok: %d; fail: %d; left: %d) %s" % (
            time.strftime(TIMESTAMP_FORMAT),
            self._name,
            self.percent_complete,
            ""
            if self._in_flight is None
            else "in-flight: {}; ".format(self._in_flight),
            self.ok,
            self.failed,
            self.remaining,
            next(self._spinner) if show_spinner else "",
        )

        self._fd.write(fmt % output)


class RateLimitedProgressReporter(ProgressReporter):
    """
    The same as ProgressReporter, but doesn't generate output more than
    once every max_reporting_interval seconds.  The final progress report
    is always generated.
    """

    def __init__(self, *args, **kwargs):
        self._last_report_time = 0
        self._max_reporting_interval = kwargs.get("max_reporting_interval", 30)
        if "max_reporting_interval" in kwargs:
            del kwargs["max_reporting_interval"]
        super().__init__(*args, **kwargs)

    def _progress(self, *args, **kwargs):
        now = time.time()

        if (
            not self._finished
            and now - self._last_report_time < self._max_reporting_interval
        ):
            # Not enough time has elapsed since the last report
            return

        super()._progress(*args, **kwargs)

        self._last_report_time = now


class SpiderpigProgressReporter(RateLimitedProgressReporter):
    def _progress(self, *args, **kwargs):
        super()._progress(*args, **kwargs)

        scap.spiderpig.io.report_progress(
            scap.spiderpig.io.SpiderpigProgressReportRecord(
                name=self._name,
                totalTasks=self._expect,
                tasksInFlight=self._in_flight,
                tasksFinishedOk=self.ok,
                tasksFinishedFailed=self.failed,
                tasksFinishedTotal=self._done,
                tasksPercentComplete=self.percent_complete,
                tasksRemaining=self.remaining,
                progressFinished=self._finished,
            )
        )


class MuteReporter(ProgressReporter):
    """A report that declines to report anything."""

    def __init__(self, name="", expect=0, fd=sys.stderr):
        super().__init__(name)

    def _progress(self):
        pass

    def finish(self):
        pass


class QueueReporter(ProgressReporter):
    """
    A ProgressReporter which sends its state-changing operations upstream via
    a queue.  It does not generate any output.
    """

    def __init__(self, name, expect=0, fd=sys.stderr, queue=None):
        self.queue = queue
        super().__init__(name, expect, fd)

    def _send(self, message):
        self.queue.put((threading.get_ident(), message))

    def expect(self, count):
        self._send(("expect", count))

    def start(self):
        pass

    def finish(self):
        pass

    def add_in_flight(self):
        self._send(("add_in_flight",))

    def add_success(self):
        self._send(("add_success",))

    def add_failure(self):
        self._send(("add_failure",))


def queue_reader(name, queue):
    r = reporter(name)
    r.start()

    expects = {}

    while True:
        (worker_id, message) = queue.get()
        operation = message[0]
        if operation == "expect":
            expects[worker_id] = message[1]
            r.expect(sum(expects.values()))
        elif operation == "add_in_flight":
            r.add_in_flight()
        elif operation == "add_success":
            r.add_success()
        elif operation == "add_failure":
            r.add_failure()
        elif operation == "terminate":
            break
    r.finish()


@contextlib.contextmanager
def MultithreadedProgressReportCollection(name):
    q = queue.Queue()

    t = threading.Thread(
        name="Queue reader", target=queue_reader, args=(name, q), daemon=True
    )
    t.start()

    try:
        yield q
    finally:
        q.put((None, ("terminate",)))
        t.join()


class DeployLogFormatter(JSONFormatter):
    """Ensure that all `deploy.log` records contain a host attribute."""

    def format(self, record):
        if not hasattr(record, "host"):
            record.host = socket.gethostname()

        return super().format(record)


class DeployLogHandler(logging.FileHandler):
    """Handler for `scap/deploy.log`."""

    def __init__(self, log_file):
        super().__init__(log_file)
        self.setFormatter(DeployLogFormatter())
        self.setLevel(logging.DEBUG)


class Filter(object):
    """
    Generic log filter that matches record attributes against criteria.

    You can provide either a glob pattern, regular expression, or lambda as
    each attribute criterion, and invert the logic by passing filter=False.

    Examples::

        Filter({'name': '*.target.*', 'host': 'scap-target-01'})
        Filter({'msg': re.compile('some annoying (message|msg)')})
        Filter({'levelno': lambda lvl: lvl < logging.WARNING})
        Filter({'name': '*.target.*'}, invert=False)

    Equivalent DSL examples::

        Filter.loads('name == *.target.* host == scap-target-01')
        Filter.loads('msg ~ "some annoying (message|msg)"')
        Filter.loads('levelno < WARNING')f
        Filter.loads('name == *.target.*', invert=False)
    """

    OPERATORS = {"=", "==", "~", ">", ">=", "<", "<="}
    COMPARISONS = {">": "gt", ">=": "ge", "<": "lt", "<=": "le"}
    LOG_LEVELS = ["CRITICAL", "ERROR", "WARNING", "INFO", "DEBUG"]

    @staticmethod
    def loads(expression, invert=True):
        """
        Construct a `Filter` from the given free-form expression.

        See :class:`Filter` for examples.
        """

        criteria = []

        for lhs, op, rhs in Filter.parse(expression):
            if lhs == "levelno" and rhs in Filter.LOG_LEVELS:
                criterion = getattr(logging, rhs)
            elif rhs.isdigit():
                criterion = int(rhs)
            else:
                criterion = rhs

            if op in Filter.COMPARISONS:
                # map to a "rich comparison" operator
                # e.g. foo < 10 becomes `operator.lt(foo, 10)`
                func = getattr(operator, Filter.COMPARISONS[op])
                criterion = partial(lambda f, c, v: f(v, c), func, criterion)
            elif op == "~":
                criterion = re.compile(criterion)

            criteria.append((lhs, criterion))

        return Filter(criteria, invert=invert)

    @staticmethod
    def parse(expression):
        """
        Parse the given filter expression and generates its parts.

        :param expression: Filter expression.
        :type expression: str

        :yields: (lhs, op, rhs)
        """

        parts = shlex.split(expression)

        # check that we're dealing with tuples of 3s (lhs, operator, rhs)
        if len(parts) % 3 > 0:
            raise ValueError("invalid expression '{}'".format(expression))

        for i in range(0, len(parts), 3):
            lhs, op, rhs = parts[i : i + 3]

            if op not in Filter.OPERATORS:
                raise ValueError("invalid operator '{}'".format(op))

            yield lhs, op, rhs

    def __init__(self, criteria, invert=True):
        self._invert = invert
        self.criteria = []
        self.append(criteria)

    def append(self, criteria):
        """
        Append the filter with the given criteria.

        :param criteria: Filter criteria
        :type criteria: iter
        """

        if hasattr(criteria, "items"):
            criteria = criteria.items()

        # Normalize all globs into regexs into lambdas
        for attr, criterion in criteria:
            if isinstance(criterion, str):
                criterion = re.compile(fnmatch.translate(criterion))

            if not hasattr(criterion, "__call__"):
                criterion = partial(lambda c, v: c.search(v), criterion)

            self.criteria.append((attr, criterion))

    def filter(self, record):
        """
        Perform filtering on a given log record.

        :param record: Log record.
        :type record: LogRecord
        """

        record = record.__dict__
        matches = True

        for attr, criterion in self.criteria:
            if attr not in record or not criterion(record.get(attr)):
                matches = False
                break

        if self._invert:
            return not matches
        return matches

    def isfiltering(self, attribute):
        """Whether the filter has criteria for the given attribute."""

        return any(attr == attribute for attr, _ in self.criteria)


class Stats(object):
    """
    A simple StatsD metric client.

    It can log measurements and counts to a remote StatsD host.
    See <https://github.com/etsy/statsd/wiki/Protocol> for details.
    """

    @utils.log_context("stats")
    def __init__(self, host, port, logger=None):
        self.logger = logger
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.address = (host, port)

    def timing(self, name, milliseconds):
        """Report a timing measurement in milliseconds."""
        metric = "%s:%s|ms" % (name, int(round(milliseconds)))
        self._send_metric(metric)

    def increment(self, name, value=1):
        """Increment a measurement."""
        metric = "%s:%s|c" % (name, value)
        self._send_metric(metric)

    def _send_metric(self, metric):
        try:
            if isinstance(metric, str):
                metric = metric.encode("UTF-8")
            assert isinstance(metric, bytes)
            self.socket.sendto(metric, self.address)
        except Exception:
            self.logger.exception('Failed to send metric "%s"', metric)


class Timer(object):
    """
    Context manager to track and record the time taken to execute a block.

    Elapsed time will be recorded to a logger and optionally a StatsD server.

    >>> with Timer('example'):
    ...     time.sleep(0.1)

    >>> s = Stats('127.0.0.1', 2003)
    >>> with Timer('example', stats=s):
    ...     time.sleep(0.1)
    """

    @utils.log_context("timer")
    def __init__(self, description, name="unsupplied", stats=None, logger=None):
        """
        :param description: The human friendly description of the operation being timed.
        :type description: str
        :param name: The measurement name to use when transmitting the operation time the stats recording system.

                     If `name` is not supplied, the description will be used as the measurement name.

                     If None, no measurement will be transmitted.

                     Note that the measurement name will have "scap." prefixed to it and any non-word
                     characters in the name will be replaced with underscores.
        :type name: str|None
        :param stats: StatsD client to record block invocation and duration
        :type stats: scap.log.Stats
        """
        self.description = description
        self.name = description if name == "unsupplied" else name
        self.stats = stats
        self.logger = logger
        self.start = None
        self.end = None

    def __enter__(self):
        """
        Enter the runtime context.

        :returns: self
        """
        self.start = time.time()
        self.logger.info(
            "Started %s" % self.description,
            extra={
                "event.action": self.name,
                "event.start": int(self.start * pow(10, 3)),
            },
        )
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Exit the runtime context."""
        self.end = time.time()
        self._record_elapsed(self.end - self.start)

    def _record_elapsed(self, elapsed):
        """
        Log the elapsed duration.

        :param elapsed: Elapsed duration
        :type elapsed: float
        """

        extras = {
            "event.action": self.name,
            "event.start": int(self.start * pow(10, 3)),
            "event.duration": int(elapsed * pow(10, 9)),  # nanoseconds
        }
        if self.end is not None:
            extras["event.end"] = int(self.end * pow(10, 3))

        self.logger.info(
            "Finished %s (duration: %s)",
            self.description,
            utils.human_duration(elapsed),
            extra=extras,
        )
        if self.name and self.stats:
            massaged_name = re.sub(r"\W", "_", self.name.lower())
            self.stats.timing("scap.%s" % massaged_name, elapsed * 1000)


class Udp2LogHandler(logging.handlers.DatagramHandler):
    """Log handler for udp2log."""

    def __init__(self, host, port, prefix="scap"):
        """
        :param host: Hostname or ip address
        :param port: Port
        :param prefix: Line prefix (udp2log destination)
        """
        super().__init__(host, port)
        self.prefix = prefix

    def makePickle(self, record):
        """
        Format record as a udp2log packet.

        >>> Udp2LogHandler('127.0.0.1', 12345).makePickle(
        ...     logging.makeLogRecord({'msg':'line1\\nline2'}))
        'scap line1\\nscap line2\\n'
        >>> Udp2LogHandler('127.0.0.1', 12345).makePickle(
        ...     logging.makeLogRecord({'msg':'%s12'% ('0'*65500)}))
        ...     # doctest: +ELLIPSIS
        'scap 00000...00001\\n'
        """
        text = self.format(record)
        if self.prefix:
            text = re.sub(r"^", self.prefix + " ", text, flags=re.MULTILINE)
        if len(text) > 65506:
            text = text[:65506]
        if text[-1] != "\n":
            text = text + "\n"
        return text.encode()


def setup_loggers(cfg, console_level=logging.INFO, handlers=None):
    """
    Setup the logging system.

    * Configure the root logger to use :class:`DiffLogFormatter`
    * Optionally add a :class:`Udp2LogHandler` to send logs to a udp2log server
    * Optional add a :class:`IRCSocketHandler` for the `scap.announce` log
      channel to send messages to a tcpircbot server

    :param cfg: Dict of global configuration values
    :param console_level: Logging level for the local console appender
    :param handlers: Additional handlers
    """

    # The INFO level for scap.sh is pretty verbose
    if console_level == logging.DEBUG:
        logging.getLogger("scap.sh").setLevel(logging.INFO)

    # Set logger levels
    logging.root.setLevel(logging.DEBUG)
    logging.root.handlers[0].setLevel(console_level)

    # Log messages matching these filters will be prevented from reaching the console.
    logging.root.handlers[0].addFilter(Filter({"name": "target.*"}))

    # Normally we don't want scap.k8s.build and scap.k8s.deploy channel debug messages to reach the console, but we
    # do want to see them when scap is run with the -v flag (which causes console_level to be logging.DEBUG).
    logging.root.handlers[0].addFilter(
        Filter({"name": "scap.k8s.build", "levelno": lambda lvl: lvl < console_level})
    )
    logging.root.handlers[0].addFilter(
        Filter({"name": "scap.k8s.deploy", "levelno": lambda lvl: lvl < console_level})
    )

    if cfg["log_json"]:
        logging.root.handlers[0].setFormatter(JSONFormatter())
    else:
        logging.root.handlers[0].setFormatter(
            DiffLogFormatter("%(asctime)s %(message)s", "%H:%M:%S")
        )

    if cfg["udp2log_host"]:
        # Send a copy of all logs to the udp2log relay
        udp_handler = Udp2LogHandler(cfg["udp2log_host"], int(cfg["udp2log_port"]))
        udp_handler.setLevel(logging.DEBUG)
        udp_handler.setFormatter(LogstashFormatter())
        logging.root.addHandler(udp_handler)

    if cfg["use_syslog"]:
        # Send a copy of all logs to local syslog
        syslog_handler = logging.handlers.SysLogHandler("/dev/log")
        syslog_handler.setLevel(logging.DEBUG)
        syslog_handler.setFormatter(SyslogFormatter())
        logging.root.addHandler(syslog_handler)

    if cfg["tcpircbot_host"]:
        # Send 'scap.announce' messages to irc relay
        irc_logger = logging.getLogger("scap.announce")
        irc_logger.addHandler(
            IRCSocketHandler(cfg["tcpircbot_host"], int(cfg["tcpircbot_port"]))
        )

    if handlers is not None:
        for handler in handlers:
            logging.root.addHandler(handler)


def log_large_message(message, logger, log_level):
    """
    Logs 'message' to 'logger' at the specified 'log_level'.
    'message' is broken into multiple messages if it exceeds
    MAX_MESSAGE_SIZE.
    """
    MAX_MESSAGE_SIZE = 50000
    num_segments = math.ceil(len(message) / MAX_MESSAGE_SIZE)

    if num_segments <= 1:
        logger.log(log_level, "%s", message)
        return

    for i in range(num_segments):
        logger.log(
            log_level,
            "[%d/%d] %s",
            i + 1,
            num_segments,
            message[:MAX_MESSAGE_SIZE],
        )
        message = message[MAX_MESSAGE_SIZE:]


@contextlib.contextmanager
def pipe(logger=None, level=logging.INFO):
    """
    Yields a write-only file descriptor that forwards lines to the given
    logger.
    """
    if logger is None:
        logger = logging.getLogger()

    (reader, writer) = os.pipe()

    def log_lines(logger, level, fd):
        with os.fdopen(fd) as reader:
            for line in reader:
                logger.log(level, line.rstrip())

    thread = threading.Thread(
        target=log_lines,
        args=(
            logger,
            level,
            reader,
        ),
    )

    thread.start()

    try:
        yield writer
    finally:
        # Note the reader is closed by os.fdopen in log_lines
        os.close(writer)
        thread.join(timeout=10)
