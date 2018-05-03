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
from __future__ import absolute_import

import fnmatch
from functools import partial
import json
import logging
import logging.handlers
import math
import operator
import re
import shlex
import socket
import sys
import time
import traceback

import pygments

try:
    from pygments.formatters import TerminalFormatter
    from pygments.lexers.diff import DiffLexer
except ImportError:
    DiffLexer = None

from scap.terminal import TERM
import scap.utils as utils

# Format string for log messages. Interpolates LogRecord attributes.
# See <http://docs.python.org/2/library/logging.html#logrecord-attributes>
# for attribute names you can include here.
CONSOLE_LOG_FORMAT = '%(asctime)s %(levelname)-8s - %(message)s'


class AnsiColorFormatter(logging.Formatter):
    """Colorize output according to logging level."""

    colors = {
        'CRITICAL': '41;37',  # white on red
        'ERROR': '31',        # red
        'WARNING': '33',      # yellow
        'INFO': '32',         # green
        'DEBUG': '36',        # cyan
    }

    def __init__(self, fmt=None, datefmt=None, colors=None):
        """
        :param fmt: Message format string
        :param datefmt: Time format string
        :param colors: Dict of {'levelname': ANSI SGR parameters}

        .. seealso:: https://en.wikipedia.org/wiki/ANSI_escape_code
        """
        super(AnsiColorFormatter, self).__init__(fmt, datefmt)
        if colors:
            self.colors.update(colors)

    def format(self, record):
        msg = super(AnsiColorFormatter, self).format(record)
        color = self.colors.get(record.levelname, '0')
        return '\x1b[%sm%s\x1b[0m' % (color, msg)


class DiffLogFormatter(AnsiColorFormatter):
    lex = None
    formatter = None

    def __init__(self, fmt=None, datefmt=None, colors=None):
        if DiffLexer:
            self.lex = DiffLexer()
            self.formatter = TerminalFormatter()
        super(DiffLogFormatter, self).__init__(fmt, datefmt, colors)

    def format(self, record):

        if getattr(record, 'type', None) == 'config_diff':
            if self.lex:
                return pygments.highlight(record.output, self.lex,
                                          self.formatter)
            return record.output
        return super(DiffLogFormatter, self).format(record)


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
        super(IRCSocketHandler, self).__init__()
        self.addr = (host, port)
        self.level = logging.INFO
        self.timeout = timeout

    def emit(self, record):
        message = '!log %s@%s %s' % (
            utils.get_real_username(),
            socket.gethostname(),
            record.getMessage())
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.timeout)
            sock.connect(self.addr)
            sock.sendall(message)
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
        ('name', ''),
        ('levelno', logging.INFO),
        ('filename', None),
        ('lineno', None),
        ('msg', ''),
        ('args', ()),
        ('exc_info', None),
        ('funcName', None)]

    # LogRecord fields that we omit because we can't reliably serialize them
    UNSERIALIZABLE = {'exc_info'}

    # Native fields that we propagate as is
    PRESERVED = {'created', 'msecs', 'relativeCreated', 'exc_text'}

    # Extrapolate efficient sets of mapped and built-in LogRecord attributes
    FIELDS = ({k for k, _ in DEFAULTS} | PRESERVED) - UNSERIALIZABLE
    NATIVE = set(logging.LogRecord(*[v for _, v in DEFAULTS]).__dict__.keys())

    @staticmethod
    def make_record(data):
        fields = json.loads(data)
        args = []
        for k, v in JSONFormatter.DEFAULTS:
            val = fields.get(k, v)

            if k == 'args':
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
        if rec.get('exc_info', None):
            fields['exc_text'] = self.formatException(rec['exc_info'])

        def serialize_obj(obj):
            if hasattr(obj, 'getvalue'):
                return obj.getvalue()
            elif hasattr(obj, '__dict__'):
                return obj.__dict__
            return None

        return json.dumps(fields, default=serialize_obj, skipkeys=True)

    def _isvalid(self, field):
        return field in self.FIELDS or field not in self.NATIVE


class LogstashFormatter(logging.Formatter):
    """Format log messages for logstash."""

    converter = time.gmtime

    def __init__(self, fmt=None, datefmt='%Y-%m-%dT%H:%M:%SZ',
                 log_type='scap'):
        """
        :param fmt: Message format string (not used)
        :param datefmt: Time format string
        :param type: Logstash event type
        """
        super(LogstashFormatter, self).__init__(fmt, datefmt)
        self.type = log_type
        self.host = socket.gethostname()
        self.script = sys.argv[0]
        self.user = utils.get_real_username()

    def format(self, record):
        """Format a record as a logstash v1 JSON string."""
        fields = record.__dict__.copy()

        # Rename fields
        fields['channel'] = fields.pop('name', 'unnamed')
        fields['@timestamp'] = self.formatTime(record, self.datefmt)

        # Ensure message is populated
        if 'message' not in fields:
            try:
                if 'args' in fields and fields['args']:
                    fields['message'] = fields['msg'] % fields['args']
                else:
                    fields['message'] = fields['msg']
            except TypeError as typee:
                # This sometimes happens if the fields['msg'] has a
                # '%<something>' in it some place.
                #
                # Re-raising and dumping the message and fields seems like it
                # may be more helpful in tracking down the root cause of an
                # error than the output, "not enough arguments for format
                # string"
                raise TypeError(
                    'error: ({}); '
                    'format string: ({}); '
                    'arguments: ({})'.format(
                        str(typee),
                        fields['msg'],
                        fields['args']))

        # Format exception
        if 'exc_info' in fields and fields['exc_info']:
            fields['exception'] = self.formatException(fields['exc_info'])

        # Remove fields
        remove_fields = [
            'args',
            'asctime',
            'created',
            'exc_info',
            'exc_text',
            'levelno',
            'msecs',
            'msg',
            'processName',
            'relativeCreated',
            'thread',
            'threadName',
        ]
        for field in remove_fields:
            fields.pop(field, None)

        logstash_record = {
            '@version': 1,
            'type': self.type,
            'host': self.host,
            'script': self.script,
            'user': self.user,
        }
        logstash_record.update(fields)

        return json.dumps(logstash_record, default=str)

    def formatException(self, exc_info):
        """Format the given exception as a dict."""
        ex_type, ex_value, ex_traceback = exc_info
        return {
            'class': ex_type.__name__,
            'message': '%s' % ex_value,
            'stacktrace': [{
                'file': fname,
                'line': line,
                'function': func,
                'text': text,
            } for fname, line, func, text in traceback.extract_tb(
                ex_traceback)]
        }


def reporter(message, fancy=False):
    """
    Instantiate progress reporter

    :message: - string that will be displayed to user
    :fancy: - boolean that determines the progress bar type
    """
    if fancy:
        return FancyProgressReporter(message)

    return ProgressReporter(message)


class ProgressReporter(object):
    """
    Track and display progress of a process.

    Report on the status of a multi-step process by displaying the completion
    percentage and succes, failure and remaining task counts on a single
    output line.
    """

    def __init__(self, name, expect=0, fd=sys.stderr):
        """
        :param name: Name of command being monitored
        :param expect: Number of results to expect
        :param fd: File handle to write status messages to
        """
        self._name = name
        self._expect = expect
        self._done = 0
        self._ok = 0
        self._failed = 0
        self._fd = fd

    @property
    def ok(self):
        return self._ok

    @property
    def failed(self):
        return self._failed

    @property
    def remaining(self):
        return self._expect - self._done

    @property
    def done(self):
        return self._done

    @property
    def percent_complete(self):
        return math.floor(100.0 * (float(self._done) / max(self._expect, 1)))

    def expect(self, count):
        """Set expected result count."""
        self._expect = count

    def start(self):
        """Start tracking progress."""
        self._progress()

    def finish(self):
        """Finish tracking progress."""
        self._progress()
        self._fd.write('\n')

    def add_success(self):
        """Record a sucessful task completion."""
        self._done += 1
        self._ok += 1
        self._progress()

    def add_failure(self):
        """Record a failed task completion."""
        self._done += 1
        self._failed += 1
        self._progress()

    def _progress(self):
        if sys.stdout.isatty():
            fmt = '%-80s\r'
        else:
            fmt = '%-80s\n'
        self._fd.write(fmt % self._output())

    def _output(self):
        return '%s: %3.0f%% (ok: %d; fail: %d; left: %d)' % (
            self._name, self.percent_complete,
            self.ok, self.failed, self.remaining)


class FancyProgressReporter(ProgressReporter):

    def __init__(self, name='', expect=0, fd=sys.stderr):
        TERM.scroll_region(0, TERM.height - 3)
        TERM.scroll_forward(1)
        TERM.register_cleanup_callback(self.cleanup)
        super(FancyProgressReporter, self).__init__(name, expect=expect, fd=fd)

    def finish(self):
        """Finish tracking progress."""
        self._progress()

        message = "Finished: %s (%s failed) " % \
            (self._name, self._failed)
        width = min((TERM.width, 80)) - len(message)
        bars = width - 4
        prog_bar = '=' * bars
        self.cleanup()
        TERM.fg(7).write(message) \
            .fg(4).write(prog_bar).nl()

    def cleanup(self, term=TERM):
        height = term.height
        term.save()
        term.move(height - 2, 0).clear_eol() \
            .move(height - 1, 0).clear_eol() \
            .move(height, 0).clear_eol()
        term.scroll_region(0, height)
        term.reset_colors()
        term.restore()

    def _progress(self):
        width = TERM.width
        bottom = TERM.height
        label_width = len(self._name) + 12
        scale = int(width / 2)
        scale = min(width - label_width, scale)
        scale = max(scale, 15)
        partial_bar = (' ', '▎', '▎', '▍', '▌', '▋', '▊', '▉', '▉', '█')
        pct = float(self.percent_complete) / 100
        progress = scale * pct
        filled_bars = int(progress)
        remain = 0
        if filled_bars > 0 and progress > filled_bars:
            remain = int((progress % filled_bars) * 10)

        prog_bar = '█' * int(filled_bars)
        prog_bar = prog_bar + partial_bar[remain]

        TERM.save() \
            .move(bottom - 1, 0) \
            .fg(15).write("| ok: ") \
            .fg(2).write(str(self.ok)) \
            .fg(15).write(" | fail: ") \
            .fg(1).write(str(self.failed)) \
            .fg(15).write(" | remain: ") \
            .fg(7).write(str(self.remaining), ' | ') \
            .clear_eol()

        TERM.move(bottom - 2, 0) \
            .fg(15).write('| ') \
            .fg(7).write(self._name) \
            .fg(15).write(" | ") \
            .write(self.percent_complete, '% ') \
            .fg(4).write(prog_bar) \
            .clear_eol()

        TERM.restore().flush()

    def _output(self):
        return '%s: %3.0f%% (ok: %d; fail: %d; left: %d)' % (
            self._name, self.percent_complete,
            self.ok, self.failed, self.remaining)


class MuteReporter(ProgressReporter):
    """A report that declines to report anything."""

    def __init__(self, name='', expect=0, fd=sys.stderr):
        super(MuteReporter, self).__init__(name)

    def _progress(self):
        pass

    def finish(self):
        pass


class DeployLogFormatter(JSONFormatter):
    """Ensure that all `deploy.log` records contain a host attribute."""

    def format(self, record):
        if not hasattr(record, 'host'):
            record.host = socket.gethostname()

        return super(DeployLogFormatter, self).format(record)


class DeployLogHandler(logging.FileHandler):
    """Handler for `scap/deploy.log`."""

    def __init__(self, log_file):
        super(DeployLogHandler, self).__init__(log_file)
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

    OPERATORS = {'=', '==', '~', '>', '>=', '<', '<='}
    COMPARISONS = {'>': 'gt', '>=': 'ge', '<': 'lt', '<=': 'le'}
    LOG_LEVELS = ['CRITICAL', 'ERROR', 'WARNING', 'INFO', 'DEBUG']

    @staticmethod
    def loads(expression, invert=True):
        """
        Construct a `Filter` from the given free-form expression.

        See :class:`Filter` for examples.
        """

        criteria = []

        for lhs, op, rhs in Filter.parse(expression):
            if lhs == 'levelno' and rhs in Filter.LOG_LEVELS:
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
            elif op == '~':
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
            lhs, op, rhs = parts[i:i + 3]

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

        if hasattr(criteria, 'items'):
            criteria = criteria.items()

        # Normalize all globs into regexs into lambdas
        for attr, criterion in criteria:
            if isinstance(criterion, str):
                criterion = re.compile(fnmatch.translate(criterion))

            if not hasattr(criterion, '__call__'):
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

        for attr, _ in self.criteria:
            if attr == attribute:
                return True

        return False


class Stats(object):
    """
    A simple StatsD metric client.

    It can log measurements and counts to a remote StatsD host.
    See <https://github.com/etsy/statsd/wiki/Protocol> for details.
    """

    @utils.log_context('stats')
    def __init__(self, host, port, logger=None):
        self.logger = logger
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.address = (host, port)

    def timing(self, name, milliseconds):
        """Report a timing measurement in milliseconds."""
        metric = '%s:%s|ms' % (name, int(round(milliseconds)))
        self._send_metric(metric)

    def increment(self, name, value=1):
        """Increment a measurement."""
        metric = '%s:%s|c' % (name, value)
        self._send_metric(metric)

    def _send_metric(self, metric):
        try:
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
    >>> with Timer('example', s):
    ...     time.sleep(0.1)

    Sub-interval times can also be recorded using the :meth:`mark` method.

    >>> with Timer('file copy') as t:
    ...     time.sleep(0.1)
    ...     x = t.mark('copy phase 1')
    ...     time.sleep(0.1)
    ...     y = t.mark('copy phase 2')
    """

    @utils.log_context('timer')
    def __init__(self, label, stats=None, logger=None):
        """
        :param label: Label for block (e.g. 'scap' or 'rsync')
        :type label: str
        :param stats: StatsD client to record block invocation and duration
        :type stats: scap.log.Stats
        """
        self.label = label
        self.stats = stats
        self.logger = logger
        self.mark_start = None
        self.start = None

    def mark(self, label):
        """
        Log the interval elapsed since the last mark call.

        :param label: Label for block (e.g. 'scap' or 'rsync')
        :type label: str
        """
        now = time.time()
        elapsed = now - self.mark_start
        self._record_elapsed(label, elapsed)
        self.mark_start = now
        return elapsed

    def __enter__(self):
        """
        Enter the runtime context.

        :returns: self
        """
        self.start = time.time()
        self.mark_start = self.start
        self.logger.info('Started %s' % self.label)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Exit the runtime context."""
        self._record_elapsed(self.label, time.time() - self.start)

    def _record_elapsed(self, label, elapsed):
        """
        Log the elapsed duration.

        :param label: Label for elapsed time
        :type label: str
        :param elapsed: Elapsed duration
        :type elapsed: float
        """
        self.logger.info('Finished %s (duration: %s)',
                         label, utils.human_duration(elapsed))
        if self.stats:
            label = re.sub(r'\W', '_', label.lower())
            self.stats.timing('scap.%s' % label, elapsed * 1000)


class Udp2LogHandler(logging.handlers.DatagramHandler):
    """Log handler for udp2log."""

    def __init__(self, host, port, prefix='scap'):
        """
        :param host: Hostname or ip address
        :param port: Port
        :param prefix: Line prefix (udp2log destination)
        """
        super(Udp2LogHandler, self).__init__(host, port)
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
            text = re.sub(r'^', self.prefix + ' ', text, flags=re.MULTILINE)
        if len(text) > 65506:
            text = text[:65506]
        if text[-1] != '\n':
            text = text + '\n'
        return text


def setup_loggers(cfg, console_level=logging.INFO, handlers=None):
    """
    Setup the logging system.

    * Configure the root logger to use :class:`AnsiColorFormatter`
    * Optionally add a :class:`Udp2LogHandler` to send logs to a udp2log server
    * Optional add a :class:`IRCSocketHandler` for the `scap.announce` log
      channel to send messages to a tcpircbot server

    :param cfg: Dict of global configuration values
    :param console_level: Logging level for the local console appender
    :param handlers: Additional handlers
    """

    # The INFO level for scap.sh is pretty verbose
    if console_level == logging.DEBUG:
        logging.getLogger('scap.sh').setLevel(logging.INFO)

    # Set logger levels
    logging.root.setLevel(logging.DEBUG)
    logging.root.handlers[0].setLevel(console_level)

    # Filter target output from the main handler
    logging.root.handlers[0].addFilter(Filter({'name': 'target.*'}))

    if cfg['log_json']:
        logging.root.handlers[0].setFormatter(JSONFormatter())
    else:
        # Colorize log messages sent to stderr
        logging.root.handlers[0].setFormatter(DiffLogFormatter(
            '%(asctime)s %(message)s', '%H:%M:%S'))

    if cfg['udp2log_host']:
        # Send a copy of all logs to the udp2log relay
        udp_handler = Udp2LogHandler(
            cfg['udp2log_host'], int(cfg['udp2log_port']))
        udp_handler.setLevel(logging.DEBUG)
        udp_handler.setFormatter(LogstashFormatter())
        logging.root.addHandler(udp_handler)

    if cfg['tcpircbot_host']:
        # Send 'scap.announce' messages to irc relay
        irc_logger = logging.getLogger('scap.announce')
        irc_logger.addHandler(IRCSocketHandler(
            cfg['tcpircbot_host'], int(cfg['tcpircbot_port'])))

    if handlers is not None:
        for handler in handlers:
            logging.root.addHandler(handler)
