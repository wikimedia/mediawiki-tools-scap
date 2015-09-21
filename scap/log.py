# -*- coding: utf-8 -*-
"""
    scap.log
    ~~~~~~~~
    Helpers for routing and formatting log data.

"""
import json
import logging
import math
import logging.handlers
import re
import socket
import sys
import time
import traceback

from . import utils

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
        super(self.__class__, self).__init__(fmt, datefmt)
        if colors:
            self.colors.extend(colors)

    def format(self, record):
        msg = super(self.__class__, self).format(record)
        color = self.colors.get(record.levelname, '0')
        return '\x1b[%sm%s\x1b[0m' % (color, msg)


class IRCSocketHandler(logging.Handler):
    """Log handler for logmsgbot on #wikimedia-operation.

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
    """Serialize logging output as JSON.

    Can be used to maintain logged event structure between the deployment host
    and remote targets.
    """

    FIELDS = ['created', 'name', 'levelno', 'pathname', 'lineno']
    encoder = None

    def format(self, record):
        rec = record.__dict__
        fields = {field: rec[field] for field in rec if field in self.FIELDS}
        fields['message'] = super(self.__class__, self).format(record)

        return self._encode(fields)

    def _encode(self, fields):
        if self.encoder is None:
            self.encoder = json.JSONEncoder()
        return self.encoder.encode(fields)


class LogstashFormatter(logging.Formatter):
    """Format log messages for logstash."""

    converter = time.gmtime

    def __init__(self, fmt=None, datefmt='%Y-%m-%dT%H:%M:%SZ', type='scap'):
        """
        :param fmt: Message format string (not used)
        :param datefmt: Time format string
        :param type: Logstash event type
        """
        super(self.__class__, self).__init__(fmt, datefmt)
        self.type = type
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
            fields['message'] = fields['msg'] % fields['args']

        # Format exception
        if 'exc_info' in fields and fields['exc_info']:
            fields['exception'] = self.formatException(fields['exc_info'])

        # Remove fields
        for field in (
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
        ):
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
        """Formats the given exception as a dict."""
        ex_type, ex_value, ex_traceback = exc_info
        return {
            'class': ex_type.__name__,
            'message': '%s' % ex_value,
            'stacktrace': [{
                'file': fname,
                'line': line,
                'function': func,
                'text': text,
            } for fname, line, func, text in
                traceback.extract_tb(ex_traceback)]
        }


class Message:
    """Structured log message."""

    loglevel = None
    meta = {}

    def __init__(self, **meta):
        self.loglevel = meta.get('loglevel', logging.INFO)
        self.meta = meta

    def __str__(self):
        return self.meta.get('message', '')


class ProgressReporter(object):
    """Track and display progress of a process.

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


class MuteReporter(ProgressReporter):
    """A report that declines to report anything."""
    def __init__(self, name='', expect=0, fd=sys.stderr):
        super(self.__class__, self).__init__(name)

    def _progress(self):
        pass

    def finish(self):
        pass


class Stats(object):
    """A simple StatsD metric client that can log measurements and counts to
    a remote StatsD host.

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
    """Context manager to track and record the time taken to execute a block.

    Elapsed time will be recorded to a logger and optionally a StatsD server.

    >>> with Timer('example'):
    ...     time.sleep(0.1)

    >>> s = Stats('127.0.0.1', 2003)
    >>> with Timer('example', s):
    ...     time.sleep(0.1)

    Sub-interval times can also be recorded using the :meth:`mark` method.

    >>> with Timer('file copy') as t:
    ...     time.sleep(0.1)
    ...     t.mark('copy phase 1')
    ...     time.sleep(0.1)
    ...     t.mark('copy phase 2')
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

    def mark(self, label):
        """
        Log the interval elapsed since the last mark call.

        :param label: Label for block (e.g. 'scap' or 'rsync')
        :type label: str
        """
        now = time.time()
        self._record_elapsed(label, now - self.mark_start)
        self.mark_start = now

    def __enter__(self):
        """Enter the runtime context.
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
        """Log the elapsed duration.

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
        super(self.__class__, self).__init__(host, port)
        self.prefix = prefix

    def makePickle(self, record):
        """Format record as a udp2log packet.

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


def setup_loggers(cfg, console_level=logging.INFO):
    """Setup the logging system.

    * Configure the root logger to use :class:`AnsiColorFormatter`
    * Optionally add a :class:`Udp2LogHandler` to send logs to a udp2log server
    * Optional add a :class:`IRCSocketHandler` for the `scap.announce` log
      channel to send messages to a tcpircbot server

    :param cfg: Dict of global configuration values
    :param console_level: Logging level for the local console appender
    """
    # Set logger levels
    logging.root.setLevel(logging.DEBUG)
    logging.root.handlers[0].setLevel(console_level)

    if cfg['log_json']:
        logging.root.handlers[0].setFormatter(JSONFormatter())
    else:
        # Colorize log messages sent to stderr
        logging.root.handlers[0].setFormatter(AnsiColorFormatter(
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
