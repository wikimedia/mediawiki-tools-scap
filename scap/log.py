# -*- coding: utf-8 -*-
"""
    scap.log
    ~~~~~~~~
    Helpers for routing and formatting log data.

"""
import logging
import logging.handlers
import os
import re
import socket
import sys
import time

from . import utils

# Format string for log messages. Interpolates LogRecord attributes.
# See <http://docs.python.org/2/library/logging.html#logrecord-attributes>
# for attribute names you can include here.
CONSOLE_LOG_FORMAT = '%(asctime)s %(levelname)-8s - %(message)s'

FILE_LOG_FORMAT = '%(asctime)s %(levelname)s %(name)s - %(message)s'

# A tuple of (host, port) representing the address of a tcpircbot instance to
# use for logging messages. tcpircbot is a simple script that listens for
# line-oriented data on a TCP socket and outputs it to IRC.
# See <https://doc.wikimedia.org/puppet/classes/tcpircbot.html>.
IRC_LOG_ENDPOINT = ('neon.wikimedia.org', 9200)

# A tuple of (host, port) representing the address of a udp2log instance to
# use for logging messages. udp2log is a log aggregation service that listens
# for UDP datagrams and writes their contents to files and/or relays the
# messages to other services.
# See <https://wikitech.wikimedia.org/wiki/Udp2log>
UDP2LOG_LOG_ENDPOINT = ('10.64.0.21', 8420)


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
        message = '!log %s %s' % (os.getlogin(), record.getMessage())
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.timeout)
            sock.connect(self.addr)
            sock.sendall(message.encode('utf-8'))
            sock.close()
        except (socket.timeout, socket.error, socket.gaierror):
            self.handleError(record)


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


class Stats(object):
    """A simple StatsD metric client that can log measurements and counts to
    a remote StatsD host.

    See <https://github.com/etsy/statsd/wiki/Protocol> for details.
    """

    def __init__(self, host, port):
        self.logger = logging.getLogger('stats')
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
            self.socket.sendto(metric.encode('utf-8'), self.address)
        except Exception:
            self.logger.exception('Failed to send metric "%s"', metric)


def setup_loggers():
    """Setup the root logger and a special scap logger.

    The 'scap' logger uses :class:`IRCSocketHandler` to send log messages of
    level info or higher to logmsgbot.
    """
    logging.basicConfig(
        level=logging.DEBUG,
        format=CONSOLE_LOG_FORMAT,
        datefmt='%H:%M:%S',
        stream=sys.stdout)

    # Colorize log messages to stdout
    logging.root.handlers[0].setFormatter(AnsiColorFormatter(
        CONSOLE_LOG_FORMAT, '%H:%M:%S'))

    # Send a copy of all logs to the udp2log relay
    udp_handler = Udp2LogHandler(*UDP2LOG_LOG_ENDPOINT)
    udp_handler.setLevel(logging.DEBUG)
    udp_handler.setFormatter(logging.Formatter(
        FILE_LOG_FORMAT, '%Y-%m-%dT%H:%M:%S.000-%Z'))
    logging.root.addHandler(udp_handler)

    # Send 'scap' messages to irc relay
    irc_logger = logging.getLogger('scap')
    irc_logger.addHandler(IRCSocketHandler(*IRC_LOG_ENDPOINT))


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

    def __init__(self, label, stats=None):
        """
        :param label: Label for block (e.g. 'scap' or 'rsync')
        :type label: str
        :param stats: StatsD client to record block invocation and duration
        :type stats: scap.log.Stats
        """
        self.label = label
        self.stats = stats
        self.logger = logging.getLogger('timer')

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
        self.logger.debug('Started %s' % self.label)
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
            self.stats.increment('scap.%s' % label)
            self.stats.timing('scap.%s' % label, elapsed * 1000)
