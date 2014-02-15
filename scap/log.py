# -*- coding: utf-8 -*-
"""
    scap.log
    ~~~~~~~~
    Helpers for routing and formatting log data.

"""
import logging
import os
import socket


# Format string for log messages. Interpolates LogRecord attributes.
# See <http://docs.python.org/2/library/logging.html#logrecord-attributes>
# for attribute names you can include here.
LOG_FORMAT = '%(asctime)s %(levelname)s - %(message)s'

# A tuple of (host, port) representing the address of a tcpircbot instance to
# use for logging messages. tcpircbot is a simple script that listens for
# line-oriented data on a TCP socket and outputs it to IRC.
# See <https://doc.wikimedia.org/puppet/classes/tcpircbot.html>.
IRC_LOG_ENDPOINT = ('neon.wikimedia.org', 9200)


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
        format=LOG_FORMAT,
        datefmt='%H:%M:%S')

    logger = logging.getLogger('scap')
    logger.addHandler(IRCSocketHandler(*IRC_LOG_ENDPOINT))
