# -*- coding: utf-8 -*-
#
# Copyright (c) 2014 Wikimedia Foundation and contributors
"""
Logging support
"""
import logging
import os
import socket


LOG_FORMAT = '%(asctime)s %(levelname)s - %(message)s'
IRC_LOG_ENDPOINT = ('neon.wikimedia.org', 9200)


class Stats(object):
    """A simple StatsD metric client."""

    def __init__(self, host, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.address = (host, port)

    def timing(self, name, milliseconds):
        """Report a timing measurement in milliseconds."""
        metric = '%s:%s|ms' % (name, int(round(milliseconds)))
        self.socket.sendto(metric.encode('utf-8'), self.address)

    def increment(self, name, value=1):
        """Increment a measurement."""
        metric = '%s:%s|c' % (name, value)
        self.socket.sendto(metric.encode('utf-8'), self.address)


class IRCSocketHandler(logging.Handler):
    """Log handler for logmsgbot on #wikimedia-operation."""

    def __init__(self, host, port, timeout=1.0):
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


def setup_loggers():
    """Setup the root logger and a special scap logger"""
    logging.basicConfig(
        level=logging.DEBUG,
        format=LOG_FORMAT,
        datefmt='%H:%M:%S')

    logger = logging.getLogger('scap')
    logger.addHandler(IRCSocketHandler(*IRC_LOG_ENDPOINT))
