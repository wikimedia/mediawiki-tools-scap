#!/usr/bin/env python2

from __future__ import absolute_import

import json
import logging
import re
from StringIO import StringIO
import sys
from textwrap import dedent
import unittest

from scap import log


class FilterTest(unittest.TestCase):

    def setUp(self):
        self.root_logger = logging.getLogger('A')
        self.root_logger.setLevel(logging.INFO)

        for handler in self.root_logger.handlers:
            self.root_logger.removeHandler(handler)

        self.stream = StringIO()
        self.root_handler = logging.StreamHandler(self.stream)
        self.root_logger.addHandler(self.root_handler)

        self.b_logger = self.root_logger.getChild('B')
        self.c_logger = self.b_logger.getChild('C')

    def test_filter_filters_matching(self):
        logfilter = log.Filter({'name': 'A.B.*'})

        self.root_handler.addFilter(logfilter)

        self.b_logger.info('please log this')
        self.c_logger.info('do not log this')

        assert self.stream.getvalue() == "please log this\n"

    def test_filter_filters_matching_regex(self):
        logfilter = log.Filter({'name': re.compile(r'^A\.B\.')})

        self.root_handler.addFilter(logfilter)

        self.b_logger.info('please log this')
        self.c_logger.info('do not log this')

        assert self.stream.getvalue() == "please log this\n"

    def test_filter_filters_matching_lambda(self):
        logfilter = log.Filter({'levelno': lambda lvl: lvl < logging.WARNING})

        self.root_handler.addFilter(logfilter)

        self.b_logger.warning('please log this')
        self.c_logger.info('do not log this')

        assert self.stream.getvalue() == "please log this\n"

    def test_filter_can_invert_behavior(self):
        logfilter = log.Filter({'name': 'A.B.*'}, invert=False)

        self.root_handler.addFilter(logfilter)

        self.b_logger.info('please log this')
        self.c_logger.info('do not log this')

        assert self.stream.getvalue() == "do not log this\n"

    def test_helpful_typeerror(self):
        """
        Ensure TypeError message is helpful.

        This test ensures that even if the LogstashFormatter fails to format a
        message, we still see the information it was trying to format.
        """
        args = ['10']
        msg = r'I think this \%s is escaped! Favorite Number: %d'
        record = logging.makeLogRecord({'msg': msg, 'args': args})
        formatter = log.LogstashFormatter()
        with self.assertRaises(TypeError) as e:
            formatter.format(record)

        self.assertIn(args[0], str(e.exception))
        self.assertIn(msg, str(e.exception))


def test_filter_loads():
    expression = r"""
        foo == 'bar.*' baz ~ "qux"
        blah = splat.*
    """

    logfilter = log.Filter.loads(expression)

    assert isinstance(logfilter, log.Filter)


def test_filter_loads_filters_correctly():
    expression = r"""
        name == foo.*.baz
        msg ~ 'log'
    """

    logfilter = log.Filter.loads(expression, invert=False)

    record = ['foo.bar.baz', logging.DEBUG, '', 0, 'bologna', [], None]
    record = logging.LogRecord(*record)
    assert logfilter.filter(record)

    record = ['bar.baz', logging.DEBUG, '', 0, 'bologna', [], None]
    record = logging.LogRecord(*record)
    assert not logfilter.filter(record)


def test_filter_loads_supports_numeric_comparisons():
    expression = r"""
        levelno > 10
    """

    logfilter = log.Filter.loads(expression, invert=False)

    record = ['', 10, '', 0, 'bologna', [], None]
    record = logging.LogRecord(*record)
    assert not logfilter.filter(record)

    record = ['', 20, '', 0, '', [], None]
    record = logging.LogRecord(*record)
    assert logfilter.filter(record)


def test_filter_parse():
    expression = r"""
        foo == 'bar.*' baz ~ "qux"
        blah = splat.*
    """

    elements = list(log.Filter.parse(expression))

    assert elements == [
        ('foo', '==', 'bar.*'),
        ('baz', '~', 'qux'),
        ('blah', '=', 'splat.*')]


def test_make_record():
    data = dedent("""
        {
            "name": "foo",
            "filename": "foo_file",
            "levelno": 10,
            "lineno": 123,
            "msg": "foo message",
            "exc_text": "foo exception",
            "foo_extra": "bar",
            "funcName": null,
            "created": 1444873813.116016,
            "msecs": 116.01591110229492,
            "relativeCreated": 38272.172927856445
        }
    """)

    record = log.JSONFormatter.make_record(data)

    assert isinstance(record, logging.LogRecord)

    assert record.name == 'foo'
    assert record.args == ()
    assert record.filename == 'foo_file'
    assert record.levelno == 10
    assert record.lineno == 123
    assert record.msg == 'foo message'
    assert record.exc_text == 'foo exception'
    assert record.foo_extra == 'bar'
    assert record.created == 1444873813.116016
    assert record.msecs == 116.01591110229492
    assert record.relativeCreated == 38272.172927856445


def test_format_includes_all_serializable_logrecord_fields():
    formatter = log.JSONFormatter()

    args = ['foo', logging.DEBUG, 'foo_file', 123, 'foo message', [], None]
    record = logging.LogRecord(*args)
    record.exc_text = 'foo exception text'
    line = formatter.format(record)

    # we can't reliably test an unordered JSON string so load it back in
    parsed = json.loads(line)

    assert isinstance(parsed, dict)
    assert 'name' in parsed
    assert 'levelno' in parsed
    assert 'filename' in parsed
    assert 'lineno' in parsed
    assert 'msg' in parsed
    assert 'funcName' in parsed
    assert 'created' in parsed
    assert 'msecs' in parsed
    assert 'relativeCreated' in parsed
    assert 'exc_text' in parsed

    assert parsed['name'] == 'foo'
    assert parsed['levelno'] == logging.DEBUG
    assert parsed['filename'] == 'foo_file'
    assert parsed['lineno'] == 123
    assert parsed['msg'] == 'foo message'
    assert parsed['funcName'] is None
    assert parsed['created'] == record.created
    assert parsed['msecs'] == record.msecs
    assert parsed['relativeCreated'] == record.relativeCreated
    assert parsed['exc_text'] == record.exc_text


def test_format_includes_extra_fields():
    formatter = log.JSONFormatter()

    args = ['foo', logging.DEBUG, 'foo_file', 123, 'foo message', [], None]
    record = logging.LogRecord(*args)
    record.__dict__['foo_extra'] = 'bar'
    line = formatter.format(record)

    # we can't reliably test an unordered JSON string so load it back in
    parsed = json.loads(line)

    assert isinstance(parsed, dict)
    assert 'foo_extra' in parsed
    assert parsed['foo_extra'] == 'bar'


def test_format_includes_exceptions_as_text():
    formatter = log.JSONFormatter()

    args = ['foo', logging.DEBUG, 'foo_file', 123, 'foo message', [], None]
    record = logging.LogRecord(*args)

    try:
        raise RuntimeError('fail fail fail')
    except RuntimeError:
        record.exc_info = sys.exc_info()

    line = formatter.format(record)

    # we can't reliably test an unordered JSON string so load it back in
    parsed = json.loads(line)

    assert isinstance(parsed, dict)
    assert 'exc_info' not in parsed
    assert 'exc_text' in parsed
    assert 'RuntimeError: fail fail fail' in parsed['exc_text']


def test_progress():
    reporter = log.ProgressReporter(name='TestProgress', expect=100)
    reporter.start()
    for _ in range(0, 100):
        reporter.add_success()
    reporter.finish()

    assert reporter.percent_complete == 100
    assert (reporter.ok + reporter.failed) == reporter.done
