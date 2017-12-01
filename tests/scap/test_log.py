#!/usr/bin/env python2

from __future__ import absolute_import

import json
import logging
import re
from StringIO import StringIO
import sys
from textwrap import dedent
import unittest

import scap.log as log


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

    def test_filter_loads(self):
        expression = r"""
            foo == 'bar.*' baz ~ "qux"
            blah = splat.*
        """

        logfilter = log.Filter.loads(expression)

        self.assertIsInstance(logfilter, log.Filter)

    def test_filter_loads_filters_correctly(self):
        expression = r"""
            name == foo.*.baz
            msg ~ 'log'
        """

        logfilter = log.Filter.loads(expression, invert=False)

        record = ['foo.bar.baz', logging.DEBUG, '', 0, 'bologna', [], None]
        record = logging.LogRecord(*record)
        self.assertTrue(logfilter.filter(record))

        record = ['bar.baz', logging.DEBUG, '', 0, 'bologna', [], None]
        record = logging.LogRecord(*record)
        self.assertFalse(logfilter.filter(record))

    def test_filter_loads_supports_numeric_comparisons(self):
        expression = r"""
            levelno > 10
        """

        logfilter = log.Filter.loads(expression, invert=False)

        record = ['', 10, '', 0, 'bologna', [], None]
        record = logging.LogRecord(*record)
        self.assertFalse(logfilter.filter(record))

        record = ['', 20, '', 0, '', [], None]
        record = logging.LogRecord(*record)
        self.assertTrue(logfilter.filter(record))

    def test_filter_parse(self):
        expression = r"""
            foo == 'bar.*' baz ~ "qux"
            blah = splat.*
        """

        elements = list(log.Filter.parse(expression))

        self.assertEqual(elements, [
            ('foo', '==', 'bar.*'),
            ('baz', '~', 'qux'),
            ('blah', '=', 'splat.*')])

    def test_filter_filters_matching(self):
        logfilter = log.Filter({'name': 'A.B.*'})

        self.root_handler.addFilter(logfilter)

        self.b_logger.info('please log this')
        self.c_logger.info('do not log this')

        self.assertEqual("please log this\n", self.stream.getvalue())

    def test_filter_filters_matching_regex(self):
        logfilter = log.Filter({'name': re.compile(r'^A\.B\.')})

        self.root_handler.addFilter(logfilter)

        self.b_logger.info('please log this')
        self.c_logger.info('do not log this')

        self.assertEqual("please log this\n", self.stream.getvalue())

    def test_filter_filters_matching_lambda(self):
        logfilter = log.Filter({'levelno': lambda lvl: lvl < logging.WARNING})

        self.root_handler.addFilter(logfilter)

        self.b_logger.warning('please log this')
        self.c_logger.info('do not log this')

        self.assertEqual("please log this\n", self.stream.getvalue())

    def test_filter_can_invert_behavior(self):
        logfilter = log.Filter({'name': 'A.B.*'}, invert=False)

        self.root_handler.addFilter(logfilter)

        self.b_logger.info('please log this')
        self.c_logger.info('do not log this')

        self.assertEqual("do not log this\n", self.stream.getvalue())


class JSONFormatterTest(unittest.TestCase):

    def test_make_record(self):
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

        self.assertIsInstance(record, logging.LogRecord)

        self.assertEqual(record.name, 'foo')
        self.assertEqual(record.args, ())
        self.assertEqual(record.filename, 'foo_file')
        self.assertEqual(record.levelno, 10)
        self.assertEqual(record.lineno, 123)
        self.assertEqual(record.msg, 'foo message')
        self.assertEqual(record.exc_text, 'foo exception')
        self.assertEqual(record.foo_extra, 'bar')
        self.assertEqual(record.created, 1444873813.116016)
        self.assertEqual(record.msecs, 116.01591110229492)
        self.assertEqual(record.relativeCreated, 38272.172927856445)

    def test_format_includes_all_serializable_logrecord_fields(self):
        formatter = log.JSONFormatter()

        args = ['foo', logging.DEBUG, 'foo_file', 123, 'foo message', [], None]
        record = logging.LogRecord(*args)
        record.exc_text = 'foo exception text'
        line = formatter.format(record)

        # we can't reliably test an unordered JSON string so load it back in
        parsed = json.loads(line)

        self.assertIs(type(parsed), dict)
        self.assertIn('name', parsed)
        self.assertIn('levelno', parsed)
        self.assertIn('filename', parsed)
        self.assertIn('lineno', parsed)
        self.assertIn('msg', parsed)
        self.assertIn('funcName', parsed)
        self.assertIn('created', parsed)
        self.assertIn('msecs', parsed)
        self.assertIn('relativeCreated', parsed)
        self.assertIn('exc_text', parsed)

        self.assertEqual(parsed['name'], 'foo')
        self.assertEqual(parsed['levelno'], logging.DEBUG)
        self.assertEqual(parsed['filename'], 'foo_file')
        self.assertEqual(parsed['lineno'], 123)
        self.assertEqual(parsed['msg'], 'foo message')
        self.assertEqual(parsed['funcName'], None)
        self.assertEqual(parsed['created'], record.created)
        self.assertEqual(parsed['msecs'], record.msecs)
        self.assertEqual(parsed['relativeCreated'], record.relativeCreated)
        self.assertEqual(parsed['exc_text'], record.exc_text)

    def test_format_includes_extra_fields(self):
        formatter = log.JSONFormatter()

        args = ['foo', logging.DEBUG, 'foo_file', 123, 'foo message', [], None]
        record = logging.LogRecord(*args)
        record.__dict__['foo_extra'] = 'bar'
        line = formatter.format(record)

        # we can't reliably test an unordered JSON string so load it back in
        parsed = json.loads(line)

        self.assertIs(type(parsed), dict)
        self.assertIn('foo_extra', parsed)
        self.assertEqual(parsed['foo_extra'], 'bar')

    def test_format_includes_exceptions_as_text(self):
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

        self.assertIs(type(parsed), dict)
        self.assertNotIn('exc_info', parsed)
        self.assertIn('exc_text', parsed)
        self.assertIn('RuntimeError: fail fail fail', parsed['exc_text'])


class LogstashFormatterTest(unittest.TestCase):

    def test_helpful_typeerror(self):
        """
        Ensure TypeError message is helpful.

        This test ensures that even if the LogstashFormatter fails to format a
        message, we still see the information it was trying to format.
        """
        args = ['10']
        msg = 'I think this \%s is escaped! Favorite Number: %d'
        record = logging.makeLogRecord({'msg': msg, 'args': args})
        formatter = log.LogstashFormatter()
        with self.assertRaises(TypeError) as e:
            formatter.format(record)

        self.assertIn(args[0], str(e.exception))
        self.assertIn(msg, str(e.exception))


class ProgressReporterTest(unittest.TestCase):

    def test_progress(self):
        reporter = log.ProgressReporter(name='TestProgress', expect=100)
        reporter.start()
        for i in range(0, 100):
            reporter.add_success()
        reporter.finish()

        self.assertEqual(reporter.percent_complete, 100)
        self.assertEqual(reporter.ok + reporter.failed, reporter.done)


if __name__ == '__main__':
    unittest.main()
