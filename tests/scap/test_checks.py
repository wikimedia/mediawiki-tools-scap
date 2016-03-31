#!/usr/bin/env python2

import logging
import select
from StringIO import StringIO
from textwrap import dedent
import unittest

from scap import checks


class ChecksConfigTest(unittest.TestCase):
    def test_load_valid_config(self):
        chks = checks.load(dedent("""
            ---
            checks:
              foo:
                type: command
                command: /bin/true
                stage: promote
                timeout: 60
              bar:
                type: command
                command: /bin/false
                stage: promote
        """))

        self.assertEqual(len(chks), 2)

        self.assertIn('foo', chks)
        self.assertIsInstance(chks['foo'], checks.Check)
        self.assertEqual(chks['foo'].stage, 'promote')
        self.assertEqual(chks['foo'].command, '/bin/true')

        self.assertIn('bar', chks)
        self.assertIsInstance(chks['bar'], checks.Check)
        self.assertEqual(chks['bar'].stage, 'promote')
        self.assertEqual(chks['bar'].command, '/bin/false')

    def test_load_tolerates_empty_config(self):
        chks = checks.load("")

        self.assertEqual(len(chks), 0)

    def test_load_tolerates_empty_checks(self):
        chks = checks.load(dedent("""
            ---
            checks:
        """))

        self.assertEqual(len(chks), 0)

    def test_load_bad_type(self):
        with self.assertRaises(checks.CheckInvalid):
            checks.load(dedent("""
                ---
                checks:
                  foo:
                    type: badtype
            """))

    def test_custom_type(self):
        @checks.checktype('custom')
        class CustomCheck(checks.Check):
            pass

        try:
            chks = checks.load(dedent("""
                ---
                checks:
                  foo:
                    type: custom
                    command: /bin/true
                    stage: promote
            """))

            self.assertEqual(len(chks), 1)

            self.assertIn('foo', chks)
            self.assertIsInstance(chks['foo'], CustomCheck)
            self.assertEqual(chks['foo'].stage, 'promote')
            self.assertEqual(chks['foo'].command, '/bin/true')

        finally:
            del checks._types['custom']


@unittest.skipUnless(hasattr(select, 'epoll'), 'Platforms with epoll only')
class ChecksExecuteTest(unittest.TestCase):
    def setUp(self):
        self.log_stream = StringIO()
        self.log_handler = logging.StreamHandler(self.log_stream)
        self.logger = logging.getLogger('test logger')
        self.logger.setLevel(logging.INFO)
        self.logger.handlers = [self.log_handler]

    def assertLogged(self, msg):
        self.assertIn(msg, self.log_stream.getvalue())

    def test_execute(self):
        chks = [checks.Check('foo', stage='x', command='echo foo test')]
        result, done = checks.execute(chks, logger=self.logger)

        self.assertEqual(len(done), 1)
        self.assertTrue(result)

    def test_execute_failure(self):
        chks = [checks.Check('foo', stage='x', command='false')]

        result, done = checks.execute(chks, logger=self.logger)

        self.assertEqual(len(done), 1)
        self.assertFalse(result)
        self.assertLogged("Check 'foo' failed")

    def test_execute_concurrency(self):
        chks = [
            checks.Check('foo{}'.format(i), stage='x', command='sleep 0.1')
            for i in range(4)]

        result, done = checks.execute(chks, logger=self.logger, concurrency=3)

        self.assertEqual(len(done), 4)
        self.assertTrue(result)

        self.assertLess(done[1].started - done[0].started, 0.1)
        self.assertLess(done[2].started - done[0].started, 0.1)
        self.assertGreater(done[3].started - done[0].started, 0.1)

    def test_execute_timeout(self):
        chks = [checks.Check('foo', stage='x', command='sleep 0.1',
                             timeout=0.01)]

        result, done = checks.execute(chks, logger=self.logger)

        self.assertEqual(len(done), 0)
        self.assertFalse(result)
        self.assertLogged("Check 'foo' exceeded 0.01s timeout")


if __name__ == '__main__':
    unittest.main()
