#!/usr/bin/env python2

import logging
import unittest

from scap import ssh
from StringIO import StringIO


class JSONOutputHandlerTest(unittest.TestCase):
    def setUp(self):
        self.host = 'host1'
        self.output_handler = ssh.JSONOutputHandler(self.host)

        self.logger = logging.getLogger(self.host)
        self.logger.setLevel(logging.DEBUG)

        for log_handler in self.logger.handlers:
            self.logger.removeHandler(log_handler)

        self.stream = StringIO()
        self.log_handler = logging.StreamHandler(self.stream)
        self.logger.addHandler(self.log_handler)

    def assertLogged(self, message):
        self.assertEqual(message, self.stream.getvalue())

    def test_lines_buffers_partial_lines(self):
        lines = self.output_handler.lines("one\ntwo\nthre")
        self.assertEqual(['one', 'two'], list(lines))

        lines = self.output_handler.lines("e\nfour\n")
        self.assertEqual(['three', 'four'], list(lines))

    def test_accept_parses_and_logs_json_messages(self):
        self.output_handler.accept("{ \"message\": \"foo\" }\n")
        self.assertLogged("foo\n")

    def test_accept_ignores_bad_json(self):
        self.output_handler.accept('invalid json')
        self.assertLogged('')

    def tearDown(self):
        self.logger.removeHandler(self.log_handler)
        self.log_handler.close()

if __name__ == '__main__':
    unittest.main()
