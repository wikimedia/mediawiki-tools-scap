from __future__ import absolute_import

import logging
import unittest
from io import StringIO

from scap import ssh


class JSONOutputHandlerTest(unittest.TestCase):
    def setUp(self):
        host = "host1"
        self.output_handler = ssh.JSONOutputHandler(host)

        self.logger = logging.getLogger("target." + host)
        self.logger.setLevel(logging.INFO)

        for log_handler in self.logger.handlers:
            self.logger.removeHandler(log_handler)

        self.stream = StringIO()
        self.log_handler = logging.StreamHandler(self.stream)
        self.logger.addHandler(self.log_handler)

    def assert_logged(self, message):
        assert message == self.stream.getvalue()

    def test_lines_buffers_partial_lines(self):
        lines = self.output_handler.lines("one\ntwo\nthre")
        assert ["one", "two"] == list(lines)

        lines = self.output_handler.lines("e\nfour\n")
        assert ["three", "four"] == list(lines)

    def test_accept_parses_and_logs_json_messages(self):
        self.output_handler.accept('{ "msg": "foo" }\n')
        self.assert_logged("foo\n")

    def test_accept_ignores_bad_json(self):
        self.output_handler.accept("invalid json")
        self.assert_logged("")

    def tearDown(self):
        self.logger.removeHandler(self.log_handler)
        self.log_handler.close()
