#!/usr/bin/env python2

import logging
import re
from StringIO import StringIO
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
        filter = log.Filter(name='A.B.*')

        self.root_handler.addFilter(filter)

        self.b_logger.info('please log this')
        self.c_logger.info('do not log this')

        self.assertEqual("please log this\n", self.stream.getvalue())

    def test_filter_filters_matching_regex(self):
        filter = log.Filter(name=re.compile(r'^A\.B\.'))

        self.root_handler.addFilter(filter)

        self.b_logger.info('please log this')
        self.c_logger.info('do not log this')

        self.assertEqual("please log this\n", self.stream.getvalue())

    def test_filter_filters_matching_lambda(self):
        filter = log.Filter(levelno=lambda lvl: lvl < logging.WARNING)

        self.root_handler.addFilter(filter)

        self.b_logger.warning('please log this')
        self.c_logger.info('do not log this')

        self.assertEqual("please log this\n", self.stream.getvalue())

    def test_filter_can_invert_behavior(self):
        filter = log.Filter(filter=False, name='A.B.*')

        self.root_handler.addFilter(filter)

        self.b_logger.info('please log this')
        self.c_logger.info('do not log this')

        self.assertEqual("do not log this\n", self.stream.getvalue())


if __name__ == '__main__':
    unittest.main()
