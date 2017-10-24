#!/usr/bin/env python2

from __future__ import absolute_import

import os
import sys
import tempfile
import unittest

from datetime import datetime, timedelta
from subprocess import CalledProcessError

import scap.tasks as tasks


class TasksTest(unittest.TestCase):

    @unittest.skipIf(sys.platform == 'darwin', 'Requires GNU find')
    def test_check_valid_syntax__invalid_php_file_raise_exception(self):
        with tempfile.NamedTemporaryFile(suffix=".php") as f:
            f.write('<?php blba')
            f.flush()
            with self.assertRaises(CalledProcessError) as cm:
                tasks.check_valid_syntax(f.name)
            exc = cm.exception
            self.assertEqual(
                exc.returncode, 124,
                'php -l command exited with status 255')

    @unittest.skipIf(sys.platform == 'darwin', 'Requires GNU find')
    def test_check_valid_syntax__skip_dir_matching_name_predicate(self):
        with tempfile.NamedTemporaryFile(suffix=".php") as f:
            # Delete temp file and make it a dir
            f_name = f.name
            f.close()
            os.mkdir(f_name)
            tasks.check_valid_syntax(f_name)

    def test_get_old_wikiversions(self):
        now = datetime.utcnow()

        one_week_ago = now - timedelta(weeks=1)
        two_weeks_ago = now - timedelta(weeks=2)
        three_weeks_ago = now - timedelta(weeks=5)
        ten_weeks_ago = now - timedelta(weeks=10)

        # What is expected is that the 2 new versions (wmf.4 & wmf.5) will
        # be kept, regardless of when we determined the branching to happen.
        #
        # Of the remaining versions, 2 are within a period of time where we
        # would like to keep their static assets (wmf.2 & wmf.3).  The
        # remaining version is 10 weeks old, that should be removed entirely
        versions = [
            ('php-1.29.0-wmf.4', now),
            ('php-1.29.0-wmf.3', one_week_ago),
            ('php-1.29.0-wmf.2', two_weeks_ago),
            ('php-1.29.0-wmf.1', ten_weeks_ago),
            ('php-1.29.0-wmf.5', three_weeks_ago)]

        remove, remove_static = tasks.get_old_wikiversions(versions)

        self.assertEqual(remove, ['php-1.29.0-wmf.1'])

        # Returns versions to remove in reverse order
        self.assertEqual(
            remove_static, ['php-1.29.0-wmf.3', 'php-1.29.0-wmf.2'])


if __name__ == '__main__':
    unittest.main()
