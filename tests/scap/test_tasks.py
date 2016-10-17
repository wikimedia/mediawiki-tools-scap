#!/usr/bin/env python2

import os
import sys
import tempfile
import unittest
from subprocess import CalledProcessError

from scap import tasks


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


if __name__ == '__main__':
    unittest.main()
