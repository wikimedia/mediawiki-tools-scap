#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
    scap.test_lint
    ~~~~~~~~~~~~~~
    Tests for linting things

    Copyright © 2014-2017 Wikimedia Foundation and Contributors.

    This file is part of Scap.

    Scap is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, version 3.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""
from __future__ import absolute_import

import os
import tempfile
import unittest
import sys

from subprocess import CalledProcessError

import scap.lint as lint


class LintTest(unittest.TestCase):
    """Various lint related tests"""

    @unittest.skipIf(sys.platform == 'darwin', 'Requires GNU find')
    def test_check_valid_syntax__invalid_php_file_raise_exception(self):
        """Make sure we raise exceptions when passed bad PHP files"""
        with tempfile.NamedTemporaryFile(suffix=".php") as php_file:
            php_file.write('<?php blba')
            php_file.flush()
            with self.assertRaises(CalledProcessError) as cpe:
                lint.check_valid_syntax(php_file.name)
            exc = cpe.exception
            self.assertEqual(
                exc.returncode, 124,
                'php -l command exited with status 255')

    @unittest.skipIf(sys.platform == 'darwin', 'Requires GNU find')
    def test_check_valid_syntax__skip_dir_matching_name_predicate(self):
        """Make sure that we skip directories that look like php files"""
        with tempfile.NamedTemporaryFile(suffix=".php") as php_file:
            # Delete temp file and make it a dir
            f_name = php_file.name
            php_file.close()
            os.mkdir(f_name)
            lint.check_valid_syntax(f_name)

    def test_check_php_opening_tag(self):
        """See if various files have correct PHP opening tags"""
        php_dir = os.path.join(os.path.dirname(__file__), 'php-data')
        for filename in os.listdir(php_dir):
            path = os.path.join(php_dir, filename)
            if filename.startswith('good'):
                lint.check_php_opening_tag(path)
                # No exception thrown
                self.assertTrue(True)
            elif filename.startswith('bad'):
                self.assertRaises(
                    ValueError,
                    lint.check_php_opening_tag,
                    path
                )

    def test_check_valid_json_file(self):
        """See if some files are good JSON or not"""
        json_dir = os.path.join(os.path.dirname(__file__), 'json-data')
        for filename in os.listdir(json_dir):
            path = os.path.join(json_dir, filename)
            if filename.startswith('good'):
                lint.check_valid_json_file(path)
                # No exception thrown
                self.assertTrue(True)
            elif filename.startswith('bad'):
                self.assertRaises(
                    ValueError,
                    lint.check_valid_json_file,
                    path
                )
