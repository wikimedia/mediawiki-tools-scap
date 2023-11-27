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
import os
import tempfile
import sys

import pytest

from scap import lint


def test_clean_lint_output():
    output = """
No syntax errors detected in /srv/mediawiki-staging/wmf-config/filebackend.php
No syntax errors detected in /srv/mediawiki-staging/wmf-config/mc-labs.php
No syntax errors detected in /srv/mediawiki-staging/wmf-config/abusefilter.php
PHP Parse error:  syntax error, unexpected 'if' (T_IF) in /srv/mediawiki-staging/wmf-config/CommonSettings.php on line 52
Errors parsing /srv/mediawiki-staging/wmf-config/CommonSettings.php
No syntax errors detected in /srv/mediawiki-staging/wmf-config/wikitech.php
No syntax errors detected in /srv/mediawiki-staging/wmf-config/db-codfw.php
No syntax errors detected in /srv/mediawiki-staging/wmf-config/Wikibase.php
No syntax errors detected in /srv/mediawiki-staging/wmf-config/PoolCounterSettings.php
xargs: php: exited with status 255; aborting
No syntax errors detected in /srv/mediawiki-staging/wmf-config/InitialiseSettings.php
No syntax errors detected in /srv/mediawiki-staging/wmf-config/mc.php
No syntax errors detected in /srv/mediawiki-staging/wmf-config/profiler.php
No syntax errors detected in /srv/mediawiki-staging/wmf-config/trusted-xff.php
"""
    assert (
        lint.clean_lint_output(output)
        == """
PHP Parse error:  syntax error, unexpected 'if' (T_IF) in /srv/mediawiki-staging/wmf-config/CommonSettings.php on line 52
Errors parsing /srv/mediawiki-staging/wmf-config/CommonSettings.php"""
    )


@pytest.mark.skipif(sys.platform == "darwin", reason="Requires GNU find")
def test_check_valid_syntax__invalid_php_file_raise_exception():
    """Make sure we raise exceptions when passed bad PHP files"""
    with tempfile.NamedTemporaryFile(suffix=".php") as php_file:
        php_file.write(b"<?php blba")
        php_file.flush()
        with pytest.raises(SystemExit):
            lint.check_valid_syntax(php_file.name)


@pytest.mark.skipif(sys.platform == "darwin", reason="Requires GNU find")
def test_check_valid_syntax__invalid_json_file_raise_exception():
    """Make sure we raise exceptions when passed a single bad JSON file (T272756)"""
    with tempfile.NamedTemporaryFile(suffix=".json") as json_file:
        json_file.write(b"{")
        json_file.flush()
        with pytest.raises(ValueError) as ve:
            lint.check_valid_syntax(json_file.name)
        assert "is an invalid JSON file" in str(ve.value)


@pytest.mark.skipif(sys.platform == "darwin", reason="Requires GNU find")
def test_check_valid_syntax__skip_dir_matching_name_predicate():
    """Make sure that we skip directories that look like php files"""
    with tempfile.NamedTemporaryFile(suffix=".php") as php_file:
        # Delete temp file and make it a dir
        f_name = php_file.name
        php_file.close()
        os.mkdir(f_name)
        lint.check_valid_syntax(f_name)


def test_check_php_opening_tag():
    """See if various files have correct PHP opening tags"""
    php_dir = os.path.join(os.path.dirname(__file__), "php-data")
    for filename in os.listdir(php_dir):
        path = os.path.join(php_dir, filename)
        if filename.startswith("good"):
            lint.check_php_opening_tag(path)
            # No exception thrown
        elif filename.startswith("bad"):
            pytest.raises(ValueError, lint.check_php_opening_tag, path)


def test_check_valid_json_file():
    """See if some files are good JSON or not"""
    json_dir = os.path.join(os.path.dirname(__file__), "json-data")
    for filename in os.listdir(json_dir):
        path = os.path.join(json_dir, filename)
        if filename.startswith("good"):
            lint.check_valid_json_file(path)
            # No exception thrown
        elif filename.startswith("bad"):
            pytest.raises(ValueError, lint.check_valid_json_file, path)
