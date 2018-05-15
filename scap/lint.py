# -*- coding: utf-8 -*-
"""
    scap.lint
    ~~~~~~~~~
    Stuff about linting

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

import json
import logging
import os
import subprocess


def check_valid_syntax(paths, procs=1):
    """Run php -l in parallel on `paths`; raise CalledProcessError if nonzero
    exit."""
    if isinstance(paths, str):
        paths = [paths]
    elif not isinstance(paths, list):
        raise ValueError('paths must be a path or list of paths')

    logger = logging.getLogger('check_php_syntax')
    quoted_paths = ["'%s'" % x for x in paths]
    cmd = (
        "find "
        "-O2 "  # -O2 get -type executed after
        "%s "
        "-not -type d "  # makes no sense to lint a dir named 'less.php'
        "-name '*.php' -not -name 'autoload_static.php' "
        " -or -name '*.inc' | xargs -n1 -P%d -exec php -l >/dev/null 2>&1"
    ) % (' '.join(quoted_paths), procs)
    logger.debug('Running command: `%s`', cmd)
    subprocess.check_call(cmd, shell=True)
    # Check for anything that isn't a shebang before <?php (T92534)
    for path in paths:
        for root, _, files in os.walk(path):
            for filename in files:
                abspath = os.path.join(root, filename)
                check_php_opening_tag(abspath)
                check_valid_json_file(abspath)


def check_valid_json_file(path):
    """
    Is a file actually JSON?

    :param path: Location of file
    :raises: ValueError for an invalid file
    """
    if not path.endswith('.json'):
        return
    with open(path) as json_file:
        try:
            json.load(json_file)
        except ValueError:
            raise ValueError('%s is an invalid JSON file' % path)


def check_php_opening_tag(path):
    """
    Check a PHP file to make sure nothing is before the opening <?php.

    Except for shebangs.

    :param path: Location of file
    :raises: ValueError on invalid file
    """
    if not path.endswith(('.php', '.inc')):
        return
    with open(path) as php_file:
        text = php_file.read()

        # Empty files are ok
        if len(text) < 1:
            return

        # Best case scenario to begin with the php open tag
        if text.lower().startswith('<?php'):
            return

        # Also reasonable to start with a doctype declaration
        if text.startswith('<!DOCTYPE'):
            return

        # If the first line is a shebang and the
        # second has <?php, that's ok
        lines = text.splitlines()

        if (len(lines) > 1 and
                lines[0].startswith('#!') and
                lines[1].lower().startswith('<?php')):
            return

        # None of the return conditions matched, the file must contain <?php
        # but with some content preceeding it.
        raise ValueError(
            '%s has content before opening <?php tag' % path)
