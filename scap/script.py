# -*- coding: utf-8 -*-
"""
    scap.script
    ~~~~~~~~~
    Run script after stage

    checks the scap/scripts directory for scripts

    Example `checks.yaml`:
        checks:
          do_after:
            type: script
            command: do_after.sh
            stage: promote


    Copyright © 2014-2018 Wikimedia Foundation and Contributors.

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

import scap.checks as checks

_SCRIPTS = {}


def register_directory(scripts_path):
    """"
    Load available executable checks scripts from a given path

    :param scripts_path: directory in which to look for check scripts
    """
    if not os.path.isdir(scripts_path):
        return

    for script in list(next(os.walk(scripts_path))[-1]):
        script_path = os.path.join(scripts_path, script)
        if os.path.isfile(script_path) and os.access(script_path, os.X_OK):
            register(script, script_path)


def register(script, script_path):
    """Register global scripts for use in check validation."""
    _SCRIPTS[script] = script_path


@checks.checktype('script')
class ScriptCheck(checks.Check):
    """Represent a loaded 'nrpe' check."""

    def validate(self):
        """Validates that the configured NRPE check is available."""

        checks.Check.validate(self)

        if self.command in _SCRIPTS:
            self.command = _SCRIPTS[self.command]
        else:
            msg = "the script '{}' was not found".format(self.command)
            raise checks.CheckInvalid(msg)
