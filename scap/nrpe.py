# -*- coding: utf-8 -*-
"""
    scap.nrpe
    ~~~~~~~~~
    NRPE based deployment checks.

    Available Icinga/NRPE command definitions must be loaded and registered
    for use using `load` or `load_directory`, and `register`, before they can
    be used in `checks.yaml` configuration.

    Example `checks.yaml`:
        checks:
          service_endpoints:
            type: nrpe
            command: check_service_endpoints
            stage: promote


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
import re

from . import checks

_commands = {}


def load(config):
    """
    Load NRPE command definitions from the given configuration.

    :param config: NRPE configuration string

    :yields: (name, command)
    """

    for line in config.splitlines():
        line = line.strip()

        if line.startswith('#'):
            continue

        match = re.match(r'command\[(.+)\] *= *(.+)$', line)

        if match:
            yield match.group(1), match.group(2)


def load_directory(config_dir):
    """
    Load available local NRPE check commands from the given directory.

    :param config_dir: directory in which to look for NRPE configuration

    :yields: (name, command)
    """

    try:
        for path in os.listdir(config_dir):
            config_file = os.path.join(config_dir, path)

            if not os.path.isfile(config_file):
                continue

            with open(config_file, 'r') as cfg:
                for name, command in load(cfg.read()):
                    yield name, command

    except OSError:
        pass


def register(commands):
    """Register global NRPE commands for use in check configuration."""

    _commands.update(commands)


@checks.checktype('nrpe')
class NRPECheck(checks.Check):
    """Represent a loaded 'nrpe' check."""

    def validate(self):
        """Validates that the configured NRPE check is available."""

        checks.Check.validate(self)

        if self.command in _commands:
            self.command = _commands[self.command]
        else:
            msg = "command '{}' is not a valid NRPE check".format(self.command)
            raise checks.CheckInvalid(msg)
