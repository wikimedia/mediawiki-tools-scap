# -*- coding: utf-8 -*-
"""
    scap.template
    ~~~~~~~~~~~~~
    Module for working with file templates

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
from __future__ import unicode_literals

import jinja2
import yaml


VALID_OUTPUT_FORMATS = ['yaml', 'yml']
OUTPUT_ALIASES = {
    'yml': 'yaml'
}


def yaml_finalize(value):
    """
    Output yaml values rather than pythonic values
    """
    if value is None:
        return 'null'

    if value is True:
        return 'true'

    if value is False:
        return 'false'

    return value


def guess_format(fn):
    """
    Guess the output format based on config file extension.
    """
    if '.' not in fn:
        return None

    parts = fn.split('.')

    # if the file ends in j2, drop the j2
    if parts[::-1][0] == 'j2':
        parts = parts[:-1]

    fmt = parts[::-1][0]
    if fmt not in VALID_OUTPUT_FORMATS:
        return None

    return OUTPUT_ALIASES.get(fmt, fmt)


def get_output_formatter(fmt):
    """
    Get output formatter based on desired output format.
    """
    return globals()['{}_finalize'.format(OUTPUT_ALIASES.get(fmt, fmt))]


class Template(object):
    """Adapter class that wraps jinja2 templates."""
    def __init__(self, name, loader, erb_syntax=False, var_file=None,
                 overrides=None, output_format=None):
        env_args = self._make_env_args(loader, erb_syntax, output_format)
        self._env = jinja2.Environment(**env_args)
        self._template = self._env.get_template(name)
        self._overrides = overrides
        self.var_file = var_file

    def _make_env_args(self, loader, erb_syntax, output_format):
        """Generate properties to pass to the jinja template."""
        loader = jinja2.DictLoader(loader)
        env_args = {
            'loader': loader,
        }

        if output_format in VALID_OUTPUT_FORMATS:
            finalize_func = get_output_formatter(output_format)
            env_args['finalize'] = finalize_func

        if erb_syntax:
            env_args.update({
                'block_start_string': '<%',
                'block_end_string': '%>',
                'variable_start_string': '<%=',
                'variable_end_string': '%>',
                'comment_start_string': '<%#',
                'comment_end_string': '%>',
            })

        return env_args

    def _get_file_vars(self):
        """
        Load yaml var file if it exists.

        :return: dict variables for template use
        """
        if not self.var_file:
            return {}

        with open(self.var_file, 'r') as variables:
            return yaml.load(variables.read())

    def render(self):
        """
        Renders the templates specified by `self.name`.

        It uses the variables sourced from the import yaml
        file specified by `self.var_file`
        """
        template_vars = self._get_file_vars()
        if self._overrides:
            overrides = self._overrides
            overrides.update(template_vars)
            template_vars = overrides
        return self._template.render(template_vars)
