# -*- coding: utf-8 -*-
"""
    scap.template
    ~~~~~~~~~~
    Module for working with file templates
"""

import jinja2
import yaml


class Template(object):
    """Adapter class that wraps jinja2 templates
    """
    def __init__(self, name, loader, var_file=None, overrides=None):
        loader = jinja2.DictLoader(loader)
        self._env = jinja2.Environment(loader=loader)
        self._template = self._env.get_template(name)
        self._overrides = overrides
        self.var_file = var_file

    def _get_file_vars(self):
        if not self.var_file:
            return {}

        with open(self.var_file, 'r') as variables:
            return yaml.load(variables.read())

    def render(self):
        """Renders the templates specified by `self.name` using the
        variables sourced from the import yaml file specified by
        `self.var_file`
        """
        template_vars = self._get_file_vars()
        if self._overrides:
            overrides = self._overrides
            overrides.update(template_vars)
            template_vars = overrides
        return self._template.render(template_vars)
