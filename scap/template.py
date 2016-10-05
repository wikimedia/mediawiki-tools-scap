# -*- coding: utf-8 -*-
"""
    scap.template
    ~~~~~~~~~~~~~
    Module for working with file templates
"""

import jinja2
import yaml


class Template(object):
    """Adapter class that wraps jinja2 templates."""
    def __init__(self, name, loader, erb_syntax=False, var_file=None,
                 overrides=None):
        env_args = self._make_env_args(loader, erb_syntax)
        self._env = jinja2.Environment(**env_args)
        self._template = self._env.get_template(name)
        self._overrides = overrides
        self.var_file = var_file

    def _make_env_args(self, loader, erb_syntax):
        """Generate properties to pass to the jinja template."""
        loader = jinja2.DictLoader(loader)
        env_args = {'loader': loader}
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
