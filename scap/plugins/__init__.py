# -*- coding: utf-8 -*-
"""
    scap.plugins
    ~~~~~~~~~~~~
    Scap plugin architecture

    .. function:: find_plugins(plugin_dirs)

        Get a list of all plugins found in in plugin_dirs

        :param list plugin_dirs: directories to search for plugins
        :return: list of all plugin commands found in plugin_dirs
    .. function:: load_plugins([plugin_dir])

        load scap plugin modules.

        :type plugin_dir: str or None
        :param str plugin_dir: an additional location to search

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

import importlib
import logging
import os
import sys
from scap.cli import Application
from scap.plugins.say import Say

THIS_MODULE = sys.modules[__name__]
LOADED_PLUGINS = {}
__all__ = ['Say']


def find_plugins(plugin_dirs):
    """
    returns a list of all plugin commands found in plugin_dirs
    """
    plugins = []

    for d in plugin_dirs:

        if d is None or not os.path.exists(d):
            continue

        file_list = os.listdir(os.path.realpath(d))
        if len(file_list) < 1:
            continue

        try:
            for f in file_list:
                if not f.startswith('_') and f.endswith('.py'):
                    plugins.append(f[:-3])

        except OSError:
            continue

        # add plugin_dir to this module's search path
        if d not in __path__:
            __path__.append(d)

    return plugins


def load_plugins(plugin_dir=None):
    """
    load plugins from ./scap/plugins/*.py and add them to the scap.plugins
    module namespace.
    """

    if LOADED_PLUGINS:
        # prevent loading plugins multiple times
        return

    plugin_dirs = [
        plugin_dir,
        os.path.join(os.getcwd(), 'scap', 'plugins'),
        os.path.join(os.path.expanduser('~'), '.scap', 'plugins')
    ]

    plugins = find_plugins(plugin_dirs)
    if len(plugins) < 1:
        return

    # Turn off those obnoxious *.pyc files for plugins so we don't litter
    maybe_write_bytecode = sys.dont_write_bytecode
    sys.dont_write_bytecode = True

    # import each of the plugin modules
    for plugin in plugins:
        # module path relative to scap.plugins:
        plugin_module = ".%s" % plugin
        try:
            mod = importlib.import_module(plugin_module, "scap.plugins")
            # find classes in mod which extend scap.cli.Application
            for objname in dir(mod):
                obj = getattr(mod, objname)
                if isinstance(obj, type) and issubclass(obj, Application):
                    if objname in LOADED_PLUGINS:
                        # duplicate: another plugin already used the same name
                        msg = 'Duplicate plugin named %s, skipping.'
                        logging.getLogger().warning(msg, objname)
                        continue
                    # copy the class into the scap.plugins namespace
                    setattr(THIS_MODULE, objname, obj)
                    LOADED_PLUGINS[objname] = obj
                    __all__.append(objname)
        except Exception as e:
            msg = 'Problem loading plugins from module: scap.plugins.%s (%s)'
            err_msg = type(e).__name__ + ':' + str(e)
            logging.getLogger().warning(msg, plugin, err_msg)

    # Restore the original setting
    sys.dont_write_bytecode = maybe_write_bytecode
