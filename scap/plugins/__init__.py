import importlib
import logging
import os
import sys
from ..cli import Application

this_module = sys.modules[__name__]

loaded_plugins = {}
__all__ = []


def find_plugins(plugin_dirs):
    """
    Returns a list of all plugin commands found in plugin_dir

    Returns an empty list if no commands are defined.
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

    if len(loaded_plugins):
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
    # import each of the plugin modules
    for plugin in plugins:
        # module path relative to scap.plugins:
        plugin_module = ".%s" % plugin
        mod = importlib.import_module(plugin_module, "scap.plugins")
        # find classes in mod which extend scap.cli.Application
        for objname in dir(mod):
            obj = getattr(mod, objname)
            if type(obj) is type and issubclass(obj, Application):
                if objname in loaded_plugins:
                    # duplicate: another plugin already used the same name
                    msg = 'Duplicate plugin named %s, skipping.' % objname
                    logging.getLogger().warning(msg)
                    continue
                # copy the class into the scap.plugins namespace
                setattr(this_module, objname, obj)
                loaded_plugins[objname] = obj
                __all__.append(objname)
