import importlib
import logging
import os
import sys
from ..cli import Application

this_module = sys.modules[__name__]

loaded_plugins = {}
__all__ = []


def find_plugins(plugin_dir):
    """
    Returns a list of all plugin commands found in plugin_dir

    Returns an empty list if no commands are defined.
    """

    if not os.path.isdir(plugin_dir):
        return []

    try:
        return [f[:-3] for f in os.listdir(plugin_dir)
                if not f.startswith('_') and f.endswith('.py')]
    except OSError:
        return []


def load_plugins(plugin_dir=None):
    """
    load plugins from ./scap/plugins/*.py and add them to the scap.plugins
    module namespace.
    """

    if len(loaded_plugins):
        # prevent loading plugins multiple times
        return

    if plugin_dir is None:
        plugin_dir = os.path.join(os.getcwd(), 'scap', 'plugins')

    plugins = find_plugins(plugin_dir)
    if len(plugins):
        # add plugin_dir to this module's search path
        if plugin_dir not in __path__:
            __path__.append(plugin_dir)
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
