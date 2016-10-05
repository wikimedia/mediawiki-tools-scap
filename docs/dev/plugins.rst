
.. _plugins:

####################
Writing scap plugins
####################

As of version 3.3.0, scap supports a rudimentary plugin API to allow users to
build their own command line interfaces utilizing scap's argument parser and
rich set of utility methods.

Features
--------

Scap provides a robust command line tool with infrastructure for easily defining
and validating arguments. In addition to validation, scap will handle generating
`--help` documentation for your commands and comprehensive bash autocompletion
of complex argument values.

How it works
------------

#. At runtime, scap calls :func:`scap.plugins.find_plugins` which scans for any
   `*.py` files in a couple of standard locations:

  * the current project :file:`./scap/plugins/{*}.py`
  * the current user's home directory :file:`~/.scap/plugins/{*}.py`

#. Any matching files are loaded by :func:`scap.plugins.load_plugins` and
   the code evaluated in the context of the scap.plugins package namespace.
#. Finally, any classes within a plugin module which inherit from
   :class:`scap.cli.Application` are registered as scap subcommands
   (see :func:`scap.cli.subcommand`).

So, for example, if you have a file named `scap/plugins/hello.py` relative
to either the current working directory or your user's home directory, then
scap will evaluate hello.py as a module named :mod:`scap.plugins.hello`.
Now if you want it to actually work, then hopefully you have included a
class inside hello.py, defined similarly to the following example.

**Subcommand Example**::

    from scap import cli

    @cli.command('hello', subcommands=True,
                 help='prints "hello world" and exits',)
    class HelloCommand(cli.Application):
        @cli.subcommand('world')
        def world_subcommand(extra_args):
            print('hello world')

.. seealso::
   * The core command-line application infrastructure is in :mod:`scap.cli`
   * Many of the built-in subcommands are defined in :mod:`scap.main`
   * :class:`scap.plugins.Say`
