# -*- coding: utf-8 -*-
"""
    scap.cmd
    ~~~~~~~~~~~
    command execution

"""


class Command(object):
    """
    Command is used to store a parameterized shell command for reuse
    by cluster_ssh and other utilities. The idea is that we store the command,
    along with any arguments that need to be filled in, then we can call the
    command later just like a normal python function, which returns the shell
    command to be ran.

    Usage Example::

        import scap.cmd as cmd

        ssh = cmd.Command('/usr/bin/ssh', cmd.arg('user', '-oUser={}'))
        sudo = cmd.Command('sudo', cmd.arg('user', '-u {}'), '-n', '--')
        ssh('some.host', sudo('remote_cmd', 'some', 'args', user='sudo_user'),
            user='ssh_user')

        # result:

        ['/usr/bin/ssh',
         '-oUser=ssh_user',
         'some.host',
         'sudo',
         '-u sudo_user',
         '-n',
         '--',
         'remote_cmd',
         'some',
         'args']
    """
    def __init__(self, *cmds):
        self.cmds = cmds

    def __call__(self, *args, **values):
        result = []
        parts = self.cmds + args
        for cmd in parts:
            rendered = cmd
            if callable(cmd):
                rendered = cmd(**values)
            if rendered:
                if type(rendered) is str:
                    result.append(rendered)
                else:
                    result.extend(rendered)
        return result


class arg(object):
    """
    an `arg` represents a named parameter which will be passed to an instance
    of Command.

    For example, ``arg('user', '-u {}')`` creates an arg called 'user' which
    will evaluate to ``"-u luser"`` when the command is evaluated with
    ``some_command(user='luser')``

    :param name: The name of the parameter, used to specify it's value later
                 when the command is evaluated.
    :param cmd: string, the argument text format string.
    """
    def __init__(self, name, cmd):
        self.name = name
        self.cmd = cmd

    def required(self, required=True):
        self._required = required
        return self

    def __call__(self, **values):
        if self.name in values and values[self.name]:
            val = values[self.name]
            return self.cmd.format(val)
        else:
            return ''
