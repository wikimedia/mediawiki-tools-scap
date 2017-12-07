#!/usr/bin/env python2

from __future__ import absolute_import

import unittest

import scap.cmd as cmd


class CommandTest(unittest.TestCase):

    def test_command_args(self):
        ssh = cmd.Command('/usr/bin/ssh', cmd.arg('user', '-oUser={}'))
        sudo = cmd.Command('sudo', cmd.arg('user', '-u {}'), '-n', '--')
        cmdline = ssh('some.host', sudo('remote_cmd', 'some', 'args',
                                        user='sudo_user'),
                      user='ssh_user')

        self.assertEqual(cmdline, ['/usr/bin/ssh',
                                   '-oUser=ssh_user',
                                   'some.host',
                                   'sudo',
                                   '-u sudo_user',
                                   '-n',
                                   '--',
                                   'remote_cmd',
                                   'some',
                                   'args'])
