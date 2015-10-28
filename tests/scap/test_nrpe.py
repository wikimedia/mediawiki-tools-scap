#!/usr/bin/env python2

from textwrap import dedent
import unittest

from scap import checks
from scap import nrpe


class NRPETest(unittest.TestCase):
    def test_load(self):
        config = dedent("""
        # command[foo]=/path/to/foo
        command[bar]=/path/to/bar a b
        command[baz] = /path/to/baz a b c
        """)

        commands = dict(nrpe.load(config))

        self.assertEqual(2, len(commands))
        self.assertNotIn('foo', commands)
        self.assertIn('bar', commands)
        self.assertIn('baz', commands)

        self.assertEqual('/path/to/bar a b', commands['bar'])
        self.assertEqual('/path/to/baz a b c', commands['baz'])

    def test_registered_check(self):
        nrpe.register({'foo': '/path/to/foo'})

        try:
            check = nrpe.NRPECheck('name', stage='promote', command='foo')
            self.assertIsInstance(check, nrpe.NRPECheck)
        finally:
            nrpe._commands = {}

    def test_unregistered_check(self):
        with self.assertRaises(checks.CheckInvalid):
            nrpe.NRPECheck('name', stage='promote', command='foo')


if __name__ == '__main__':
    unittest.main()
