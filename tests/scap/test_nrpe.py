from __future__ import absolute_import

from textwrap import dedent
import pytest

from scap import checks
from scap import nrpe


def test_load():
    config = dedent("""
    # command[foo]=/path/to/foo
    command[bar]=/path/to/bar a b
    command[baz] = /path/to/baz a b c
    """)

    commands = dict(nrpe.load(config))

    assert len(commands) == 2
    assert 'foo' not in commands
    assert 'bar' in commands
    assert 'baz' in commands

    assert commands['bar'] == '/path/to/bar a b'
    assert commands['baz'] == '/path/to/baz a b c'


def test_registered_check():
    nrpe.register({'foo': '/path/to/foo'})

    try:
        check = nrpe.NRPECheck('name', stage='promote', command='foo')
        assert isinstance(check, nrpe.NRPECheck)
    finally:
        nrpe._COMMANDS = {}


def test_unregistered_check():
    with pytest.raises(checks.CheckInvalid):
        nrpe.NRPECheck('name', stage='promote', command='foo')
