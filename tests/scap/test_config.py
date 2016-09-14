#!/usr/bin/env python2

from contextlib import contextmanager
import os
import unittest
import tempfile
from textwrap import dedent

from scap import config


class ConfigTest(unittest.TestCase):

    @contextmanager
    def config_file(self, content):
        cfg_file = tempfile.NamedTemporaryFile(delete=False)

        try:
            cfg_file.write(content)
            cfg_file.close()
            yield cfg_file.name
        finally:
            os.unlink(cfg_file.name)

    @contextmanager
    def default_config(self, items):
        orig = config.DEFAULT_CONFIG
        config.DEFAULT_CONFIG = items

        try:
            yield
        finally:
            config.DEFAULT_CONFIG = orig

    def test_load(self):
        default_config = {
            'foo': (str, None),
            'bar': (int, None),
            'baz': (bool, True),
        }

        config_file_content = dedent("""
            [global]
            foo: blah
            bar: 1
        """)

        expected_config = {
            'environment': None,
            'foo': 'blah',
            'bar': 1,
            'baz': True
        }

        with self.default_config(default_config):
            with self.config_file(config_file_content) as cfg_file:
                result = config.load(cfg_file)
                self.assertEqual(result, expected_config)

    def test_load_with_overrides(self):
        default_config = {
            'foo': (str, None),
            'bar': (int, None),
        }

        config_file_content = dedent("""
            [global]
            foo: blah
            bar: 1
        """)

        overrides = {'bar': '2'}

        expected_config = {'environment': None, 'foo': 'blah', 'bar': 2}

        with self.default_config(default_config):
            with self.config_file(config_file_content) as cfg_file:
                result = config.load(cfg_file, overrides=overrides)
                self.assertEqual(result, expected_config)

    def test_coerce(self):
        with self.default_config({'foo': (int, None)}):
            self.assertEqual(config.coerce_value('foo', '1'), 1)
            self.assertEqual(config.coerce_value('foo', '0'), 0)
            self.assertEqual(config.coerce_value('foo', '-1'), -1)

            with self.assertRaises(ValueError):
                self.assertEqual(config.coerce_value('foo', 'bar'), True)

    def test_coerce_bool(self):
        with self.default_config({'foo': (bool, None)}):
            self.assertEqual(config.coerce_value('foo', '1'), True)
            self.assertEqual(config.coerce_value('foo', 'Yes'), True)
            self.assertEqual(config.coerce_value('foo', 'True'), True)
            self.assertEqual(config.coerce_value('foo', '0'), False)
            self.assertEqual(config.coerce_value('foo', 'No'), False)
            self.assertEqual(config.coerce_value('foo', 'False'), False)

            with self.assertRaises(ValueError):
                self.assertEqual(config.coerce_value('foo', 'bar'), True)


if __name__ == '__main__':
    unittest.main()
