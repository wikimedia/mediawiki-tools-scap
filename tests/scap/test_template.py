#!/usr/bin/env python2

import unittest

import scap.template as template


class TemplateTest(unittest.TestCase):

    def test_guess_formatter(self):
        formats = [
            ('config.yaml', 'yaml'),
            ('config.yml', 'yaml'),
            ('config.yml.j2', 'yaml'),
            ('config.yaml.j2', 'yaml'),
            ('the.config.yaml.j2', 'yaml'),
            ('/some/config/file/the.config.yaml.j2', 'yaml'),
            ('/some/config/file/the.config.json.j2', None),
            ('config.json.j2', None),
            ('config.json.yml.j2', 'yaml'),
            ('config.yaml.json.j2', None),
        ]

        for fn, fmt in formats:
            self.assertEqual(template.guess_format(fn), fmt)


if __name__ == '__main__':
    unittest.main()
