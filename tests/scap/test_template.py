from __future__ import absolute_import

from scap import template


def test_guess_formatter():
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

    for file_name, fmt in formats:
        assert template.guess_format(file_name) == fmt
