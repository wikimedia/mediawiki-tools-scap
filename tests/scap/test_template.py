# -*- coding: utf-8 -*-
from __future__ import absolute_import

import os

from scap import template


def test_guess_formatter():
    formats = [
        ("config.yaml", "yaml"),
        ("config.yml", "yaml"),
        ("config.yml.j2", "yaml"),
        ("config.yaml.j2", "yaml"),
        ("the.config.yaml.j2", "yaml"),
        ("/some/config/file/the.config.yaml.j2", "yaml"),
        ("/some/config/file/the.config.json.j2", None),
        ("config.json.j2", None),
        ("config.json.yml.j2", "yaml"),
        ("config.yaml.json.j2", None),
    ]

    for file_name, fmt in formats:
        assert template.guess_format(file_name) == fmt


def test_render_utf8_vars():
    test_file = "test.out.yaml"

    template_path = os.path.join(
        os.path.dirname(__file__), "template-data", "test.yaml"
    )

    # Contains ❤
    override_path = os.path.join(
        os.path.dirname(__file__), "template-data", "vars-utf8.yaml"
    )

    with open(template_path, "r") as f:
        template_data = f.read()

    tmp = template.Template(
        name=test_file, loader={test_file: template_data}, var_file=override_path
    )

    assert u"❤".encode("utf-8") in tmp.render()


def test_render_utf8_template():
    test_file = "test.out.yaml"

    template_path = os.path.join(
        os.path.dirname(__file__), "template-data", "test-utf8.yaml"
    )

    with open(template_path, "r") as f:
        template_data = f.read()

    tmp = template.Template(name=test_file, loader={test_file: template_data})
    assert u"⚡".encode("utf-8") in tmp.render()
