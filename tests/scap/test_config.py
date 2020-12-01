from __future__ import absolute_import

from contextlib import contextmanager
import os
import tempfile
from textwrap import dedent
import pytest

from scap import config


@contextmanager
def config_file(content):
    cfg_file = tempfile.NamedTemporaryFile(delete=False)

    try:
        cfg_file.write(content.encode("utf-8"))
        cfg_file.close()
        yield cfg_file.name
    finally:
        os.unlink(cfg_file.name)


@contextmanager
def override_default_config(items):
    orig = config.DEFAULT_CONFIG
    config.DEFAULT_CONFIG = items

    try:
        yield
    finally:
        config.DEFAULT_CONFIG = orig


def test_load():
    default_config = {"foo": (str, None), "bar": (int, None), "baz": (bool, True)}

    config_file_content = dedent(
        """
        [global]
        foo: blah
        bar: 1
    """
    )

    expected_config = {"environment": None, "foo": "blah", "bar": 1, "baz": True}

    with override_default_config(default_config):
        with config_file(config_file_content) as cfg_file:
            result = config.load(cfg_file)
            assert result == expected_config


def test_load_with_overrides():
    default_config = {"foo": (str, None), "bar": (int, None)}

    config_file_content = dedent(
        """
        [global]
        foo: blah
        bar: 1
    """
    )

    overrides = {"bar": "2"}

    expected_config = {"environment": None, "foo": "blah", "bar": 2}

    with override_default_config(default_config):
        with config_file(config_file_content) as cfg_file:
            result = config.load(cfg_file, overrides=overrides)
            assert result == expected_config


def test_coerce():
    with override_default_config({"foo": (int, None)}):
        assert config.coerce_value("foo", "1") == 1
        assert config.coerce_value("foo", "0") == 0
        assert config.coerce_value("foo", "-1") == -1

        with pytest.raises(ValueError):
            assert config.coerce_value("foo", "bar")


def test_coerce_bool():
    with override_default_config({"foo": (bool, None)}):
        assert config.coerce_value("foo", "1")
        assert config.coerce_value("foo", "Yes")
        assert config.coerce_value("foo", "True")
        assert not config.coerce_value("foo", "0")
        assert not config.coerce_value("foo", "No")
        assert not config.coerce_value("foo", "False")

        with pytest.raises(ValueError):
            assert config.coerce_value("foo", "bar")
