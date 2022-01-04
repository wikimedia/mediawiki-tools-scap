from __future__ import absolute_import

from contextlib import contextmanager
import os
import tempfile
from textwrap import dedent
import pytest

from scap import config


@contextmanager
def config_file(content, environment=None):
    with tempfile.TemporaryDirectory() as base_dir:
        scap_dir = os.path.join(base_dir, "scap")
        os.mkdir(scap_dir)

        if environment:
            os.mkdir(os.path.join(scap_dir, "environments"))
            os.mkdir(os.path.join(scap_dir, "environments", environment))
            cfg_file_name = os.path.join(
                scap_dir, "environments", environment, "scap.cfg"
            )
        else:
            cfg_file_name = os.path.join(scap_dir, "scap.cfg")

        with open(cfg_file_name, "w") as cfg_file:
            cfg_file.write(content)

        yield cfg_file_name


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


def test_existing_environment(monkeypatch):
    default_config = {"foo": (str, None)}

    config_file_content = dedent(
        """
        [global]
        foo: blah
    """
    )

    expected_config = {"environment": "staging", "foo": "blah"}

    with override_default_config(default_config):
        with config_file(config_file_content, environment="staging") as cfg_file:
            # the returned filename is some-base-path/scap/environments/staging/scap.cfg
            # this ugly dirname() stack returns some-base-path (which is what we want)
            monkeypatch.chdir(
                os.path.dirname(
                    os.path.dirname(os.path.dirname(os.path.dirname(cfg_file)))
                )
            )

            assert config.load(environment="staging") == expected_config


def test_nonexistent_environment(monkeypatch):
    default_config = {"foo": (str, None)}

    config_file_content = dedent(
        """
        [global]
        foo: blah
    """
    )

    with override_default_config(default_config):
        with config_file(config_file_content) as cfg_file:
            monkeypatch.chdir(os.path.dirname(cfg_file))

            with pytest.raises(RuntimeError):
                config.load(environment="staging")


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
