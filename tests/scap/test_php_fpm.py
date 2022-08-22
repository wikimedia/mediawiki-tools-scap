# -*- coding: utf-8 -*-
"""
    scap.test_php_restart.py
    ~~~~~~~~~~
    Contains misc utility functions.

"""
from __future__ import absolute_import


import copy
import pytest
import sys

from scap import php_fpm, ssh


PHPRESTART_PARAMS = {
    "php_fpm_restart_script": "/bin/foo",
    "php_fpm": "php7.2-fpm",
    "php_fpm_opcache_threshold": 100,
    "ssh_user": "mwdeploy",
}


@pytest.fixture
def php_restart():
    """
    Build a PHPRestartObject
    """
    return php_fpm.PHPRestart(PHPRESTART_PARAMS, ssh.Job())


def test_init(php_restart):
    """Test php_restart initialization"""
    assert php_restart.cmd == "/bin/foo php7.2-fpm 100"


def test_init_unsafe(php_restart):
    """Test php_restart initialization, unsafe mode"""
    params = copy.deepcopy(PHPRESTART_PARAMS)
    params["php_fpm_unsafe_restart_script"] = "/bin/foo-unsafe"
    pr = php_fpm.PHPRestart(params, ssh.Job(), True)
    assert pr.cmd == "/bin/foo-unsafe --force"


def test_build_job(php_restart):
    """
    Test build job
    """
    php_restart.set_progress_queue(None)
    php_restart._build_job(["x"])
    assert php_restart.job._hosts == ["x"]
    assert (
        php_restart.job._command == "/usr/bin/sudo -u root -- /bin/foo php7.2-fpm 100"
    )


def test_always_restart():
    """
    Test feature flag for unconditional php-fpm restarts
    """
    params = copy.deepcopy(PHPRESTART_PARAMS)
    params["php_fpm_always_restart"] = True
    php_restart = php_fpm.PHPRestart(params, ssh.Job())

    assert str(sys.maxsize) in php_restart.cmd


def test_build_job_raises():
    """Ensure that php_fpm raises AttributeError if no job is set"""
    php_restart_no_job = php_fpm.PHPRestart(PHPRESTART_PARAMS)
    with pytest.raises(AttributeError):
        php_restart_no_job._build_job(["x"])


def test_get_batch_size():
    """
    test get_batch_size function
    """
    assert php_fpm.get_batch_size(range(0, 1000)) == 100
    assert php_fpm.get_batch_size(range(0, 1000), percentage=20) == 200


def test_cmd_and_job_exist():
    """
    Test to ensure that cmd and job exist

    Many functions expect cmd and job to exist as members of the PHPRestart
    class. This ensures that they're set, even if to falsy values.
    """
    php_restart = php_fpm.PHPRestart({"ssh_user": "test"})
    assert php_restart.cmd is None
    assert php_restart.job is None
