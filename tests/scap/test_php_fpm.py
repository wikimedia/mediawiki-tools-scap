# -*- coding: utf-8 -*-
"""
    scap.test_php_restart.py
    ~~~~~~~~~~
    Contains misc utility functions.

"""
from __future__ import absolute_import


import pytest

from scap import php_fpm, ssh


PHPRESTART_PARAMS = {
    'php_fpm_restart_script': '/bin/foo',
    'php_fpm': 'php7.2-fpm',
    'php_fpm_opcache_threshold': 100,
}


@pytest.fixture
def php_restart():
    """
    Build a PHPRestartObject
    """
    return php_fpm.PHPRestart(
        PHPRESTART_PARAMS,
        ssh.Job()
    )


def test_init(php_restart):
    """Test php_restart initialization"""
    assert php_restart.cmd == ['/bin/foo', 'php7.2-fpm', '100']


def test_build_job(php_restart):
    """
    Test build job
    """
    php_restart._build_job(['x'])
    assert php_restart.job._hosts == ['x']
    assert php_restart.job._command == [
        '/bin/foo',
        'php7.2-fpm',
        '100',
    ]


def test_build_job_raises():
    """Ensure that php_fpm raises AttributeError if no job is set"""
    php_restart_no_job = php_fpm.PHPRestart(PHPRESTART_PARAMS)
    with pytest.raises(AttributeError):
        php_restart_no_job._build_job(['x'])


def test_get_batch_size():
    """
    test get_batch_size function
    """
    with pytest.raises(ValueError):
        php_fpm.get_batch_size([])
    assert php_fpm.get_batch_size(range(0, 1000)) == 100
    assert php_fpm.get_batch_size(range(0, 1000), percentage=20) == 200
