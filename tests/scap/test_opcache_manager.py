from __future__ import absolute_import

try:
    import mock
except ImportError:
    from unittest import mock

import time

import pytest
import urllib3

from scap import opcache_manager


@pytest.fixture
def om():
    o = opcache_manager.OpcacheManager()
    o.http.request = mock.MagicMock()
    return o


def test_init(om):
    """Test opcache manager initialization"""
    assert om.config == {}
    assert isinstance(om.http, urllib3.PoolManager)


def test_invalidate_host_no_file(om):
    """Test a request is made, and what happens when the response is ok """
    # The remote server responded with 200 OK
    resp = om.http.request.return_value
    resp.status = 200
    url = 'http://test.example.com:9090/mytest'
    assert om._invalidate_host(url, None) == (True, None)
    om.http.request.assert_called_with('GET', url, fields={})


def test_invalidate_host_file(om):
    """Test that a request is made, and the response is not ok"""
    resp = om.http.request.return_value
    resp.status = 500
    url = 'http://test.example.com:9090/mytest'
    assert om._invalidate_host(url, 'testfile') == \
        (False, "Response returned status 500")
    om.http.request.assert_called_with('GET', url, fields={'file': 'testfile'})


def test_invalidate_host_exception(om):
    """Test what happens when an exception is raised."""
    om.http.request.side_effect = ValueError('test test')
    assert om._invalidate_host('testurl', None) == (False, 'test test')


def invalidate_results(url, file):
    if url.find('host3') >= 0:
        return (False, "test test")
    if url.find('timeout') >= 0:
        time.sleep(0.2)
    return (True, None)


def test_invalidate(om):
    """Test that invalidate works as expected"""
    # Let's make the timeout for the greenlets very short for this test
    om.TIMEOUT = 0.1
    om._invalidate_host = mock.MagicMock(side_effect=invalidate_results)
    assert om.invalidate(['host1', 'host2', 'host3'], 9005, None) == \
        {'host3': 'test test'}
    assert om.invalidate(['timeout'], 9005, None) == \
        {'timeout': "A timeout happened before a response was received"}


def test_invalidate_all(om):
    """Test that global invalidation works as expected."""
    om.config = {'dsh-targets': 'mediawiki-installation'}
    om._invalidate_host = mock.MagicMock(side_effect=invalidate_results)
    with mock.patch('scap.targets.get') as mocker:
        mocker.all.return_value = ['host1']
        assert om.invalidate_all(9005) == {}
        mocker.assert_called_with('mediawiki-installation', om.config)
        mocker.reset_mock()
        om.config['mw_web_clusters'] = 'cl1,cl2, cl3'
        assert om.invalidate_all(9005) == {}
        mocker.assert_called_with('cl3', om.config)
