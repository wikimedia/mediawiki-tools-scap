from __future__ import absolute_import


try:
    import mock
except ImportError:
    from unittest import mock

import requests

import pytest

from scap import opcache_manager, targets


@pytest.fixture
def om():
    o = opcache_manager.OpcacheManager(777)
    return o


def req(mocker, status, side_effect=None):
    mocker.patch('requests.get')
    resp = requests.Response()
    resp.status_code = status
    requests.get.return_value = resp
    requests.get.side_effect = side_effect


def test_init(om):
    """Test opcache manager initialization"""
    assert om.admin_port == 777


def test_invalidate_host_no_file(om, mocker):
    """Test a request is made, and what happens when the response is ok """
    # The remote server responded with 200 OK
    req(mocker, 200)
    host = 'foo1.example.com'
    assert om._invalidate_host(host, None) == (True, None)
    requests.get.assert_called_with('http://foo1.example.com:777/opcache-free',
                                    params={}, timeout=om.TIMEOUT)


def test_invalidate_host_file(om, mocker):
    """Test that a request is made, and the response is not ok"""
    req(mocker, 500)
    requests.get.return_value.reason = 'fuzzbuzz'

    host = 'foo1.example.com'
    assert om._invalidate_host(host, 'testfile') == \
        (False,
         "Response returned error 500 Server Error: fuzzbuzz for url: None")
    requests.get.assert_called_with(
        'http://foo1.example.com:777/opcache-free',
        params={'file': 'testfile'}, timeout=om.TIMEOUT)


def test_invalidate_timeout(om, mocker):
    req(mocker, None, side_effect=requests.exceptions.Timeout('fail'))
    assert om._invalidate_host('testurl', None) == \
        (False, 'A timeout happened before a response was received')


def test_invalidate_host_exception(om, mocker):
    """Test what happens when an exception is raised."""
    req(mocker, None, side_effect=ValueError('test test'))
    assert om._invalidate_host('testurl', None) == (False, 'test test')


def invalidate_results(url, file):
    if url.find('host3') >= 0:
        return (False, "test test")
    return (True, None)


def test_invalidate(om):
    """Test that invalidate works as expected"""
    # Let's make the timeout for the threads very short for this test
    om._invalidate_host = mock.MagicMock(side_effect=invalidate_results)
    assert om.invalidate(['host1', 'host2', 'host3'], None) == \
        {'host3': 'test test'}
    # Check the threadpool did call all the function it should have.
    om._invalidate_host.assert_has_calls(
        [mock.call('host1', None),
         mock.call('host2', None),
         mock.call('host3', None)],
        any_order=True
    )


def test_invalidate_all(om, mocker):
    """Test that global invalidation works as expected."""
    config = {'dsh_targets': 'mediawiki-installation'}
    om.invalidate = mock.MagicMock()
    target_list_class = mocker.patch('scap.targets.DirectDshTargetList')
    target_list = target_list_class.return_value
    # Simulate no answer received
    target_list.all.__nonzero__.return_value = 0
    assert om.invalidate_all(config) == {}
    target_list_class.assert_called_with('mw_web_clusters', config)
    assert target_list.primary_key == 'dsh_targets'
    target_list.reset_mock()
    target_list.groups = {
        'cl1': targets.DeployGroup('cl1', ['host1', 'host2']),
        'cl2': targets.DeployGroup('cl2', ['host4', 'host3']),
    }
    config['mw_web_clusters'] = 'cl1,cl2, cl3'
    assert om.invalidate_all(config, 'test.php') == {}
    om.invalidate.assert_any_call(['host1', 'host2'], 'test.php')
    om.invalidate.assert_any_call(['host4', 'host3'], 'test.php')
