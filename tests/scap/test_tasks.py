from datetime import datetime, timedelta
from unittest import mock

import pytest

from scap import tasks


def test_get_old_wikiversions():
    now = datetime.utcnow()

    one_week_ago = now - timedelta(weeks=1)
    two_weeks_ago = now - timedelta(weeks=2)
    three_weeks_ago = now - timedelta(weeks=5)
    ten_weeks_ago = now - timedelta(weeks=10)

    # What is expected is that the 2 new versions (wmf.4 & wmf.5) will
    # be kept, regardless of when we determined the branching to happen.
    #
    # Of the remaining versions, 2 are within a period of time where we
    # would like to keep their static assets (wmf.2 & wmf.3).  The
    # remaining version is 10 weeks old, that should be removed entirely
    versions = [
        ("php-1.29.0-wmf.4", now),
        ("php-1.29.0-wmf.3", one_week_ago),
        ("php-1.29.0-wmf.2", two_weeks_ago),
        ("php-1.29.0-wmf.1", ten_weeks_ago),
        ("php-1.29.0-wmf.5", three_weeks_ago),
    ]

    remove, remove_static = tasks.get_old_wikiversions(versions)

    assert remove == ["php-1.29.0-wmf.1"]

    # Returns versions to remove in reverse order
    assert remove_static == ["php-1.29.0-wmf.3", "php-1.29.0-wmf.2"]


@pytest.mark.parametrize(
    "action",
    [
        "",
        "=reload",
        "=restart",
        "=reload:disable-secondary",
        "=restart:disable-secondary",
    ],
)
@pytest.mark.parametrize("on_secondary_host", [False, True])
@mock.patch("subprocess.check_call")
def test_handle_services(service_disable_call, on_secondary_host, action):
    tasks.handle_services(
        f"at_your_service{action}", on_secondary_host=on_secondary_host
    )

    # Default is restarting the service
    if action == "" or action == "=restart":
        service_disable_call.called_with(
            "sudo -n /usr/sbin/service at_your_service restart".split()
        )
    if action == "=reload":
        service_disable_call.called_with(
            "sudo -n /usr/sbin/service at_your_service reload".split()
        )
    if action == "=reload:disable-secondary":
        if on_secondary_host:
            service_disable_call.assert_has_calls(
                [
                    mock.call("sudo -n /usr/sbin/service at_your_service stop".split()),
                    mock.call("sudo -n /bin/systemctl disable at_your_service".split()),
                ]
            )
        else:
            service_disable_call.called_with(
                "sudo -n /usr/sbin/service at_your_service reload".split()
            )
    if action == "=restart:disable-secondary":
        if on_secondary_host:
            service_disable_call.assert_has_calls(
                [
                    mock.call("sudo -n /usr/sbin/service at_your_service stop".split()),
                    mock.call("sudo -n /bin/systemctl disable at_your_service".split()),
                ]
            )
        else:
            service_disable_call.called_with(
                "sudo -n /usr/sbin/service at_your_service restart".split()
            )
