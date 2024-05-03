from unittest import mock

import os
import pytest

from scap import tasks


def test_get_wikiversions_ondisk(tmpdir):
    for dir in [
        "php-master",
        "php-1.40.0-wmf.4",
        "php-1.29.0-wmf.3",
        "php-1.29.0-wmf.2",
    ]:
        os.mkdir(os.path.join(tmpdir, dir))
    open(tmpdir / "php-1.40.0-wmf.2", "w").close()
    assert tasks.get_wikiversions_ondisk(tmpdir) == [
        "1.29.0-wmf.2",
        "1.29.0-wmf.3",
        "1.40.0-wmf.4",
        "master",
    ]


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
