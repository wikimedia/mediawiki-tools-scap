import json
import tempfile
from logging import Logger
from unittest import mock
from unittest.mock import patch

import pytest
import requests
from requests import HTTPError, Response

from scap.deploy_promote import DeployPromote

messages_tests = [
    (
        "T777",
        "group3 to 1.42.0-wmf.00  refs T777",
        ("group3 to 1.42.0-wmf.00\n" "\n" "Bug: T777"),
    ),
]


@pytest.fixture
@patch.object(DeployPromote, "__init__", return_value=None)
def deploy_promote(init):
    dp = DeployPromote()
    dp.config = {}
    return dp


@pytest.mark.parametrize("task,announce,commit", messages_tests)
def test_set_messages(task, announce, commit, deploy_promote):
    p = get_deploy_promote_with_messages(task, deploy_promote)

    assert p.announce_message == announce
    assert p.commit_message == commit
    assert "\n" not in p.announce_message


def get_deploy_promote_with_messages(task, p):
    version = "1.42.0-wmf.00"
    train_info = {
        "version": version,
        "task_id": task,
        "status": "open",
        "date": "2025-07-15",
    }

    with tempfile.NamedTemporaryFile(mode="w") as f:
        json.dump(train_info, f)
        f.flush()

        p.config["train_blockers_url"] = "file://{}".format(f.name)
        p.config["web_proxy"] = None
        p.group = "group3"
        p.promote_version = version

        p._set_messages()

    return p


def test_version_check(deploy_promote):
    deploy_promote.logger = mock.MagicMock(Logger)
    deploy_promote.promote_version = "1.39.0-wmf.19"

    with mock.patch.object(requests, "get") as mock_get:
        mock_get.return_value = mock.MagicMock(Response)

        # Set the check versions timeout to zero so that these tests will complete quickly.
        with mock.patch.object(
            deploy_promote, "_get_check_versions_timeout", return_value=0
        ):
            # Version matches
            mock_get.return_value.text = (
                '<meta name="generator" content="MediaWiki 1.39.0-wmf.19"/>'
            )
            deploy_promote._check_versions()

            # Version does not match
            mock_get.return_value.text = (
                '<meta name="generator" content="MediaWiki NoVersTooBad"/>'
            )
            with pytest.raises(SystemExit):
                deploy_promote._check_versions()

            # Version could not be found in page
            mock_get.return_value.text = "garbled nonsense dadadddd"
            with pytest.raises(SystemExit):
                deploy_promote._check_versions()

            # Request failed
            with mock.patch.object(
                mock_get.return_value, "raise_for_status"
            ) as mock_raise:
                http_error = HTTPError()
                http_error.response = mock.MagicMock(Response)
                http_error.response.status_code = 500
                mock_raise.side_effect = http_error

                with pytest.raises(SystemExit):
                    deploy_promote._check_versions()
