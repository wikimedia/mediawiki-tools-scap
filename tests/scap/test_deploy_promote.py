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


DEPLOYMENT_INFO = {
    "common": [
        {
            "repo": "https://gerrit.example/mediawiki-config",
            "branch": "master",
            "commit_ref": "abc123",
        }
    ],
    "versions": {
        "1.39.0-wmf.19": [
            {
                "repo": "https://gerrit.example/mediawiki/core",
                "branch": "wmf/1.39.0-wmf.19",
                "commit_ref": "def456",
            },
            {
                "repo": "https://gerrit.example/a-library",
                "commit_ref": "789abc",
            },
        ]
    },
}


def wiki_response(version="1.39.0-wmf.19"):
    return {
        "dbname": "testwiki",
        "version": version,
        "branch": "wmf/%s" % version,
        "checkouts": DEPLOYMENT_INFO["common"]
        + DEPLOYMENT_INFO["versions"]["1.39.0-wmf.19"],
    }


def test_version_check(deploy_promote, tmp_path):
    deploy_promote.logger = mock.MagicMock(Logger)
    deploy_promote.promote_version = "1.39.0-wmf.19"
    deploy_promote.config["stage_dir"] = str(tmp_path)

    with open(tmp_path / "deployment-info.json", "w") as f:
        json.dump(DEPLOYMENT_INFO, f)

    with mock.patch.object(requests, "get") as mock_get:
        mock_get.return_value = mock.MagicMock(Response)

        # Set the check versions timeout to zero so that these tests will complete quickly.
        with mock.patch.object(
            deploy_promote, "_get_check_versions_timeout", return_value=0
        ):
            # Version and checkouts match
            mock_get.return_value.json.return_value = wiki_response()
            deploy_promote._check_versions()

            # Checkouts match in any order
            response = wiki_response()
            response["checkouts"] = list(reversed(response["checkouts"]))
            mock_get.return_value.json.return_value = response
            deploy_promote._check_versions()

            # Version does not match
            mock_get.return_value.json.return_value = wiki_response("1.39.0-wmf.18")
            with pytest.raises(SystemExit):
                deploy_promote._check_versions()

            # Version matches but a checkout is on a different commit
            response = wiki_response()
            response["checkouts"] = [
                dict(response["checkouts"][0], commit_ref="0000000")
            ] + response["checkouts"][1:]
            mock_get.return_value.json.return_value = response
            with pytest.raises(SystemExit):
                deploy_promote._check_versions()

            # The wiki reports no deployment information
            mock_get.return_value.json.return_value = {}
            with pytest.raises(SystemExit):
                deploy_promote._check_versions()

            # Request failed
            with mock.patch.object(
                mock_get.return_value, "raise_for_status"
            ) as mock_raise:
                mock_raise.side_effect = HTTPError("500 Server Error")

                with pytest.raises(SystemExit):
                    deploy_promote._check_versions()


def test_version_check_without_staged_version(deploy_promote, tmp_path):
    deploy_promote.logger = mock.MagicMock(Logger)
    deploy_promote.promote_version = "1.39.0-wmf.20"
    deploy_promote.config["stage_dir"] = str(tmp_path)

    with open(tmp_path / "deployment-info.json", "w") as f:
        json.dump(DEPLOYMENT_INFO, f)

    with pytest.raises(SystemExit):
        deploy_promote._check_versions()
