from unittest import TestCase

import io
import mock
import pytest
import subprocess

from scap.kubernetes import (
    DeploymentsConfig,
    InvalidDeploymentsConfig,
    CANARIES,
    PRODUCTION,
    TEST_SERVERS,
    built_image_ids,
    inspect_images,
)

deployment_configs = [
    # Correct configuration
    (
        """
    # single release, mapped to the testservers stage via debug: true.
    - namespace: test
      releases:
        main: {}
      mw_flavour: publish
      web_flavour: webserver
      debug: true

    # single release, mapped to the default production stage.
    - namespace: api1
      releases:
        main: {}
      mw_flavour: publish
      web_flavour: webserver
      debug: false

    # multiple releases, one mapped to the canaries stage, and another marked
    # non-deploy.
    - namespace: api2
      releases:
        main: {}
        canary:
          stage: canaries
        maintenance:
          deploy: false
      mw_flavour: publish
      web_flavour: webserver
      debug: false

    # multiple releases, one overriding the image flavours used.
    - namespace: api3
      releases:
        # main and canary use the top-level defaults.
        main: {}
        canary:
          stage: canaries
        # migration overrides the image flavours to something exciting.
        migration:
          mw_flavour: exciting-new-mediawiki
          web_flavour: exciting-new-webserver
      mw_flavour: publish
      web_flavour: webserver
      debug: false
     """,
        {
            "testservers": [
                {
                    "namespace": "test",
                    "release": "main",
                    "mw_image_fl": "publish",
                    "web_image_fl": "webserver",
                    "debug": True,
                    "deploy": True,
                }
            ],
            "canaries": [
                {
                    "namespace": "api2",
                    "release": "canary",
                    "mw_image_fl": "publish",
                    "web_image_fl": "webserver",
                    "debug": False,
                    "deploy": True,
                },
                {
                    "namespace": "api3",
                    "release": "canary",
                    "mw_image_fl": "publish",
                    "web_image_fl": "webserver",
                    "debug": False,
                    "deploy": True,
                },
            ],
            "production": [
                {
                    "namespace": "api1",
                    "release": "main",
                    "mw_image_fl": "publish",
                    "web_image_fl": "webserver",
                    "debug": False,
                    "deploy": True,
                },
                {
                    "namespace": "api2",
                    "release": "main",
                    "mw_image_fl": "publish",
                    "web_image_fl": "webserver",
                    "debug": False,
                    "deploy": True,
                },
                {
                    "namespace": "api2",
                    "release": "maintenance",
                    "mw_image_fl": "publish",
                    "web_image_fl": "webserver",
                    "debug": False,
                    "deploy": False,
                },
                {
                    "namespace": "api3",
                    "release": "main",
                    "mw_image_fl": "publish",
                    "web_image_fl": "webserver",
                    "debug": False,
                    "deploy": True,
                },
                {
                    "namespace": "api3",
                    "release": "migration",
                    "mw_image_fl": "exciting-new-mediawiki",
                    "web_image_fl": "exciting-new-webserver",
                    "debug": False,
                    "deploy": True,
                },
            ],
        },
    ),
    # Correct-but-weird: Canary releases in debug namespaces are ignored.
    (
        """
    - namespace: testservers
      releases:
        main: {}
        canary:
          stage: canaries
      mw_flavour: publish
      web_flavour: webserver
      debug: true
    """,
        {
            "testservers": [
                {
                    "namespace": "testservers",
                    "release": "main",
                    "mw_image_fl": "publish",
                    "web_image_fl": "webserver",
                    "debug": True,
                    "deploy": True,
                }
            ],
            "canaries": [],
            "production": [],
        },
    ),
    # Incorrect: Same namespace duplicated
    (
        """
    - namespace: api
      releases:
        main: {}
        canary:
          stage: canaries
      mw_flavour: publish
      web_flavour: webserver
      debug: false

    - namespace: api
      releases:
        main: {}
      mw_flavour: publish
      web_flavour: webserver
      debug: false
     """,
        None,
    ),
    # Incorrect: mw_flavour is unspecified.
    (
        """
    - namespace: api
      releases:
        main:
          web_flavour: webserver
      debug: false
     """,
        None,
    ),
    # Incorrect: web_flavour is unspecified.
    (
        """
    - namespace: api
      releases:
        main:
          mw_flavour: publish
      debug: false
     """,
        None,
    ),
    # Incorrect: unsupported stage override
    (
        """
    - namespace: api
      releases:
        main:
          stage: staging  # not a real stage
      mw_flavour: publish
      web_flavour: webserver
      debug: false
     """,
        None,
    ),
]


@pytest.mark.parametrize("config,expected_parse", deployment_configs)
def test_deployments_config_parser(config, expected_parse):
    def sort_key(config_entry: dict) -> str:
        return f"{config_entry[DeploymentsConfig.NAMESPACE]}/{config_entry[DeploymentsConfig.RELEASE]}"

    def sort_lists(stages: dict):
        stages[TEST_SERVERS] = sorted(stages[TEST_SERVERS], key=sort_key)
        stages[CANARIES] = sorted(stages[CANARIES], key=sort_key)
        stages[PRODUCTION] = sorted(stages[PRODUCTION], key=sort_key)

    with mock.patch("builtins.open", mock.mock_open(read_data=config)):
        if not expected_parse:
            with pytest.raises(InvalidDeploymentsConfig):
                DeploymentsConfig.parse("ignored")
        else:
            parsed_config = DeploymentsConfig.parse("ignored")
            sort_lists(expected_parse)
            sort_lists(parsed_config.stages)
            TestCase().assertDictEqual(parsed_config.stages, expected_parse)


@mock.patch("subprocess.Popen")
def test_built_image_ids(mock_popen):
    mock_popen.return_value.__enter__.return_value.stdout = io.StringIO(
        "a0a0\n" "b0b0\n" "b0b0\n"
    )

    assert built_image_ids() == {"a0a0", "b0b0"}

    mock_popen.assert_called_with(
        [
            "docker",
            "image",
            "ls",
            "--filter",
            "label=vnd.wikimedia.builder.name=scap",
            "--format",
            "{{.ID}}",
        ],
        stdout=subprocess.PIPE,
        text=True,
    )


@mock.patch("subprocess.Popen")
def test_inspect_images(mock_popen):
    mock_popen.return_value.__enter__.return_value.stdout = io.StringIO(
        """[{
        "Id": "a0a0a",
        "RepoTags": [
            "foo/foo:v0"
        ],
        "Config": {
            "Labels": {
                "foo": "bar"
            }
        }
    }]"""
    )

    assert inspect_images(["a0a0"]) == [
        {
            "Id": "a0a0a",
            "RepoTags": ["foo/foo:v0"],
            "Config": {
                "Labels": {
                    "foo": "bar",
                },
            },
        }
    ]

    mock_popen.assert_called_with(
        [
            "docker",
            "image",
            "inspect",
            "a0a0",
        ],
        stdout=subprocess.PIPE,
    )
