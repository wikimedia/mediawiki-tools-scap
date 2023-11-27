from unittest import TestCase

import mock
import pytest

from scap.kubernetes import (
    DeploymentsConfig,
    InvalidDeploymentsConfig,
    CANARIES,
    PRODUCTION,
    TEST_SERVERS,
)

deployment_configs = [
    # Correct configuration
    (
        """
    - namespace: testservers
      release: main
      canary:
      mw_flavour: publish
      web_flavour: webserver
      debug: true

    - namespace: api1
      release: main
      canary: canaries
      mw_flavour: publish
      web_flavour: webserver
      debug: false

    - namespace: api2
      release: main
      canary:
      mw_flavour: publish
      web_flavour: webserver
      debug: false
     """,
        {
            "testservers": [
                {
                    "namespace": "testservers",
                    "release": "main",
                    "mv_image_fl": "publish",
                    "web_image_fl": "webserver",
                    "debug": True,
                }
            ],
            "canaries": [
                {
                    "namespace": "api1",
                    "release": "canaries",
                    "mv_image_fl": "publish",
                    "web_image_fl": "webserver",
                    "debug": False,
                }
            ],
            "production": [
                {
                    "namespace": "api1",
                    "release": "main",
                    "mv_image_fl": "publish",
                    "web_image_fl": "webserver",
                    "debug": False,
                },
                {
                    "namespace": "api2",
                    "release": "main",
                    "mv_image_fl": "publish",
                    "web_image_fl": "webserver",
                    "debug": False,
                },
            ],
        },
    ),
    # Incorrect: Same namespace duplicated
    (
        """
    - namespace: api
      release: main
      canary: canaries
      mw_flavour: publish
      web_flavour: webserver
      debug: false

    - namespace: api
      release: main
      canary:
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
        return config_entry[DeploymentsConfig.NAMESPACE]

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
