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
    # Correct configuration: T299648 format.
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
    # Correct configuration: mixed T299648 and T370934 format.
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
      releases:
        # main uses the top-level defaults.
        main: {}
        # migration overrides the image flavors to something exciting.
        migration:
          mw_flavour: exciting-new-mediawiki
          web_flavour: exciting-new-webserver
        # more-canaries also uses the top-level defaults, and is updated in the
        # canaries stage.
        more-canaries:
          stage: canaries
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
                },
                {
                    "namespace": "api2",
                    "release": "more-canaries",
                    "mv_image_fl": "publish",
                    "web_image_fl": "webserver",
                    "debug": False,
                },
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
                {
                    "namespace": "api2",
                    "release": "migration",
                    "mv_image_fl": "exciting-new-mediawiki",
                    "web_image_fl": "exciting-new-webserver",
                    "debug": False,
                },
            ],
        },
    ),
    # Correct-but-weird: Canary releases in debug namespaces are ignored.
    (
        """
    - namespace: testservers
      release: main
      canary: canary
      mw_flavour: publish
      web_flavour: webserver
      debug: true
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
            "canaries": [],
            "production": [],
        },
    ),
    # Correct-but-weird: Canary releases in debug namespaces are ignored (T370934 format).
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
                    "mv_image_fl": "publish",
                    "web_image_fl": "webserver",
                    "debug": True,
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
    # Incorrect: Specifies both release and releases.
    (
        """
    - namespace: api
      release: main
      canary: canaries
      mw_flavour: publish
      web_flavour: webserver
      releases:
        more-canaries:
          stage: canaries
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
