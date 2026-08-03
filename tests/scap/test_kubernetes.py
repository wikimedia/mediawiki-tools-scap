import io
import mock
import pytest
import subprocess

from scap.kubernetes import (
    CommandsCheck,
    DeploymentsConfig,
    InvalidDeploymentsConfig,
    LogstashCheck,
    built_image_ids,
    inspect_images,
    DepConfig,
)

deployment_configs = [
    # Correct configuration
    (
        """
    # single release, explicitly mapped to the testservers stage and selecting the debug image.
    - namespace: test
      releases:
        main:
          stage: testservers
      mw_kind: debug-image
      mw_flavour: publish
      web_flavour: webserver

    # single release, mapped to the default production stage and image kind.
    # this uses a different k8s cluster
    - namespace: api1
      releases:
        main: {}
      mw_flavour: publish
      web_flavour: webserver
      dir: anothercluster
      clusters: [another-k8s-cluster]

    # multiple releases, one mapped to the canaries stage, and another marked
    # non-deploy, which also selects the cli image kind.
    - namespace: api2
      releases:
        main: {}
        canary:
          stage: canaries
        maintenance:
          deploy: false
          mw_kind: cli-image
      mw_flavour: publish
      web_flavour: webserver

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
     """,
        {
            "testservers": [
                DepConfig(
                    namespace="test",
                    release="main",
                    cluster="default-cluster",
                    scope="train",
                    mw_image_kind="debug-image",
                    mw_image_flavour="publish",
                    web_image_flavour="webserver",
                    deploy=True,
                    fq_release_name="test-main-default-cluster",
                    values_file="/helmfile-releases/test-main-default-cluster.yaml",
                    helmfile_dir="/srv/deployment-charts/helmfile.d/default/test",
                ),
            ],
            "canaries": [
                DepConfig(
                    namespace="api2",
                    release="canary",
                    cluster="default-cluster",
                    scope="train",
                    mw_image_kind=None,
                    mw_image_flavour="publish",
                    web_image_flavour="webserver",
                    deploy=True,
                    fq_release_name="api2-canary-default-cluster",
                    values_file="/helmfile-releases/api2-canary-default-cluster.yaml",
                    helmfile_dir="/srv/deployment-charts/helmfile.d/default/api2",
                ),
                DepConfig(
                    namespace="api3",
                    release="canary",
                    cluster="default-cluster",
                    scope="train",
                    mw_image_kind=None,
                    mw_image_flavour="publish",
                    web_image_flavour="webserver",
                    deploy=True,
                    fq_release_name="api3-canary-default-cluster",
                    values_file="/helmfile-releases/api3-canary-default-cluster.yaml",
                    helmfile_dir="/srv/deployment-charts/helmfile.d/default/api3",
                ),
            ],
            "production": [
                DepConfig(
                    namespace="api1",
                    release="main",
                    cluster="another-k8s-cluster",
                    scope="train",
                    mw_image_kind=None,
                    mw_image_flavour="publish",
                    web_image_flavour="webserver",
                    deploy=True,
                    fq_release_name="api1-main-another-k8s-cluster",
                    values_file="/helmfile-releases/api1-main-another-k8s-cluster.yaml",
                    helmfile_dir="/srv/deployment-charts/helmfile.d/anothercluster/api1",
                ),
                DepConfig(
                    namespace="api2",
                    release="main",
                    cluster="default-cluster",
                    scope="train",
                    mw_image_kind=None,
                    mw_image_flavour="publish",
                    web_image_flavour="webserver",
                    deploy=True,
                    fq_release_name="api2-main-default-cluster",
                    values_file="/helmfile-releases/api2-main-default-cluster.yaml",
                    helmfile_dir="/srv/deployment-charts/helmfile.d/default/api2",
                ),
                DepConfig(
                    namespace="api2",
                    release="maintenance",
                    cluster="default-cluster",
                    scope="train",
                    mw_image_kind="cli-image",
                    mw_image_flavour="publish",
                    web_image_flavour="webserver",
                    deploy=False,
                    fq_release_name="api2-maintenance-default-cluster",
                    values_file="/helmfile-releases/api2-maintenance-default-cluster.yaml",
                    helmfile_dir="/srv/deployment-charts/helmfile.d/default/api2",
                ),
                DepConfig(
                    namespace="api3",
                    release="main",
                    cluster="default-cluster",
                    scope="train",
                    mw_image_kind=None,
                    mw_image_flavour="publish",
                    web_image_flavour="webserver",
                    deploy=True,
                    fq_release_name="api3-main-default-cluster",
                    values_file="/helmfile-releases/api3-main-default-cluster.yaml",
                    helmfile_dir="/srv/deployment-charts/helmfile.d/default/api3",
                ),
                DepConfig(
                    namespace="api3",
                    release="migration",
                    cluster="default-cluster",
                    scope="train",
                    mw_image_kind=None,
                    mw_image_flavour="exciting-new-mediawiki",
                    web_image_flavour="exciting-new-webserver",
                    deploy=True,
                    fq_release_name="api3-migration-default-cluster",
                    values_file="/helmfile-releases/api3-migration-default-cluster.yaml",
                    helmfile_dir="/srv/deployment-charts/helmfile.d/default/api3",
                ),
            ],
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

    - namespace: api
      releases:
        main: {}
      mw_flavour: publish
      web_flavour: webserver
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
     """,
        None,
    ),
]


@pytest.mark.parametrize("config,expected_parse", deployment_configs)
def test_deployments_config_parser(config, expected_parse):
    app = mock.MagicMock()
    app.config = {
        "k8s_deployments_file": "mocked",
        "helmfile_mediawiki_release_dir": "/helmfile-releases",
        "helmfile_deployments_dir": "/srv/deployment-charts/helmfile.d",
        "helmfile_default_cluster_dir": "default",
    }

    with mock.patch("builtins.open", mock.mock_open(read_data=config)):
        if not expected_parse:
            with pytest.raises(InvalidDeploymentsConfig):
                DeploymentsConfig.parse(app, ["default-cluster"])
        else:
            parsed_config = DeploymentsConfig.parse(app, ["default-cluster"])
            assert parsed_config.stages == expected_parse


def _parse(config, scope=None):
    app = mock.MagicMock()
    app.config = {
        "k8s_deployments_file": "mocked",
        "helmfile_mediawiki_release_dir": "/helmfile-releases",
        "helmfile_deployments_dir": "/srv/deployment-charts/helmfile.d",
        "helmfile_default_cluster_dir": "default",
    }
    with mock.patch("builtins.open", mock.mock_open(read_data=config)):
        return DeploymentsConfig.parse(app, ["default-cluster"], scope)


# A config in the mapping format, exercising scopes and supervision rules.
MAP_CONFIG = """
deployment_targets:
  - namespace: mw-pretrain
    scope: pretrain
    releases:
      web-canary:
        stage: canaries
      web:
        stage: production
    mw_flavour: publish
    web_flavour: webserver
  - namespace: mw-web
    scope: train
    releases:
      canary:
        stage: canaries
      main: {}
    mw_flavour: publish
    web_flavour: webserver
  - namespace: mw-legacy
    # No scope: defaults to "train".
    releases:
      main: {}
    mw_flavour: publish
    web_flavour: webserver

supervision_rules:
  - scope: pretrain
    stages:
      testservers:
        - commands:
            - "httpbb --hosts=mwdebug-pretrain.discovery.wmnet"
      canaries:
        - logstash_check: {threshold: 5}
        - logstash_check: {scope: train, stage: production, threshold: 150}
      production:
        - logstash_check: {threshold: 20}
        - logstash_check: {scope: train, stage: production, threshold: 150}
  - scope: train
    stages:
      testservers:
        - commands:
            - "httpbb --hosts=mwdebug.discovery.wmnet"
            - "httpbb --hosts=mwdebug-next.discovery.wmnet"
      canaries:
        - logstash_check: {threshold: 10}
      production:
        - logstash_check: {threshold: 150}
"""


def test_map_format_scopes():
    parsed = _parse(MAP_CONFIG)

    # An absent scope defaults to "train".
    scopes = {dc.namespace: dc.scope for dc in parsed.stages["production"]}
    assert scopes == {
        "mw-pretrain": "pretrain",
        "mw-web": "train",
        "mw-legacy": "train",
    }


def test_map_format_supervision_rules_parsed():
    parsed = _parse(MAP_CONFIG)

    assert parsed.supervision_rules is not None
    rules = {rule.scope: rule for rule in parsed.supervision_rules}
    assert set(rules) == {"pretrain", "train"}

    pretrain_canaries = rules["pretrain"].stages["canaries"]
    assert all(isinstance(c, LogstashCheck) for c in pretrain_canaries)
    # A bare check inherits scope/stage from context (left None on the check).
    assert (pretrain_canaries[0].threshold, pretrain_canaries[0].scope) == (5, None)
    # The guardrail check supervises a different scope and stage.
    assert pretrain_canaries[1].scope == "train"
    assert pretrain_canaries[1].stage == "production"
    assert pretrain_canaries[1].threshold == 150

    # A commands check is parsed into a CommandsCheck.
    (testservers_check,) = rules["train"].stages["testservers"]
    assert isinstance(testservers_check, CommandsCheck)
    assert testservers_check.commands == [
        "httpbb --hosts=mwdebug.discovery.wmnet",
        "httpbb --hosts=mwdebug-next.discovery.wmnet",
    ]


def test_legacy_format_has_no_supervision_rules():
    # A bare list (legacy format) yields no supervision rules and all scopes.
    parsed = _parse(
        """
        - namespace: api
          releases:
            main: {}
          mw_flavour: publish
          web_flavour: webserver
        """
    )
    assert parsed.supervision_rules is None
    assert parsed.scope_filter is None
    assert parsed.scopes_deployed() == {"train"}


def test_scope_filters_deployed_targets():
    parsed = _parse(MAP_CONFIG, scope={"pretrain"})

    assert parsed.scope_filter == {"pretrain"}
    assert parsed.scopes_deployed() == {"pretrain"}

    # Deploy operations only see pretrain targets ...
    deployed = parsed.deployed_stage_dep_configs("production")
    assert {dc.namespace for dc in deployed} == {"mw-pretrain"}

    # ... but supervision can still cover the (untouched) train fleet.
    supervised_train = parsed.supervised_dep_configs("train", "production")
    assert {dc.namespace for dc in supervised_train} == {"mw-web", "mw-legacy"}


def test_resolve_logstash_checks_inherits_context():
    parsed = _parse(MAP_CONFIG)

    # A bare {threshold} inherits the rule's scope and the current stage; a check
    # with explicit scope/stage does not. Both pretrain rules resolve here (all
    # scopes deployed), so the guardrail appears once (de-duplicated). Returned
    # checks have inheritance applied, so scope/stage are always concrete.
    assert parsed.resolve_logstash_checks("canaries") == [
        LogstashCheck(threshold=5, scope="pretrain", stage="canaries"),
        LogstashCheck(threshold=150, scope="train", stage="production"),
        LogstashCheck(threshold=10, scope="train", stage="canaries"),
    ]
    assert parsed.resolve_logstash_checks("production") == [
        LogstashCheck(threshold=20, scope="pretrain", stage="production"),
        LogstashCheck(threshold=150, scope="train", stage="production"),
    ]


def test_resolve_logstash_checks_respects_deployed_scope():
    # With --scope pretrain, only the pretrain rule fires, but its guardrail still
    # resolves to the (untouched) train production fleet.
    parsed = _parse(MAP_CONFIG, scope={"pretrain"})
    assert parsed.resolve_logstash_checks("canaries") == [
        LogstashCheck(threshold=5, scope="pretrain", stage="canaries"),
        LogstashCheck(threshold=150, scope="train", stage="production"),
    ]


def test_resolve_logstash_checks_without_rules():
    parsed = _parse(
        """
        - namespace: api
          releases:
            main: {}
          mw_flavour: publish
          web_flavour: webserver
        """
    )
    assert parsed.resolve_logstash_checks("production") == []
    assert parsed.resolve_command_checks("testservers") == []


def test_resolve_command_checks():
    parsed = _parse(MAP_CONFIG)
    # Union of both rules' testservers commands, in rule order.
    assert parsed.resolve_command_checks("testservers") == [
        "httpbb --hosts=mwdebug-pretrain.discovery.wmnet",
        "httpbb --hosts=mwdebug.discovery.wmnet",
        "httpbb --hosts=mwdebug-next.discovery.wmnet",
    ]
    # Stages without commands yield none.
    assert parsed.resolve_command_checks("canaries") == []


def test_resolve_command_checks_respects_deployed_scope():
    parsed = _parse(MAP_CONFIG, scope={"pretrain"})
    assert parsed.resolve_command_checks("testservers") == [
        "httpbb --hosts=mwdebug-pretrain.discovery.wmnet",
    ]


def test_invalid_deployments_config_suppresses_backtrace(monkeypatch):
    # A config error is user-facing and should not print a stack trace ...
    monkeypatch.delenv("SCAP_BACKTRACE", raising=False)
    assert InvalidDeploymentsConfig("bad").__dict__["_scap_no_backtrace"] is True

    # ... unless SCAP_BACKTRACE is set, for debugging.
    monkeypatch.setenv("SCAP_BACKTRACE", "1")
    assert InvalidDeploymentsConfig("bad").__dict__["_scap_no_backtrace"] is False


def test_multiple_scopes():
    # --scope accepts more than one scope; all named scopes are deployed.
    parsed = _parse(MAP_CONFIG, scope={"pretrain", "train"})
    assert parsed.scope_filter == {"pretrain", "train"}
    assert {dc.namespace for dc in parsed.deployed_stage_dep_configs("production")} == {
        "mw-pretrain",
        "mw-web",
        "mw-legacy",
    }


def test_unknown_scope_is_rejected():
    with pytest.raises(InvalidDeploymentsConfig):
        _parse(MAP_CONFIG, scope={"nonexistent"})


def test_unknown_scope_in_set_is_rejected():
    # A valid scope mixed with an unknown one still fails, naming the bad one.
    with pytest.raises(InvalidDeploymentsConfig):
        _parse(MAP_CONFIG, scope={"pretrain", "nope"})


def test_map_format_requires_deployment_targets():
    with pytest.raises(InvalidDeploymentsConfig):
        _parse("supervision_rules: []\n")


@pytest.mark.parametrize(
    "rules",
    [
        # Unsupported stage name
        "supervision_rules:\n  - scope: train\n    stages:\n      staging:\n        - logstash_check: {threshold: 1}\n",
        # A check naming more than one type
        "supervision_rules:\n  - scope: train\n    stages:\n      canaries:\n        - logstash_check: {threshold: 1}\n          commands: [x]\n",
        # Unknown check type
        "supervision_rules:\n  - scope: train\n    stages:\n      canaries:\n        - bogus_check: {threshold: 1}\n",
        # Missing scope
        "supervision_rules:\n  - stages:\n      canaries:\n        - logstash_check: {threshold: 1}\n",
        # logstash_check missing a threshold
        "supervision_rules:\n  - scope: train\n    stages:\n      canaries:\n        - logstash_check: {scope: train}\n",
        # commands must be a list of strings, not a mapping
        "supervision_rules:\n  - scope: train\n    stages:\n      testservers:\n        - commands: {cmd: x}\n",
        # logstash_check is not supported at the testservers stage
        "supervision_rules:\n  - scope: train\n    stages:\n      testservers:\n        - logstash_check: {threshold: 1}\n",
    ],
)
def test_invalid_supervision_rules_rejected(rules):
    config = (
        "deployment_targets:\n"
        "  - namespace: mw-web\n"
        "    releases:\n"
        "      main: {}\n"
        "    mw_flavour: publish\n"
        "    web_flavour: webserver\n" + rules
    )
    with pytest.raises(InvalidDeploymentsConfig):
        _parse(config)


@mock.patch("subprocess.Popen")
def test_built_image_ids(mock_popen):
    mock_popen.return_value.__enter__.return_value.stdout = io.StringIO(
        "a0a0\n" "b0b0\n" "b0b0\n"
    )

    assert sorted(built_image_ids()) == ["a0a0", "b0b0"]

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
