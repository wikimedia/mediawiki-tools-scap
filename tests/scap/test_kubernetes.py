import io
import mock
import pytest
import subprocess

from scap.kubernetes import (
    DeploymentsConfig,
    InvalidDeploymentsConfig,
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
