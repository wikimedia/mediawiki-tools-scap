import os
import textwrap

import pytest

from scap.deploy_service import (
    InvalidDeployServiceConfig,
    load_cluster_groups,
    load_primary_datacenter,
    load_service_catalog,
    plan,
)

SERVICE_CATALOG = """
services:
  eventstreams:
    cluster_group: wikikube
    namespaces:
      eventstreams:
        environments:
          staging: {}
          codfw: {}
          eqiad: {}
  api2:
    cluster_group: dse-k8s
    namespaces:
      api2:
        environments:
          dse-k8s-eqiad: {}
"""

CLUSTER_GROUPS = """
wikikube:
  environments:
    staging-codfw:
      datacenter: codfw
      type: STAGING
    staging-eqiad:
      datacenter: eqiad
      type: STAGING
      aliases: [staging]
    codfw:
      datacenter: codfw
      type: PRODUCTION
    eqiad:
      datacenter: eqiad
      type: PRODUCTION
  environment_stages: [CANARY, DEFAULT]
  environment_default_stage: DEFAULT

dse-k8s:
  dir: dse-k8s-services
  environments:
    dse-k8s-codfw:
      datacenter: codfw
      type: PRODUCTION
    dse-k8s-eqiad:
      datacenter: eqiad
      type: PRODUCTION
  environment_stages: [CANARY, DEFAULT]
  environment_default_stage: DEFAULT
"""

CONFTOOL_STATE = """
mw:
  primary_dc: eqiad
  read_only:
    codfw: false
    eqiad: false
"""

SHELLBOX = """
cluster_group: wikikube
namespaces:
  shellbox:
    environments:
      staging: {}
      codfw:
        releases:
          canary:
            stage: CANARY
          production: {}
      eqiad:
        releases:
          canary:
            stage: CANARY
          production: {}
"""


def write(tmp_path, name, contents) -> str:
    path = os.path.join(tmp_path, name)
    with open(path, "w") as f:
        f.write(contents)
    return path


@pytest.fixture
def cluster_groups(tmp_path):
    return load_cluster_groups(
        write(tmp_path, "cluster-groups.yaml", CLUSTER_GROUPS), "services"
    )


def load(tmp_path, cluster_groups, contents, service="shellbox"):
    """Load `contents` as the config of the named service in a service catalog."""
    catalog = load_service_catalog(
        write(
            tmp_path,
            "service-catalog.yaml",
            f"services:\n  {service}:\n"
            + textwrap.indent(contents.strip("\n"), "    "),
        ),
        cluster_groups,
    )
    return catalog[service]


def commands(service_config, primary_datacenter="eqiad"):
    """Returns (directory, command) pairs, with "/" as the deployments dir."""
    return [
        (command.directory, " ".join(command.command))
        for command in plan(service_config, primary_datacenter, "/")
    ]


def test_load_cluster_groups(cluster_groups):
    assert sorted(cluster_groups) == ["dse-k8s", "wikikube"]
    group = cluster_groups["wikikube"]
    assert list(group.environments) == [
        "staging-codfw",
        "staging-eqiad",
        "codfw",
        "eqiad",
    ]
    assert group.environment_stages == ["CANARY", "DEFAULT"]
    assert group.environment_default_stage == "DEFAULT"
    assert group.dir == "services"
    assert cluster_groups["dse-k8s"].dir == "dse-k8s-services"
    assert group.resolve("staging").name == "staging-eqiad"
    assert group.resolve("codfw").type == "PRODUCTION"


def test_load_service_catalog(tmp_path, cluster_groups):
    """Every service in the catalog is parsed, and may use a different cluster group."""
    catalog = load_service_catalog(
        write(tmp_path, "service-catalog.yaml", SERVICE_CATALOG), cluster_groups
    )
    assert list(catalog) == ["eventstreams", "api2"]
    assert catalog["eventstreams"].cluster_group.name == "wikikube"
    assert catalog["api2"].cluster_group.name == "dse-k8s"
    assert commands(catalog["api2"]) == [
        ("/dse-k8s-services/api2", "helmfile apply -e dse-k8s-eqiad")
    ]


@pytest.mark.parametrize(
    "contents,expected",
    [
        ("services: {}\n", "defines no services"),
        ("services:\n  - shellbox\n", "the services of .* must be a mapping"),
        ("shellbox: {}\n", r"unexpected key\(s\): shellbox"),
        ("services:\n  shellbox:\n", "service shellbox of .* must be a mapping"),
        (
            "services:\n  shellbox:\n    dir: shellbox\n",
            r"service shellbox of .* has unexpected key\(s\): dir",
        ),
    ],
)
def test_invalid_service_catalog(tmp_path, cluster_groups, contents, expected):
    with pytest.raises(InvalidDeployServiceConfig, match=expected):
        load_service_catalog(
            write(tmp_path, "service-catalog.yaml", contents), cluster_groups
        )


def test_load_primary_datacenter(tmp_path):
    assert (
        load_primary_datacenter(write(tmp_path, "conftool.yaml", CONFTOOL_STATE))
        == "eqiad"
    )


def test_plan(tmp_path, cluster_groups):
    """Staging first, then production; within a type, stage by stage, primary datacenter last."""
    assert commands(load(tmp_path, cluster_groups, SHELLBOX)) == [
        ("/services/shellbox", "helmfile apply -e staging"),
        ("/services/shellbox", "helmfile apply -e codfw -l name=canary"),
        ("/services/shellbox", "helmfile apply -e eqiad -l name=canary"),
        ("/services/shellbox", "helmfile apply -e codfw -l name=production"),
        ("/services/shellbox", "helmfile apply -e eqiad -l name=production"),
    ]


def test_plan_namespace_selects_the_helmfile_directory(tmp_path, cluster_groups):
    service_config = load(
        tmp_path,
        cluster_groups,
        """
cluster_group: wikikube
namespaces:
  shellbox-media:
    environments:
      codfw: {}
      eqiad: {}
  shellbox-syntaxhighlight:
    environments:
      eqiad: {}
""",
    )
    assert commands(service_config) == [
        ("/services/shellbox-media", "helmfile apply -e codfw"),
        ("/services/shellbox-media", "helmfile apply -e eqiad"),
        ("/services/shellbox-syntaxhighlight", "helmfile apply -e eqiad"),
    ]


def test_plan_for_a_cluster_group_without_staging(tmp_path, cluster_groups):
    service_config = load(
        tmp_path,
        cluster_groups,
        """
cluster_group: dse-k8s
namespaces:
  api2:
    environments:
      dse-k8s-codfw:
        releases:
          canary:
            stage: CANARY
          main: {}
      dse-k8s-eqiad: {}
""",
    )
    assert commands(service_config) == [
        ("/dse-k8s-services/api2", "helmfile apply -e dse-k8s-codfw -l name=canary"),
        ("/dse-k8s-services/api2", "helmfile apply -e dse-k8s-codfw -l name=main"),
        ("/dse-k8s-services/api2", "helmfile apply -e dse-k8s-eqiad"),
    ]


def test_plan_honors_primary_datacenter(tmp_path, cluster_groups):
    service_config = load(tmp_path, cluster_groups, SHELLBOX)
    assert commands(service_config, primary_datacenter="codfw") == [
        ("/services/shellbox", "helmfile apply -e staging"),
        ("/services/shellbox", "helmfile apply -e eqiad -l name=canary"),
        ("/services/shellbox", "helmfile apply -e codfw -l name=canary"),
        ("/services/shellbox", "helmfile apply -e eqiad -l name=production"),
        ("/services/shellbox", "helmfile apply -e codfw -l name=production"),
    ]


def test_plan_honors_environment_stages_order(tmp_path):
    cluster_groups = load_cluster_groups(
        write(
            tmp_path,
            "cluster-groups.yaml",
            CLUSTER_GROUPS.replace(
                "environment_stages: [CANARY, DEFAULT]",
                "environment_stages: [DEFAULT, CANARY]",
            ),
        ),
        "services",
    )
    assert commands(load(tmp_path, cluster_groups, SHELLBOX)) == [
        ("/services/shellbox", "helmfile apply -e staging"),
        ("/services/shellbox", "helmfile apply -e codfw -l name=production"),
        ("/services/shellbox", "helmfile apply -e eqiad -l name=production"),
        ("/services/shellbox", "helmfile apply -e codfw -l name=canary"),
        ("/services/shellbox", "helmfile apply -e eqiad -l name=canary"),
    ]


def test_plan_with_cluster_group_specific_stage_names(tmp_path):
    """Stage names are whatever the cluster group declares, and the default stage
    is the one an environment with no releases is deployed at."""
    cluster_groups = load_cluster_groups(
        write(
            tmp_path,
            "cluster-groups.yaml",
            """
ml-serve:
  dir: ml-services
  environments:
    ml-serve-codfw:
      datacenter: codfw
      type: PRODUCTION
    ml-serve-eqiad:
      datacenter: eqiad
      type: PRODUCTION
  environment_stages: [experimental, rollout]
  environment_default_stage: rollout
""",
        ),
        "services",
    )
    service_config = load(
        tmp_path,
        cluster_groups,
        """
cluster_group: ml-serve
namespaces:
  revscoring:
    environments:
      ml-serve-codfw:
        releases:
          trial:
            stage: experimental
          main: {}
      ml-serve-eqiad: {}
""",
        service="revscoring",
    )
    assert commands(service_config) == [
        ("/ml-services/revscoring", "helmfile apply -e ml-serve-codfw -l name=trial"),
        ("/ml-services/revscoring", "helmfile apply -e ml-serve-codfw -l name=main"),
        ("/ml-services/revscoring", "helmfile apply -e ml-serve-eqiad"),
    ]


@pytest.mark.parametrize(
    "contents,expected",
    [
        (
            CLUSTER_GROUPS.replace("type: STAGING", "type: TESTING", 1),
            "must be one of STAGING, PRODUCTION",
        ),
        (
            CLUSTER_GROUPS.replace("[CANARY, DEFAULT]", "[CANARY, CANARY]"),
            "contains duplicate entries",
        ),
        (
            CLUSTER_GROUPS.replace("environment_default_stage: DEFAULT", "", 1),
            "environment_default_stage of cluster group wikikube must be a non-empty string",
        ),
        (
            CLUSTER_GROUPS.replace("environment_default_stage: DEFAULT", "x: 1", 1),
            r"unexpected key\(s\): x",
        ),
        (
            CLUSTER_GROUPS.replace(
                "environment_default_stage: DEFAULT",
                "environment_default_stage: PRODUCTION",
                1,
            ),
            "must be one of its environment_stages \\(CANARY, DEFAULT\\), not 'PRODUCTION'",
        ),
        (
            CLUSTER_GROUPS.replace("dir: dse-k8s-services", "dir: []"),
            "the dir of cluster group dse-k8s must be a non-empty string",
        ),
        (
            CLUSTER_GROUPS.replace("aliases: [staging]", "aliases: [codfw]"),
            "'codfw' refers to more than one environment",
        ),
        (
            CLUSTER_GROUPS.replace("      datacenter: codfw\n", "", 1),
            "datacenter of environment staging-codfw",
        ),
        (
            CLUSTER_GROUPS.replace("  environment_stages: [CANARY, DEFAULT]\n", ""),
            "environment_stages of cluster group wikikube must be a non-empty list",
        ),
        (
            CLUSTER_GROUPS.replace("aliases:", "alias:"),
            r"unexpected key\(s\): alias",
        ),
    ],
)
def test_invalid_cluster_groups(tmp_path, contents, expected):
    with pytest.raises(InvalidDeployServiceConfig, match=expected):
        load_cluster_groups(
            write(tmp_path, "cluster-groups.yaml", contents), "services"
        )


@pytest.mark.parametrize(
    "contents,expected",
    [
        (
            SHELLBOX.replace("cluster_group: wikikube", "cluster_group: dse"),
            "undefined cluster group 'dse'",
        ),
        (
            SHELLBOX.replace("      staging: {}", "      testing: {}"),
            "no environment named or aliased 'testing'",
        ),
        (
            SHELLBOX.replace(
                "      staging: {}", "      staging: {}\n      staging-eqiad: {}"
            ),
            "both refer to environment staging-eqiad",
        ),
        (
            SHELLBOX.replace("stage: CANARY", "stage: TESTSERVERS"),
            "must be one of CANARY, DEFAULT",
        ),
        (
            SHELLBOX.replace(
                "          production: {}",
                "          production: {}\n        release: {}",
                1,
            ),
            r"unexpected key\(s\): release",
        ),
        (
            "cluster_group: wikikube\nnamespaces: {}\n",
            "defines no namespaces",
        ),
        (
            "cluster_group: wikikube\nnamespaces:\n  shellbox:\n    environments:\n      staging:\n        releases: {}\n",
            "omit the key to deploy all releases",
        ),
    ],
)
def test_invalid_service_config(tmp_path, cluster_groups, contents, expected):
    with pytest.raises(InvalidDeployServiceConfig, match=expected):
        load(tmp_path, cluster_groups, contents)
