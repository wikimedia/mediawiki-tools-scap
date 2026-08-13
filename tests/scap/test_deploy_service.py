import argparse
import contextlib
from datetime import datetime, timezone
import logging
import os
import textwrap
import threading

import pytest

from scap import kubernetes, lock
from scap.deploy_service import (
    DeployService,
    HelmfileCommand,
    Rollback,
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
        (command.directory, " ".join(command.apply_command()))
        for group in plan(service_config, primary_datacenter, "/", 5)
        for command in group
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
        ("/dse-k8s-services/api2", "helmfile -e dse-k8s-eqiad apply --context 5")
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
        ("/services/shellbox", "helmfile -e staging apply --context 5"),
        (
            "/services/shellbox",
            "helmfile -e codfw --selector name=canary apply --context 5",
        ),
        (
            "/services/shellbox",
            "helmfile -e eqiad --selector name=canary apply --context 5",
        ),
        (
            "/services/shellbox",
            "helmfile -e codfw --selector name=production apply --context 5",
        ),
        (
            "/services/shellbox",
            "helmfile -e eqiad --selector name=production apply --context 5",
        ),
    ]


def test_diff_job_of_each_command(tmp_path, cluster_groups):
    """Each apply command has a diff job with the same selectors."""
    commands = [
        command
        for group in plan(load(tmp_path, cluster_groups, SHELLBOX), "eqiad", "/", 5)
        for command in group
    ]
    jobs = [command.diff_job() for command in commands]
    assert [" ".join(job.command) for job in jobs] == [
        "helmfile -e staging diff --context 5 --detailed-exitcode --suppress-secrets",
        "helmfile -e codfw --selector name=canary diff --context 5 --detailed-exitcode --suppress-secrets",
        "helmfile -e eqiad --selector name=canary diff --context 5 --detailed-exitcode --suppress-secrets",
        "helmfile -e codfw --selector name=production diff --context 5 --detailed-exitcode --suppress-secrets",
        "helmfile -e eqiad --selector name=production diff --context 5 --detailed-exitcode --suppress-secrets",
    ]
    assert commands[0].label == "shellbox in staging (STAGING/DEFAULT)"
    # The job runs where the step does, and its group is the environment.
    assert jobs[0].directory == commands[0].directory
    assert jobs[0].group == "staging"
    assert jobs[0].label == "shellbox in staging (STAGING/DEFAULT)"


def test_label_names_the_release(tmp_path, cluster_groups):
    """Two releases of one namespace at one stage have different labels."""
    service_config = load(
        tmp_path,
        cluster_groups,
        """
cluster_group: wikikube
namespaces:
  shellbox:
    environments:
      codfw:
        releases:
          canary:
            stage: CANARY
          canary-media:
            stage: CANARY
""",
    )
    (group,) = plan(service_config, "eqiad", "/", 5)
    assert [command.label for command in group] == [
        "canary of shellbox in codfw (PRODUCTION/CANARY)",
        "canary-media of shellbox in codfw (PRODUCTION/CANARY)",
    ]
    # A step that names a release does not repeat it.
    assert group[0].release_label("canary") == group[0].label
    # A step that deploys every release names the one that is rolling back.
    step = plan(load(tmp_path, cluster_groups, SHELLBOX), "eqiad", "/", 5)[0][0]
    assert step.release_label("main") == "main of shellbox in staging (STAGING/DEFAULT)"


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
        ("/services/shellbox-media", "helmfile -e codfw apply --context 5"),
        ("/services/shellbox-media", "helmfile -e eqiad apply --context 5"),
        ("/services/shellbox-syntaxhighlight", "helmfile -e eqiad apply --context 5"),
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
        (
            "/dse-k8s-services/api2",
            "helmfile -e dse-k8s-codfw --selector name=canary apply --context 5",
        ),
        (
            "/dse-k8s-services/api2",
            "helmfile -e dse-k8s-codfw --selector name=main apply --context 5",
        ),
        ("/dse-k8s-services/api2", "helmfile -e dse-k8s-eqiad apply --context 5"),
    ]


def test_plan_honors_primary_datacenter(tmp_path, cluster_groups):
    service_config = load(tmp_path, cluster_groups, SHELLBOX)
    assert commands(service_config, primary_datacenter="codfw") == [
        ("/services/shellbox", "helmfile -e staging apply --context 5"),
        (
            "/services/shellbox",
            "helmfile -e eqiad --selector name=canary apply --context 5",
        ),
        (
            "/services/shellbox",
            "helmfile -e codfw --selector name=canary apply --context 5",
        ),
        (
            "/services/shellbox",
            "helmfile -e eqiad --selector name=production apply --context 5",
        ),
        (
            "/services/shellbox",
            "helmfile -e codfw --selector name=production apply --context 5",
        ),
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
        ("/services/shellbox", "helmfile -e staging apply --context 5"),
        (
            "/services/shellbox",
            "helmfile -e codfw --selector name=production apply --context 5",
        ),
        (
            "/services/shellbox",
            "helmfile -e eqiad --selector name=production apply --context 5",
        ),
        (
            "/services/shellbox",
            "helmfile -e codfw --selector name=canary apply --context 5",
        ),
        (
            "/services/shellbox",
            "helmfile -e eqiad --selector name=canary apply --context 5",
        ),
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
        (
            "/ml-services/revscoring",
            "helmfile -e ml-serve-codfw --selector name=trial apply --context 5",
        ),
        (
            "/ml-services/revscoring",
            "helmfile -e ml-serve-codfw --selector name=main apply --context 5",
        ),
        ("/ml-services/revscoring", "helmfile -e ml-serve-eqiad apply --context 5"),
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


def test_steps_that_run_at_the_same_time(tmp_path, cluster_groups):
    """The namespaces of one environment deploy at the same time."""
    service_config = load(
        tmp_path,
        cluster_groups,
        """
cluster_group: wikikube
namespaces:
  shellbox:
    environments:
      codfw:
        releases:
          canary:
            stage: CANARY
          main: {}
      eqiad: {}
  shellbox-media:
    environments:
      codfw:
        releases:
          canary:
            stage: CANARY
          main: {}
      eqiad: {}
""",
    )
    assert [
        (
            group[0].environment_type,
            group[0].stage,
            group[0].environment,
            [command.namespace for command in group],
        )
        for group in plan(service_config, "eqiad", "/", 5)
    ] == [
        ("PRODUCTION", "CANARY", "codfw", ["shellbox", "shellbox-media"]),
        ("PRODUCTION", "DEFAULT", "codfw", ["shellbox", "shellbox-media"]),
        ("PRODUCTION", "DEFAULT", "eqiad", ["shellbox", "shellbox-media"]),
    ]


def _app(tmp_path, **arguments):
    """A DeployService that reads its config from tmp_path."""
    app = DeployService("deploy-service")
    app.arguments = argparse.Namespace(
        **{
            "service": "shellbox",
            "message": "(no justification provided)",
            # arg.py defines this for every command that announces.
            "no_log_message": False,
            "dry_run": False,
            "confirm_diffs": False,
            **arguments,
        }
    )
    app.config = {
        "cluster_groups_file": write(tmp_path, "cluster-groups.yaml", CLUSTER_GROUPS),
        "helmfile_default_cluster_dir": "services",
        "service_catalog_file": write(
            tmp_path,
            "service-catalog.yaml",
            "services:\n  shellbox:\n" + textwrap.indent(SHELLBOX.strip("\n"), "    "),
        ),
        "conftool_state_file": write(tmp_path, "conftool.yaml", CONFTOOL_STATE),
        "helmfile_deployments_dir": "/",
        "k8s_helmfile_diff_context_lines": 5,
        "k8s_max_concurrent_deployments_per_cluster": 2,
        "lock_dir": "/var/lock",
    }
    return app


@contextlib.contextmanager
def _no_lock(*args, **kwargs):
    """The lock of the service, for a test that does not need one."""
    yield


def test_main_locks_the_service(tmp_path, monkeypatch):
    """Two deployments of one service do not run at the same time.

    The message of the command line is the reason of the lock, for whoever
    waits.
    """
    app = _app(tmp_path, message="T435419 new images")
    locked = []

    class FakeLock:
        def __init__(self, lock_file, **kwargs):
            locked.append((lock_file, kwargs))

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

    monkeypatch.setattr(lock, "Lock", FakeLock)
    monkeypatch.setattr(app, "_deploy", lambda steps: 0)
    monkeypatch.setattr(app, "_assert_releases_exist", lambda steps: None)

    assert app.main() == 0
    assert len(locked) == 1
    lock_file, arguments = locked[0]
    assert lock_file == "/var/lock/scap.deploy-service.shellbox.lock"
    assert arguments["reason"] == "T435419 new images"
    assert arguments["name"] == "deploy-service"


def test_main_asks_for_a_message(tmp_path, monkeypatch):
    """A command line with no message asks for one, and the lock reports it."""
    app = _app(tmp_path)
    locked = []
    prompts = []

    class FakeLock:
        def __init__(self, lock_file, **kwargs):
            locked.append((lock_file, kwargs))

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

    def input_line(prompt):
        prompts.append(prompt)
        return "T435419 new images"

    monkeypatch.setattr(lock, "Lock", FakeLock)
    monkeypatch.setattr(app, "input_line", input_line)
    monkeypatch.setattr(app, "_deploy", lambda steps: 0)
    monkeypatch.setattr(app, "_assert_releases_exist", lambda steps: None)

    assert app.main() == 0
    assert prompts == ["Log message (press enter for none): "]
    assert locked[0][1]["reason"] == "T435419 new images"


def _announcements(app, monkeypatch):
    """Collects the SAL announcements of an application."""
    said = []
    monkeypatch.setattr(app, "announce", lambda *args: said.append(args[0] % args[1:]))
    monkeypatch.setattr(
        app, "announce_final", lambda *args: said.append(args[0] % args[1:])
    )
    return said


def test_main_announces_the_deployment(tmp_path, monkeypatch):
    """The entries of scap bracket the entries that helmfile makes."""
    app = _app(tmp_path, message="T435419 new images")
    said = _announcements(app, monkeypatch)
    monkeypatch.setattr(app, "lock", _no_lock)
    monkeypatch.setattr(app, "_assert_releases_exist", lambda steps: None)
    monkeypatch.setattr(app, "_deploy", lambda steps: 0)

    assert app.main() == 0
    assert said[0] == "Started scap deploy-service shellbox: T435419 new images"
    assert said[1].startswith(
        "Finished scap deploy-service shellbox: T435419 new images (duration: "
    )


def test_main_announces_a_deployment_that_failed(tmp_path, monkeypatch):
    """A status that is not zero ends the announcements with a failure."""
    app = _app(tmp_path, message="T435419 new images")
    said = _announcements(app, monkeypatch)
    monkeypatch.setattr(app, "lock", _no_lock)
    monkeypatch.setattr(app, "_assert_releases_exist", lambda steps: None)
    monkeypatch.setattr(app, "_deploy", lambda steps: 1)

    assert app.main() == 1
    assert said[1].startswith("Failed scap deploy-service shellbox: ")


def test_main_announces_a_deployment_that_stopped(tmp_path, monkeypatch):
    """An interrupt ends the announcements as well."""
    app = _app(tmp_path, message="T435419 new images")
    said = _announcements(app, monkeypatch)
    monkeypatch.setattr(app, "lock", _no_lock)
    monkeypatch.setattr(app, "_assert_releases_exist", lambda steps: None)

    def interrupt(steps):
        raise KeyboardInterrupt()

    monkeypatch.setattr(app, "_deploy", interrupt)

    with pytest.raises(KeyboardInterrupt):
        app.main()

    assert said[1].startswith("Failed scap deploy-service shellbox: ")


def test_main_reports_a_diff_command_that_failed(tmp_path, monkeypatch):
    """A failed diff makes scap exit 1 after a deployment that worked."""
    app = _app(tmp_path, confirm_diffs=True)
    reviewed = []
    monkeypatch.setattr(app, "lock", _no_lock)
    monkeypatch.setattr(app, "_deploy", lambda steps: 0)
    monkeypatch.setattr(app, "_assert_releases_exist", lambda steps: None)
    monkeypatch.setattr(
        kubernetes.K8sRunner, "review_diffs", lambda self, jobs: len(reviewed)
    )

    assert app.main() == 0

    reviewed.append("one failure")
    assert app.main() == 1


def test_main_keeps_the_status_of_a_deployment_that_failed(tmp_path, monkeypatch):
    app = _app(tmp_path, confirm_diffs=True)
    monkeypatch.setattr(app, "lock", _no_lock)
    monkeypatch.setattr(app, "_deploy", lambda steps: 1)
    monkeypatch.setattr(app, "_assert_releases_exist", lambda steps: None)
    monkeypatch.setattr(kubernetes.K8sRunner, "review_diffs", lambda self, jobs: 0)
    assert app.main() == 1


def test_main_rejects_a_release_that_the_environment_does_not_install(
    tmp_path, monkeypatch
):
    """A name with a mistake stops the deployment before the first step."""
    app = _app(tmp_path)
    monkeypatch.setattr(
        kubernetes.K8sRunner, "installed_releases", lambda self, invocation: ["main"]
    )

    with pytest.raises(InvalidDeployServiceConfig, match="does not install"):
        app.main()


TWO_NAMESPACES = """
cluster_group: wikikube
namespaces:
  shellbox:
    environments:
      codfw:
        releases:
          main: {}
  shellbox-media:
    environments:
      codfw:
        releases:
          main: {}
"""


def test_apply_retries_only_the_steps_that_failed(
    tmp_path, cluster_groups, monkeypatch
):
    """A retry deploys the steps that failed, and repairs them first."""
    app = _app(tmp_path)
    app.logger = logging.getLogger("test")
    monkeypatch.setattr(app, "prompt_choices", lambda *args, **kwargs: "r")

    repaired = []
    monkeypatch.setattr(
        app, "_repair", lambda command, release: repaired.append(release)
    )

    class FakeRunner:
        def releases(self, command):
            return [command.release]

    app.k8s = FakeRunner()

    (group,) = plan(load(tmp_path, cluster_groups, TWO_NAMESPACES), "eqiad", "/", 5)
    failed = kubernetes.CommandResult(1, "", "Error: UPGRADE FAILED")
    ok = kubernetes.CommandResult(0, "", "")
    attempts = []

    def fake_apply(steps):
        attempts.append([command.namespace for command in steps])
        # The first namespace fails, and works when it runs again.
        return [
            ok if len(attempts) > 1 or command.namespace != "shellbox" else failed
            for command in steps
        ]

    monkeypatch.setattr(app, "_apply", fake_apply)

    assert app._apply_until_it_works(group) is None
    assert attempts == [["shellbox", "shellbox-media"], ["shellbox"]]
    # The step that failed is repaired before it runs again.
    assert repaired == ["main"]


def test_apply_that_keeps_failing_rolls_back(tmp_path, cluster_groups, monkeypatch):
    """The user may stop the deployment, and choose what happens next."""
    app = _app(tmp_path)
    app.logger = logging.getLogger("test")
    monkeypatch.setattr(app, "prompt_choices", lambda *args, **kwargs: "b")
    failed = kubernetes.CommandResult(1, "", "Error: UPGRADE FAILED")
    monkeypatch.setattr(app, "_apply", lambda steps: [failed for _ in steps])

    (group,) = plan(load(tmp_path, cluster_groups, TWO_NAMESPACES), "eqiad", "/", 5)
    assert app._apply_until_it_works(group) == "b"


def _rollback(environment, namespace, release, revision=1):
    command = HelmfileCommand(
        "/services/shellbox",
        environment,
        None,
        5,
        "PRODUCTION",
        "PRODUCTION",
        namespace,
    )
    return Rollback(
        command,
        release,
        kubernetes.RecordedRevision(
            revision, datetime(2026, 8, 21, tzinfo=timezone.utc)
        ),
    )


def test_roll_back_unwinds_the_later_group_first(tmp_path, monkeypatch):
    """The groups roll back in the reverse of the order that deployed them."""
    app = _app(tmp_path)
    app.logger = logging.getLogger("test")
    app.k8s = kubernetes.K8sRunner(app, app.logger)
    monkeypatch.setattr(app, "prompt_user_for_confirmation", lambda *a, **k: True)
    monkeypatch.setattr(
        kubernetes.K8sRunner,
        "watch_releases",
        lambda self, invocations: contextlib.nullcontext({}),
    )

    rolled_back = []
    monkeypatch.setattr(
        app,
        "_roll_back_one",
        lambda rollback: rolled_back.append(rollback.release) or True,
    )

    app._roll_back(
        [
            [_rollback("codfw", "shellbox", "canary")],
            [_rollback("eqiad", "shellbox", "production")],
        ]
    )
    assert rolled_back == ["production", "canary"]


def test_roll_back_of_one_group_runs_at_the_same_time(tmp_path, monkeypatch):
    """The releases of one group roll back together, as they deployed.

    The barrier only opens when two rollbacks are in it at once, so a rollback
    that waits for the one before it times out.
    """
    app = _app(tmp_path)
    app.logger = logging.getLogger("test")
    app.k8s = kubernetes.K8sRunner(app, app.logger)
    monkeypatch.setattr(app, "prompt_user_for_confirmation", lambda *a, **k: True)
    monkeypatch.setattr(
        kubernetes.K8sRunner,
        "watch_releases",
        lambda self, invocations: contextlib.nullcontext({}),
    )

    barrier = threading.Barrier(2, timeout=10)
    monkeypatch.setattr(
        app, "_roll_back_one", lambda rollback: barrier.wait() is not None or True
    )

    app._roll_back(
        [
            [
                _rollback("eqiad", "shellbox", "canary"),
                _rollback("eqiad", "shellbox-media", "canary"),
            ]
        ]
    )
    assert not barrier.broken
