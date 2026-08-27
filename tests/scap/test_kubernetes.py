import concurrent.futures
import contextlib
from datetime import datetime, timedelta, timezone
import io
import queue
import json
import shlex
import logging
import mock
import pytest
import subprocess
import time

import scap.kubernetes
from scap.kubernetes import (
    K8sOps,
    CommandsCheck,
    DeploymentsConfig,
    CommandJob,
    InvalidDeploymentsConfig,
    LogstashCheck,
    built_image_ids,
    helm_command,
    HelmfileInvocation,
    inspect_images,
    K8sRunner,
    DepConfig,
    rollout_is_complete,
    rollout_was_abandoned,
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


class FakeApp:
    """Records the output and the approval prompt of review_diffs()."""

    def __init__(self, approve=True, max_workers=2):
        self.approve = approve
        self.lines = []
        self.cancelled = False
        self.config = {
            "k8s_max_concurrent_deployments_per_cluster": max_workers,
            "k8s_deployments_info_target_freshness": 1,
            "k8s_helmfile_diff_context_lines": 5,
        }

    def output_line(self, line, sensitive=False):
        self.lines.append(line)

    def prompt_for_approval_or_exit(self, question, cancel_message):
        if not self.approve:
            self.cancelled = True

    @contextlib.contextmanager
    def Timer(self, description, name="unsupplied", logger=None):
        yield


def _runner(app=None, logger=None, max_workers=2):
    """A K8sRunner with a fake app, and no HELM_* variables."""
    return K8sRunner(
        app or FakeApp(max_workers=max_workers),
        logger or logging.getLogger("test"),
        helm_env={},
    )


def _job(label, text, group, tmp_path, fail=False, exit_status=None):
    """A job whose command prints `text` and exits with `exit_status`.

    The default is the status of a diff that found a change.
    """
    if exit_status is None:
        # The status of a diff that found a change.
        exit_status = 2
    command = (
        ["false"]
        if fail
        else ["bash", "-c", f"echo {shlex.quote(text)}; exit {exit_status}"]
    )
    return CommandJob(label, group, str(tmp_path), command)


def test_run_jobs_keeps_the_order_of_the_jobs(tmp_path):
    jobs = [
        _job("one", "first", "codfw", tmp_path),
        _job("two", "second", "eqiad", tmp_path),
        _job("three", "third", "codfw", tmp_path),
    ]
    results = _runner().run_jobs(jobs, failed=scap.kubernetes.diff_failed)
    assert [result.stdout for result in results] == ["first\n", "second\n", "third\n"]


def test_run_jobs_reports_each_failure(tmp_path):
    jobs = [
        _job("ok", "output", "codfw", tmp_path),
        # A non-zero exit status.
        _job("bad status", "", "codfw", tmp_path, fail=True),
        # A command that does not exist raises, and must not stop the other jobs.
        CommandJob("no such command", "eqiad", str(tmp_path), ["scap-no-such-command"]),
    ]
    results = _runner().run_jobs(jobs, failed=scap.kubernetes.diff_failed)
    assert [scap.kubernetes.diff_failed(result) for result in results] == [
        False,
        True,
        True,
    ]
    assert results[0].stdout == "output\n"


def test_review_diffs_prints_each_diff_and_counts_failures(tmp_path):
    app = FakeApp()
    jobs = [
        _job("shellbox in codfw", "changed", "codfw", tmp_path),
        _job("shellbox in eqiad", "", "eqiad", tmp_path, fail=True),
    ]
    failures = _runner(app).review_diffs(jobs)
    assert failures == 1
    assert app.lines == ["=== Diff for shellbox in codfw ===\nchanged\n"]


def test_review_diffs_cancels_without_approval(tmp_path):
    app = FakeApp(approve=False)
    jobs = [_job("shellbox in codfw", "changed", "codfw", tmp_path)]
    _runner(app).review_diffs(jobs)
    assert app.cancelled


def test_run_jobs_survives_an_interrupt(tmp_path):
    """An interrupt stops the collection, but run_jobs() still returns."""
    jobs = [
        _job("one", "first", "codfw", tmp_path),
        _job("two", "second", "codfw", tmp_path),
    ]
    with mock.patch("scap.kubernetes.run_command", side_effect=KeyboardInterrupt):
        results = _runner(max_workers=1).run_jobs(
            jobs, failed=scap.kubernetes.diff_failed
        )
    # Each job reports why it has no exit status of its own. The first job
    # raises the interrupt, and the second reports the interrupt as well when
    # it started before the shutdown, or that it did not start when the
    # shutdown cancelled it first.
    assert [result.returncode for result in results] == [None, None]
    assert results[0].stderr == "KeyboardInterrupt: "
    assert results[1].stderr in ("KeyboardInterrupt: ", "The command did not start")


def test_finished_result_of_a_job_that_did_not_start():
    """A job that an interrupt cancelled reports why it has no exit status."""
    future = concurrent.futures.Future()
    future.cancel()
    result = scap.kubernetes._finished_result(future)
    assert result.returncode is None
    assert result.status == "no exit status"
    assert result.stderr == "The command did not start"


def test_review_diffs_requests_approval_after_an_interrupt(tmp_path):
    app = FakeApp(approve=False)
    jobs = [_job("shellbox in codfw", "changed", "codfw", tmp_path)]
    with mock.patch("scap.kubernetes.run_command", side_effect=KeyboardInterrupt):
        failures = _runner(app, max_workers=1).review_diffs(jobs)
    assert failures == 1
    assert app.lines == []
    assert app.cancelled


def test_helmfile_invocation_commands():
    """One release, every release, and extra arguments for the subcommand."""
    one_release = HelmfileInvocation("/services/shellbox", "codfw", "canary", 5)
    every_release = HelmfileInvocation("/services/shellbox", "staging", None, 5)

    assert one_release.apply_command() == [
        "helmfile",
        "-e",
        "codfw",
        "--selector",
        "name=canary",
        "apply",
        "--context",
        "5",
    ]
    assert every_release.apply_command() == [
        "helmfile",
        "-e",
        "staging",
        "apply",
        "--context",
        "5",
    ]
    assert one_release.diff_command() == [
        "helmfile",
        "-e",
        "codfw",
        "--selector",
        "name=canary",
        "diff",
        "--context",
        "5",
        "--detailed-exitcode",
        "--suppress-secrets",
    ]
    assert every_release.build_command() == ["helmfile", "-e", "staging", "build"]
    assert one_release.write_values_command("/tmp/values.yaml") == [
        "helmfile",
        "-e",
        "codfw",
        "--selector",
        "name=canary",
        "write-values",
        "--output-file-template",
        "/tmp/values.yaml",
    ]


def test_helmfile_invocation_diff_job():
    invocation = HelmfileInvocation("/services/shellbox", "codfw", "canary", 5)
    job = invocation.diff_job("shellbox in codfw")
    assert job.label == "shellbox in codfw"
    assert job.directory == "/services/shellbox"
    # One pool for each environment.
    assert job.group == "codfw"


def test_diff_job_runs_the_diff_command(tmp_path):
    invocation = HelmfileInvocation(str(tmp_path), "codfw", "canary", 5)
    job = invocation.diff_job("shellbox in codfw")
    result = scap.kubernetes.CommandResult(0, "+ replicas: 3", "")
    with mock.patch("scap.kubernetes.run_command", return_value=result) as run:
        assert _runner().run_jobs([job]) == [result]
    assert run.call_args[0][0] == invocation.diff_command()
    assert run.call_args[0][1] == str(tmp_path)


def test_helm_command():
    assert helm_command("/etc/kubernetes/a.config", "rollback", "canary") == [
        "helm",
        "--kubeconfig",
        "/etc/kubernetes/a.config",
        "rollback",
        "canary",
    ]
    assert helm_command("/etc/kubernetes/a.config", "ls", "-o", "json") == [
        "helm",
        "--kubeconfig",
        "/etc/kubernetes/a.config",
        "ls",
        "-o",
        "json",
    ]


def test_helmfile_invocation_kubeconfig(tmp_path):
    invocation = HelmfileInvocation(str(tmp_path), "codfw", "canary", 5)
    logger = logging.getLogger("test")
    build_output = """
helmDefaults:
  args:
    - --kubeconfig
    - /etc/kubernetes/shellbox-deploy-codfw.config
"""

    with mock.patch.object(K8sRunner, "stdout", return_value=build_output) as run:
        assert (
            _runner(logger=logger).kubeconfig(invocation)
            == "/etc/kubernetes/shellbox-deploy-codfw.config"
        )
    # The kubeconfig comes from "helmfile build" in the directory of the invocation.
    assert run.call_args[0][0] == invocation.build_command()
    assert run.call_args[0][1] == str(tmp_path)

    # A helmfile that fails stops scap. Every later step needs this state.
    with mock.patch.object(K8sRunner, "stdout", return_value=None):
        with pytest.raises(scap.kubernetes.HelmfileError, match="helmfile state"):
            _runner(logger=logger).kubeconfig(invocation)

    # A helmfile that declares no kubeconfig stops scap too.
    with mock.patch.object(
        K8sRunner, "stdout", return_value="helmDefaults:\n  args: []\n"
    ):
        with pytest.raises(scap.kubernetes.HelmfileError, match="no kubeconfig"):
            _runner(logger=logger).kubeconfig(invocation)


def test_review_diffs_reports_a_job_with_no_changes(tmp_path):
    """The exit status says whether a release has a change."""
    app = FakeApp()
    jobs = [
        _job("shellbox in codfw", "+ replicas: 3", "codfw", tmp_path),
        _job("shellbox in eqiad", "no change here", "eqiad", tmp_path, exit_status=0),
    ]
    assert _runner(app).review_diffs(jobs) == 0
    assert app.lines == [
        "=== Diff for shellbox in codfw ===\n+ replicas: 3\n",
        "=== No changes for shellbox in eqiad ===",
    ]


def test_apply_job_runs_the_apply_command(tmp_path):
    invocation = HelmfileInvocation(str(tmp_path), "codfw", "canary", 5)
    job = invocation.apply_job("shellbox in codfw")
    result = scap.kubernetes.CommandResult(0, "UPDATED RELEASES:", "")
    with mock.patch("scap.kubernetes.run_command", return_value=result) as run:
        assert _runner().run_jobs([job]) == [result]
    assert run.call_args[0][0] == invocation.apply_command()
    assert run.call_args[0][1] == str(tmp_path)


def test_apply_job_and_diff_job_use_their_own_command():
    invocation = HelmfileInvocation("/services/shellbox", "codfw", "canary", 5)
    assert invocation.apply_job("label").command == invocation.apply_command()
    assert invocation.diff_job("label").command == invocation.diff_command()
    # Both jobs of one environment share a thread pool.
    assert invocation.apply_job("label").group == "codfw"


def test_deployments_of_release(tmp_path):
    items = {
        "items": [
            {
                "metadata": {"name": "shellbox.codfw.main"},
                "spec": {"replicas": 3},
                "status": {"availableReplicas": 2},
            },
            {
                "metadata": {"name": "shellbox.codfw.main-tls"},
                "spec": {"replicas": 1},
                "status": {},
            },
        ]
    }
    ok = subprocess.CompletedProcess([], 0, json.dumps(items), "")
    with mock.patch("subprocess.run", return_value=ok) as run:
        # Every Deployment of the release, and not only the one that the chart
        # names after the release.
        assert [
            deployment["metadata"]["name"]
            for deployment in scap.kubernetes.deployments_of_release(
                "/etc/kubernetes/a", "main"
            )
        ] == ["shellbox.codfw.main", "shellbox.codfw.main-tls"]
    assert run.call_args[0][0][:4] == [
        "kubectl",
        "--kubeconfig",
        "/etc/kubernetes/a",
        "get",
    ]
    assert "release=main" in run.call_args[0][0]

    # A cluster that scap cannot reach must not stop a deployment.
    failed = subprocess.CompletedProcess([], 1, "", "connection refused")
    with mock.patch("subprocess.run", return_value=failed):
        assert scap.kubernetes.deployments_of_release("/etc/kubernetes/a", "main") == []


def test_releases_state():
    """One read gives the revision and the status of each release."""
    invocation = HelmfileInvocation("/services/shellbox", "codfw", None, 5)
    listing = (
        '[{"name":"main","revision":"41","status":"deployed"},'
        '{"name":"migration","revision":"7","status":"pending-upgrade"}]'
    )
    with mock.patch.object(K8sRunner, "kubeconfig", return_value="/etc/kubernetes/a"):
        with mock.patch.object(K8sRunner, "stdout", return_value=listing) as run:
            assert _runner().releases_state(invocation) == {
                "main": scap.kubernetes.ReleaseState(41, "deployed"),
                "migration": scap.kubernetes.ReleaseState(7, "pending-upgrade"),
            }

        with mock.patch.object(K8sRunner, "stdout", return_value=None):
            with pytest.raises(
                scap.kubernetes.HelmfileError, match="list the releases"
            ):
                _runner().releases_state(invocation)
    assert run.call_args[0][0] == [
        "helm",
        "--kubeconfig",
        "/etc/kubernetes/a",
        "ls",
        # -a, so that a release in a pending state also appears.
        "-a",
        "-o",
        "json",
        # An explicit limit, because `helm ls` asks for 256 without one.
        "--max=1000",
    ]


def test_releases_state_of_a_list_that_may_leave_releases_out():
    """A full list is not release state, because helm reports no truncation."""
    invocation = HelmfileInvocation("/services/shellbox", "codfw", None, 5)
    listing = json.dumps(
        [
            {"name": f"release{number}", "revision": "1", "status": "deployed"}
            for number in range(1000)
        ]
    )
    with mock.patch.object(K8sRunner, "kubeconfig", return_value="/etc/kubernetes/a"):
        with mock.patch.object(K8sRunner, "stdout", return_value=listing):
            with pytest.raises(
                scap.kubernetes.HelmfileError, match="might be truncated"
            ):
                _runner().releases_state(invocation)


def test_fix_pending_state_repairs_a_pending_release():
    """A pending release accepts no deployment, so scap clears the state."""
    invocation = HelmfileInvocation("/services/shellbox", "codfw", "canary", 5)
    state = {"canary": scap.kubernetes.ReleaseState(7, "pending-upgrade")}
    ok = scap.kubernetes.CommandResult(0, "", "")
    with mock.patch.object(K8sRunner, "kubeconfig", return_value="/etc/kubernetes/a"):
        with mock.patch.object(K8sRunner, "releases_state", return_value=state):
            with mock.patch.object(K8sRunner, "run", return_value=ok) as run:
                assert (
                    _runner().fix_pending_state(invocation, "canary")
                    == "/etc/kubernetes/a"
                )
    assert run.call_args[0][0] == [
        "helm",
        "--kubeconfig",
        "/etc/kubernetes/a",
        "rollback",
        "canary",
    ]


def test_fix_pending_state_of_a_release_that_is_not_pending():
    """A release in a good state runs no repair."""
    invocation = HelmfileInvocation("/services/shellbox", "codfw", "canary", 5)
    state = {"canary": scap.kubernetes.ReleaseState(7, "deployed")}
    with mock.patch.object(K8sRunner, "kubeconfig", return_value="/etc/kubernetes/a"):
        with mock.patch.object(K8sRunner, "releases_state", return_value=state):
            with mock.patch.object(K8sRunner, "run") as run:
                _runner().fix_pending_state(invocation, "canary")
    run.assert_not_called()


STATE_WITHOUT_WAIT = {"helmDefaults": {"args": ["--kubeconfig", "/etc/kubernetes/a"]}}
STATE_WITH_WAIT = {
    "helmDefaults": {
        "args": ["--kubeconfig", "/etc/kubernetes/a"],
        "wait": True,
        "timeout": 600,
    }
}


RECORDED_AT = datetime(2026, 8, 21, tzinfo=timezone.utc)


def _recorded(revision, recorded_at=RECORDED_AT):
    """The revision that scap recorded before a deployment."""
    return scap.kubernetes.RecordedRevision(revision, recorded_at)


def test_roll_back_names_the_revision():
    """The command names the revision, so that a second attempt does the same."""
    invocation = HelmfileInvocation("/services/shellbox", "codfw", "canary", 5)
    ok = scap.kubernetes.CommandResult(0, "Rollback was a success", "")
    with mock.patch.object(K8sRunner, "state", return_value=STATE_WITHOUT_WAIT):
        with mock.patch.object(K8sRunner, "run", return_value=ok) as run:
            assert _runner().roll_back(
                invocation, "canary", _recorded(41), "the rollback"
            )
    assert run.call_args[0][0] == [
        "helm",
        "--kubeconfig",
        "/etc/kubernetes/a",
        "rollback",
        "canary",
        "41",
    ]


def test_roll_back_waits_for_an_atomic_release():
    """helm waits by itself for --atomic, and a rollback has no --atomic."""
    invocation = HelmfileInvocation("/services/shellbox", "codfw", "canary", 5)
    state = {
        "helmDefaults": {
            "args": ["--kubeconfig", "/etc/kubernetes/a"],
            "wait": False,
            "atomic": True,
            "timeout": 600,
        }
    }
    ok = scap.kubernetes.CommandResult(0, "", "")
    with mock.patch.object(K8sRunner, "state", return_value=state):
        with mock.patch.object(K8sRunner, "run", return_value=ok) as run:
            assert _runner().roll_back(
                invocation, "canary", _recorded(41), "the rollback"
            )
    assert run.call_args[0][0][-4:] == ["41", "--wait", "--timeout", "600s"]


def test_roll_back_waits_when_the_helmfile_waits():
    """A rollback completes in the same way as the deployment of the release."""
    invocation = HelmfileInvocation("/services/shellbox", "codfw", "canary", 5)
    ok = scap.kubernetes.CommandResult(0, "Rollback was a success", "")
    with mock.patch.object(K8sRunner, "state", return_value=STATE_WITH_WAIT):
        with mock.patch.object(K8sRunner, "run", return_value=ok) as run:
            assert _runner().roll_back(
                invocation, "canary", _recorded(41), "the rollback"
            )
    assert run.call_args[0][0][-4:] == ["41", "--wait", "--timeout", "600s"]


def test_replica_progress_reports_nothing_without_replicas():
    with scap.kubernetes.replica_progress("test", 0) as report_queue:
        assert report_queue is None


def test_helmfile_invocation_installed_releases():
    invocation = HelmfileInvocation("/services/shellbox", "traindev", None, 5)
    # icu72 is declared, but this environment does not install it.
    listing = json.dumps(
        [
            {"name": "main", "enabled": True, "installed": True},
            {"name": "migration", "enabled": True, "installed": True},
            {"name": "icu72", "enabled": True, "installed": False},
        ]
    )
    with mock.patch.object(K8sRunner, "stdout", return_value=listing) as run:
        assert _runner().installed_releases(invocation) == [
            "main",
            "migration",
        ]
    assert run.call_args[0][0] == invocation.list_command()

    with mock.patch.object(K8sRunner, "stdout", return_value=None):
        with pytest.raises(scap.kubernetes.HelmfileError, match="list the releases"):
            _runner().installed_releases(invocation)


def _values_writer(*runs):
    """A run_command that writes the values of each run, or fails.

    Each entry of `runs` is a {release: document} mapping for one call, or None
    for a call that fails. The file names follow --output-file-template, as
    helmfile expands it for each release.
    """
    remaining = list(runs)

    def run(cmd, directory, logger, env):
        documents = remaining.pop(0)
        if documents is None:
            return scap.kubernetes.CommandResult(1, "", "Error: no such environment")

        template = cmd[cmd.index("--output-file-template") + 1]
        for release, document in documents.items():
            path = template.replace("{{ .Release.Name }}.yaml", f"{release}.yaml")
            with open(path, "w") as f:
                f.write(document)
        return scap.kubernetes.CommandResult(0, "", "")

    return run


def test_expected_replicas_reads_the_rendered_values():
    """The count is the target of the deployment, over every release."""
    invocations = [
        HelmfileInvocation("/services/shellbox", "codfw", "canary", 5),
        HelmfileInvocation("/services/shellbox-media", "codfw", "main", 5),
    ]
    with mock.patch(
        "scap.kubernetes.run_command",
        side_effect=_values_writer(
            {"canary": "resources:\n  replicas: 2\n"},
            {"main": "resources:\n  replicas: 5\n"},
        ),
    ):
        assert _runner().expected_replicas(invocations) == 7


def test_expected_replicas_of_a_chart_without_the_key():
    """A chart that holds its count elsewhere reports no replicas."""
    invocation = HelmfileInvocation("/services/flink", "codfw", "main", 5)
    with mock.patch(
        "scap.kubernetes.run_command",
        side_effect=_values_writer({"main": "replicaCount: 3\n"}),
    ):
        assert _runner().expected_replicas([invocation]) == 0


# How many times _rendered_values reads the values of the releases.
ATTEMPTS = 3


def test_expected_replicas_tries_again_then_gives_up(monkeypatch):
    """A failure is retried, and the deployment continues without a report."""
    # The delay between two attempts is local to the method, so the test
    # takes the sleep away instead.
    monkeypatch.setattr(scap.kubernetes.time, "sleep", lambda seconds: None)
    invocation = HelmfileInvocation("/services/shellbox", "codfw", "canary", 5)

    # The second attempt works.
    with mock.patch(
        "scap.kubernetes.run_command",
        side_effect=_values_writer(None, {"canary": "resources:\n  replicas: 4\n"}),
    ) as run:
        assert _runner().expected_replicas([invocation]) == 4
    assert run.call_count == 2

    # Every attempt fails.
    with mock.patch(
        "scap.kubernetes.run_command",
        side_effect=_values_writer(*[None] * ATTEMPTS),
    ) as run:
        assert _runner().expected_replicas([invocation]) == 0
    assert run.call_count == ATTEMPTS


def test_expected_replicas_of_every_release_of_an_environment():
    """One call renders each release that the invocation selects."""
    invocation = HelmfileInvocation("/services/shellbox", "staging", None, 5)
    with mock.patch(
        "scap.kubernetes.run_command",
        side_effect=_values_writer(
            {
                "main": "resources:\n  replicas: 2\n",
                "media": "resources:\n  replicas: 1\n",
            }
        ),
    ) as run:
        assert _runner().expected_replicas([invocation]) == 3
    assert run.call_count == 1
    # No selector, so helmfile writes the values of every release.
    assert "--selector" not in run.call_args[0][0]


def test_roll_back_of_a_new_release_uninstalls_it():
    """A release that the deployment installed returns to its state by going away."""
    invocation = HelmfileInvocation("/services/shellbox", "codfw", "canary", 5)
    ok = scap.kubernetes.CommandResult(0, 'release "canary" uninstalled', "")
    # helm first deployed the release after scap recorded that it was not there.
    installed = RECORDED_AT + timedelta(minutes=5)
    with mock.patch.object(K8sRunner, "state", return_value=STATE_WITHOUT_WAIT):
        with mock.patch.object(K8sRunner, "first_deployed", return_value=installed):
            with mock.patch.object(K8sRunner, "run", return_value=ok) as run:
                assert _runner().roll_back(
                    invocation, "canary", _recorded(None), "the rollback"
                )
    assert run.call_args[0][0] == [
        "helm",
        "--kubeconfig",
        "/etc/kubernetes/a",
        "uninstall",
        "canary",
        # A second attempt also succeeds.
        "--ignore-not-found",
    ]


def test_roll_back_keeps_a_release_that_the_deployment_did_not_install():
    """A record of "not installed" is not enough to uninstall a release.

    `helm ls` reports no release whose record it cannot read, so scap reads the
    one release before it removes it. A release that stays is no failure of the
    rollback, so the result is True.
    """
    invocation = HelmfileInvocation("/services/shellbox", "codfw", "canary", 5)
    older = RECORDED_AT - timedelta(days=9)

    with mock.patch.object(K8sRunner, "state", return_value=STATE_WITHOUT_WAIT):
        with mock.patch.object(K8sRunner, "run") as run:
            # helm first deployed the release before scap recorded the state.
            with mock.patch.object(K8sRunner, "first_deployed", return_value=older):
                assert _runner().roll_back(
                    invocation, "canary", _recorded(None), "the rollback"
                )

            # Scap cannot read the release.
            with mock.patch.object(K8sRunner, "first_deployed", return_value=None):
                assert _runner().roll_back(
                    invocation, "canary", _recorded(None), "the rollback"
                )

    run.assert_not_called()


def test_first_deployed():
    """The time that helm reports for one release, and None when it stops."""
    invocation = HelmfileInvocation("/services/shellbox", "codfw", "canary", 5)
    status = scap.kubernetes.CommandResult(
        0,
        '{"name":"canary","info":{"first_deployed":"2026-08-12T20:09:48.018880843Z"}}',
        "",
    )
    with mock.patch.object(K8sRunner, "kubeconfig", return_value="/etc/kubernetes/a"):
        with mock.patch.object(K8sRunner, "run", return_value=status) as run:
            assert _runner().first_deployed(invocation, "canary") == datetime(
                2026, 8, 12, 20, 9, 48, 18880, tzinfo=timezone.utc
            )
        assert run.call_args[0][0] == [
            "helm",
            "--kubeconfig",
            "/etc/kubernetes/a",
            "status",
            "canary",
            "-o",
            "json",
        ]

        # helm stops for a release that it does not hold, and for a record that
        # it cannot read.
        failed = scap.kubernetes.CommandResult(1, "", "Error: release: not found")
        with mock.patch.object(K8sRunner, "run", return_value=failed):
            assert _runner().first_deployed(invocation, "canary") is None


def _k8s_object(name, revision, available=0, owner=None, wanted=3):
    """A Deployment or a ReplicaSet, as kubectl reports it."""
    return {
        "metadata": {
            "name": name,
            "generation": 1,
            "annotations": {"deployment.kubernetes.io/revision": revision},
            **({"ownerReferences": [{"name": owner}]} if owner else {}),
        },
        "spec": {"replicas": wanted},
        "status": {
            "observedGeneration": 1,
            "availableReplicas": available,
        },
    }


def test_new_replicas_counts_only_the_pods_of_the_deployment():
    """The pods from before the deployment do not count as progress."""
    deployments = {
        "items": [
            # Rolled to revision 5, and two of its new pods are available.
            _k8s_object("shellbox.codfw.main", "5", available=2),
            # Still at the revision it had before.
            _k8s_object("shellbox.codfw.main-tls", "3", available=3),
        ]
    }
    replicasets = {
        "items": [
            # The old pods of main are still available, and must not count.
            _k8s_object("main-old", "4", available=3, owner="shellbox.codfw.main"),
            _k8s_object("main-new", "5", available=2, owner="shellbox.codfw.main"),
        ]
    }

    def kubectl(cmd, **kwargs):
        listing = replicasets if "replicaset" in cmd else deployments
        return subprocess.CompletedProcess([], 0, json.dumps(listing), "")

    before = {"shellbox.codfw.main": "4", "shellbox.codfw.main-tls": "3"}
    with mock.patch("subprocess.run", side_effect=kubectl):
        # main rolled and two of its three new pods are available, so k8s has
        # not finished it. main-tls did not roll, and k8s calls it complete.
        assert scap.kubernetes._new_replicas_of_release(
            "/etc/kubernetes/a", "main", before
        ) == {
            "shellbox.codfw.main": (2, False, False),
            "shellbox.codfw.main-tls": (0, True, False),
        }

        # At the end of a deployment, a Deployment that keeps its revision made
        # no new pod, so its pods count (T375514).
        assert scap.kubernetes._new_replicas_of_release(
            "/etc/kubernetes/a", "main", before, unchanged_is_done=True
        ) == {
            "shellbox.codfw.main": (2, False, False),
            "shellbox.codfw.main-tls": (3, True, False),
        }


def _deployment(generation=1, observed=1, wanted=3):
    return {
        "metadata": {"name": "mw-web.traindev.main", "generation": generation},
        "spec": {"replicas": wanted},
        "status": {"observedGeneration": observed},
    }


def test_rollout_is_complete():
    """The rule that says when to stop counting the new pods."""
    assert rollout_is_complete(_deployment(), 3)

    # k8s has not seen the last change of the Deployment yet.
    assert not rollout_is_complete(_deployment(generation=2, observed=1), 3)
    # The new pods are not all available.
    assert not rollout_is_complete(_deployment(), 2)


def test_new_replicas_of_a_rollout_that_k8s_has_not_finished():
    """A Deployment that kept its revision counts only when k8s finished it.

    The revision comes from the deployment controller, so a Deployment whose
    revision is only late must not report the pods of before (T375514).
    """
    listing = {
        "items": [
            # k8s has not seen the last change of this Deployment yet.
            _k8s_object("shellbox.codfw.main", "5", available=3)
        ]
    }
    listing["items"][0]["metadata"]["generation"] = 2

    def kubectl(cmd, **kwargs):
        return subprocess.CompletedProcess([], 0, json.dumps(listing), "")

    with mock.patch("subprocess.run", side_effect=kubectl):
        assert scap.kubernetes._new_replicas_of_release(
            "/etc/kubernetes/a",
            "main",
            {"shellbox.codfw.main": "5"},
            unchanged_is_done=True,
        ) == {"shellbox.codfw.main": (0, False, False)}


def test_rollout_that_the_old_pods_make_look_complete():
    """The pods that drain must not end the count of the new pods (T375514).

    helm returns when the Deployment holds the number of pods that it wants.
    At that moment the pods of the revision from before are still available, so
    `status.availableReplicas` is at the wanted number while the new
    ReplicaSet still misses pods.
    """
    deployments = {"items": [_k8s_object("shellbox.codfw.main", "5", available=3)]}
    replicasets = {
        "items": [
            # One pod of the revision from before is still draining.
            _k8s_object("main-old", "4", available=1, owner="shellbox.codfw.main"),
            _k8s_object("main-new", "5", available=2, owner="shellbox.codfw.main"),
        ]
    }

    def kubectl(cmd, **kwargs):
        listing = replicasets if "replicaset" in cmd else deployments
        return subprocess.CompletedProcess([], 0, json.dumps(listing), "")

    with mock.patch("subprocess.run", side_effect=kubectl):
        assert scap.kubernetes._new_replicas_of_release(
            "/etc/kubernetes/a", "main", {"shellbox.codfw.main": "4"}
        ) == {"shellbox.codfw.main": (2, False, False)}


def test_monitor_release_reports_until_the_replicas_arrive(monkeypatch):
    """helm returns early, so the report continues for a moment (T375514)."""

    # The replicas arrive after the command returns.
    counts = [
        {"main": (1, False, False)},
        {"main": (2, False, False)},
        {"main": (3, True, False)},
    ]
    seen = []

    def new_replicas(kubeconfig, release, before, unchanged_is_done=False):
        seen.append(unchanged_is_done)
        return counts[min(len(seen) - 1, len(counts) - 1)]

    monkeypatch.setattr(scap.kubernetes, "_new_replicas_of_release", new_replicas)
    monkeypatch.setattr(scap.kubernetes, "deployments_of_release", lambda *a: [])

    reports = queue.Queue()
    with scap.kubernetes.monitor_release("/etc/kubernetes/a", "main", reports, 60):
        pass

    # The report of the last pod arrives one time, and not two.
    assert list(reports.queue) == [("main", 1), ("main", 2), ("main", 3)]
    assert seen[-1] is True


def test_monitor_release_waits_while_k8s_works_on_the_rollout(monkeypatch):
    """A pod that arrives long after helm returns still counts (T375514).

    helm returns before the last pods of a rollout are available. k8s says that
    it is still working on the rollout, so the count waits for it.
    """
    # Nothing arrives for several passes, and then the last pod does.
    counts = [{"main": (2, False, False)}] * 4 + [{"main": (3, True, False)}]
    seen = []

    def new_replicas(kubeconfig, release, before, unchanged_is_done=False):
        seen.append(unchanged_is_done)
        return counts[min(len(seen) - 1, len(counts) - 1)]

    monkeypatch.setattr(scap.kubernetes, "_new_replicas_of_release", new_replicas)
    monkeypatch.setattr(scap.kubernetes, "deployments_of_release", lambda *a: [])

    reports = queue.Queue()
    # A pod that arrives after four cycles that count the same pods.
    with scap.kubernetes.monitor_release("/etc/kubernetes/a", "main", reports, 60):
        pass

    assert list(reports.queue)[-1] == ("main", 3)


def test_monitor_release_when_kubectl_fails(monkeypatch):
    """A cycle that reads nothing must not end the count (T415839).

    kubectl reports no object when it fails, and a release that reports no
    Deployment would otherwise read as a rollout that k8s finished.
    """
    calls = []

    def kubectl(cmd, **kwargs):
        calls.append(cmd)
        # The Deployments of the release are there for the first read, which
        # records the revisions, and kubectl fails for every read after it.
        if len(calls) == 1:
            listing = {"items": [_k8s_object("shellbox.codfw.main", "4", available=3)]}
            return subprocess.CompletedProcess([], 0, json.dumps(listing), "")

        return subprocess.CompletedProcess([], 1, "", "The connection was refused")

    reports = queue.Queue()
    started = time.monotonic()
    with mock.patch("subprocess.run", side_effect=kubectl):
        with scap.kubernetes.monitor_release("/etc/kubernetes/a", "main", reports, 60):
            pass

    # READ_ATTEMPTS reads bound the wait, and nothing was reported.
    waited = time.monotonic() - started
    assert waited < 10, f"waited {waited}s"
    assert list(reports.queue) == []


def _ops_for_rollback(revisions):
    """A K8sOps with only what the rollback of one release reads."""
    ops = mock.Mock()
    ops.rollback_revisions = {
        name: _recorded(revision) for name, revision in revisions.items()
    }
    ops.app.config = {"k8s_deployments_info_target_freshness": 1}
    return ops


def test_record_rollback_revisions_notes_a_release_that_is_not_installed():
    """A release that is not installed is recorded as None, not left out.

    The rollback of such a release uninstalls it, so it needs an entry. A
    release that scap does not deploy needs none, because the rollback does not
    touch it.
    """
    ops = mock.Mock()
    ops.rollback_revisions = {}
    ops.runner.read_directories = _runner().read_directories
    ops.runner.releases_state.return_value = {
        "main": scap.kubernetes.ReleaseState(7, "deployed")
    }
    dep_configs = [
        DepConfig(
            namespace="mw-web",
            release=release,
            cluster="eqiad",
            scope="train",
            mw_image_kind=None,
            mw_image_flavour="publish",
            web_image_flavour="webserver",
            deploy=release != "maintenance",
            fq_release_name=f"mw-web-{release}-eqiad",
            values_file=f"/values/mw-web-{release}-eqiad.yaml",
            helmfile_dir="/helmfile/mw-web",
        )
        for release in ["main", "canary", "maintenance"]
    ]

    K8sOps._record_rollback_revisions(ops, dep_configs)

    # main is installed, canary is not, and scap does not deploy maintenance.
    assert {
        name: recorded.revision for name, recorded in ops.rollback_revisions.items()
    } == {
        "mw-web-main-eqiad": 7,
        "mw-web-canary-eqiad": None,
    }
    # One read of the state, so both releases record the same time.
    assert (
        len({recorded.recorded_at for recorded in ops.rollback_revisions.values()}) == 1
    )


def test_record_rollback_revisions_of_a_stage_that_cannot_read_the_state():
    """A read that fails records no revision of the stage.

    The stage stops before it deploys anything, so it must roll nothing back.
    Every directory reads, so one report names each one that failed.
    """
    dep_configs = [
        DepConfig(
            namespace=namespace,
            release="main",
            cluster="eqiad",
            scope="train",
            mw_image_kind=None,
            mw_image_flavour="publish",
            web_image_flavour="webserver",
            deploy=True,
            fq_release_name=f"{namespace}-main-eqiad",
            values_file=f"/values/{namespace}-main-eqiad.yaml",
            helmfile_dir=f"/helmfile/{namespace}",
        )
        for namespace in ["mw-web", "mw-api-ext"]
    ]

    def releases_state(invocation):
        # One directory reads, and helm fails for the other one.
        if invocation.directory == "/helmfile/mw-web":
            return {"main": scap.kubernetes.ReleaseState(7, "deployed")}
        raise scap.kubernetes.HelmfileError("Could not list the releases")

    ops = mock.Mock()
    ops.rollback_revisions = {}
    ops.runner.read_directories = _runner().read_directories
    ops._invocation.side_effect = lambda dep_config: HelmfileInvocation(
        dep_config.helmfile_dir, "eqiad", dep_config.release, 5
    )
    ops.runner.releases_state.side_effect = releases_state

    with pytest.raises(scap.kubernetes.HelmfileError) as error:
        K8sOps._record_rollback_revisions(ops, dep_configs)

    assert "/helmfile/mw-api-ext in eqiad" in str(error.value)
    assert ops.runner.releases_state.call_count == 2
    assert ops.rollback_revisions == {}


def test_record_rollback_revisions_covers_every_stage():
    """One record covers the releases of every stage.

    Scap records the revisions before the first stage deploys, so each release
    rolls back to the revision it had before the run.
    """
    ops = mock.Mock()
    ops.app.config = {"deploy_mw_container_image": True}
    ops.app.Timer = mock.MagicMock()
    ops.k8s_deployments_config = _parse(MAP_CONFIG)
    ops.get_stage_dep_configs.side_effect = lambda stage: K8sOps.get_stage_dep_configs(
        ops, stage
    )

    K8sOps.record_rollback_revisions(ops)

    ops._record_rollback_revisions.assert_called_once()
    (dep_configs,) = ops._record_rollback_revisions.call_args.args
    assert {dep_config.fq_release_name for dep_config in dep_configs} == {
        "mw-pretrain-web-canary-default-cluster",
        "mw-pretrain-web-default-cluster",
        "mw-web-canary-default-cluster",
        "mw-web-main-default-cluster",
        "mw-legacy-main-default-cluster",
    }


def test_roll_back_k8s_release_uninstalls_one_that_was_not_installed():
    """A release that the deployment installed goes away again."""
    dep_config = DepConfig(
        namespace="mw-web",
        release="canary",
        cluster="eqiad",
        scope="train",
        mw_image_kind=None,
        mw_image_flavour="publish",
        web_image_flavour="webserver",
        deploy=True,
        fq_release_name="mw-web-canary-eqiad",
        values_file="/values/mw-web-canary-eqiad.yaml",
        helmfile_dir="/helmfile/mw-web",
    )
    ops = _ops_for_rollback({"mw-web-canary-eqiad": None})
    ops.runner.with_logger.return_value.roll_back.return_value = True

    with mock.patch("scap.kubernetes.monitor_release") as monitor:
        monitor.return_value.__enter__.return_value = scap.kubernetes.ReleaseMonitor()
        K8sOps._roll_back_k8s_release_for_cluster(ops, dep_config, None)

    # The revision of None is what makes roll_back() uninstall the release,
    # and the time of the record is what says that this deployment installed it.
    args = ops.runner.with_logger.return_value.roll_back.call_args[0]
    assert args[1] == "canary"
    assert args[2] == _recorded(None)


def test_get_stage_dep_configs_excludes_a_release_that_scap_does_not_deploy():
    """The releases that scap deploys at a stage.

    A release that scap does not deploy is not one of them, so it counts for no
    error-rate check and it does not roll back.
    """
    dep_configs = [
        DepConfig(
            namespace="mw-web",
            release=release,
            cluster="eqiad",
            scope="train",
            mw_image_kind=None,
            mw_image_flavour="publish",
            web_image_flavour="webserver",
            deploy=release != "maintenance",
            fq_release_name=f"mw-web-{release}-eqiad",
            values_file=f"/values/mw-web-{release}-eqiad.yaml",
            helmfile_dir="/helmfile/mw-web",
        )
        for release in ["main", "canary", "maintenance"]
    ]

    ops = mock.Mock()
    ops.app.config = {"deploy_mw_container_image": True}
    ops.k8s_deployments_config.deployed_stage_dep_configs.return_value = dep_configs

    deployed = K8sOps.get_stage_dep_configs(ops, "production")

    assert [dep_config.release for dep_config in deployed] == ["main", "canary"]

    # A run that deploys no container image has no such release.
    ops.app.config = {"deploy_mw_container_image": False}
    assert K8sOps.get_stage_dep_configs(ops, "production") == []


def test_rollout_was_abandoned():
    """k8s says when it stopped expecting the rollout to finish."""
    working = {"status": {"conditions": [{"type": "Progressing", "status": "True"}]}}
    assert not rollout_was_abandoned(working)

    gave_up = {
        "status": {
            "conditions": [
                {
                    "type": "Progressing",
                    "status": "False",
                    "reason": "ProgressDeadlineExceeded",
                }
            ]
        }
    }
    assert rollout_was_abandoned(gave_up)

    # k8s reports the deadline on a condition that it leaves True as well.
    exceeded = {
        "status": {
            "conditions": [
                {
                    "type": "Progressing",
                    "status": "True",
                    "reason": "ProgressDeadlineExceeded",
                }
            ]
        }
    }
    assert rollout_was_abandoned(exceeded)

    # k8s has not given up on a Deployment that holds no such condition.
    assert not rollout_was_abandoned({"status": {}})


def test_monitor_release_gives_up_when_k8s_stops(monkeypatch):
    """A rollout that k8s gave up on ends the count at once."""
    seen = []

    def new_replicas(kubeconfig, release, before, unchanged_is_done=False):
        seen.append(unchanged_is_done)
        # The same count each time, and k8s gave up on the rollout.
        return {"main": (2, False, True)}

    monkeypatch.setattr(scap.kubernetes, "_new_replicas_of_release", new_replicas)
    monkeypatch.setattr(scap.kubernetes, "deployments_of_release", lambda *a: [])

    reports = queue.Queue()
    started = time.monotonic()
    with scap.kubernetes.monitor_release("/etc/kubernetes/a", "main", reports, 60):
        pass

    assert time.monotonic() - started < 5


def test_monitor_release_of_a_deployment_that_changed_nothing(monkeypatch):
    """A sync that makes no new pod waits for nothing, and reports 100%."""

    unchanged = _k8s_object("shellbox.codfw.main", "5", available=3)
    listing = subprocess.CompletedProcess([], 0, json.dumps({"items": [unchanged]}), "")

    reports = queue.Queue()
    started = time.monotonic()
    with mock.patch("subprocess.run", return_value=listing):
        with scap.kubernetes.monitor_release("/etc/kubernetes/a", "main", reports, 60):
            pass

    # No wait: k8s says the rollout is complete, so the count ends at once,
    # and the last report counts the pods that are there.
    assert time.monotonic() - started < 5
    assert list(reports.queue)[-1] == ("shellbox.codfw.main", 3)


def test_monitor_release_of_a_deployment_that_failed():
    """A failure stops the report, because the count would describe the rollback.

    helm returns an atomic release to its prior revision, so the pods that are
    there are of neither revision that this deployment knows.
    """

    # One of the three pods that this Deployment wants is available.
    incomplete = _k8s_object("shellbox.codfw.main", "5", available=1)
    listing = subprocess.CompletedProcess(
        [], 0, json.dumps({"items": [incomplete]}), ""
    )

    reports = queue.Queue()
    started = time.monotonic()
    with mock.patch("subprocess.run", return_value=listing):
        with scap.kubernetes.monitor_release(
            "/etc/kubernetes/a", "main", reports, 60
        ) as monitor:
            monitor.ok = False

    # The failure ends the count at once, and it reports nothing.
    assert time.monotonic() - started < 5
    assert list(reports.queue) == []
