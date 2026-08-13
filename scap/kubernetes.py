import base64
from collections import defaultdict
import concurrent.futures
import contextlib
import copy
import dataclasses
from dataclasses import dataclass
from datetime import datetime, timezone
from dateutil import parser as dateutil_parser
import glob
import logging
import json
import os
import pathlib
import re
import shlex
import subprocess
import tempfile
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple
import queue

import yaml

from scap import utils, log, git, version, interaction
from scap.cli import Application
from scap.runcmd import gitcmd

# Deployment stages #

# Test servers are also referred to as mwdebug servers
TEST_SERVERS = "testservers"
CANARIES = "canaries"
PRODUCTION = "production"

STAGES = [TEST_SERVERS, CANARIES, PRODUCTION]
"""All supported deployment stages, ordered by scope (increasing)."""

LABEL_BUILDER_NAME = "vnd.wikimedia.builder.name"
LABEL_BUILDER_VERSION = "vnd.wikimedia.builder.version"
LABEL_SCAP_STAGE_DIR = "vnd.wikimedia.scap.stage_dir"
LABEL_SCAP_BUILD_STATE_DIR = "vnd.wikimedia.scap.build_state_dir"
LABEL_BUILD_TYPE = "vnd.wikimedia.build-type"
LABEL_PARENT_IMAGE = "vnd.wikimedia.parent-image"


class HelmfileError(utils.NoBacktraceError):
    """A helmfile command that scap needs did not complete."""


class DepConfigsFailed(Exception):
    """The releases that an operation of K8sOps did not complete."""

    def __init__(self, message: str, dep_configs: List["DepConfig"]):
        super().__init__(message)
        self.dep_configs = dep_configs


class InvalidDeploymentsConfig(utils.NoBacktraceError):
    """The deployments config that scap read is not valid."""


# The default production image kind producted by the image build process.
_DEFAULT_IMAGE_KIND = "image"

# The scope assigned to a deployment target when none is specified.
_DEFAULT_SCOPE = "train"


@dataclass
class _HelmfileReleaseValues:
    """Represents the helmfile values written by scap for a given MediaWiki-on-k8s release."""

    registry: str
    """The address of the docker registry."""
    mw_image_tag: str
    """The mediawiki app-image tag."""
    mw_metadata: dict
    """The metadata attributes describing the mediawiki app image."""
    web_image_tag: str
    """The httpd web-image tag."""
    web_metadata: dict
    """The metadata attributes describing the httpd web image."""

    def to_values(self) -> dict:
        """Returns a dict of helmfile values in the format expected by the mediawiki chart."""
        values = {
            "docker": {
                "registry": self.registry,
            },
            "main_app": {
                "image": self.mw_image_tag,
            },
            "mw": {
                "httpd": {
                    "image_tag": self.web_image_tag,
                }
            },
        }
        if "php_version" in self.mw_metadata:
            values["php"] = {"version": self.mw_metadata["php_version"]}
        return values


@dataclass(frozen=True)
class DepConfig:
    namespace: str
    # Helmfile release
    release: str
    cluster: str
    # The scope this target belongs to (e.g. "train" or "pretrain"). Used to
    # filter what scap deploys (see --scope) and to select supervision rules.
    scope: str

    mw_image_kind: Optional[str]
    mw_image_flavour: str
    web_image_flavour: str
    deploy: bool

    fq_release_name: str
    values_file: str
    helmfile_dir: str


# The exit status of "helmfile diff --detailed-exitcode" when no release has a
# change, and when a release has one.
DIFF_NO_CHANGES_EXIT_STATUS = 0
DIFF_CHANGES_EXIT_STATUS = 2


def objects_of_release(kubeconfig: str, kind: str, release: str) -> List[dict]:
    """Returns the k8s objects of one kind that belong to one helm release.

    The charts of deployment-charts label each object with the name of its
    release.

    Since this function is used for progress reporting (which is not critical),
    it returns an empty list if kubectl fails.
    """
    cmd = [
        "kubectl",
        "--kubeconfig",
        kubeconfig,
        "get",
        kind,
        "-l",
        f"release={release}",
        "-o",
        "json",
    ]
    ret = subprocess.run(cmd, text=True, capture_output=True)
    if ret.returncode != 0:
        return []

    return json.loads(ret.stdout).get("items", [])


def deployments_of_release(kubeconfig: str, release: str) -> List[dict]:
    """Returns the k8s Deployments of one helm release."""
    return objects_of_release(kubeconfig, "deployment", release)


def _revision_of(obj: dict) -> Optional[str]:
    """Returns the revision that k8s gave to a Deployment or to a ReplicaSet."""
    return (
        obj["metadata"].get("annotations", {}).get("deployment.kubernetes.io/revision")
    )


def rollout_is_complete(deployment: dict, new_replicas: int) -> bool:
    """Returns True if k8s finished the rollout of a Deployment.

    `new_replicas` is the available replicas of the new revision.
    """
    status = deployment["status"]

    return status.get("observedGeneration", 0) >= deployment["metadata"].get(
        "generation", 0
    ) and new_replicas >= deployment["spec"].get("replicas", 0)


def rollout_was_abandoned(deployment: dict) -> bool:
    """Returns True when k8s gave up on the rollout of a Deployment.

    k8s holds a Progressing condition for each Deployment, and gives it the
    reason ProgressDeadlineExceeded when it stops expecting the rollout to
    finish. Scap counts the replicas until then. A Deployment that holds no
    such condition yet is one that k8s has not given up on.
    """
    for condition in deployment["status"].get("conditions", []):
        if condition.get("type") == "Progressing":
            return (
                condition.get("status") != "True"
                or condition.get("reason") == "ProgressDeadlineExceeded"
            )

    return False


def _new_replicas_of_release(
    kubeconfig: str,
    release: str,
    revisions_before: Dict[str, Optional[str]],
    unchanged_is_done: bool = False,
) -> Dict[str, Tuple[int, bool, bool]]:
    """Returns the available replicas of the new pods of each Deployment.

    The count is of the new pods: a Deployment that keeps the revision it had
    before the deployment reports no replicas, and one that changed reports the
    available replicas of the ReplicaSet of its new revision. The pods from
    before the deployment therefore do not count as progress.

    `unchanged_is_done` counts a Deployment that keeps its revision as
    complete, for the report that a deployment makes at its end. That
    deployment made no new pod, so there is nothing to wait for (T375514).

    The second value of each entry says if k8s finished the rollout, which is
    when scap stops counting. The third value is True if k8s gave up on the
    rollout; This value is only meaningful when the second is False.
    """
    replicasets = objects_of_release(kubeconfig, "replicaset", release)

    reports = {}
    for deployment in deployments_of_release(kubeconfig, release):
        name = deployment["metadata"]["name"]
        revision = _revision_of(deployment)

        if revision is None or revision == revisions_before.get(name):
            # A deployment that changes no image makes no new pod, and the
            # pods that are there are the pods that it wants (T375514).
            available = deployment["status"].get("availableReplicas", 0)
            complete = rollout_is_complete(deployment, available)
            reports[name] = (
                available if unchanged_is_done and complete else 0,
                complete,
                rollout_was_abandoned(deployment),
            )
            continue

        available = 0
        for replicaset in replicasets:
            owners = replicaset["metadata"].get("ownerReferences", [])
            if _revision_of(replicaset) == revision and any(
                owner.get("name") == name for owner in owners
            ):
                available = replicaset["status"].get("availableReplicas", 0)
                break

        reports[name] = (
            available,
            rollout_is_complete(deployment, available),
            rollout_was_abandoned(deployment),
        )

    return reports


@dataclass
class ReleaseMonitor:
    ok: bool = True


# How many reads in a row may fail before scap stops counting the replicas of
# a release. kubectl reads nothing when it cannot reach the cluster.
READ_ATTEMPTS = 3


@contextlib.contextmanager
def monitor_release(
    kubeconfig: str,
    release: str,
    report_queue,
    freshness: int,
):
    """Periodically reports (to `report_queue`) the number of available replicas
    of the new pods of a release.

    Reports every `freshness` seconds, and then until k8s finishes every
    rollout of the release or gives up on one, because helm returns before
    every new pod is available (T375514).

    Does nothing when `report_queue` is None. A failure of kubectl skips one
    cycle, because a brief loss of connection to the cluster must not stop the
    deployment (T415839).
    """
    monitor = ReleaseMonitor()
    if report_queue is None:
        yield monitor
        return

    revisions_before = {
        deployment["metadata"]["name"]: _revision_of(deployment)
        for deployment in deployments_of_release(kubeconfig, release)
    }
    stop_event = threading.Event()

    def report(unchanged_is_done: bool = False) -> Tuple[bool, Optional[int], bool]:
        """Reports the replicas of the release.

        Returns whether k8s finished every rollout, how many replicas this
        report counted, and whether k8s gave up on every rollout. A count of
        None says that this cycle read nothing.
        """
        counts = _new_replicas_of_release(
            kubeconfig, release, revisions_before, unchanged_is_done
        )
        if revisions_before and not counts:
            # kubectl reported no Deployment for a release that has them, so
            # this cycle knows nothing. An empty report must not read as a
            # rollout that k8s finished (T415839).
            return (False, None, False)

        for name, (available, _, _) in counts.items():
            report_queue.put((name, available))

        return (
            all(complete for _, complete, _ in counts.values()),
            sum(available for available, _, _ in counts.values()),
            all(abandoned for _, _, abandoned in counts.values()),
        )

    def watch():
        while not stop_event.wait(timeout=freshness):
            report()
        # Reach here when the stop_event has been signaled.

        if not monitor.ok:
            # The command of the release failed, and helm returns an atomic
            # release to its prior revision. The Deployment then has a
            # revision that is neither the one from before nor the one that
            # this deployment wanted, so a count of its pods describes neither.
            # The report stops here instead.
            return

        # helm returns as soon as the new ReplicaSet of a Deployment has the
        # replicas that it wants, less the ones that its strategy allows to be
        # unavailable, so the last pods of a rollout arrive after the command
        # of the release returns. Scap counts them until k8s finishes every
        # rollout or gives up on one. A Deployment that changed nothing made no
        # new pod, so it is complete already. (T375514)
        unreadable = 0
        while True:
            complete, counted, abandoned = report(unchanged_is_done=True)
            if complete:
                return

            if counted is None:
                # kubectl read nothing. A brief loss of the connection to the
                # cluster must only skip the cycle (T415839).
                unreadable += 1
                if unreadable == READ_ATTEMPTS:
                    return
            elif abandoned:
                return
            else:
                unreadable = 0

            time.sleep(1)

    thread = threading.Thread(target=watch, name=f"k8s monitor {release}")
    thread.start()
    try:
        yield monitor
    finally:
        stop_event.set()
        thread.join()


@contextlib.contextmanager
def replica_progress(name: str, expected_replicas: int):
    """Reports the progress of a deployment, as its pods become available.

    Yields a queue. A monitor puts a (deployment name, available replicas) pair
    on it, and the reporter shows the sum of the last value of each deployment.
    Yields None when no replica is expected, because there is nothing to report.
    """
    if expected_replicas <= 0:
        yield None
        return

    report_queue = queue.Queue()

    def report():
        reports = {}
        reporter = log.reporter(name)
        reporter.expect(expected_replicas)
        reporter.start()

        while True:
            data = report_queue.get()
            if data == "stop":
                break

            deployment, available_replicas = data
            if reports.get(deployment) != available_replicas:
                reports[deployment] = available_replicas
                reporter.set_success(sum(reports.values()))

        reporter.finish()

    thread = threading.Thread(target=report, name="k8s deployment reporter")
    thread.start()
    try:
        yield report_queue
    finally:
        report_queue.put("stop")
        thread.join()


@dataclass(frozen=True)
class CommandJob:
    """One command to run, and the label that identifies it."""

    # The text that names this job to the user.
    label: str
    # The jobs of one group share a thread pool. The group is the k8s cluster,
    # so that scap makes a limited number of requests to each cluster.
    group: str
    directory: str
    command: List[str]
    # The name that the duration of the command reports to statsd.
    timer_name: str = "unsupplied"


@dataclass(frozen=True)
class CommandResult:
    """The outcome of one subprocess.

    `returncode` is None when the command gives no exit status of its own,
    because it did not start, or an interrupt stopped it. `stderr` then says
    what happened.
    """

    returncode: Optional[int]
    stdout: str
    stderr: str

    @property
    def ok(self) -> bool:
        return self.returncode == 0

    @property
    def status(self) -> str:
        """The exit status, for a report to the user."""
        if self.returncode is None:
            return "no exit status"
        return f"exit status {self.returncode}"

    @property
    def output(self) -> str:
        """The stdout and the stderr, for a report to the user."""
        return f"stdout: {self.stdout}\nstderr: {self.stderr}"


@dataclass(frozen=True)
class RecordedRevision:
    """The revision of a release before a deployment, and the time of the record.

    A revision of None means that helm reported no such release.
    """

    revision: Optional[int]
    recorded_at: datetime


@dataclass(frozen=True)
class ReleaseState:
    """What helm reports for an installed release, before a deployment changes it."""

    revision: int
    status: Optional[str]


@dataclass(frozen=True)
class HelmfileInvocation:
    """A helmfile directory, environment and release, and the commands for them.

    `release` limits each command to one release. If `release` is None,
    every release of the environment is targeted.
    """

    directory: str
    environment: str
    release: Optional[str]
    # The apply command runs a diff first, so both commands limit that diff to
    # this number of context lines. A failed deployment prints the diff, and
    # without the limit that output can reach tens of thousands of lines
    # (T424975).
    context_lines: int

    def apply_command(self) -> List[str]:
        """Returns the command that deploys the release."""
        return self._command("apply", "--context", str(self.context_lines))

    def diff_command(self) -> List[str]:
        """Returns the command that shows what the deployment changes.

        The exit status says whether a release has a change, so that scap does
        not have to read the format of the output.
        """
        return self._command(
            "diff",
            "--context",
            str(self.context_lines),
            "--detailed-exitcode",
            # The diffs reach the terminal and the logs, and a reviewer does not
            # need the secret values.
            "--suppress-secrets",
        )

    def build_command(self) -> List[str]:
        """Returns the command that prints the helmfile state as YAML."""
        return self._command("build")

    def list_command(self) -> List[str]:
        """Returns the command that lists the releases of the environment."""
        return self._command("list", "--output", "json")

    def write_values_command(self, output_file_template: str) -> List[str]:
        """Returns the command that writes the values of the release to a file."""
        return self._command(
            "write-values", "--output-file-template", output_file_template
        )

    def _command(self, subcommand: str, *arguments: str) -> List[str]:
        """Returns the command line of one helmfile invocation (as a list of strings)."""
        selector = ["--selector", f"name={self.release}"] if self.release else []
        return (
            ["helmfile", "-e", self.environment]
            + selector
            + [subcommand]
            + list(arguments)
        )

    def every_release(self) -> "HelmfileInvocation":
        """Returns this invocation, without the selector of one release."""
        return dataclasses.replace(self, release=None)

    def diff_job(self, label: str) -> CommandJob:
        """Returns the job that shows what the deployment changes."""
        return self._job(label, self.diff_command(), "helmfile_diff")

    def apply_job(self, label: str) -> CommandJob:
        """Returns the job that deploys the release."""
        return self._job(label, self.apply_command(), "helmfile_apply")

    def _job(self, label: str, command: List[str], timer_name: str) -> CommandJob:
        return CommandJob(label, self.environment, self.directory, command, timer_name)


def _helm_time(value: str) -> Optional[datetime]:
    """Returns the time of a helm timestamp, or None it cannot be parsed

    helm uses RFC 3339 with fractional seconds.

    >>> _helm_time("2026-08-12T20:09:48.018880843Z") == datetime(
    ...     2026, 8, 12, 20, 9, 48, 18880, timezone.utc
    ... )
    True
    >>> _helm_time("") is None
    True
    """
    try:
        return dateutil_parser.isoparse(value)
    except ValueError:
        return None


def helm_command(kubeconfig: str, subcommand: str, *arguments: str) -> List[str]:
    """Returns the command line of one helm invocation (as a list of strings).

    `kubeconfig` comes from K8sRunner.kubeconfig(). Each helm subcommand names
    its release in its own way, so the caller supplies that.
    """
    return ["helm", "--kubeconfig", kubeconfig, subcommand] + list(arguments)


def _cache_key(invocation: HelmfileInvocation) -> tuple:
    return (invocation.directory, invocation.environment, invocation.release)


def _kubeconfig_of(state: dict, invocation: HelmfileInvocation) -> str:
    """Returns the kubeconfig file of a helmfile state.

    Raises HelmfileError if the helmfile declares no kubeconfig.
    """
    for arg in state["helmDefaults"]["args"]:
        if re.search(r"/etc/kubernetes/", arg):
            return arg

    raise HelmfileError(
        f"The helmfile in {invocation.directory} declares no kubeconfig for "
        f"environment {invocation.environment}"
    )


def _helmfile_setting(state: dict, release: Optional[str], name: str):
    """Returns one setting of a release, or of helmDefaults, or None."""
    for entry in state.get("releases", []):
        if entry.get("name") == release and name in entry:
            return entry[name]

    return state.get("helmDefaults", {}).get(name)


def _wait_arguments(state: dict, release: str) -> List[str]:
    """Returns the arguments that make helm wait for the pods of a release.

    helmfile passes these to helm when it applies a release, and the timeout is
    the one that the helmfile declares. An atomic release waits as well,
    because helm sets --wait by itself for --atomic, and `helm rollback` has
    no --atomic of its own.
    """
    if not (
        _helmfile_setting(state, release, "wait")
        or _helmfile_setting(state, release, "atomic")
    ):
        return []

    timeout = _helmfile_setting(state, release, "timeout")
    return ["--wait"] + (["--timeout", f"{timeout}s"] if timeout else [])


class K8sRunner:
    """Runs the helmfile, helm and kubectl commands of one scap application.

    A HelmfileInvocation says which commands to run, and this class runs them.
    It holds the logger, the environment of each subprocess, and the config
    that says how many commands may run at the same time.
    """

    def __init__(self, app, logger, helm_env: Optional[dict] = None):
        self.app = app
        self.logger = logger
        self.env = helm_augmented_environment(helm_env)
        # What helmfile reports about an invocation, keyed by _cache_key().
        self._states = {}
        self._installed_releases = {}

    def with_logger(self, logger) -> "K8sRunner":
        """Returns a runner that reports to another logger."""
        other = copy.copy(self)
        other.logger = logger
        return other

    @property
    def max_workers_per_group(self) -> int:
        return self.app.config["k8s_max_concurrent_deployments_per_cluster"]

    @contextlib.contextmanager
    def group_pools(self, items: list, group_of: Callable[[Any], str]):
        """Yields a dict containing one thread pool for each group of the items.

        `group_of` is a function that returns the group of an item. The group
        is the k8s cluster, so that scap makes a limited number of requests to
        each one. The end of the block waits for the threads of every pool.
        """
        items_by_group = defaultdict(list)
        for item in items:
            items_by_group[group_of(item)].append(item)

        with contextlib.ExitStack() as stack:
            yield {
                group: stack.enter_context(
                    concurrent.futures.ThreadPoolExecutor(
                        max_workers=min(len(group_items), self.max_workers_per_group)
                    )
                )
                for group, group_items in items_by_group.items()
            }

    @contextlib.contextmanager
    def timer(self, description: str, name: str):
        """Times an operation, and reports the duration to statsd only."""
        quiet = logging.Logger("silence")
        quiet.addHandler(logging.NullHandler(level=0))
        with self.app.Timer(description, name=name, logger=quiet):
            yield

    def run(
        self, cmd: List[str], directory: str, env: Optional[dict] = None
    ) -> CommandResult:
        """Runs a command and returns its result. See run_command().

        `env` holds variables to add for this command only.
        """
        return run_command(cmd, directory, self.logger, {**self.env, **(env or {})})

    def stdout(self, cmd: List[str], directory: str) -> Optional[str]:
        """Runs a command and returns its stdout.

        Returns None if the exit status is not zero, and reports the failure.
        """
        result = self.run(cmd, directory)
        if result.ok:
            return result.stdout

        self.logger.error(
            f"Non-zero exit status ({result.returncode}) from "
            f"{utils.command_line(cmd, directory)}"
        )
        log.log_large_message(f"stdout: {result.stdout}", self.logger, logging.ERROR)
        log.log_large_message(f"stderr: {result.stderr}", self.logger, logging.ERROR)

        return None

    def run_jobs(
        self, jobs: List[CommandJob], failed=lambda result: not result.ok
    ) -> List[CommandResult]:
        """Runs each job in a subprocess. The jobs of one group share a pool.

        `failed` is a function which decides if a result is a failure. A diff
        needs its own function, because its exit status also reports a change.

        Returns the result of each job, in the order of `jobs`. A job that did
        not run gets a result with no exit status, which says why. Reports each
        job that failed.
        """

        def run(job) -> CommandResult:
            try:
                with self.timer(f"Running {job.label}", job.timer_name):
                    return self.run(job.command, job.directory)
            # A command that does not exist, or a directory that does not exist.
            except Exception as e:
                return CommandResult(None, "", f"{type(e).__name__}: {e}")

        with self.group_pools(jobs, lambda job: job.group) as pools:
            futures = [pools[job.group].submit(run, job) for job in jobs]

            try:
                results = [future.result() for future in futures]
            # BaseException also catches KeyboardInterrupt.
            except BaseException as e:
                self.logger.error(f"Caught {type(e).__name__} {e}")
                for pool in pools.values():
                    # Cancel the jobs that did not start yet, and wait for the
                    # jobs that are already in progress.
                    pool.shutdown(cancel_futures=True)
                results = [_finished_result(future) for future in futures]

        for job, result in zip(jobs, results):
            if failed(result):
                log_command_failure(
                    job.label, job.command, job.directory, result, self.logger
                )

        return results

    def review_diffs(self, jobs: List[CommandJob]) -> int:
        """Displays the diff of each job, then requests approval.

        Exits if the user does not approve. Returns the number of jobs that
        failed.
        """
        self.logger.info("Collecting helmfile diffs for review")
        results = self.run_jobs(jobs, failed=diff_failed)

        failures = 0
        changed = 0
        for job, result in zip(jobs, results):
            if diff_failed(result):
                failures += 1
            elif result.returncode == DIFF_CHANGES_EXIT_STATUS:
                changed += 1
                self.app.output_line(
                    f"=== Diff for {job.label} ===\n{result.stdout}",
                    sensitive=True,  # Diffs may contain sensitive values.
                )
            else:
                self.app.output_line(f"=== No changes for {job.label} ===")

        if failures:
            self.logger.error(f"{failures} of {len(jobs)} diff commands failed")
        if not changed:
            self.logger.info("None of the diffs show a change")

        self.app.prompt_for_approval_or_exit(
            "Note: Diffs are relative to the current helm charts and helmfile values. "
            "These may become outdated if new changes are merged.\n"
            "Continue with the deployment?",
            "Deployment cancelled.",
        )
        return failures

    def _cached(self, cache: dict, invocation: HelmfileInvocation, compute):
        """Returns what helmfile reports about an invocation, computed one time.

        Two threads that ask at the same moment both run the command, which
        costs one extra command and gives the same answer.
        """
        key = _cache_key(invocation)
        if key not in cache:
            cache[key] = compute()

        return cache[key]

    def state(self, invocation: HelmfileInvocation) -> dict:
        """Returns the helmfile state of an environment, as helmfile resolves it.

        The result is cached.

        Raises HelmfileError if helmfile fails. Scap reads the state to find the
        kubeconfig and to decide how to repair a release, so a state that scap
        cannot read makes every later step unreliable.
        """

        def read() -> dict:
            cmd = invocation.build_command()
            stdout = self.stdout(cmd, invocation.directory)
            if stdout is None:
                raise HelmfileError(
                    "Could not read the helmfile state: "
                    f"{utils.command_line(cmd, invocation.directory)}"
                )

            return yaml.safe_load(stdout)

        return self._cached(self._states, invocation, read)

    def kubeconfig(self, invocation: HelmfileInvocation) -> str:
        """Returns the kubeconfig file that helmfile gives to helm.

        Raises HelmfileError if the helmfile declares no kubeconfig.
        """
        return _kubeconfig_of(self.state(invocation), invocation)

    def installed_releases(self, invocation: HelmfileInvocation) -> List[str]:
        """Returns the releases that an environment installs.

        Uses `helmfile list -e <env>` to list every release of the helmfile,
        and selects those which have the `installed` and `enabled` flags set.

        The result is cached.

        Raises HelmfileError if helmfile fails.
        """

        def read() -> List[str]:
            cmd = invocation.list_command()
            stdout = self.stdout(cmd, invocation.directory)
            if stdout is None:
                raise HelmfileError(
                    "Could not list the releases: "
                    f"{utils.command_line(cmd, invocation.directory)}"
                )

            return [
                release["name"]
                for release in json.loads(stdout)
                if release.get("installed") and release.get("enabled")
            ]

        return self._cached(self._installed_releases, invocation, read)

    def releases(self, invocation: HelmfileInvocation) -> List[str]:
        """Returns the releases that an invocation deploys.

        An invocation that names no release deploys every release that the
        environment installs, so ask helmfile for those names.
        """
        if invocation.release:
            return [invocation.release]

        return self.installed_releases(invocation)

    def releases_state(self, invocation: HelmfileInvocation) -> Dict[str, ReleaseState]:
        """Returns helm release state for each release of a directory.

        Raises HelmfileError if scap cannot read the kubeconfig, if helm fails,
        or if the list has as many releases as the command asks for, because
        such a list may be truncated.
        """
        # `helm ls` asks for 256 releases without this. Note that `--max 0`
        # does not remove the limit: it asks for the default of the server.
        max_releases = 1000

        cmd = helm_command(
            self.kubeconfig(invocation),
            "ls",
            "-a",
            "-o",
            "json",
            f"--max={max_releases}",
        )
        stdout = self.stdout(cmd, invocation.directory)
        if stdout is None:
            raise HelmfileError(
                f"Could not list the releases: {utils.command_line(cmd, invocation.directory)}"
            )

        releases = json.loads(stdout)
        if len(releases) >= max_releases:
            # TODO: Pagination?
            raise HelmfileError(
                f"{utils.command_line(cmd, invocation.directory)} reported "
                f"{max_releases} releases, which might be truncated."
            )

        return {
            release["name"]: ReleaseState(
                int(release["revision"]), release.get("status")
            )
            for release in releases
        }

    def first_deployed(
        self, invocation: HelmfileInvocation, release: str
    ) -> Optional[datetime]:
        """Returns when helm first deployed a release, or None if unknown

        Raises HelmfileError if the kubeconfig cannot be read.
        """
        cmd = helm_command(self.kubeconfig(invocation), "status", release, "-o", "json")
        result = self.run(cmd, invocation.directory)
        if not result.ok:
            self.logger.warning(
                f"Could not get the status of release {release}: "
                f"{utils.command_line(cmd, invocation.directory)}: "
                f"{result.stderr.strip()}"
            )
            return None

        info = json.loads(result.stdout).get("info", {})
        return _helm_time(info.get("first_deployed", ""))

    def roll_back(
        self,
        invocation: HelmfileInvocation,
        release: str,
        recorded: RecordedRevision,
        label: str,
    ) -> bool:
        """Rolls back one release to the recorded revision. Returns True if successful.

        Raises HelmfileError if the kubeconfig cannot be read.
        """
        state = self.state(invocation)
        kubeconfig = _kubeconfig_of(state, invocation)
        wait = _wait_arguments(state, release)
        if recorded.revision is None:
            # Safeguard: Don't attempt to uninstall a release that was not installed in this run of scap.
            first_deployed = self.first_deployed(invocation, release)
            if first_deployed is None:
                self.logger.warning(
                    f"helm did not report when it first deployed {release}, for safety, not uninstalling it"
                )
                return True

            if first_deployed < recorded.recorded_at:
                self.logger.warning(
                    f"{release} was first deployed some time before this run, so not uninstalling it."
                )
                return True

            self.logger.info(
                f"{release} was first deployed during this run, so rolling back uninstalls it."
            )

            # --ignore-not-found, so that a second attempt also succeeds.
            cmd = helm_command(
                kubeconfig, "uninstall", release, "--ignore-not-found", *wait
            )
        else:
            cmd = helm_command(
                kubeconfig, "rollback", release, str(recorded.revision), *wait
            )

        with self.timer(f"Rolling back {label}", "helm_rollback"):
            result = self.run(cmd, invocation.directory)

        if result.ok:
            return True

        log_command_failure(label, cmd, invocation.directory, result, self.logger)
        return False

    def fix_pending_state(self, invocation: HelmfileInvocation, release: str) -> str:
        """Repairs a release that helm left in a pending state.

        A pending release accepts no upgrade and no rollback, so scap clears the
        state first. It uninstalls a pending install, because that release never
        completed, and it rolls back a pending upgrade or rollback.

        Returns the kubeconfig that helmfile gives to helm, for the caller to
        reuse. Raises HelmfileError if the repair fails.
        """
        kubeconfig = self.kubeconfig(invocation)
        state = self.releases_state(invocation).get(release)
        status = state.status if state else None
        self.logger.debug(
            f"Status is '{status}' for release {release} in {invocation.directory}"
        )

        recovery_commands = {
            "pending-install": "uninstall",
            "pending-upgrade": "rollback",
            "pending-rollback": "rollback",
        }
        recovery_command = recovery_commands.get(status)
        if not recovery_command:
            return kubeconfig

        self.logger.warning(
            f"Release {release} in {invocation.directory} is in {status} state. "
            "Attempting to clean up"
        )
        cmd = helm_command(kubeconfig, recovery_command, release)
        with self.timer(f"Repairing {release}", f"helm_{recovery_command}"):
            result = self.run(cmd, invocation.directory)

        if not result.ok:
            log_command_failure(
                f"The repair of {release} in {invocation.environment}",
                cmd,
                invocation.directory,
                result,
                self.logger,
            )
            raise HelmfileError(f"Could not repair the {status} release {release}")

        return kubeconfig

    def expected_replicas(self, invocations: List[HelmfileInvocation]) -> int:
        """Returns the number replicas that the releases of these invocations specify.

        The count comes from the values that helmfile renders, so it is the
        target of the deployment, and not what the cluster holds now. The charts
        of deployment-charts hold it in resources.replicas.

        Returns 0 when scap cannot read the values of a release, because a
        progress report is not worth stopping a deployment for. A caller that
        gets 0 reports no progress.
        """
        total = 0
        for invocation in invocations:
            values = self._rendered_values(invocation)
            if values is None:
                self.logger.warning(
                    "Scap reports no deployment progress, because it cannot read "
                    "the values of the releases"
                )
                return 0

            total += sum(
                release_values.get("resources", {}).get("replicas", 0)
                for release_values in values
            )

        return total

    def _rendered_values(self, invocation: HelmfileInvocation) -> Optional[List[dict]]:
        """Returns the values that helmfile renders, one entry for each release.

        Returns None when every attempt fails. Scap tries again after a delay,
        because the command updates the chart repositories before it renders,
        and that update can fail for a moment.
        """

        # How many times scap reads the values of the releases, and the delay
        # between two attempts, in seconds.
        attempts = 3
        retry_delay = 2

        def load_values(path: str) -> dict:
            with open(path) as f:
                return yaml.safe_load(f) or {}

        for attempt in range(1, attempts + 1):
            with tempfile.TemporaryDirectory() as output_dir:
                # helmfile expands this Go template one time for each release
                # that it writes.
                cmd = invocation.write_values_command(
                    os.path.join(output_dir, "{{ .Release.Name }}.yaml")
                )
                result = self.run(cmd, invocation.directory)
                files = sorted(glob.glob(os.path.join(output_dir, "*.yaml")))
                if result.ok and files:
                    return [load_values(path) for path in files]

            # A run that writes no file also counts as a failure, whatever its
            # exit status, so this is not log_command_failure().
            log.log_large_message(
                f"The values of the releases in {invocation.environment} are not "
                f"readable (attempt {attempt} of {attempts}): "
                f"{utils.command_line(cmd, invocation.directory)}\n{result.output}",
                self.logger,
                logging.WARNING,
            )
            if attempt < attempts:
                time.sleep(retry_delay)

        return None

    @contextlib.contextmanager
    def watch_releases(self, invocations: List[HelmfileInvocation]):
        """Reports the progress of the releases that these invocations deploy.

        Yields the monitors of each invocation, so that the caller can say how
        the command of that invocation ended.

        Raises HelmfileError if scap cannot read the kubeconfig or the releases
        of an invocation.
        """
        watched = [
            (invocation, self.kubeconfig(invocation), release)
            for invocation in invocations
            for release in self.releases(invocation)
        ]
        freshness = self.app.config["k8s_deployments_info_target_freshness"]

        with (
            replica_progress(
                "Deployment progress", self.expected_replicas(invocations)
            ) as report_queue,
            contextlib.ExitStack() as stack,
        ):
            monitors = defaultdict(list)
            for invocation, kubeconfig, release in watched:
                monitors[invocation].append(
                    stack.enter_context(
                        monitor_release(
                            kubeconfig,
                            release,
                            report_queue,
                            freshness,
                        )
                    )
                )
            yield monitors


@dataclass
class SupervisionCheck:
    """A single health check to run at a deployment stage."""


@dataclass
class LogstashCheck(SupervisionCheck):
    """An error-rate check evaluated against logstash.

    `threshold` is the error count at or above which the check fails. By default
    the check supervises the targets of its rule's scope at the current stage;
    `scope` and `stage` override which deployments it supervises (see DeploymentsConfig
    docstring).
    """

    threshold: int
    scope: Optional[str] = None
    stage: Optional[str] = None


@dataclass
class CommandsCheck(SupervisionCheck):
    """A list of shell commands to run (e.g. httpbb invocations)."""

    commands: List[str]


@dataclass
class SupervisionRule:
    """Binds a set of per-stage checks to a deployment scope.

    When scap deploys a target of `scope`, the checks in `stages[<stage>]` are
    run at that stage. The overall set of checks executed at a stage is the
    union across all rules whose scope was deployed in this run.
    """

    scope: str
    # Maps a deployment stage name to the checks to run at that stage.
    stages: Dict[str, List[SupervisionCheck]]


class DeploymentsConfig:
    """
    Represents the configuration of MediaWiki deployments for use by Scap.

    The deployments config is loaded from a YAML config file. Two top-level formats are
    supported:

      * Legacy: a bare list of deployment config items (as produced by the mw_releases
        hieradata value).
      * Mapping: a dict with a "deployment_targets" key holding the list of deployment
        config items, and an optional "supervision_rules" key (see below). This format
        allows supervision configuration to live alongside the deployment targets.

    Both formats are accepted so that scap continues to work before and after the source
    of truth (puppet's mw_releases) migrates to the mapping format.

    Each deployment config item has the following fields:
      namespace: A k8s namespace containing one or more helmfile releases
      scope: An arbitrary label grouping related targets (e.g. "train" or "pretrain").
        Used by the --scope command line argument to filter what scap deploys, and to
        select the applicable supervision rules. Optional (default: "train").
      releases: A mapping from helmfile release name to release configuration
      mw_kind: Default image kind for the MediaWiki image used by releases in this namespace. At a
        high level, an image kind is analogous to an image build target. This value must correspond
        an image kind built and published by the build process - e.g., cli-image or debug-image.
        Optional (default: if unset, use the default production image).
      mw_flavour: Default image flavour for the MediaWiki image used by releases in this namespace.
        At a high level, an image flavour corresponds to a set of image build arguments. This value
        must correspond to an image flavour used by the build process when building and publishing
        images for the specified image kind.
      web_flavour: Default image flavour for the httpd image used by releases in this namespace.
        This must correspond to an image flavour built and published by the build process.
      dir: The directory, under the configured helmfile_deployments_dir, in which the releases are
        found. Optional (default: the value of helmfile_default_cluster_dir).
      clusters: A list of k8s clusters where this namespace and associated releases are deployed.
        In practice, these align with the helmfile environment names relevant to these releases,
        which we use as a mechanism to select cluster-specific configuration. Optional (default:
        the value of k8s_clusters).

    The configuration of each release has the following fields:
      stage: Name of the stage in which this release should be updated - one of production, canary,
        testservers. Optional (default: production)
      mw_kind: Release-specific override to the namespace-level equivalent
      mw_flavour: Release-specific override to the namespace-level equivalent
      web_flavour: Release-specific override to the namespace-level equivalent
      deploy: Whether this release should be deployed by scap. If false, scap will manage only the
        helmfile release values files for this release. Optional (default: true)

    In the mapping format, an optional top-level "supervision_rules" key describes how to
    verify the health of deployed targets. It is a list of rules, each with:
      scope: When scap deploys a target of this scope, this rule's checks are run.
      stages: A mapping from deployment stage name (testservers, canaries, production) to a
        list of checks to run at that stage. Each check is a single-key mapping naming the
        check type ("logstash_check" or "commands"). The set of checks executed at a stage is
        the union across all rules whose scope was deployed in this run. At each stage, any
        "commands" checks run before the "logstash_check" checks, so that the error rate the
        latter measure reflects the traffic the commands generated.

    A "commands" check is a list of shell command strings to run (e.g. httpbb invocations).
    A "logstash_check" takes a "threshold" (error count) and optional "scope" and "stage"
    overrides. By default a check supervises the targets of its rule's scope that were deployed
    at the current stage. The overrides let a check instead supervise a different scope and/or
    stage - for example, a pretrain canary-stage check can supervise the train production fleet
    as a guardrail:

      supervision_rules:
        - scope: pretrain
          stages:
            canaries:
              - logstash_check: {threshold: 5}
              - logstash_check: {scope: train, stage: production, threshold: 150}
        - scope: train
          stages:
            testservers:
              - commands:
                  - "httpbb /srv/deployment/httpbb-tests/appserver/* --hosts=mwdebug.discovery.wmnet --https_port=4444 --retry_on_timeout"
                  - "httpbb /srv/deployment/httpbb-tests/appserver/* --hosts=mwdebug-next.discovery.wmnet --https_port=4453 --retry_on_timeout"
            canaries:
              - logstash_check: {threshold: 10}
            production:
              - logstash_check: {threshold: 150}

    When no supervision_rules are configured (e.g. the legacy format), scap falls back to the
    canary_threshold and production_error_threshold scap.cfg settings.

    Instances of this class translate that format into one that represents Scap workflow better by
    organizing the configurations around deployment stages.

    For example, the following input config file:

    - namespace: testservers
      releases:
        debug:
          stage: testservers
      mw_kind: debug-image
      mw_flavour: publish
      web_flavour: webserver
    - namespace: api1
      releases:
        main: {}
        canary:
          stage: canaries
      mw_flavour: publish
      web_flavour: webserver
    - namespace: api2
      releases:
        main: {}
        experimental:
          mw_flavour: publish-experimental
        maintenance:
          mw_kind: cli-image
          deploy: false
      mw_flavour: publish
      web_flavour: webserver
      dir: dse
      clusters: ["dse-eqiad"]

    will produce the following DeploymentsConfig.stages:

     {
      "testservers": [DepConfig(
        namespace="testservers",
        release="debug",
        cluster="default-cluster",
        scope="train",
        mw_image_kind="debug-image",
        mw_image_flavour="publish",
        web_image_flavour="webserver",
        deploy=True,
        fq_release_name="testservers-debug-default-cluster",
        values_file="/path/to/testservers-debug-default-cluster.yaml",
        helmfile_dir="/path/to/helmfile/testservers"
      )],
      "canaries": [DepConfig(
        namespace="api1",
        release="canary",
        cluster="default-cluster",
        scope="train",
        mw_image_kind=None,
        mw_image_flavour="publish",
        web_image_flavour="webserver",
        deploy=True,
        fq_release_name="api1-canary-default-cluster",
        values_file="/path/to/api1-canary-default-cluster.yaml",
        helmfile_dir="/path/to/helmfile/api1"
      )],
      "production": [DepConfig(
        namespace="api1",
        release="main",
        cluster="default-cluster",
        scope="train",
        mw_image_kind=None,
        mw_image_flavour="publish",
        web_image_flavour="webserver",
        deploy=True,
        fq_release_name="api1-main-default-cluster",
        values_file="/path/to/api1-main-default-cluster.yaml",
        helmfile_dir="/path/to/helmfile/api1"
        ), DepConfig(
        namespace="api2",
        release="main",
        cluster="dse-eqiad",
        scope="train",
        mw_image_kind=None,
        mw_image_flavour="publish",
        web_image_flavour="webserver",
        deploy=True,
        fq_release_name="api2-main-dse-eqiad",
        values_file="/path/to/api2-main-dse-eqiad.yaml",
        helmfile_dir="/path/to/helmfile/dse/api2"
        ), DepConfig(
        namespace="api2",
        release="experimental",
        cluster="dse-eqiad",
        scope="train",
        mw_image_kind=None,
        mw_image_flavour="publish-experimental",
        web_image_flavour="webserver",
        deploy=True,
        fq_release_name="api2-experimental-dse-eqiad",
        values_file="/path/to/api2-experimental-dse-eqiad.yaml",
        helmfile_dir="/path/to/helmfile/dse/api2"
        ), DepConfig(
        namespace="api2",
        release="maintenance",
        cluster="dse-eqiad",
        scope="train",
        mw_image_kind="cli-image",
        mw_image_flavour="publish",
        web_image_flavour="webserver",
        deploy=False,
        fq_release_name="api2-maintenance-dse-eqiad",
        values_file="/path/to/api2-maintenance-dse-eqiad.yaml",
        helmfile_dir="/path/to/helmfile/dse/api2"
      )]
     }

    Note that if a non-boolean value is provided for the `deploy` field, it is interpreted
    according to https://docs.python.org/3/library/stdtypes.html#truth-value-testing.

    For historical evolution of the deployments configuration YAML format, see:
    * https://phabricator.wikimedia.org/T299648
    * https://phabricator.wikimedia.org/T370934
    * https://phabricator.wikimedia.org/T387917
    * https://phabricator.wikimedia.org/T389499
    * https://phabricator.wikimedia.org/T388761
    """

    def __init__(
        self,
        testservers: List[DepConfig],
        canaries: List[DepConfig],
        production: List[DepConfig],
        supervision_rules: Optional[List[SupervisionRule]] = None,
        scope_filter: Optional[set] = None,
    ):
        self.stages = {
            TEST_SERVERS: testservers,
            CANARIES: canaries,
            PRODUCTION: production,
        }
        # None means "no supervision configured" (fall back to scap.cfg thresholds).
        self.supervision_rules = supervision_rules
        # The set of scopes that --scope restricts this run to. None means no
        # restriction (all scopes). This is the requested filter; scopes_deployed()
        # derives the scopes actually being deployed from it.
        self.scope_filter = scope_filter

    def deployed_stage_dep_configs(self, stage: str) -> List[DepConfig]:
        """
        Returns a list of DepConfigs for the specified stage, limited to the scopes
        being deployed.

        Used by deploy operations so that --scope restricts what scap touches.
        Includes non-deploy releases (whose helmfile values files scap still
        manages); filter on .deploy separately if needed.
        """
        dep_configs = self.stages[stage]
        if self.scope_filter is None:
            return dep_configs
        return [dc for dc in dep_configs if dc.scope in self.scope_filter]

    def supervised_dep_configs(self, scope: str, stage: str) -> List[DepConfig]:
        """Deployable DepConfigs of a given scope at a given stage.

        Used by supervision to determine which targets a check supervises. This is
        NOT limited by the scope_filter: a check may supervise a scope other than the
        one being deployed (e.g. a guardrail watching the train production fleet
        while deploying pretrain).
        """
        return [dc for dc in self.stages[stage] if dc.deploy and dc.scope == scope]

    def scopes_deployed(self) -> set:
        """The set of scopes actually deployed in this run.

        Restricted to scopes that have at least one deployable target, and further
        limited to the scope_filter when --scope was used.
        """
        present = {
            dc.scope
            for dep_configs in self.stages.values()
            for dc in dep_configs
            if dc.deploy
        }
        if self.scope_filter is None:
            return present
        return present & self.scope_filter

    def resolve_logstash_checks(self, stage: str) -> List[LogstashCheck]:
        """
        Return the logstash checks to run at a stage, de-duplicated.

        Collects, for the given deployment stage, every logstash check defined by a rule
        whose scope was deployed in this run. The returned LogstashCheck objects have
        inheritance already applied: each supervises a (scope, stage) pair inherited from
        its rule's scope and the current stage, unless the authored check overrode "scope"
        and/or "stage". So, unlike an as-authored check, a returned check always has a
        concrete scope and stage (never None). For example, at the pretrain canaries stage:

            {threshold: 5}                                     supervises (pretrain, canaries)
            {scope: train, stage: production, threshold: 150}  supervises (train, production)

        The second is a system-wide guardrail: while deploying pretrain canaries, it watches
        the train production fleet.

        Returns an empty list when no supervision rules are configured, or when no rule
        matches the deployed scopes at this stage.
        """
        if self.supervision_rules is None:
            return []

        deployed_scopes = self.scopes_deployed()

        resolved = []
        for rule in self.supervision_rules:
            if rule.scope in deployed_scopes:
                for check in rule.stages.get(stage, []):
                    if isinstance(check, LogstashCheck):
                        resolved_check = LogstashCheck(
                            threshold=check.threshold,
                            scope=check.scope or rule.scope,
                            stage=check.stage or stage,
                        )
                        if resolved_check not in resolved:
                            resolved.append(resolved_check)

        return resolved

    def resolve_command_checks(self, stage: str) -> List[str]:
        """
        Return the shell commands to run at a stage, in order and de-duplicated.

        Collects, for the given deployment stage, the commands from every "commands"
        check defined by a rule whose scope was deployed in this run. Unlike logstash
        checks, commands are plain shell strings with no scope/stage of their own.

        Returns an empty list when no supervision rules are configured, or when no rule
        defines commands for the deployed scopes at this stage.
        """
        if self.supervision_rules is None:
            return []

        deployed_scopes = self.scopes_deployed()

        commands = []
        for rule in self.supervision_rules:
            if rule.scope in deployed_scopes:
                for check in rule.stages.get(stage, []):
                    if isinstance(check, CommandsCheck):
                        for command in check.commands:
                            if command not in commands:
                                commands.append(command)

        return commands

    @classmethod
    def parse(
        cls, app, default_clusters: List[str], scope: Optional[set] = None
    ) -> "DeploymentsConfig":
        """Parse the deployments config file.

        `scope` limits the deployment to targets in the given set of scopes; None (the
        default) means all scopes.
        """
        testservers = []
        canaries = []
        production = []

        with open(app.config["k8s_deployments_file"]) as f:
            doc = yaml.safe_load(f)

        # Two top-level formats are supported (see class docstring): a bare list of
        # deployment targets (legacy), or a mapping with a "deployment_targets" key
        # and an optional "supervision_rules" key.
        if isinstance(doc, list):
            deployments = doc
            raw_supervision_rules = None
        elif isinstance(doc, dict):
            if "deployment_targets" not in doc:
                raise InvalidDeploymentsConfig(
                    'deployments config mapping is missing the "deployment_targets" key'
                )
            deployments = doc["deployment_targets"]
            raw_supervision_rules = doc.get("supervision_rules")
        else:
            raise InvalidDeploymentsConfig(
                "deployments config must be a list or a mapping"
            )

        namespaces = defaultdict(set)  # Key is cluster
        scopes_seen = set()

        for dep_config in deployments:
            namespace = dep_config["namespace"]
            dep_scope = dep_config.get("scope", _DEFAULT_SCOPE)
            scopes_seen.add(dep_scope)

            for cluster in dep_config.get("clusters", default_clusters):
                if namespace in namespaces[cluster]:
                    raise InvalidDeploymentsConfig(
                        f'"{namespace}" deployment is already defined for cluster "{cluster}"'
                    )
                namespaces[cluster].add(namespace)

                for release, config in dep_config["releases"].items():
                    mw_flavour = config.get("mw_flavour", dep_config.get("mw_flavour"))
                    if not mw_flavour:
                        raise InvalidDeploymentsConfig(
                            f'"{release}" in "{namespace}" has no mw_flavour set'
                        )
                    web_flavour = config.get(
                        "web_flavour", dep_config.get("web_flavour")
                    )
                    if not web_flavour:
                        raise InvalidDeploymentsConfig(
                            f'"{release}" in "{namespace}" has no web_flavour set'
                        )
                    stage = config.get("stage", PRODUCTION)
                    if stage not in STAGES:
                        raise InvalidDeploymentsConfig(
                            f'"{release}" in "{namespace}" specified unsupported stage "{stage}"'
                        )

                    mw_image_kind = config.get("mw_kind", dep_config.get("mw_kind"))
                    fq_release_name = f"{namespace}-{release}-{cluster}"
                    cluster_dir = dep_config.get(
                        "dir", app.config["helmfile_default_cluster_dir"]
                    )

                    parsed_dep_config = DepConfig(
                        namespace=namespace,
                        release=release,
                        cluster=cluster,
                        scope=dep_scope,
                        mw_image_kind=mw_image_kind,
                        mw_image_flavour=mw_flavour,
                        web_image_flavour=web_flavour,
                        deploy=bool(config.get("deploy", True)),
                        fq_release_name=fq_release_name,
                        values_file=os.path.join(
                            app.config["helmfile_mediawiki_release_dir"],
                            f"{fq_release_name}.yaml",
                        ),
                        helmfile_dir=os.path.join(
                            app.config["helmfile_deployments_dir"],
                            cluster_dir,
                            namespace,
                        ),
                    )

                    if stage == TEST_SERVERS:
                        testservers.append(parsed_dep_config)
                    elif stage == CANARIES:
                        canaries.append(parsed_dep_config)
                    else:
                        production.append(parsed_dep_config)

        if scope is None:
            scope_filter = None
        else:
            unknown = scope - scopes_seen
            if unknown:
                raise InvalidDeploymentsConfig(
                    "--scope {} matches no deployment targets".format(
                        ", ".join(sorted(unknown))
                    )
                )
            scope_filter = set(scope)

        supervision_rules = cls._parse_supervision_rules(raw_supervision_rules)

        return cls(
            testservers,
            canaries,
            production,
            supervision_rules,
            scope_filter,
        )

    @classmethod
    def _parse_supervision_rules(cls, raw_rules) -> Optional[List[SupervisionRule]]:
        if raw_rules is None:
            return None

        rules = []
        for raw_rule in raw_rules:
            scope = raw_rule.get("scope")
            if not scope:
                raise InvalidDeploymentsConfig("supervision rule is missing a scope")

            stages = {}
            for stage, raw_checks in raw_rule.get("stages", {}).items():
                if stage not in STAGES:
                    raise InvalidDeploymentsConfig(
                        f'supervision rule for scope "{scope}" references '
                        f'unsupported stage "{stage}"'
                    )
                stages[stage] = [
                    cls._parse_supervision_check(scope, stage, raw_check)
                    for raw_check in raw_checks
                ]

            rules.append(SupervisionRule(scope=scope, stages=stages))

        return rules

    @staticmethod
    def _parse_supervision_check(
        scope: str, stage: str, raw_check: dict
    ) -> SupervisionCheck:
        """Build a SupervisionCheck subclass from a single-key YAML mapping."""
        if len(raw_check) != 1:
            raise InvalidDeploymentsConfig(
                f'a supervision check for scope "{scope}" must name '
                "exactly one check type"
            )
        ((check_type, value),) = raw_check.items()

        if check_type == "logstash_check":
            # logstash_check measures an error rate under load; the testservers
            # stage runs commands only (see AbstractSync.check_testservers), so a
            # logstash_check there would be silently ignored - reject it.
            if stage == TEST_SERVERS:
                raise InvalidDeploymentsConfig(
                    f'logstash_check is not supported at the "{TEST_SERVERS}" '
                    f'stage (scope "{scope}")'
                )
            params = value or {}
            if not isinstance(params, dict) or "threshold" not in params:
                raise InvalidDeploymentsConfig(
                    f'logstash_check for scope "{scope}" is missing a threshold'
                )
            return LogstashCheck(
                threshold=params["threshold"],
                scope=params.get("scope"),
                stage=params.get("stage"),
            )

        if check_type == "commands":
            if not (
                isinstance(value, list) and all(isinstance(cmd, str) for cmd in value)
            ):
                raise InvalidDeploymentsConfig(
                    f'commands for scope "{scope}" must be a list of command strings'
                )
            return CommandsCheck(commands=value)

        raise InvalidDeploymentsConfig(
            f'unsupported supervision check type "{check_type}" for scope "{scope}"'
        )


class K8sOps:
    """
    MediaWiki Kubernetes deployment operations
    """

    def __init__(
        self,
        app: Application,
        suffix: str = "",
        update_releases_repo: bool = True,
        scope: Optional[set] = None,
    ):
        self.app = app
        self.suffix = suffix
        self.update_releases_repo = update_releases_repo
        # The set of deployment-target scopes scap deploys. None means all scopes.
        # Only affects Kubernetes deployments; bare-metal targets are unaffected.
        self.scope = scope
        self.logger = app.get_logger()
        self.default_clusters = re.split(r"[,\s]+", self.app.config["k8s_clusters"])
        self.traindev = self.default_clusters == ["traindev"]

        if app.config["build_mw_container_image"]:
            self._verify_build_and_push_prereqs()
        if app.config["deploy_mw_container_image"]:
            self._verify_deployment_prereqs()

        self.build_logfile = os.path.join(
            pathlib.Path.home(), "scap-image-build-and-push-log" + suffix
        )
        self.helm_env = collect_helm_env()
        self.runner = K8sRunner(app, self.logger, self.helm_env)
        self.original_helmfile_values = {}
        # The current revision of each release, before scap deploys.
        # Keyed by the fully qualified release name.
        self.rollback_revisions = {}
        self.build_state_dir = os.path.join(
            app.config["stage_dir"], "scap", "image-build" + suffix
        )
        if update_releases_repo and self.app.config["build_mw_container_image"]:
            release_repo_update_cmd = self.app.config["release_repo_update_cmd"]
            if release_repo_update_cmd:
                try:
                    self.logger.info("Running {}".format(release_repo_update_cmd))
                    with utils.suppress_backtrace():
                        subprocess.run(release_repo_update_cmd, shell=True, check=True)
                except Exception:
                    message = f"Could not update local release repository using: '${release_repo_update_cmd}'"
                    if not interaction.interactive():
                        utils.abort(message)

                    self.logger.error(message)
                    self.app.prompt_for_approval_or_exit(
                        prompt_message="Do you want to proceed anyway?",
                        exit_message="Canceled by user",
                        exit_code=1,
                    )

    def _common_build_images_args(self) -> List[str]:
        build_images_args = []

        if self.app.config["mediawiki_image_extra_packages"]:
            build_images_args += [
                "--mediawiki-image-extra-packages",
                self.app.config["mediawiki_image_extra_packages"],
            ]

        if self.app.config["mediawiki_image_extra_ca_cert"]:
            with open(self.app.config["mediawiki_image_extra_ca_cert"], "rb") as f:
                dev_ca_crt = base64.b64encode(f.read()).decode("utf-8")
                if dev_ca_crt:
                    build_images_args += ["--mediawiki-extra-ca-cert", dev_ca_crt]

        http_proxy = self.app.config["web_proxy"]
        if http_proxy:
            build_images_args += [
                "--http-proxy",
                http_proxy,
                "--https-proxy",
                http_proxy,
            ]

        return build_images_args

    def build_k8s_images(
        self,
        mediawiki_versions: list,
        force_version: bool = False,
        latest_tag: str = "latest",
    ):
        if not self.app.config["build_mw_container_image"]:
            return

        utils.mkdir_p(self.build_state_dir)

        make_container_image_dir = os.path.join(
            self.app.config["release_repo_dir"], "make-container-image"
        )
        registry = self.app.config["docker_registry"]

        mw_versions_list = ",".join(mediawiki_versions)
        stage_dir = self.app.config["stage_dir"]

        build_images_args = self._common_build_images_args() + [
            self.build_state_dir,
            "--staging-dir",
            stage_dir,
            "--mediawiki-versions",
            mw_versions_list,
            "--multiversion-image-basename",
            f"{registry}/{self.app.config['mediawiki_mv_image_basename']}",
            "--singleversion-image-basename",
            f"{registry}/{self.app.config['mediawiki_sv_image_basename']}",
            "--webserver-image-name",
            f"{registry}/{self.app.config['webserver_image_name']}",
            "--latest-tag",
            latest_tag,
            "--label",
            f"{LABEL_BUILDER_NAME}=scap",
            "--label",
            f"{LABEL_BUILDER_VERSION}={version.__version__}",
            "--label",
            f"{LABEL_SCAP_STAGE_DIR}={stage_dir}",
            "--label",
            f"{LABEL_SCAP_BUILD_STATE_DIR}={self.build_state_dir}",
        ]

        if force_version:
            if len(mediawiki_versions) > 1:
                raise ValueError(
                    "cannot force a single version if multiple versions are given"
                )

            build_images_args += [
                "--force-version",
                mediawiki_versions[0],
            ]

        if self.app.config["full_image_build"]:
            build_images_args.append("--full")

        with utils.suppress_backtrace():
            cmd = "{} {}".format(
                self.app.config["release_repo_build_and_push_images_cmd"],
                " ".join(map(shlex.quote, build_images_args)),
            )
            self.logger.info(
                "K8s images build/push output redirected to {}".format(
                    self.build_logfile
                )
            )
            self._run_cmd(
                cmd,
                make_container_image_dir,
                self.build_logfile,
                logging.getLogger("scap.k8s.build"),
                shell=True,
            )

    def _invocation(self, dep_config: DepConfig) -> HelmfileInvocation:
        """Returns a HelmfileInvocation corresponding to DepConfig"""
        return HelmfileInvocation(
            dep_config.helmfile_dir,
            dep_config.cluster,
            dep_config.release,
            self.app.config["k8s_helmfile_diff_context_lines"],
        )

    # Called by AbstractSync._confirm_k8s_diffs
    def diff_jobs_for_stage(self, stage: str) -> List[CommandJob]:
        """Returns the diff of each release that scap deploys at this stage.

        The jobs are sorted by cluster, namespace and release.
        """
        dep_configs = [
            dep_config
            for dep_config in self.k8s_deployments_config.deployed_stage_dep_configs(
                stage
            )
            if dep_config.deploy
        ]

        return [
            self._invocation(dep_config).diff_job(
                f"{dep_config.cluster}/{dep_config.namespace}-{dep_config.release} in {stage}"
            )
            for dep_config in sorted(
                dep_configs,
                key=lambda dep_config: (
                    dep_config.cluster,
                    dep_config.namespace,
                    dep_config.release,
                ),
            )
        ]

    # Called by AbstractSync.main()
    def review_diffs(self, jobs: List[CommandJob]) -> int:
        """Displays the diff of each job, then requests approval.

        Exits if the user does not approve. Returns the number of jobs that
        failed.
        """
        return self.runner.review_diffs(jobs)

    # Called by AbstractSync.main()
    def update_helmfile_files(self):
        if not self.app.config["deploy_mw_container_image"]:
            return

        # Collect helmfile values for all stages and releases, ensuring all
        # referenced image kinds and flavours exist prior to attempting updates.
        images_info = self._get_built_images_report()
        values = {}
        for stage in self.k8s_deployments_config.stages:
            dep_configs = self.k8s_deployments_config.deployed_stage_dep_configs(stage)
            values[stage] = self._collect_helmfile_values_for(dep_configs, images_info)

        for stage in self.k8s_deployments_config.stages:
            dep_configs = self.k8s_deployments_config.deployed_stage_dep_configs(stage)
            self.original_helmfile_values[stage] = self._read_helmfile_files(
                dep_configs
            )
            self._update_helmfile_files(dep_configs, values[stage])

    def revert_helmfile_files(self) -> None:
        """
        Reverts helmfile values files to their original state.

        The files hold the images of the revision that the rollback returns each
        release to, so the cluster and the files agree after a rollback.
        """
        if not self.app.config["deploy_mw_container_image"]:
            return

        for stage in self.k8s_deployments_config.stages:
            self._revert_helmfile_files(
                self.k8s_deployments_config.deployed_stage_dep_configs(stage),
                self.original_helmfile_values[stage],
            )

    # Called by AbstractSync.main()
    def deploy_k8s_images_for_stage(
        self, stage: str, for_rollback: bool = False
    ) -> Optional[str]:
        """
        Deploy the configured releases for a stage.

        When for_rollback is False, deploys the stage's configured releases and returns None.

        When for_rollback is True, returns each release of the stage to the
        revision that record_rollback_revisions recorded, and returns one of:
        * "nothing-to-roll-back-to" when the stage deploys no release
        * "rolled-back" when the releases of the stage rolled back
        """
        if not self.app.config["deploy_mw_container_image"]:
            return

        if for_rollback:
            # Every release that scap deploys at this stage rolls back, because
            # record_rollback_revisions recorded the revision of each one.
            dep_configs = self.get_stage_dep_configs(stage)
        else:
            dep_configs = self.k8s_deployments_config.deployed_stage_dep_configs(stage)

        self._deploy_to_clusters(dep_configs, for_rollback)

        if not for_rollback:
            return None

        if dep_configs:
            return "rolled-back"

        return "nothing-to-roll-back-to"

    def get_stage_dep_configs(self, stage: str) -> list:
        if not self.app.config["deploy_mw_container_image"]:
            return []

        return [
            dep_config
            # Limited to the scopes being deployed (see --scope), so a caller
            # sees only what scap deploys in this run.
            for dep_config in self.k8s_deployments_config.deployed_stage_dep_configs(
                stage
            )
            # Exclude non-deploy releases from contributing to the stage deployment list,
            # since they will not have been deployed in that stage.
            if dep_config.deploy
        ]

    def _read_helmfile_files(self, dep_configs: List[DepConfig]) -> dict:
        res = {}

        for dep_config in dep_configs:
            dep_config_values_file = dep_config.values_file
            if os.path.exists(dep_config_values_file):
                with open(dep_config_values_file) as f:
                    res[dep_config.fq_release_name] = yaml.safe_load(f)

        return res

    def _update_helmfile_files(
        self, dep_configs: List[DepConfig], helmfile_values: dict
    ) -> None:
        for dep_config in dep_configs:
            self._update_helmfile_values_for(
                dep_config, helmfile_values[dep_config.fq_release_name]
            )

    def _revert_helmfile_files(
        self, dep_configs: List[DepConfig], saved_values: dict
    ) -> None:
        """
        Reverts helmfile values files for the given dep_configs to the provided saved_values.

        A release with no saved values had no values file before the deployment,
        so its file goes away again.
        """
        commit = False

        with utils.cd(self.app.config["helmfile_mediawiki_release_dir"]):
            for dep_config in dep_configs:
                values_file = dep_config.values_file

                if dep_config.fq_release_name in saved_values:
                    values = saved_values[dep_config.fq_release_name]
                    utils.write_file_if_needed(values_file, yaml.dump(values))
                    if git.file_has_unstaged_changes(values_file):
                        gitcmd("add", values_file)
                        commit = True
                elif os.path.exists(values_file):
                    gitcmd("rm", values_file)
                    commit = True

            if commit:
                gitcmd("commit", "-m", "Configuration(s) reverted")

    # Called by AbstractSync.main()
    def record_rollback_revisions(self) -> None:
        """Records the revision of every release that scap deploys in this run.

        Every release records the revision it had at the same moment, before the
        first stage deployed.

        Raises HelmfileError if helm fails, which stops the run before it
        changes the cluster.
        """
        if not self.app.config["deploy_mw_container_image"]:
            return

        self._record_rollback_revisions(
            [
                dep_config
                for stage in self.k8s_deployments_config.stages
                for dep_config in self.get_stage_dep_configs(stage)
            ]
        )

    def _record_rollback_revisions(self, dep_configs: List[DepConfig]) -> None:
        """Records the current revision of each release, before the deployment.

        Non-deploy releases are excluded.

        Raises HelmfileError if helm fails, which stops the run before it
        changes anything.
        """
        recorded_at = datetime.now(timezone.utc)
        by_directory = {}
        revisions = {}

        for dep_config in dep_configs:
            if not dep_config.deploy:
                continue

            invocation = self._invocation(dep_config)
            key = (invocation.directory, invocation.environment)
            if key not in by_directory:
                # One read of a directory serves the releases of every stage.
                by_directory[key] = self.runner.releases_state(invocation)

            state = by_directory[key].get(dep_config.release)
            # A release that is not installed records None (meaning rollback
            # will uninstall it).
            revisions[dep_config.fq_release_name] = RecordedRevision(
                state.revision if state else None, recorded_at
            )

        self.rollback_revisions = revisions

    def _deploy_to_clusters(
        self, dep_configs: List[DepConfig], for_rollback: bool
    ) -> None:
        self._foreach_depconfig(
            dep_configs,
            (
                self._roll_back_k8s_release_for_cluster
                if for_rollback
                else self._deploy_k8s_images_for_cluster
            ),
            "Rollback" if for_rollback else "Deployment",
        )

    def _start_deployment_reporter(self, progress, dep_configs: List[DepConfig]):
        return replica_progress(
            "K8s deployment progress",
            (
                self.runner.expected_replicas(
                    [self._invocation(dep_config) for dep_config in dep_configs]
                )
                if progress
                else 0
            ),
        )

    def _foreach_depconfig(
        self,
        dep_configs: List[DepConfig],
        func,
        description,
        progress=True,
    ):
        """
        Invokes 'func' over all dep_configs.

        'func' must take two arguments: dep_config, report_queue.

        Note: Invocations of 'func' are concurrent and so must be threadsafe.

        'description' should be a human-readable description of the action
        performed by func (for diagnostic output / error handling).

        If 'progress' is True, a progress indicator will be displayed
        during the operation.

        Non-deploy releases are excluded.

        Returns: A list of values returned by func.
        """

        dep_configs = [dep_config for dep_config in dep_configs if dep_config.deploy]

        if not dep_configs:
            return []

        with self._start_deployment_reporter(progress, dep_configs) as report_queue:
            with self.runner.group_pools(
                dep_configs, lambda dep_config: dep_config.cluster
            ) as pools:
                futures = []
                results = []
                failed = []
                messages = []

                for dep_config in dep_configs:
                    future = pools[dep_config.cluster].submit(
                        func, dep_config, report_queue
                    )
                    future._scap_dep_config = dep_config
                    futures.append(future)

                for future in concurrent.futures.as_completed(futures):
                    exception = future.exception()
                    if exception:
                        failed.append(future._scap_dep_config)
                        messages.append(
                            "{} of {} failed: {}".format(
                                description,
                                future._scap_dep_config.fq_release_name,
                                exception,
                            )
                        )
                    else:
                        results.append(future.result())

            if failed:
                raise DepConfigsFailed(
                    f"K8s {description} had the following errors:\n "
                    + "\n".join(messages),
                    failed,
                )

            return results

    def _deploy_k8s_images_for_cluster(self, dep_config: DepConfig, report_queue):
        logger = logging.getLogger("scap.k8s.deploy")

        release = dep_config.release
        namespace = dep_config.namespace
        cluster = dep_config.cluster
        helmfile_dir = dep_config.helmfile_dir

        kubeconfig = self.runner.with_logger(logger).fix_pending_state(
            self._invocation(dep_config), dep_config.release
        )

        cmd = self._invocation(dep_config).apply_command()
        runner = self.runner.with_logger(logger)

        with monitor_release(
            kubeconfig,
            release,
            report_queue,
            self.app.config["k8s_deployments_info_target_freshness"],
        ) as monitor:
            with runner.timer(
                f"Deploying {dep_config.fq_release_name}",
                f"helmfile_apply_{namespace}_{release}_{cluster}",
            ):
                # helmfile_log_sal.sh (operations/puppet) reads SUPPRESS_SAL.
                # Scap makes its own SAL entries for a MediaWiki deployment.
                result = runner.run(cmd, helmfile_dir, {"SUPPRESS_SAL": "true"})

            monitor.ok = result.ok

        if not result.ok:
            log_command_failure(
                dep_config.fq_release_name, cmd, helmfile_dir, result, logger
            )
            raise Exception(f"The deployment of {dep_config.fq_release_name} failed")

    def _roll_back_k8s_release_for_cluster(self, dep_config: DepConfig, report_queue):
        """
        Rolls back one release to the revision it had before the deployment.
        """
        logger = logging.getLogger("scap.k8s.deploy")

        release = dep_config.release
        recorded = self.rollback_revisions[dep_config.fq_release_name]

        kubeconfig = self.runner.with_logger(logger).fix_pending_state(
            self._invocation(dep_config), dep_config.release
        )

        with monitor_release(
            kubeconfig,
            release,
            report_queue,
            self.app.config["k8s_deployments_info_target_freshness"],
        ) as monitor:
            monitor.ok = self.runner.with_logger(logger).roll_back(
                self._invocation(dep_config),
                release,
                recorded,
                dep_config.fq_release_name,
            )

        if not monitor.ok:
            raise Exception(f"The rollback of {dep_config.fq_release_name} failed")

    def _verify_build_and_push_prereqs(self):
        if self.app.config["release_repo_dir"] is None:
            raise SystemExit(
                "release_repo_dir must be configured when build_mw_container_image is True"
            )

    def _verify_deployment_prereqs(self):
        if self.app.config["release_repo_dir"] is None:
            raise SystemExit(
                "release_repo_dir must be configured when deploy_mw_container_image is True"
            )

        self.k8s_deployments_config = DeploymentsConfig.parse(
            self.app, self.default_clusters, self.scope
        )

    def _collect_helmfile_values_for(
        self, dep_configs: List[DepConfig], images_info: dict
    ) -> dict:
        registry = self.app.config["docker_registry"]

        def strip_registry(fqin):
            registry_prefix = registry + "/"
            if fqin.startswith(registry_prefix):
                return fqin[len(registry_prefix) :]

            return fqin

        def find_image_and_metadata(flavours, flavour, image_kind=None):
            if flavour not in flavours:
                raise ValueError(
                    f"Image flavour '{flavour}' not found among built images in {flavours}"
                )

            kinds = flavours[flavour]["kinds"]

            if image_kind is None:
                image_kind = _DEFAULT_IMAGE_KIND

            if image_kind not in kinds:
                raise ValueError(
                    f"Image kind '{image_kind}' not found among built images for flavour "
                    f"'{flavour}' in {kinds}"
                )

            return (
                kinds[image_kind]["image_name"],
                flavours[flavour]["metadata"],
            )

        values = {}

        # An exception raised by find_image_and_metadata indicates a misconfiguration, in which
        # case, a backtrace is not useful.
        with utils.suppress_backtrace():
            for dep_config in dep_configs:
                mw_img, mw_metadata = find_image_and_metadata(
                    images_info["mediawiki"]["flavours"],
                    dep_config.mw_image_flavour,
                    image_kind=dep_config.mw_image_kind,
                )

                web_img, web_metadata = find_image_and_metadata(
                    images_info["webserver"]["flavours"],
                    dep_config.web_image_flavour,
                )

                values[dep_config.fq_release_name] = _HelmfileReleaseValues(
                    registry=registry,
                    mw_image_tag=strip_registry(mw_img),
                    mw_metadata=mw_metadata,
                    web_image_tag=strip_registry(web_img),
                    web_metadata=web_metadata,
                )

        return values

    def _update_helmfile_values_for(
        self, dep_config: DepConfig, helmfile_values: _HelmfileReleaseValues
    ):
        """
        Note: Due to the git operations and change of the working directory, this function
        is not thread safe.
        """

        values = helmfile_values.to_values()

        # Train-dev hack.  This is to override the mw-web canary-values.yaml
        # (which is read after values-traindev.yaml) which sets replicas to a
        # value suitable for production but too high for train-dev.
        # k8s_traindev_replicas raises it, to watch a deployment of several
        # replicas.
        if self.traindev:
            values["resources"] = {"replicas": self.app.config["k8s_traindev_replicas"]}

        values_file = dep_config.values_file
        utils.write_file_if_needed(values_file, yaml.dump(values))
        with utils.cd(self.app.config["helmfile_mediawiki_release_dir"]):
            if git.file_has_unstaged_changes(values_file):
                msg = (
                    "Updating release '%s'\n\n"
                    "MediaWiki image is: '%s'\n"
                    "Webserver image is: '%s'"
                ) % (
                    dep_config.fq_release_name,
                    helmfile_values.mw_image_tag,
                    helmfile_values.web_image_tag,
                )

                gitcmd("add", values_file)
                gitcmd("commit", "-m", msg)

    def _get_built_images_report(self) -> dict:
        """
        Return a data structure containing the fully qualified image names of the
        images most recently built by build_k8s_images().
        """

        report_file = os.path.join(self.build_state_dir, "report.json")

        with open(report_file) as f:
            report = json.load(f)

        return report

    def _run_cmd(self, cmd, dir, logfile, logger, shell=False, env=None):
        """
        Runs a subprocess, logging its output at debug level unless the
        subprocess failed (exited non-zero) in which case the output is
        logged at error level.
        """
        try:
            with open(logfile, "w") as logstream:
                logger.debug("Running {} in {}".format(cmd, dir))
                subprocess.run(
                    cmd,
                    shell=shell,
                    check=True,
                    cwd=dir,
                    stdout=logstream,
                    stderr=subprocess.STDOUT,
                    env=env,
                )
            with open(logfile) as logstream:
                log.log_large_message(logstream.read(), logger, logging.DEBUG)
        except subprocess.CalledProcessError as e:
            # Print the error message, which contains the command that was executed and its
            # exit status.
            logger.error(e)
            logger.error("Stdout/stderr follows:")
            with open(logfile) as logstream:
                log.log_large_message(logstream.read(), logger, logging.ERROR)
            raise

    # T331479


def collect_helm_env() -> dict:
    """Returns the HELM_* variables that /etc/profile.d/kube-env.sh sets.

    The result is empty if the host does not have that file.
    """
    env = dict()

    filename = "/etc/profile.d/kube-env.sh"
    if not os.path.exists(filename):
        return env

    vars = ["HELM_HOME", "HELM_CONFIG_HOME", "HELM_DATA_HOME", "HELM_CACHE_HOME"]

    cmd = f"source {filename}"

    for var in vars:
        cmd += f" && echo {var}=${var}"

    cmd = ["bash", "-c", cmd]
    output = subprocess.check_output(cmd, text=True)

    for line in output.splitlines():
        m = re.match(r"([^=]+)=(.*)$", line)
        if not m:
            raise RuntimeError(f"Unexpected output from {cmd}:\n{output}")

        env[m[1]] = m[2]

    return env


def log_command_failure(
    label: str, cmd: List[str], directory: str, result: CommandResult, logger
):
    """Reports a command that failed, with a command line that the user can run."""
    log.log_large_message(
        f"{label} failed ({result.status}): "
        f"{utils.command_line(cmd, directory)}\n{result.output}",
        logger,
        logging.ERROR,
    )


def diff_failed(result: CommandResult) -> bool:
    """Returns True if a diff did not run, or did not complete."""
    return result.returncode not in (
        DIFF_NO_CHANGES_EXIT_STATUS,
        DIFF_CHANGES_EXIT_STATUS,
    )


def _finished_result(future) -> CommandResult:
    """Returns the result of a job, or a result that says why there is none."""
    if future.cancelled():
        return CommandResult(None, "", "The command did not start")
    if not future.done():
        return CommandResult(None, "", "The command did not finish")
    try:
        return future.result()
    except BaseException as e:
        return CommandResult(None, "", f"{type(e).__name__}: {e}")


def helm_augmented_environment(helm_env: Optional[dict] = None) -> dict:
    """Returns a copy of the environment of scap, with the HELM_* variables.

    `helm_env` holds the HELM_* variables from collect_helm_env(), and comes
    from a new call to collect_helm_env() when the caller supplies none. The
    result is a new dictionary, which the caller can change.
    """
    res = os.environ.copy()
    res.update(collect_helm_env() if helm_env is None else helm_env)
    return res


def run_command(cmd, dir, logger, env: dict) -> CommandResult:
    """Runs cmd in a subprocess in the specified directory.

    Logs at DEBUG level only, so that the caller controls the report of a
    failure. `env` is the full environment of the process.

    The result holds all of the output in memory, so a command that writes a
    very large amount of output needs another way. `helmfile apply` and
    `helmfile diff` keep their output small with --context (T424975).
    """
    # This is similiar to scap.runcmd._runcmd() except:
    # * a specific logger can be supplied
    # * stdout/stderr are logged (debug level)

    logger.debug("Running {} in {}".format(cmd, dir))
    proc = subprocess.Popen(
        cmd,
        cwd=dir,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        env=env,
    )
    stdout, stderr = proc.communicate()
    log.log_large_message(f"stdout: {stdout}", logger, logging.DEBUG)
    log.log_large_message(f"stderr: {stderr}", logger, logging.DEBUG)
    logger.debug("exit status: %s", proc.returncode)

    return CommandResult(proc.returncode, stdout, stderr)


def build_states(state_dir):
    """
    Yields each image build state stored in the given directory.
    """
    for path in glob.glob(os.path.join(state_dir, "*-state.json")):
        try:
            with open(path, "r") as f:
                yield json.load(f)
        except OSError:
            pass


def built_image_ids() -> List[str]:
    """
    Return the IDs of all local images built by scap.
    """
    image_ids = set()

    cmd = [
        "docker",
        "image",
        "ls",
        "--filter",
        f"label={LABEL_BUILDER_NAME}=scap",
        "--format",
        "{{.ID}}",
    ]

    with subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True) as proc:
        for line in proc.stdout:
            image_ids.add(line.rstrip())

    return list(image_ids)


def inspect_images(image_ids: list) -> list:
    """
    Return details of the given images.
    """
    if not image_ids:
        return []
    cmd = ["docker", "image", "inspect", *image_ids]
    with subprocess.Popen(cmd, stdout=subprocess.PIPE) as proc:
        return json.load(proc.stdout)


@utils.log_context("kubernetes.prune_local_images")
def prune_local_images(logger=None, dry_run=False):
    """
    Untag/remove local images that were built during k8s deployment but are no
    longer referenced by any of the state files used for incremental builds.

    Note that `docker image rm` only deletes the image if there are no longer
    any tags associated with it, which is what we want, so we provide the
    command image refs instead of image IDs.
    """
    verbose_level = logging.INFO if dry_run else logging.DEBUG

    images = inspect_images(built_image_ids())

    images_by_name = {}
    state_dirs = set()
    skipped = 0
    deleted = 0
    untagged = 0

    for image in images:
        for ref in image["RepoTags"]:
            images_by_name[ref] = image

        labels = image["Config"]["Labels"]
        state_dir = labels.get(LABEL_SCAP_BUILD_STATE_DIR)
        if state_dir:
            state_dirs.add(state_dir)

    def mark_image(image):
        nonlocal skipped

        if image.get("marked") is True:
            return

        image["marked"] = True
        skipped += 1

        labels = image["Config"]["Labels"]
        if labels.get(LABEL_BUILD_TYPE) == "incremental":
            parent = labels.get(LABEL_PARENT_IMAGE)
            if parent:
                logger.log(
                    verbose_level,
                    "Skipped %s due to being a parent of skipped image %s",
                    parent,
                    image["RepoTags"],
                )
                mark_image_by_name(parent)

    def mark_image_by_name(name):
        image = images_by_name.get(name)
        if image:
            mark_image(image)

    for state_dir in state_dirs:
        for state in build_states(state_dir):
            last_image = state.get("last_image")
            if last_image:
                logger.log(
                    verbose_level,
                    "Skipped %s due to references in %s",
                    last_image,
                    state_dir,
                )
                mark_image_by_name(last_image)

    refs_to_remove = set()
    for image in images:
        if image.get("marked") is True:
            continue
        for ref in image["RepoTags"]:
            refs_to_remove.add(ref)

    if refs_to_remove and not dry_run:
        with log.pipe(logger=logger, level=logging.DEBUG) as debug:
            with subprocess.Popen(
                ["docker", "image", "rm", *refs_to_remove],
                stderr=debug,
                stdout=subprocess.PIPE,
            ) as proc:
                for line in proc.stdout:
                    os.write(debug, line)
                    if line.startswith(b"Deleted: "):
                        deleted += 1
                    elif line.startswith(b"Untagged: "):
                        untagged += 1

    if logger:
        logger.info("Untagged %d unused refs", untagged)
        logger.info("Deleted %d image layers", deleted)
        logger.info("Skipped %d used refs", skipped)
