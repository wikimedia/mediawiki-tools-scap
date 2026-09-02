# -*- coding: utf-8 -*-
"""
    scap.deploy_service
    ~~~~~~~~~~~~~~~~~~~
    Deployment of a Kubernetes service with helmfile.

    Copyright © 2026-2026 Wikimedia Foundation and Contributors.

    This file is part of Scap.

    Scap is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, version 3.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""
import os
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Dict, List, Optional

import yaml

from scap import checks as scap_checks
from scap import cli, kubernetes, utils

STAGING = "STAGING"
PRODUCTION = "PRODUCTION"
# The order in which environment types are deployed.
ENVIRONMENT_TYPES = [STAGING, PRODUCTION]


class InvalidDeployServiceConfig(utils.NoBacktraceError):
    """The service catalog or the cluster groups that scap read are not valid."""


@dataclass(frozen=True)
class Environment:
    """A helmfile environment, as defined by a cluster group."""

    name: str
    datacenter: str
    type: str  # One of ENVIRONMENT_TYPES
    aliases: List[str] = field(default_factory=list)


@dataclass(frozen=True)
class ClusterGroup:
    """A named set of helmfile environments sharing a set of deployment stages."""

    name: str
    # The directory under helmfile_deployments_dir holding this group's services.
    dir: str
    # Keyed by environment name, in the order the environments were declared.
    environments: Dict[str, Environment]
    # The stages to deploy, in order.
    environment_stages: List[str]
    # The stage that is used when a release does not declare a stage.
    # It must be one of environment_stages.
    environment_default_stage: str
    # Maps each environment name and alias to the environment it refers to.
    _index: Dict[str, Environment]

    def resolve(self, ref: str, what: str) -> Environment:
        """Return the environment named or aliased by `ref`.

        `what` says where the reference comes from, so that the error names the
        part of the config to repair.
        """
        if ref not in self._index:
            raise InvalidDeployServiceConfig(
                f"{what[:1].upper()}{what[1:]} names environment '{ref}', which "
                f"cluster group {self.name} has no environment or alias for "
                f"(known: {', '.join(sorted(self._index))})"
            )
        return self._index[ref]


@dataclass(frozen=True)
class Release:
    """A helmfile release selected for deployment at a given stage."""

    name: str
    stage: str


@dataclass(frozen=True)
class ServiceEnvironment:
    """An environment a service deploys to, and the releases to deploy there."""

    # The name used to refer to the environment in the service config. This may be an
    # alias, and is what gets passed to `helmfile -e`.
    ref: str
    environment: Environment
    # An empty list means "all releases".
    releases: List[Release]


@dataclass(frozen=True)
class ServiceNamespace:
    name: str
    # Keyed by canonical environment name.
    environments: Dict[str, ServiceEnvironment]


@dataclass(frozen=True)
class DeploymentGroup:
    """One environment of a cluster group at one stage."""

    environment: str  # The canonical name of the environment of the cluster group.
    stage: str

    def __str__(self) -> str:
        return f"{self.environment} at stage {self.stage}"


@dataclass(frozen=True)
class GroupChecks:
    """The checks that run after one group of the plan deploys."""

    # The shell commands, in the order they were declared.
    commands: List[str] = field(default_factory=list)
    # The questions of the manual checks, in the order they were declared. A
    # deployer answers each one after the commands pass. An empty question
    # takes the name of the group.
    manual: List[str] = field(default_factory=list)


Checks = Dict[DeploymentGroup, GroupChecks]


@dataclass(frozen=True)
class ServiceConfig:
    """The deployment config of a single service, as declared in the service catalog."""

    name: str
    cluster_group: ClusterGroup
    # In the order the namespaces were declared.
    namespaces: List[ServiceNamespace]
    # In the order they were declared.
    checks: Checks = field(default_factory=dict)


@dataclass(frozen=True)
class HelmfileCommand(kubernetes.HelmfileInvocation):
    """One step of the plan: a helmfile invocation, and where it belongs.

    The environment is the name that the service config uses, which may be an
    alias of the environment of the cluster group.
    """

    environment_type: str
    stage: str
    namespace: str

    @property
    def label(self) -> str:
        """The text that identifies this step of the plan to the user."""
        target = self.namespace
        if self.release:
            target = f"{self.release} of {target}"
        return f"{target} in {self.environment} ({self.environment_type}/{self.stage})"

    def release_label(self, release: str) -> str:
        """The text that identifies one release of this step to the user."""
        if release == self.release:
            return self.label
        return f"{release} of {self.label}"

    def diff_job(self) -> kubernetes.CommandJob:
        """Returns the review job of this step of the plan."""
        return super().diff_job(self.label)

    def __str__(self) -> str:
        return utils.command_line(self.apply_command(), self.directory)


Plan = Dict[DeploymentGroup, List[HelmfileCommand]]


@dataclass(frozen=True)
class Rollback:
    """One release to return to the state it had before the deployment.

    `recorded` holds the revision of the release before the deployment, and the
    time of that record. A revision of None is a release that was not
    installed, and the rollback of such a release uninstalls it.
    """

    command: HelmfileCommand
    release: str
    recorded: kubernetes.RecordedRevision

    @property
    def label(self) -> str:
        """The text that identifies this rollback to the user."""
        what = self.command.release_label(self.release)
        if self.recorded.revision is None:
            return f"{what} (uninstall)"

        return f"{what} to revision {self.recorded.revision}"


def _assert_mapping(value, what: str) -> dict:
    if not isinstance(value, dict):
        raise InvalidDeployServiceConfig(f"{what} must be a mapping")
    return value


def _assert_nonempty_string(value, what: str) -> str:
    if not isinstance(value, str) or not value:
        raise InvalidDeployServiceConfig(f"{what} must be a non-empty string")
    return value


def _assert_no_unexpected_keys(value: dict, expected: List[str], what: str):
    unexpected = [key for key in value if key not in expected]
    if unexpected:
        raise InvalidDeployServiceConfig(
            f"{what} has unexpected key(s): {', '.join(map(str, unexpected))} (expected: {', '.join(expected)})"
        )


def _load_yaml_file(path: str, what: str):
    try:
        with open(path) as f:
            doc = yaml.safe_load(f)
    except OSError as e:
        raise InvalidDeployServiceConfig(f"could not read {what} {path}: {e}")

    if doc is None:
        raise InvalidDeployServiceConfig(f"{what} {path} is empty")

    return doc


def _parse_environment(group_name: str, name: str, doc) -> Environment:
    what = f"environment {name} of cluster group {group_name}"
    doc = _assert_mapping(doc, what)
    _assert_no_unexpected_keys(doc, ["datacenter", "type", "aliases"], what)

    env_type = _assert_nonempty_string(doc.get("type"), f"the type of {what}")
    if env_type not in ENVIRONMENT_TYPES:
        raise InvalidDeployServiceConfig(
            f"the type of {what} must be one of {', '.join(ENVIRONMENT_TYPES)}, not '{env_type}'"
        )

    aliases = doc.get("aliases", [])
    if not isinstance(aliases, list):
        raise InvalidDeployServiceConfig(f"the aliases of {what} must be a list")
    for alias in aliases:
        _assert_nonempty_string(alias, f"each alias of {what}")

    return Environment(
        name=name,
        datacenter=_assert_nonempty_string(
            doc.get("datacenter"), f"the datacenter of {what}"
        ),
        type=env_type,
        aliases=aliases,
    )


def _parse_environment_stages(group_name: str, doc) -> List[str]:
    what = f"the environment_stages of cluster group {group_name}"
    stages = doc.get("environment_stages")
    if not isinstance(stages, list) or not stages:
        raise InvalidDeployServiceConfig(f"{what} must be a non-empty list")

    for stage in stages:
        _assert_nonempty_string(stage, f"each entry of {what}")
    if len(set(stages)) != len(stages):
        raise InvalidDeployServiceConfig(f"{what} contains duplicate entries")

    return stages


def _parse_environment_default_stage(group_name: str, doc, stages: List[str]) -> str:
    what = f"the environment_default_stage of cluster group {group_name}"
    stage = _assert_nonempty_string(doc.get("environment_default_stage"), what)
    if stage not in stages:
        raise InvalidDeployServiceConfig(
            f"{what} must be one of its environment_stages ({', '.join(stages)}), not '{stage}'"
        )
    return stage


def _parse_cluster_group(name: str, doc, default_dir: str) -> ClusterGroup:
    what = f"cluster group {name}"
    doc = _assert_mapping(doc, what)
    _assert_no_unexpected_keys(
        doc,
        ["dir", "environments", "environment_stages", "environment_default_stage"],
        what,
    )

    environments = _assert_mapping(
        doc.get("environments"), f"the environments of {what}"
    )
    if not environments:
        raise InvalidDeployServiceConfig(f"{what} defines no environments")

    parsed = {}
    index = {}
    for env_name, env_doc in environments.items():
        env_name = _assert_nonempty_string(env_name, f"each environment name of {what}")
        env = _parse_environment(name, env_name, env_doc)
        parsed[env_name] = env
        for ref in [env_name] + env.aliases:
            if ref in index:
                raise InvalidDeployServiceConfig(
                    f"'{ref}' refers to more than one environment of {what}"
                )
            index[ref] = env

    stages = _parse_environment_stages(name, doc)

    return ClusterGroup(
        name=name,
        dir=_assert_nonempty_string(doc.get("dir", default_dir), f"the dir of {what}"),
        environments=parsed,
        environment_stages=stages,
        environment_default_stage=_parse_environment_default_stage(name, doc, stages),
        _index=index,
    )


def load_cluster_groups(path: str, default_dir: str) -> Dict[str, ClusterGroup]:
    """Load cluster group definitions, keyed by cluster group name.

    `default_dir` is the directory used by groups that declare no `dir`.
    """
    doc = _assert_mapping(
        _load_yaml_file(path, "cluster groups file"), f"the cluster groups file {path}"
    )
    return {
        _assert_nonempty_string(
            name, f"each cluster group name in {path}"
        ): _parse_cluster_group(name, group_doc, default_dir)
        for name, group_doc in doc.items()
    }


def load_primary_datacenter(path: str) -> str:
    """Load the name of the primary datacenter from the conftool state file."""
    doc = _assert_mapping(
        _load_yaml_file(path, "conftool state file"), f"the conftool state file {path}"
    )
    mw = _assert_mapping(
        doc.get("mw"), f"the mw section of the conftool state file {path}"
    )
    return _assert_nonempty_string(
        mw.get("primary_dc"), f"mw.primary_dc of the conftool state file {path}"
    )


def _parse_releases(what: str, doc, default_stage: str) -> List[Release]:
    releases = doc.get("releases")
    if releases is None:
        # No releases means all releases.
        return []

    releases = _assert_mapping(releases, f"the releases of {what}")
    if not releases:
        raise InvalidDeployServiceConfig(
            f"the releases of {what} is empty; omit the key to deploy all releases"
        )

    return [
        _parse_release(
            what,
            _assert_nonempty_string(name, f"each release name of {what}"),
            release_doc,
            default_stage,
        )
        for name, release_doc in releases.items()
    ]


def _parse_release(env_what: str, name: str, doc, default_stage: str) -> Release:
    what = f"release {name} of {env_what}"
    doc = _assert_mapping(doc, what)
    _assert_no_unexpected_keys(doc, ["stage"], what)
    return Release(
        name=name,
        stage=_assert_nonempty_string(
            doc.get("stage", default_stage), f"the stage of {what}"
        ),
    )


def _parse_namespace(
    service_what: str, name: str, doc, cluster_group: ClusterGroup
) -> ServiceNamespace:
    what = f"namespace {name} of {service_what}"
    doc = _assert_mapping(doc, what)
    _assert_no_unexpected_keys(doc, ["environments"], what)

    environments = _assert_mapping(
        doc.get("environments"), f"the environments of {what}"
    )
    if not environments:
        raise InvalidDeployServiceConfig(f"{what} defines no environments")

    parsed = {}
    for ref, env_doc in environments.items():
        ref = _assert_nonempty_string(ref, f"each environment name of {what}")
        env = cluster_group.resolve(ref, f"the environments of {what}")
        if env.name in parsed:
            raise InvalidDeployServiceConfig(
                f"'{ref}' and '{parsed[env.name].ref}' both refer to environment {env.name} of {what}"
            )

        env_what = f"environment {ref} of {what}"
        env_doc = _assert_mapping(env_doc, env_what)
        _assert_no_unexpected_keys(env_doc, ["releases"], env_what)

        releases = _parse_releases(
            env_what, env_doc, cluster_group.environment_default_stage
        )
        for release in releases:
            if release.stage not in cluster_group.environment_stages:
                raise InvalidDeployServiceConfig(
                    f"the stage of release {release.name} of {env_what} must be one of "
                    f"{', '.join(cluster_group.environment_stages)} (the environment_stages of cluster group "
                    f"{cluster_group.name}), not '{release.stage}'"
                )

        parsed[env.name] = ServiceEnvironment(
            ref=ref, environment=env, releases=releases
        )

    return ServiceNamespace(name=name, environments=parsed)


def _parse_checks(service_what: str, doc, cluster_group: ClusterGroup) -> Checks:
    if doc is None:
        return {}

    what = f"the checks of {service_what}"
    doc = _assert_mapping(doc, what)

    checks = {}
    refs = {}
    for ref, raw_stages in doc.items():
        ref = _assert_nonempty_string(ref, f"each environment name of {what}")
        env = cluster_group.resolve(ref, what)
        if env.name in refs:
            raise InvalidDeployServiceConfig(
                f"'{ref}' and '{refs[env.name]}' both refer to environment "
                f"{env.name} of {what}"
            )
        refs[env.name] = ref

        env_what = f"environment {ref} of {service_what}"
        for stage, raw_checks in _assert_mapping(raw_stages, env_what).items():
            if stage not in cluster_group.environment_stages:
                raise InvalidDeployServiceConfig(
                    f"{env_what} names stage '{stage}', which cluster group "
                    f"{cluster_group.name} does not define "
                    f"(known: {', '.join(cluster_group.environment_stages)})"
                )

            checks[DeploymentGroup(env.name, stage)] = _parse_group_checks(
                f"the checks of stage {stage} of {env_what}", raw_checks
            )

    return checks


def _parse_group_checks(what: str, raw_checks) -> GroupChecks:
    """Returns the checks of one environment at one stage.

    Duplicate commands are removed.
    """
    if not isinstance(raw_checks, list):
        raise InvalidDeployServiceConfig(f"{what} must be a list")

    commands = []
    manual = []
    for raw_check in raw_checks:
        check = _assert_mapping(raw_check, f"each check of {what}")
        if len(check) != 1:
            raise InvalidDeployServiceConfig(
                f"each check of {what} must name exactly one check type"
            )

        ((check_type, value),) = check.items()
        if check_type == "commands":
            if not (
                isinstance(value, list) and all(isinstance(cmd, str) for cmd in value)
            ):
                raise InvalidDeployServiceConfig(
                    f"the commands of {what} must be a list of command strings"
                )

            for command in value:
                if command not in commands:
                    commands.append(command)
        elif check_type == "manual":
            if value is not None and not isinstance(value, str):
                raise InvalidDeployServiceConfig(
                    f"the manual check of {what} must be the question to ask, "
                    "or nothing"
                )

            manual.append(value or "")
        else:
            raise InvalidDeployServiceConfig(
                f"{what} names unsupported check type '{check_type}' "
                "(expected: commands, manual)"
            )

    return GroupChecks(commands, manual)


def _parse_service(
    catalog_what: str, name: str, doc, cluster_groups: Dict[str, ClusterGroup]
) -> ServiceConfig:
    what = f"service {name} of {catalog_what}"
    doc = _assert_mapping(doc, what)
    _assert_no_unexpected_keys(doc, ["cluster_group", "checks", "namespaces"], what)

    group_name = _assert_nonempty_string(
        doc.get("cluster_group"), f"the cluster_group of {what}"
    )
    if group_name not in cluster_groups:
        raise InvalidDeployServiceConfig(
            f"the cluster_group of {what} refers to undefined cluster group '{group_name}' "
            f"(known: {', '.join(sorted(cluster_groups))})"
        )
    cluster_group = cluster_groups[group_name]

    namespaces = _assert_mapping(doc.get("namespaces"), f"the namespaces of {what}")
    if not namespaces:
        raise InvalidDeployServiceConfig(f"{what} defines no namespaces")

    return ServiceConfig(
        name=name,
        cluster_group=cluster_group,
        checks=_parse_checks(what, doc.get("checks"), cluster_group),
        namespaces=[
            _parse_namespace(
                what,
                _assert_nonempty_string(ns_name, f"each namespace name of {what}"),
                ns_doc,
                cluster_group,
            )
            for ns_name, ns_doc in namespaces.items()
        ],
    )


def load_service_catalog(
    path: str, cluster_groups: Dict[str, ClusterGroup]
) -> Dict[str, ServiceConfig]:
    """Load the deployment config of every service that may be deployed.

    The catalog is the source of truth for which services deploy-service accepts.
    """
    what = f"the service catalog {path}"
    doc = _assert_mapping(_load_yaml_file(path, "service catalog"), what)
    _assert_no_unexpected_keys(doc, ["services"], what)

    services = _assert_mapping(doc.get("services"), f"the services of {what}")
    if not services:
        raise InvalidDeployServiceConfig(f"{what} defines no services")

    return {
        _assert_nonempty_string(name, f"each service name of {what}"): _parse_service(
            what, name, service_doc, cluster_groups
        )
        for name, service_doc in services.items()
    }


def load_catalog(config) -> Dict[str, ServiceConfig]:
    """Load the service catalog that the scap config names."""
    return load_service_catalog(
        config["service_catalog_file"],
        load_cluster_groups(
            config["cluster_groups_file"],
            config["helmfile_default_cluster_dir"],
        ),
    )


def _environments_to_deploy(
    service_config: ServiceConfig, environment_type: str, primary_datacenter: str
) -> List[Environment]:
    """Returns Environments of the given type referenced by the service, in deployment order.

    Environments in the primary datacenter are deployed last. Otherwise the
    declaration order of the cluster group is preserved.
    """
    envs = [
        env
        for env in service_config.cluster_group.environments.values()
        if env.type == environment_type
        and any(env.name in ns.environments for ns in service_config.namespaces)
    ]
    return [env for env in envs if env.datacenter != primary_datacenter] + [
        env for env in envs if env.datacenter == primary_datacenter
    ]


def plan(
    service_config: ServiceConfig,
    primary_datacenter: str,
    deployments_dir: str,
    context_lines: int,
) -> Plan:
    steps = {}
    cluster_group = service_config.cluster_group

    for environment_type in ENVIRONMENT_TYPES:
        for stage in cluster_group.environment_stages:
            for env in _environments_to_deploy(
                service_config, environment_type, primary_datacenter
            ):
                concurrent = []
                for namespace in service_config.namespaces:
                    service_env = namespace.environments.get(env.name)
                    if service_env is None:
                        continue

                    for release in _releases_to_deploy(
                        service_env, stage, cluster_group.environment_default_stage
                    ):
                        concurrent.append(
                            HelmfileCommand(
                                directory=os.path.join(
                                    deployments_dir, cluster_group.dir, namespace.name
                                ),
                                environment=service_env.ref,
                                release=release,
                                context_lines=context_lines,
                                environment_type=environment_type,
                                stage=stage,
                                namespace=namespace.name,
                            )
                        )

                if concurrent:
                    steps[DeploymentGroup(env.name, stage)] = concurrent

    return steps


def _releases_to_deploy(
    service_env: ServiceEnvironment, stage: str, default_stage: str
) -> List[Optional[str]]:
    """Returns the releases of an environment to deploy at this stage.

    An entry of None deploys every release of the environment.
    """
    if not service_env.releases:
        # An environment with no releases deploys all of its releases, at the
        # cluster group's default stage.
        return [None] if stage == default_stage else []

    return [release.name for release in service_env.releases if release.stage == stage]


def _namespaces_of(group: List[HelmfileCommand]) -> List[str]:
    """The namespaces of one group, in the order of the plan.

    A namespace with more than one release at this stage appears one time.
    """
    return list(dict.fromkeys(command.namespace for command in group))


@cli.command(
    "deploy-service",
    help="Deploy a service to Kubernetes",
)
class DeployService(cli.Application):
    @cli.argument(
        "--confirm-diffs",
        action="store_true",
        help="Display the helmfile diffs and request approval before the deployment.",
    )
    @cli.argument(
        "--dry-run",
        action="store_true",
        help="Display the helmfile commands of the deployment, but run none of them.",
    )
    @cli.argument("service", help="Name of the service to deploy")
    @cli.argument("message", nargs="*", help="Log message that says why")
    def main(self, *extra_args):
        self.logger = self.get_logger()
        self.k8s = kubernetes.K8sRunner(self, self.logger)

        service = self.arguments.service
        service_config = self.service_config = self._service_config(service)
        primary_datacenter = load_primary_datacenter(self.config["conftool_state_file"])

        self.logger.info(
            f"Deploying {service} using cluster group "
            f"{service_config.cluster_group.name} (primary datacenter: {primary_datacenter})"
        )

        steps = plan(
            service_config,
            primary_datacenter,
            self.config["helmfile_deployments_dir"],
            self.config["k8s_helmfile_diff_context_lines"],
        )
        if not steps:
            self.logger.warning(f"Nothing to deploy for {service}")
            return 0

        self._assert_releases_exist(steps)
        self._assert_checks_run(steps)

        if self.arguments.dry_run:
            self._show_plan(steps)
            return 0

        # The message is the reason of the lock, for whoever waits for it.
        self.prompt_for_message()

        with self.lock(self._lock_file(service)):
            soft_errors = False
            if self.arguments.confirm_diffs:
                soft_errors = self._confirm_diffs(steps) > 0

            status = self._deploy_and_announce(service, steps)

        if status == 0 and soft_errors:
            self.logger.warning("The deployment is complete, but a diff command failed")
            return 1
        return status

    def _lock_file(self, service: str) -> str:
        """The lock file of one service.

        Each service has its own lock, so two separate services can deploy at the same time.
        """
        return os.path.join(
            self.config["lock_dir"],
            f"scap.deploy-service.{service.replace('/', '_')}.lock",
        )

    def _service_config(self, service: str) -> ServiceConfig:
        """Returns the config of one service of the catalog."""
        catalog_file = self.config["service_catalog_file"]
        catalog = load_catalog(self.config)
        if service not in catalog:
            raise InvalidDeployServiceConfig(
                f"'{service}' is not a service in {catalog_file} "
                f"(known: {', '.join(sorted(catalog))})"
            )

        return catalog[service]

    def _assert_releases_exist(self, plan: Plan):
        """Checks that each release of the catalog exists in its environment.

        `helmfile --selector name=<release>` selects nothing when the name is
        not one that the environment installs, so a name with a mistake would
        stop the deployment at that step, after the earlier steps deployed.
        """
        catalog_file = self.config["service_catalog_file"]

        for group in plan.values():
            for command in group:
                if command.release is None:
                    continue

                installed = self.k8s.installed_releases(
                    command.without_release_selector()
                )
                if command.release not in installed:
                    raise InvalidDeployServiceConfig(
                        f"{catalog_file} names release '{command.release}' of "
                        f"{command.namespace} in {command.environment}, which that "
                        f"environment does not install "
                        f"(it installs: {', '.join(installed)})"
                    )

    def _show_plan(self, plan: Plan):
        """Displays the plan, one group of commands at a time.

        The commands of one group are indented under the group that holds them,
        to show that they run at the same time.
        """
        for deployment_group, group in plan.items():
            first = group[0]
            at_once = f", {len(group)} at once" if len(group) > 1 else ""
            self.logger.info(
                f"[{first.environment_type}/{first.stage}] {first.environment}{at_once}:"
            )
            for command in group:
                self.logger.info(f"    {command}")

            checks = self._checks_of(deployment_group)
            for command in checks.commands:
                self.logger.info(f"    check: {command}")

            for question in checks.manual:
                self.logger.info(f"    manual: {question or '(no question)'}")

    def _deploy_and_announce(self, service: str, plan: Plan) -> int:
        """Deploys the service, and announces the start and the end.

        The announcements bracket the deployment. The SAL entries that helmfile
        makes for each apply are between them.
        """
        what = f"scap deploy-service {service}: {self.message_argument}"
        self.announce(f"Started {what}")
        started = time.time()

        def announce_end(outcome: str):
            duration = utils.human_duration(time.time() - started)
            self.announce_final(f"{outcome} {what} (duration: {duration})")

        try:
            status = self._deploy(plan)
        # BaseException, so that an interrupt also ends the announcements.
        except BaseException:
            announce_end("Failed")
            raise

        announce_end("Finished" if status == 0 else "Failed")
        return status

    def _deploy(self, plan: Plan) -> int:
        """Deploys the service, one group of the plan at a time.

        The steps of one group deploy at the same time. A failure stops the
        deployment and offers to roll back what was already deployed.
        Returns the exit status of scap.
        """
        # What a rollback must restore, one list for each group of the plan, in
        # the order that the groups deployed.
        rollbacks = []

        for deployment_group, group in plan.items():
            first = group[0]
            self.logger.info(
                f"[{first.environment_type}/{first.stage}] Deploying "
                f"{', '.join(_namespaces_of(group))} "
                f"in {first.environment}"
            )
            # The revisions come from before the deployment, so that a rollback
            # names the revision to return to. A release in a pending state
            # accepts no deployment, so scap repairs it here.
            group_rollbacks = [self._prepare(command) for command in group]

            # Every release of the group rolls back, and not only the releases
            # that deployed: helm undoes an atomic release that fails, but that
            # rollback of helm can fail as well. A rollback that names the
            # revision returns a release to the same revision each time, so it
            # is safe to roll back a release that helm already returned.
            rollbacks.append(
                [
                    rollback
                    for step_rollbacks in group_rollbacks
                    for rollback in step_rollbacks
                ]
            )

            answer = self._apply_until_it_works(group)
            if answer is None:
                answer = self._check_until_it_works(deployment_group, group)

            if answer is None:
                continue

            self.logger.error(f"Deployment of {self.arguments.service} stopped")
            if answer == "b":
                self._roll_back(rollbacks)
            else:
                self.logger.warning("What already deployed stays as it is")
            return 1

        self.logger.info(f"Deployment of {self.arguments.service} complete")
        return 0

    def _apply_until_it_works(self, group: List[HelmfileCommand]) -> Optional[str]:
        """Deploys the steps of one group, and offers a retry of what failed.

        Returns None when every step of the group deployed, and the choice of
        the user when one did not. When the session is not interactive, scap
        rolls back, as it does for a MediaWiki deployment.
        """
        first = group[0]
        pending = list(group)
        attempts = 0

        def apply_pending() -> bool:
            nonlocal pending, attempts
            attempts += 1
            if attempts > 1:
                self.logger.info(
                    f"Deploying again: {', '.join(c.label for c in pending)}"
                )
                # An apply that stopped in the middle can leave a release in a
                # pending state. Clean up first.
                for command in pending:
                    for release in self.k8s.releases(command):
                        self._repair(command, release)

            results = self._apply(pending)
            pending = [
                command for command, result in zip(pending, results) if not result.ok
            ]
            return not pending

        return self.retry_until(
            f"the deployment of {', '.join(_namespaces_of(group))} "
            f"in {first.environment}",
            apply_pending,
            {
                "Roll back what was already deployed": "b",
                "Exit without rolling back": "x",
            },
            self._prompt_default("r", "b"),
        )

    def _assert_checks_run(self, plan: Plan):
        """Checks that each DeploymentGroup of the checks is in the plan."""
        for deployment_group in self.service_config.checks:
            if deployment_group not in plan:
                raise InvalidDeployServiceConfig(
                    f"the checks of service {self.service_config.name} of "
                    f"{self.config['service_catalog_file']} name "
                    f"{deployment_group}, which deploys no release "
                    "(the deployment of the service has: "
                    f"{', '.join(sorted(str(one) for one in plan))})"
                )

    def _checks_of(self, deployment_group: DeploymentGroup) -> GroupChecks:
        """The checks that run after one group deploys."""
        return self.service_config.checks.get(deployment_group, GroupChecks())

    def _check_until_it_works(
        self, deployment_group: DeploymentGroup, group: List[HelmfileCommand]
    ) -> Optional[str]:
        """Runs the checks of the stage of one group, offering to retry on failure.

        The commands run first, and the manual checks follow them (just like the
        commands of a MediaWiki deployment run before its logstash checks). The
        order of the list of checks does not decide this.

        Returns None if no checks are defined.
        Returns None if every check was successful (or ignored).
        Otherwise returns the user's response to a failure.
        """
        checks = self._checks_of(deployment_group)

        if checks.commands:
            # Each check carries the name of its command, so that the log says
            # which one ran and which one failed.
            checkslist = [
                scap_checks.Check(command, command=command, timeout=120, shell=True)
                for command in checks.commands
            ]

            def run_checks() -> bool:
                ok, _ = scap_checks.execute(
                    checkslist, self.logger, concurrency=len(checkslist)
                )
                return ok

            answer = self.retry_until(
                f"the checks of {group[0].environment}",
                run_checks,
                {
                    "Ignore the failure and continue": "i",
                    "Roll back what was already deployed": "b",
                    "Exit without rolling back": "x",
                },
                self._prompt_default("r", "b"),
            )
            if answer == "i":
                self.logger.warning("Ignoring check failures and continuing")
            elif answer is not None:
                return answer

        return self._approve(deployment_group, checks)

    def _approve(
        self, deployment_group: DeploymentGroup, checks: GroupChecks
    ) -> Optional[str]:
        """Asks the questions of the manual checks of one group.

        Returns None when a deployer answers each question with a continue, and
        the answer that stops the deployment otherwise. A session with no
        terminal has nobody to ask, so it rolls back.
        """
        for question in checks.manual:
            answer = self.prompt_choices(
                f"{question or f'The deployment of {deployment_group} is complete'}."
                "\n\nWhat do you want to do?",
                {
                    "Continue the deployment": "c",
                    "Roll back what was already deployed": "b",
                    "Exit without rolling back": "x",
                },
                self._prompt_default("c", "b"),
            )
            if answer != "c":
                return answer

        return None

    def _apply(self, group: List[HelmfileCommand]) -> List[kubernetes.CommandResult]:
        """Deploys the steps of one group at the same time."""
        with self.k8s.watch_releases(group) as monitors:
            results = self.k8s.run_jobs(
                [command.apply_job(command.label) for command in group]
            )

            # The report of a step that failed does not count its release as
            # complete, because the release is not in the state that this
            # deployment wanted.
            for command, result in zip(group, results):
                for monitor in monitors[command]:
                    monitor.ok = result.ok

            return results

    def _prepare(self, command: HelmfileCommand) -> List[Rollback]:
        """Repairs each release of one step, and returns information needed to roll back if necessary."""
        recorded_at = datetime.now(timezone.utc)
        state = self.k8s.releases_state(command)

        rollbacks = []
        for release in self.k8s.releases(command):
            release_state = state.get(release)
            if release_state and (release_state.status or "").startswith("pending-"):
                self._repair(command, release)

            rollbacks.append(
                Rollback(
                    command,
                    release,
                    kubernetes.RecordedRevision(
                        release_state and release_state.revision, recorded_at
                    ),
                )
            )

        return rollbacks

    def _repair(self, command: HelmfileCommand, release: str):
        """Clears the pending state of a release, so that it accepts a deployment.

        A failure asks the user to retry or to exit, because a pending release
        accepts no deployment. When the session is not interactive, scap exits.
        """
        label = command.release_label(release)

        def repair() -> bool:
            try:
                self.k8s.fix_pending_state(command, release)
                return True
            except kubernetes.HelmfileError as e:
                self.logger.error(f"Could not repair {label}: {e}")
                return False

        self.retry_ignore_or_exit(f"the repair of {label}", repair, may_ignore=False)

    def _roll_back(self, rollbacks: List[List[Rollback]]):
        """Offers to return each deployed release to its revision.

        The groups roll back in the reverse of the order that deployed them,
        and the releases of one group roll back at the same time, as they
        deployed.
        """
        groups = [group for group in reversed(rollbacks) if group]
        if not groups:
            self.logger.info("Nothing deployed, so there is nothing to roll back")
            return

        lines = []
        for group in groups:
            first = group[0].command
            namespaces = _namespaces_of([rollback.command for rollback in group])
            lines.append(
                f"    [{first.environment_type}/{first.stage}] {first.environment}: "
                f"{', '.join(namespaces)}"
            )
            lines += [
                f"        uninstalls {rollback.release} of {rollback.command.namespace}"
                for rollback in group
                if rollback.recorded.revision is None
            ]

        if not self.prompt_user_for_confirmation(
            "Roll back what was already deployed?\n" + "\n".join(lines),
            default="y",
        ):
            self.logger.warning("Not rolling back")
            return

        # The count of replicas is the target of the deployment. A rollback
        # that changes it reports a total that does not match.
        commands = list(
            dict.fromkeys(rollback.command for group in groups for rollback in group)
        )
        with self.k8s.watch_releases(commands):
            for group in groups:
                self._roll_back_group(group)

    def _roll_back_group(self, rollbacks: List[Rollback]):
        """Rolls back one group, and offers a retry of the releases that failed."""
        pending = list(rollbacks)
        first = pending[0].command

        def roll_back_pending() -> bool:
            nonlocal pending
            agreed = self._roll_back_all(pending)
            pending = [rollback for rollback, ok in zip(pending, agreed) if not ok]
            return not pending

        self.retry_ignore_or_exit(
            f"the rollback of "
            f"{', '.join(_namespaces_of([r.command for r in rollbacks]))} "
            f"in {first.environment}",
            roll_back_pending,
        )

    def _roll_back_all(self, rollbacks: List[Rollback]) -> List[bool]:
        """Rolls back the releases of one group at the same time.

        Returns whether helm agreed to each rollback, in the order given.
        """
        with self.k8s.group_pools(
            rollbacks, lambda rollback: rollback.command.environment
        ) as pools:
            futures = [
                pools[rollback.command.environment].submit(
                    self._roll_back_one, rollback
                )
                for rollback in rollbacks
            ]

            return [future.result() for future in futures]

    def _roll_back_one(self, rollback: Rollback) -> bool:
        """Returns one release to its revision. Returns True if helm agreed."""
        self.logger.info(f"Rolling back {rollback.label}")
        return self.k8s.roll_back(
            rollback.command,
            rollback.release,
            rollback.recorded,
            f"The rollback of {rollback.label}",
        )

    def _confirm_diffs(self, plan: Plan) -> int:
        """Displays the diff of each step of the plan, then requests approval.

        Returns the number of diff commands that failed.
        """
        return self.k8s.review_diffs(
            [command.diff_job() for group in plan.values() for command in group]
        )
