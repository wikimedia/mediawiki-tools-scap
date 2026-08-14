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
import shlex
from dataclasses import dataclass, field
from typing import Dict, List

import yaml

from scap import cli, utils

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
    type: str
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

    def resolve(self, ref: str) -> Environment:
        """Return the environment named or aliased by `ref`."""
        if ref not in self._index:
            raise InvalidDeployServiceConfig(
                f"cluster group {self.name} has no environment named or aliased '{ref}' (known: {', '.join(sorted(self._index))})"
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
class ServiceConfig:
    """The deployment config of a single service, as declared in the service catalog."""

    name: str
    cluster_group: ClusterGroup
    # In the order the namespaces were declared.
    namespaces: List[ServiceNamespace]


@dataclass(frozen=True)
class HelmfileCommand:
    """A single helmfile invocation, and the context it was generated for."""

    directory: str
    command: List[str]
    environment_type: str
    stage: str
    namespace: str

    def __str__(self) -> str:
        return f"(cd {shlex.quote(self.directory)} && {shlex.join(self.command)})"


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
    service_what: str, name: str, doc, group: ClusterGroup
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
        env = group.resolve(ref)
        if env.name in parsed:
            raise InvalidDeployServiceConfig(
                f"'{ref}' and '{parsed[env.name].ref}' both refer to environment {env.name} of {what}"
            )

        env_what = f"environment {ref} of {what}"
        env_doc = _assert_mapping(env_doc, env_what)
        _assert_no_unexpected_keys(env_doc, ["releases"], env_what)

        releases = _parse_releases(env_what, env_doc, group.environment_default_stage)
        for release in releases:
            if release.stage not in group.environment_stages:
                raise InvalidDeployServiceConfig(
                    f"the stage of release {release.name} of {env_what} must be one of "
                    f"{', '.join(group.environment_stages)} (the environment_stages of cluster group "
                    f"{group.name}), not '{release.stage}'"
                )

        parsed[env.name] = ServiceEnvironment(
            ref=ref, environment=env, releases=releases
        )

    return ServiceNamespace(name=name, environments=parsed)


def _parse_service(
    catalog_what: str, name: str, doc, cluster_groups: Dict[str, ClusterGroup]
) -> ServiceConfig:
    what = f"service {name} of {catalog_what}"
    doc = _assert_mapping(doc, what)
    _assert_no_unexpected_keys(doc, ["cluster_group", "namespaces"], what)

    group_name = _assert_nonempty_string(
        doc.get("cluster_group"), f"the cluster_group of {what}"
    )
    if group_name not in cluster_groups:
        raise InvalidDeployServiceConfig(
            f"the cluster_group of {what} refers to undefined cluster group '{group_name}' "
            f"(known: {', '.join(sorted(cluster_groups))})"
        )
    group = cluster_groups[group_name]

    namespaces = _assert_mapping(doc.get("namespaces"), f"the namespaces of {what}")
    if not namespaces:
        raise InvalidDeployServiceConfig(f"{what} defines no namespaces")

    return ServiceConfig(
        name=name,
        cluster_group=group,
        namespaces=[
            _parse_namespace(
                what,
                _assert_nonempty_string(ns_name, f"each namespace name of {what}"),
                ns_doc,
                group,
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
    service_config: ServiceConfig, primary_datacenter: str, deployments_dir: str
) -> List[HelmfileCommand]:
    """Return the helmfile commands that deploy the service, in order."""
    commands = []
    group = service_config.cluster_group

    for environment_type in ENVIRONMENT_TYPES:
        for stage in group.environment_stages:
            for env in _environments_to_deploy(
                service_config, environment_type, primary_datacenter
            ):
                for namespace in service_config.namespaces:
                    service_env = namespace.environments.get(env.name)
                    if service_env is None:
                        continue

                    for command in _commands_for(
                        service_env, stage, group.environment_default_stage
                    ):
                        commands.append(
                            HelmfileCommand(
                                directory=os.path.join(
                                    deployments_dir, group.dir, namespace.name
                                ),
                                command=command,
                                environment_type=environment_type,
                                stage=stage,
                                namespace=namespace.name,
                            )
                        )

    return commands


def _commands_for(
    service_env: ServiceEnvironment, stage: str, default_stage: str
) -> List[List[str]]:
    base = ["helmfile", "apply", "-e", service_env.ref]

    if not service_env.releases:
        # An environment with no releases deploys all of its releases, at the
        # cluster group's default stage.
        return [base] if stage == default_stage else []

    # Releases are selected by helmfile's built-in "name" label, one invocation per
    # release, as scap already does elsewhere (see K8sOps).
    return [
        base + ["-l", f"name={release.name}"]
        for release in service_env.releases
        if release.stage == stage
    ]


@cli.command(
    "deploy-service",
    help="Deploy a service to Kubernetes",
)
class DeployService(cli.Application):
    @cli.argument("service", help="Name of the service to deploy")
    def main(self, *extra_args):
        logger = self.get_logger()

        service = self.arguments.service
        catalog_file = self.config["service_catalog_file"]

        cluster_groups = load_cluster_groups(
            self.config["cluster_groups_file"],
            self.config["helmfile_default_cluster_dir"],
        )
        catalog = load_service_catalog(catalog_file, cluster_groups)
        if service not in catalog:
            raise InvalidDeployServiceConfig(
                f"'{service}' is not a service in {catalog_file} "
                f"(known: {', '.join(sorted(catalog))})"
            )
        service_config = catalog[service]

        primary_datacenter = load_primary_datacenter(self.config["conftool_state_file"])

        logger.info(
            f"Deploying {service} using cluster group "
            f"{service_config.cluster_group.name} (primary datacenter: {primary_datacenter})"
        )

        commands = plan(
            service_config,
            primary_datacenter,
            self.config["helmfile_deployments_dir"],
        )
        if not commands:
            logger.warning(f"Nothing to deploy for {service}")
            return 0

        # TODO: Run these instead of echoing them.
        for command in commands:
            logger.info(f"[{command.environment_type}/{command.stage}] {command}")

        return 0
