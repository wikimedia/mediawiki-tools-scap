import base64
from collections import defaultdict
import concurrent.futures
import contextlib
from dataclasses import dataclass
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
from typing import List
import queue

import yaml

from scap import utils, log, git, version
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
LABEL_MEDIAWIKI_VERSIONS = "vnd.wikimedia.mediawiki.versions"
LABEL_MEDIAWIKI_PHP_VERSION = "vnd.wikimedia.mediawiki.php.version"
LABEL_SCAP_STAGE_DIR = "vnd.wikimedia.scap.stage_dir"
LABEL_SCAP_BUILD_STATE_DIR = "vnd.wikimedia.scap.build_state_dir"
LABEL_BUILD_TYPE = "vnd.wikimedia.build-type"
LABEL_PARENT_IMAGE = "vnd.wikimedia.parent-image"


class InvalidDeploymentsConfig(Exception):
    pass


# The default production image kind producted by the image build process.
_DEFAULT_IMAGE_KIND = "image"


@dataclass
class _HelmfileReleaseValues:
    """Represents the helmfile values written by scap for a given MediaWiki-on-k8s release."""

    registry: str
    """The address of the docker registry."""
    mw_image_tag: str
    """The mediawiki app-image tag."""
    mw_image_php_version: str
    """The PHP version selected when building the mediawiki app image."""
    web_image_tag: str
    """The httpd web-image tag."""

    def to_values(self) -> dict:
        """Returns a dict of helmfile values in the format expected by the mediawiki chart."""
        return {
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
            "php": {
                "version": self.mw_image_php_version,
            },
        }


class DeploymentsConfig:
    """
    Represents the configuration of MediaWiki deployments for use by Scap.

    The deployments config is loaded from a YAML config file, containing a list of deployments
    config items.

    Each deployments config item has the following fields:
      namespace: A k8s namespace containing one or more helmfile releases
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
      "testservers": [{
        "namespace": "testservers",
        "release": "debug",
        "mw_image_kind": "debug-image",
        "mw_image_fl": "publish",
        "web_image_fl": "webserver",
        "deploy": True,
        "cluster_dir": None,
        "clusters": None,
      }],
      "canaries": [{
        "namespace": "api1",
        "release": "canary",
        "mw_image_kind": None,
        "mw_image_fl": "publish",
        "web_image_fl": "webserver",
        "deploy": True,
        "cluster_dir": None,
        "clusters": None,
      }],
      "production": [{
        "namespace": "api1",
        "release": "main",
        "mw_image_kind": None,
        "mw_image_fl": "publish",
        "web_image_fl": "webserver",
        "deploy": True,
        "cluster_dir": None,
        "clusters": None,
        }, {
        "namespace": "api2",
        "release": "main",
        "mw_image_kind": None,
        "mw_image_fl": "publish",
        "web_image_fl": "webserver",
        "deploy": True,
        "cluster_dir": "dse",
        "clusters": ["dse-eqiad"],
        }, {
        "namespace": "api2",
        "release": "experimental",
        "mw_image_kind": None,
        "mw_image_fl": "publish-experimental",
        "web_image_fl": "webserver",
        "deploy": True,
        "cluster_dir": "dse",
        "clusters": ["dse-eqiad"],
        }, {
        "namespace": "api2",
        "release": "maintenance",
        "mw_image_kind": "cli-image",
        "mw_image_fl": "publish",
        "web_image_fl": "webserver",
        "deploy": False,
        "cluster_dir": "dse"
        "clusters": ["dse-eqiad"],
      }]
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

    # The k8s clusters in which this namespace is defined and associated releases are deployed.
    CLUSTERS = "clusters"
    # The directory under which helmfile configurations for these k8s clusters are located.
    CLUSTER_DIR = "cluster_dir"
    NAMESPACE = "namespace"
    # Helmfile release
    RELEASE = "release"
    MW_IMAGE_KIND = "mw_image_kind"
    MW_IMAGE_FLAVOUR = "mw_image_fl"
    WEB_IMAGE_FLAVOUR = "web_image_fl"
    DEPLOY = "deploy"

    def __init__(
        self, testservers: List[dict], canaries: List[dict], production: List[dict]
    ):
        self.stages = {
            TEST_SERVERS: testservers,
            CANARIES: canaries,
            PRODUCTION: production,
        }

    @classmethod
    def parse(cls, deployments_file: str) -> "DeploymentsConfig":
        testservers = []
        canaries = []
        production = []

        with open(deployments_file) as f:
            deployments = yaml.safe_load(f)

        namespaces = set()

        for dep_config in deployments:
            dep_namespace = dep_config["namespace"]
            cluster_dir = dep_config.get("dir", None)
            clusters = dep_config.get("clusters")

            if dep_namespace in namespaces:
                raise InvalidDeploymentsConfig(
                    """"%s" deployment is already defined""" % dep_namespace
                )
            namespaces.add(dep_namespace)

            for release, config in dep_config["releases"].items():
                mw_flavour = config.get("mw_flavour", dep_config.get("mw_flavour"))
                if not mw_flavour:
                    raise InvalidDeploymentsConfig(
                        f'"{release}" in "{dep_namespace}" has no mw_flavour set'
                    )
                web_flavour = config.get("web_flavour", dep_config.get("web_flavour"))
                if not web_flavour:
                    raise InvalidDeploymentsConfig(
                        f'"{release}" in "{dep_namespace}" has no web_flavour set'
                    )
                stage = config.get("stage", PRODUCTION)
                if stage not in STAGES:
                    raise InvalidDeploymentsConfig(
                        f'"{release}" in "{dep_namespace}" specified unsupported stage "{stage}"'
                    )
                mw_image_kind = config.get("mw_kind", dep_config.get("mw_kind"))
                parsed_dep_config = {
                    cls.NAMESPACE: dep_namespace,
                    cls.RELEASE: release,
                    cls.MW_IMAGE_KIND: mw_image_kind,
                    cls.MW_IMAGE_FLAVOUR: mw_flavour,
                    cls.WEB_IMAGE_FLAVOUR: web_flavour,
                    cls.DEPLOY: config.get("deploy", True),
                    cls.CLUSTER_DIR: cluster_dir,
                    cls.CLUSTERS: clusters,
                }
                if stage == TEST_SERVERS:
                    testservers.append(parsed_dep_config)
                elif stage == CANARIES:
                    canaries.append(parsed_dep_config)
                else:
                    production.append(parsed_dep_config)

        return cls(testservers, canaries, production)


class K8sOps:
    """
    Kubernetes operations
    """

    def __init__(
        self,
        app: Application,
        suffix: str = "",
        update_releases_repo: bool = True,
    ):
        self.app = app
        self.suffix = suffix
        self.update_releases_repo = update_releases_repo
        self.logger = app.get_logger()

        if app.config["build_mw_container_image"]:
            self._verify_build_and_push_prereqs()
        if app.config["deploy_mw_container_image"]:
            self._verify_deployment_prereqs()

        self.build_logfile = os.path.join(
            pathlib.Path.home(), "scap-image-build-and-push-log" + suffix
        )
        self.helm_env = self._collect_helm_env()
        self.original_helmfile_values = {}
        self.default_clusters = re.split(r"[,\s]+", self.app.config["k8s_clusters"])
        self.traindev = self.default_clusters == ["traindev"]
        self.build_state_dir = os.path.join(
            app.config["stage_dir"], "scap", "image-build" + suffix
        )

        if update_releases_repo and self.app.config["build_mw_container_image"]:
            release_repo_update_cmd = self.app.config["release_repo_update_cmd"]
            if release_repo_update_cmd:
                self.logger.info("Running {}".format(release_repo_update_cmd))
                with utils.suppress_backtrace():
                    subprocess.run(release_repo_update_cmd, shell=True, check=True)

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

        dev_ca_crt = ""
        if self.app.config["mediawiki_image_extra_ca_cert"]:
            with open(self.app.config["mediawiki_image_extra_ca_cert"], "rb") as f:
                dev_ca_crt = base64.b64encode(f.read()).decode("utf-8")

        mw_versions_list = ",".join(mediawiki_versions)
        stage_dir = self.app.config["stage_dir"]
        mw_image_basename = (
            self.app.config["mediawiki_sv_image_basename"]
            if mediawiki_versions == ["next"]
            else self.app.config["mediawiki_mv_image_basename"]
        )
        build_images_args = [
            self.build_state_dir,
            "--staging-dir",
            stage_dir,
            "--mediawiki-versions",
            mw_versions_list,
            "--multiversion-image-name",
            f"{registry}/{mw_image_basename}",
            "--multiversion-debug-image-name",
            f"{registry}/{mw_image_basename}-debug",
            "--multiversion-cli-image-name",
            f"{registry}/{mw_image_basename}-cli",
            "--webserver-image-name",
            f"{registry}/{self.app.config['webserver_image_name']}",
            "--latest-tag",
            latest_tag,
            "--label",
            f"{LABEL_BUILDER_NAME}=scap",
            "--label",
            f"{LABEL_BUILDER_VERSION}={version.__version__}",
            "--label",
            f"{LABEL_MEDIAWIKI_VERSIONS}={mw_versions_list}",
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

        if self.app.config["mediawiki_image_extra_packages"]:
            build_images_args += [
                "--mediawiki-image-extra-packages",
                self.app.config["mediawiki_image_extra_packages"],
            ]
        if dev_ca_crt:
            build_images_args += ["--mediawiki-extra-ca-cert", dev_ca_crt]
        if self.app.config["full_image_build"]:
            build_images_args.append("--full")

        http_proxy = self.app.config["web_proxy"]
        if http_proxy:
            build_images_args += [
                "--http-proxy",
                http_proxy,
                "--https-proxy",
                http_proxy,
            ]

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

    # Called by AbstractSync.main
    def helmfile_diffs_for_stage(self, stage: str):
        def diff_for_cluster_and_deployment(cluster, dep_config, report_queue):
            namespace = dep_config[DeploymentsConfig.NAMESPACE]
            release = dep_config[DeploymentsConfig.RELEASE]
            helmfile_dir = self._get_helmfile_path_for(dep_config)
            cmd = [
                "helmfile",
                "-e",
                cluster,
                "--selector",
                "name={}".format(release),
                "diff",
                "--context",
                "5",
            ]
            logger = logging.getLogger("scap.k8s.diff")
            return {
                "cluster": cluster,
                "namespace": namespace,
                "release": release,
                "diff_stdout": self._cmd_stdout(cmd, helmfile_dir, logger),
            }

        dep_configs = self.k8s_deployments_config.stages[stage]
        try:
            return self._foreach_cluster_and_deployment(
                dep_configs,
                diff_for_cluster_and_deployment,
                "Diff",
                progress=False,
                # Exclude non-deploy releases from diffs, since no subsequent deploy will apply them.
                deploy_only=True,
            )
        # Using BaseException so that we catch KeyboardInterrupt too
        except BaseException as e:
            self.app.soft_errors = True
            self.logger.error(
                "K8s helmfile diffs for stage %s failed: %s %s",
                stage,
                type(e).__name__,
                e,
            )
        return []

    # Called by AbstractSync.main()
    def update_helmfile_files(self):
        if not self.app.config["deploy_mw_container_image"]:
            return

        # Collect helmfile values for all stages and releases, ensuring all
        # referenced image kinds and flavours exist prior to attempting updates.
        images_info = self._get_built_images_report()
        values = {}
        for stage, dep_configs in self.k8s_deployments_config.stages.items():
            values[stage] = self._collect_helmfile_values_for(dep_configs, images_info)

        for stage, dep_configs in self.k8s_deployments_config.stages.items():
            self.original_helmfile_values[stage] = self._read_helmfile_files(
                dep_configs
            )
            self._update_helmfile_files(dep_configs, values[stage])

    # Called by AbstractSync.main()
    def deploy_k8s_images_for_stage(self, stage: str):
        if not self.app.config["deploy_mw_container_image"]:
            return

        dep_configs = self.k8s_deployments_config.stages[stage]
        try:
            self._deploy_to_clusters(dep_configs)
        # Using BaseException so that we catch KeyboardInterrupt too
        except BaseException as e:
            self.logger.error("K8s deployment to stage %s failed: %s", stage, e)

            # If build_mw_container_image is False, there will be no prior state stored.
            saved_values = self.original_helmfile_values.get(stage)
            if saved_values:
                self.logger.error("Rolling back to prior state...")
                self._revert_helmfile_files(dep_configs, saved_values)
                try:
                    self._deploy_to_clusters(dep_configs)
                    self.logger.info("Rollback completed")
                except BaseException as rollback_e:
                    self.logger.error(
                        "Caught another exception while trying to roll back. Giving up: %s",
                        rollback_e,
                    )
            else:
                self.logger.error("No known prior state to roll back to")

            raise e

    def get_canary_namespaces(self) -> list:
        if not self.app.config["deploy_mw_container_image"]:
            return []

        res = set()

        for dep_config in self.k8s_deployments_config.stages[CANARIES]:
            # Exclude non-deploy releases from contributing to the canary-stage namespace list,
            # since they will not have been deployed in that stage.
            if dep_config[DeploymentsConfig.DEPLOY]:
                res.add(dep_config[DeploymentsConfig.NAMESPACE])

        return list(res)

    def _get_clusters_for_dep_config(self, dep_config):
        clusters = dep_config[DeploymentsConfig.CLUSTERS]
        return self.default_clusters if clusters is None else clusters

    def _get_helmfile_path_for(self, dep_config):
        cluster_dir = dep_config[DeploymentsConfig.CLUSTER_DIR]
        if cluster_dir is None:
            cluster_dir = self.app.config["helmfile_default_cluster_dir"]
        return (
            pathlib.Path(self.app.config["helmfile_deployments_dir"])
            / cluster_dir
            / dep_config[DeploymentsConfig.NAMESPACE]
        )

    def _read_helmfile_files(self, dep_configs) -> dict:
        res = {}

        for dep_config in dep_configs:
            fq_release_name = self._dep_config_fq_release_name(dep_config)
            dep_config_values_file = self._dep_config_values_file(dep_config)
            if os.path.exists(dep_config_values_file):
                with open(dep_config_values_file) as f:
                    res[fq_release_name] = yaml.safe_load(f)

        return res

    def _update_helmfile_files(self, dep_configs, helmfile_values):
        for dep_config in dep_configs:
            fq_release_name = self._dep_config_fq_release_name(dep_config)
            self._update_helmfile_values_for(
                dep_config, helmfile_values[fq_release_name]
            )

    def _revert_helmfile_files(self, dep_configs, saved_values):
        commit = False

        with utils.cd(self.app.config["helmfile_mediawiki_release_dir"]):
            for dep_config in dep_configs:
                fq_release_name = self._dep_config_fq_release_name(dep_config)
                values = saved_values[fq_release_name]
                values_file = self._dep_config_values_file(dep_config)

                utils.write_file_if_needed(values_file, yaml.dump(values))
                if git.file_has_unstaged_changes(values_file):
                    gitcmd("add", values_file)
                    commit = True

            if commit:
                gitcmd("commit", "-m", "Configuration(s) reverted")

    def _deploy_to_clusters(self, dep_configs):
        self._foreach_cluster_and_deployment(
            dep_configs,
            self._deploy_k8s_images_for_cluster,
            "Deployment",
            deploy_only=True,  # Exclude non-deploy releases
        )

    def _foreach_cluster_and_deployment(
        self,
        dep_configs,
        func,
        description,
        progress=True,
        deploy_only=False,
    ):
        """
        Invokes 'func' over all dep_configs and their relevant clusters.
        'func' must take three arguments: cluster, dep_config, report_queue.

        Note: Invocations of 'func' are concurrent and so must be threadsafe.

        'description' should be a human-readable description of the action
        performed by func (for diagnostic output / error handling).

        If 'progress' is True, a progress indicator will be displayed
        during the operation.

        If 'deploy_only' is True, dep_configs corresponding to non-deploy
        releases will be excluded.

        Returns: A list of values returned by func.
        """

        if deploy_only:
            dep_configs = [
                dep_config
                for dep_config in dep_configs
                if dep_config[DeploymentsConfig.DEPLOY]
            ]

        cluster_dep_configs = defaultdict(list)
        for dep_config in dep_configs:
            for cluster in self._get_clusters_for_dep_config(dep_config):
                cluster_dep_configs[cluster].append(dep_config)

        if not cluster_dep_configs:
            return []

        def foreach_deployment_in_cluster(cluster, report_queue):
            with concurrent.futures.ThreadPoolExecutor(
                max_workers=min(
                    len(cluster_dep_configs[cluster]),
                    self.app.config["k8s_max_concurrent_deployments_per_cluster"],
                )
            ) as pool:
                futures = []

                for dep_config in cluster_dep_configs[cluster]:
                    future = pool.submit(func, cluster, dep_config, report_queue)
                    future._scap_dep_config = dep_config
                    futures.append(future)

                results = []
                failed = []

                for future in concurrent.futures.as_completed(futures):
                    exception = future.exception()

                    if exception:
                        fq_release_name = self._dep_config_fq_release_name(
                            future._scap_dep_config
                        )
                        failed.append(
                            "{} of {} failed: {}".format(
                                description, fq_release_name, exception
                            )
                        )
                    else:
                        results.append(future.result())

                if failed:
                    raise Exception("\n".join(failed))

                return results

        report_queue = None
        if progress:
            total_expected_replicas = self._get_total_expected_replicas(dep_configs)
            report_queue = queue.Queue()
            reporter = threading.Thread(
                target=self._deployment_reporter,
                args=(report_queue, self.logger, total_expected_replicas),
                name="k8s deployment reporter",
            )
            reporter.start()

        with concurrent.futures.ThreadPoolExecutor(
            max_workers=len(cluster_dep_configs)
        ) as pool:
            futures = []

            for cluster in cluster_dep_configs:
                future = pool.submit(
                    foreach_deployment_in_cluster, cluster, report_queue
                )
                future._scap_cluster = cluster
                futures.append(future)

            results = []
            failed = []

            try:
                for future in concurrent.futures.as_completed(futures):
                    exception = future.exception()
                    if exception:
                        failed.append("{}: {}".format(future._scap_cluster, exception))
                    else:
                        results.extend(future.result())
            finally:
                if progress:
                    report_queue.put("stop")
                    reporter.join()

            if failed:
                raise Exception(
                    f"K8s {description} had the following errors:\n "
                    + "\n".join(failed)
                )

            return results

    def _deployment_reporter(self, report_queue, logger, total_expected_replicas):
        reports = {}
        reporter = log.reporter("K8s deployment progress")
        reporter.expect(total_expected_replicas)
        reporter.start()

        while True:
            # Normal entries are added to the queue in _deployment_monitor(),
            # and _deploy_to_clusters() adds "stop".
            data = report_queue.get()

            if data == "stop":
                break

            deployment, availableReplicas = data

            if reports.get(deployment) != availableReplicas:
                reports[deployment] = availableReplicas
                reporter.set_success(sum(reports.values()))

        reporter.finish()

    def _get_total_expected_replicas(self, dep_configs) -> int:
        total = 0

        for dep_config in dep_configs:
            release = dep_config[DeploymentsConfig.RELEASE]
            helmfile_dir = self._get_helmfile_path_for(dep_config)
            for cluster in self._get_clusters_for_dep_config(dep_config):
                with tempfile.NamedTemporaryFile() as tmp:
                    cmd = [
                        "helmfile",
                        "-e",
                        cluster,
                        "--selector",
                        f"name={release}",
                        "write-values",
                        "--output-file-template",
                        tmp.name,
                    ]
                    self._run_timed_cmd_quietly(
                        cmd, helmfile_dir, self.logger, really_quiet=True
                    )
                    total += yaml.safe_load(tmp).get("resources", {}).get("replicas", 0)

        return total

    def _get_kubeconfig(self, cluster, helmfile_dir, release, logger):
        cmd = ["helmfile", "-e", cluster, "-l", "name={}".format(release), "build"]
        stdout = self._cmd_stdout(cmd, helmfile_dir, logger)
        if not stdout:
            return None

        data = yaml.safe_load(stdout)
        for arg in data["helmDefaults"]["args"]:
            if re.search(r"/etc/kubernetes/", arg):
                return arg
        logger.warning(
            "Could not figure out which kubeconfig file to use for cluster %s, helmfile_dir %s, release %s",
            cluster,
            helmfile_dir,
            release,
        )
        return None

    def _get_helm_release_status(self, kubeconfig, helmfile_dir, release, logger):
        cmd = [
            "helm",
            "--kubeconfig",
            kubeconfig,
            "ls",
            "-l",
            "name={}".format(release),
            "-a",
            "-o",
            "json",
        ]
        stdout = self._cmd_stdout(cmd, helmfile_dir, logger)
        if not stdout:
            return None

        data = json.loads(stdout)
        if not data:
            return None

        assert len(data) == 1

        return data[0].get("status")

    def _helm_fix_pending_state(
        self, cluster, helmfile_dir, namespace, release, logger
    ):
        """
        Fix the release if it is in a pending-* state
        """

        kubeconfig = self._get_kubeconfig(cluster, helmfile_dir, release, logger)

        if not kubeconfig:
            return

        status = self._get_helm_release_status(
            kubeconfig, helmfile_dir, release, logger
        )
        logger.debug(
            "Status is '%s' for cluster %s, helmfile_dir %s, release %s",
            status,
            cluster,
            helmfile_dir,
            release,
        )

        recovery_commands = {
            "pending-install": "uninstall",
            "pending-upgrade": "rollback",
            "pending-rollback": "rollback",
        }

        recovery_command = recovery_commands.get(status)
        if recovery_command:
            logger.warning(
                "Release %s for cluster %s in %s is in %s state.  Attempting to clean up",
                release,
                cluster,
                helmfile_dir,
                status,
            )
            # Should this use --wait ?
            cmd = ["helm", "--kubeconfig", kubeconfig, recovery_command, release]
            timer_name = f"helm_{recovery_command}_{namespace}_{release}_{cluster}"
            self._run_timed_cmd_quietly(
                cmd, helmfile_dir, logger, timer_name=timer_name
            )

    def _deploy_k8s_images_for_cluster(self, cluster, dep_config, report_queue):
        """
        cluster will be something like "eqiad" or "codfw" or "traindev"
        """

        logger = logging.getLogger("scap.k8s.deploy")

        release = dep_config[DeploymentsConfig.RELEASE]
        namespace = dep_config[DeploymentsConfig.NAMESPACE]
        helmfile_dir = self._get_helmfile_path_for(dep_config)

        self._helm_fix_pending_state(cluster, helmfile_dir, namespace, release, logger)

        cmd = [
            "helmfile",
            "-e",
            cluster,
            "--selector",
            "name={}".format(release),
            "apply",
        ]

        with self._k8s_deployment_monitoring(dep_config, cluster, report_queue):
            timer_name = f"helmfile_apply_{namespace}_{release}_{cluster}"
            self._run_timed_cmd_quietly(
                cmd, helmfile_dir, logger, really_quiet=True, timer_name=timer_name
            )

    @contextlib.contextmanager
    def _k8s_deployment_monitoring(self, dep_config, cluster, report_queue):
        stop_event = threading.Event()
        monitor = threading.Thread(
            target=self._deployment_monitor,
            args=(dep_config, cluster, stop_event, report_queue),
            name="K8s deployment monitor",
        )
        monitor.start()

        try:
            yield
        finally:
            stop_event.set()
            monitor.join()

    def _deployment_monitor(self, dep_config, cluster, stop_event, report_queue):
        namespace = dep_config[DeploymentsConfig.NAMESPACE]
        release = dep_config[DeploymentsConfig.RELEASE]
        kubeconfig = f"/etc/kubernetes/{namespace}-deploy-{cluster}.config"
        deployment_name = (
            f"{namespace}.dev.{release}"
            if cluster == "traindev"
            else f"{namespace}.{cluster}.{release}"
        )

        def kubectl_cmd(*args) -> tuple:
            return ("kubectl", "--kubeconfig", kubeconfig) + args

        def get_deployment():
            cmd = kubectl_cmd(
                "get",
                "deployment",
                deployment_name,
                "-o",
                "json",
            )
            ret = subprocess.run(cmd, text=True, capture_output=True)
            if ret.returncode == 0:
                return json.loads(ret.stdout)
            if "(NotFound)" in ret.stderr:
                return None
            raise Exception(" ".join(cmd) + f" failed:\n{ret.stderr}")

        def get_deployment_revision():
            d = get_deployment()
            if not d:
                return None
            return d["metadata"]["annotations"].get("deployment.kubernetes.io/revision")

        def get_current_replicaset(deployment_revision):
            cmd = kubectl_cmd(
                "get",
                "rs",
                "-o",
                "json",
                "-l",
                f"deployment={namespace}",
                "-l",
                f"release={release}",
            )
            data = json.loads(subprocess.check_output(cmd, text=True))
            assert data["kind"] == "List"

            for rs in data["items"]:
                revision = rs["metadata"]["annotations"][
                    "deployment.kubernetes.io/revision"
                ]
                if revision == deployment_revision:
                    return rs

            return None

        initial_revision = get_deployment_revision()

        # Wait for the deployment revision to change
        while not stop_event.wait(timeout=1):
            revision = get_deployment_revision()
            if revision != initial_revision:
                break

        if stop_event.is_set():
            return

        # Find the corresponding replicaset
        rs = get_current_replicaset(revision)
        rs_name = rs["metadata"]["name"]

        def do_report():
            cmd = kubectl_cmd("get", "rs", rs_name, "-o", "json")
            data = json.loads(subprocess.check_output(cmd, text=True))
            status = data["status"]
            availableReplicas = status.get("availableReplicas", 0)

            report_queue.put((deployment_name, availableReplicas))

        while not stop_event.wait(
            timeout=self.app.config["k8s_deployments_info_target_freshness"]
        ):
            do_report()

        # Perform one final report before stopping
        do_report()

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
            self.app.config["k8s_deployments_file"]
        )

    def _dep_config_fq_release_name(self, dep_config) -> str:
        return "{}-{}".format(
            dep_config[DeploymentsConfig.NAMESPACE],
            dep_config[DeploymentsConfig.RELEASE],
        )

    def _dep_config_values_file(self, dep_config) -> str:
        """
        Returns the path to the values.yaml file associated with dep_config
        """
        helmfile_mediawiki_release_dir = self.app.config[
            "helmfile_mediawiki_release_dir"
        ]
        fq_release_name = self._dep_config_fq_release_name(dep_config)
        return os.path.join(
            helmfile_mediawiki_release_dir, "{}.yaml".format(fq_release_name)
        )

    def _collect_helmfile_values_for(self, dep_configs, images_info) -> dict:
        registry = self.app.config["docker_registry"]

        def strip_registry(fqin):
            registry_prefix = registry + "/"
            if fqin.startswith(registry_prefix):
                return fqin[len(registry_prefix) :]

            return fqin

        def find_image_flavour(image_info, flavour, image_kind=None):
            if flavour not in image_info:
                raise ValueError(
                    f"Image flavour '{flavour}' not found among built images in {image_info}"
                )

            if image_kind is None:
                image_kind = _DEFAULT_IMAGE_KIND

            if image_kind not in image_info[flavour]:
                raise ValueError(
                    f"Image kind '{image_kind}' not found among built images for flavour "
                    f"'{flavour}' in {image_info[flavour]}"
                )

            return image_info[flavour][image_kind]

        values = {}
        mw_img_php_versions = {}

        # An exception raised by find_image_flavour indicates a misconfiguration, in which case,
        # a backtrace is not useful.
        with utils.suppress_backtrace():
            for dep_config in dep_configs:
                mw_img = find_image_flavour(
                    images_info["mediawiki"]["by-flavour"],
                    dep_config[DeploymentsConfig.MW_IMAGE_FLAVOUR],
                    image_kind=dep_config[DeploymentsConfig.MW_IMAGE_KIND],
                )

                web_img = find_image_flavour(
                    images_info["webserver"]["by-flavour"],
                    dep_config[DeploymentsConfig.WEB_IMAGE_FLAVOUR],
                )

                php_version = mw_img_php_versions.get(mw_img)
                if php_version is None:
                    [image] = inspect_images([mw_img])
                    php_version = image["Config"]["Labels"].get(
                        LABEL_MEDIAWIKI_PHP_VERSION
                    )
                    if php_version is None:
                        raise RuntimeError(
                            f"No {LABEL_MEDIAWIKI_PHP_VERSION} label available on image "
                            f"{image['Id']} ({image['RepoTags']})"
                        )
                    mw_img_php_versions[mw_img] = php_version

                fq_release_name = self._dep_config_fq_release_name(dep_config)
                values[fq_release_name] = _HelmfileReleaseValues(
                    registry=registry,
                    mw_image_tag=strip_registry(mw_img),
                    mw_image_php_version=php_version,
                    web_image_tag=strip_registry(web_img),
                )

        return values

    def _update_helmfile_values_for(
        self, dep_config, helmfile_values: _HelmfileReleaseValues
    ):
        """
        Note: Due to the git operations and change of the working directory, this function
        is not thread safe.
        """

        values = helmfile_values.to_values()

        # Train-dev hack.  This is to override the mw-web canary-values.yaml
        # (which is read after values-traindev.yaml) which sets replicas to a
        # value suitable for production but too high for train-dev.
        if self.traindev:
            values["resources"] = {"replicas": 1}

        values_file = self._dep_config_values_file(dep_config)
        utils.write_file_if_needed(values_file, yaml.dump(values))
        with utils.cd(self.app.config["helmfile_mediawiki_release_dir"]):
            if git.file_has_unstaged_changes(values_file):
                fq_release_name = self._dep_config_fq_release_name(dep_config)
                msg = (
                    "Updating release '%s'\n\n"
                    "MediaWiki image is: '%s'\n"
                    "Webserver image is: '%s'"
                ) % (
                    fq_release_name,
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

    def _helm_augmented_environment(self, env: dict) -> dict:
        """
        Returns a new dictionary that is a copy of the current environment, with
        HELM_* variables set, plus any additional variables from 'env'.
        """
        res = os.environ.copy()
        res.update(self.helm_env)
        res.update(env)
        return res

    def _cmd_stdout(self, cmd, dir, logger, env={}):
        """
        Runs cmd in a subprocess.  The process has a zero exit
        status, return its stdout as a string.  Otherwise
        return None.
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
            env=self._helm_augmented_environment(env),
        )
        stdout, stderr = proc.communicate()
        log.log_large_message(f"stdout: {stdout}", logger, logging.DEBUG)
        log.log_large_message(f"stderr: {stderr}", logger, logging.DEBUG)
        logger.debug("exit status: %s", proc.returncode)

        if proc.returncode == 0:
            return stdout

        logger.error("Non-zero exit status (%d) from %s", proc.returncode, cmd)
        log.log_large_message(f"stdout: {stdout}", logger, logging.ERROR)
        log.log_large_message(f"stderr: {stderr}", logger, logging.ERROR)

        return None

    def _run_timed_cmd_quietly(
        self, cmd, dir, logger, env={}, really_quiet=False, timer_name=None
    ):
        env = self._helm_augmented_environment(env)
        env["SUPPRESS_SAL"] = "true"

        timer_logger = None
        if really_quiet:
            timer_logger = logging.Logger("silence")
            timer_logger.addHandler(logging.NullHandler(level=0))

        with self.app.Timer(
            "Running {} in {}".format(" ".join(cmd), dir),
            name=timer_name,
            logger=timer_logger,
        ):
            with tempfile.NamedTemporaryFile() as logstream:
                with utils.suppress_backtrace():
                    self._run_cmd(cmd, dir, logstream.name, logger, env=env)

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
    def _collect_helm_env(self) -> dict:
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
