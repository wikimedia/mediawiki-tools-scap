import base64
import concurrent.futures
import contextlib
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

from scap import utils, log, git, ssh, targets
from scap.cli import Application
from scap.runcmd import gitcmd

# Deployment stages #

# Test servers are also referred to as mwdebug servers
TEST_SERVERS = "testservers"
CANARIES = "canaries"
PRODUCTION = "production"

STAGES = [TEST_SERVERS, CANARIES, PRODUCTION]
"""All supported deployment stages, ordered by scope (increasing)."""


class InvalidDeploymentsConfig(Exception):
    pass


class DeploymentsConfig:
    """
    Deployment configuration files are specified in the format laid out in https://phabricator.wikimedia.org/T299648
    (modifications mentioned in the comments included)

    Instances of this class translate that format into one that represents Scap workflow better by organizing the
    configurations around deployment stages.

    For example, the following input config file:

    - namespace: testservers
      release: debug
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

    will produce the following output:

     {
      "testservers": [{
        "namespace": "testservers",
        "release": "debug",
        "mv_image_fl": "publish",
        "web_image_fl": "webserver",
        "debug": True,
      }],
      "canaries": [{
        "namespace": "api1",
        "release": "canaries",
        "mv_image_fl": "publish",
        "web_image_fl": "webserver",
        "debug": False,
      }],
      }],
      "production": [{
        "namespace": "api1",
        "release": "main",
        "mv_image_fl": "publish",
        "web_image_fl": "webserver",
        "debug": False,
        }, {
        "namespace": "api2",
        "release": "main",
        "mv_image_fl": "publish",
        "web_image_fl": "webserver",
        "debug": False,
      }]
     }

    Also, note that values of the `debug` field are interpreted as true according to Python's rules:
    https://docs.python.org/3/library/stdtypes.html#truth-value-testing
    """

    # The K8s namespace is also sometimes referred to as "cluster"
    NAMESPACE = "namespace"
    # Helmfile release
    RELEASE = "release"
    MULTIVER_IMAGE_FLAVOR = "mv_image_fl"
    WEB_IMAGE_FLAVOR = "web_image_fl"
    DEBUG = "debug"

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
        def is_testservers_config(dep_config: dict) -> bool:
            return dep_config.get("debug")

        testservers = {}
        canaries = {}
        production = {}

        with open(deployments_file) as f:
            deployments = yaml.safe_load(f)

        for dep_config in deployments:
            dep_namespace = dep_config["namespace"]

            if dep_namespace in {**testservers, **production}:
                raise InvalidDeploymentsConfig(
                    """"%s" deployment is already defined""" % dep_namespace
                )

            parsed_dep_config = {
                cls.NAMESPACE: dep_namespace,
                cls.RELEASE: dep_config["release"],
                cls.MULTIVER_IMAGE_FLAVOR: dep_config["mw_flavour"],
                cls.WEB_IMAGE_FLAVOR: dep_config["web_flavour"],
                cls.DEBUG: is_testservers_config(dep_config),
            }

            if is_testservers_config(dep_config):
                testservers[dep_namespace] = parsed_dep_config
            else:
                production[dep_namespace] = parsed_dep_config

                canary_release = str(dep_config["canary"]).strip()
                if canary_release not in ["None", ""]:
                    parsed_canary_dep_config = dict(parsed_dep_config)
                    parsed_canary_dep_config[cls.RELEASE] = canary_release
                    canaries[dep_namespace] = parsed_canary_dep_config

        return cls(
            list(testservers.values()),
            list(canaries.values()),
            list(production.values()),
        )


class K8sOps:
    """
    Kubernetes operations
    """

    def __init__(self, app: Application):
        self.app = app
        self.logger = app.get_logger()

        if app.config["build_mw_container_image"]:
            self._verify_build_and_push_prereqs()
        if app.config["deploy_mw_container_image"]:
            self._verify_deployment_prereqs()

        self.build_logfile = os.path.join(
            pathlib.Path.home(), "scap-image-build-and-push-log"
        )
        self.helm_env = self._collect_helm_env()
        self.original_helmfile_values = {}
        self.traindev = self._get_deployment_datacenters() == ["traindev"]
        self.build_state_dir = os.path.join(
            app.config["stage_dir"], "scap", "image-build"
        )

    def build_k8s_images(self):
        def build_and_push_images():
            with log.Timer("build-and-push-container-images", self.app.get_stats()):
                utils.mkdir_p(self.build_state_dir)

                make_container_image_dir = os.path.join(
                    release_repo_dir, "make-container-image"
                )
                registry = self.app.config["docker_registry"]

                dev_ca_crt = ""
                if self.app.config["mediawiki_image_extra_ca_cert"]:
                    with open(
                        self.app.config["mediawiki_image_extra_ca_cert"], "rb"
                    ) as f:
                        dev_ca_crt = base64.b64encode(f.read()).decode("utf-8")

                make_parameters = {
                    "IMAGE_BUILD_STATE_DIR": self.build_state_dir,
                    "MW_CONFIG_BRANCH": self.app.config[
                        "operations_mediawiki_config_branch"
                    ],
                    "workdir_volume": self.app.config["stage_dir"],
                    "mv_image_name": "{}/{}".format(
                        registry, self.app.config["mediawiki_image_name"]
                    ),
                    "mv_debug_image_name": "{}/{}".format(
                        registry, self.app.config["mediawiki_debug_image_name"]
                    ),
                    "webserver_image_name": "{}/{}".format(
                        registry, self.app.config["webserver_image_name"]
                    ),
                    "MV_BASE_PACKAGES": self.app.config[
                        "mediawiki_image_extra_packages"
                    ],
                    "MV_EXTRA_CA_CERT": dev_ca_crt,
                    "FORCE_FULL_BUILD": (
                        "true" if self.app.config["full_image_build"] else "false"
                    ),
                }
                with utils.suppress_backtrace():
                    cmd = "{} {}".format(
                        self.app.config["release_repo_build_and_push_images_cmd"],
                        " ".join(
                            [
                                shlex.quote("=".join(pair))
                                for pair in make_parameters.items()
                            ]
                        ),
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

        def update_helmfile_files():
            for stage, dep_configs in self.k8s_deployments_config.stages.items():
                self.original_helmfile_values[stage] = self._read_helmfile_files(
                    dep_configs
                )
                self._update_helmfile_files(dep_configs)

        if not self.app.config["build_mw_container_image"]:
            return

        release_repo_dir = self.app.config["release_repo_dir"]
        release_repo_update_cmd = self.app.config["release_repo_update_cmd"]

        if release_repo_update_cmd:
            self.logger.info("Running {}".format(release_repo_update_cmd))
            with utils.suppress_backtrace():
                subprocess.run(release_repo_update_cmd, shell=True, check=True)

        build_and_push_images()
        update_helmfile_files()

    def pull_image_on_nodes(self):
        """Pull the multiversion image down on all k8s nodes."""
        if not self.app.config.get("mw_k8s_nodes", False):
            return

        container_image_names = self._get_container_image_names()
        k8s_nodes = targets.get("mw_k8s_nodes", self.app.config).all
        image_tag = container_image_names["multiversion"].split(":").pop()
        cmd = f"/usr/bin/sudo /usr/local/sbin/mediawiki-image-download {image_tag}"
        pull = ssh.Job(
            hosts=k8s_nodes,
            command=cmd,
            user=self.app.config["ssh_user"],
            key=self.app.get_keyholder_key(),
            verbose=False,
            logger=self.logger,
        )
        with log.Timer("docker pull on k8s nodes"):
            pull.progress(log.reporter("docker_pull_k8s"))
            _, failed = pull.run()
            if failed:
                self.app.soft_errors = True
                self.logger.error(
                    f"{failed} K8s nodes failed to pull the multiversion image"
                )

    # Called by AbstractSync.main
    def helmfile_diffs_for_stage(self, stage: str):
        def diff_for_datacenter_and_deployment(datacenter, dep_config, report_queue):
            namespace = dep_config[DeploymentsConfig.NAMESPACE]
            release = dep_config[DeploymentsConfig.RELEASE]
            helmfile_dir = os.path.join(
                self.app.config["helmfile_services_dir"], namespace
            )
            cmd = [
                "helmfile",
                "-e",
                datacenter,
                "--selector",
                "name={}".format(release),
                "diff",
                "--context",
                "5",
            ]
            logger = logging.getLogger("scap.k8s.diff")
            return {
                "datacenter": datacenter,
                "namespace": namespace,
                "release": release,
                "diff_stdout": self._cmd_stdout(cmd, helmfile_dir, logger),
            }

        dep_configs = self.k8s_deployments_config.stages[stage]
        datacenters = self._get_deployment_datacenters()
        try:
            return self._foreach_datacenter_and_deployment(
                datacenters,
                dep_configs,
                diff_for_datacenter_and_deployment,
                "Diff",
                progress=False,
            )
        # Using BaseException so that we catch KeyboardInterrupt too
        except BaseException as e:
            self.app.soft_errors = True
            self.logger.error("K8s helmfile diffs for stage %s failed: %s", stage, e)
        return []

    # Called by AbstractSync.main()
    def deploy_k8s_images_for_stage(self, stage: str):
        if not self.app.config["deploy_mw_container_image"]:
            return

        dep_configs = self.k8s_deployments_config.stages[stage]
        datacenters = self._get_deployment_datacenters()
        try:
            self._deploy_to_datacenters(datacenters, dep_configs)
        # Using BaseException so that we catch KeyboardInterrupt too
        except BaseException as e:
            self.logger.error("K8s deployment to stage %s failed: %s", stage, e)

            saved_values = self.original_helmfile_values[stage]
            if saved_values:
                self.logger.error("Rolling back to prior state...")
                self._revert_helmfile_files(dep_configs, saved_values)
                try:
                    self._deploy_to_datacenters(datacenters, dep_configs)
                    self.logger.info("Rollback completed")
                except BaseException as e:
                    self.logger.error(
                        "Caught another exception while trying to roll back. Giving up: %s",
                        e,
                    )
            else:
                self.logger.error("No known prior state to roll back to")

            raise e

    def get_canary_namespaces(self) -> list:
        if not self.app.config["deploy_mw_container_image"]:
            return []

        res = set()

        for dep_config in self.k8s_deployments_config.stages[CANARIES]:
            res.add(dep_config[DeploymentsConfig.NAMESPACE])

        return list(res)

    def _get_deployment_datacenters(self) -> List[str]:
        # FIXME: Rename this config value
        return re.split(r"[,\s]+", self.app.config["k8s_clusters"])

    def _read_helmfile_files(self, dep_configs) -> dict:
        res = {}

        for dep_config in dep_configs:
            fq_release_name = self._dep_config_fq_release_name(dep_config)
            dep_config_values_file = self._dep_config_values_file(dep_config)
            if os.path.exists(dep_config_values_file):
                with open(dep_config_values_file) as f:
                    res[fq_release_name] = yaml.safe_load(f)

        return res

    def _update_helmfile_files(self, dep_configs):
        container_image_names = self._get_container_image_names()
        for dep_config in dep_configs:
            self._update_helmfile_values_for(dep_config, container_image_names)

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

    def _deploy_to_datacenters(self, datacenters, dep_configs):
        self._foreach_datacenter_and_deployment(
            datacenters,
            dep_configs,
            self._deploy_k8s_images_for_datacenter,
            "Deployment",
        )

    def _foreach_datacenter_and_deployment(
        self, datacenters, dep_configs, func, description, progress=True
    ):
        """
        Invokes 'func' over the product of all datacenters and dep_configs.
        'func' must take three arguments: datacenter, dep_config, report_queue.

        Note: Invocations of 'func' are concurrent and so must be threadsafe.

        'description' should be a human-readable description of the action
        performed by func (for diagnostic output / error handling).

        If 'progress' is True, a progress indicator will be displayed
        during the operation.

        Returns: A list of values returned by func.
        """

        if len(dep_configs) == 0 or len(datacenters) == 0:
            return []

        def foreach_deployment_in_datacenter(datacenter, report_queue):
            with concurrent.futures.ThreadPoolExecutor(
                max_workers=min(
                    len(dep_configs),
                    self.app.config["k8s_max_concurrent_deployments_per_dc"],
                )
            ) as pool:
                futures = []

                for dep_config in dep_configs:
                    future = pool.submit(func, datacenter, dep_config, report_queue)
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

        total_expected_replicas = self._get_total_expected_replicas(
            datacenters, dep_configs
        )

        report_queue = None
        if progress:
            report_queue = queue.Queue()
            reporter = threading.Thread(
                target=self._deployment_reporter,
                args=(report_queue, self.logger, total_expected_replicas),
                name="k8s deployment reporter",
            )
            reporter.start()

        with concurrent.futures.ThreadPoolExecutor(
            max_workers=len(datacenters)
        ) as pool:
            futures = []

            for datacenter in datacenters:
                future = pool.submit(
                    foreach_deployment_in_datacenter, datacenter, report_queue
                )
                future._scap_datacenter = datacenter
                futures.append(future)

            results = []
            failed = []

            try:
                for future in concurrent.futures.as_completed(futures):
                    exception = future.exception()
                    if exception:
                        failed.append(
                            "{}: {}".format(future._scap_datacenter, exception)
                        )
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
            # and _deploy_to_datacenters() adds "stop".
            data = report_queue.get()

            if data == "stop":
                break

            deployment, availableReplicas = data

            if reports.get(deployment) != availableReplicas:
                reports[deployment] = availableReplicas
                reporter.set_success(sum(reports.values()))

        reporter.finish()

    def _get_total_expected_replicas(self, datacenters, dep_configs) -> int:
        total = 0

        for datacenter in datacenters:
            for dep_config in dep_configs:
                release = dep_config[DeploymentsConfig.RELEASE]
                namespace = dep_config[DeploymentsConfig.NAMESPACE]
                helmfile_dir = os.path.join(
                    self.app.config["helmfile_services_dir"], namespace
                )

                with tempfile.NamedTemporaryFile() as tmp:
                    cmd = [
                        "helmfile",
                        "-e",
                        datacenter,
                        "--selector",
                        f"name={release}",
                        "write-values",
                        "--output-file-template",
                        tmp.name,
                    ]
                    self._run_timed_cmd_quietly(
                        cmd, helmfile_dir, self.logger, really_quiet=True
                    )
                    total += yaml.safe_load(tmp)["resources"]["replicas"]

        return total

    def _get_kubeconfig(self, datacenter, helmfile_dir, release, logger):
        cmd = ["helmfile", "-e", datacenter, "-l", "name={}".format(release), "build"]
        stdout = self._cmd_stdout(cmd, helmfile_dir, logger)
        if not stdout:
            return None

        data = yaml.safe_load(stdout)
        for arg in data["helmDefaults"]["args"]:
            if re.search(r"/etc/kubernetes/", arg):
                return arg
        logger.warning(
            "Could not figure out which kubeconfig file to use for datacenter %s, helmfile_dir %s, release %s",
            datacenter,
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

    def _helm_fix_pending_state(self, datacenter, helmfile_dir, release, logger):
        """
        Fix the release if it is in a pending-* state
        """

        kubeconfig = self._get_kubeconfig(datacenter, helmfile_dir, release, logger)

        if not kubeconfig:
            return

        status = self._get_helm_release_status(
            kubeconfig, helmfile_dir, release, logger
        )
        logger.debug(
            "Status is '%s' for datacenter %s, helmfile_dir %s, release %s",
            status,
            datacenter,
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
                "Release %s for datacenter %s in %s is in %s state.  Attempting to clean up",
                release,
                datacenter,
                helmfile_dir,
                status,
            )
            # Should this use --wait ?
            cmd = ["helm", "--kubeconfig", kubeconfig, recovery_command, release]
            self._run_timed_cmd_quietly(cmd, helmfile_dir, logger)

    def _deploy_k8s_images_for_datacenter(self, datacenter, dep_config, report_queue):
        """
        datacenter will be something like "eqiad" or "codfw" or "traindev"
        """

        logger = logging.getLogger("scap.k8s.deploy")

        release = dep_config[DeploymentsConfig.RELEASE]
        namespace = dep_config[DeploymentsConfig.NAMESPACE]
        helmfile_dir = os.path.join(self.app.config["helmfile_services_dir"], namespace)

        self._helm_fix_pending_state(datacenter, helmfile_dir, release, logger)

        cmd = [
            "helmfile",
            "-e",
            datacenter,
            "--selector",
            "name={}".format(release),
            "apply",
        ]

        with self._k8s_deployment_monitoring(dep_config, datacenter, report_queue):
            self._run_timed_cmd_quietly(cmd, helmfile_dir, logger, really_quiet=True)

    @contextlib.contextmanager
    def _k8s_deployment_monitoring(self, dep_config, dc, report_queue):
        stop_event = threading.Event()
        monitor = threading.Thread(
            target=self._deployment_monitor,
            args=(dep_config, dc, stop_event, report_queue),
            name="K8s deployment monitor",
        )
        monitor.start()

        try:
            yield
        finally:
            stop_event.set()
            monitor.join()

    def _deployment_monitor(self, dep_config, dc, stop_event, report_queue):
        namespace = dep_config[DeploymentsConfig.NAMESPACE]
        release = dep_config[DeploymentsConfig.RELEASE]
        kubeconfig = f"/etc/kubernetes/{namespace}-deploy-{dc}.config"
        deployment_name = (
            f"{namespace}.dev.{release}"
            if dc == "traindev"
            else f"{namespace}.{dc}.{release}"
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
            return d["metadata"]["annotations"]["deployment.kubernetes.io/revision"]

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

    def _update_helmfile_values_for(self, dep_config, images_info):
        """
        Note: Due to the git operations and change of the working directory, this function
        is not thread safe.
        """
        registry = self.app.config["docker_registry"]

        def strip_registry(fqin):
            registry_prefix = registry + "/"
            if fqin.startswith(registry_prefix):
                return fqin[len(registry_prefix) :]

            return fqin

        if dep_config[DeploymentsConfig.DEBUG]:
            mv_img = strip_registry(images_info["debug"])
        else:
            mv_img = strip_registry(images_info["multiversion"])

        web_img = strip_registry(images_info["webserver"])

        values = {
            "docker": {
                "registry": registry,
            },
            "main_app": {
                "image": mv_img,
            },
            "mw": {
                "httpd": {
                    "image_tag": web_img,
                }
            },
        }

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
                    "Multiversion image is: '%s'\n"
                    "Webserver image is: '%s'"
                ) % (fq_release_name, mv_img, web_img)

                gitcmd("add", values_file)
                gitcmd("commit", "-m", msg)

    def _get_container_image_names(self) -> dict:
        """
        Return a data structure containing the fully qualified image names of the
        images most recently built by build_k8s_images().
        """

        report_file = os.path.join(self.build_state_dir, "report.json")

        with open(report_file) as f:
            report = json.load(f)

        return {
            "multiversion": report["mediawiki"]["multiversion-image"],
            "debug": report["mediawiki"]["multiversion-debug-image"],
            "webserver": report["webserver"]["image"],
        }

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
        logger.debug("stdout: %s", stdout)
        logger.debug("stderr: %s", stderr)
        logger.debug("exit status: %s", proc.returncode)

        if proc.returncode == 0:
            return stdout

        logger.error("Non-zero exit status (%d) from %s", proc.returncode, cmd)
        logger.error("stdout: %s", stdout)
        logger.error("stderr: %s", stderr)

        return None

    def _run_timed_cmd_quietly(self, cmd, dir, logger, env={}, really_quiet=False):
        env = self._helm_augmented_environment(env)
        env["SUPPRESS_SAL"] = "true"

        timer_logger = None
        if really_quiet:
            timer_logger = logging.Logger("silence")
            timer_logger.addHandler(logging.NullHandler(level=0))

        with log.Timer(
            "Running {} in {}".format(" ".join(cmd), dir),
            self.app.get_stats(),
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
