import base64
import concurrent.futures
import os
import pathlib
import re
import shlex
import subprocess
import tempfile
from typing import List

import yaml

from scap import utils, log, git, ssh, targets
from scap.cli import Application
from scap.runcmd import gitcmd

# Deployment stages #

# Test servers are also referred to as mwdebug servers
TEST_SERVERS = "testservers"
CANARIES = "canaries"
PRODUCTION = "production"


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
      }],
      "canaries": [{
        "namespace": "api1",
        "release": "canaries",
        "mv_image_fl": "publish",
        "web_image_fl": "webserver",
      }],
      "production": [{
        "namespace": "api1",
        "release": "main",
        "mv_image_fl": "publish",
        "web_image_fl": "webserver",
        }, {
        "namespace": "api2",
        "release": "main",
        "mv_image_fl": "publish",
        "web_image_fl": "webserver",
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

    def __init__(self, testservers: List[dict], canaries: List[dict], production: List[dict]):
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

        return cls(list(testservers.values()), list(canaries.values()), list(production.values()))


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

        self.build_logfile = os.path.join(pathlib.Path.home(), "scap-image-build-and-push-log")

    def build_k8s_images(self):
        if not self.app.config["build_mw_container_image"]:
            return

        release_repo_dir = self.app.config["release_repo_dir"]
        release_repo_update_cmd = self.app.config["release_repo_update_cmd"]

        if release_repo_update_cmd:
            self.logger.info("Running {}".format(release_repo_update_cmd))
            with utils.suppress_backtrace():
                subprocess.run(release_repo_update_cmd, shell=True, check=True)

        with log.Timer("build-and-push-container-images", self.app.get_stats()):
            make_container_image_dir = os.path.join(release_repo_dir, "make-container-image")
            registry = self.app.config["docker_registry"]

            dev_ca_crt = ""
            if self.app.config["mediawiki_image_extra_ca_cert"]:
                with open(self.app.config["mediawiki_image_extra_ca_cert"], "rb") as f:
                    dev_ca_crt = base64.b64encode(f.read()).decode("utf-8")

            make_parameters = {
                "GIT_BASE": self.app.config["gerrit_url"],
                "MW_CONFIG_BRANCH": self.app.config["operations_mediawiki_config_branch"],
                "workdir_volume": self.app.config["stage_dir"],
                "mv_image_name": "{}/{}".format(registry, self.app.config["mediawiki_image_name"]),
                "mv_debug_image_name": "{}/{}".format(registry, self.app.config["mediawiki_debug_image_name"]),
                "webserver_image_name": "{}/{}".format(registry, self.app.config["webserver_image_name"]),
                "MV_BASE_PACKAGES": self.app.config["mediawiki_image_extra_packages"],
                "MV_EXTRA_CA_CERT": dev_ca_crt,
            }
            with utils.suppress_backtrace():
                cmd = "{} {}".format(
                    self.app.config["release_repo_build_and_push_images_cmd"],
                    " ".join([shlex.quote("=".join(pair)) for pair in make_parameters.items()])
                )

                self.logger.info("K8s images build/push output redirected to {}".format(self.build_logfile))
                K8sOps._ensure_file_deleted(self.build_logfile)
                try:
                    utils.subprocess_check_run_quietly_if_ok(cmd, make_container_image_dir,
                                                             self.build_logfile, self.logger, shell=True)
                except subprocess.CalledProcessError:
                    self.app.soft_errors = True

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
            pull.progress(log.reporter("docker_pull_k8s", self.app.config["fancy_progress"]))
            _, failed = pull.run()
            if failed:
                self.app.soft_errors = True
                self.logger.error(f"{failed} K8s nodes failed to pull the multiversion image")

    # Called by AbstractSync.main()
    def deploy_k8s_images_for_stage(self, stage: str):
        if not self.app.config["deploy_mw_container_image"]:
            return

        dep_configs = self.k8s_deployments_config.stages[stage]
        datacenters = re.split(r'[,\s]+', self.app.config["k8s_clusters"])  # FIXME: Rename this config value
        container_image_names = self._get_container_image_names()

        saved_values = self._read_current_values(dep_configs)

        try:
            self._update_values(dep_configs, container_image_names)
            self._deploy_to_datacenters(datacenters, dep_configs)
        # Using BaseException so that we catch KeyboardInterrupt too
        except BaseException as e:
            self.app.soft_errors = True

            # FIXME: Interruptions during helmfile apply will need to use something like
            # helm3 --kubeconfig /etc/kubernetes/mwdebug-deploy-eqiad.config rollback pinkunicorn --namespace mwdebug
            # to avoid leaving things in a broken state.
            self.logger.error("K8s deployment to stage %s failed: %s", stage, e)

            if saved_values:
                self.logger.error("Rolling back to prior state...")
                self._revert_values(dep_configs, saved_values)
                try:
                    self._deploy_to_datacenters(datacenters, dep_configs)
                    self.logger.error("Rollback completed")
                except BaseException as e:
                    self.logger.error("Caught another exception while trying to roll back. Giving up: %s", e)
            else:
                self.logger.error("No known prior state to roll back to")

    def _read_current_values(self, dep_configs) -> dict:
        res = {}

        for dep_config in dep_configs:
            fq_release_name = self._dep_config_fq_release_name(dep_config)
            dep_config_values_file = self._dep_config_values_file(dep_config)
            if os.path.exists(dep_config_values_file):
                with open(dep_config_values_file) as f:
                    res[fq_release_name] = yaml.safe_load(f)

        return res

    def _update_values(self, dep_configs, container_image_names):
        for dep_config in dep_configs:
            self._update_helmfile_values_for(dep_config, container_image_names)

    def _revert_values(self, dep_configs, saved_values):
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
        def deploy(datacenter, dep_config):
            namespace = dep_config[DeploymentsConfig.NAMESPACE]
            release = dep_config[DeploymentsConfig.RELEASE]
            helmfile_dir = os.path.join(self.app.config["helmfile_services_dir"], namespace)
            self._deploy_k8s_images_for_datacenter(datacenter, helmfile_dir, release)

        def deploy_to_datacenter(datacenter):
            with concurrent.futures.ThreadPoolExecutor(max_workers=max(len(dep_configs), 1)) as pool:
                futures = []

                for dep_config in dep_configs:
                    future = pool.submit(deploy, datacenter, dep_config)
                    future._scap_dep_config = dep_config
                    futures.append(future)

                failed = []

                for future in concurrent.futures.as_completed(futures):
                    exception = future.exception()

                    if exception:
                        fq_release_name = self._dep_config_fq_release_name(future._scap_dep_config)
                        failed.append("Deployment of {} failed: {}".format(fq_release_name, exception))

                if failed:
                    raise Exception("\n".join(failed))

        with concurrent.futures.ThreadPoolExecutor(max_workers=max(len(datacenters), 1)) as pool:
            futures = []

            for datacenter in datacenters:
                future = pool.submit(deploy_to_datacenter, datacenter)
                future._scap_datacenter = datacenter
                futures.append(future)

            failed = []

            for future in concurrent.futures.as_completed(futures):
                exception = future.exception()
                if exception:
                    failed.append("{}: {}".format(future._scap_datacenter, exception))

            if failed:
                raise Exception("K8s deployment had the following errors:\n " + "\n".join(failed))

    def _deploy_k8s_images_for_datacenter(self, datacenter, helmfile_dir, release):
        """
        datacenter will be something like "eqiad" or "codfw" or "traindev"
        """

        cmd = ["helmfile", "-e", datacenter, "--selector", "name={}".format(release), "apply"]

        with log.Timer("Running {} in {}".format(" ".join(cmd), helmfile_dir), self.app.get_stats()):
            # FIXME: Make sure command output gets sent to the logger at debug level
            with tempfile.NamedTemporaryFile() as logstream:
                with utils.suppress_backtrace():
                    # FIXME: error output needs to be prefixed w/ the datacenter name.
                    env = os.environ.copy()
                    env['SUPPRESS_SAL'] = 'true'
                    utils.subprocess_check_run_quietly_if_ok(
                        cmd,
                        helmfile_dir, logstream.name, self.logger, env=env
                    )

    def _verify_build_and_push_prereqs(self):
        if self.app.config["release_repo_dir"] is None:
            self.logger.error("release_repo_dir must be configured when build_mw_container_image is True")
            self.app.soft_errors = True
            self.logger.warning("Disabling build/push of K8s images")
            self.app.config["build_mw_container_image"] = False

    def _verify_deployment_prereqs(self):
        def disable_deployments():
            self.logger.warning("Disabling K8s deployments")
            self.app.config["deploy_mw_container_image"] = False

        if self.app.config["release_repo_dir"] is None:
            self.logger.error("release_repo_dir must be configured when deploy_mw_container_image is True")
            self.app.soft_errors = True
            disable_deployments()
            return

        try:
            self.k8s_deployments_config = DeploymentsConfig.parse(self.app.config["k8s_deployments_file"])
        except InvalidDeploymentsConfig as e:
            self.logger.error("Failed to parse K8s deployments config: {}".format(str(e)))
            self.app.soft_errors = True
            disable_deployments()

    def _dep_config_fq_release_name(self, dep_config) -> str:
        return "{}-{}".format(dep_config[DeploymentsConfig.NAMESPACE], dep_config[DeploymentsConfig.RELEASE])

    def _dep_config_values_file(self, dep_config) -> str:
        """
        Returns the path to the values.yaml file associated with dep_config
        """
        helmfile_mediawiki_release_dir = self.app.config["helmfile_mediawiki_release_dir"]
        fq_release_name = self._dep_config_fq_release_name(dep_config)
        return os.path.join(helmfile_mediawiki_release_dir, "{}.yaml".format(fq_release_name))

    def _update_helmfile_values_for(self, dep_config, images_info):
        """
        Note: Due to the git operations and change of the working directory, this function
        is not thread safe.
        """
        registry = self.app.config["docker_registry"]

        def strip_registry(fqin):
            registry_prefix = registry + "/"
            if fqin.startswith(registry_prefix):
                return fqin[len(registry_prefix):]

            return fqin

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
        make_container_image_dir = os.path.join(self.app.config["release_repo_dir"], "make-container-image")

        return {
            "multiversion": utils.read_first_line_from_file(
                os.path.join(make_container_image_dir, "last-build")
            ),
            "webserver": utils.read_first_line_from_file(
                os.path.join(make_container_image_dir, "webserver", "last-build")
            ),
        }

    @staticmethod
    def _ensure_file_deleted(file: str):
        if os.path.lexists(file):
            os.unlink(file)
