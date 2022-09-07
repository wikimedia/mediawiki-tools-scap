import base64
import numbers
import os
import pathlib
import re
import shlex
import subprocess
from distutils.util import strtobool
from typing import List, Any

import yaml

from scap import utils, log, git
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

    Also, note that values of the `debug` field are interpreted as "truthy"
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
            return cls._parse_bool(dep_config.get("debug"))

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

    @staticmethod
    def _parse_bool(val: Any) -> bool:
        if type(val) == bool:
            return val
        if isinstance(val, numbers.Number):
            return bool(val)
        if type(val) == str:
            return bool(strtobool(val))
        raise ValueError("""Don't know how to parse value "%s" into a boolean""" % val)


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

        # TODO: reset logs here
        self.build_logfile = os.path.join(pathlib.Path.home(), "scap-image-build-and-push-log")
        self.deploy_logfile = os.path.join(pathlib.Path.home(), "scap-image-deploy-log")

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
                utils.subprocess_check_run_quietly_if_ok(cmd, make_container_image_dir,
                                                         self.build_logfile, self.logger, shell=True)

    # FIXME: Parallelize as appropriate
    def deploy_k8s_images_for(self, stage: str):
        if not self.app.config["deploy_mw_container_image"]:
            return

        for stage_dep_config in self.k8s_deployments_config.stages[stage]:
            namespace = stage_dep_config[DeploymentsConfig.NAMESPACE]
            fq_release_name = "{}-{}".format(namespace, stage_dep_config[DeploymentsConfig.RELEASE])

            self._update_helmfile_values_for(fq_release_name)

            helmfile_dir = os.path.join(self.app.config["helmfile_services_dir"], namespace)
            self.logger.info(
                """K8s release "{}" deployment output redirected to {}""".format(fq_release_name, self.deploy_logfile)
            )

            for cluster in re.split(r'[,\s]+', self.app.config["k8s_clusters"]):
                for operation in ["apply", "test"]:
                    cmd = ["helmfile", "-e", cluster, operation]

                    with log.Timer("Running {} in {}".format(" ".join(cmd), helmfile_dir), self.app.get_stats()):
                        with utils.suppress_backtrace():
                            utils.subprocess_check_run_quietly_if_ok(
                                cmd,
                                helmfile_dir, self.deploy_logfile, self.logger
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

    def _update_helmfile_values_for(self, fq_release_name: str):
        helmfile_mediawiki_release_dir = self.app.config["helmfile_mediawiki_release_dir"]
        values_file = os.path.join(helmfile_mediawiki_release_dir, "{}.yaml".format(fq_release_name))
        images_info = self._get_container_image_names()
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

        utils.write_file_if_needed(values_file, yaml.dump(values))
        with utils.cd(helmfile_mediawiki_release_dir):
            if git.file_has_unstaged_changes(values_file):
                msg = (
                  "Updating release '%s'\n\n"
                  "Multiversion image is: '%s'\n"
                  "Webserver image is: '%s'"
                ) % (fq_release_name, mv_img, web_img)

                gitcmd("add", values_file)
                gitcmd("commit", "-m", msg)

    def _get_container_image_names(self) -> dict:
        make_container_image_dir = os.path.join(self.app.config["release_repo_dir"], "make-container-image")

        return {
            "multiversion": utils.read_first_line_from_file(
                os.path.join(make_container_image_dir, "last-build")
            ),
            "webserver": utils.read_first_line_from_file(
                os.path.join(make_container_image_dir, "webserver", "last-build")
            ),
        }
