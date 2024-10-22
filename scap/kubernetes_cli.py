# -*- coding: utf-8 -*-
"""
Scap command for performing MediaWiki on Kubernetes related operations.
"""

import os

from scap import cli
from scap.kubernetes import K8sOps
import scap.tasks as tasks


@cli.command(
    "build-images",
    help="Build MediaWiki container images",
)
class BuildImages(cli.Application):
    @cli.argument(
        "--single-version",
        default=None,
        metavar="VERSION",
        help="Perform a single-version image build using only this MediaWiki version",
    )
    @cli.argument("message", nargs="*", help="Log message for SAL")
    def main(self, *extra_args):
        """
        Build MediaWiki container images.
        """
        os.umask(self.config["umask"])

        # Hardcode config that influences checks in K8sOps
        self.config["build_mw_container_image"] = True
        self.config["deploy_mw_container_image"] = False

        versions = []
        force_version = False
        if self.arguments.single_version is not None:
            force_version = True
            versions = [self.arguments.single_version]
        else:
            versions = self.active_wikiversions("stage")

        with self.lock_and_announce():
            k8s_ops = K8sOps(self)

            if not force_version:
                self.timed(tasks.compile_wikiversions, "stage", self.config)

            for version in versions:
                self.timed(tasks.cache_git_info, version, self.config)
                self.timed(tasks.update_localization_cache, version, self, json=False)

            k8s_ops.build_k8s_images(versions, force_version=force_version)
