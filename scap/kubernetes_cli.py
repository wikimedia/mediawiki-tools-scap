# -*- coding: utf-8 -*-
"""
Scap command for performing MediaWiki on Kubernetes related operations.
"""

import os

from scap import cli
from scap.kubernetes import K8sOps, prune_local_images
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
    @cli.argument(
        "--latest-tag",
        default="latest",
        metavar="TAG",
        help="Tag to use that represents the latest build",
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
            versions = self.get_versions_to_include_in_image()

        with self.lock_mediawiki_staging_and_announce():
            k8s_ops = K8sOps(self)

            if not force_version:
                self.timed(tasks.compile_wikiversions, "stage", self.config)

            for version in versions:
                self.timed(tasks.cache_git_info, version, self.config)
                self.timed(tasks.update_localization_cache, version, self, json=False)

            with self.Timer("build-and-push-container-images"):
                k8s_ops.build_k8s_images(
                    versions,
                    force_version=force_version,
                    latest_tag=self.arguments.latest_tag,
                )


@cli.command(
    "clean-images",
    help="Remove unused local images that were built by scap",
)
class CleanImages(cli.Application):
    @cli.argument(
        "--dry-run",
        action="store_true",
        help="Report intended operations without actually performing them",
    )
    def main(self, *extra_args):
        """
        Untag/remove local images that were built by scap but are no longer
        referenced by any of the state files used for incremental builds.

        This subcommand requires access to Docker and may need to be executed
        via sudo.
        """
        prune_local_images(
            logger=self.get_logger(),
            dry_run=self.arguments.dry_run,
        )
