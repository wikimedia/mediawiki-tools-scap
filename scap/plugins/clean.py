# -*- coding: utf-8 -*-
"""For cleaning up old MediaWiki."""
from datetime import datetime, timedelta
import os
import shutil
import subprocess
import sys

from scap import cli
from scap import git
from scap import log
from scap import main
from scap import ssh
from scap import tasks
from scap import utils

# Inactive train branches created more than this number of days will be selected when
# 'auto' is supplied as the branch name on the command line.
AUTO_CLEAN_THRESHOLD = 8


@cli.command("clean", affected_by_blocked_deployments=True)
class Clean(main.AbstractSync):
    """Scap sub-command to clean old branches."""

    @cli.argument(
        "branch",
        help="The name of the branch to clean.  Specify 'auto' to select all inactive branches that were created more than {} days ago.".format(
            AUTO_CLEAN_THRESHOLD
        ),
    )
    @cli.argument(
        "--delete",
        action="store_true",
        help="Delete everything.  This is a legacy option which does not need to be supplied.",
    )
    @cli.argument(
        "--delete-gerrit-branch",
        action="store_true",
        help="Delete the branch on gerrit as well.",
    )
    @cli.argument(
        "--no-logo",
        action="store_false",
        help="Do not print the Scap logo",
        dest="logo",
    )
    def main(self, *extra_args):
        """Clean old branches from the cluster for space savings."""

        self.logo = self.arguments.logo

        if self.arguments.branch == "auto":
            self.branches_to_remove = self._autoselect_versions_to_remove()

            if not self.branches_to_remove:
                self.get_logger().info("No eligible versions to remove.")
                sys.exit()

        else:
            self.branches_to_remove = [self.arguments.branch]

        self.get_logger().info(
            "Cleaning branch(es): {}".format(", ".join(self.branches_to_remove))
        )
        self.arguments.message = "Pruned MediaWiki: {}".format(
            ", ".join(self.branches_to_remove)
        )
        self.arguments.force = False
        self.arguments.stop_before_sync = False
        # There's no need to build or deploy container images during scap clean
        self.config["build_mw_container_image"] = False
        self.config["deploy_mw_container_image"] = False
        return super().main(*extra_args)

    def _before_cluster_sync(self):
        for branch in self.branches_to_remove:
            self.cleanup_branch(branch)

    def cleanup_branch(self, branch):
        """
        Given a branch, go through the cleanup process on the master.

        (1) Remove files owned by www-data
        (2) Remove <staging>/wmf-config/ExtensionMessages-<branch>.php file
        (3) Prune git branches [if --delete-gerrit-branch is supplied]
        (4) Remove all branch files
        (5) Remove security patches
        """
        branch_dir = os.path.join(self.config["stage_dir"], "php-%s" % branch)

        if branch in self.active_wikiversions("stage"):
            raise SystemExit('Branch "%s" is still in use, aborting' % branch)

        if os.path.exists(branch_dir):
            for user in ["www-data"]:
                with log.Timer("clean-{}-owned-files".format(user), self.get_stats()):
                    utils.sudo_check_call(
                        user, "find %s -user %s -delete" % (branch_dir, user)
                    )

        with log.Timer("clean-ExtensionMessages"):
            ext_msg = os.path.join(
                self.config["stage_dir"],
                "wmf-config",
                "ExtensionMessages-%s.php" % branch,
            )
            self._maybe_delete(ext_msg)

        logger = self.get_logger()

        # Moved behind a feature flag until T218750 is resolved
        if self.arguments.delete_gerrit_branch:
            if not os.path.exists(branch_dir):
                logger.warn(
                    "Cannot perform --delete-gerrit-branch because {} does not exist".format(
                        branch_dir
                    )
                )
            else:
                git_prune_cmd = [
                    "git",
                    "push",
                    "origin",
                    "--quiet",
                    "--delete",
                    "wmf/%s" % branch,
                ]
                with log.Timer("prune-git-branches", self.get_stats()):
                    # Prune all the submodules' remote branches
                    with utils.cd(branch_dir):
                        submodule_cmd = 'git submodule foreach "{} ||:"'.format(
                            " ".join(git_prune_cmd)
                        )
                        subprocess.check_output(submodule_cmd, shell=True)
                        if subprocess.call(git_prune_cmd) != 0:
                            logger.info("Failed to prune core branch")
        with log.Timer("removing-local-copy"):
            self._maybe_delete(branch_dir)
        with log.Timer("cleaning-unused-patches", self.get_stats()):
            patch_base_dir = self.config["patch_path"]
            self._maybe_delete(os.path.join(patch_base_dir, branch))
            srv_patches_git_message = 'Scap clean for "{}"'.format(branch)
            git.add_all(patch_base_dir, message=srv_patches_git_message)

    def _after_cluster_sync(self):
        """
        Need to remove cache dirs manually after sync
        """

        cache_dirs = [
            os.path.join(self.config["deploy_dir"], "php-%s" % branch, "cache")
            for branch in self.branches_to_remove
        ]

        target_hosts = self._get_target_list()
        cmd = ["/bin/rm", "-rf"] + cache_dirs
        with log.Timer("clean-remote-caches", self.get_stats()):
            remove_remote_dirs = ssh.Job(
                target_hosts, user=self.config["ssh_user"], key=self.get_keyholder_key()
            )

            remove_remote_dirs.command(cmd)
            remove_remote_dirs.progress(
                log.reporter("clean-remote-caches", self.config["fancy_progress"])
            )
            success, fail = remove_remote_dirs.run()
            if fail:
                self.get_logger().warning("%d hosts failed to remove cache", fail)
                self.soft_errors = True

    def _maybe_delete(self, path):
        if os.path.exists(path):
            if os.path.isdir(path):
                shutil.rmtree(path)
            else:
                os.remove(path)
        else:
            self.get_logger().info("Unable to delete %s, already missing", path)

    def _after_lock_release(self):
        self.announce(
            self.arguments.message
            + " (duration: %s)" % utils.human_duration(self.get_duration())
        )

    def _autoselect_versions_to_remove(self):
        # Inactive branches created before the cutoff will be selected
        cutoff = datetime.utcnow() - timedelta(days=AUTO_CLEAN_THRESHOLD)

        active = self.active_wikiversions("stage")

        versions_to_remove = []
        for version, created in tasks.get_wikiversions_ondisk_ex(
            self.config["stage_dir"]
        ):
            if version not in active and created < cutoff:
                versions_to_remove.append(version)

        return versions_to_remove
