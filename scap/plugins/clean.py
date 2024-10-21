# -*- coding: utf-8 -*-
"""For cleaning up old MediaWiki."""
import os
import shutil
import subprocess

from scap import ansi, cli, git, log, main, ssh, tasks, utils


@cli.command("clean", primary_deploy_server_only=True)
class Clean(main.AbstractSync):
    """Scap sub-command to clean old branches."""

    @cli.argument(
        "branch",
        type=utils.version_argument_parser,
        help="The name of the branch to clean. Specify 'auto' to select all inactive branches except the most recent one.",
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
    @cli.argument(
        "--dry-run",
        action="store_true",
        help="Print what would be cleaned without cleaning",
    )
    def main(self, *extra_args):
        """Clean old branches from the cluster for space savings."""

        if self.arguments.logo:
            print(ansi.logo(color=utils.should_colorize_output()))

        self._assert_auth_sock()

        if self.arguments.branch == "auto":
            self.branches_to_remove = self._autoselect_versions_to_remove()

            if not self.branches_to_remove:
                self.get_logger().info("No eligible versions to remove.")
                return 0

        else:
            self.branches_to_remove = [self.arguments.branch]

        branches_string = ", ".join(self.branches_to_remove)

        if self.arguments.dry_run:
            self.get_logger().info(
                "DRY-RUN mode: Would clean %s: %s",
                utils.pluralize("branch", self.branches_to_remove),
                branches_string,
            )
            return 0

        reason = "Cleaning {}: {}".format(
            utils.pluralize("branch", self.branches_to_remove),
            branches_string,
        )

        with self.lock(reason=reason):
            self.get_logger().info(reason)

            for branch in self.branches_to_remove:
                self.cleanup_branch(branch)

            target_hosts = self._get_target_list()

            self._sync_masters()
            self._sync_proxies_and_apaches(target_hosts)
            self._clean_remote_caches(target_hosts)

        self.announce(
            f"Pruned MediaWiki: {branches_string} (duration: {utils.human_duration(self.get_duration())})"
        )

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

        logger = self.get_logger()

        if os.path.exists(branch_dir):
            logger.info("Clean files owned by www-data")
            utils.sudo_check_call(
                "www-data", f"find {branch_dir} -user www-data -delete"
            )

        ext_msg = os.path.join(
            self.config["stage_dir"],
            "wmf-config",
            "ExtensionMessages-%s.php" % branch,
        )
        logger.info("Clean %s", ext_msg)
        self._maybe_delete(ext_msg)

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
                with self.Timer("prune-git-branches"):
                    # Prune all the submodules' remote branches
                    with utils.cd(branch_dir):
                        submodule_cmd = 'git submodule foreach "{} ||:"'.format(
                            " ".join(git_prune_cmd)
                        )
                        subprocess.check_output(submodule_cmd, shell=True)
                        if subprocess.call(git_prune_cmd) != 0:
                            logger.info("Failed to prune core branch")

        logger.info("Clean %s", branch_dir)
        self._maybe_delete(branch_dir)

        patch_base_dir = self.config["patch_path"]
        branch_patch_dir = os.path.join(patch_base_dir, branch)
        logger.info("Clean %s", branch_patch_dir)
        self._maybe_delete(branch_patch_dir)
        srv_patches_git_message = 'Scap clean for "{}"'.format(branch)
        git.add_all(patch_base_dir, message=srv_patches_git_message)

    def _clean_remote_caches(self, target_hosts):
        """
        Need to remove cache dirs manually after sync
        """

        cache_dirs = [
            os.path.join(self.config["deploy_dir"], "php-%s" % branch, "cache")
            for branch in self.branches_to_remove
        ]

        cmd = ["/bin/rm", "-rf"] + cache_dirs
        with self.Timer("clean-remote-caches"):
            remove_remote_dirs = ssh.Job(
                target_hosts, user=self.config["ssh_user"], key=self.get_keyholder_key()
            )

            remove_remote_dirs.command(cmd)
            remove_remote_dirs.progress(log.reporter("clean-remote-caches"))
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

    def _autoselect_versions_to_remove(self):
        """
        Returns a list of old on-disk versions, excluding the
        most recent old version.
        """
        active_versions = self.active_wikiversions("stage")

        # Collect an ascending list of old versions, stopping collection
        # once we run into a live version.
        old_versions = []
        for version in tasks.get_wikiversions_ondisk(self.config["stage_dir"]):
            if version in active_versions:
                break
            old_versions.append(version)

        # Return all but the most recent old version.
        return old_versions[:-1]
