# -*- coding: utf-8 -*-
"""For cleaning up old MediaWiki."""
import os
import shutil
import subprocess
import sys

from scap import ansi, cli, git, log, main, mwscript, ssh, utils


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
            self.versions_to_remove = self._autoselect_versions_to_remove()

            if not self.versions_to_remove:
                self.get_logger().info("No eligible versions to remove.")
                return 0

        else:
            self.versions_to_remove = [self.arguments.branch]

        branches_string = ", ".join(self.versions_to_remove)

        if self.arguments.dry_run:
            self.get_logger().info(
                "DRY-RUN mode: Would clean %s: %s",
                utils.pluralize("branch", self.versions_to_remove),
                branches_string,
            )
            return 0

        reason = "Cleaning {}: {}".format(
            utils.pluralize("branch", self.versions_to_remove),
            branches_string,
        )

        with self.lock_mediawiki_staging(reason=reason):
            self.get_logger().info(reason)

            for branch in self.versions_to_remove:
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
            runtime_user = self.config["mediawiki_runtime_user"]
            logger.info(f"Clean files owned by {runtime_user}")
            mwscript.run_shell(
                self,
                "find {} -user {} -delete",
                branch_dir,
                runtime_user,
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
                logger.warning(
                    "Cannot perform --delete-gerrit-branch because {} does not exist".format(
                        branch_dir
                    )
                )
            else:
                prune_branch_args = [
                    "push",
                    "origin",
                    "--quiet",
                    "--delete",
                    "wmf/%s" % branch,
                ]
                with self.Timer("prune-git-branches"):
                    # Deletions are done as trainbranchbot if available
                    gerrit_ssh_env = self.get_gerrit_ssh_env()

                    logger.info("Deleting mediawiki/core branch")
                    if (
                        subprocess.call(
                            ["git", "-C", branch_dir] + prune_branch_args,
                            env=gerrit_ssh_env,
                        )
                        != 0
                    ):
                        # When the deletion for core has failed, the branch is
                        # still a superproject in Gerrit and the deletion of
                        # subprojects would cause it to try to craft an object
                        # for the deletion. That is unsupported and fail in
                        # Gerrit 3.10.2.
                        #
                        # We thus must have successfuly deleted the
                        # superproject before the tracked projects and
                        # since that failed: We Skip 'Em All ™
                        logger.info(
                            "Failed to prune core branch, "
                            "skipping git branch deletions"
                        )
                    else:
                        # Prune all the submodules' remote branches
                        logger.info("Deleting branch from other projects")
                        for submodule_path in Clean._get_submodules_paths(branch_dir):
                            completed = subprocess.run(
                                ["git", "-C", submodule_path] + prune_branch_args,
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.PIPE,
                                env=gerrit_ssh_env,
                            )
                            if completed.returncode != 0:
                                logger.info(
                                    "Failed deleting branch in %s", submodule_path
                                )
                                print(completed.stderr, file=sys.stderr)

        logger.info("Clean %s", branch_dir)
        self._maybe_delete(branch_dir)

        patch_base_dir = self.config["patch_path"]
        branch_patch_dir = os.path.join(patch_base_dir, branch)
        logger.info("Clean %s", branch_patch_dir)
        self._maybe_delete(branch_patch_dir)
        srv_patches_git_message = 'Scap clean for "{}"'.format(branch)
        git.add_all(patch_base_dir, message=srv_patches_git_message)

    @staticmethod
    def _get_submodules_paths(git_dir):
        "Return a list of absolute paths for each submodule within `git_dir`"
        # fmt: off
        try:
            submodules_paths = subprocess.check_output([
                "git",
                "-C", git_dir,
                "config",
                "--null",
                "--file", ".gitmodules",
                "--get-regexp", "^submodule\\..*\\.path$",
            ], text=True)
        except Exception as e:
            raise e
        # fmt: on
        return [
            os.path.join(git_dir, kv.split("\n", maxsplit=1)[1])
            for kv in submodules_paths.rstrip("\0").split("\0")
        ]

    def _clean_remote_caches(self, target_hosts):
        """
        Need to remove cache dirs manually after sync
        """

        cache_dirs = [
            os.path.join(self.config["deploy_dir"], "php-%s" % branch, "cache")
            for branch in self.versions_to_remove
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
        for version in utils.get_wikiversions_ondisk(self.config["stage_dir"]):
            if version in active_versions:
                break
            old_versions.append(version)

        # Return all but the most recent old version.
        return old_versions[:-1]
