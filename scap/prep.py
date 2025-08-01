# -*- coding: utf-8 -*-
"""Scap command for setting up a new version of MediaWiki for deployment."""
import glob
import os
import re
import shlex
import shutil
import subprocess

from scap import cli, git, history, patches, utils
from scap.patches import SecurityPatches, finalize_next_patches, update_next_patches

HISTORY_ABORT_STATUS = 127


def update_update_strategy(path):
    """For all submodules, update the merge strategy."""
    base_cmd = "/usr/bin/git -C %s config " % shlex.quote(path)
    base_cmd += "submodule.$name.update rebase"
    git.gitcmd("submodule", "foreach", "--recursive", base_cmd, cwd=path)


def write_settings_stub(dest):
    """Write a silly little PHP file that includes another."""
    file_stub = (
        "<?php\n"
        + "# Managed by scap prep\n"
        + "# WARNING: This file is publically viewable on the web. "
        + "Do not put private data here.\n"
        + 'require __DIR__ . "/../wmf-config/CommonSettings.php";\n'
    )

    if os.path.exists(dest):
        with open(dest, "r") as destfile:
            if destfile.read() == file_stub:
                return

    with open(dest, "w+") as destfile:
        destfile.write(file_stub)


@cli.command(
    "prep",
    help="Checkout MediaWiki version to staging",
    primary_deploy_server_only=True,
)
class CheckoutMediaWiki(cli.Application):
    """Checkout a version of MediaWiki to staging.

    scap prep ensures that the specified version of MediaWiki (and
    submodules) is checked out into the staging directory.  The checkout
    will be updated to match origin.  Any previously applied security
    patches or local changes will be discarded.  Security patches are
    reapplied after the checkout is updated.

    If you use 'scap prep auto', operations/mediawiki-config will be
    cloned/updated in the staging directory and all active MediaWiki
    versions will be prepped and security patches will be applied.

    This operation can be run as many times as needed.

    """

    replay_history = None

    @cli.argument(
        "branch",
        metavar="BRANCH",
        type=utils.version_argument_parser,
        help="The name of the branch to operate on.  Specify 'auto' to enable auto mode and select all active branches",
    )
    @cli.argument(
        "--auto",
        help="Enable automatic mode which will check out operations/mediawiki-config",
        action="store_true",
    )
    @cli.argument(
        "-p",
        "--prefix",
        nargs=1,
        required=False,
        default="php-",
        metavar="PREFIX",
        help="Directory prefix to checkout version to.",
    )
    @cli.argument(
        "--history",
        help=(
            "Browse prep transaction history and choose a previous set of "
            "refs to checkout. Only used in auto mode."
        ),
        action="store_true",
    )
    @cli.argument(
        "--copy-private-settings",
        default=None,
        metavar="FILE",
        help=(
            "Copy the specified file(s) into private/ in the staging "
            "directory. Supports glob patterns. Only used in auto mode."
        ),
    )
    @cli.argument(
        "--lock-timeout",
        type=int,
        help="Timeout to wait for the prep concurrency lock to be released. In minutes",
    )
    @cli.argument(
        "--no-patches",
        dest="apply_patches",
        action="store_false",
        help="Don't apply security patches.",
    )
    @cli.argument(
        "--reference",
        default=None,
        metavar="DIR",
        help="Git reference directory to use when fetching MediaWiki branches.",
    )
    def main(self, *extra_args):
        """Checkout next MediaWiki."""

        logger = self.get_logger()

        os.umask(self.config["umask"])

        if self.arguments.branch == "auto":
            self.arguments.auto = True

        lock_timeout = (
            {"timeout": self.arguments.lock_timeout}
            if self.arguments.lock_timeout
            else {}
        )
        with self.lock_mediawiki_staging(name="concurrent prep", **lock_timeout):
            if self.arguments.auto and self.arguments.history:
                display_repos = ["mediawiki/core", "operations/mediawiki-config"]
                logger.info("Browsing history")
                hist = history.History(self.scap_history_dbfile())
                self.replay_history = hist.browse(display_repos)
                if self.replay_history is None:
                    logger.info("No history selected. Aborting.")
                    return HISTORY_ABORT_STATUS
                else:
                    summary = self.replay_history.summary(display_repos)
                    prompt = "Replay checkouts from %s?" % summary
                    if not self.prompt_user_for_confirmation(prompt):
                        logger.info("Aborting.")
                        return HISTORY_ABORT_STATUS
                    logger.info("Replaying history: %s" % summary)

            with self.Timer(
                description="scap prep {}".format(self.arguments.branch), name="prep"
            ):
                if self.arguments.auto:
                    self.report_status("Checking out mediawiki-config")
                    self._clone_or_update_repo(
                        os.path.join(
                            self.config["gerrit_url"], "operations/mediawiki-config"
                        ),
                        self.config["operations_mediawiki_config_branch"],
                        self.config["stage_dir"],
                        logger,
                    )

                    if self.arguments.copy_private_settings:
                        self._copy_private_settings(
                            self.arguments.copy_private_settings, logger
                        )

                versions_to_prep = (
                    self.active_wikiversions("stage")
                    if self.arguments.branch == "auto"
                    else [self.arguments.branch]
                )

                for version in versions_to_prep:
                    with self.reported_status(f"Checking out {version}"):
                        self._prep_mw_branch(
                            version,
                            logger,
                            apply_patches=self.arguments.apply_patches,
                            reference_dir=self.arguments.reference,
                        )

    def _copy_private_settings(self, src_glob, logger):
        dest = os.path.join(self.config["stage_dir"], "private")
        for src in glob.glob(src_glob):
            logger.info("Copying {} to {}".format(src, dest))
            # copy2 preserves file mode and modification time
            shutil.copy2(src, dest)

    def _prep_mw_branch(self, branch, logger, apply_patches=False, reference_dir=None):
        dest_dir = os.path.join(
            self.config["stage_dir"],
            "{}{}".format(self.arguments.prefix, branch),
        )

        checkout_version = "master"
        if branch != "master":
            checkout_version = "wmf/%s" % branch

        if reference_dir is None:
            reference_dir = self._select_reference_directory()

        if checkout_version != "master":
            self._setup_patches(branch)

        _patches = patches.SecurityPatches(
            os.path.join(self.config["patch_path"], branch)
        )
        pre_patch_state = _patches.get_pre_patch_state(dest_dir)

        # Note that this discards any local commits (e.g., security patches).
        self._clone_or_update_repo(
            os.path.join(self.config["gerrit_url"], "mediawiki/core"),
            checkout_version,
            dest_dir,
            logger,
            reference=reference_dir,
        )

        cache_dir = os.path.join(dest_dir, "cache")
        if os.geteuid() == os.stat(cache_dir).st_uid:
            logger.debug("Making cache dir world writable")
            os.chmod(cache_dir, os.stat(cache_dir).st_mode | 0o777)

        if checkout_version == "master":
            self._master_stuff(dest_dir, logger)
        else:
            gitmodules = os.path.join(dest_dir, ".gitmodules")
            if not os.path.exists(gitmodules):
                raise SystemExit(
                    "{} does not exist. Did the train branch commit get merged?".format(
                        gitmodules
                    )
                )

        # This is only needed while people still do manual checkout
        # manipulation (a practice which needs to end).
        update_update_strategy(dest_dir)

        logger.debug("Creating LocalSettings.php stub")
        write_settings_stub(os.path.join(dest_dir, "LocalSettings.php"))

        if apply_patches:
            with utils.suppress_backtrace():
                args = [
                    self.get_script_path(),
                    "apply-patches",
                    "--abort-git-am-on-fail",
                    "--train",
                    branch,
                ] + self.format_passthrough_args()
                subprocess.check_call(args)
            SecurityPatches.fix_mtimes(pre_patch_state)

        logger.info("MediaWiki %s successfully checked out." % checkout_version)

    def _select_reference_directory(self):
        """
        Find the latest /srv/mediawiki-staging/php-<version> directory to use as a reference
        when cloning a new mediawiki checkout.

        Returns None if unavailable.
        """

        latest_path = None
        latest_version = None

        for path in glob.glob(os.path.join(self.config["stage_dir"], "php-*")):
            version = os.path.basename(path)[len("php-") :]
            if not utils.valid_version(version):
                continue

            version = utils.parse_wmf_version(version)
            if latest_version is None or version > latest_version:
                latest_version = version
                latest_path = path

        return latest_path

    def _setup_patches(self, version):
        logger = self.get_logger()
        patch_base_dir = self.config["patch_path"]
        patch_path = os.path.join(patch_base_dir, version)

        logger.debug("Setting up patches for {}".format(version))

        if version == "next":
            update_next_patches(patch_base_dir, logger)

        if os.path.exists(patch_path):
            logger.debug("Patches already set up for {}".format(version))
            return

        reference_patches = utils.select_latest_patches(patch_base_dir)

        if not reference_patches:
            logger.warning("No reference patches available to copy")
            return

        next_patches_dir = os.path.join(patch_base_dir, "next")
        if reference_patches == next_patches_dir:
            update_next_patches(patch_base_dir, logger)
            finalize_next_patches(next_patches_dir, logger)

        logger.info(
            "Copying patches from {} to {}".format(reference_patches, patch_path)
        )
        shutil.copytree(reference_patches, patch_path)

        # This also commits.
        git.add_all(patch_base_dir, message='Scap prep for "{}"'.format(version))
        logger.debug("Done setting patches for {}".format(version))

    def _clone_or_update_repo(self, repo, branch, dir, logger, reference=None):
        """
        Note that this discards any local commits (e.g., security patches)
        """

        ref = None
        if self.replay_history is not None:
            ref = self.replay_history.lookup(repo, branch, dir)

        with utils.suppress_backtrace():
            git.clone_or_update_repo(dir, repo, branch, logger, reference, ref=ref)

            # Ensure we have a ssh push on the parent project
            repo_name = os.path.relpath(repo, self.config["gerrit_url"])
            git.gitcmd(
                "remote",
                "set-url",
                "--push",
                "origin",
                os.path.join(self.config["gerrit_push_url"], repo_name),
                cwd=dir,
            )

            # And `scap clean` needs to be able to push to the branch deletions
            # in submodules. Set pushInsteadOf in the local config
            git.gitcmd(
                "submodule",
                "foreach",
                "git",
                "config",
                "--local",
                "--replace-all",
                "url.%s.pushInsteadOf" % self.config["gerrit_push_url"].rstrip("/"),
                self.config["gerrit_url"].rstrip("/"),
                cwd=dir,
            )

            # If the repo checkout is being reused, that can cause problems when a
            # module is removed upstream from `.gitmodules`. We detect that situation
            # and clean the local checkout
            self._check_for_removed_submodules(dir)

    def _check_for_removed_submodules(self, local_repo_dir):
        status_output = git.gitcmd("status", "--porcelain", cwd=local_repo_dir)
        untracked_files = [
            # `out` looks like something like "?? extensions/Cite/"
            re.sub(r"/$", "", out.split()[1])
            for out in status_output.splitlines()
            if "??" in out
        ]

        for untracked_file in untracked_files:
            # Check if submodule dir
            try:
                subprocess.check_call(
                    ["git", "config", f"submodule.{untracked_file}.url"],
                    cwd=local_repo_dir,
                )
            except subprocess.CalledProcessError:
                # Not a leftover submodule dir, ignore
                continue

            # If no exception, we found a dirty submodule dir. Let's clean

            submodule_dir = os.path.join(local_repo_dir, untracked_file)
            self.get_logger().info(f"Found orphan submodule. Deleting {submodule_dir}")
            shutil.rmtree(submodule_dir)

            git_submodule_dir = os.path.join(
                local_repo_dir, ".git/modules", untracked_file
            )
            if os.path.isdir(git_submodule_dir):
                shutil.rmtree(git_submodule_dir)

            git.gitcmd(
                "config",
                "--remove-section",
                f"submodule.{untracked_file}",
                cwd=local_repo_dir,
            )

    def _master_stuff(self, branch_dir, logger):
        # On train branches of mediawiki/core, extensions/vendors/skins are submodules.
        # On the master branch of mediawiki/core, they are not submodules and must be handled
        # specially.

        gitignore_path = os.path.join(branch_dir, ".gitignore")

        for type in ["extensions", "vendor", "skins"]:
            repo = os.path.join(self.config["gerrit_url"], "mediawiki/{}".format(type))
            path = os.path.join(branch_dir, type)

            # If mediawiki/core has just been freshly cloned, the
            # extensions/ and skins/ directories will be populated
            # with a few placeholder files (including .gitignore).  We
            # need the directories to be empty or non-existent so that
            # we can clone the corresponding mediawiki/<repo>.

            if os.path.exists(path) and not git.is_dir(path):
                logger.info(
                    "Deleting placeholder {} so it will be replaced with a clone of {}".format(
                        path, repo
                    )
                )
                shutil.rmtree(path)

            self._clone_or_update_repo(repo, "master", path, logger)

            with open(gitignore_path, "a") as f:
                f.write("# Added by scap prep auto\n")
                f.write("/{}\n".format(type))

        # After checking out extensions/skins, some of the placeholder
        # files from mediawiki/core will have been replaced.  This
        # add_all captures any of those differences so that we end up
        # with a clean checkout.
        git.add_all(branch_dir, "scap prep auto setup")  # This commits
