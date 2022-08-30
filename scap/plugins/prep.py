# -*- coding: utf-8 -*-
"""Scap plugin for setting up a new version of MediaWiki for deployment."""
import argparse
import glob
import os
import re
import shutil
import subprocess

from scap import cli
from scap import git
from scap import history
from scap import log
from scap import utils
from scap.lock import TimeoutLock


HISTORY_ABORT_STATUS = 127


def version_parser(ver):
    """Validate our version number formats."""
    if ver == "auto":
        return ver

    match = re.match(r"(1\.\d\d\.\d+-wmf\.\d+|master)", ver)

    if match:
        return match.group(0)
    raise argparse.ArgumentTypeError(
        "Branch '%s' does not match required format" % ver
    )


def update_update_strategy(path):
    """For all submodules, update the merge strategy."""
    base_cmd = "/usr/bin/git -C %s config " % path
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


@cli.command("prep", help="Checkout MediaWiki version to staging")
class CheckoutMediaWiki(cli.Application):
    """Checkout a version of MediaWiki to staging.

scap prep ensures that the specified version of MediaWiki (and
submodules) is checked out into the staging directory.  The checkout
will be updated to match origin.  Any previously applied security
patches or local changes will be discarded.

If you use 'scap prep auto', operations/mediawiki-config will be
cloned/updated in the staging directory and all active MediaWiki
versions will be prepped and security patches will be applied.

This operation can be run as many times as needed.

    """

    replay_history = None
    new_history = None

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
        action='store_true',
    )
    @cli.argument(
        "branch",
        metavar="BRANCH",
        type=version_parser,
        help="The name of the branch to operate on.  Specify 'auto' to check out operations/mediawiki-config and all active branches, and apply patches",
    )
    @cli.argument(
        "--copy-private-settings",
        default=None,
        metavar="FILE",
        help="Copy the specified file into private/ in the staging directory.  Only used in auto mode.",
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
        help="Don't apply security patches.  Only used in auto mode.",
    )
    def main(self, *extra_args):
        """Checkout next MediaWiki."""

        logger = self.get_logger()

        os.umask(self.config["umask"])

        lock_timeout = \
            {"timeout": self.arguments.lock_timeout} if self.arguments.lock_timeout else {}
        with TimeoutLock(self.get_serial_lock_file(), name="concurrent prep", **lock_timeout):

            self.new_history = history.Entry.now()

            if self.arguments.branch == "auto" and self.arguments.history:
                display_repos = ['mediawiki/core', 'operations/mediawiki-config']
                logger.info("Browsing history")
                hist = history.load(self.config["history_log"], display_repos=display_repos)
                self.replay_history = hist.browse()
                if self.replay_history is None:
                    logger.info("No history selected. Aborting.")
                    return HISTORY_ABORT_STATUS
                else:
                    summary = self.replay_history.summary(display_repos)
                    prompt = "Replay checkouts from %s?" % summary
                    if not utils.prompt_user_for_confirmation(prompt):
                        logger.info("Aborting.")
                        return HISTORY_ABORT_STATUS
                    logger.info("Replaying history: %s" % summary)

            with log.Timer("scap prep {}".format(self.arguments.branch), self.get_stats()):
                try:
                    if self.arguments.branch == "auto":
                        self._clone_or_update_repo(
                            os.path.join(self.config["gerrit_url"], "operations/mediawiki-config"),
                            self.config["operations_mediawiki_config_branch"],
                            self.config["stage_dir"],
                            logger,
                            )

                        if self.arguments.copy_private_settings:
                            self._copy_private_settings(self.arguments.copy_private_settings, logger)

                        for version in self.active_wikiversions("stage"):
                            self._prep_mw_branch(version, logger, apply_patches=self.arguments.apply_patches)
                    else:
                        self._prep_mw_branch(self.arguments.branch, logger)

                    self.new_history.completed = True
                finally:
                    history.log(self.new_history, self.config["history_log"])

    def _copy_private_settings(self, src, logger):
        dest = os.path.join(self.config["stage_dir"], "private", "PrivateSettings.php")
        logger.info("Copying {} to {}".format(src, dest))
        # copy2 preserves file mode and modification time
        shutil.copy2(src, dest)

    def _prep_mw_branch(self, branch, logger, apply_patches=False):
        dest_dir = os.path.join(
            self.config["stage_dir"],
            "{}{}".format(self.arguments.prefix, branch),
        )

        checkout_version = "master"
        if branch != "master":
            checkout_version = "wmf/%s" % branch

        reference_dir = None
        if checkout_version != "master":
            reference_dir = self._select_reference_directory()
            self._setup_patches(branch)

        # Note that this discards any local commits (e.g., security patches).
        self._clone_or_update_repo(
            os.path.join(self.config["gerrit_url"], "mediawiki/core"),
            checkout_version,
            dest_dir,
            logger,
            reference=reference_dir,
        )

        if checkout_version == "master":
            self._master_stuff(dest_dir, logger)
        else:
            gitmodules = os.path.join(dest_dir, ".gitmodules")
            if not os.path.exists(gitmodules):
                raise SystemExit("{} does not exist. Did the train branch commit get merged?".format(gitmodules))

        # This is only needed while people still do manual checkout
        # manipulation (a practice which needs to end).
        update_update_strategy(dest_dir)

        logger.debug("Creating LocalSettings.php stub")
        write_settings_stub(os.path.join(dest_dir, "LocalSettings.php"))

        cache_dir = os.path.join(dest_dir, "cache")
        if os.geteuid() == os.stat(cache_dir).st_uid:
            logger.debug("Making cache dir world writable")
            os.chmod(cache_dir, os.stat(cache_dir).st_mode | 0o777)

        if apply_patches:
            with utils.suppress_backtrace():
                args = [
                    self.get_script_path(), "apply-patches",
                    "--abort-git-am-on-fail", "--train", branch
                ] + self.format_passthrough_args()
                subprocess.check_call(args)

        logger.info(
            "MediaWiki %s successfully checked out." % checkout_version
        )

    def _select_reference_directory(self):
        """
        Find the latest /srv/mediawiki-staging/php-<version> directory to use as a reference
        when cloning a new mediawiki checkout.

        Returns None if unavailable.
        """
        candidates = glob.glob(os.path.join(self.config["stage_dir"], "php-*"))

        if not candidates:
            return None

        return sorted(candidates, key=utils.parse_wmf_version)[-1]

    def _select_reference_patches(self):
        """
        Find the latest /srv/patches/<version> directory to copy
        to set up a new version.

        Returns None if unavailable.
        """

        patch_base_dir = self.config["patch_path"]

        candidates = [
            name
            for name in os.listdir(patch_base_dir)
            if re.match(utils.BRANCH_RE, name)
        ]

        if not candidates:
            return None

        latest_patches_vers = sorted(candidates, key=utils.parse_wmf_version)[-1]

        return os.path.join(patch_base_dir, latest_patches_vers)

    def _setup_patches(self, version):
        logger = self.get_logger()

        logger.debug("Setting up patches for {}".format(version))

        patch_base_dir = self.config["patch_path"]
        patch_path = os.path.join(patch_base_dir, version)
        if os.path.exists(patch_path):
            logger.debug("Patches already set up for {}".format(version))
            return

        reference_patches = self._select_reference_patches()

        if not reference_patches:
            logger.warn("No reference patches available to copy")
            return

        logger.info("Copying patches from {} to {}".format(reference_patches, patch_path))
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
            head = git.clone_or_update_repo(dir, repo, branch, logger, reference,
                                            ref=ref)

            # Ensure all repositories have a ssh push url
            repo_name = os.path.relpath(repo, self.config["gerrit_url"])
            git.gitcmd("remote", "set-url", "--push", "origin",
                       os.path.join(self.config["gerrit_push_url"], repo_name),
                       cwd=dir)

        self.new_history.update(repo, branch, dir, head)

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
                logger.info("Deleting placeholder {} so it will be replaced with a clone of {}".format(path, repo))
                shutil.rmtree(path)

            self._clone_or_update_repo(repo,
                                       "master",
                                       path,
                                       logger)

            with open(gitignore_path, "a") as f:
                f.write("# Added by scap prep auto\n")
                f.write("/{}\n".format(type))

        # After checking out extensions/skins, some of the placeholder
        # files from mediawiki/core will have been replaced.  This
        # add_all captures any of those differences so that we end up
        # with a clean checkout.
        git.add_all(branch_dir, "scap prep auto setup")  # This commits
