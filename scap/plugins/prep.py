# -*- coding: utf-8 -*-
"""Scap plugin for setting up a new version of MediaWiki for deployment."""
import argparse
import distutils
import glob
import json
import os
import re
import shutil
import subprocess

from scap import cli
from scap import git
from scap import log
from scap import utils

SOURCE_URL = "https://gerrit.wikimedia.org/r/"


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
        "--fingerprint",
        help="Check out according to the supplied fingerprint.  Only used in auto mode.",
        required=False,
        default=None,
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
    def main(self, *extra_args):
        """Checkout next MediaWiki."""

        logger = self.get_logger()

        with log.Timer("prep", self.get_stats()):
            if self.arguments.branch == "auto":
                logger.info("auto mode")

                fingerprint = self._decode_fingerprint(self.arguments.fingerprint)

                self._clone_or_update_repo(os.path.join(SOURCE_URL, "operations/mediawiki-config"),
                                           self.config["operations_mediawiki_config_branch"],
                                           self.config["stage_dir"],
                                           logger,
                                           fingerprint=fingerprint,
                                           )

                if self.arguments.copy_private_settings:
                    self._copy_private_settings(self.arguments.copy_private_settings, logger)

                for version in self.active_wikiversions("stage"):
                    self._prep_mw_branch(version, logger, apply_patches=True,
                                         fingerprint=fingerprint)

                print("fingerprint:")
                print(json.dumps(fingerprint, sort_keys=True))
            else:
                self._prep_mw_branch(self.arguments.branch, logger)

    def _copy_private_settings(self, src, logger):
        dest = os.path.join(self.config["stage_dir"], "private", "PrivateSettings.php")
        logger.info("Copying {} to {}".format(src, dest))
        # copy2 preserves file mode and modification time
        shutil.copy2(src, dest)

    def _prep_mw_branch(self, branch, logger, apply_patches=False,
                        fingerprint=None):
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
        self._clone_or_update_repo(os.path.join(SOURCE_URL, "mediawiki/core"),
                                   checkout_version,
                                   dest_dir,
                                   logger,
                                   reference=reference_dir,
                                   fingerprint=fingerprint)

        if checkout_version == "master":
            self._master_stuff(dest_dir, logger, fingerprint)

        # This is only needed while people still do manual checkout
        # manipulation (a practice which needs to end).
        update_update_strategy(dest_dir)

        logger.info("Creating LocalSettings.php stub")
        write_settings_stub(os.path.join(dest_dir, "LocalSettings.php"))

        logger.info("Making cache dir world writable")
        cache_dir = os.path.join(dest_dir, "cache")
        os.chmod(cache_dir, 0o777)

        logger.info(
            "MediaWiki %s successfully checked out." % checkout_version
        )

        if apply_patches:
            with utils.suppress_backtrace():
                args = [
                    self.get_script_path(), "apply-patches",
                    "--abort-git-am-on-fail", "--train", branch
                ] + self.format_passthrough_args()
                subprocess.check_call(args)

    def _select_reference_directory(self):
        """
        Find the latest /srv/mediawiki-staging/php-<version> directory to use as a reference
        when cloning a new mediawiki checkout.

        Returns None if unavailable.
        """
        candidates = glob.glob(os.path.join(self.config["stage_dir"], "php-*"))

        if not candidates:
            return None

        return sorted(candidates, key=distutils.version.LooseVersion)[-1]

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

        latest_patches_vers = sorted(candidates, key=distutils.version.LooseVersion)[-1]

        return os.path.join(patch_base_dir, latest_patches_vers)

    def _setup_patches(self, version):
        logger = self.get_logger()

        logger.info("Setting up patches for {}".format(version))

        patch_base_dir = self.config["patch_path"]
        patch_path = os.path.join(patch_base_dir, version)
        if os.path.exists(patch_path):
            logger.info("Patches already set up for {}".format(version))
            return

        reference_patches = self._select_reference_patches()

        if not reference_patches:
            logger.warn("No reference patches available to copy")
            return

        logger.info("Copying patches from {} to {}".format(reference_patches, patch_path))
        shutil.copytree(reference_patches, patch_path)

        # This also commits.
        git.add_all(patch_base_dir, message='Scap prep for "{}"'.format(version))
        logger.info("Done setting patches for {}".format(version))

    def _clone_or_update_repo(self, repo, branch, dir, logger, reference=None,
                              fingerprint=None):
        """
        Note that this discards any local commits (e.g., security patches)
        """

        ref = None
        if isinstance(fingerprint, dict):
            ref = self._lookup_fingerprint(fingerprint, repo, branch)

        with utils.suppress_backtrace():
            head = git.clone_or_update_repo(dir, repo, branch, logger, reference,
                                            ref=ref)

        if isinstance(fingerprint, dict):
            self._update_fingerprint(fingerprint, repo, branch, head)

    def _update_fingerprint(self, fingerprint, repo, branch, head):
        branches = fingerprint.get(repo)
        if branches is None:
            branches = fingerprint[repo] = {}

        branches[branch] = head

    def _lookup_fingerprint(self, fingerprint, repo, branch):
        return fingerprint.get(repo, {}).get(branch)

    def _decode_fingerprint(self, thing):
        if not isinstance(thing, str):
            return {}

        return json.loads(thing)

    def _master_stuff(self, branch_dir, logger, fingerprint):
        # On train branches of mediawiki/core, extensions/vendors/skins are submodules.
        # On the master branch of mediawiki/core, they are not submodules and must be handled
        # specially.

        gitignore_path = os.path.join(branch_dir, ".gitignore")

        for type in ["extensions", "vendor", "skins"]:

            repo = os.path.join(SOURCE_URL, "mediawiki/{}".format(type))
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
                                       logger,
                                       fingerprint=fingerprint)

            with open(gitignore_path, "a") as f:
                f.write("# Added by scap prep auto\n")
                f.write("/{}\n".format(type))

        # After checking out extensions/skins, some of the placeholder
        # files from mediawiki/core will have been replaced.  This
        # add_all captures any of those differences so that we end up
        # with a clean checkout.
        git.add_all(branch_dir, "scap prep auto setup")  # This commits
