# -*- coding: utf-8 -*-

"""Scap plugin for applying security patches for the train."""

import os
import re
import sys

from scap import cli, utils
from scap.runcmd import gitcmd, FailedCommand

APPLIED = 1
ALREADY_APPLIED = 2
FAILED = 3
SKIPPED = 4

KNOWN_RESULTS = {
    APPLIED: "APPLIED",
    ALREADY_APPLIED: "ALREADY APPLIED",
    FAILED: "FAILED",
    SKIPPED: "SKIPPED",
}


@cli.command(
    "apply-patches",
    help="Apply security patches for train",
    affected_by_blocked_deployments=True,
)
class ApplyPatches(cli.Application):
    """Apply security patches for train"""

    @cli.argument(
        "--train",
        action="store",
        help='train version number (e.g., "1.35.0-wmf30")',
        metavar="TRAIN",
        required=True,
    )
    @cli.argument(
        "--abort-git-am-on-fail",
        action="store_true",
        help='Run "git am --abort" if patch application fails',
        required=False,
    )
    def main(self, *extra_args):
        train = self.arguments.train
        srcroot = os.path.abspath(
            os.path.join(self.config["stage_dir"], "php-%s" % train)
        )
        patch_path = os.path.abspath(os.path.join(self.config["patch_path"], train))

        patches = SecurityPatches(patch_path)

        results = []

        curdir = None
        apply_in_curdir = True

        if len(patches) == 0 and self.config["require_security_patches"]:
            raise SystemExit(
                f"No security patches found for {train}!\nRun again with -Drequire_security_patches:False to disable this check."
            )

        for patch in patches:
            if patch.dirname() != curdir:
                apply_in_curdir = True
                curdir = patch.dirname()

            if apply_in_curdir:
                ret = patch.apply(srcroot, self.arguments.abort_git_am_on_fail)
                if ret not in KNOWN_RESULTS:
                    sys.exit("Patch.apply returned unknown value {!r}".format(ret))
                if ret == FAILED:
                    apply_in_curdir = False
            else:
                ret = SKIPPED

            results.append((ret, patch))

        for ret, patch in results:
            print("[{}] {}".format(KNOWN_RESULTS[ret], patch.path()))

        any_failed = any(ret == FAILED for (ret, _) in results)
        if any_failed:
            sys.exit("At least one patch failed to apply")

        return 0


class SecurityPatches:
    """A list of security patches."""

    def __init__(self, root):
        self._patches = self._find(root)

    def _find(self, root):
        core = os.path.abspath(os.path.join(root, "core"))
        exts = os.path.abspath(os.path.join(root, "extensions"))
        skins = os.path.abspath(os.path.join(root, "skins"))

        patches = []

        simples = [(core, ".")]
        for dirname, relative in simples:
            if os.path.exists(dirname):
                for filename in os.listdir(dirname):
                    patch_pathname = os.path.join(dirname, filename)
                    patches.append(Patch(patch_pathname, relative))

        extradirs = []
        if os.path.exists(exts):
            extradirs.append(exts)
        if os.path.exists(skins):
            extradirs.append(skins)

        for extradir in extradirs:
            basename = os.path.basename(extradir)
            for subdir in os.listdir(extradir):
                dirname = os.path.join(extradir, subdir)
                for filename in os.listdir(dirname):
                    patch_pathname = os.path.join(dirname, filename)
                    patches.append(
                        Patch(patch_pathname, os.path.join(basename, subdir))
                    )

        return list(sorted(patches, key=lambda p: p.path()))

    def __iter__(self):
        for patch in self._patches:
            yield patch

    def __len__(self):
        return len(self._patches)

    def get_pre_patch_state(self, srcroot) -> dict:
        """
        Returns a dictionary representing the state (mtime and hash) of each file
        affected by security patches.   This is expected to be called before
        scap prep auto does any modifications to the mediawiki checkouts.
        This information should be passed to fix_mtimes() after patches are applied.
        """

        res = {}
        for patch in self:
            res.update(patch.get_pre_patch_state(srcroot))
        return res

    def fix_mtimes(self, pre_patch_state):
        """
        This method is expected to be called after security patches have been
        applied.  It reverts the mtime on any patched files if their contents
        haven't changed since get_pre_patch_state() was called.  This is to avoid
        unnecessary mtime-change-triggered operations that might follow (such as
        l10n rebuild).
        """
        for filename in pre_patch_state:
            if os.path.exists(filename):
                new_mtime = os.stat(filename).st_mtime
                new_hash = utils.md5_file(filename)
                old_mtime, old_hash = pre_patch_state[filename]

                if old_mtime != new_mtime and old_hash == new_hash:
                    # mtime changed but contents didn't.  Set it back
                    os.utime(filename, (old_mtime, old_mtime))


class Patch:
    def __init__(self, pathname, relative):
        self._filename = pathname
        self._relative = relative

    def dirname(self):
        return os.path.dirname(self._filename)

    def path(self):
        return self._filename

    def _affected_files(self, srcroot) -> list:
        res = []
        with open(self.path()) as f:
            for line in f.readlines():
                m = re.match(r"diff --git a/(.*) b/(.*)", line)
                if m:
                    a, b = m[1], m[2]
                    if a != b:
                        raise Exception(f"Patch a != b: {a} != {b}")
                    res.append(os.path.join(srcroot, self._relative, a))
        return res

    def get_pre_patch_state(self, srcroot):
        state = {}
        for filename in self._affected_files(srcroot):
            if os.path.exists(filename):
                state[filename] = (os.stat(filename).st_mtime, utils.md5_file(filename))
        return state

    def apply(self, srcroot, abort_git_am_on_fail):
        srcdir = os.path.join(srcroot, self._relative)
        print("Applying patch %s in %s" % (self.path(), srcroot))

        try:
            if not git_is_clean(srcdir):
                print("ERROR: git is not clean: %s" % srcdir)
                return FAILED
        except FailedCommand as e:
            print("ERROR: git is clean: %s" % e.stderr)
            return FAILED

        try:
            output = gitcmd("am", "--3way", self.path(), cwd=srcdir)
        except FailedCommand as e:
            print("ERROR: git am: %s" % e.stderr)
            if abort_git_am_on_fail:
                try:
                    gitcmd("am", "--abort", cwd=srcdir)
                except Exception:
                    pass
            return FAILED

        if "already applied" in output:
            return ALREADY_APPLIED
        return APPLIED


def valid_patch_filename(filename):
    m = re.match(r"^\d+-\S+\.patch$", filename)
    return m is not None


def git_is_clean(dirname):
    output = gitcmd("status", "--ignore-submodules", cwd=dirname)
    if "Changes not staged" in output:
        return False
    if "You are in the middle of an am session." in output:
        return False
    return True
