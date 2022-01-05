# -*- coding: utf-8 -*-

"""Scap plugin for applying security patches for the train."""

import os
import re
import sys

from scap import cli
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


@cli.command("apply-patches", help="Apply security patches for train")
class ApplyPatches(cli.Application):
    """Apply security patches for train"""

    @cli.argument(
        "--train",
        action="store",
        help='train version number (e.g., "1.35.0-wmf30")',
        metavar="TRAIN",
        required=True,
    )
    def main(self, *extra_args):
        train = self.arguments.train
        srcroot = os.path.abspath(
            os.path.join(self.config["stage_dir"], "php-%s" % train)
        )
        patch_path = os.path.abspath(
            os.path.join(self.config["patch_path"], train)
        )

        patches = SecurityPatches(patch_path)

        results = []

        curdir = None
        apply_in_curdir = True

        for patch in patches:
            if patch.dirname() != curdir:
                apply_in_curdir = True
                curdir = patch.dirname()

            if apply_in_curdir:
                ret = patch.apply(srcroot)
                if ret not in KNOWN_RESULTS:
                    sys.exit("Patch.apply returned unknown value {!r}".format(ret))
                if ret == FAILED:
                    apply_in_curdir = False
            else:
                ret = SKIPPED

            results.append((ret, patch))

        for (ret, patch) in results:
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


class Patch:
    def __init__(self, pathname, relative):
        self._filename = pathname
        self._relative = relative

    def dirname(self):
        return os.path.dirname(self._filename)

    def path(self):
        return self._filename

    def apply(self, srcroot):
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
