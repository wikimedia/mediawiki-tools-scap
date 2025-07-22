# -*- coding: utf-8 -*-

"""Scap command for applying security patches for the train."""
import argparse
import filecmp
import os
import re
import shutil
import sys
from abc import abstractmethod
from datetime import timedelta, datetime
from typing import Iterator, Optional

from scap import cli, utils, git
from scap.phorge_conduit import PhorgeConduit
from scap.runcmd import gitcmd, FailedCommand
from scap.utils import BRANCH_RE

APPLIED = 1
ALREADY_APPLIED = 2
FAILED = 3
SKIPPED = 4
GIT_NOT_CLEAN = 5
ERROR = 6

KNOWN_RESULTS = {
    APPLIED: "APPLIED",
    ALREADY_APPLIED: "ALREADY APPLIED",
    FAILED: "FAILED",
    SKIPPED: "SKIPPED",
    GIT_NOT_CLEAN: "GIT NOT CLEAN",
    ERROR: "ERROR",
}


@cli.command(
    "apply-patches",
    help="Apply security patches for train",
    primary_deploy_server_only=True,
)
class ApplyPatches(cli.Application):
    """Apply security patches for train"""

    class NotificationInfo:
        def __init__(self, patches_path, patch, target_release_for_fixes):
            patch_info_re = rf"{patches_path}/[^/]+/(?P<module>.+)/(?P<patch_name>(\d+-)?(?P<patch_task>T\d+).patch)$"
            patch_info = re.match(patch_info_re, patch.path())

            self.patch_name = patch_info.group("patch_name")
            self.patch_task = patch_info.group("patch_task")
            # Core, or particular extension/skin
            self.module = patch_info.group("module")
            self.target_release_for_fixes = target_release_for_fixes

    @cli.argument(
        "--train",
        action="store",
        help='train version number (e.g., "1.35.0-wmf30")',
        metavar="TRAIN",
        required=True,
        type=lambda arg: utils.version_argument_parser(arg, allow_auto=False),
    )
    @cli.argument(
        "--abort-git-am-on-fail",
        action="store_true",
        help='Run "git am --abort" if patch application fails',
        required=False,
    )
    def main(self, *extra_args):
        def unsuccessful(res):
            return res in [FAILED, GIT_NOT_CLEAN, ERROR]

        self._post_init()

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

        def output_line(line):
            self.output_line(line, sensitive=True)

        for patch in patches:
            if patch.dirname() != curdir:
                apply_in_curdir = True
                curdir = patch.dirname()

            if apply_in_curdir and not patch.path().endswith(".dropped"):
                ret = patch.apply(
                    srcroot, self.arguments.abort_git_am_on_fail, output_line
                )
                if ret not in KNOWN_RESULTS:
                    sys.exit("Patch.apply returned unknown value {!r}".format(ret))
                if unsuccessful(ret):
                    apply_in_curdir = False
            else:
                ret = SKIPPED

            results.append((ret, patch))

        for ret, patch in results:
            output_line("[{}] {}".format(KNOWN_RESULTS[ret], patch.path()))

        any_failed = any(ret == FAILED for (ret, _) in results)
        if any_failed and self.config["notify_patch_failures"]:
            self._notify_failures(results)

        any_unsuccessful = any(unsuccessful(ret) for (ret, _) in results)
        if any_unsuccessful:
            sys.exit("At least one patch did not apply successfully")

        return 0

    def _post_init(self):
        """
        Initialization that requires self.config
        """

        def disable_patch_failure_notifications():
            self.get_logger().warning("Disabling patch failure notifications")
            self.config["notify_patch_failures"] = False

        if self.config["notify_patch_failures"]:
            patch_bot_phorge_token = self.config["patch_bot_phorge_token"]
            if not patch_bot_phorge_token or str(patch_bot_phorge_token).strip() == "":
                self.get_logger().warning("No patch bot token configured")
                disable_patch_failure_notifications()
                return

            self.phorge_conduit = PhorgeConduit(
                self.config["phorge_url"],
                self.config["patch_bot_phorge_token"],
                self.config["web_proxy"],
            )
            try:
                # Retrieve patch bot PHID programmatically
                patch_bot_name = self.config["patch_bot_phorge_name"]
                patch_bot_user = self.phorge_conduit.user_by_name(
                    patch_bot_name, {"isBot": True}
                )
                if patch_bot_user:
                    self.security_patch_bot_phid = patch_bot_user["phid"]
                else:
                    self.get_logger().warning(
                        f'Could not find patch bot user with name "{patch_bot_name}"'
                    )
                    disable_patch_failure_notifications()
            except Exception as e:
                self.get_logger().warning("Could not retrieve PHID of patch bot: %s", e)
                disable_patch_failure_notifications()

    def _notify_failures(self, results):
        self.get_logger().info("Notifying security patch tasks of the failures")

        try:
            target_release_for_fixes = self._determine_release_for_fixes()
            for patch in [patch for ret, patch in results if ret == FAILED]:
                self._notify_failure(patch, target_release_for_fixes)
        except Exception as e:
            self.get_logger().warning(
                "Failure while trying to notify security patch errors:\n%s", e
            )

    def _determine_release_for_fixes(self):
        def is_valid(task):
            return task["status"] not in ["declined", "invalid"]

        train_tasks = utils.get_train_blockers_info(
            self.config["train_blockers_url"], self.config["web_proxy"]
        )
        current = train_tasks["current"]

        # ## If the patches are being applied on a MW version branch (typically as part of the branch cut process) ## #
        if BRANCH_RE.match(self.arguments.train):
            if current["version"] != self.arguments.train:
                raise Exception(
                    f"Tried to apply patches to version {self.arguments.train} but current train version is"
                    f" {current['version']}. This situation is currently not handled by the patch notifier"
                )

            return current

        # ## If the patches are being applied on mainline branch (typically as part of nightly checks) ## #

        # If the current task is valid and its corresponding branch hasn't been created yet, that's where we want the
        # fixed patches to be
        most_recent_version = utils.get_current_train_version_from_gerrit(
            self.config["gerrit_url"]
        )
        if is_valid(current) and current["version"] != most_recent_version:
            return current

        # Otherwise, let's find the next valid train task
        tasks_by_date = train_tasks["dated"]
        current_task_date = datetime.strptime(current["date"], "%Y-%m-%d").date()
        next_task_date = current_task_date + timedelta(days=7)
        for _ in range(1, len(tasks_by_date)):
            next_task = tasks_by_date[next_task_date.isoformat()]
            if is_valid(next_task):
                return next_task
            next_task_date += timedelta(days=7)

        raise Exception(
            "Could not find a single valid release task scheduled! This should not happen"
        )

    def _notify_failure(self, patch, target_release_for_fixes):
        notification_info = ApplyPatches.NotificationInfo(
            self.config["patch_path"], patch, target_release_for_fixes
        )

        # If the failure was already reported, don't spam
        if self._patch_task_has_been_notified(notification_info):
            self.get_logger().info(
                f"""Failure for "{notification_info.module}/{notification_info.patch_name}" already notified."""
                " Skipping"
            )
            return

        release_task_id = notification_info.target_release_for_fixes["task_id"]
        release_task_phid = self.phorge_conduit.base_task_info(release_task_id)["phid"]
        notification = ApplyPatches._compose_notification(notification_info)
        self.phorge_conduit.edit_task(
            notification_info.patch_task,
            [
                {"type": "status", "value": "open"},
                {"type": "priority", "value": "unbreak"},
                {"type": "parents.add", "value": [release_task_phid]},
                {"type": "comment", "value": notification},
            ],
        )

    def _patch_task_has_been_notified(self, notification_info):
        base_task_info = self.phorge_conduit.base_task_info(
            notification_info.patch_task
        )

        def task_is_open():
            return base_task_info["fields"]["status"]["name"] == "Open"

        def priority_is_unbreak():
            return base_task_info["fields"]["priority"]["name"] == "Unbreak Now!"

        def is_subtask_of_release_task():
            release_task = notification_info.target_release_for_fixes["task_id"]
            return release_task in self.phorge_conduit.parents_of(
                notification_info.patch_task
            )

        def bot_comment_for_patch_is_present():
            bot_comments = self.phorge_conduit.task_comments(
                notification_info.patch_task,
                {"authorPHIDs": [self.security_patch_bot_phid]},
            )
            for bot_comment in bot_comments:
                # ["comments"][0] holds the latest revision of a comment
                text = bot_comment["comments"][0]["content"]["raw"]
                if (
                    notification_info.patch_name in text
                    and notification_info.module in text
                ):
                    return True
            return False

        return (
            task_is_open()
            and priority_is_unbreak()
            and is_subtask_of_release_task()
            and bot_comment_for_patch_is_present()
        )

    @staticmethod
    def _compose_notification(notification_info):
        patch_name = notification_info.patch_name
        module = notification_info.module
        target_release_task = notification_info.target_release_for_fixes["task_id"]
        target_release_version = notification_info.target_release_for_fixes["version"]

        return (
            f"Patch `{patch_name}` is currently failing to apply for the most recent code in the mainline branch"
            f" of `{module}`. This is blocking MediaWiki release `{target_release_version}`({target_release_task})\n"
            "\n"
            "//If the patch needs to be rebased//\n"
            "\n"
            "To unblock the release, a new version of the patch can be placed at the right location in the"
            " deployment server with the following Scap command:\n"
            "```\n"
            "REVISED_PATCH=<path_to_revised_patch>\n"
            "scap update-patch --message-body 'Rebase to solve merge conflicts with mainline code'"
            f""" /srv/patches/{target_release_version}/{module}/{patch_name} "$REVISED_PATCH" """
            "```\n"
            "---\n"
            "//If the patch has been made public//\n"
            "\n"
            "To unblock the release, the patch can be removed for the right version from the deployment server"
            " with the following Scap command:\n"
            "```\n"
            "scap remove-patch --message-body 'Remove patch already made public'"
            f" /srv/patches/{target_release_version}/{module}/{patch_name}"
            "```\n"
            "(Note that if patches for the version don't exist yet, they will be created and the patch you specified"
            " removed)"
        )


class PatchUserManipulation(cli.Application):
    def __init__(self, exe_name):
        super().__init__(exe_name)

        self.patch_name = None
        # Core, or particular extension/skin
        self.module = None
        self.target_version = None
        self.target_version_dir = None

    @cli.argument(
        "--message-body",
        default=None,
        help="Optional body to add to the commit message",
    )
    @cli.argument(
        "patch_path",
        help="Full path to the security patch",
    )
    def main(self, *extra_args):
        try:
            self._post_init()
            if not git_is_clean(self.config["patch_path"]):
                utils.abort(f"git is not clean: {self.config['patch_path']}")

            git.set_env_vars_for_user()
            if not os.path.isdir(self.target_version_dir):
                self.get_logger().info(
                    f"Directory for version {self.target_version} does not exist yet and will be created"
                )
                self._prime_target_version_dir()

            self._operation_on_patch()
            self._git_add_patch_changes()
            if git_is_clean(self.config["patch_path"]):
                self.get_logger().info("Patch has not changed. Nothing else to do")
                return

            commit_message = [
                "-m",
                f"Scap {self._operation_name()}-patch: {self.target_version}/{self.module}/{self.patch_name}",
            ]
            if self.arguments.message_body:
                commit_message += ["-m", self.arguments.message_body]
            gitcmd("commit", *commit_message, cwd=self.config["patch_path"])

            self.get_logger().info(
                f"Patch {self.arguments.patch_path} {self._operation_name()}d"
            )
        except Exception as e:
            utils.abort(f"Failed to {self._operation_name()} patch: {e}")

    def _post_init(self):
        """
        Initialization that requires self.config and self.arguments
        """
        patch_info_re = (
            rf"{self.config['patch_path']}/(?P<version>[^/]+)/(?P<module>.+)"
            r"/(?P<patch_name>\d+-T\d+\.patch)$"
        )
        patch_info = re.match(patch_info_re, self.arguments.patch_path)
        if not patch_info:
            raise Exception(
                f'"{self.arguments.patch_path}" does not look like a security patch path or the location is not right'
            )

        self.patch_name = patch_info.group("patch_name")
        self.module = patch_info.group("module")
        self.target_version = patch_info.group("version")
        self.target_version_dir = os.path.join(
            self.config["patch_path"], self.target_version
        )

    def _prime_target_version_dir(self):
        latest_patches = utils.select_latest_patches(self.config["patch_path"])
        if not latest_patches:
            self.get_logger().warning(
                f"Could not find any previous security patches. Is the {self.config['patch_path']} dir in a"
                " healthy state?"
            )
            return

        self.get_logger().info(
            f"Copying patches from {latest_patches} to {self.target_version_dir}"
        )
        shutil.copytree(latest_patches, self.target_version_dir)

        self._git_add_patch_changes()
        gitcmd(
            "commit",
            "-m",
            f"Scap {self._operation_name()}-patch: initial patches for {self.target_version}",
            cwd=self.config["patch_path"],
        )

    def _git_add_patch_changes(self):
        gitcmd("add", "--all", cwd=self.config["patch_path"])

    @abstractmethod
    def _operation_on_patch(self):
        pass

    @abstractmethod
    def _operation_name(self):
        pass


@cli.command("update-patch", primary_deploy_server_only=True)
class UpdatePatch(PatchUserManipulation):
    """
    Update/create a security patch and commit the changes

    The MediaWiki version directory is not required to exist yet, in that case a
    new directory for the version will be created and populated with the patches
    from the most recent available version. The specified patch will then be
    updated

    Example usage:
    scap update-patch --message-body "Rebase to solve merge conflicts with mainline code" \\
        /srv/patches/1.42.0-wmf.20/core/01-T123456.patch /home/jdoe/01-T123456-revised.patch
    """

    @cli.argument(
        "--message-body",
        default=None,
        help="Optional body to add to the commit message",
    )
    @cli.argument(
        "patch_path",
        help="Full path to the security patch to update",
    )
    @cli.argument(
        "revised_patch_path",
        type=argparse.FileType("r"),
        help="Full path to the revised security patch",
    )
    def main(self, *extra_args):
        return super().main(*extra_args)

    def _operation_on_patch(self):
        os.makedirs(os.path.dirname(self.arguments.patch_path), exist_ok=True)
        shutil.copyfile(
            self.arguments.revised_patch_path.name, self.arguments.patch_path
        )

    def _operation_name(self):
        return "update"


@cli.command("remove-patch", primary_deploy_server_only=True)
class RemovePatch(PatchUserManipulation):
    """
    Remove a security patch and commit the change

    The MediaWiki version directory is not required to exist yet, in that case a
    new directory for the version will be created and populated with the patches
    from the most recent available version. The specified patch will then be
    removed

    Example usage:
    scap remove-patch --message-body "Remove patch already made public" \\
        /srv/patches/1.42.0-wmf.20/extensions/GrowthExperiments/01-T123456.patch
    """

    @cli.argument(
        "--message-body",
        default=None,
        help="Optional body to add to the commit message",
    )
    @cli.argument(
        "patch_path",
        help="Full path to the security patch to remove",
    )
    def main(self, *extra_args):
        return super().main(*extra_args)

    def _operation_on_patch(self):
        os.unlink(self.arguments.patch_path)
        # Remove any empty dirs that may have been left behind
        for d, _, _ in os.walk(self.target_version_dir, topdown=False):
            try:
                os.rmdir(d)
            except OSError:
                # Dir wasn't empty, ignore
                pass

    def _operation_name(self):
        return "remove"


@cli.command("update-next-patches", primary_deploy_server_only=True)
class UpdateNextPatches(cli.Application):
    next_patches_dir: str = None

    @cli.argument(
        "--dry-run",
        "-n",
        action="store_true",
        help="Say what would happen without performing the actions",
        required=False,
    )
    def main(self, *extra_args):
        logger = self.get_logger()
        patches_dir = self.config["patch_path"]

        if not git_is_clean(patches_dir):
            utils.abort(f"git is not clean: {patches_dir}")

        git.set_env_vars_for_user()

        self.next_patches_dir = os.path.join(patches_dir, "next")
        source_patches_dir = self._select_source_patches_dir()

        dry_run_prefix = "DRY-RUN: " if self.arguments.dry_run else ""
        logger.info(
            f"{dry_run_prefix}Updating patches in {self.next_patches_dir} from {source_patches_dir}"
        )
        source_patches = SecurityPatches(source_patches_dir)
        adds = []
        removes = []

        # Add phase
        for patch in source_patches:
            source_path = patch.path()
            source_rel = os.path.relpath(source_path, source_patches_dir)
            target_path = os.path.join(self.next_patches_dir, source_rel)

            dropped = target_path + ".dropped"
            updated = target_path + ".updated"
            if os.path.exists(dropped):
                logger.info(f"Skipping {source_rel} since {dropped} exists")
                continue
            if os.path.exists(updated):
                logger.info(f"Skipping {source_rel} since {updated} exists")
                continue

            if os.path.exists(target_path) and filecmp.cmp(source_path, target_path):
                logger.info(
                    f"Skipping {source_rel} since it is identical to {target_path}"
                )
                continue

            if self.arguments.dry_run:
                logger.info(f"Would copy {source_rel} to {target_path}")
            else:
                logger.info(f"Copying {source_rel} to {target_path}")
                os.makedirs(os.path.dirname(target_path), exist_ok=True)
                shutil.copy2(source_path, target_path)
                adds.append(target_path)

        # Removal phase
        target_patches = SecurityPatches(self.next_patches_dir)
        for patch in target_patches:
            path = patch.path()
            if path.endswith(".dropped") or path.endswith(".updated"):
                continue
            path_rel = os.path.relpath(path, self.next_patches_dir)
            if source_patches.find(path_rel):
                continue
            source_path = os.path.join(source_patches_dir, path_rel)
            if self.arguments.dry_run:
                logger.info(f"Would remove {path} since {source_path} no longer exists")
            else:
                logger.info(f"Removing {path} since {source_path} no longer exists")
                removes.append(path)

        if adds:
            gitcmd("add", *adds, cwd=self.next_patches_dir)
        if removes:
            gitcmd("rm", *removes, cwd=self.next_patches_dir)

        if git_is_clean(patches_dir):
            return

        commit_message = "Patches updated by scap update-next-patches\n\n"
        if adds:
            patches = utils.pluralize("patch", len(adds))
            commit_message += f"{len(adds)} {patches} copied from {source_patches_dir} to {self.next_patches_dir}\n"
        if removes:
            patches = utils.pluralize("patch", len(removes))
            commit_message += (
                f"{len(removes)} {patches} removed from {self.next_patches_dir}\n"
            )

        gitcmd(
            "commit",
            "-m",
            commit_message,
            cwd=self.next_patches_dir,
        )
        logger.info("Committed the changes")

    def _select_source_patches_dir(self):
        source_patches_dir = None
        for dir in utils.get_patch_dirs(self.config["patch_path"]):
            if dir == self.next_patches_dir:
                break
            source_patches_dir = dir
        if not source_patches_dir:
            raise SystemExit(f"There are no patches prior to {self.next_patches_dir}")

        return source_patches_dir


class SecurityPatches:
    """A list of security patches."""

    def __init__(self, root):
        """
        `root` will be something like /srv/patches/1.45.0-wmf.7
        """
        self.root = root
        self._patches = self._populate(root)

    def _populate(self, root):
        core = os.path.abspath(os.path.join(root, "core"))
        exts = os.path.abspath(os.path.join(root, "extensions"))
        skins = os.path.abspath(os.path.join(root, "skins"))
        vendor = os.path.abspath(os.path.join(root, "vendor"))

        patches = []

        simples = [(core, "."), (vendor, "vendor")]
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

    def __iter__(self) -> Iterator["Patch"]:
        for patch in self._patches:
            yield patch

    def __len__(self):
        return len(self._patches)

    def find(self, relpath) -> Optional["Patch"]:
        for patch in self._patches:
            patch_rel = os.path.relpath(patch.path(), self.root)
            if patch_rel == relpath:
                return patch
        return None

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

    @staticmethod
    def fix_mtimes(pre_patch_state):
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

    def path(self) -> str:
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

    def apply(self, srcroot, abort_git_am_on_fail, output_line) -> int:
        srcdir = os.path.join(srcroot, self._relative)
        output_line("Applying patch %s in %s" % (self.path(), srcroot))

        try:
            if not git_is_clean(srcdir):
                output_line("ERROR: git is not clean: %s" % srcdir)
                return GIT_NOT_CLEAN
        except FailedCommand as e:
            output_line("ERROR: while checking if git is clean: %s" % e.stderr)
            return ERROR

        try:
            output = gitcmd("am", "--3way", self.path(), cwd=srcdir)
        except FailedCommand as e:
            output_line("ERROR: git am: %s" % e.stderr)
            if abort_git_am_on_fail:
                try:
                    gitcmd("am", "--abort", cwd=srcdir)
                except Exception:
                    pass
            return FAILED

        if "already applied" in output:
            return ALREADY_APPLIED
        return APPLIED


def git_is_clean(dirname):
    output = gitcmd("status", "--ignore-submodules", cwd=dirname)
    return "working tree clean" in output
