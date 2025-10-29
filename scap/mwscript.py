# -*- coding: utf-8 -*-
"""
Module providing containerized execution of MediaWiki scripts.
"""

import re
import shlex
import subprocess
import sys
import tempfile

from scap import cli
from scap.utils import log_context


PHP_WARNING = r"^(?:PHP )?(Notice|Warning)(.*)$"


@log_context("mwscript.run")
def run(
    app,
    script,
    *args,
    wiki="aawiki",
    version=None,
    network=False,
    check_warnings=False,
    logger=None,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    check=True,
    **kwargs,
) -> subprocess.CompletedProcess:
    """
    Run the given MediaWiki script via `scap mwscript`, using sudo if
    necessary.
    """
    flags = _run_flags(app)
    mwscript_flags = []

    if wiki is not None:
        mwscript_flags += [f"--wiki={wiki}"]

    if version is not None:
        mwscript_flags += ["--force-version", version]

    if network:
        flags += ["--network"]

    args = [str(arg) for arg in args]

    try:
        proc = app.scap_call(
            ["mwscript", *flags, "--", script, *mwscript_flags, *args],
            user=app.config["docker_user"],
            passthrough_arguments=False,
            stdout=stdout,
            stderr=stderr,
            check=check,
            **kwargs,
        )

        if check_warnings:
            warnings = ""

            for match in re.finditer(PHP_WARNING, proc.stderr, re.MULTILINE):
                warnings += match.group(0) + "\n"

            if warnings:
                raise SystemExit(
                    "{} generated PHP notices/warnings:\n{}".format(
                        script,
                        warnings,
                    )
                )

        return proc
    except subprocess.CalledProcessError as err:
        if err.stdout:
            logger.error(err.stdout)
        if err.stderr:
            logger.error(err.stderr)
        raise err


@log_context("mwscript.run_shell")
def run_shell(
    app,
    command,
    *args,
    logger=None,
    check=True,
    **kwargs,
) -> subprocess.CompletedProcess:
    """
    Run the given shell command via `scap mwshell` as the privileged docker
    user.
    """
    flags = _run_flags(app)
    quoted_args = [shlex.quote(arg) for arg in args]
    try:
        return app.scap_call(
            ["mwshell", *flags, "--", command.format(*quoted_args)],
            user=app.config["docker_user"],
            passthrough_arguments=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=check,
            **kwargs,
        )
    except subprocess.CalledProcessError as err:
        if err.stdout:
            logger.error(err.stdout)
        if err.stderr:
            logger.error(err.stderr)
        raise err


def _run_flags(app) -> list:
    return [
        "--no-local-config",
        "--directory",
        app.config["stage_dir"],
        "--user",
        app.config["mediawiki_runtime_user"],
    ]


DIRECTORY_HELP = "MediaWiki directory to mount within the container.  Default is stage_dir from scap.cfg."
USER_HELP = "MediaWiki runtime user.  Default is mediawiki_runtime_user from scap.cfg."


class MediaWikiRuntimeApp(cli.Application):
    def main(self, *extra_args):
        """
        Execute containerized PHP scripts.
        """
        if not self.arguments.directory:
            self.arguments.directory = self.config["stage_dir"]
        if not self.arguments.user:
            self.arguments.user = self.config["mediawiki_runtime_user"]

        self.runtime = Runtime(
            self.config,
            self.arguments.directory,
            self.arguments.user,
            offline=not self.arguments.network,
        )


@cli.command(
    "php",
    help="Execute PHP scripts within a Docker container",
)
class PHPApp(MediaWikiRuntimeApp):
    @cli.argument(
        "--directory",
        type=str,
        help=DIRECTORY_HELP,
    )
    @cli.argument(
        "--user",
        type=str,
        help=USER_HELP,
    )
    @cli.argument(
        "--network",
        default=False,
        action="store_true",
        help="Enable network access for the script.",
    )
    @cli.argument(
        "script",
        help="PHP script to run.",
    )
    @cli.argument(
        "arguments",
        nargs="...",
        help="script arguments",
    )
    def main(self, *extra_args):
        """
        Execute containerized PHP scripts.
        """
        super().main(*extra_args)
        proc = self.runtime.run_php(
            self.arguments.script,
            self.arguments.arguments,
            network=self.arguments.network,
        )
        sys.exit(proc.returncode)


@cli.command(
    "mwscript",
    help="Execute MediaWiki scripts within a Docker container",
)
class Mwscript(MediaWikiRuntimeApp):
    @cli.argument(
        "--directory",
        type=str,
        help=DIRECTORY_HELP,
    )
    @cli.argument(
        "--user",
        type=str,
        help=USER_HELP,
    )
    @cli.argument(
        "--network",
        default=False,
        action="store_true",
        help="Enable network access for the script.",
    )
    @cli.argument(
        "script",
        help="MediaWiki maintenance script to run.",
    )
    @cli.argument(
        "arguments",
        nargs="...",
        help="MWScript.php arguments",
    )
    def main(self, *extra_args):
        """
        Execute containerized MediaWiki PHP scripts via MWScript.php.
        """
        super().main(*extra_args)
        proc = self.runtime.run_mwscript(
            self.arguments.script,
            self.arguments.arguments,
            network=self.arguments.network,
        )
        sys.exit(proc.returncode)


@cli.command(
    "mwshell",
    help="Execute a shell command within a MediaWiki Docker container",
)
class Mwshell(cli.Application):
    @cli.argument(
        "--directory",
        type=str,
        required=True,
        help="MediaWiki directory to mount within the container.",
    )
    @cli.argument(
        "--user",
        type=str,
        required=True,
        help="MediaWiki runtime user.",
    )
    @cli.argument(
        "shell_command",
        help="Shell command.",
    )
    def main(self, *extra_args):
        """
        Execute a shell command within a MediaWiki Docker container.
        """
        runtime = Runtime(
            self.config,
            self.arguments.directory,
            self.arguments.user,
        )
        sys.exit(runtime.run_shell(self.arguments.shell_command).returncode)


class Runtime:
    """
    Provides containerized execution of MediaWiki scripts.
    """

    def __init__(self, cfg: dict, directory, user, temp_dir=None, offline=True):
        """
        Initializes a MediaWiki runtime.

        :param cfg: Scap config dict.
        :param directory: MediaWiki multiversion/config directory.
        :param user: MediaWiki runtime user.
        :param temp_dir: Temporary directory to mount.
        """
        self.datacenter = cfg["datacenter"]
        self.realm = cfg["wmf_realm"]
        self.image = cfg["mediawiki_runtime_image"]
        self.dir = directory
        self.user = user
        self.temp_dir = temp_dir
        self.offline = offline

        if self.temp_dir is None:
            self.temp_dir = tempfile.gettempdir()

    def run_php(
        self,
        script,
        args=[],
        **kwargs,
    ) -> subprocess.CompletedProcess:
        """
        Execute the given MediaWiki PHP script.
        """
        php_args = ["-d", "display_errors=stderr", "-d", "log_errors=off"]

        env = {"WMF_DATACENTER": "labs" if self.realm == "labs" else self.datacenter}

        if self.offline:
            env["WMF_MAINTENANCE_OFFLINE"] = "1"

        env["MESH_CHECK_SKIP"] = "1"

        return self._run(
            "/usr/bin/php",
            [*php_args, script, *args],
            env=env,
            **kwargs,
        )

    def run_mwscript(
        self,
        script,
        args=[],
        **kwargs,
    ) -> subprocess.CompletedProcess:
        """
        Execute the given MediaWiki PHP script via multiversion's
        MWScript.php.
        """
        return self.run_php("multiversion/MWScript.php", [script, *args], **kwargs)

    def run_shell(self, command: str, **kwargs) -> subprocess.CompletedProcess:
        """
        Execute the given bash shell command within a MediaWiki container and
        return the completed process.
        """
        return self._run("/bin/bash", ["-c", command], **kwargs)

    def _run(
        self,
        entrypoint,
        args: list,
        env=None,
        network=False,
    ) -> subprocess.CompletedProcess:
        """
        Execute the given command within a MediaWiki container and return the
        completed process.
        """
        # fmt: off
        cmd = [
            "docker", "run",
            "--rm",
            "--attach", "stdout",
            "--attach", "stderr",
            "--user", self.user,
            "--mount", f"type=bind,source={self.dir},target={self.dir}",
            "--mount", f"type=bind,source={self.temp_dir},target={self.temp_dir}",
            "--workdir", self.dir,
            "--entrypoint", entrypoint,
            "--network", "host" if network else "none"
        ]

        if network:
            # Ensure that environment-specific SSL certs are available (e.g., for
            # when wmf-config accesses Etcd)
            cmd += [
                "--mount", "type=bind,source=/etc/ssl/certs,target=/etc/ssl/certs,readonly",
                "--mount", "type=bind,source=/usr/local/share/ca-certificates,target=/usr/local/share/ca-certificates,readonly",
            ]

        if env is not None:
            cmd += [arg for k, v in env.items() for arg in ("--env", f"{k}={v}")]

        cmd += [self.image] + [str(arg) for arg in args]

        return subprocess.run(cmd, check=False)
