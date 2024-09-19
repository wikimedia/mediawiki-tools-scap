# -*- coding: utf-8 -*-
"""
Module providing containerized execution of MediaWiki scripts.
"""

import logging
import re
import shlex
import subprocess
import sys
import tempfile

from scap.utils import log_context


PHP_WARNING = r"^(?:PHP )?(Notice|Warning)(.*)$"


class Runtime:
    """
    Provides containerized execution of MediaWiki scripts.
    """

    def __init__(self, cfg: dict, temp_dir=None):
        """
        Initializes a MediaWiki runtime.

        :param cfg: Scap config dict.
        """
        self.datacenter = cfg["datacenter"]
        self.dir = cfg["stage_dir"]
        self.image = cfg["mediawiki_runtime_image"]
        self.user = cfg["mediawiki_runtime_user"]
        self.temp_dir = temp_dir

        if self.temp_dir is None:
            self.temp_dir = tempfile.gettempdir()

    def run_mwscript(
        self,
        script,
        argv=[],
        *args,
        wiki="aawiki",
        version=None,
        env=None,
        check_warnings=False,
        **kwargs,
    ) -> subprocess.CompletedProcess:
        """
        Execute the given MediaWiki PHP script via multiversion's
        MWScript.php.
        """
        php_args = ["-d", "display_errors=stderr", "-d", "log_errors=off"]

        mwscript_args = []

        if wiki is not None:
            mwscript_args += [f"--wiki={wiki}"]

        if version is not None:
            mwscript_args += ["--force-version", version]

        if env is None:
            env = {}

        env["WMF_DATACENTER"] = self.datacenter
        env["WMF_MAINTENANCE_OFFLINE"] = "1"

        php_args += ["multiversion/MWScript.php", script] + mwscript_args + argv

        proc = self._run(
            "/usr/bin/php",
            php_args,
            *args,
            env=env,
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

    def run_shell(self, command, *args, **kwargs) -> subprocess.CompletedProcess:
        """
        Execute the given bash shell command within a MediaWiki container and
        return the completed process.
        """
        args = [shlex.quote(arg) for arg in args]
        return self._run("/bin/bash", ["-c", command.format(*args)], **kwargs)

    @log_context("mwscript.run")
    def _run(
        self,
        entrypoint,
        argv: list,
        *args,
        env=None,
        check=True,
        user=None,
        network=False,
        logger=None,
        stdout=subprocess.PIPE,
        stdout_log_level=logging.DEBUG,
        stderr=subprocess.PIPE,
        **kwargs,
    ) -> subprocess.CompletedProcess:
        """
        Execute the given command within a MediaWiki container and return the
        completed process.
        """
        if user is None:
            user = self.user

        # fmt: off
        cmd = [
            "docker", "run",
            "--rm",
            "--attach", "stdin",
            "--attach", "stdout",
            "--attach", "stderr",
            "--user", user,
            "--mount", f"type=bind,source={self.dir},target={self.dir}",
            "--mount", f"type=bind,source={self.temp_dir},target={self.temp_dir}",
            "--workdir", self.dir,
            "--entrypoint", entrypoint,
            "--network", "host" if network else "none"
        ]

        if env is not None:
            cmd += [arg for k, v in env.items() for arg in ("--env", f"{k}={v}")]

        cmd += [self.image] + [str(arg) for arg in argv]

        try:
            if logger is not None:
                logger.debug("Running: %s", " ".join(map(shlex.quote, cmd)))

            proc = subprocess.Popen(
                cmd,
                *args,
                stdout=subprocess.PIPE,
                stderr=stderr,
                text=True,
                **kwargs,
            )

            # Note that stdout is always subprocess.PIPE above so it can be
            # processed here, sent to the logger and either captured or
            # forwarded to sys.stdout.
            stdout_text = ""

            for out in proc.stdout:
                if logger is not None:
                    logger.log(stdout_log_level, out.strip())

                if stdout is None:
                    sys.stdout.write(out)
                elif stdout == subprocess.PIPE:
                    stdout_text += out

            (_, stderr_text) = proc.communicate()

            completed = subprocess.CompletedProcess(
                args=proc.args,
                returncode=proc.returncode,
                stdout=stdout_text,
                stderr=stderr_text,
            )

            if check:
                completed.check_returncode()

            return completed

        except subprocess.CalledProcessError as err:
            logger.error(err.stderr)
            raise err
