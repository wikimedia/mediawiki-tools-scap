from contextlib import contextmanager
from dataclasses import dataclass, field
import datetime
import json
import logging
import os
import queue
import signal
import stat
import subprocess
import threading
import time
from typing import Optional

import scap.cli as cli
from scap.interaction import TerminalIO

import scap.spiderpig
from scap.spiderpig.model import (
    JobrunnerStatus,
    Job,
    Interaction,
    Interruption,
    setup_db,
)

from sqlalchemy import Engine
from sqlalchemy.orm import Session


@cli.command(
    "spiderpig-jobrunner",
    help="Run the SpiderPig Job Runner",
    primary_deploy_server_only=True,
)
class JobRunner(cli.Application):
    @cli.argument(
        "--polling-interval",
        default=2.0,
        type=float,
        help="How often to check for new jobs (in seconds). Default is 2.",
    )
    def main(self, *extra_args):
        with self.lock(
            self.spiderpig_jobrunner_lockfile(),
            reason="Spiderpig jobrunner running",
            timeout=0,
        ):
            logger = self.get_logger()
            logger.info(
                "SpiderPig Job Runner started.  Checking for new jobs every %s seconds",
                self.arguments.polling_interval,
            )

            self.started_at = time.time()

            spiderpig_dir = self.spiderpig_dir()
            os.makedirs(spiderpig_dir, exist_ok=True)
            os.chmod(spiderpig_dir, 0o770 | stat.S_ISGID)

            logdir = self.spiderpig_joblogdir()
            os.makedirs(logdir, exist_ok=True)
            os.chmod(logdir, 0o770 | stat.S_ISGID)

            db_filename = self.spiderpig_dbfile()
            engine = scap.spiderpig.engine(db_filename)
            setup_db(engine, db_filename)

            with Session(engine) as session:
                try:
                    while True:
                        self._set_status(session, "idle")

                        job = Job.pop(session)
                        if job is None:
                            time.sleep(self.arguments.polling_interval)
                            continue

                        exit_status = None
                        try:
                            exit_status = run_job(
                                job,
                                engine,
                                session,
                                logger,
                                logdir,
                                self._set_running_job_status,
                            )
                        finally:
                            job.finish(session, exit_status)
                finally:
                    self._set_status(session, "Terminated", clear_pid=True)

    def _set_status(
        self,
        session: Session,
        status: str,
        job_id: Optional[int] = None,
        clear_pid: bool = False,
    ):
        JobrunnerStatus.set(session, status, job_id, clear_pid)

    def _set_running_job_status(
        self, session: Session, job_id: int, sub_status: Optional[str] = None
    ):
        status = f"Running job {job_id}"
        if sub_status:
            status += f", {sub_status}"
        self._set_status(session, status, job_id)


class EndOfStdout:
    pass


@dataclass
class RunningJob:
    engine: Engine
    job_id: int
    proc: subprocess.Popen
    iokey: str
    job_queue: queue.Queue = field(default_factory=queue.Queue)
    stop: threading.Event = field(default_factory=threading.Event)

    def monitor_output(self):
        while not self.stop.is_set():
            line = self.proc.stdout.readline()
            if not line:
                self.job_queue.put(EndOfStdout())
                break
            msg = self.decode_message(line)
            # This will enqueue either a dictionary (representing
            # a message) or a string (a line of output)
            self.job_queue.put(msg or line)

    def decode_message(self, line) -> Optional[dict]:
        if line[0] != "{":
            return

        try:
            msg = json.loads(line)
        except Exception:
            return

        if msg.get("iokey") == self.iokey:
            return msg

    def monitor_interruptions(self):
        POLLING_INTERVAL = 1.0

        with Session(self.engine) as session:
            while not self.stop.is_set():
                i = Interruption.pop(session, self.job_id)
                if i is None:
                    session.rollback()
                    time.sleep(POLLING_INTERVAL)
                    continue
                # Found an interruption
                self.job_queue.put(i)
                return

    def monitor_interaction_responses(self):
        POLLING_INTERVAL = 1.0

        with Session(self.engine) as session:
            while not self.stop.is_set():
                resp = Interaction.pop_responded(session, self.job_id)
                if resp:
                    self.job_queue.put(resp)
                else:
                    time.sleep(POLLING_INTERVAL)
                    continue

    def start_monitors(self):
        for monitor in [
            self.monitor_output,
            self.monitor_interruptions,
            self.monitor_interaction_responses,
        ]:
            threading.Thread(target=monitor, daemon=True).start()

    def stop_monitors(self):
        self.stop.set()

    def get(self):
        return self.job_queue.get()


@contextmanager
def running_job(engine: Engine, job_id: int, proc: subprocess.Popen, iokey: str):
    rj = RunningJob(engine, job_id, proc, iokey)
    rj.start_monitors()
    try:
        yield rj
    finally:
        rj.stop_monitors()


def run_job(
    job: Job,
    engine: Engine,
    session: Session,
    logger: logging.Logger,
    logdir: str,
    set_jobrunner_status: callable,
) -> Optional[int]:
    """
    Returns the exit status of the subprocess (if any)
    """
    set_jobrunner_status(session, job.id)
    logger.info(
        "Running job %d created by %s at %s",
        job.id,
        job.user,
        datetime.datetime.fromtimestamp(job.queued_at, datetime.timezone.utc).ctime(),
    )

    i = Interruption.peek(session, job.id)
    if i:
        logger.warning(
            "Job %d was interrupted by %s before it started.", job.id, i.user
        )
        return

    command = json.loads(job.command)
    logger.info("Command: %s", command)

    jsonlogfile = os.path.join(logdir, f"{job.id}.log.jsonl")
    jsonloglasttimestamp = time.time()

    iokey = hex(int.from_bytes(os.urandom(32), "big"))  # 256 random bits

    env = os.environ.copy()
    env["SPIDERPIG_IO_KEY"] = iokey
    env["FORCE_COLOR"] = "1"

    with open(jsonlogfile, "w") as jsonlogstream:

        def jsonlog(payload: dict):
            nonlocal jsonloglasttimestamp

            now = time.time()
            gap = now - jsonloglasttimestamp
            jsonloglasttimestamp = now

            rec = {"timestamp": now, "payload": payload, "gap": gap}
            print(json.dumps(rec), file=jsonlogstream, flush=True)

        def log(line, level=logging.DEBUG, dojsonlog=True):
            """
            Add a line of subprocess output to the logfile.  'line' is usually
            terminated by a newline.
            """
            line_stripped = line.rstrip()

            logger.log(level, line_stripped)
            if dojsonlog:
                jsonlog({"type": "line", "line": line_stripped})

        try:
            p = subprocess.Popen(
                command,
                text=True,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                env=env,
                preexec_fn=os.setsid,
            )
        except Exception as e:
            msg = f"Failed to start {command}: {e}\n"
            log(msg, logging.ERROR)
            return

        with running_job(engine, job.id, p, iokey) as rj:
            job_status = None
            old_job_status = None

            while True:
                got = rj.get()

                if isinstance(got, EndOfStdout):
                    break

                if isinstance(got, Interruption):
                    i = got
                    msg = f"[Jobrunner: {i.user} {i.type}ed]\n"
                    log(msg, logging.WARNING)
                    jsonlog(
                        {
                            "type": "signal",
                            "user": i.user,
                            "signal": i.type,
                        }
                    )
                    signo = signal.SIGINT if i.type == "interrupt" else signal.SIGKILL
                    os.killpg(os.getpgid(p.pid), signo)
                    continue

                if isinstance(got, str):
                    log(got)
                    continue

                if isinstance(got, dict):
                    msg = got
                    jsonlog(msg)

                    type = msg.get("type")

                    if type == "status":
                        job_status = msg.get("status")
                        job.set_status(session, job_status)
                        continue

                    if type == "line":
                        line = msg.get("line") + "\n"
                        sensitive = msg.get("sensitive")
                        if not sensitive:
                            log(line, dojsonlog=False)
                        continue

                    if type != "interaction":
                        logger.warning(
                            "Unexpected message type '%s' received from subprocess.  Ignoring.",
                            type,
                        )
                        continue
                    # Set up an interaction
                    subtype = msg.get("subtype")
                    if subtype not in ["choices", "input_line"]:
                        logger.warning(
                            "Unexpected interaction type '%s' received from subprocess.  Ignoring.",
                            subtype,
                        )
                        continue
                    prompt = msg["prompt"]
                    choices = msg["choices"] if subtype == "choices" else None
                    default = msg.get("default")

                    # Show the terminal style prompt in the job log
                    if subtype == "input_line":
                        log(prompt)
                    else:
                        log(TerminalIO.generate_prompt_text(prompt, choices, default))

                    Interaction.register(
                        session, job.id, subtype, prompt, choices, default
                    )
                    set_jobrunner_status(session, job.id, "awaiting user interaction")
                    old_job_status = job_status
                    job.set_status(session, "Awaiting user interaction")
                    logger.info("Waiting for an interaction")
                    continue

                if isinstance(got, Interaction):
                    message = (
                        f"User '{got.responded_by}' responded with '{got.response}'"
                    )
                    logger.info(message)
                    jsonlog(
                        {
                            "type": "response",
                            "user": got.responded_by,
                            "response": got.response,
                        }
                    )
                    print(got.response, file=rj.proc.stdin, flush=True)
                    set_jobrunner_status(session, job.id)  # Clear jobrunner status
                    job.set_status(session, old_job_status)
                    continue

                raise Exception(f"Unexpected item retrieved from job_queue: {got}")

        p.stdin.close()
        exit_status = p.wait()
        jsonlog({"type": "exit", "status": exit_status})
        if exit_status == 0:
            final_status = f"Job {job.id} finished normally"
        elif exit_status < 0:
            final_status = f"Job {job.id} terminated by signal {-exit_status}"
        else:
            final_status = f"Job {job.id} finished with status {exit_status}"
        logger.info(final_status)
        job.set_status(session, final_status)

        return exit_status
