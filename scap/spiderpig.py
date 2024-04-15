import base64
import dataclasses
import json
import math
import os
import pickle
import signal
from eventlet.green import subprocess
from eventlet import queue
import time

from flask import Flask, render_template, request, url_for
import flask_socketio
from flask_socketio import SocketIO, emit

from scap import cli, utils, kubernetes, interaction


@dataclasses.dataclass
class Prompt:
    job_id: int = None
    message: str = None
    # The key is the choice text and the value is the response data for the choice.
    # For example: {"Retry the operation": "r", "Cancel": "c"}
    # or           {"Yes": "y", "No": "n"}
    choices: dict = None


@cli.command("spiderpig", help="Start Spiderpig server")
class SpiderPig(cli.Application):
    def main(self, *extra_args):
        socketio = SocketIO(app, logger=False, engineio_logger=False)
        k8s_ops = kubernetes.K8sOps(self)
        ns = SpiderPigSocketIONamespace(
            socketio, os.path.join(self.config["stage_dir"], "scap/spiderpig"), k8s_ops
        )
        socketio.on_namespace(ns)
        app.config["SPIDERPIGNS"] = ns
        # FIXME: This is required due to the use of url_for() in SpiderPigSocketIONamespace.set_scap_job_id().  I should
        # change that.
        app.config["SERVER_NAME"] = "localhost:5555"
        socketio.run(app, host="0.0.0.0", debug=True)


class SpiderPigSocketIONamespace(flask_socketio.Namespace):
    socketio = None
    k8s_ops = None
    logs_dir = None
    sequence_file = None
    scap_thread = None
    scap_subprocess = None
    scap_status = "Idle"
    scap_status_url = None
    scap_job_log_url = None
    scap_job_id = None
    history = None
    prompts = None

    def __init__(self, socketio, state_dir, k8s_ops):
        self.socketio = socketio
        self.k8s_ops = k8s_ops
        self.logs_dir = os.path.join(state_dir, "logs")
        self.sequence_file = os.path.join(state_dir, "seq")

        os.makedirs(self.logs_dir, exist_ok=True)

        self.history = History(os.path.join(state_dir, "history"))

        self.prompts = {}

        super().__init__("/")

    def get_sequence_number(self):
        seq = 0
        if os.path.exists(self.sequence_file):
            with open(self.sequence_file) as f:
                seq = int(f.readline())
        seq += 1

        with utils.temp_to_permanent_file(self.sequence_file) as f:
            print(seq, file=f)

        return seq

    # Socketio event handlers
    def on_connect(self):
        self.emit_scap_status()
        self.emit_prompts()

    def on_get_scap_status(self, data):
        return self.scap_status

    def on_stop_scap(self, data):
        job_id = data.get("job_id")
        if self.scap_subprocess and job_id == self.scap_job_id:
            pgid = os.getpgid(self.scap_subprocess.pid)
            os.killpg(pgid, signal.SIGTERM)

    def on_start_sync_world(self, data):
        if self.scap_thread is None:
            self.scap_thread = self.socketio.start_background_task(
                self.run_scap,
                "sync-world",
                "-Dbuild_mw_container_image:False",
                "-Ddeploy_mw_container_image:False",
                "-w5",
                data.get("justification"),
            )
        else:
            print("Ignoring a start_sync_world message since scap is already running")

    def on_start_backport(self, data):
        change_numbers = data.get("change_numbers")
        if not change_numbers:
            return

        if self.scap_thread is None:
            self.scap_thread = self.socketio.start_background_task(
                self.run_scap,
                "backport",
                "-Dbuild_mw_container_image:False",
                "-Ddeploy_mw_container_image:False",
                change_numbers,
            )

        else:
            print("Ignoring a start_backport message since scap is already running")

    def on_get_job_log(self, data):
        job_id = data["job_id"]
        start = int(data.get("start", 0))
        size = int(data.get("size", -1))

        logfile = self.logfile_path(job_id)

        if not os.path.exists(logfile):
            emit("add-output", f"No logfile for job {job_id} found\r\n")
            return

        emitted_anything = False

        with open(logfile, "rb") as f:
            filelen = os.fstat(f.fileno()).st_size

            f.seek(start)

            remaining = size if size >= 0 else math.inf

            while remaining > 0:
                got = f.read(min(remaining, 64 * 1024))
                if len(got) == 0:
                    # EOF
                    break
                remaining -= len(got)
                emit(
                    "add-output",
                    {
                        "data": got,
                        "filepos": f.tell(),
                        "filelen": filelen,
                    },
                )
                emitted_anything = True

            if not emitted_anything:
                # Make it clear to callers when they've asked to read at EOF
                emit(
                    "add-output",
                    {
                        "data": b"",
                        "filepos": f.tell(),
                        "filelen": filelen,
                    },
                )

    def on_prompt_response(self, data):
        job_id = data["job_id"]
        response = data["response"]

        prompt = self.prompts.get(job_id)
        q = self.scap_io_queue
        if not prompt or not q:
            return

        q.put({"type": "prompt-response", "response": response})

        self.emit_clear_prompt(prompt)

    # End Socketio event handlers

    def emit_scap_status(self):
        self.socketio.emit(
            "update-scap-status",
            {
                "status": self.scap_status,
                "job_id": self.scap_job_id,
                "status_url": self.scap_status_url,
                "job_log_url": self.scap_job_log_url,
            },
        )

    def emit_clear_prompt(self, prompt: Prompt):
        self.socketio.emit(
            "clear-prompt",
            {
                "job_id": prompt.job_id,
            },
        )

    def emit_prompt(self, prompt: Prompt):
        self.socketio.emit(
            "prompt",
            {
                "job_id": prompt.job_id,
                "prompt_mesage": prompt.message,
                "choices": prompt.choices,
            },
        )

    def emit_prompts(self):
        for prompt in self.prompts.values():
            self.emit_prompt(prompt)

    def emit_update_history(self):
        self.socketio.emit("update-history")

    def set_scap_status(self, status):
        self.scap_status = status
        self.emit_scap_status()

    def set_scap_job_id(self, job_id):
        self.scap_job_id = job_id
        if job_id:
            with app.app_context():
                self.scap_status_url = url_for("job_status", job_id=job_id)
                self.scap_job_log_url = url_for("job_log", job_id=job_id)
        else:
            self.scap_status_url = None
            self.scap_job_log_url = None

    def logfile_path(self, job_id):
        return os.path.join(self.logs_dir, f"{job_id}.log")

    def planfile_path(self, job_id):
        return os.path.join(self.logs_dir, f"{job_id}.plan")

    def register_prompt(self, job_id, message, choices):
        prompt = Prompt(job_id, message, choices)
        self.prompts[job_id] = prompt
        return prompt

    def unregister_prompt(self, job_id):
        del self.prompts[job_id]

    def run_scap(self, *args, **kwargs):
        try:
            self.run_scap1(*args, **kwargs)
        finally:
            self.scap_thread = None
            self.set_scap_job_id(None)
            self.set_scap_status("Idle")

    def run_scap1(self, *args, **kwargs):
        cmd = ["scap"] + list(args)
        cmd_desc = " ".join(cmd)

        job_id = self.get_sequence_number()
        logfile = self.logfile_path(job_id)
        planfile = self.planfile_path(job_id)

        self.set_scap_job_id(job_id)
        self.set_scap_status(f"Running {cmd_desc}")

        histentry = self.history.add(job_id, what=cmd_desc)

        self.emit_update_history()

        env = os.environ.copy()
        env["SPIDERPIG_JOB_ID"] = str(job_id)
        env["FORCE_COLOR"] = "1"

        plan = None

        p = self.scap_subprocess = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=env,
            preexec_fn=os.setsid,
        )

        q = self.scap_io_queue = queue.Queue()

        def scap_reader(p, q):
            while True:
                line = p.stdout.readline()
                q.put({"type": "scap-output", "line": line})
                if line == b"":
                    break

        self.socketio.start_background_task(scap_reader, p, q)

        with open(logfile, "wb") as logstream:

            def output(line):
                if isinstance(line, str):
                    line = line.encode()
                assert logstream.write(line) == len(line)
                logstream.flush()

            while True:
                msg = q.get()
                msg_type = msg["type"]

                if msg_type == "prompt-response":
                    self.unregister_prompt(job_id)
                    resp = msg["response"] + "\n"
                    output(resp)
                    try:
                        p.stdin.write(resp.encode())
                    except BrokenPipeError:
                        pass
                    continue

                if msg["type"] != "scap-output":
                    raise Exception(f"Unexpected message from queue: {msg}")

                line = msg["line"]
                if line == b"":
                    break

                if line[0] == ord("{"):
                    # FIXME: Make this robust against invalid JSON, or rogue JSON supplied by a user in a commit message, operation description, etc.
                    rec = json.loads(line)
                    if rec.get("name") == "spiderpig-message":
                        type = rec["type"]
                        if type == "plan":
                            plan = pickle.loads(base64.b64decode(rec["plan"]))
                            self.write_plan(plan, planfile)
                            self.emit_update_history()
                        elif type == "running-status-change":
                            self.update_plan(
                                plan,
                                rec["path_id"],
                                rec["running-status"],
                                logstream,
                                planfile,
                            )
                            self.emit_update_history()
                        elif type == "prompt":
                            prompt_message = rec["prompt_message"]
                            choices = rec["choices"]
                            output(
                                interaction.TerminalInteraction.generate_prompt_text(
                                    prompt_message, choices
                                )
                            )
                            prompt = self.register_prompt(
                                job_id, prompt_message, choices
                            )
                            self.emit_prompt(prompt)
                        else:
                            print(f"Invalid spiderpig-message received: {rec}")
                    else:
                        output(line)
                else:
                    output(line)

            p.wait()
            if p.returncode:
                if p.returncode > 0:
                    output(f"Process exited with status {p.returncode}")
                else:
                    output(f"Process terminated by signal {-p.returncode}")
            else:
                output("Process terminated normally")
        self.history.finalize(histentry, p.returncode)
        self.scap_subprocess = None
        self.scap_io_queue = None
        self.emit_update_history()

    def write_plan(self, plan, planfile):
        with utils.temp_to_permanent_file(planfile, "wb") as f:
            pickle.dump(plan, f)

    def update_plan(self, plan, path_id, running_status, logstream, planfile):
        thing = plan.get_by_path_id(path_id)

        if running_status == "started":
            thing.logfile_start = logstream.tell()
            thing.mark_start()
        elif running_status == "finished":
            thing.logfile_stop = logstream.tell()
            thing.mark_stop()
        else:
            raise Exception(f"Unexpected running_status: {running_status}")

        self.write_plan(plan, planfile)


app = Flask(__name__)
app.config["SECRET_KEY"] = "secret!"


@app.route("/")
def home():
    return render_template("index.html")


@app.route("/scap/backport")
def backport():
    return render_template("backport.html")


@app.route("/scap/jobs/<int:job_id>/status")
def job_status(job_id):
    planfile = app.config["SPIDERPIGNS"].planfile_path(job_id)

    if not os.path.exists(planfile):
        # Fall back to showing the job log
        return job_log(job_id)

    with open(planfile, "rb") as f:
        plan = pickle.load(f)

    return render_template(
        "job_overview.html",
        job_id=job_id,
        plan=plan,
        job_log_url=url_for("job_log", job_id=job_id),
    )


@app.route("/scap/jobs/<int:job_id>/log")
def job_log(job_id):
    job = app.config["SPIDERPIGNS"].history.get(job_id)

    return render_template(
        "job_log.html", job_id=job_id, job=job, position=request.args.get("position")
    )


@app.route("/scap/history")
def job_history():
    return render_template(
        "job_history.html", history=app.config["SPIDERPIGNS"].history
    )


@app.route("/kubernetes/deployments")
def deployments_status():
    return render_template(
        "k8s_deployments.html",
        data=app.config["SPIDERPIGNS"].k8s_ops.get_deployments_info(),
    )


@app.route("/kubernetes/deployments/<stage>")
def stage_deployments_status(stage):
    # FIXME: Graceful handling of invalid stage name
    deployments = app.config["SPIDERPIGNS"].k8s_ops.get_deployments_for_stage(stage)
    return render_template(
        "k8s_stage_deployments.html", stage=stage, deployments=deployments
    )


@dataclasses.dataclass
class HistoryEntry:
    id: int
    who: str
    what: str
    why: str
    start_time: float = dataclasses.field(default_factory=time.time)
    stop_time: float = None
    status: str = ""
    exitcode: int = None

    @property
    def start_time_string(self):
        if self.start_time is None:
            return "Not started"
        return time.ctime(self.start_time)

    @property
    def stop_time_string(self):
        if self.stop_time is None:
            return "Not finished"
        return time.ctime(self.stop_time)

    @property
    def job_overview_url(self):
        with app.app_context():
            return url_for("job_status", job_id=self.id)

    @property
    def started(self) -> bool:
        return self.start_time is not None

    @property
    def stopped(self) -> bool:
        return self.stop_time is not None

    @property
    def running(self) -> bool:
        return self.started and not self.stopped

    @property
    def running_status(self) -> str:
        if not self.started:
            return "Not started"
        if self.running:
            return "Running"
        return "Finished"

    @property
    def duration(self) -> str:
        if not self.started:
            return "Not started"
        if self.running:
            return f"{time.time() - self.start_time:.2f} secs"
        return f"{self.stop_time - self.start_time:.2f} secs"


class History:
    entries = None
    filename = None

    def __init__(self, filename):
        # Key is id, value is an entry
        self.entries = {}
        self.filename = filename
        if os.path.exists(filename):
            self._load()

    def add(self, job_id, what=None, who=None, why=None):
        if what is None:
            raise Exception("'what' must be supplied")
        if who is None:
            who = os.environ.get("USER")
        if why is None:
            why = "No justification provided"

        if job_id in self.entries:
            raise Exception(f"job_id {job_id} already exists in history")

        entry = HistoryEntry(id=job_id, who=who, what=what, why=why)

        self.entries[job_id] = entry

        self._save()

        return entry

    def finalize(self, histentry, exitcode):
        histentry.stop_time = time.time()
        histentry.exitcode = exitcode
        self._save()

    def get(self, job_id):
        return self.entries.get(job_id)

    def get_entries(self):
        """
        Return entries from most recent to oldest.
        """
        return [self.entries[id] for id in sorted(self.entries.keys(), reverse=True)]

    def _save(self):
        with utils.temp_to_permanent_file(self.filename, "w") as f:
            f.write(repr(self.entries))

    def _load(self):
        with open(self.filename) as f:
            self.entries = eval(f.read())
        need_save = False
        for id, entry in self.entries.items():
            if entry.running:
                entry.stop_time = time.time()
                entry.exitcode = 1000
                need_save = True
        if need_save:
            self._save()
