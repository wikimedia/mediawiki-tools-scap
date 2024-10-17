import os
import shlex
import threading
import time

import scap.cli as cli

import scap.spiderpig
from scap.spiderpig.model import Job, Interaction

from sqlalchemy.orm import Session


@cli.command(
    "spiderpig-testclient",
    primary_deploy_server_only=True,
)
class TestClient(cli.Application):
    def main(self, *extra_args):
        user = os.getenv("USER")
        cmd = input("What command do you want to run?: ")

        db_filename = self.spiderpig_dbfile()
        logdir = self.spiderpig_joblogdir()

        with Session(scap.spiderpig.engine(db_filename)) as session:
            job_id = Job.add(
                session,
                user=user,
                command=shlex.split(cmd),
            )
            print(f"Added job {job_id}")

            logfile = os.path.join(logdir, f"{job_id}.log")

            stop = threading.Event()
            threading.Thread(
                target=tail_file, args=(logfile, stop), daemon=True
            ).start()

            interrupted = False

            try:
                while True:
                    job = Job.get(session, job_id)
                    assert job

                    try:
                        if job.finished_at:
                            print(
                                f"Job finished at {job.finished_at} with status {job.exit_status}"
                            )
                            return job.exit_status

                        i = None
                        if not interrupted:
                            # Locate a non-responded-to interaction for this job.
                            i = Interaction.lookup_pending(session, job_id)
                        if not i:
                            session.rollback()
                            time.sleep(1)
                            continue

                        # Collect relevant information from the Interaction object, then terminate
                        # the transaction before waiting for the user to respond.
                        type = i.type
                        prompt = i.prompt
                        choices = i.choices
                        default = i.default
                        session.rollback()

                        # Since this sample code is tailing the log, the prompt is
                        # already in there, so we don't need to display it here (which
                        # is why type, prompt. We "use" the prompt information
                        # here to satisfy the linter.
                        type
                        prompt
                        choices
                        default

                        # Note, while blocked on input here, we don't notice if the job terminates
                        resp = input()
                        i.respond(session, user, resp)
                    except KeyboardInterrupt:
                        interrupted = True
                        print("Control-c received.  Interrupting job")
                        job.interrupt(session, user)
            finally:
                stop.set()


def tail_file(filename, stop: threading.Event):
    while not os.path.exists(filename):
        time.sleep(1)

    with open(filename) as f:
        while not stop.is_set():
            line = f.readline()
            if line:
                print(line, end="", flush=True)
                continue
            # EOF
            time.sleep(0.5)
