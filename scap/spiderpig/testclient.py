from contextlib import contextmanager
import json
import os
import requests
import sys
import threading
import time
from typing import List, Tuple, Optional
import urllib.parse

import scap.cli as cli


class SpiderpigApiClient:
    def __init__(self, base_url: str, user: str):
        """
        'base_url' should be something like http://127.0.0.1:8000
        """
        self.base_url = base_url
        self.user = user
        self.session = requests.Session()
        self.session.headers["Authorization"] = f"Bearer {user}"

    def jobrunner_status(self) -> dict:
        url = urllib.parse.urljoin(self.base_url, "/api/jobrunner/status")
        r = self.session.get(url)
        r.raise_for_status()
        return r.json()

    def backport(self, change_urls: List[str]) -> int:
        """
        Returns the job id.
        """
        url = urllib.parse.urljoin(self.base_url, "/api/jobs/backport")
        r = self.session.post(url, params={"change_url": change_urls})
        # FIXME: Improve error reporting.. use the information in the error response
        r.raise_for_status()
        return r.json()["id"]

    def get(self, job_id: int) -> Tuple[dict, Optional[dict]]:
        url = urllib.parse.urljoin(self.base_url, f"/api/jobs/{job_id}")
        r = self.session.get(url)
        r.raise_for_status()
        r = r.json()
        return r["job"], r["pending_interaction"]

    def respond(self, interaction: dict, response: str):
        """
        Respond to an interaction that was returned as the second entry in the tuple
        returned by 'get'.
        """
        job_id = interaction["job_id"]
        iid = interaction["id"]
        url = urllib.parse.urljoin(self.base_url, f"/api/jobs/{job_id}/interact/{iid}")
        r = self.session.post(url, data=json.dumps(response))
        r.raise_for_status()

    def stream_log(self, job_id: int):
        """
        This is a generator that yields batches of bytes
        """
        url = urllib.parse.urljoin(self.base_url, f"/api/jobs/{job_id}/log")
        r = self.session.get(url, stream=True)
        r.raise_for_status()
        for chunk in r.iter_content(chunk_size=None):
            yield chunk

    def signal(self, job_id: int, type: str):
        url = urllib.parse.urljoin(self.base_url, f"/api/jobs/{job_id}/signal/{type}")
        r = self.session.post(url)
        r.raise_for_status()


@cli.command(
    "spiderpig-testclient",
    primary_deploy_server_only=True,
)
class TestClient(cli.Application):
    def main(self, *extra_args):
        self.base_url = "http://127.0.0.1:8000"  # FIXME
        self.user = os.getenv("USER")
        client = SpiderpigApiClient(self.base_url, self.user)

        while True:
            jobrunner_status = client.jobrunner_status()
            status = jobrunner_status["status"]

            # Wait for jobrunner to become idle
            if status != "idle":
                job_id = jobrunner_status["job_id"]
                print(f"Jobrunner status: {status}")
                if job_id:
                    print(f"Job {job_id} is running.  Attaching to it")
                    self.handle_job(client, job_id)
                else:
                    time.sleep(1)
                continue

            print("Jobrunner is idle.")
            changes = input("What changes do you want to backport?: ").split()
            if not changes:
                print("Terminating")
                return
            job_id = client.backport(changes)
            print(f"Added job {job_id}")
            self.handle_job(client, job_id)

    def watch_logfile(self, job_id: int, stop: threading.Event):
        # This is intended to be run in a separate thread, so create a fresh
        # session for it.
        client = SpiderpigApiClient(self.base_url, self.user)
        # FIXME: Not responsive to the stop event while waiting for the next chunk
        for chunk in client.stream_log(job_id):
            # Sadly sys.stdout.write(chunk) will complain because chunk
            # is a byte array, so we have to do it this way.
            sys.stdout.buffer.write(chunk)
            sys.stdout.buffer.flush()
            if stop.is_set():
                return

    @contextmanager
    def logfile_watcher(self, job_id: int):
        stop = threading.Event()
        tail_thread = threading.Thread(
            target=self.watch_logfile, args=(job_id, stop), daemon=True
        )
        tail_thread.start()

        try:
            yield
        finally:
            stop.set()
            tail_thread.join()

    def handle_job(self, client: SpiderpigApiClient, job_id: int):
        interrupted = False

        with self.logfile_watcher(job_id):
            while True:
                job, interaction = client.get(job_id)

                try:
                    finished_at = job["finished_at"]
                    if finished_at:
                        exit_status = job["exit_status"]
                        print(
                            f"Job finished at {finished_at} with status {exit_status}"
                        )
                        return exit_status

                    # Don't bother processing interactions if we've tried to
                    # interrupt the backport.... just wait for the job
                    # to terminate.
                    if interrupted or not interaction:
                        time.sleep(1)
                        continue

                    # Collect relevant information from the Interaction object
                    type = interaction["type"]
                    prompt = interaction["prompt"]
                    choices = interaction["choices"]
                    default = interaction["default"]

                    # Since this sample client is tailing the log, the prompt is
                    # already in there, so we don't need to display it here (which
                    # is why type, prompt. We "use" the prompt information
                    # here to satisfy the linter.
                    type
                    prompt
                    choices
                    default

                    # Note, while blocked on input here, we don't notice if the job terminates
                    resp = input()
                    client.respond(interaction, resp)
                except KeyboardInterrupt:
                    interrupted = True
                    print("Control-c received.  Interrupting job")
                    client.signal(job_id, "interrupt")
