#!/usr/bin/env python3

import argparse
import dataclasses
import json
import math

import scap.spiderpig.io


def simulate_large_k8s_deployment(o):
    numTasks = 100

    progress = scap.spiderpig.io.SpiderpigProgressReportRecord(
        name="K8s deployment progress",
        totalTasks=numTasks,
        tasksInFlight=None,
        tasksFinishedOk=0,
        tasksFinishedFailed=0,
        tasksFinishedTotal=0,
        tasksPercentComplete=0,
        tasksRemaining=numTasks,
        progressFinished=False,
    )

    for tasksFinished in range(0, numTasks + 1):
        progress.tasksFinishedOk = tasksFinished
        progress.tasksFinishedTotal = tasksFinished
        progress.tasksPercentComplete = math.floor(
            100.0 * (float(tasksFinished) / max(numTasks, 1))
        )
        progress.tasksRemaining = numTasks - tasksFinished
        rec = {
            "timestamp": 0,
            "payload": {"type": "progress", "progress": dataclasses.asdict(progress)},
            "gap": 0.5,
        }
        print(json.dumps(rec), file=o)
    progress.progressFinished = True
    print(json.dumps(rec), file=o)


def generate(source_filename):
    with open(source_filename) as f:
        lines = f.readlines()

        def lines_generator():
            while lines:
                yield lines.pop(0)

        def eat_until_blank_status(o):
            for line in lines_generator():
                rec = json.loads(line)
                payload = rec["payload"]
                if payload["type"] == "status" and payload["status"] is None:
                    o.write(line)
                    return
            raise Exception("Did not find the expected null status payload")

        def process(o, search=None) -> bool:
            """
            Returns True if a line with the search string was found (and written).
            """
            for line in lines_generator():
                rec = json.loads(line)
                payload = rec["payload"]
                if payload.get("progress", {}).get("name") == "K8s deployment progress":
                    simulate_large_k8s_deployment(o)
                    eat_until_blank_status(o)
                    continue

                o.write(line)
                if search and search in line:
                    return True

            return False

        for line in lines_generator():
            if "Voting on 1 change(s)" in line:
                break
        if not line:
            raise Exception("Did not find the expected 'Voting on 1 change(s)' line")

        with open("prep_and_testservers_sync.jsonl", "w") as o:
            o.write(line)
            if not process(o, "Finished check-testservers"):
                raise Exception(
                    "Did not find the expected 'Finished check-testservers' line"
                )

        with open("sync_remainder.jsonl", "w") as o:
            for line in lines_generator():
                if "Started sync-canaries-k8s" in line:
                    o.write(line)
                    break
            if not line:
                raise Exception(
                    "Did not find the expected 'Started sync-canaries-k8s' line"
                )
            process(o)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "input_file",
        help="A JSONlines file from mediawiki-staging/spiderpig/scap/jobs",
    )

    args = ap.parse_args()

    generate(args.input_file)


if __name__ == "__main__":
    main()
