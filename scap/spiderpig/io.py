import json
import os
import sys

import dataclasses
from typing import Optional

from scap.sigint import sigint_interruptible


def send_message(rec: dict):
    """
    Send a message to the jobrunner by generating a specially formatted
    line of output.
    """
    rec["iokey"] = os.environ["SPIDERPIG_IO_KEY"]
    print(json.dumps(rec))
    sys.stdout.flush()


def output_line(line: str, sensitive=False):
    rec = {
        "type": "line",
        "line": line,
        "sensitive": sensitive,
    }
    send_message(rec)


def input_line(prompt) -> str:
    rec = {
        "type": "interaction",
        "subtype": "input_line",
        "prompt": prompt,
    }
    send_message(rec)
    with sigint_interruptible():
        return input()


def prompt_choices(prompt, choices: dict, default) -> str:
    rec = {
        "type": "interaction",
        "subtype": "choices",
        "prompt": prompt,
        "choices": choices,
        "default": default,
    }
    send_message(rec)
    with sigint_interruptible():
        return input()


def report_status(status: Optional[str]):
    rec = {
        "type": "status",
        "status": status,
    }
    send_message(rec)


@dataclasses.dataclass
class SpiderpigProgressReportRecord:
    # Use javascript-friendly names (camelCase)
    name: str
    totalTasks: int
    tasksInFlight: Optional[int]
    tasksFinishedOk: int
    tasksFinishedFailed: int
    tasksFinishedTotal: int
    tasksPercentComplete: int
    tasksRemaining: int
    progressFinished: bool


def report_progress(progress: SpiderpigProgressReportRecord):
    rec = {
        "type": "progress",
        "progress": dataclasses.asdict(progress),
    }
    send_message(rec)
