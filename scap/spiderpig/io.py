import json
import os
import sys

from typing import Optional


def send_message(rec: dict):
    rec["iokey"] = os.environ["SPIDERPIG_IO_KEY"]
    print(json.dumps(rec))
    sys.stdout.flush()


def input_line(prompt) -> str:
    rec = {
        "type": "interaction",
        "subtype": "input_line",
        "prompt": prompt,
    }
    send_message(rec)
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
    return input()


def report_status(status: Optional[str]):
    rec = {
        "type": "status",
        "status": status,
    }
    send_message(rec)
