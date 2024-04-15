import base64
import json
import pickle
import sys


def send_message(rec):
    print(json.dumps(rec))
    sys.stdout.flush()


def send_plan(plan):
    rec = {
        "name": "spiderpig-message",
        "type": "plan",
        "plan": base64.b64encode(pickle.dumps(plan)).decode(),
    }
    send_message(rec)


def prompt(prompt_message, choices: dict, default) -> str:
    rec = {
        "name": "spiderpig-message",
        "type": "prompt",
        "prompt_message": prompt_message,
        "choices": choices,
        "default": default,
    }
    send_message(rec)
    return input()
