import json
import os
import sys
import time

from scap import cli, backport, fake_backport_data
from scap.gerrit import GerritSession
from scap.main import AbstractSync

# fake-backport exists to aid in the development of the SpiderPig UI without
# requiring a train-dev environment.


@cli.command(
    "fake-backport",
    help="Simulate a backport operation",
)
class FakeBackport(AbstractSync):
    @cli.argument(
        "change_numbers", nargs="*", help="Change numbers/URLs to backport or revert"
    )
    def main(self, *extra_args):
        logger = self.get_logger()
        logger.info("Welcome to fake scap backport")

        bp = backport.Backport("scap")
        bp.gerrit = GerritSession(url=self.config["gerrit_url"])
        bp.backport_or_revert = "backport"
        change_numbers = [bp._change_number(n) for n in self.arguments.change_numbers]
        bp.backports = backport.GerritChanges(logger, bp.gerrit, change_numbers)
        table = backport.make_table(
            list(change.details for change in bp.backports.changes.values()),
            False,
        )
        bp._prompt_for_approval_or_exit(
            "The following changes are scheduled for backport:\n%s\n"
            "Backport the changes?" % table.get_string()
        )
        self.playback_transcript("prep_and_testservers_sync.jsonl")

        self.arguments.message = bp._build_sal()
        self.arguments.pause_after_testserver_sync = True
        self._pause_after_testserver_sync()
        self.playback_transcript("sync_remainder.jsonl")

    def playback_transcript(self, filename):
        PLAYBACK_SPEED = 3
        MAX_GAP = 1

        transcripts_dir = fake_backport_data.__path__[0]
        with open(os.path.join(transcripts_dir, filename)) as f:
            for rec in f.readlines():
                rec = json.loads(rec)
                payload = rec["payload"]
                type = payload["type"]

                gap = min(rec["gap"] / PLAYBACK_SPEED, MAX_GAP)
                time.sleep(gap)

                if type == "line":
                    line = payload["line"]
                    self.output_line(line, payload.get("sensitive"))
                elif type == "status":
                    self.report_status(payload["status"])
                elif type == "exit":
                    sys.exit(payload["status"])
                else:
                    raise Exception(f"Unexpected payload type: {type}")
