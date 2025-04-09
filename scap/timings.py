import json
from typing import Optional

from scap import utils


class TimingsDatabase:
    """
    A class to manage timings for various operations.
    """

    def __init__(self, filename: str, max_samples: int = 100):
        self.filename = filename
        self.max_samples = max_samples

    def _load(self):
        self.timings = {}

        try:
            with open(self.filename) as f:
                self.timings = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            # Treat non-existent or invalid files as empty
            pass

    def _save(self):
        with utils.temp_to_permanent_file(self.filename) as f:
            json.dump(self.timings, f)

    def add(self, operation: str, time_taken: float):
        """
        Add a timing for a specific operation.

        :param operation: The name of the operation.
        :param time_taken: The time taken for the operation in seconds.
        """

        self._load()

        if operation not in self.timings:
            self.timings[operation] = []

        timings = self.timings[operation]
        timings.append(time_taken)
        # Remove old timings
        while len(timings) > self.max_samples:
            timings.pop(0)

        self._save()

    def estimate(self, operation: str) -> Optional[float]:
        """
        Get a time estimate for a specific operation.

        :param operation: The name of the operation.
        :return: The estimated time for the operation in seconds,
                 or None if no estimate is available.
        """
        self._load()

        timings = self.timings.get(operation)
        if not timings:
            return None

        return sum(timings) / len(timings)
