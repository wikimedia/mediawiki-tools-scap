import asyncio
import json
import logging
import time
from typing import AsyncIterator, Callable, Dict, Optional, Set

import anyio.to_thread

logger = logging.getLogger(__name__)


class Broadcaster:
    """Sends changes in server state to each client stream.

    One task takes a snapshot of the state each interval, for all clients.
    A snapshot is a dict.  Each key is an event type and each value is the
    data of that event.
    """

    def __init__(
        self,
        snapshot_func: Callable[[], Dict],
        interval: float = 1,
        keepalive_interval: float = 15,
        stall_timeout: float = 30,
    ):
        self._snapshot_func = snapshot_func
        self._interval = interval
        self._keepalive_interval = keepalive_interval
        self._stall_timeout = stall_timeout
        self._snapshot: Optional[Dict] = None
        self._snapshot_time = time.monotonic()
        # Each stream has a queue that holds only the latest snapshot.
        self._queues: Set[asyncio.Queue] = set()

    async def run(self):
        """Takes a snapshot each interval until cancelled.

        The snapshot function runs in a worker thread.
        """
        while True:
            try:
                snapshot = await anyio.to_thread.run_sync(self._snapshot_func)
            except Exception as e:
                logger.warning(f"Cannot get the state for the event streams: {e}")
            else:
                self._snapshot_time = time.monotonic()
                if snapshot != self._snapshot:
                    self._snapshot = snapshot
                    for queue in self._queues:
                        self._put_latest(queue, snapshot)
            await asyncio.sleep(self._interval)

    def latest_snapshot(self) -> Optional[Dict]:
        """Returns the latest snapshot, or None when no snapshot has succeeded for the stall timeout."""
        if time.monotonic() - self._snapshot_time > self._stall_timeout:
            return None
        return self._snapshot

    @staticmethod
    def _put_latest(queue: asyncio.Queue, snapshot: Dict):
        if queue.full():
            queue.get_nowait()
        queue.put_nowait(snapshot)

    async def stream(self) -> AsyncIterator[str]:
        """Yields events as JSON lines.

        The stream starts with each event of the latest snapshot.  After that,
        it yields an event only when its data changes.  A "keepalive" event
        comes after each keepalive interval with no other events.  The stream
        ends when no snapshot has succeeded for the stall timeout.  Example:

            {"type": "jobs_digest", "data": "5f2c..."}
            {"type": "keepalive"}
        """
        queue = asyncio.Queue(maxsize=1)
        if self._snapshot is not None:
            queue.put_nowait(self._snapshot)
        self._queues.add(queue)

        sent = {}
        try:
            while True:
                try:
                    snapshot = await asyncio.wait_for(
                        queue.get(), self._keepalive_interval
                    )
                except asyncio.TimeoutError:
                    age = time.monotonic() - self._snapshot_time
                    if age > self._stall_timeout:
                        logger.warning(
                            f"Closing an event stream: the last snapshot is {age:.0f} seconds old"
                        )
                        return
                    yield json.dumps({"type": "keepalive"}) + "\n"
                    continue

                for type, data in snapshot.items():
                    if type not in sent or sent[type] != data:
                        sent[type] = data
                        yield json.dumps({"type": type, "data": data}) + "\n"
        finally:
            self._queues.discard(queue)
