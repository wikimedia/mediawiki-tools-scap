import asyncio
import json

import pytest

from scap.spiderpig.events import Broadcaster


class Snapshots:
    """A snapshot function that returns the given snapshots, then the last one again."""

    def __init__(self, *snapshots):
        self.snapshots = list(snapshots)

    def __call__(self):
        snapshot = self.snapshots[0]
        if len(self.snapshots) > 1:
            self.snapshots.pop(0)
        if isinstance(snapshot, Exception):
            raise snapshot
        return snapshot


async def next_event(stream) -> dict:
    return json.loads(await asyncio.wait_for(stream.__anext__(), 5))


@pytest.fixture
async def run_broadcaster():
    tasks = []

    def run(broadcaster):
        tasks.append(asyncio.create_task(broadcaster.run()))

    yield run

    for task in tasks:
        task.cancel()


@pytest.mark.anyio
async def test_stream_sends_each_event_then_only_changes(run_broadcaster):
    broadcaster = Broadcaster(
        Snapshots(
            {"jobrunner": {"status": "idle"}, "jobs_digest": "a"},
            {"jobrunner": {"status": "idle"}, "jobs_digest": "b"},
            {"jobrunner": {"status": "running"}, "jobs_digest": "b"},
        ),
        interval=0.01,
    )
    stream = broadcaster.stream()
    first = asyncio.ensure_future(next_event(stream))
    run_broadcaster(broadcaster)

    assert await first == {"type": "jobrunner", "data": {"status": "idle"}}
    assert await next_event(stream) == {"type": "jobs_digest", "data": "a"}
    assert await next_event(stream) == {"type": "jobs_digest", "data": "b"}
    assert await next_event(stream) == {
        "type": "jobrunner",
        "data": {"status": "running"},
    }
    await stream.aclose()


@pytest.mark.anyio
async def test_new_stream_starts_with_the_latest_snapshot(run_broadcaster):
    broadcaster = Broadcaster(Snapshots({"jobs_digest": "a"}), interval=0.01)
    first_stream = broadcaster.stream()
    first = asyncio.ensure_future(next_event(first_stream))
    run_broadcaster(broadcaster)
    await first

    second_stream = broadcaster.stream()

    assert await next_event(second_stream) == {"type": "jobs_digest", "data": "a"}
    await first_stream.aclose()
    await second_stream.aclose()


@pytest.mark.anyio
async def test_stream_sends_keepalives_when_nothing_changes(run_broadcaster):
    broadcaster = Broadcaster(
        Snapshots({"jobs_digest": "a"}), interval=0.01, keepalive_interval=0.05
    )
    stream = broadcaster.stream()
    first = asyncio.ensure_future(next_event(stream))
    run_broadcaster(broadcaster)
    await first

    assert await next_event(stream) == {"type": "keepalive"}
    await stream.aclose()


@pytest.mark.anyio
async def test_broadcaster_continues_after_a_failed_snapshot(run_broadcaster, caplog):
    broadcaster = Broadcaster(
        Snapshots(RuntimeError("database is locked"), {"jobs_digest": "a"}),
        interval=0.01,
    )
    stream = broadcaster.stream()
    first = asyncio.ensure_future(next_event(stream))
    run_broadcaster(broadcaster)

    assert await first == {"type": "jobs_digest", "data": "a"}
    assert "database is locked" in caplog.text
    await stream.aclose()


@pytest.mark.anyio
async def test_closed_stream_stops_getting_snapshots(run_broadcaster):
    broadcaster = Broadcaster(Snapshots({"jobs_digest": "a"}), interval=0.01)
    stream = broadcaster.stream()
    first = asyncio.ensure_future(next_event(stream))
    run_broadcaster(broadcaster)
    await first

    await stream.aclose()

    assert broadcaster._queues == set()


@pytest.mark.anyio
async def test_stream_ends_when_snapshots_stall(caplog):
    # No run() task, so no snapshot ever succeeds.
    broadcaster = Broadcaster(
        Snapshots({"jobs_digest": "a"}), keepalive_interval=0.01, stall_timeout=0.05
    )

    events = [json.loads(line) async for line in broadcaster.stream()]

    assert events and all(event == {"type": "keepalive"} for event in events)
    assert "Closing an event stream" in caplog.text
    assert broadcaster._queues == set()
