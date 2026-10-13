import json
import os
import stat
import time

import pytest

from scap.metrics import FileCollector, Metrics, MetricsFile

ROLLBACK = {"outcome": "rolled-back", "reason": "checks-failed", "stage": "canaries"}
ROLLBACK_SERIES = ("scap_sync_rollbacks_total", tuple(sorted(ROLLBACK.items())))


def collect(directory) -> dict:
    return {
        (sample.name, tuple(sorted(sample.labels.items()))): sample.value
        for family in FileCollector(MetricsFile(str(directory))).collect()
        for sample in family.samples
    }


def write_series(directory, all_series: dict):
    (directory / "metrics.json").write_text(json.dumps(all_series))


def read_series(directory) -> dict:
    with open(directory / "metrics.json") as f:
        return json.load(f)


def test_observe_records_a_histogram(tmp_path):
    metrics = Metrics(str(tmp_path))

    metrics.operation_duration.observe(42, operation="sync_prod_k8s")
    metrics.operation_duration.observe(4000, operation="sync_prod_k8s")

    assert read_series(tmp_path) == {
        "scap_operation_duration_seconds": [
            {
                "labels": {"operation": "sync_prod_k8s"},
                "value": {"buckets": {"60": 1, "+Inf": 1}, "sum": 4042.0},
            }
        ]
    }

    collected = collect(tmp_path)
    name = "scap_operation_duration_seconds"
    op = ("operation", "sync_prod_k8s")
    assert collected[(f"{name}_bucket", (("le", "30"), op))] == 0
    assert collected[(f"{name}_bucket", (("le", "60"), op))] == 1
    assert collected[(f"{name}_bucket", (("le", "3600"), op))] == 1
    assert collected[(f"{name}_bucket", (("le", "+Inf"), op))] == 2
    assert collected[(f"{name}_count", (op,))] == 2
    assert collected[(f"{name}_sum", (op,))] == 4042


def test_observe_has_buckets_below_one_second(tmp_path):
    metrics = Metrics(str(tmp_path))

    metrics.operation_duration.observe(0.2, operation="helmfile_write_values")

    collected = collect(tmp_path)
    name = "scap_operation_duration_seconds_bucket"
    op = ("operation", "helmfile_write_values")
    assert collected[(name, (("le", "0.1"), op))] == 0
    assert collected[(name, (("le", "0.25"), op))] == 1


def test_inc_records_a_counter(tmp_path):
    metrics = Metrics(str(tmp_path))

    metrics.sync_rollbacks.inc(2, **ROLLBACK)
    metrics.sync_rollbacks.inc(**ROLLBACK)
    metrics.sync_rollbacks.inc(**{**ROLLBACK, "outcome": "cancelled"})

    assert read_series(tmp_path) == {
        "scap_sync_rollbacks_total": [
            {"labels": ROLLBACK, "value": 3},
            {"labels": {**ROLLBACK, "outcome": "cancelled"}, "value": 1},
        ]
    }
    assert collect(tmp_path)[ROLLBACK_SERIES] == 3


def test_inc_requires_the_labels_of_the_metric():
    with pytest.raises(ValueError, match="has the labels"):
        Metrics().sync_rollbacks.inc(stage="canaries")


def test_without_directory_discards_the_metrics(tmp_path):
    metrics = Metrics(str(tmp_path / "absent"))

    metrics.sync_rollbacks.inc(**ROLLBACK)

    assert not (tmp_path / "absent").exists()


def test_file_is_group_readable_whatever_the_umask(tmp_path):
    metrics = Metrics(str(tmp_path))
    old_umask = os.umask(0o077)
    try:
        metrics.sync_rollbacks.inc(**ROLLBACK)
    finally:
        os.umask(old_umask)

    mode = os.stat(tmp_path / "metrics.json").st_mode
    assert stat.S_IMODE(mode) == 0o640


def test_concurrent_processes_lose_no_update(tmp_path):
    metrics = Metrics(str(tmp_path))

    pids = []
    for _ in range(4):
        pid = os.fork()
        if pid == 0:
            try:
                for _ in range(25):
                    metrics.sync_rollbacks.inc(**ROLLBACK)
            finally:
                os._exit(0)
        pids.append(pid)
    for pid in pids:
        os.waitpid(pid, 0)

    assert collect(tmp_path)[ROLLBACK_SERIES] == 100


def test_update_replaces_a_file_it_cannot_write(tmp_path):
    # Another deployer wrote the file last.
    write_series(
        tmp_path, {"scap_sync_rollbacks_total": [{"labels": ROLLBACK, "value": 1}]}
    )
    (tmp_path / "metrics.json").chmod(0o440)

    Metrics(str(tmp_path)).sync_rollbacks.inc(**ROLLBACK)

    assert collect(tmp_path)[ROLLBACK_SERIES] == 2
    assert os.listdir(tmp_path) == ["metrics.json"]


def test_update_with_unreadable_file_keeps_the_file(tmp_path):
    (tmp_path / "metrics.json").write_text("{")

    Metrics(str(tmp_path)).sync_rollbacks.inc(**ROLLBACK)

    assert (tmp_path / "metrics.json").read_text() == "{"


def test_update_replaces_a_value_of_the_wrong_shape(tmp_path):
    write_series(
        tmp_path,
        {
            "scap_sync_rollbacks_total": [
                {"labels": ROLLBACK, "value": {"buckets": {}, "sum": 0}}
            ],
            "scap_operation_duration_seconds": [
                {"labels": {"operation": "prep"}, "value": 3}
            ],
        },
    )
    metrics = Metrics(str(tmp_path))

    metrics.sync_rollbacks.inc(**ROLLBACK)
    metrics.operation_duration.observe(2, operation="prep")

    assert read_series(tmp_path) == {
        "scap_sync_rollbacks_total": [{"labels": ROLLBACK, "value": 1}],
        "scap_operation_duration_seconds": [
            {
                "labels": {"operation": "prep"},
                "value": {"buckets": {"5": 1}, "sum": 2.0},
            }
        ],
    }


def test_update_replaces_series_of_an_older_format(tmp_path):
    write_series(tmp_path, {"scap_sync_rollbacks_total": {'["canaries", "x", "y"]': 1}})

    Metrics(str(tmp_path)).sync_rollbacks.inc(**ROLLBACK)

    assert read_series(tmp_path) == {
        "scap_sync_rollbacks_total": [{"labels": ROLLBACK, "value": 1}]
    }


def test_collector_skips_series_of_an_older_version_of_a_metric(tmp_path):
    write_series(
        tmp_path,
        {
            "scap_sync_rollbacks_total": [
                {"labels": {**ROLLBACK, "x": "y"}, "value": 1},
                {"labels": {"stage": "canaries"}, "value": 1},
            ]
        },
    )

    assert collect(tmp_path) == {}


def test_collector_skips_a_value_of_the_wrong_shape(tmp_path):
    write_series(
        tmp_path,
        {
            "scap_sync_rollbacks_total": [
                {"labels": ROLLBACK, "value": {"buckets": {}, "sum": 0}}
            ],
            "scap_operation_duration_seconds": [
                {"labels": {"operation": "prep"}, "value": 3}
            ],
        },
    )

    assert collect(tmp_path) == {}


def test_collector_with_unreadable_file_exports_no_series(tmp_path):
    (tmp_path / "metrics.json").write_text("{")

    assert collect(tmp_path) == {}


def test_collector_without_file_exports_no_series(tmp_path):
    assert collect(tmp_path) == {}


def test_remove_stale_temp_files(tmp_path):
    stale = tmp_path / "tmpabc"
    fresh = tmp_path / "tmpdef"
    metrics_file = tmp_path / "metrics.json"
    for path in (stale, fresh, metrics_file):
        path.write_text("{}")
    two_hours_ago = time.time() - 7200
    for path in (stale, metrics_file):
        os.utime(path, (two_hours_ago, two_hours_ago))

    MetricsFile(str(tmp_path)).remove_stale_data()

    assert sorted(os.listdir(tmp_path)) == ["metrics.json", "tmpdef"]


def test_remove_stale_series(tmp_path):
    current = {"labels": ROLLBACK, "value": 2}
    write_series(
        tmp_path,
        {
            "scap_sync_rollbacks_total": [
                current,
                {"labels": {**ROLLBACK, "zone": "canaries"}, "value": 1},
                {"labels": ROLLBACK, "value": {"sum": 1}},
                ["canaries", "checks-failed", "rolled-back"],
            ],
            "scap_operation_duration_seconds": {'["prep"]': 1},
            "scap_removed_metric_total": [{"labels": {}, "value": 1}],
        },
    )

    MetricsFile(str(tmp_path)).remove_stale_data()

    assert read_series(tmp_path) == {"scap_sync_rollbacks_total": [current]}


def test_remove_stale_series_keeps_a_file_it_cannot_read(tmp_path):
    (tmp_path / "metrics.json").write_text("{")

    MetricsFile(str(tmp_path)).remove_stale_data()

    assert (tmp_path / "metrics.json").read_text() == "{"


def test_a_file_of_the_wrong_shape_stops_nothing(tmp_path):
    write_series(tmp_path, [])

    Metrics(str(tmp_path)).sync_rollbacks.inc(**ROLLBACK)
    MetricsFile(str(tmp_path)).remove_stale_data()

    assert collect(tmp_path) == {}
    assert read_series(tmp_path) == []
