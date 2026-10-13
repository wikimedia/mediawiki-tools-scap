"""
    scap.metrics
    ~~~~~~~~~~~~
    Prometheus metrics of scap commands.

    A scap command on the deployment server records its metrics in SpiderPig's
    metrics directory. The SpiderPig apiserver reads the file
    and exports the metrics. A scap command running on a non-deployment host
    discards its metrics.

    All metrics are defined in the Metrics class, so that the apiserver knows
    about all of them.
"""

import contextlib
import fcntl
import glob
import json
import logging
import math
import os
import time
from typing import List, Optional

from prometheus_client.metrics_core import CounterMetricFamily, HistogramMetricFamily
from prometheus_client.registry import Collector

import scap.utils as utils

logger = logging.getLogger("scap.metrics")


class MetricsFile:
    """
    The file "metrics.json" in `directory`.

    The file maps the name of each metric to a list of its series. Each
    series has a set of labels and a value. The value of a counter is a number.
    The value of a histogram has the count of each bucket and the sum. For
    example:

        {
            "scap_sync_rollbacks_total": [
                {
                    "labels": {
                        "outcome": "rolled-back",
                        "reason": "checks-failed",
                        "stage": "canaries"
                    },
                    "value": 2
                },
                {
                    "labels": {
                        "outcome": "cancelled",
                        "reason": "deploy-failed",
                        "stage": "production"
                    },
                    "value": 1
                }
            ],
            "scap_operation_duration_seconds": [
                {
                    "labels": {"operation": "sync_prod_k8s"},
                    "value": {"buckets": {"60": 1, "+Inf": 1}, "sum": 4042.0}
                },
                {
                    "labels": {"operation": "sync_canaries_k8s"},
                    "value": {"buckets": {"15": 3, "30": 1}, "sum": 61.5}
                }
            ]
        }
    """

    def __init__(self, directory: str):
        self.directory = directory
        self.filename = os.path.join(directory, "metrics.json")

    def read(self) -> dict:
        try:
            with open(self.filename) as f:
                return json.load(f)
        except FileNotFoundError:
            return {}

    def _write(self, all_series: dict):
        with utils.temp_to_permanent_file(self.filename, permissions=0o640) as f:
            json.dump(all_series, f)

    @contextlib.contextmanager
    def _locked(self):
        # Note: The lock is on the directory, not the metrics file itself
        lock = os.open(self.directory, os.O_RDONLY)
        try:
            fcntl.flock(lock, fcntl.LOCK_EX)
            yield
        finally:
            os.close(lock)

    def update(self, name: str, labels: dict, update_func):
        """
        Atomically replace the value of the series of metric `name` with
        `labels` with the result of evaluating `update_func(old_value)`.
        """
        with self._locked():
            all_series = self.read()
            metric_series = all_series.get(name)
            if not isinstance(metric_series, list):
                metric_series = all_series[name] = []
            for series in metric_series:
                if isinstance(series, dict) and series.get("labels") == labels:
                    series["value"] = update_func(series.get("value"))
                    break
            else:
                metric_series.append({"labels": labels, "value": update_func(None)})
            self._write(all_series)

    def remove_stale_data(self):
        """Remove the stale temp files and the stale series of the directory."""
        self._remove_stale_temp_files()
        self._remove_stale_series(Metrics().all())

    def _remove_stale_series(self, metrics: List["_Metric"]):
        """
        Remove stored series that do not match a metric in `metrics`.

        A series is stale when its metric is not in `metrics`, when its
        labels differ from the labels of its metric, or when its value has a
        different shape, for example a number for a histogram.
        """
        defined = {metric.name: metric for metric in metrics}
        try:
            with self._locked():
                all_series = self.read()
                kept = {
                    name: [
                        series
                        for series in metric_series
                        if defined[name]._exportable(series)
                    ]
                    for name, metric_series in all_series.items()
                    if name in defined and isinstance(metric_series, list)
                }
                if kept != all_series:
                    self._write(kept)
        except (OSError, ValueError) as e:
            logger.warning(f"Could not remove the stale series of {self.filename}: {e}")

    def _remove_stale_temp_files(self):
        max_age = 3600
        for tmp in glob.glob(os.path.join(self.directory, "tmp*")):
            try:
                if time.time() - os.stat(tmp).st_mtime > max_age:
                    os.unlink(tmp)
            except FileNotFoundError:
                pass
            except OSError as e:
                logger.warning(f"Could not remove {tmp}: {e}")


class _Metric:
    def __init__(
        self,
        file: Optional[MetricsFile],
        name: str,
        documentation: str,
        labelnames: List[str],
    ):
        self.file = file
        self.name = name
        self.documentation = documentation
        self.labelnames = labelnames

    def _update(self, labels: dict, update_func):
        """
        Replace the value of the series of `labels` with `update_func(old_value)`.

        `old_value` is None for a new series, or when the stored value has the
        wrong shape. Without a file, the update is discarded.
        """
        if set(labels) != set(self.labelnames):
            raise ValueError(
                f"{self.name} has the labels {self.labelnames}, not {sorted(labels)}"
            )
        if self.file is None:
            return

        # Ensure all label values are strings.
        labels = {name: str(value) for name, value in labels.items()}

        def checked_update(value):
            return update_func(value if self._valid_value(value) else None)

        try:
            self.file.update(self.name, labels, checked_update)
        except (OSError, ValueError) as e:
            logger.warning(f"Could not record {self.name}: {e}")

    def _label_values(self, series) -> Optional[List[str]]:
        """
        Returns the label values of `series`, in the order of `labelnames`.

        Returns None when the labels of `series` are not the labels of this
        metric.
        """
        labels = series.get("labels") if isinstance(series, dict) else None
        # The file can have series of an older version of the metric.
        if not isinstance(labels, dict) or set(labels) != set(self.labelnames):
            return None
        if not all(isinstance(value, str) for value in labels.values()):
            return None
        return [labels[name] for name in self.labelnames]

    def _exportable(self, series) -> bool:
        """
        Returns True when `series` has the labels of this metric and a value
        of the right shape.
        """
        return self._label_values(series) is not None and self._valid_value(
            series.get("value")
        )

    def _series(self, all_series: dict):
        """Yields the label values and the value of each series of this metric."""
        metric_series = all_series.get(self.name)
        if not isinstance(metric_series, list):
            return
        for series in metric_series:
            if self._exportable(series):
                yield self._label_values(series), series["value"]

    def _valid_value(self, value) -> bool:
        """
        Returns True when `value` has the shape of a value of this type of
        metric, for example a number for a counter.
        """
        raise NotImplementedError

    def family(self, all_series: dict):
        """
        Returns the Prometheus metric family of this metric, for the collector
        to export. `all_series` is the content of the metrics file. The family
        has each series of this metric from it, except a series with other
        labels or a value of the wrong shape.
        """
        raise NotImplementedError


def _is_number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


class Counter(_Metric):
    def inc(self, amount=1, **labels):
        self._update(labels, lambda value: (value or 0) + amount)

    def _valid_value(self, value) -> bool:
        return _is_number(value)

    def family(self, all_series: dict):
        family = CounterMetricFamily(
            self.name, self.documentation, labels=self.labelnames
        )
        for label_values, value in self._series(all_series):
            family.add_metric(label_values, value)
        return family


class Histogram(_Metric):
    def __init__(self, file, name, documentation, labelnames, buckets):
        super().__init__(file, name, documentation, labelnames)
        self.buckets = (*buckets, math.inf)

    def observe(self, amount, **labels):
        """
        Add `amount` to the histogram of `labels`.

        The value of a series is like {"buckets": {"60": 2, "+Inf": 1}, "sum": 4242.0}.
        The count of a bucket is the number of observations in that bucket only.
        """
        bucket = _le(next(bound for bound in self.buckets if amount <= bound))

        def update(value):
            value = value or {"buckets": {}, "sum": 0.0}
            value["buckets"][bucket] = value["buckets"].get(bucket, 0) + 1
            value["sum"] += amount
            return value

        self._update(labels, update)

    def _valid_value(self, value) -> bool:
        if not isinstance(value, dict) or not _is_number(value.get("sum")):
            return False
        buckets = value.get("buckets")
        if not isinstance(buckets, dict):
            return False
        for bound, count in buckets.items():
            try:
                float(bound)
            except ValueError:
                return False
            if not _is_number(count):
                return False
        return True

    def family(self, all_series: dict):
        family = HistogramMetricFamily(
            self.name, self.documentation, labels=self.labelnames
        )
        for label_values, value in self._series(all_series):
            counts = {float(b): c for b, c in value["buckets"].items()}
            buckets = [
                (_le(le), sum(c for bound, c in counts.items() if bound <= le))
                for le in self.buckets
            ]
            family.add_metric(label_values, buckets, value["sum"])
        return family


def _le(bound) -> str:
    return "+Inf" if bound == math.inf else str(bound)


class Metrics:
    """
    The metrics of scap commands.

    The metrics are recorded in the file "metrics.json" in `directory`. If
    `directory` is None or does not exist, the metrics are discarded.
    """

    def __init__(self, directory: Optional[str] = None):
        file = (
            MetricsFile(directory) if directory and os.path.isdir(directory) else None
        )
        durations = (0.1, 0.25, 0.5, 1, 5, 15, 30, 60, 120, 300, 600, 1200, 1800, 3600)

        self.command_duration = Histogram(
            file,
            "scap_command_duration_seconds",
            "The duration of each scap command, by exit status.",
            ["command", "exit_status"],
            durations,
        )
        self.operation_duration = Histogram(
            file,
            "scap_operation_duration_seconds",
            "The duration of each timed operation of scap commands.",
            ["operation"],
            durations,
        )
        self.sync_rollbacks = Counter(
            file,
            "scap_sync_rollbacks_total",
            "The Kubernetes rollbacks of sync commands, by the stage that failed.",
            ["stage", "reason", "outcome"],
        )
        self.backport_changes = Counter(
            file,
            "scap_backport_changes_total",
            "The changes that scap backport synced, by the outcome of the sync.",
            ["outcome"],
        )
        self.train_promotion_duration = Histogram(
            file,
            "scap_train_promotion_duration_seconds",
            "The duration of each train promotion, by group and outcome.",
            ["group", "outcome"],
            durations,
        )

    def all(self) -> List[_Metric]:
        return [value for value in vars(self).values() if isinstance(value, _Metric)]


class FileCollector(Collector):
    """Exports the metrics in `file`."""

    def __init__(self, file: MetricsFile):
        self.file = file

    def collect(self):
        try:
            all_series = self.file.read()
        except (OSError, ValueError) as e:
            logger.warning(f"Could not read {self.file.filename}: {e}")
            all_series = {}

        for metric in Metrics().all():
            yield metric.family(all_series)
