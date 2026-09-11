#!/usr/bin/env python3
"""Inspect JSONL telemetry written by the on-board flight recorder."""

from __future__ import annotations

import argparse
import json
import math
import re
import time
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, RadioButtons, Slider
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


G0 = 9.80665
# The IMU is fixed in the rocket.  This proper rotation maps an IMU/body-frame
# vector into the project rocket frame, whose +Z axis points to the nosecone.
R_ROCKET_FROM_IMU = np.array(
    [
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ]
)
# Kept explicit for a future sensor-origin model.  Translation does not affect
# the free vectors displayed by the replay (acceleration, gravity, omega, mag).
IMU_ORIGIN_ROCKET_M = np.zeros(3)
REPLAY_SPEEDS = (0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0)
LEGACY_FSM_RE = re.compile(r"FSM transition t=(\d+) ms:\s*(\S+)\s*->\s*(\S+)")
FSM_RE = re.compile(r"(?:FSM transition t=\d+ ms:\s*)?(\S+)\s*->\s*(\S+)")


@dataclass(frozen=True)
class FsmTransition:
    """One recorded FSM transition, expressed relative to the log origin."""

    time_s: float
    old_state: str
    new_state: str


@dataclass(frozen=True)
class ReplayStream:
    """Timestamped scalar or vector samples, preserved in their recorded units."""

    time_s: np.ndarray
    values: np.ndarray
    label: str
    unit: str


@dataclass(frozen=True)
class FlightReplayData:
    """Recorded signals synchronized by the replay clock, not a reconstructed trajectory."""

    timeline_s: np.ndarray
    rotations_world_from_rocket: np.ndarray
    imu_source: str
    acceleration: ReplayStream | None
    linear_acceleration: ReplayStream | None
    imu_temperature: ReplayStream | None
    angular_rate: ReplayStream | None
    magnetic_field: ReplayStream | None
    gravity: ReplayStream | None
    pressure: ReplayStream | None
    temperature: ReplayStream | None
    barometric_altitude: ReplayStream | None
    transitions: list[FsmTransition]
    launch_time_s: float | None


def transform_imu_to_rocket(vector_imu: np.ndarray | Iterable[float]) -> np.ndarray:
    """Map an IMU-frame free vector into the right-handed rocket body frame."""
    vector = np.asarray(vector_imu, dtype=float)
    return R_ROCKET_FROM_IMU @ vector


def _verify_imu_to_rocket_transform() -> None:
    """Keep the mounting convention executable and close to its definition."""
    assert np.allclose(transform_imu_to_rocket([1, 0, 0]), [0, 0, -1])
    assert np.allclose(transform_imu_to_rocket([0, 1, 0]), [0, 1, 0])
    assert np.allclose(transform_imu_to_rocket([0, 0, 1]), [1, 0, 0])


_verify_imu_to_rocket_transform()


def load_records(path: Path) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Load valid records and retain diagnostics for every malformed line."""
    records: list[dict[str, Any]] = []
    malformed: list[dict[str, Any]] = []

    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line_number, raw_line in enumerate(stream, 1):
            line = raw_line.rstrip("\r\n")
            if not line.strip():
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError as error:
                malformed.append(
                    {
                        "line": line_number,
                        "column": error.colno,
                        "error": error.msg,
                        "text": line,
                    }
                )
                continue

            if not isinstance(obj, dict):
                malformed.append(
                    {
                        "line": line_number,
                        "column": 1,
                        "error": "top-level JSON value is not an object",
                        "text": line,
                    }
                )
                continue

            payload = obj.get("sensorData")
            if not isinstance(payload, dict):
                payload = obj.get("content")
            if not isinstance(payload, dict):
                payload = {}

            source = str(obj.get("source") or obj.get("type") or "UNKNOWN")
            timestamp_ms = payload.get("timestamp", payload.get("t"))
            message = str(payload.get("message", ""))
            legacy_fsm_match = LEGACY_FSM_RE.search(message)
            fsm_match = FSM_RE.search(message)
            if timestamp_ms is None and legacy_fsm_match:
                timestamp_ms = int(legacy_fsm_match.group(1))

            try:
                timestamp_ms = float(timestamp_ms) if timestamp_ms is not None else None
            except (TypeError, ValueError):
                timestamp_ms = None

            records.append(
                {
                    "line": line_number,
                    "source": source,
                    "timestamp_ms": timestamp_ms,
                    "payload": payload,
                    "type": obj.get("type"),
                    "is_event": isinstance(obj.get("type"), str) and isinstance(obj.get("content"), dict),
                    "fsm": (
                        (fsm_match.group(1), fsm_match.group(2))
                        if fsm_match
                        else None
                    ),
                }
            )

    return records, malformed


def split_records(records: Iterable[dict[str, Any]]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Split sensor data records from recorder events without changing JSONL storage."""
    datalog: list[dict[str, Any]] = []
    events: list[dict[str, Any]] = []
    for record in records:
        (events if record["is_event"] else datalog).append(record)
    return datalog, events


def group_records(records: Iterable[dict[str, Any]]) -> dict[str, list[dict[str, Any]]]:
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for record in records:
        grouped[record["source"]].append(record)
    return dict(grouped)


def numeric_series(
    source_records: Iterable[dict[str, Any]], field: str, origin_ms: float
) -> tuple[np.ndarray, np.ndarray]:
    points: list[tuple[float, float]] = []
    for record in source_records:
        timestamp = record["timestamp_ms"]
        value = record["payload"].get(field)
        if timestamp is None or isinstance(value, bool) or not isinstance(value, (int, float)):
            continue
        value = float(value)
        if math.isfinite(value):
            points.append(((timestamp - origin_ms) / 1000.0, value))
    if not points:
        return np.array([]), np.array([])
    points.sort(key=lambda point: point[0])
    return np.asarray([point[0] for point in points]), np.asarray([point[1] for point in points])


def vector_series(
    source_records: Iterable[dict[str, Any]], fields: tuple[str, str, str] | tuple[str, str, str, str], origin_ms: float
) -> tuple[np.ndarray, np.ndarray]:
    """Return only complete finite vector samples, retaining their source timestamps."""
    points: list[tuple[float, list[float]]] = []
    for record in source_records:
        timestamp = record["timestamp_ms"]
        values = [record["payload"].get(field) for field in fields]
        if timestamp is None or any(isinstance(value, bool) or not isinstance(value, (int, float)) for value in values):
            continue
        numeric_values = [float(value) for value in values]
        if all(math.isfinite(value) for value in numeric_values):
            points.append(((timestamp - origin_ms) / 1000.0, numeric_values))
    if not points:
        return np.array([]), np.empty((0, len(fields)))
    points.sort(key=lambda point: point[0])
    return np.asarray([point[0] for point in points]), np.asarray([point[1] for point in points])


def sample_and_hold(stream: ReplayStream | None, time_s: float) -> np.ndarray | None:
    """Return the newest recorded value at or before replay time; never invent samples."""
    if stream is None or not stream.time_s.size:
        return None
    index = int(np.searchsorted(stream.time_s, time_s, side="right") - 1)
    return stream.values[max(index, 0)]


def quaternion_wxyz_to_matrix(quaternion: np.ndarray | Iterable[float]) -> np.ndarray:
    """Return R_world_from_imu for the recorded W, X, Y, Z quaternion convention."""
    w, x, y, z = np.asarray(quaternion, dtype=float)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1e-9:
        raise ValueError("zero-length recorded quaternion")
    w, x, y, z = (w / norm, x / norm, y / norm, z / norm)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def source_has_fields(source_records: list[dict[str, Any]], *fields: str) -> bool:
    present = {key for record in source_records for key in record["payload"]}
    return all(field in present for field in fields)


def fsm_transitions(records: list[dict[str, Any]], origin_ms: float) -> list[FsmTransition]:
    """Extract ordered FSM transitions without changing the event parser."""
    transitions: list[FsmTransition] = []
    for record in records:
        if record["timestamp_ms"] is None or not record["fsm"]:
            continue
        old_state, new_state = record["fsm"]
        transitions.append(
            FsmTransition((record["timestamp_ms"] - origin_ms) / 1000.0, old_state, new_state)
        )
    return sorted(transitions, key=lambda transition: transition.time_s)


def fsm_state_intervals(
    transitions: list[FsmTransition], start_s: float, end_s: float
) -> list[tuple[float, float, str]]:
    """Return the state occupied by each interval covered by the plotted log."""
    if not transitions or end_s <= start_s:
        return []

    intervals: list[tuple[float, float, str]] = []
    first = transitions[0]
    if first.time_s > start_s:
        intervals.append((start_s, first.time_s, first.old_state))
    for index, transition in enumerate(transitions):
        interval_end = transitions[index + 1].time_s if index + 1 < len(transitions) else end_s
        if interval_end > transition.time_s:
            intervals.append((transition.time_s, interval_end, transition.new_state))
    return intervals


def select_source_with_fields(
    grouped: dict[str, list[dict[str, Any]]], *fields: str
) -> str | None:
    """Choose the most complete source for one replay signal family."""
    candidates = [
        (source, source_records)
        for source, source_records in grouped.items()
        if source_has_fields(source_records, *fields)
    ]
    return max(candidates, key=lambda candidate: len(candidate[1]))[0] if candidates else None


def build_flight_replay(
    records: list[dict[str, Any]],
    events: list[dict[str, Any]],
    grouped: dict[str, list[dict[str, Any]]],
) -> FlightReplayData | None:
    """Build a replay from recorded signals; refuse unsafe gyro-only attitude inference."""
    timestamped = [
        record["timestamp_ms"]
        for record in (*records, *events)
        if record["timestamp_ms"] is not None
    ]
    if not timestamped:
        return None
    origin_ms = min(timestamped)
    imu_source = select_source_with_fields(grouped, "qw", "qx", "qy", "qz")
    if imu_source is None:
        print("[REPLAY] unavailable: no recorded quaternion; refusing uncorrected gyro integration.")
        return None

    quaternion_time_s, quaternions = vector_series(
        grouped[imu_source], ("qw", "qx", "qy", "qz"), origin_ms
    )
    valid = np.linalg.norm(quaternions, axis=1) > 1e-9
    quaternion_time_s, quaternions = quaternion_time_s[valid], quaternions[valid]
    if not quaternion_time_s.size:
        print("[REPLAY] unavailable: all recorded quaternions are invalid.")
        return None

    # Firmware/HIL logs quaternion fields as W, X, Y, Z.  HIL explicitly
    # defines it as IMU/body -> world.  For a physical BNO055 log we preserve
    # its reported orientation but remove the first-sample offset: launch-pad
    # world and rocket body therefore coincide at t0 by construction.
    recorded_world_from_imu = np.asarray([quaternion_wxyz_to_matrix(q) for q in quaternions])
    recorded_world_from_rocket = recorded_world_from_imu @ R_ROCKET_FROM_IMU.T
    initial_world_from_rocket = recorded_world_from_rocket[0]
    rotations_world_from_rocket = initial_world_from_rocket.T @ recorded_world_from_rocket

    def imu_vector(fields: tuple[str, str, str], label: str, unit: str) -> ReplayStream | None:
        time_s, values = vector_series(grouped[imu_source], fields, origin_ms)
        return ReplayStream(time_s, values, label, unit) if time_s.size else None

    acceleration = imu_vector(("ax", "ay", "az"), "IMU total acceleration", "m/s²")
    linear_acceleration = imu_vector(("lax", "lay", "laz"), "IMU linear acceleration", "m/s²")
    imu_temperature_time_s, imu_temperature_values = numeric_series(grouped[imu_source], "te", origin_ms)
    imu_temperature = (
        ReplayStream(imu_temperature_time_s, imu_temperature_values, "IMU temperature", "°C")
        if imu_temperature_time_s.size
        else None
    )
    angular_rate = imu_vector(("avx", "avy", "avz"), "IMU angular rate", "rad/s")
    magnetic_field = imu_vector(("mx", "my", "mz"), "Magnetic field", "µT")
    gravity = imu_vector(("gx", "gy", "gz"), "BNO055 gravity estimate", "m/s²")

    pressure_source = select_source_with_fields(grouped, "pressure", "temperature")
    pressure = temperature = barometric_altitude = None
    if pressure_source is not None:
        pressure_time_s, pressure_values = numeric_series(grouped[pressure_source], "pressure", origin_ms)
        temperature_time_s, temperature_values = numeric_series(grouped[pressure_source], "temperature", origin_ms)
        if pressure_time_s.size:
            pressure = ReplayStream(pressure_time_s, pressure_values, f"{pressure_source} pressure", "Pa")
            positive = pressure_values[pressure_values > 0.0]
            if positive.size:
                reference_pressure_pa = float(np.median(positive[: min(20, positive.size)]))
                altitude_m = 44330.0 * (1.0 - np.power(pressure_values / reference_pressure_pa, 0.19029495))
                barometric_altitude = ReplayStream(
                    pressure_time_s, altitude_m, "Barometric altitude relative to log start", "m"
                )
        if temperature_time_s.size:
            temperature = ReplayStream(temperature_time_s, temperature_values, f"{pressure_source} temperature", "°C")

    transitions = fsm_transitions(events, origin_ms)
    launch_time_s = next((transition.time_s for transition in transitions if transition.new_state == "LAUNCH"), None)
    available_streams = [
        stream.label
        for stream in (acceleration, linear_acceleration, imu_temperature, angular_rate, magnetic_field, gravity, pressure, temperature)
        if stream is not None
    ]
    print(
        f"[REPLAY] attitude={imu_source} quaternion ({len(quaternion_time_s)} samples); "
        f"signals={', '.join(available_streams) or 'none'}"
    )
    return FlightReplayData(
        timeline_s=quaternion_time_s,
        rotations_world_from_rocket=rotations_world_from_rocket,
        imu_source=imu_source,
        acceleration=acceleration,
        linear_acceleration=linear_acceleration,
        imu_temperature=imu_temperature,
        angular_rate=angular_rate,
        magnetic_field=magnetic_field,
        gravity=gravity,
        pressure=pressure,
        temperature=temperature,
        barometric_altitude=barometric_altitude,
        transitions=transitions,
        launch_time_s=launch_time_s,
    )


def replay_state_at_time(transitions: list[FsmTransition], time_s: float) -> str:
    if not transitions:
        return "UNKNOWN"
    state = transitions[0].old_state
    for transition in transitions:
        if transition.time_s > time_s:
            break
        state = transition.new_state
    return state


def fsm_color(state: str) -> Any:
    """Use a deterministic state colour without maintaining a second FSM enum here."""
    palette = plt.get_cmap("tab20").colors
    return palette[sum(state.encode("utf-8")) % len(palette)]


def overlay_fsm_states(
    axis: Any,
    transitions: list[FsmTransition],
    start_s: float,
    end_s: float,
    *,
    labels: bool = True,
) -> None:
    """Draw exact transition lines and unobtrusive state intervals on a time-series axis."""
    intervals = fsm_state_intervals(transitions, start_s, end_s)
    span = end_s - start_s
    minimum_label_width = max(0.75, span * 0.08)
    for interval_start, interval_end, state in intervals:
        axis.axvspan(interval_start, interval_end, color=fsm_color(state), alpha=0.09, zorder=0)
        if labels and interval_end - interval_start >= minimum_label_width:
            axis.text(
                (interval_start + interval_end) / 2.0,
                0.98,
                state,
                transform=axis.get_xaxis_transform(),
                ha="center",
                va="top",
                fontsize=7,
                color="0.25",
                clip_on=True,
            )
    for index, transition in enumerate(transitions, 1):
        axis.axvline(transition.time_s, color="0.25", linestyle=":", linewidth=0.8, alpha=0.8, zorder=1)
        axis.text(
            transition.time_s,
            0.02 + 0.07 * (index % 2),
            f"T{index}",
            transform=axis.get_xaxis_transform(),
            ha="center",
            va="bottom",
            fontsize=6,
            color="0.25",
            clip_on=True,
        )


def finish_axis(
    axis: Any,
    title: str,
    ylabel: str,
    transitions: list[FsmTransition],
    start_s: float,
    end_s: float,
) -> None:
    axis.set_title(title)
    axis.set_xlabel("Time since first timestamp [s]")
    axis.set_ylabel(ylabel)
    axis.grid(True, alpha=0.3)
    if end_s > start_s:
        axis.set_xlim(start_s, end_s)
    overlay_fsm_states(axis, transitions, start_s, end_s)
    handles, labels = axis.get_legend_handles_labels()
    if handles:
        axis.legend(fontsize=8, ncols=min(4, len(handles)))


def plot_fields(
    axis: Any,
    grouped: dict[str, list[dict[str, Any]]],
    sources: Iterable[str],
    fields: Iterable[str],
    origin_ms: float,
    *,
    include_source: bool = True,
) -> None:
    for source in sources:
        for field in fields:
            time_s, values = numeric_series(grouped[source], field, origin_ms)
            if values.size:
                label = f"{source}: {field}" if include_source else field
                axis.plot(time_s, values, label=label, linewidth=1.1)


def timing_statistics(source_records: list[dict[str, Any]]) -> dict[str, float | int]:
    timestamps = np.asarray(
        [record["timestamp_ms"] for record in source_records if record["timestamp_ms"] is not None],
        dtype=float,
    )
    if timestamps.size < 2:
        return {"timestamped": int(timestamps.size)}

    differences = np.diff(timestamps)
    positive = differences[differences > 0]
    result: dict[str, float | int] = {
        "timestamped": int(timestamps.size),
        "zero_dt": int(np.count_nonzero(differences == 0)),
        "backwards": int(np.count_nonzero(differences < 0)),
    }
    if positive.size:
        median = float(np.median(positive))
        result.update(
            {
                "median_ms": median,
                "mean_ms": float(np.mean(positive)),
                "p95_ms": float(np.percentile(positive, 95)),
                "max_ms": float(np.max(positive)),
                "rate_hz": 1000.0 / median,
                "gaps": int(np.count_nonzero(positive > 3.0 * median)),
            }
        )
    return result


def print_report(
    path: Path,
    records: list[dict[str, Any]],
    events: list[dict[str, Any]],
    malformed: list[dict[str, Any]],
    grouped: dict[str, list[dict[str, Any]]],
) -> None:
    timestamps = [record["timestamp_ms"] for record in records if record["timestamp_ms"] is not None]
    print("\n========== FLIGHT RECORDER SUMMARY ==========")
    print(f"File:               {path}")
    print(f"Size:               {path.stat().st_size:,} bytes")
    print(f"Valid records:      {len(records):,}")
    print(f"Data records:       {len(records) - len(events):,}")
    print(f"Event records:      {len(events):,}")
    print(f"Malformed records:  {len(malformed):,}")
    print(f"Without timestamp:  {sum(record['timestamp_ms'] is None for record in records):,}")
    if timestamps:
        print(f"Timestamp range:    {min(timestamps):.0f} .. {max(timestamps):.0f} ms")
        print(f"Recorded duration:  {(max(timestamps) - min(timestamps)) / 1000.0:.3f} s")

    counts = Counter(record["source"] for record in records)
    print("\nRecord count by source:")
    for source, count in sorted(counts.items(), key=lambda item: (-item[1], item[0])):
        print(f"  {source:<22} {count:6d} ({100.0 * count / len(records):5.1f}%)")

    print("\nPer-source timing (positive intervals only):")
    for source, source_records in sorted(grouped.items()):
        stats = timing_statistics(source_records)
        if "median_ms" not in stats:
            print(f"  {source:<22} count={len(source_records):5d} timestamped={stats['timestamped']:5d}")
            continue
        print(
            f"  {source:<22} count={len(source_records):5d} "
            f"median={stats['median_ms']:8.2f} ms rate={stats['rate_hz']:7.2f} Hz "
            f"p95={stats['p95_ms']:8.2f} ms max={stats['max_ms']:8.2f} ms "
            f"gaps={stats['gaps']:3d} zero={stats['zero_dt']:3d} backwards={stats['backwards']:3d}"
        )

    if malformed:
        print("\nMalformed JSONL records:")
        for issue in malformed[:20]:
            excerpt = issue["text"]
            column = int(issue["column"])
            left = max(0, column - 61)
            excerpt = excerpt[left : left + 140]
            print(f"  line {issue['line']}, column {column}: {issue['error']}")
            print(f"    {excerpt}")
        if len(malformed) > 20:
            print(f"  ... {len(malformed) - 20} additional malformed records")

    if events:
        print("\nEvent timeline:")
        for event in events:
            timestamp = event["timestamp_ms"]
            timestamp_text = f"{timestamp:.0f} ms" if timestamp is not None else "no timestamp"
            print(f"  {timestamp_text:>12}  {event['type']:<7} {event['source']}: {event['payload'].get('message', '')}")
    print("=============================================\n")


def create_plots(
    records: list[dict[str, Any]],
    events: list[dict[str, Any]],
    malformed: list[dict[str, Any]],
    grouped: dict[str, list[dict[str, Any]]],
) -> list[tuple[str, Any]]:
    # Events can precede the first sensor sample; keep one origin for every plot lane.
    timestamps = [
        record["timestamp_ms"]
        for record in (*records, *events)
        if record["timestamp_ms"] is not None
    ]
    if not timestamps:
        return []
    origin_ms = min(timestamps)
    end_s = (max(timestamps) - origin_ms) / 1000.0
    transitions = fsm_transitions(events, origin_ms)
    plots: list[tuple[str, Any]] = []

    imu_sources = [source for source, values in grouped.items() if source_has_fields(values, "ax", "ay", "az", "qw")]
    accel_sources = [source for source, values in grouped.items() if source_has_fields(values, "acceleration_x", "acceleration_y", "acceleration_z")]
    pressure_sources = [source for source, values in grouped.items() if source_has_fields(values, "pressure", "temperature")]
    gps_sources = [source for source, values in grouped.items() if source_has_fields(values, "latitude", "longitude", "altitude")]

    # Record density, FSM state intervals, and timing quality share one time base.
    fig = plt.figure(figsize=(14, 10), constrained_layout=True)
    grid = fig.add_gridspec(3, 2, width_ratios=(4.5, 1.5), height_ratios=(3.0, 1.15, 2.0))
    record_axis = fig.add_subplot(grid[0, 0])
    fsm_axis = fig.add_subplot(grid[1, 0], sharex=record_axis)
    interval_axis = fig.add_subplot(grid[2, 0], sharex=record_axis)
    transition_axis = fig.add_subplot(grid[:, 1])
    timestamped_sources = [source for source, values in sorted(grouped.items()) if any(r["timestamp_ms"] is not None for r in values)]
    for row, source in enumerate(timestamped_sources):
        times = [(r["timestamp_ms"] - origin_ms) / 1000.0 for r in grouped[source] if r["timestamp_ms"] is not None]
        record_axis.scatter(times, np.full(len(times), row), marker="|", s=90, label=source)
        source_times = np.asarray([r["timestamp_ms"] for r in grouped[source] if r["timestamp_ms"] is not None], dtype=float)
        if source_times.size > 1:
            dt = np.diff(source_times)
            interval_axis.plot((source_times[1:] - origin_ms) / 1000.0, dt, label=source, linewidth=0.9)
    record_axis.set_yticks(range(len(timestamped_sources)), timestamped_sources)
    record_axis.set_title(f"Record timeline ({len(records)} valid, {len(malformed)} malformed)")
    record_axis.set_ylabel("Record source")
    record_axis.grid(True, axis="x", alpha=0.3)
    record_axis.set_xlim(0.0, end_s)
    fsm_axis.set_title("FSM state timeline", fontsize=10)
    fsm_axis.set_yticks([])
    fsm_axis.set_ylabel("State")
    fsm_axis.grid(True, axis="x", alpha=0.3)
    fsm_axis.set_ylim(0.0, 1.0)
    for interval_start, interval_end, state in fsm_state_intervals(transitions, 0.0, end_s):
        fsm_axis.broken_barh(
            [(interval_start, interval_end - interval_start)], (0.3, 0.42), facecolors=fsm_color(state), alpha=0.65
        )
        if interval_end - interval_start >= max(0.75, end_s * 0.06):
            fsm_axis.text((interval_start + interval_end) / 2.0, 0.51, state, ha="center", va="center", fontsize=8)
    for index, transition in enumerate(transitions, 1):
        fsm_axis.axvline(transition.time_s, color="0.15", linewidth=0.9)
        fsm_axis.text(transition.time_s, 0.08, f"T{index}", ha="center", va="bottom", fontsize=7)
    finish_axis(interval_axis, "Per-source record intervals", "Δ timestamp [ms]", transitions, 0.0, end_s)
    transition_axis.axis("off")
    transition_lines = ["FSM transitions"]
    transition_lines.extend(
        f"T{index:02d}  {transition.time_s:8.3f} s\n      {transition.old_state} → {transition.new_state}"
        for index, transition in enumerate(transitions, 1)
    )
    if len(transition_lines) == 1:
        transition_lines.append("No recorded FSM transition")
    transition_axis.text(0.0, 1.0, "\n\n".join(transition_lines), va="top", fontsize=8, family="monospace")
    plots.append(("overview", fig))

    # Acceleration and attitude, following the HIL capture views.
    if imu_sources or accel_sources:
        fig, axes = plt.subplots(4, 1, figsize=(14, 13), sharex=True, constrained_layout=True)
        for source in imu_sources:
            time_s, ax = numeric_series(grouped[source], "ax", origin_ms)
            _, ay = numeric_series(grouped[source], "ay", origin_ms)
            _, az = numeric_series(grouped[source], "az", origin_ms)
            if len(time_s) == len(ax) == len(ay) == len(az):
                axes[0].plot(time_s, ax / G0, label=f"{source}: ax")
                axes[0].plot(time_s, ay / G0, label=f"{source}: ay")
                axes[0].plot(time_s, az / G0, label=f"{source}: az")
                axes[0].plot(time_s, np.sqrt(ax**2 + ay**2 + az**2) / G0, "--", label=f"{source}: |a|")
        for source in accel_sources:
            time_s, ax = numeric_series(grouped[source], "acceleration_x", origin_ms)
            _, ay = numeric_series(grouped[source], "acceleration_y", origin_ms)
            _, az = numeric_series(grouped[source], "acceleration_z", origin_ms)
            if len(time_s) == len(ax) == len(ay) == len(az):
                axes[1].plot(time_s, ax / G0, label=f"{source}: x")
                axes[1].plot(time_s, ay / G0, label=f"{source}: y")
                axes[1].plot(time_s, az / G0, label=f"{source}: z")
                axes[1].plot(time_s, np.sqrt(ax**2 + ay**2 + az**2) / G0, "--", label=f"{source}: |a|")
        plot_fields(axes[2], grouped, imu_sources, ("lax", "lay", "laz"), origin_ms)
        plot_fields(axes[3], grouped, imu_sources, ("gx", "gy", "gz"), origin_ms)
        finish_axis(axes[0], "IMU measured acceleration", "Acceleration [g]", transitions, 0.0, end_s)
        finish_axis(axes[1], "Dedicated accelerometer", "Acceleration [g]", transitions, 0.0, end_s)
        finish_axis(axes[2], "IMU linear acceleration", "Acceleration [m/s²]", transitions, 0.0, end_s)
        finish_axis(axes[3], "IMU gravity vector", "Acceleration [m/s²]", transitions, 0.0, end_s)
        plots.append(("motion", fig))

    if imu_sources:
        fig, axes = plt.subplots(5, 1, figsize=(14, 15), sharex=True, constrained_layout=True)
        plot_fields(axes[0], grouped, imu_sources, ("ox", "oy", "oz"), origin_ms)
        plot_fields(axes[1], grouped, imu_sources, ("avx", "avy", "avz"), origin_ms)
        plot_fields(axes[2], grouped, imu_sources, ("mx", "my", "mz"), origin_ms)
        for source in imu_sources:
            plot_fields(axes[3], grouped, [source], ("qw", "qx", "qy", "qz"), origin_ms)
            arrays = [numeric_series(grouped[source], key, origin_ms)[1] for key in ("qw", "qx", "qy", "qz")]
            time_s = numeric_series(grouped[source], "qw", origin_ms)[0]
            if arrays and all(len(values) == len(time_s) for values in arrays):
                axes[3].plot(time_s, np.sqrt(sum(values**2 for values in arrays)), "--", label=f"{source}: |q|")
        plot_fields(axes[4], grouped, imu_sources, ("csys", "cgyro", "caccel", "cmag"), origin_ms)
        finish_axis(axes[0], "Euler attitude", "Angle [deg]", transitions, 0.0, end_s)
        finish_axis(axes[1], "Angular velocity", "Angular velocity [rad/s]", transitions, 0.0, end_s)
        finish_axis(axes[2], "Magnetometer", "Magnetic field [µT]", transitions, 0.0, end_s)
        finish_axis(axes[3], "Orientation quaternion and norm", "Quaternion", transitions, 0.0, end_s)
        finish_axis(axes[4], "IMU calibration status", "Status [0..3]", transitions, 0.0, end_s)
        axes[4].set_yticks([0, 1, 2, 3])
        plots.append(("imu", fig))

    if pressure_sources or gps_sources:
        fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True, constrained_layout=True)
        plot_fields(axes[0], grouped, pressure_sources, ("pressure",), origin_ms)
        plot_fields(axes[1], grouped, pressure_sources, ("temperature",), origin_ms)
        for source in pressure_sources:
            time_s, pressure = numeric_series(grouped[source], "pressure", origin_ms)
            positive = pressure[pressure > 0]
            if positive.size:
                reference = float(np.median(positive[: min(20, positive.size)]))
                relative_altitude = 44330.0 * (1.0 - np.power(pressure / reference, 0.19029495))
                axes[2].plot(time_s, relative_altitude, label=f"{source}: pressure altitude")
        plot_fields(axes[2], grouped, gps_sources, ("altitude",), origin_ms)
        plot_fields(axes[3], grouped, imu_sources, ("te",), origin_ms)
        finish_axis(axes[0], "Barometer pressure", "Pressure [Pa]", transitions, 0.0, end_s)
        finish_axis(axes[1], "Barometer temperature", "Temperature [°C]", transitions, 0.0, end_s)
        finish_axis(axes[2], "Altitude comparison", "Altitude / relative altitude [m]", transitions, 0.0, end_s)
        finish_axis(axes[3], "IMU temperature", "Temperature [°C]", transitions, 0.0, end_s)
        plots.append(("environment", fig))

    for gps_source in gps_sources:
        time_s, latitude = numeric_series(grouped[gps_source], "latitude", origin_ms)
        _, longitude = numeric_series(grouped[gps_source], "longitude", origin_ms)
        _, altitude = numeric_series(grouped[gps_source], "altitude", origin_ms)
        if not (len(time_s) == len(latitude) == len(longitude) == len(altitude)):
            continue
        latitude_rad = np.radians(latitude)
        east = np.radians(longitude - longitude[0]) * 6_371_000.0 * np.cos(latitude_rad[0])
        north = np.radians(latitude - latitude[0]) * 6_371_000.0

        fig, axes = plt.subplots(2, 2, figsize=(13, 10), constrained_layout=True)
        axes[0, 0].plot(east, north)
        axes[0, 0].scatter(east[0], north[0], marker="o", label="START")
        axes[0, 0].scatter(east[-1], north[-1], marker="X", label="END")
        axes[0, 0].set_title(f"{gps_source} ground track")
        axes[0, 0].set_xlabel("East [m]")
        axes[0, 0].set_ylabel("North [m]")
        axes[0, 0].axis("equal")
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].legend()
        axes[0, 1].plot(time_s, altitude, label="altitude")
        finish_axis(axes[0, 1], "GPS altitude", "Altitude [m]", transitions, 0.0, end_s)
        plot_fields(axes[1, 0], grouped, [gps_source], ("ground_speed", "hdop"), origin_ms, include_source=False)
        finish_axis(axes[1, 0], "GPS speed and HDOP", "Reported value", transitions, 0.0, end_s)
        plot_fields(axes[1, 1], grouped, [gps_source], ("satellites", "fixType"), origin_ms, include_source=False)
        finish_axis(axes[1, 1], "GPS fix quality", "Count / fix type", transitions, 0.0, end_s)
        plots.append((f"gps_{gps_source}", fig))

        fig = plt.figure(figsize=(11, 8), constrained_layout=True)
        axis = fig.add_subplot(111, projection="3d")
        axis.plot(east, north, altitude, label=gps_source)
        axis.scatter(east[0], north[0], altitude[0], marker="o", s=70, label="START")
        axis.scatter(east[-1], north[-1], altitude[-1], marker="X", s=70, label="END")
        apogee = int(np.argmax(altitude))
        axis.scatter(east[apogee], north[apogee], altitude[apogee], marker="^", s=70, label="MAX GPS ALTITUDE")
        axis.set_title("3D GPS flight-recorder trajectory")
        axis.set_xlabel("East [m]")
        axis.set_ylabel("North [m]")
        axis.set_zlabel("GPS altitude [m]")
        axis.legend()
        plots.append((f"trajectory_3d_{gps_source}", fig))

    return plots


def rocket_faces(
    position_world: np.ndarray, rotation_world_from_rocket: np.ndarray, length: float
) -> list[list[np.ndarray]]:
    """Return a simple cylinder, nose cone, and three fins; rocket +Z is the nose."""
    radius = 0.13 * length
    tail_z, body_top_z, nose_z = -0.42 * length, 0.28 * length, 0.52 * length
    angles = np.linspace(0.0, 2.0 * np.pi, 13)
    tail_ring = np.column_stack((radius * np.cos(angles), radius * np.sin(angles), np.full_like(angles, tail_z)))
    top_ring = np.column_stack((radius * np.cos(angles), radius * np.sin(angles), np.full_like(angles, body_top_z)))
    nose = np.array([0.0, 0.0, nose_z])

    def world(points: np.ndarray) -> np.ndarray:
        return position_world + (rotation_world_from_rocket @ points.T).T

    faces: list[list[np.ndarray]] = []
    for index in range(len(angles) - 1):
        faces.append(list(world(np.vstack((tail_ring[index], tail_ring[index + 1], top_ring[index + 1], top_ring[index])))))
        faces.append(list(world(np.vstack((top_ring[index], top_ring[index + 1], nose)))))

    # Three simple fins are placed at the tail in the rocket body frame.
    for angle in (0.0, 2.0 * np.pi / 3.0, 4.0 * np.pi / 3.0):
        radial = np.array([np.cos(angle), np.sin(angle), 0.0])
        fin = np.vstack(
            (
                radial * radius + np.array([0.0, 0.0, tail_z]),
                radial * (2.4 * radius) + np.array([0.0, 0.0, tail_z - 0.05 * length]),
                radial * radius + np.array([0.0, 0.0, tail_z + 0.24 * length]),
            )
        )
        faces.append(list(world(fin)))
    return faces


def create_flight_replay(data: FlightReplayData, playback_speed: float = 1.0) -> tuple[Any, FuncAnimation]:
    """Create an engineering replay using real timestamps and recorded attitude only."""
    if playback_speed not in REPLAY_SPEEDS:
        raise ValueError(f"playback speed must be one of: {', '.join(map(str, REPLAY_SPEEDS))}")

    timeline = data.timeline_s
    start_s, end_s = float(timeline[0]), float(timeline[-1])
    altitude_values = data.barometric_altitude.values if data.barometric_altitude is not None else np.array([0.0])
    altitude_min, altitude_max = float(np.min(altitude_values)), float(np.max(altitude_values))
    scene_span = max(12.0, altitude_max - altitude_min, 4.0)
    rocket_length = max(2.5, scene_span * 0.08)  # A visible attitude glyph, deliberately not scale-drawn.

    fig = plt.figure(figsize=(18, 10))
    fig.subplots_adjust(left=0.05, right=0.96, top=0.94, bottom=0.26, wspace=0.35, hspace=0.60)
    grid = fig.add_gridspec(4, 3, width_ratios=(1.25, 1.25, 1.0), height_ratios=(1.0, 1.0, 1.0, 0.42))
    axis_3d = fig.add_subplot(grid[:3, :2], projection="3d")
    axis_info = fig.add_subplot(grid[0, 2])
    axis_altitude = fig.add_subplot(grid[1, 2])
    axis_motion = fig.add_subplot(grid[2, 2])
    axis_fsm = fig.add_subplot(grid[3, :])

    axis_3d.set_title("Flight replay — rocket attitude and body-frame vectors")
    axis_3d.set_xlabel("WORLD +X [m]")
    axis_3d.set_ylabel("WORLD +Y [m]")
    axis_3d.set_zlabel("WORLD +Z / altitude [m]")
    axis_3d.set_xlim(-scene_span * 0.25, scene_span * 0.25)
    axis_3d.set_ylim(-scene_span * 0.25, scene_span * 0.25)
    axis_3d.set_zlim(altitude_min - rocket_length, altitude_max + rocket_length)
    axis_3d.set_box_aspect((1.0, 1.0, max(1.0, (altitude_max - altitude_min) / (scene_span * 0.5))))
    axis_3d.text2D(0.02, 0.02, "Rocket is an attitude glyph; vertical position is barometric altitude only.", transform=axis_3d.transAxes, fontsize=8)

    # ``barometric_altitude`` is relative to the median of the first valid
    # pressure samples, which represent the stationary ready-for-launch pad.
    # The ground plane therefore stays at that same base-pressure altitude.
    ground_extent = scene_span * 0.28
    ground_x, ground_y = np.meshgrid(
        np.array([-ground_extent, ground_extent]), np.array([-ground_extent, ground_extent])
    )
    axis_3d.plot_surface(
        ground_x,
        ground_y,
        np.zeros((2, 2)),
        color="tab:olive",
        alpha=0.22,
        shade=False,
    )
    axis_3d.text(ground_extent, ground_extent, 0.0, "Base-pressure ground plane", fontsize=8, color="0.25")

    # Launch-pad WORLD axes remain fixed.  The rocket model and body vectors rotate independently.
    world_axis_length = 0.24 * scene_span
    for vector, color, label in (
        (np.array([world_axis_length, 0.0, 0.0]), "tab:red", "WORLD +X"),
        (np.array([0.0, world_axis_length, 0.0]), "tab:green", "WORLD +Y"),
        (np.array([0.0, 0.0, world_axis_length]), "tab:blue", "WORLD +Z"),
    ):
        axis_3d.quiver(0.0, 0.0, 0.0, *vector, color=color, linewidth=1.6, arrow_length_ratio=0.12)
        axis_3d.text(*vector, label, color=color, fontsize=8)
    axis_3d.quiver(0.0, 0.0, 0.0, 0.0, 0.0, -1.0, length=rocket_length * 0.7, color="0.25", arrow_length_ratio=0.15)
    axis_3d.text(0.0, 0.0, -rocket_length * 0.75, "WORLD gravity", fontsize=8)

    if data.barometric_altitude is not None:
        axis_3d.plot(
            np.zeros_like(data.barometric_altitude.values),
            np.zeros_like(data.barometric_altitude.values),
            data.barometric_altitude.values,
            color="0.65",
            linewidth=1.0,
            label="Barometric vertical path",
        )

    initial_altitude = sample_and_hold(data.barometric_altitude, start_s)
    initial_position = np.array([0.0, 0.0, float(initial_altitude) if initial_altitude is not None else 0.0])
    rocket = Poly3DCollection(
        rocket_faces(initial_position, data.rotations_world_from_rocket[0], rocket_length),
        facecolors="0.70",
        edgecolors="0.15",
        linewidths=0.45,
        alpha=0.95,
    )
    axis_3d.add_collection3d(rocket)
    body_axes = []
    body_axis_labels = []
    for color, label in (("tab:red", "+X ROCKET"), ("tab:green", "+Y ROCKET"), ("tab:blue", "+Z ROCKET / NOSE")):
        line, = axis_3d.plot([], [], [], color=color, linewidth=2.0)
        body_axes.append(line)
        body_axis_labels.append(axis_3d.text(0.0, 0.0, 0.0, label, color=color, fontsize=8))

    vector_specs = (
        ("acceleration", data.acceleration, "magenta", G0, "acceleration"),
        ("gravity", data.gravity, "tab:cyan", G0, "gravity"),
        ("angular_rate", data.angular_rate, "tab:orange", 1.0, "angular rate"),
        ("magnetic_field", data.magnetic_field, "tab:purple", 50.0, "magnetic field"),
    )
    vector_offsets = {
        "acceleration": np.array([0.0, 0.0, 0.0]),
        "gravity": np.array([0.17 * rocket_length, 0.0, 0.0]),
        "angular_rate": np.array([0.0, 0.17 * rocket_length, 0.0]),
        "magnetic_field": np.array([-0.17 * rocket_length, 0.0, 0.0]),
    }
    vector_artists: dict[str, Any] = {name: None for name, *_ in vector_specs}
    vector_labels = {name: axis_3d.text(0.0, 0.0, 0.0, "", fontsize=8, color=color) for name, _, color, _, _ in vector_specs}

    axis_info.axis("off")
    status_text = axis_info.text(0.0, 1.0, "", va="top", family="monospace", fontsize=9)

    altitude_cursor = pressure_cursor = motion_cursor = angular_motion_cursor = None
    if data.barometric_altitude is not None:
        axis_altitude.plot(data.barometric_altitude.time_s, data.barometric_altitude.values, color="tab:blue", linewidth=1.0)
        altitude_cursor = axis_altitude.axvline(start_s, color="0.15", linestyle=":")
        axis_altitude.set_title("Barometric altitude (derived from recorded pressure)")
        axis_altitude.set_ylabel("Altitude relative to log start [m]")
        axis_altitude.grid(True, alpha=0.3)
    if data.pressure is not None:
        pressure_axis = axis_altitude.twinx()
        pressure_axis.plot(data.pressure.time_s, data.pressure.values, color="tab:gray", alpha=0.55, linewidth=0.8)
        pressure_axis.set_ylabel("Pressure [Pa]")
        pressure_cursor = pressure_axis.axvline(start_s, color="0.15", linestyle=":")
    if data.acceleration is not None:
        acceleration_magnitude = np.linalg.norm(data.acceleration.values, axis=1) / G0
        axis_motion.plot(data.acceleration.time_s, acceleration_magnitude, color="magenta", label="|acceleration| [g]")
    else:
        acceleration_magnitude = None
    if data.angular_rate is not None:
        angular_rate_magnitude = np.linalg.norm(data.angular_rate.values, axis=1)
        angular_motion_axis = axis_motion.twinx()
        angular_motion_axis.plot(data.angular_rate.time_s, angular_rate_magnitude, color="tab:orange", label="|angular rate| [rad/s]")
        angular_motion_axis.set_ylabel("Angular rate [rad/s]")
        angular_motion_cursor = angular_motion_axis.axvline(start_s, color="0.15", linestyle=":")
    else:
        angular_rate_magnitude = None
    motion_cursor = axis_motion.axvline(start_s, color="0.15", linestyle=":")
    axis_motion.set_title("Recorded acceleration and angular-rate magnitudes")
    axis_motion.set_ylabel("Acceleration [g]")
    axis_motion.grid(True, alpha=0.3)
    axis_motion.legend(fontsize=8)

    axis_fsm.set_title("FSM timeline")
    axis_fsm.set_ylim(0.0, 1.0)
    axis_fsm.set_yticks([])
    axis_fsm.set_xlabel("Log time [s]")
    axis_fsm.grid(True, axis="x", alpha=0.3)
    for interval_start, interval_end, state in fsm_state_intervals(data.transitions, start_s, end_s):
        axis_fsm.broken_barh([(interval_start, interval_end - interval_start)], (0.28, 0.46), facecolors=fsm_color(state), alpha=0.7)
        if interval_end - interval_start >= max(0.75, (end_s - start_s) * 0.05):
            axis_fsm.text((interval_start + interval_end) / 2.0, 0.51, state, ha="center", va="center", fontsize=8)
    for transition in data.transitions:
        axis_fsm.axvline(transition.time_s, color="0.2", linewidth=0.8)
    fsm_cursor = axis_fsm.axvline(start_s, color="black", linewidth=2.0)
    axis_fsm.set_xlim(start_s, end_s)

    play_axis = fig.add_axes([0.05, 0.045, 0.065, 0.05])
    pause_axis = fig.add_axes([0.125, 0.045, 0.065, 0.05])
    restart_axis = fig.add_axes([0.20, 0.045, 0.07, 0.05])
    slider_axis = fig.add_axes([0.32, 0.135, 0.44, 0.03])
    speed_axis = fig.add_axes([0.80, 0.015, 0.14, 0.16])
    play_button, pause_button, restart_button = Button(play_axis, "Play"), Button(pause_axis, "Pause"), Button(restart_axis, "Reset")
    time_slider = Slider(slider_axis, "Replay time [s]", start_s, end_s, valinit=start_s, valfmt="%.3f")
    speed_labels = [f"{speed:g}x" for speed in REPLAY_SPEEDS]
    speed_radio = RadioButtons(speed_axis, speed_labels, active=REPLAY_SPEEDS.index(playback_speed))

    current_time_s = start_s
    playing = False
    speed = playback_speed
    updating_slider = False
    last_tick = time.monotonic()

    def update(current_s: float, *, update_slider: bool = False) -> None:
        nonlocal current_time_s, updating_slider
        current_time_s = min(max(current_s, start_s), end_s)
        frame_index = int(np.searchsorted(timeline, current_time_s, side="right") - 1)
        frame_index = max(0, min(frame_index, len(timeline) - 1))
        rotation = data.rotations_world_from_rocket[frame_index]
        altitude = sample_and_hold(data.barometric_altitude, current_time_s)
        position = np.array([0.0, 0.0, float(altitude) if altitude is not None else 0.0])
        rocket.set_verts(rocket_faces(position, rotation, rocket_length))

        for axis_index, (line, label) in enumerate(zip(body_axes, body_axis_labels)):
            endpoint = position + rotation[:, axis_index] * rocket_length * 0.62
            line.set_data_3d([position[0], endpoint[0]], [position[1], endpoint[1]], [position[2], endpoint[2]])
            label.set_position_3d(endpoint)

        telemetry_lines = [
            f"Attitude source: recorded quaternion ({data.imu_source})",
            "WORLD is fixed; initial ROCKET frame is WORLD.",
            f"Log time:      {current_time_s:8.3f} s",
            f"Since launch:  {current_time_s - data.launch_time_s:8.3f} s" if data.launch_time_s is not None else "Since launch:  unavailable",
            f"FSM:           {replay_state_at_time(data.transitions, current_time_s)}",
            "Vector values: ROCKET frame; arrows: WORLD orientation.",
            "'acceleration' is recorded BNO total acceleration, not inertial acceleration.",
            "",
        ]
        previous_transition = next(
            (transition for transition in reversed(data.transitions) if transition.time_s <= current_time_s), None
        )
        if previous_transition is not None:
            telemetry_lines.append(
                f"Last transition: {previous_transition.old_state} → {previous_transition.new_state} at {previous_transition.time_s:.3f} s"
            )
        for name, stream, color, reference, label in vector_specs:
            value_imu = sample_and_hold(stream, current_time_s)
            if value_imu is None:
                vector_labels[name].set_text("")
                continue
            value_rocket = transform_imu_to_rocket(value_imu)
            value_world = rotation @ value_rocket
            magnitude = float(np.linalg.norm(value_rocket))
            arrow_origin = position + rotation @ (IMU_ORIGIN_ROCKET_M + vector_offsets[name])
            if vector_artists[name] is not None:
                vector_artists[name].remove()
                vector_artists[name] = None
            if magnitude > 1e-9:
                arrow_length = rocket_length * 0.55 * min(magnitude / reference, 2.0)
                direction = value_world / magnitude
                vector_artists[name] = axis_3d.quiver(*arrow_origin, *direction, length=arrow_length, color=color, linewidth=2.0, arrow_length_ratio=0.18)
                vector_labels[name].set_position_3d(arrow_origin + direction * arrow_length)
                vector_labels[name].set_text(label)
            telemetry_lines.append(
                f"{label:<14} ({value_rocket[0]:7.2f}, {value_rocket[1]:7.2f}, {value_rocket[2]:7.2f}) {stream.unit}"
            )
        linear_acceleration = sample_and_hold(data.linear_acceleration, current_time_s)
        if linear_acceleration is not None:
            linear_rocket = transform_imu_to_rocket(linear_acceleration)
            telemetry_lines.append(
                f"linear accel   ({linear_rocket[0]:7.2f}, {linear_rocket[1]:7.2f}, {linear_rocket[2]:7.2f}) m/s²"
            )
        for label, stream in (
            ("Pressure", data.pressure),
            ("Baro temperature", data.temperature),
            ("Baro altitude", data.barometric_altitude),
            ("IMU temperature", data.imu_temperature),
        ):
            value = sample_and_hold(stream, current_time_s)
            if value is not None:
                telemetry_lines.append(f"{label:<14} {float(value):10.3f} {stream.unit}")
        status_text.set_text("\n".join(telemetry_lines))

        for cursor in (altitude_cursor, pressure_cursor, motion_cursor, angular_motion_cursor, fsm_cursor):
            if cursor is not None:
                cursor.set_xdata([current_time_s, current_time_s])
        if update_slider:
            updating_slider = True
            time_slider.set_val(current_time_s)
            updating_slider = False
        fig.canvas.draw_idle()

    def on_play(_event: Any) -> None:
        nonlocal playing, last_tick
        playing, last_tick = True, time.monotonic()
        animation.event_source.start()

    def on_pause(_event: Any) -> None:
        nonlocal playing
        playing = False

    def on_restart(_event: Any) -> None:
        nonlocal playing
        playing = False
        update(start_s, update_slider=True)

    def on_slider(value: float) -> None:
        nonlocal playing
        if not updating_slider:
            playing = False
            update(value)

    def on_speed(label: str) -> None:
        nonlocal speed
        speed = float(label[:-1])

    def on_timer(_frame: int) -> None:
        nonlocal playing, last_tick
        now = time.monotonic()
        elapsed = now - last_tick
        last_tick = now
        if playing:
            next_time = current_time_s + elapsed * speed
            if next_time >= end_s:
                playing = False
                next_time = end_s
            update(next_time, update_slider=True)

    play_button.on_clicked(on_play)
    pause_button.on_clicked(on_pause)
    restart_button.on_clicked(on_restart)
    time_slider.on_changed(on_slider)
    speed_radio.on_clicked(on_speed)
    animation = FuncAnimation(fig, on_timer, interval=40, cache_frame_data=False)
    animation.event_source.start()
    # Matplotlib holds axes, not widget objects.  Keep controls alive with the
    # figure or the visible buttons/slider lose their registered callbacks.
    fig._flight_replay_animation = animation
    fig._flight_replay_controls = (play_button, pause_button, restart_button, time_slider, speed_radio)
    update(start_s)
    return fig, animation


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Analyze a flight-recorder JSONL telemetry file.")
    parser.add_argument(
        "log",
        nargs="?",
        type=Path,
        default=Path("flash_logs/flight_telemetry_from_web.jsonl"),
        help="JSONL file to inspect (default: flash_logs/flight_telemetry_from_web.jsonl)",
    )
    parser.add_argument("--save-dir", type=Path, help="Save every figure as a PNG in this directory.")
    parser.add_argument("--no-show", action="store_true", help="Do not open interactive plot windows.")
    parser.add_argument("--dpi", type=int, default=150, help="PNG resolution when --save-dir is used.")
    parser.add_argument(
        "--replay",
        action="store_true",
        help="Open an interactive 3D replay using recorded quaternion and sensor data.",
    )
    parser.add_argument(
        "--replay-speed",
        type=float,
        default=1.0,
        choices=REPLAY_SPEEDS,
        help="Initial Flight Replay speed multiplier (default: 1x).",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if not args.log.is_file():
        print(f"Error: telemetry file not found: {args.log}")
        return 2

    records, malformed = load_records(args.log)
    datalog, events = split_records(records)
    grouped = group_records(datalog)
    print_report(args.log, records, events, malformed, grouped)
    plots = create_plots(datalog, events, malformed, grouped)
    if args.replay:
        replay = build_flight_replay(datalog, events, grouped)
        if replay is not None:
            replay_figure, _animation = create_flight_replay(replay, args.replay_speed)
            plots.append(("flight_replay", replay_figure))

    if args.save_dir:
        args.save_dir.mkdir(parents=True, exist_ok=True)
        for name, figure in plots:
            output = args.save_dir / f"{name}.png"
            figure.savefig(output, dpi=args.dpi, bbox_inches="tight")
            print(f"Saved {output}")

    if plots and not args.no_show:
        plt.show()
    else:
        plt.close("all")
    return 0 if records else 1


if __name__ == "__main__":
    raise SystemExit(main())
