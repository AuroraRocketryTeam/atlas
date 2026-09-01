#!/usr/bin/env python3
"""Inspect JSONL telemetry written by the on-board flight recorder."""

from __future__ import annotations

import argparse
import json
import math
import re
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Iterable

import matplotlib.pyplot as plt
import numpy as np


G0 = 9.80665
LEGACY_FSM_RE = re.compile(r"FSM transition t=(\d+) ms:\s*(\S+)\s*->\s*(\S+)")
FSM_RE = re.compile(r"(?:FSM transition t=\d+ ms:\s*)?(\S+)\s*->\s*(\S+)")


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


def source_has_fields(source_records: list[dict[str, Any]], *fields: str) -> bool:
    present = {key for record in source_records for key in record["payload"]}
    return all(field in present for field in fields)


def event_times(records: list[dict[str, Any]], origin_ms: float) -> list[tuple[float, str]]:
    events: list[tuple[float, str]] = []
    for record in records:
        if record["timestamp_ms"] is None:
            continue
        if record["fsm"]:
            old_state, new_state = record["fsm"]
            events.append(((record["timestamp_ms"] - origin_ms) / 1000.0, f"{old_state} → {new_state}"))
        else:
            message = str(record["payload"].get("message", ""))
            if message:
                events.append(
                    (
                        (record["timestamp_ms"] - origin_ms) / 1000.0,
                        f"{record['type']}: {record['source']}: {message}",
                    )
                )
    return events


def mark_events(axis: Any, events: list[tuple[float, str]], *, annotate: bool = False) -> None:
    for index, (time_s, label) in enumerate(events):
        axis.axvline(time_s, color="0.45", linestyle=":", linewidth=0.8, alpha=0.65)
        if annotate:
            axis.annotate(
                label,
                xy=(time_s, 1.0),
                xycoords=("data", "axes fraction"),
                xytext=(3, -4 - 13 * (index % 3)),
                textcoords="offset points",
                rotation=90,
                va="top",
                fontsize=7,
            )


def finish_axis(axis: Any, title: str, ylabel: str, events: list[tuple[float, str]]) -> None:
    axis.set_title(title)
    axis.set_xlabel("Time since first timestamp [s]")
    axis.set_ylabel(ylabel)
    axis.grid(True, alpha=0.3)
    mark_events(axis, events)
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
    timestamps = [record["timestamp_ms"] for record in records if record["timestamp_ms"] is not None]
    if not timestamps:
        return []
    origin_ms = min(timestamps)
    event_markers = event_times(events, origin_ms)
    plots: list[tuple[str, Any]] = []

    imu_sources = [source for source, values in grouped.items() if source_has_fields(values, "ax", "ay", "az", "qw")]
    accel_sources = [source for source, values in grouped.items() if source_has_fields(values, "acceleration_x", "acceleration_y", "acceleration_z")]
    pressure_sources = [source for source, values in grouped.items() if source_has_fields(values, "pressure", "temperature")]
    gps_sources = [source for source, values in grouped.items() if source_has_fields(values, "latitude", "longitude", "altitude")]

    # Record density and timing quality.
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), constrained_layout=True)
    timestamped_sources = [source for source, values in sorted(grouped.items()) if any(r["timestamp_ms"] is not None for r in values)]
    for row, source in enumerate(timestamped_sources):
        times = [(r["timestamp_ms"] - origin_ms) / 1000.0 for r in grouped[source] if r["timestamp_ms"] is not None]
        axes[0].scatter(times, np.full(len(times), row), marker="|", s=90, label=source)
        source_times = np.asarray([r["timestamp_ms"] for r in grouped[source] if r["timestamp_ms"] is not None], dtype=float)
        if source_times.size > 1:
            dt = np.diff(source_times)
            axes[1].plot((source_times[1:] - origin_ms) / 1000.0, dt, label=source, linewidth=0.9)
    axes[0].set_yticks(range(len(timestamped_sources)), timestamped_sources)
    axes[0].set_title(f"Record timeline ({len(records)} valid, {len(malformed)} malformed)")
    axes[0].set_xlabel("Time since first timestamp [s]")
    axes[0].grid(True, axis="x", alpha=0.3)
    mark_events(axes[0], event_markers, annotate=True)
    finish_axis(axes[1], "Per-source record intervals", "Δ timestamp [ms]", event_markers)
    counts = Counter(record["source"] for record in records)
    axes[2].bar(list(counts), list(counts.values()))
    axes[2].set_title("Record count by source")
    axes[2].set_ylabel("Records")
    axes[2].tick_params(axis="x", rotation=25)
    axes[2].grid(True, axis="y", alpha=0.3)
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
        finish_axis(axes[0], "IMU measured acceleration", "Acceleration [g]", event_markers)
        finish_axis(axes[1], "Dedicated accelerometer", "Acceleration [g]", event_markers)
        finish_axis(axes[2], "IMU linear acceleration", "Acceleration [m/s²]", event_markers)
        finish_axis(axes[3], "IMU gravity vector", "Acceleration [m/s²]", event_markers)
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
        finish_axis(axes[0], "Euler attitude", "Angle [deg]", event_markers)
        finish_axis(axes[1], "Angular velocity", "Angular velocity [rad/s]", event_markers)
        finish_axis(axes[2], "Magnetometer", "Magnetic field [µT]", event_markers)
        finish_axis(axes[3], "Orientation quaternion and norm", "Quaternion", event_markers)
        finish_axis(axes[4], "IMU calibration status", "Status [0..3]", event_markers)
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
        finish_axis(axes[0], "Barometer pressure", "Pressure [stored units]", event_markers)
        finish_axis(axes[1], "Barometer temperature", "Temperature [°C]", event_markers)
        finish_axis(axes[2], "Altitude comparison", "Altitude / relative altitude [m]", event_markers)
        finish_axis(axes[3], "IMU temperature", "Temperature [°C]", event_markers)
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
        finish_axis(axes[0, 1], "GPS altitude", "Altitude [m]", event_markers)
        plot_fields(axes[1, 0], grouped, [gps_source], ("ground_speed", "hdop"), origin_ms, include_source=False)
        finish_axis(axes[1, 0], "GPS speed and HDOP", "Reported value", event_markers)
        plot_fields(axes[1, 1], grouped, [gps_source], ("satellites", "fixType"), origin_ms, include_source=False)
        finish_axis(axes[1, 1], "GPS fix quality", "Count / fix type", event_markers)
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
