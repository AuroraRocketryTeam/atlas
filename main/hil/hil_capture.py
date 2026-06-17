"""
hil_capture.py

Utilities to save, load, and plot HIL captures produced by hil_rocketpy.py.

Typical usage from hil_rocketpy.py:

    from hil_capture import create_capture_file, save_hil_capture, plot_hil_log

    capture_file = create_capture_file(BASE_DIR / "hil_captures")

    ...

    save_hil_capture(
        filename=capture_file,
        hil_log=hil_log,
        hil_events=hil_events,
        metadata={
            "sampling_rate_hz": sampling_rate,
            "rocket": rocket_model,
        },
    )

    plot_hil_log(hil_log, hil_events)

Typical direct usage:

    python hil_capture.py hil_captures/hil_capture_2026-05-18_12-00-00.json

Optional:

    python hil_capture.py hil_captures/run.json --no-show
"""

from __future__ import annotations

import argparse
import json
from datetime import datetime
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np


# ----------------------------------------------------------------------
# CONSTANTS
# ----------------------------------------------------------------------

G0 = 9.80665

# Keep this local so hil_capture.py can plot saved captures without importing
# hil_communication.py. These names mirror RocketState / hil_communication.
FSM_STATE_ORDER = {
    "INACTIVE": 0,
    "CALIBRATING": 1,
    "READY_FOR_LAUNCH": 2,
    "LAUNCH": 3,
    "ACCELERATED_FLIGHT": 4,
    "BALLISTIC_FLIGHT": 5,
    "APOGEE": 6,
    "STABILIZATION": 7,
    "DECELERATION": 8,
    "LANDING": 9,
    "RECOVERED": 10,
}

FSM_STATE_MARKERS = ["o", "s", "^", "D", "v", "P", "X", "*", "h", "<", ">"]


# ----------------------------------------------------------------------
# JSON SAVE / LOAD API
# ----------------------------------------------------------------------

def create_capture_file(
    output_dir: str | Path,
    *,
    prefix: str = "hil_capture",
    suffix: str = ".json",
) -> Path:
    """
    Create a timestamped capture file path.

    This does not write the file. It only creates the directory and returns
    a path such as:

        hil_captures/hil_capture_2026-05-18_12-00-00.json
    """
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    return output_dir / f"{prefix}_{timestamp}{suffix}"


def make_json_safe(obj: Any) -> Any:
    """
    Recursively convert numpy/scalar objects to JSON-safe Python types.
    """
    if isinstance(obj, dict):
        return {str(k): make_json_safe(v) for k, v in obj.items()}

    if isinstance(obj, list):
        return [make_json_safe(v) for v in obj]

    if isinstance(obj, tuple):
        return [make_json_safe(v) for v in obj]

    if isinstance(obj, np.ndarray):
        return obj.tolist()

    if isinstance(obj, np.integer):
        return int(obj)

    if isinstance(obj, np.floating):
        return float(obj)

    if isinstance(obj, np.bool_):
        return bool(obj)

    return obj


def save_hil_capture(
    filename: str | Path,
    hil_log: dict[str, list[Any]],
    hil_events: dict[str, list[Any]] | None = None,
    metadata: dict[str, Any] | None = None,
) -> Path:
    """
    Save the HIL data stream and event markers to a JSON file.
    """
    filename = Path(filename)
    filename.parent.mkdir(parents=True, exist_ok=True)

    capture = {
        "metadata": metadata or {},
        "hil_log": hil_log,
        "hil_events": hil_events or {},
    }

    with filename.open("w", encoding="utf-8") as f:
        json.dump(make_json_safe(capture), f, indent=2)

    print(f"[SAVE] HIL capture saved to: {filename}")
    return filename


def load_hil_capture(
    filename: str | Path,
) -> tuple[dict[str, list[Any]], dict[str, list[Any]], dict[str, Any]]:
    """
    Load a previously saved HIL capture JSON.
    """
    filename = Path(filename)

    with filename.open("r", encoding="utf-8") as f:
        capture = json.load(f)

    if "hil_log" not in capture:
        raise ValueError(f"Invalid capture file: missing 'hil_log': {filename}")

    hil_log = capture["hil_log"]
    hil_events = capture.get("hil_events", {})
    metadata = capture.get("metadata", {})

    print(f"[LOAD] HIL capture loaded from: {filename}")

    if metadata:
        print("\n========== CAPTURE METADATA ==========")
        for key, value in metadata.items():
            print(f"{key}: {value}")
        print("======================================\n")

    return hil_log, hil_events, metadata


# ----------------------------------------------------------------------
# NUMERIC / EVENT HELPERS
# ----------------------------------------------------------------------

def _as_array(log: dict[str, list[Any]], key: str) -> np.ndarray:
    if key not in log:
        raise KeyError(f"Missing key in hil_log: {key}")

    return np.asarray(log[key], dtype=float)


def _as_optional_array(log: dict[str, list[Any]], key: str, length: int) -> np.ndarray:
    if key not in log:
        return np.full(length, np.nan, dtype=float)

    arr = np.asarray(log[key], dtype=float)
    if len(arr) != length:
        return np.full(length, np.nan, dtype=float)

    return arr


def _lat_lon_to_local_meters(
    lat_deg: np.ndarray,
    lon_deg: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Convert latitude/longitude to a local tangent-plane approximation.

    Output:
        x = east displacement [m]
        y = north displacement [m]
    """
    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)

    lat0 = lat[0]
    lon0 = lon[0]

    earth_radius = 6_371_000.0

    x = earth_radius * (lon - lon0) * np.cos(lat0)
    y = earth_radius * (lat - lat0)

    return x, y


def _nearest_sample_index(sim_time_s: np.ndarray, target_sim_time_s: float) -> int:
    return int(np.argmin(np.abs(sim_time_s - target_sim_time_s)))


def _sanitize_event_times(raw_events: Any) -> list[float]:
    if raw_events is None:
        return []

    out: list[float] = []

    for item in raw_events:
        try:
            out.append(float(item))
        except (TypeError, ValueError):
            continue

    return out


def _sanitize_airbrake_events(raw_events: Any) -> list[tuple[float, float]]:
    if raw_events is None:
        return []

    out: list[tuple[float, float]] = []

    for item in raw_events:
        if isinstance(item, (list, tuple)) and len(item) >= 2:
            try:
                out.append((float(item[0]), float(item[1])))
            except (TypeError, ValueError):
                continue

    return out


def _sanitize_fsm_events(raw_events: Any) -> list[tuple[float, str]]:
    """
    Accept current and likely historical FSM event formats:

      - [(time, "STATE"), ...]
      - [[time, "STATE"], ...]
      - [{"time": t, "state": "STATE"}, ...]
      - [{"t": t, "name": "STATE"}, ...]
    """
    if raw_events is None:
        return []

    out: list[tuple[float, str]] = []

    for item in raw_events:
        event_t: Any = None
        state: Any = None

        if isinstance(item, dict):
            event_t = item.get("time", item.get("t", item.get("sim_time")))
            state = item.get("state", item.get("name", item.get("fsm_state")))
        elif isinstance(item, (list, tuple)) and len(item) >= 2:
            event_t = item[0]
            state = item[1]

        try:
            event_t_f = float(event_t)
        except (TypeError, ValueError):
            continue

        if state is None:
            continue

        out.append((event_t_f, str(state)))

    # Keep chronological order even if the source list was somehow unordered.
    out.sort(key=lambda x: x[0])
    return out


def _fsm_state_to_y(state: str, state_to_y: dict[str, int]) -> int:
    if state not in state_to_y:
        state_to_y[state] = len(state_to_y)
    return state_to_y[state]


def _event_marker_for_state(state: str) -> str:
    order = FSM_STATE_ORDER.get(state)
    if order is None:
        order = abs(hash(state))
    return FSM_STATE_MARKERS[order % len(FSM_STATE_MARKERS)]


# ----------------------------------------------------------------------
# PLOTTING HELPERS
# ----------------------------------------------------------------------

def _mark_event_lines(ax, event_times: list[float], label: str, *, linestyle: str = "--") -> None:
    first = True

    for event_t in event_times:
        ax.axvline(
            event_t,
            linestyle=linestyle,
            linewidth=1,
            label=label if first else None,
        )
        first = False


def _mark_fsm_event_lines(ax, fsm_events: list[tuple[float, str]]) -> None:
    first = True

    for event_t, state in fsm_events:
        ax.axvline(
            event_t,
            linestyle=":",
            linewidth=1,
            alpha=0.7,
            label="FSM transition" if first else None,
        )
        first = False


def _mark_event_points_2d(
    ax,
    sim_time_s: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    event_times: list[float],
    label: str,
    marker: str,
) -> None:
    first = True

    for event_t in event_times:
        idx = _nearest_sample_index(sim_time_s, event_t)
        ax.scatter(
            x[idx],
            y[idx],
            s=70,
            marker=marker,
            label=label if first else None,
        )
        first = False


def _mark_fsm_points_2d(
    ax,
    sim_time_s: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    fsm_events: list[tuple[float, str]],
    *,
    annotate: bool = True,
) -> None:
    first = True

    for event_t, state in fsm_events:
        idx = _nearest_sample_index(sim_time_s, event_t)
        ax.scatter(
            x[idx],
            y[idx],
            s=55,
            marker=_event_marker_for_state(state),
            label="FSM state" if first else None,
        )
        if annotate:
            ax.annotate(
                f"{state}\nt={event_t:.2f}s",
                xy=(x[idx], y[idx]),
                xytext=(6, 6),
                textcoords="offset points",
                fontsize=8,
            )
        first = False


def _mark_event_points_3d(
    ax,
    sim_time_s: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    event_times: list[float],
    label: str,
    marker: str,
) -> None:
    first = True

    for event_t in event_times:
        idx = _nearest_sample_index(sim_time_s, event_t)

        ax.scatter(
            x[idx],
            y[idx],
            z[idx],
            s=90,
            marker=marker,
            label=label if first else None,
        )
        ax.text(
            x[idx],
            y[idx],
            z[idx],
            f" {label}\nt={event_t:.2f}s",
        )

        first = False


def _mark_fsm_points_3d(
    ax,
    sim_time_s: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    fsm_events: list[tuple[float, str]],
) -> None:
    first = True

    for event_t, state in fsm_events:
        idx = _nearest_sample_index(sim_time_s, event_t)
        ax.scatter(
            x[idx],
            y[idx],
            z[idx],
            s=65,
            marker=_event_marker_for_state(state),
            label="FSM state" if first else None,
        )
        ax.text(
            x[idx],
            y[idx],
            z[idx],
            f" {state}\nt={event_t:.2f}s",
        )
        first = False


def _set_legend_if_needed(ax) -> None:
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        # De-duplicate labels while preserving order.
        unique: dict[str, Any] = {}
        for handle, label in zip(handles, labels):
            if label and label not in unique:
                unique[label] = handle
        ax.legend(unique.values(), unique.keys())


# ----------------------------------------------------------------------
# MAIN ANALYSIS / PLOT API
# ----------------------------------------------------------------------

def plot_hil_log(
    hil_log: dict[str, list[Any]],
    hil_events: dict[str, list[Any]] | None = None,
    *,
    show: bool = True,
) -> None:
    """
    Plot a live or saved HIL capture.

    Core plots:
      - mandatory 3D trajectory with event markers and timing
      - altitude and barometer pressure together
      - GPS ground track
      - acceleration payload
      - FSM state timeline
      - airbrakes command, when present

    FSM events are expected in hil_events["fsm_state"] as (time, state_name),
    matching hil_rocketpy.py's current capture format.
    """
    if hil_events is None:
        hil_events = {}

    sim_time_s = _as_array(hil_log, "sim_time_s")

    if len(sim_time_s) == 0:
        print("[PLOT] No HIL samples logged.")
        return

    seq = _as_array(hil_log, "seq")

    accel_x_m_s2 = _as_array(hil_log, "accel_x_m_s2")
    accel_y_m_s2 = _as_array(hil_log, "accel_y_m_s2")
    accel_z_m_s2 = _as_array(hil_log, "accel_z_m_s2")
    pressure_pa = _as_array(hil_log, "pressure_pa")
    temperature_k = _as_optional_array(hil_log, "temperature_k", len(sim_time_s))
    latitude_deg = _as_array(hil_log, "latitude_deg")
    longitude_deg = _as_array(hil_log, "longitude_deg")
    altitude_m = _as_array(hil_log, "altitude_m")

    gps_x, gps_y = _lat_lon_to_local_meters(latitude_deg, longitude_deg)

    accel_x_g = accel_x_m_s2 / G0
    accel_y_g = accel_y_m_s2 / G0
    accel_z_g = accel_z_m_s2 / G0
    accel_norm_g = np.sqrt(accel_x_m_s2**2 + accel_y_m_s2**2 + accel_z_m_s2**2) / G0

    drogue_times = _sanitize_event_times(hil_events.get("open_drogue", []))
    main_times = _sanitize_event_times(hil_events.get("open_main", []))
    airbrake_events = _sanitize_airbrake_events(hil_events.get("airbrakes", []))
    fsm_events = _sanitize_fsm_events(hil_events.get("fsm_state", []))

    apogee_idx = int(np.argmax(altitude_m))
    sample_periods_s = np.diff(sim_time_s)

    print("\n========== HIL DATA SUMMARY ==========")
    print(f"Samples sent:        {len(sim_time_s)}")
    print(f"First seq:           {seq[0]:.0f}")
    print(f"Last seq:            {seq[-1]:.0f}")
    print(f"Start sim time:      {sim_time_s[0]:.6f} s")
    print(f"End sim time:        {sim_time_s[-1]:.6f} s")
    print(f"Duration:            {sim_time_s[-1] - sim_time_s[0]:.6f} s")

    if len(sample_periods_s) > 0:
        print(f"Mean sample period:  {np.mean(sample_periods_s):.6f} s")
        print(f"Effective rate:      {1.0 / np.mean(sample_periods_s):.3f} Hz")

    print(f"Max altitude:        {np.max(altitude_m):.3f} m")
    print(f"Apogee sim time:     {sim_time_s[apogee_idx]:.3f} s")
    print(f"Max |accel|:         {np.max(accel_norm_g):.3f} g")
    print(f"Drogue events:       {drogue_times}")
    print(f"Main events:         {main_times}")
    print(f"Airbrake changes:    {airbrake_events}")
    print(f"FSM transitions:     {fsm_events}")
    print("======================================\n")

    # ------------------------------------------------------------------
    # Mandatory 3D trajectory with event timing
    # ------------------------------------------------------------------
    fig_3d = plt.figure(figsize=(11, 8))
    ax_3d = fig_3d.add_subplot(111, projection="3d")

    ax_3d.plot(gps_x, gps_y, altitude_m, label="GPS trajectory sent to FC")

    ax_3d.scatter(gps_x[0], gps_y[0], altitude_m[0], s=90, marker="o", label="START")
    ax_3d.text(gps_x[0], gps_y[0], altitude_m[0], f" START\nsim_time={sim_time_s[0]:.2f}s")

    ax_3d.scatter(gps_x[-1], gps_y[-1], altitude_m[-1], s=90, marker="X", label="END")
    ax_3d.text(gps_x[-1], gps_y[-1], altitude_m[-1], f" END\nsim_time={sim_time_s[-1]:.2f}s")

    ax_3d.scatter(
        gps_x[apogee_idx],
        gps_y[apogee_idx],
        altitude_m[apogee_idx],
        s=90,
        marker="^",
        label="APOGEE",
    )
    ax_3d.text(
        gps_x[apogee_idx],
        gps_y[apogee_idx],
        altitude_m[apogee_idx],
        f" APOGEE\nsim_time={sim_time_s[apogee_idx]:.2f}s\nalt={altitude_m[apogee_idx]:.1f}m",
    )

    _mark_event_points_3d(ax_3d, sim_time_s, gps_x, gps_y, altitude_m, drogue_times, "OPEN_DROGUE", "v")
    _mark_event_points_3d(ax_3d, sim_time_s, gps_x, gps_y, altitude_m, main_times, "OPEN_MAIN", "s")
    _mark_fsm_points_3d(ax_3d, sim_time_s, gps_x, gps_y, altitude_m, fsm_events)

    ax_3d.set_title("3D GPS Payload Trajectory Sent to Flight Controller")
    ax_3d.set_xlabel("local GPS x / east [m]")
    ax_3d.set_ylabel("local GPS y / north [m]")
    ax_3d.set_zlabel("GPS altitude [m]")
    _set_legend_if_needed(ax_3d)
    fig_3d.tight_layout()

    # ------------------------------------------------------------------
    # Altitude + barometer pressure together
    # ------------------------------------------------------------------
    fig_alt_pressure, ax_alt = plt.subplots(figsize=(12, 5))
    ax_pressure = ax_alt.twinx()

    ax_alt.plot(sim_time_s, altitude_m, label="GPS altitude", linewidth=1.8)
    ax_pressure.plot(sim_time_s, pressure_pa, linestyle="--", label="barometer pressure")

    ax_alt.scatter(sim_time_s[0], altitude_m[0], s=50, marker="o", label="START")
    ax_alt.scatter(sim_time_s[-1], altitude_m[-1], s=50, marker="X", label="END")
    ax_alt.scatter(sim_time_s[apogee_idx], altitude_m[apogee_idx], s=60, marker="^", label="APOGEE")

    _mark_event_lines(ax_alt, drogue_times, "OPEN_DROGUE")
    _mark_event_lines(ax_alt, main_times, "OPEN_MAIN")
    _mark_fsm_event_lines(ax_alt, fsm_events)
    _mark_fsm_points_2d(ax_alt, sim_time_s, sim_time_s, altitude_m, fsm_events, annotate=True)

    ax_alt.set_title("Altitude and Barometer Pressure Sent to FC")
    ax_alt.set_xlabel("Simulation time [s]")
    ax_alt.set_ylabel("Altitude [m]")
    ax_pressure.set_ylabel("Pressure [Pa]")
    ax_alt.grid(True)

    handles_alt, labels_alt = ax_alt.get_legend_handles_labels()
    handles_pressure, labels_pressure = ax_pressure.get_legend_handles_labels()
    # De-duplicate while preserving order.
    combined = {}
    for handle, label in zip(handles_alt + handles_pressure, labels_alt + labels_pressure):
        if label and label not in combined:
            combined[label] = handle
    ax_alt.legend(combined.values(), combined.keys())
    fig_alt_pressure.tight_layout()

    # ------------------------------------------------------------------
    # GPS ground track
    # ------------------------------------------------------------------
    fig_track, ax_track = plt.subplots(figsize=(8, 8))
    ax_track.plot(gps_x, gps_y, label="GPS ground track")
    ax_track.scatter(gps_x[0], gps_y[0], s=50, marker="o", label="START")
    ax_track.scatter(gps_x[-1], gps_y[-1], s=50, marker="X", label="END")
    ax_track.scatter(gps_x[apogee_idx], gps_y[apogee_idx], s=60, marker="^", label="APOGEE")
    _mark_event_points_2d(ax_track, sim_time_s, gps_x, gps_y, drogue_times, "OPEN_DROGUE", "v")
    _mark_event_points_2d(ax_track, sim_time_s, gps_x, gps_y, main_times, "OPEN_MAIN", "s")
    _mark_fsm_points_2d(ax_track, sim_time_s, gps_x, gps_y, fsm_events, annotate=True)
    ax_track.set_title("GPS Ground Track Sent to FC")
    ax_track.set_xlabel("local GPS x / east [m]")
    ax_track.set_ylabel("local GPS y / north [m]")
    ax_track.axis("equal")
    ax_track.grid(True)
    _set_legend_if_needed(ax_track)
    fig_track.tight_layout()

    # ------------------------------------------------------------------
    # Acceleration payload
    # ------------------------------------------------------------------
    fig_accel, ax_acc = plt.subplots(figsize=(12, 5))
    ax_acc.plot(sim_time_s, accel_x_g, label="ax [g]")
    ax_acc.plot(sim_time_s, accel_y_g, label="ay [g]")
    ax_acc.plot(sim_time_s, accel_z_g, label="az [g]")
    ax_acc.plot(sim_time_s, accel_norm_g, linestyle="--", label="|a| [g]")
    _mark_event_lines(ax_acc, drogue_times, "OPEN_DROGUE")
    _mark_event_lines(ax_acc, main_times, "OPEN_MAIN")
    _mark_fsm_event_lines(ax_acc, fsm_events)
    ax_acc.set_title("Accelerometer Payload Sent to FC")
    ax_acc.set_xlabel("Simulation time [s]")
    ax_acc.set_ylabel("Acceleration [g]")
    ax_acc.grid(True)
    _set_legend_if_needed(ax_acc)
    fig_accel.tight_layout()

    # ------------------------------------------------------------------
    # Optional temperature payload
    # ------------------------------------------------------------------
    if not np.all(np.isnan(temperature_k)):
        fig_temp, ax_temp = plt.subplots(figsize=(12, 4))
        ax_temp.plot(sim_time_s, temperature_k, label="temperature [K]")
        _mark_event_lines(ax_temp, drogue_times, "OPEN_DROGUE")
        _mark_event_lines(ax_temp, main_times, "OPEN_MAIN")
        _mark_fsm_event_lines(ax_temp, fsm_events)
        ax_temp.set_title("Temperature Payload Sent to FC")
        ax_temp.set_xlabel("Simulation time [s]")
        ax_temp.set_ylabel("Temperature [K]")
        ax_temp.grid(True)
        _set_legend_if_needed(ax_temp)
        fig_temp.tight_layout()

    # ------------------------------------------------------------------
    # FSM state timeline
    # ------------------------------------------------------------------
    if len(fsm_events) > 0:
        state_to_y: dict[str, int] = {}
        event_t_values: list[float] = []
        event_y_values: list[int] = []

        # Prefer RocketState ordering when known, but preserve unknown states too.
        for _event_t, state in fsm_events:
            if state in FSM_STATE_ORDER and state not in state_to_y:
                state_to_y[state] = FSM_STATE_ORDER[state]

        for event_t, state in fsm_events:
            event_t_values.append(event_t)
            event_y_values.append(_fsm_state_to_y(state, state_to_y))

        fig_fsm, ax_fsm = plt.subplots(figsize=(12, 4))

        # Step plot: state is assumed to remain active until the next transition.
        step_t = event_t_values.copy()
        step_y = event_y_values.copy()
        if step_t[0] > sim_time_s[0]:
            step_t.insert(0, sim_time_s[0])
            step_y.insert(0, step_y[0])
        if step_t[-1] < sim_time_s[-1]:
            step_t.append(sim_time_s[-1])
            step_y.append(step_y[-1])

        ax_fsm.step(step_t, step_y, where="post", label="FSM state")
        ax_fsm.scatter(event_t_values, event_y_values, s=65, marker="o", label="FSM transition")

        for event_t, state, y_value in zip(event_t_values, [s for _, s in fsm_events], event_y_values):
            ax_fsm.annotate(
                f"{state}\nt={event_t:.2f}s",
                xy=(event_t, y_value),
                xytext=(6, 6),
                textcoords="offset points",
                fontsize=8,
            )

        # Build readable y ticks. Sort by numeric y value.
        y_to_state = {y: state for state, y in state_to_y.items()}
        used_y_values = sorted(set(event_y_values))
        ax_fsm.set_yticks(used_y_values)
        ax_fsm.set_yticklabels([y_to_state.get(y, str(y)) for y in used_y_values])
        ax_fsm.set_title("FSM State Timeline Reported by Flight Controller")
        ax_fsm.set_xlabel("Simulation time [s]")
        ax_fsm.set_ylabel("FSM state")
        ax_fsm.grid(True)
        _set_legend_if_needed(ax_fsm)
        fig_fsm.tight_layout()

    # ------------------------------------------------------------------
    # Airbrakes command level
    # ------------------------------------------------------------------
    if len(airbrake_events) > 0:
        air_t = [sim_time_s[0]]
        air_lvl = [0.0]

        for event_t, lvl in airbrake_events:
            air_t.append(event_t)
            air_lvl.append(lvl)

        air_t.append(sim_time_s[-1])
        air_lvl.append(air_lvl[-1])

        fig_air, ax_air = plt.subplots(figsize=(12, 4))
        ax_air.step(air_t, air_lvl, where="post")
        _mark_fsm_event_lines(ax_air, fsm_events)
        ax_air.set_title("Airbrakes Command Received from FC")
        ax_air.set_xlabel("Simulation time [s]")
        ax_air.set_ylabel("Deployment level")
        ax_air.set_ylim(-0.05, 1.05)
        ax_air.grid(True)
        _set_legend_if_needed(ax_air)
        fig_air.tight_layout()

    if show:
        plt.show()


# ----------------------------------------------------------------------
# CLI ENTRYPOINT
# ----------------------------------------------------------------------

def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Plot a saved RocketPy HIL capture without rerunning the simulation."
    )

    parser.add_argument(
        "capture",
        nargs="?",
        default=None,
        help="Path to a saved HIL capture JSON file.",
    )

    parser.add_argument(
        "--plot-only",
        dest="plot_only",
        default=None,
        help="Path to a saved HIL capture JSON file. Equivalent to positional capture.",
    )

    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Create figures but do not call plt.show(). Mostly useful for tests.",
    )

    return parser


def main() -> None:
    parser = _build_arg_parser()
    args = parser.parse_args()

    capture_path = args.plot_only or args.capture

    if capture_path is None:
        parser.error(
            "missing capture file. Example: "
            "python hil_capture.py hil_captures/hil_capture_2026-05-18_12-00-00.json"
        )

    hil_log, hil_events, _metadata = load_hil_capture(capture_path)

    plot_hil_log(
        hil_log,
        hil_events,
        show=not args.no_show,
    )


if __name__ == "__main__":
    main()
