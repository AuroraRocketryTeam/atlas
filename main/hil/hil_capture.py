"""
hil_capture.py

Utilities to save, load, and plot HIL captures produced by hil_rocketpy.py.

Reference-frame convention used by the 3D replay:

    S_out -> S_clean -> B -> I

where S_out is the saved accelerometer payload after cross-axis mixing,
S_clean is the ideal orthogonal sensor frame, B is the RocketPy body frame,
and I is the RocketPy inertial frame (+X east, +Y north, +Z up).

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
from matplotlib.animation import FuncAnimation

from hil_utils import (
    accelerometer_output_to_clean_sensor_matrix_from_metadata as _accelerometer_output_to_clean_sensor_matrix,
    accelerometer_sensor_to_body_from_metadata as _accelerometer_sensor_to_body_matrix,
    accelerometer_sensor_to_body_from_metadata_legacy_degrees_bug as _accelerometer_sensor_to_body_matrix_legacy_degrees_bug,
    rocketpy_body_to_inertial_matrix as _rocketpy_body_to_inertial_matrix,
)


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


# Shared frame/sensor transform helpers live in hil_utils.py and are imported
# above with the historical local names used by the plotting code.

def _active_fsm_state(
    sim_time_s: float,
    fsm_events: list[tuple[float, str]],
) -> str:
    """Return the most recent FSM state at ``sim_time_s``."""
    active_state = "UNKNOWN"

    for event_time_s, state in fsm_events:
        if event_time_s > sim_time_s:
            break
        active_state = state

    return active_state


def _set_3d_axes_equal(
    ax,
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    *,
    padding_fraction: float = 0.08,
) -> float:
    """Set equal physical scaling on all three axes and return the plot span."""
    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    if not np.any(finite):
        raise ValueError("3D replay contains no finite trajectory samples.")

    mins = np.asarray(
        [np.min(x[finite]), np.min(y[finite]), np.min(z[finite])],
        dtype=float,
    )
    maxs = np.asarray(
        [np.max(x[finite]), np.max(y[finite]), np.max(z[finite])],
        dtype=float,
    )

    center = (mins + maxs) / 2
    span = max(float(np.max(maxs - mins)), 1.0)
    half_span = span * (0.5 + padding_fraction)

    ax.set_xlim(center[0] - half_span, center[0] + half_span)
    ax.set_ylim(center[1] - half_span, center[1] + half_span)
    ax.set_zlim(center[2] - half_span, center[2] + half_span)
    ax.set_box_aspect((1, 1, 1))

    return span


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


def _set_legend_if_needed(ax, **legend_kwargs):
    handles, labels = ax.get_legend_handles_labels()
    if not handles:
        return None

    # De-duplicate labels while preserving order.
    unique: dict[str, Any] = {}
    for handle, label in zip(handles, labels):
        if label and label not in unique:
            unique[label] = handle

    if not unique:
        return None

    return ax.legend(unique.values(), unique.keys(), **legend_kwargs)


# ----------------------------------------------------------------------
# MAIN ANALYSIS / PLOT API
# ----------------------------------------------------------------------

def replay_hil_3d(
    hil_log: dict[str, list[Any]],
    hil_events: dict[str, list[Any]] | None = None,
    metadata: dict[str, Any] | None = None,
    *,
    playback_speed: float = 1.0,
    show: bool = True,
) -> tuple[Any, FuncAnimation]:
    """
    Animate the RocketPy trajectory and attitude stored in a HIL capture.

    The replay uses RocketPy's state directly:

      - ``x, y, z`` are RocketPy inertial coordinates as captured;
      - ``e0, e1, e2, e3`` define body orientation;
      - body ``+Z`` points from the center of dry mass towards the nose.

    The rocket and body axes are deliberately enlarged so their orientation
    remains visible over the complete trajectory. They are orientation glyphs,
    not a geometrically scaled rocket model.

    Controls
    --------
    Space
        Pause or resume.
    R
        Restart from the first sample.
    """
    if playback_speed <= 0:
        raise ValueError("playback_speed must be greater than zero.")

    if hil_events is None:
        hil_events = {}

    sim_time_s = _as_array(hil_log, "sim_time_s")
    if len(sim_time_s) == 0:
        raise ValueError("Cannot replay an empty HIL capture.")

    required_keys = (
        "x",
        "y",
        "z",
        "vx",
        "vy",
        "vz",
        "e0",
        "e1",
        "e2",
        "e3",
        "omega1",
        "omega2",
        "omega3",
        "accel_x_m_s2",
        "accel_y_m_s2",
        "accel_z_m_s2",
        "pressure_pa",
    )
    state = {key: _as_array(hil_log, key) for key in required_keys}

    expected_length = len(sim_time_s)
    mismatched = [
        key for key, values in state.items() if len(values) != expected_length
    ]
    if mismatched:
        raise ValueError(
            "3D replay state arrays do not match sim_time_s length: "
            + ", ".join(mismatched)
        )

    x = state["x"]
    y = state["y"]
    z = state["z"]
    accel_x_g = state["accel_x_m_s2"] / G0
    accel_y_g = state["accel_y_m_s2"] / G0
    accel_z_g = state["accel_z_m_s2"] / G0
    accel_norm_g = np.sqrt(
        state["accel_x_m_s2"] ** 2
        + state["accel_y_m_s2"] ** 2
        + state["accel_z_m_s2"] ** 2
    ) / G0
    pressure_hpa = state["pressure_pa"] / 100.0

    # Per-sample attitude: body -> inertial.
    rotations = np.asarray(
        [
            _rocketpy_body_to_inertial_matrix(e0, e1, e2, e3)
            for e0, e1, e2, e3 in zip(
                state["e0"],
                state["e1"],
                state["e2"],
                state["e3"],
            )
        ]
    )
    # Static accelerometer mounting: sensor -> body.
    accelerometer_sensor_to_body = _accelerometer_sensor_to_body_matrix(
        metadata
    )
    # Payload de-mixing: S_out -> S_clean.
    accelerometer_output_to_clean_sensor = (
        _accelerometer_output_to_clean_sensor_matrix(metadata)
    )
    # Diagnostic for launchpad/calibration: S_out -> S_clean -> body -> inertial.
    # A stationary rocket should reconstruct close to +1 g on inertial Z.
    first_accel_sensor_output = np.asarray(
        [
            state["accel_x_m_s2"][0],
            state["accel_y_m_s2"][0],
            state["accel_z_m_s2"][0],
        ],
        dtype=float,
    )
    first_accel_sensor_clean = (
        accelerometer_output_to_clean_sensor @ first_accel_sensor_output
    )
    first_accel_inertial = (
        rotations[0] @ accelerometer_sensor_to_body @ first_accel_sensor_clean
    )

    # Compatibility for captures generated before HIL converted radian Euler
    # orientations to explicit RocketPy matrices. Those old captures may contain
    # samples produced with RocketPy's accidental rad-as-deg interpretation, while
    # the metadata still stores radian angles. Prefer the new radian convention,
    # but use the legacy matrix only when it is the one that reconstructs the
    # stationary calibration vector upward.
    legacy_orientation_used = False
    if first_accel_inertial[2] < 0.0:
        legacy_sensor_to_body = _accelerometer_sensor_to_body_matrix_legacy_degrees_bug(
            metadata
        )
        if legacy_sensor_to_body is not None:
            legacy_first_accel_inertial = (
                rotations[0] @ legacy_sensor_to_body @ first_accel_sensor_clean
            )
            if legacy_first_accel_inertial[2] > 0.0:
                accelerometer_sensor_to_body = legacy_sensor_to_body
                first_accel_inertial = legacy_first_accel_inertial
                legacy_orientation_used = True

    print(
        "[REPLAY] first accel clean sensor = "
        f"({first_accel_sensor_clean[0] / G0:.3f}, "
        f"{first_accel_sensor_clean[1] / G0:.3f}, "
        f"{first_accel_sensor_clean[2] / G0:.3f}) g; "
        "inertial = "
        f"({first_accel_inertial[0] / G0:.3f}, "
        f"{first_accel_inertial[1] / G0:.3f}, "
        f"{first_accel_inertial[2] / G0:.3f}) g"
    )
    if legacy_orientation_used:
        print(
            "[REPLAY COMPAT] using legacy rad-as-deg orientation interpretation "
            "for this old capture. New captures store/use radians consistently."
        )
    elif first_accel_inertial[2] < 0.0:
        print(
            "[REPLAY WARNING] first reconstructed accelerometer vector points "
            "down in inertial Z. For stationary calibration it should point "
            "from earth to sky. Check capture metadata and attitude."
        )

    drogue_times = _sanitize_event_times(hil_events.get("open_drogue", []))
    main_times = _sanitize_event_times(hil_events.get("open_main", []))
    fsm_events = _sanitize_fsm_events(hil_events.get("fsm_state", []))

    # Keep the trajectory large while leaving two synchronized telemetry plots
    # visible throughout the replay.
    fig = plt.figure(figsize=(17, 9))
    grid = fig.add_gridspec(
        2,
        2,
        width_ratios=(1.55, 1.0),
        height_ratios=(1.0, 1.0),
        wspace=0.20,
        hspace=0.28,
    )
    ax = fig.add_subplot(grid[:, 0], projection="3d")
    ax_accel = fig.add_subplot(grid[0, 1])
    ax_pressure = fig.add_subplot(grid[1, 1], sharex=ax_accel)

    ax.plot(
        x,
        y,
        z,
        color="0.75",
        linewidth=1.0,
        label="Complete trajectory",
    )
    trail, = ax.plot([], [], [], color="tab:blue", linewidth=2.0, label="Replay trail")
    position_marker, = ax.plot(
        [],
        [],
        [],
        marker="o",
        linestyle="",
        color="black",
        markersize=5,
        label="Rocket CDM",
    )

    # Mark events in inertial space.
    _mark_event_points_3d(
        ax,
        sim_time_s,
        x,
        y,
        z,
        drogue_times,
        "OPEN_DROGUE",
        "v",
    )
    _mark_event_points_3d(
        ax,
        sim_time_s,
        x,
        y,
        z,
        main_times,
        "OPEN_MAIN",
        "s",
    )

    apogee_idx = int(np.argmax(z))
    ax.scatter(
        x[apogee_idx],
        y[apogee_idx],
        z[apogee_idx],
        s=70,
        marker="^",
        color="tab:purple",
        label="APOGEE",
    )

    plot_span = _set_3d_axes_equal(ax, x, y, z)
    body_axis_length = max(0.065 * plot_span, 1.0)
    rocket_half_length = 0.75 * body_axis_length

    # Fixed inertial reference frame at the launch point.
    inertial_axis_length = 0.8 * body_axis_length
    inertial_colors = ("tab:red", "tab:orange", "tab:green")
    inertial_labels = ("+X east", "+Y north", "+Z up")

    for axis_index, (color, label) in enumerate(
        zip(inertial_colors, inertial_labels)
    ):
        end = np.asarray([x[0], y[0], z[0]], dtype=float)
        end[axis_index] += inertial_axis_length
        ax.plot(
            [x[0], end[0]],
            [y[0], end[1]],
            [z[0], end[2]],
            color=color,
            linewidth=2.0,
        )
        ax.text(end[0], end[1], end[2], f" inertial {label}", color=color)

    # Moving body axes. RocketPy body +Z points towards the nose.
    body_axis_lines = []
    body_axis_labels = []
    body_axis_names = ("body +X", "body +Y", "body +Z / NOSE")

    for color, name in zip(inertial_colors, body_axis_names):
        line, = ax.plot([], [], [], color=color, linewidth=3.0, label=name)
        label = ax.text(0, 0, 0, "", color=color, fontsize=9)
        body_axis_lines.append(line)
        body_axis_labels.append(label)

    # Moving sensor axes. These show how the accelerometer triad is mounted
    # relative to the rocket body. The axes are drawn at the actual rocket/sensor
    # origin so the acceleration components lie directly on the displayed axes.
    sensor_axis_lines = []
    sensor_axis_labels = []
    sensor_axis_names = ("sensor +X", "sensor +Y", "sensor +Z")
    sensor_axis_colors = ("tab:purple", "tab:cyan", "tab:brown")
    sensor_axis_length = 0.8 * body_axis_length

    sensor_origin_marker, = ax.plot(
        [],
        [],
        [],
        marker="o",
        linestyle="",
        color="tab:purple",
        markersize=5,
        label="Accelerometer triad origin",
    )

    for color, name in zip(sensor_axis_colors, sensor_axis_names):
        line, = ax.plot(
            [],
            [],
            [],
            color=color,
            linewidth=2.8,
            linestyle="--",
            label=name,
        )
        label = ax.text(0, 0, 0, "", color=color, fontsize=8)
        sensor_axis_lines.append(line)
        sensor_axis_labels.append(label)

    rocket_centerline, = ax.plot(
        [],
        [],
        [],
        color="black",
        linewidth=5.0,
        solid_capstyle="round",
        label="Rocket axis (not to scale)",
    )

    # Quiver artists are recreated each frame because Matplotlib does not
    # expose an in-place 3D vector update API.
    dynamic_arrows: dict[str, Any] = {
        "nose": None,
        "acceleration": None,
        "accel_sensor_x": None,
        "accel_sensor_y": None,
        "accel_sensor_z": None,
    }

    # Legend proxies for the moving acceleration arrows.
    acceleration_legend, = ax.plot(
        [],
        [],
        [],
        color="magenta",
        linewidth=3.0,
        label="Accelerometer specific-force vector",
    )
    accel_sensor_component_legends = [
        ax.plot(
            [],
            [],
            [],
            color=color,
            linewidth=2.0,
            linestyle="-.",
            label=f"Accel component on {name}",
        )[0]
        for color, name in zip(sensor_axis_colors, sensor_axis_names)
    ]

    # Acceleration history in the sensor frame, synchronized with the 3D replay.
    accel_series = (
        ("ax [g]", accel_x_g, "tab:blue"),
        ("ay [g]", accel_y_g, "tab:orange"),
        ("az [g]", accel_z_g, "tab:green"),
        ("|a| [g]", accel_norm_g, "black"),
    )

    for label, values, color in accel_series[:3]:
        ax_accel.plot(
            sim_time_s,
            values,
            label=label,
            color=color,
            linewidth=1.2,
        )
    ax_accel.plot(
        sim_time_s,
        accel_norm_g,
        linestyle="--",
        color=accel_series[3][2],
        label=accel_series[3][0],
        linewidth=1.3,
    )
    accel_time_cursor = ax_accel.axvline(
        sim_time_s[0],
        color="red",
        linewidth=1.8,
        label="Replay time",
        zorder=10,
    )
    accel_current_markers = [
        ax_accel.plot(
            [sim_time_s[0]],
            [values[0]],
            marker="o",
            linestyle="",
            color=color,
            markersize=5,
            zorder=11,
        )[0]
        for _label, values, color in accel_series
    ]
    _mark_event_lines(ax_accel, drogue_times, "OPEN_DROGUE")
    _mark_event_lines(ax_accel, main_times, "OPEN_MAIN")
    _mark_fsm_event_lines(ax_accel, fsm_events)
    ax_accel.set_title("Accelerometer payload")
    ax_accel.set_ylabel("Specific force [g]")
    ax_accel.set_xlim(sim_time_s[0], sim_time_s[-1])
    ax_accel.grid(True, alpha=0.35)
    _set_legend_if_needed(ax_accel, fontsize=8, markerscale=0.8)

    # Barometer history. Pressure is shown in hPa for a more readable scale.
    ax_pressure.plot(
        sim_time_s,
        pressure_hpa,
        color="tab:cyan",
        label="Pressure [hPa]",
        linewidth=1.4,
    )
    pressure_time_cursor = ax_pressure.axvline(
        sim_time_s[0],
        color="red",
        linewidth=1.8,
        label="Replay time",
        zorder=10,
    )
    pressure_current_marker, = ax_pressure.plot(
        [sim_time_s[0]],
        [pressure_hpa[0]],
        marker="o",
        linestyle="",
        color="tab:cyan",
        markersize=6,
        zorder=11,
    )
    _mark_event_lines(ax_pressure, drogue_times, "OPEN_DROGUE")
    _mark_event_lines(ax_pressure, main_times, "OPEN_MAIN")
    _mark_fsm_event_lines(ax_pressure, fsm_events)
    ax_pressure.set_title("Barometer payload")
    ax_pressure.set_xlabel("Simulation time [s]")
    ax_pressure.set_ylabel("Pressure [hPa]")
    ax_pressure.set_xlim(sim_time_s[0], sim_time_s[-1])
    ax_pressure.grid(True, alpha=0.35)
    _set_legend_if_needed(ax_pressure, fontsize=8, markerscale=0.8)

    status_text = ax.text2D(
        0.02,
        0.98,
        "",
        transform=ax.transAxes,
        va="top",
        family="monospace",
        bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "0.7"},
    )
    help_text = ax.text2D(
        0.02,
        0.02,
        "Space: pause/resume    R: restart    L: show/hide 3D legend\n"
        "Black: rocket nose/body +Z    Dashed: accelerometer axes\n"
        "Magenta: accel vector    Dash-dot: accel components",
        transform=ax.transAxes,
        va="bottom",
        fontsize=8,
        bbox={"facecolor": "white", "alpha": 0.65, "edgecolor": "none"},
    )

    ax.set_title("RocketPy HIL 3D Trajectory and Attitude Replay")
    ax.set_xlabel("X east [m]")
    ax.set_ylabel("Y north [m]")
    ax.set_zlabel("RocketPy inertial Z [m]")
    ax.view_init(elev=24, azim=-58)
    legend_3d = _set_legend_if_needed(
        ax,
        loc="upper left",
        bbox_to_anchor=(0.01, 0.99),
        fontsize=7,
        markerscale=0.65,
        framealpha=0.60,
        borderpad=0.25,
        labelspacing=0.25,
        handlelength=1.2,
        handletextpad=0.4,
    )
    if legend_3d is not None:
        legend_3d.set_visible(False)

    fig.subplots_adjust(
        left=0.04,
        right=0.98,
        bottom=0.08,
        top=0.94,
    )

    positive_periods = np.diff(sim_time_s)
    positive_periods = positive_periods[positive_periods > 0]
    mean_period_s = (
        float(np.mean(positive_periods)) if len(positive_periods) else 0.05
    )
    interval_ms = max(1.0, 1000.0 * mean_period_s / playback_speed)

    animation_state = {"paused": False, "legend_visible": False}

    def update(frame_index: int):
        position = np.asarray(
            [x[frame_index], y[frame_index], z[frame_index]],
            dtype=float,
        )
        rotation = rotations[frame_index]
        current_time_s = sim_time_s[frame_index]

        trail.set_data_3d(
            x[: frame_index + 1],
            y[: frame_index + 1],
            z[: frame_index + 1],
        )
        position_marker.set_data_3d(
            [position[0]],
            [position[1]],
            [position[2]],
        )

        # Advance the same replay cursor and current-value markers through all
        # telemetry plots so every panel refers to this exact simulation sample.
        accel_time_cursor.set_xdata([current_time_s, current_time_s])
        pressure_time_cursor.set_xdata([current_time_s, current_time_s])

        for marker, values in zip(
            accel_current_markers,
            (accel_x_g, accel_y_g, accel_z_g, accel_norm_g),
        ):
            marker.set_data([current_time_s], [values[frame_index]])

        pressure_current_marker.set_data(
            [current_time_s],
            [pressure_hpa[frame_index]],
        )

        # Matrix columns are body unit axes expressed in inertial coordinates.
        for axis_index, (line, label, name) in enumerate(
            zip(body_axis_lines, body_axis_labels, body_axis_names)
        ):
            endpoint = position + body_axis_length * rotation[:, axis_index]
            line.set_data_3d(
                [position[0], endpoint[0]],
                [position[1], endpoint[1]],
                [position[2], endpoint[2]],
            )
            label.set_position_3d(endpoint)
            label.set_text(f" {name}")

        # Sensor axes: sensor -> body -> inertial. No visual offset.
        sensor_rotation = rotation @ accelerometer_sensor_to_body
        sensor_axis_origin = position.copy()
        sensor_origin_marker.set_data_3d(
            [sensor_axis_origin[0]],
            [sensor_axis_origin[1]],
            [sensor_axis_origin[2]],
        )

        for axis_index, (line, label, name) in enumerate(
            zip(sensor_axis_lines, sensor_axis_labels, sensor_axis_names)
        ):
            axis_direction = sensor_rotation[:, axis_index]
            negative_endpoint = (
                sensor_axis_origin
                - sensor_axis_length * axis_direction
            )
            positive_endpoint = (
                sensor_axis_origin
                + sensor_axis_length * axis_direction
            )
            line.set_data_3d(
                [negative_endpoint[0], positive_endpoint[0]],
                [negative_endpoint[1], positive_endpoint[1]],
                [negative_endpoint[2], positive_endpoint[2]],
            )
            label.set_position_3d(positive_endpoint)
            label.set_text(f" {name}")

        body_z_inertial = rotation[:, 2]
        tail = position - rocket_half_length * body_z_inertial
        nose = position + rocket_half_length * body_z_inertial
        rocket_centerline.set_data_3d(
            [tail[0], nose[0]],
            [tail[1], nose[1]],
            [tail[2], nose[2]],
        )

        # Use a real 3D arrow instead of a "^" marker. A marker is always
        # oriented towards the screen and therefore gave a false nose direction
        # when the rocket pointed downwards.
        if dynamic_arrows["nose"] is not None:
            dynamic_arrows["nose"].remove()
        dynamic_arrows["nose"] = ax.quiver(
            tail[0],
            tail[1],
            tail[2],
            body_z_inertial[0],
            body_z_inertial[1],
            body_z_inertial[2],
            length=2 * rocket_half_length,
            normalize=True,
            color="black",
            linewidth=2.2,
            arrow_length_ratio=0.22,
        )

        acceleration_sensor_output = np.asarray(
            [
                state["accel_x_m_s2"][frame_index],
                state["accel_y_m_s2"][frame_index],
                state["accel_z_m_s2"][frame_index],
            ],
            dtype=float,
        )
        # Acceleration payload transform: S_out -> S_clean -> body -> inertial.
        acceleration_sensor_clean = (
            accelerometer_output_to_clean_sensor @ acceleration_sensor_output
        )
        acceleration_body = (
            accelerometer_sensor_to_body @ acceleration_sensor_clean
        )
        acceleration_inertial = rotation @ acceleration_body
        acceleration_magnitude = float(np.linalg.norm(acceleration_body))
        acceleration_magnitude_g = acceleration_magnitude / G0

        for arrow_name in (
            "acceleration",
            "accel_sensor_x",
            "accel_sensor_y",
            "accel_sensor_z",
        ):
            if dynamic_arrows[arrow_name] is not None:
                dynamic_arrows[arrow_name].remove()
                dynamic_arrows[arrow_name] = None

        # Preserve the reconstructed specific-force direction. Arrow length is
        # linear up to 2 g and capped afterwards so the motor peak does not hide
        # the trajectory.
        #
        # In addition to the full inertial-space vector, draw the orthogonal
        # decomposition on the displayed sensor axes. Cross-axis output mixing
        # is inverted before this split, so the three component arrows are true
        # geometric projections and sum back to the magenta resultant. These are
        # not offset: they originate at the same rocket/sensor point.
        if acceleration_magnitude > 1e-9:
            acceleration_direction = (
                acceleration_inertial
                / np.linalg.norm(acceleration_inertial)
            )
            acceleration_length = (
                body_axis_length * min(acceleration_magnitude_g, 2.0)
            )
            dynamic_arrows["acceleration"] = ax.quiver(
                position[0],
                position[1],
                position[2],
                acceleration_direction[0],
                acceleration_direction[1],
                acceleration_direction[2],
                length=acceleration_length,
                normalize=True,
                color="magenta",
                linewidth=3.0,
                arrow_length_ratio=0.18,
            )

            for axis_index, (arrow_name, color) in enumerate(
                zip(
                    ("accel_sensor_x", "accel_sensor_y", "accel_sensor_z"),
                    sensor_axis_colors,
                )
            ):
                component_g = float(acceleration_sensor_clean[axis_index] / G0)
                component_display_g = float(np.clip(component_g, -2.0, 2.0))
                if abs(component_display_g) <= 1e-9:
                    continue

                # Draw each component directly on the corresponding sensor axis.
                # A negative sensor-frame component naturally points along the
                # negative side of that same axis.
                component_vector = (
                    sensor_rotation[:, axis_index]
                    * body_axis_length
                    * component_display_g
                )
                dynamic_arrows[arrow_name] = ax.quiver(
                    sensor_axis_origin[0],
                    sensor_axis_origin[1],
                    sensor_axis_origin[2],
                    component_vector[0],
                    component_vector[1],
                    component_vector[2],
                    color=color,
                    linewidth=4.2,
                    arrow_length_ratio=0.32,
                    linestyle="-.",
                    normalize=False,
                )

        velocity = np.asarray(
            [
                state["vx"][frame_index],
                state["vy"][frame_index],
                state["vz"][frame_index],
            ]
        )
        omega = np.asarray(
            [
                state["omega1"][frame_index],
                state["omega2"][frame_index],
                state["omega3"][frame_index],
            ]
        )
        quaternion = np.asarray(
            [
                state["e0"][frame_index],
                state["e1"][frame_index],
                state["e2"][frame_index],
                state["e3"][frame_index],
            ]
        )
        active_state = _active_fsm_state(
            sim_time_s[frame_index],
            fsm_events,
        )

        status_text.set_text(
            f"t       = {current_time_s:8.3f} s\n"
            f"frame   = {frame_index + 1:4d}/{expected_length}\n"
            f"FSM     = {active_state}\n"
            f"position= ({position[0]:7.2f}, {position[1]:7.2f}, "
            f"{position[2]:7.2f}) m\n"
            f"speed   = {np.linalg.norm(velocity):8.2f} m/s\n"
            f"q       = ({quaternion[0]: .3f}, {quaternion[1]: .3f}, "
            f"{quaternion[2]: .3f}, {quaternion[3]: .3f})\n"
            f"|q|     = {np.linalg.norm(quaternion):8.5f}\n"
            f"omega   = ({omega[0]: .3f}, {omega[1]: .3f}, "
            f"{omega[2]: .3f}) rad/s\n"
            f"accel S_out = ({acceleration_sensor_output[0] / G0: .3f}, "
            f"{acceleration_sensor_output[1] / G0: .3f}, "
            f"{acceleration_sensor_output[2] / G0: .3f}) g\n"
            f"accel S_clean = ({acceleration_sensor_clean[0] / G0: .3f}, "
            f"{acceleration_sensor_clean[1] / G0: .3f}, "
            f"{acceleration_sensor_clean[2] / G0: .3f}) g\n"
            f"accel I = ({acceleration_inertial[0] / G0: .3f}, "
            f"{acceleration_inertial[1] / G0: .3f}, "
            f"{acceleration_inertial[2] / G0: .3f}) g\n"
            f"|accel| = {acceleration_magnitude_g:8.3f} g\n"
            f"pressure= {pressure_hpa[frame_index]:8.2f} hPa"
        )

        return (
            trail,
            position_marker,
            rocket_centerline,
            sensor_origin_marker,
            acceleration_legend,
            *accel_sensor_component_legends,
            accel_time_cursor,
            pressure_time_cursor,
            pressure_current_marker,
            status_text,
            help_text,
            *accel_current_markers,
            *body_axis_lines,
            *body_axis_labels,
            *sensor_axis_lines,
            *sensor_axis_labels,
        )

    replay = FuncAnimation(
        fig,
        update,
        frames=expected_length,
        interval=interval_ms,
        repeat=False,
        blit=False,
    )

    def on_key_press(event) -> None:
        key = (event.key or "").lower()

        if key == " ":
            if animation_state["paused"]:
                replay.event_source.start()
            else:
                replay.event_source.stop()
            animation_state["paused"] = not animation_state["paused"]
        elif key == "r":
            replay.frame_seq = replay.new_frame_seq()
            replay.event_source.start()
            animation_state["paused"] = False
        elif key == "l" and legend_3d is not None:
            animation_state["legend_visible"] = not animation_state["legend_visible"]
            legend_3d.set_visible(animation_state["legend_visible"])
            fig.canvas.draw_idle()

    fig.canvas.mpl_connect("key_press_event", on_key_press)

    # Keep a strong reference for interactive backends.
    fig._hil_replay_animation = replay  # type: ignore[attr-defined]

    if show:
        plt.show()

    return fig, replay


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

    parser.add_argument(
        "--replay-3d",
        action="store_true",
        help="Display an animated 3D trajectory and rocket-attitude replay.",
    )

    parser.add_argument(
        "--replay-only",
        action="store_true",
        help="Display only the animated 3D replay, without the static report plots.",
    )

    parser.add_argument(
        "--replay-speed",
        type=float,
        default=1.0,
        metavar="FACTOR",
        help="3D replay speed multiplier. Default: 1.0.",
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

    hil_log, hil_events, metadata = load_hil_capture(capture_path)

    if args.replay_speed <= 0:
        parser.error("--replay-speed must be greater than zero.")

    replay_requested = args.replay_3d or args.replay_only

    if not args.replay_only:
        plot_hil_log(
            hil_log,
            hil_events,
            show=not args.no_show and not replay_requested,
        )

    if replay_requested:
        replay_hil_3d(
            hil_log,
            hil_events,
            metadata,
            playback_speed=args.replay_speed,
            show=not args.no_show,
        )


if __name__ == "__main__":
    main()
