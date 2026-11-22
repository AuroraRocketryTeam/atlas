"""
hil_capture.py

Utilities to save, load, and plot HIL captures produced by hil_rocketpy.py.

Captures store two parallel structures:

    hil_log["sim_time_s"]
        The compact capture time axis used by plots and replay.

    hil_events["time_eclipses"]
        Optional markers for time spans that were sent to the FC but collapsed
        out of the capture. Example:

            {
                "label": "GROUND_SERVICES",
                "time_s": 3.0,
                "omitted_duration_s": 17.5,
                "omitted_samples": 350
            }

        This means capture time jumps from calibration directly to launch at
        t=3.0 s, while the FC actually received another 17.5 s of stationary
        Ground Services samples before READY_FOR_LAUNCH.

Typical usage from hil_rocketpy.py:

    from hil_capture import create_capture_file, save_hil_capture, plot_hil_log
    capture_file = create_capture_file(BASE_DIR / "hil_captures")
    ...
    metadata = {"sampling_rate_hz": sampling_rate, "rocket": rocket_model}
    save_hil_capture(
        filename=capture_file,
        hil_log=hil_log,
        hil_events=hil_events,
        metadata=metadata,
    )
    plot_hil_log(hil_log, hil_events, metadata=metadata)

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
from typing import Any, Callable

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider

from hil_utils import (
    accelerometer_output_to_clean_sensor_matrix_from_metadata as _accelerometer_output_to_clean_sensor_matrix,
    accelerometer_sensor_to_body_from_metadata as _accelerometer_sensor_to_body_matrix,
    rocketpy_body_to_inertial_matrix as _rocketpy_body_to_inertial_matrix,
)


# ----------------------------------------------------------------------
# CONSTANTS
# ----------------------------------------------------------------------

G0 = 9.80665
FRED_SITE_IMAGE = Path(__file__).with_name("Villafranca_satellite.jpg")
FRED_SITE_ORIGIN = (44.290583, 12.027111)  # latitude, longitude of (0, 0)
FRED_LAUNCH_SITES = {FRED_SITE_ORIGIN, (44.2901999, 12.0278684)}
# Mission Analysis extent for the Esri tile image; verify with local control points.
FRED_SITE_EXTENT_M = (-1043.10, 657.12, -407.15, 593.24)  # west, east, south, north

# Keep this local so hil_capture.py can plot saved captures without importing
# hil_communication.py. These names mirror RocketState / hil_communication.
FSM_STATE_ORDER = {
    "INACTIVE": 0,
    "CALIBRATING": 1,
    "GROUND_SERVICES": 2,
    "READY_FOR_LAUNCH": 3,
    "LAUNCH": 4,
    "ACCELERATED_FLIGHT": 5,
    "BALLISTIC_FLIGHT": 6,
    "APOGEE": 7,
    "STABILIZATION": 8,
    "DECELERATION": 9,
    "LANDING": 10,
    "RECOVERED": 11,
}

FSM_STATE_MARKERS = ["o", "s", "^", "D", "v", "P", "X", "*", "h", "<", ">", "p"]


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

    required_capture_keys = ("metadata", "hil_log", "hil_events")
    missing_capture_keys = [
        key for key in required_capture_keys if key not in capture
    ]
    if missing_capture_keys:
        raise ValueError(
            f"Invalid capture file: missing keys {missing_capture_keys}: {filename}"
        )

    hil_log = capture["hil_log"]
    hil_events = capture["hil_events"]
    metadata = capture["metadata"]

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


def _lat_lon_to_local_meters(
    lat_deg: np.ndarray,
    lon_deg: np.ndarray,
    origin_lat_lon: tuple[float, float] | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Convert latitude/longitude to a local tangent-plane approximation.

    The GPS data is stored as geodetic coordinates, but the trajectory plots are
    easier to read in meters. For the short ranges covered by HIL flights, a
    first-order local tangent plane is accurate enough:

        north ~= earth_radius * delta_lat
        east  ~= earth_radius * delta_lon * cos(latitude_at_launch)

    By default the first GPS sample is the local origin. A site map can supply
    its calibrated launch point instead, so GPS noise does not shift the image.

    Output:
        x = east displacement [m]
        y = north displacement [m]
    """
    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)

    lat0, lon0 = (lat[0], lon[0]) if origin_lat_lon is None else np.deg2rad(origin_lat_lon)

    earth_radius = 6_371_000.0

    x = earth_radius * (lon - lon0) * np.cos(lat0)
    y = earth_radius * (lat - lat0)

    return x, y


def _nearest_sample_index(sim_time_s: np.ndarray, target_sim_time_s: float) -> int:
    return int(np.argmin(np.abs(sim_time_s - target_sim_time_s)))


def rocketpy_apogee_event(flight: Any, capture_time_offset_s: float) -> dict[str, float]:
    """Build the authoritative capture event from RocketPy's solved Flight."""
    event = {
        "time_s": float(capture_time_offset_s) + float(flight.apogee_time),
        "rocketpy_time_s": float(flight.apogee_time),
        "x": float(flight.apogee_x),
        "y": float(flight.apogee_y),
        "z": float(flight.apogee),
    }
    if not all(np.isfinite(value) for value in event.values()):
        raise ValueError("RocketPy returned invalid apogee data")
    return event


def _sanitize_rocketpy_apogee(raw_events: Any) -> dict[str, float] | None:
    """Return the first complete authoritative RocketPy apogee event."""
    if raw_events is None:
        return None
    if isinstance(raw_events, dict):
        raw_events = [raw_events]
    for item in raw_events:
        if not isinstance(item, dict):
            continue
        try:
            event = {key: float(item[key]) for key in ("time_s", "rocketpy_time_s", "x", "y", "z")}
        except (KeyError, TypeError, ValueError):
            continue
        if all(np.isfinite(value) for value in event.values()):
            return event
    return None


def _launch_elevation_m(metadata: dict[str, Any] | None, altitude_asl_m: np.ndarray) -> float:
    """Launch-site elevation ASL, used to turn every plotted altitude into AGL.
    Falls back to the first sample (rocket on the pad) for captures without it."""
    elevation = (metadata or {}).get("environment_elevation_m")
    if elevation is not None:
        return float(elevation)
    return float(altitude_asl_m[0]) if len(altitude_asl_m) else 0.0


def _apogee_agl(rocketpy_apogee: dict[str, float] | None, elevation_m: float) -> dict[str, float] | None:
    if rocketpy_apogee is None:
        return None
    return {**rocketpy_apogee, "z": rocketpy_apogee["z"] - elevation_m}


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
        if isinstance(item, (list, tuple)) and len(item) == 2:
            try:
                out.append((float(item[0]), float(item[1])))
            except (TypeError, ValueError):
                continue

    return out


def _sanitize_time_eclipses(raw_events: Any) -> list[dict[str, Any]]:
    """Normalize capture time-eclipse markers.

    A time eclipse is an omitted interval in the saved capture time axis. The
    simulator may still have sent those samples to the FC; the plot marker only
    tells the reader that the displayed time was compacted at ``time_s``.
    """
    if raw_events is None:
        return []

    out: list[dict[str, Any]] = []

    for item in raw_events:
        if not isinstance(item, dict):
            continue

        try:
            time_s = float(item["time_s"])
        except (KeyError, TypeError, ValueError):
            continue

        try:
            omitted_duration_s = float(item.get("omitted_duration_s", 0.0))
        except (TypeError, ValueError):
            omitted_duration_s = 0.0

        try:
            omitted_samples = int(item.get("omitted_samples", 0))
        except (TypeError, ValueError):
            omitted_samples = 0

        out.append(
            {
                "time_s": time_s,
                "label": str(item.get("label", "omitted")),
                "omitted_duration_s": max(0.0, omitted_duration_s),
                "omitted_samples": max(0, omitted_samples),
            }
        )

    out.sort(key=lambda x: x["time_s"])
    return out


def _sanitize_fsm_events(raw_events: Any) -> list[tuple[float, str]]:
    """Normalize current FSM event entries: ``[time, state_name]``."""
    if raw_events is None:
        return []

    out: list[tuple[float, str]] = []

    for item in raw_events:
        if not isinstance(item, (list, tuple)) or len(item) != 2:
            continue

        event_t, state = item

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
    """Set equal physical scaling on all three axes and return the plot span.

    Matplotlib 3D axes normally scale X/Y/Z independently, which can make a
    steep climb look shallow or exaggerate lateral drift. Here we compute the
    bounding box of all finite trajectory points, expand it to a cube, and use
    that same span on all axes. The returned span is also used to size the body
    and sensor glyphs relative to the visible flight volume.
    """
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


def _mark_time_eclipses(
    ax,
    eclipses: list[dict[str, Any]],
    sim_time_s: np.ndarray,
) -> None:
    """Draw compacted-time markers on a time-series axis."""
    if not eclipses or len(sim_time_s) == 0:
        return

    x_min = float(sim_time_s[0])
    x_max = float(sim_time_s[-1])
    span = max(x_max - x_min, 1.0)
    half_width = max(span * 0.003, 0.01)
    first = True

    for eclipse in eclipses:
        time_s = float(eclipse["time_s"])
        if time_s < x_min - half_width or time_s > x_max + half_width:
            continue

        label = (
            "Ground Services omitted"
            if first
            else None
        )
        ax.axvspan(
            time_s - half_width,
            time_s + half_width,
            color="0.5",
            alpha=0.16,
            label=label,
            zorder=0,
        )
        ax.axvline(time_s, color="0.35", linestyle="--", linewidth=1.2)

        omitted_duration_s = float(eclipse.get("omitted_duration_s", 0.0))
        if omitted_duration_s > 0.0:
            ax.annotate(
                f"{eclipse.get('label', 'omitted')}\nomitted {omitted_duration_s:.1f}s",
                xy=(time_s, 1.0),
                xycoords=("data", "axes fraction"),
                xytext=(4, -4),
                textcoords="offset points",
                va="top",
                fontsize=8,
                color="0.25",
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
            zorder=4,
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
# AIRBRAKES-ON-TRAJECTORY PLOT
# ----------------------------------------------------------------------

# The trajectory keeps its default colour while the brakes are closed; any
# opening recolours that stretch of the same curve with a light->dark orange
# ramp, so "slightly open" never looks like "closed".
AIRBRAKE_TRAJ_COLOR = "C0"
AIRBRAKE_TRAJ_LINEWIDTH = 1.8


def _airbrake_cmap():
    from matplotlib.colors import ListedColormap

    base = plt.get_cmap("Oranges")
    cmap = ListedColormap(base(np.linspace(0.35, 1.0, 256)), name="airbrakes")
    cmap.set_under(AIRBRAKE_TRAJ_COLOR)
    return cmap


def _airbrake_norm():
    from matplotlib.colors import Normalize

    # vmin slightly above zero routes exactly-closed samples to set_under().
    return Normalize(vmin=1e-6, vmax=1.0)


def _airbrake_level_per_sample(
    sim_time_s: np.ndarray,
    airbrake_events: list[tuple[float, float]],
) -> np.ndarray:
    """Hold each commanded level until the next command (step / zero-order hold)."""
    levels = np.zeros_like(sim_time_s, dtype=float)
    if not airbrake_events:
        return levels

    events = sorted(airbrake_events, key=lambda e: e[0])
    event_t = np.array([t for t, _ in events])
    event_lvl = np.clip(np.array([lvl for _, lvl in events]), 0.0, 1.0)
    # side="right": a command at t applies to the sample at t; for duplicate
    # timestamps the last command wins.
    idx = np.searchsorted(event_t, sim_time_s, side="right") - 1
    active = idx >= 0
    levels[active] = event_lvl[idx[active]]
    return levels


def _draw_altitude_context(
    ax,
    sim_time_s: np.ndarray,
    altitude_m: np.ndarray,
    fsm_events: list[tuple[float, str]],
    time_eclipses: list[dict[str, Any]],
    rocketpy_apogee: dict[str, float] | None,
    apogee_idx: int | None,
    *,
    draw_line: bool = True,
    annotate_fsm: bool = True,
) -> None:
    """Shared altitude-vs-time scaffolding: curve, START/END, apogee, FSM keypoints."""
    if draw_line:
        ax.plot(sim_time_s, altitude_m, color=AIRBRAKE_TRAJ_COLOR, linewidth=AIRBRAKE_TRAJ_LINEWIDTH,
                label="GPS altitude AGL")

    ax.scatter(sim_time_s[0], altitude_m[0], s=50, marker="o", color="k", zorder=5, label="START")
    ax.scatter(sim_time_s[-1], altitude_m[-1], s=50, marker="X", color="k", zorder=5, label="END")
    if apogee_idx is not None and rocketpy_apogee is not None:
        ax.scatter(
            rocketpy_apogee["time_s"], altitude_m[apogee_idx],
            s=70, marker="^", color="gold", edgecolors="k", zorder=6, label="RocketPy apogee time",
        )

    _mark_fsm_event_lines(ax, fsm_events)
    _mark_time_eclipses(ax, time_eclipses, sim_time_s)
    _mark_fsm_points_2d(ax, sim_time_s, sim_time_s, altitude_m, fsm_events, annotate=annotate_fsm)

    ax.set_xlabel("Simulation time [s]")
    ax.set_ylabel("Altitude AGL [m]")
    ax.grid(True, alpha=0.4)


def _colored_altitude_line(ax, sim_time_s, altitude_m, levels):
    """The altitude trajectory itself, drawn once: default style where the brakes
    are closed, orange ramp where they are open. Replaces the plain ax.plot line."""
    from matplotlib.collections import LineCollection

    # One continuous polyline per constant-level run (not one per sample pair):
    # per-pair segments antialias at their seams and make the curve look washed out.
    points = np.column_stack([sim_time_s, altitude_m])
    change = np.flatnonzero(np.diff(levels) != 0.0) + 1
    starts = np.concatenate(([0], change))
    ends = np.concatenate((change, [len(levels) - 1]))
    # Each run ends on the next run's first sample so the curve has no gaps.
    runs = [points[s:e + 1] for s, e in zip(starts, ends) if e > s]
    run_levels = [levels[s] for s, e in zip(starts, ends) if e > s]
    lc = LineCollection(
        runs,
        cmap=_airbrake_cmap(),
        norm=_airbrake_norm(),
        linewidth=AIRBRAKE_TRAJ_LINEWIDTH,
        joinstyle="round",
        capstyle="butt",
        zorder=2,  # same layer as a normal line: markers stay on top
        label="GPS altitude AGL",
    )
    lc.set_array(np.asarray(run_levels, dtype=float))
    ax.add_collection(lc)
    ax.autoscale_view()
    return lc


def _add_reference_y_tick(ax, value: float, *, decimals: int = 1) -> None:
    """Always show ``value`` as a y tick (with its exact number), dropping automatic
    ticks too close to it so labels never overlap. Survives zoom/pan."""
    from matplotlib.ticker import AutoLocator, FuncFormatter, Locator

    class _WithReference(Locator):
        def __init__(self):
            self._base = AutoLocator()

        def __call__(self):
            vmin, vmax = sorted(self.axis.get_view_interval())
            ticks = [t for t in self._base.tick_values(vmin, vmax)
                     if abs(t - value) > 0.045 * (vmax - vmin)]
            if vmin <= value <= vmax:
                ticks.append(value)
            return sorted(ticks)

    ax.yaxis.set_major_locator(_WithReference())
    ax.yaxis.set_major_formatter(FuncFormatter(
        lambda v, _: f"{v:.{decimals}f}" if abs(v - value) < 1e-9 else f"{v:g}"))


def _airbrake_colorbar(fig, mappable, cax):
    cbar = fig.colorbar(mappable, cax=cax, extend="min", extendfrac=0.03)
    cbar.set_label("Airbrakes (blue = closed)", fontsize=8, labelpad=2)
    cbar.set_ticks([0.25, 0.5, 0.75, 1.0])
    cbar.set_ticklabels(["25%", "50%", "75%", "100%"])
    cbar.ax.tick_params(labelsize=7, length=2, pad=1)
    cbar.outline.set_linewidth(0.5)
    return cbar


def _plot_airbrake_trajectory(
    sim_time_s: np.ndarray,
    altitude_m: np.ndarray,
    airbrake_events: list[tuple[float, float]],
    fsm_events: list[tuple[float, str]],
    time_eclipses: list[dict[str, Any]],
    rocketpy_apogee: dict[str, float] | None,
    apogee_idx: int | None,
) -> None:
    """Altitude vs time coloured by airbrake opening, FSM keypoints, and a
    deployment-level strip below on the same time axis."""
    levels = _airbrake_level_per_sample(sim_time_s, airbrake_events)

    fig = plt.figure(figsize=(13, 7))
    # Narrow third column holds the colorbar so both rows keep identical x extents.
    grid = fig.add_gridspec(
        2, 2,
        height_ratios=[3.5, 1],
        width_ratios=[1, 0.008],
        hspace=0.06,
        wspace=0.008,
    )
    ax_alt = fig.add_subplot(grid[0, 0])
    ax_lvl = fig.add_subplot(grid[1, 0], sharex=ax_alt)
    cax = fig.add_subplot(grid[0, 1])

    _draw_altitude_context(
        ax_alt,
        sim_time_s,
        altitude_m,
        fsm_events,
        time_eclipses,
        rocketpy_apogee,
        apogee_idx,
        draw_line=False,
    )
    lc = _colored_altitude_line(ax_alt, sim_time_s, altitude_m, levels)
    _airbrake_colorbar(fig, lc, cax)
    if rocketpy_apogee is not None:
        # Real (RocketPy) apogee, AGL like the GPS altitude on this axis.
        ax_alt.axhline(rocketpy_apogee["z"], color="0.45", linestyle="--", linewidth=0.8, zorder=1)
        _add_reference_y_tick(ax_alt, rocketpy_apogee["z"])
    ax_alt.set_xlabel("")
    ax_alt.tick_params(labelbottom=False)
    ax_alt.set_title("Altitude and Airbrake Deployment")
    _set_legend_if_needed(ax_alt, loc="upper right", fontsize=8)

    ax_lvl.fill_between(sim_time_s, levels * 100.0, step="post", color="tab:orange", alpha=0.35, linewidth=0)
    ax_lvl.step(sim_time_s, levels * 100.0, where="post", color="tab:orange", linewidth=1.6)
    _mark_fsm_event_lines(ax_lvl, fsm_events)
    ax_lvl.set_ylim(-5, 105)
    ax_lvl.set_yticks([0, 50, 100])
    ax_lvl.set_ylabel("Airbrakes [%]")
    ax_lvl.set_xlabel("Simulation time [s]")
    ax_lvl.grid(True, alpha=0.4)

    fig.subplots_adjust(left=0.06, right=0.955, top=0.94, bottom=0.08)


# ----------------------------------------------------------------------
# FC ALTITUDE ESTIMATOR VS SIMULATOR TRUTH
# ----------------------------------------------------------------------

EST_TRUTH_COLOR = "0.1"
EST_RAW_COLOR = "0.72"
EST_ESTIMATE_COLOR = "C0"
EST_ERROR_COLOR = "C3"


def _sanitize_fc_estimates(raw_events: Any) -> np.ndarray:
    """Rows of [capture_time_s, filtered_pressure_pa, altitude_m, vertical_velocity_mps]."""
    rows = []
    for item in raw_events or []:
        if isinstance(item, (list, tuple)) and len(item) == 4:
            try:
                row = [float(value) for value in item]
            except (TypeError, ValueError):
                continue
            if all(np.isfinite(row)):
                rows.append(row)
    rows.sort(key=lambda row: row[0])
    return np.asarray(rows, dtype=float).reshape(-1, 4)


def _interp_or_nan(x: np.ndarray, xp: np.ndarray, fp: np.ndarray) -> np.ndarray:
    out = np.interp(x, xp, fp)
    out[(x < xp[0]) | (x > xp[-1])] = np.nan
    return out


def _plot_estimator_vs_truth(
    hil_log: dict[str, list[Any]],
    metadata: dict[str, Any],
    estimates: np.ndarray,
    fsm_events: list[tuple[float, str]],
    time_eclipses: list[dict[str, Any]],
) -> None:
    """Altitude, vertical velocity and pressure: simulator truth vs what the FC
    computed, each with an error lane (estimate − truth, RMSE, bias) below."""
    sim_time_s = _as_array(hil_log, "sim_time_s")
    z_asl = _as_array(hil_log, "z")
    elevation_m = _launch_elevation_m(metadata, z_asl)
    if "true_pressure_pa" in hil_log and len(hil_log["true_pressure_pa"]) == len(sim_time_s):
        true_pressure = _as_array(hil_log, "true_pressure_pa")
        pressure_truth_label = "real (simulator)"
    else:
        # Captures recorded before true_pressure_pa existed: ISA at the truth altitude.
        true_pressure = 101325.0 * np.power(1.0 - 2.25577e-5 * z_asl, 5.25588)
        pressure_truth_label = "real (ISA approx., old capture)"
    raw_pressure = _as_array(hil_log, "pressure_pa")

    est_t = estimates[:, 0]
    quantities = (
        # title, unit, FC estimate, truth series, estimate label
        ("Altitude AGL", "m", estimates[:, 2], z_asl - elevation_m, "FC altitude estimate"),
        ("Vertical velocity", "m/s", estimates[:, 3], _as_array(hil_log, "vz"), "FC OLS velocity"),
        ("Pressure", "Pa", estimates[:, 1], true_pressure, "FC filtered pressure"),
    )

    fig, axes = plt.subplots(
        6, 1, figsize=(13, 14), sharex=True,
        gridspec_kw={"height_ratios": [3, 1, 3, 1, 3, 1]},
    )
    for row, (title, unit, estimate, truth, label) in enumerate(quantities):
        ax_value, ax_error = axes[2 * row], axes[2 * row + 1]
        is_pressure = title == "Pressure"

        if is_pressure:
            ax_value.plot(sim_time_s, raw_pressure, color=EST_RAW_COLOR, linewidth=0.7,
                          label="raw pressure sent to FC")
        ax_value.plot(sim_time_s, truth, color=EST_TRUTH_COLOR, linewidth=1.2,
                      label=pressure_truth_label if is_pressure else "real (simulator)")
        ax_value.plot(est_t, estimate, color=EST_ESTIMATE_COLOR, linewidth=1.3, label=label)
        if is_pressure:
            # Scale on the useful signals; raw noise may clip at the edges.
            low = np.nanmin([np.nanmin(truth), np.nanmin(estimate)])
            high = np.nanmax([np.nanmax(truth), np.nanmax(estimate)])
            margin = 0.05 * (high - low) + 1.0
            ax_value.set_ylim(low - margin, high + margin)
        _mark_fsm_event_lines(ax_value, fsm_events)
        _mark_time_eclipses(ax_value, time_eclipses, sim_time_s)
        ax_value.set_title(f"{title}: real vs FC computed")
        ax_value.set_ylabel(f"[{unit}]")
        ax_value.grid(True, alpha=0.4)
        _set_legend_if_needed(ax_value, loc="upper right", fontsize=8)

        error = estimate - _interp_or_nan(est_t, sim_time_s, truth)
        ax_error.axhline(0.0, color="0.4", linewidth=0.8)
        ax_error.plot(est_t, error, color=EST_ERROR_COLOR, linewidth=1.0)
        _mark_fsm_event_lines(ax_error, fsm_events)
        valid = np.isfinite(error)
        if valid.any():
            rmse = float(np.sqrt(np.mean(error[valid] ** 2)))
            bias = float(np.mean(error[valid]))
            ax_error.text(0.995, 0.94, f"RMSE {rmse:.2f} {unit} · bias {bias:+.2f} {unit}",
                          transform=ax_error.transAxes, ha="right", va="top", fontsize=8,
                          bbox={"boxstyle": "round,pad=0.25", "facecolor": "white", "edgecolor": "0.8"})
        ax_error.set_ylabel(f"error [{unit}]")
        ax_error.grid(True, alpha=0.4)

    axes[-1].set_xlabel("Simulation time [s]")
    fig.tight_layout()


# ----------------------------------------------------------------------
# AIRBRAKES REFERENCE PREDICTORS
# ----------------------------------------------------------------------
# Two apogee predictors used to judge the FC airbrake controller, both
# vectorised over many start states at once:
#
# * fc_model_apogee: a port of AirbrakesController::predictApogee
#   Fed with the FC inputs it must reproduce the FC prediction,
#   fed with the simulator truth it shows what
#   the controller would predict with a perfect state estimate.
#
# * AirbrakesTruthModel.physics_apogee: a 3D point-mass coast built from what
#   RocketPy actually simulates. Started from the true state it is the reference
#   "real" apogee for a given brake schedule. Replaying the brake levels
#   RocketPy really applied must land on the RocketPy apogee, which validates
#   the reference itself.
#

# Mirrors FC config.h
FC_GRAVITY = np.float32(9.80665)
ISA_T0_K = np.float32(288.15)
ISA_P0_PA = np.float32(101325.0)
ISA_LAPSE_RATE_K_PER_M = np.float32(0.0065)
ISA_GAS_CONSTANT = np.float32(287.058)
ISA_GAMMA = np.float32(1.4)
ISA_TROPOPAUSE_M = np.float32(11000.0)


def isa_atmosphere(altitude_msl_m):
    """Firmware ``isaAtmosphere``: density [kg/m^3] and speed of sound [m/s]."""
    h = np.clip(np.asarray(altitude_msl_m, dtype=np.float32), np.float32(0.0), ISA_TROPOPAUSE_M)
    exponent = FC_GRAVITY / (ISA_GAS_CONSTANT * ISA_LAPSE_RATE_K_PER_M)
    temperature = ISA_T0_K - ISA_LAPSE_RATE_K_PER_M * h
    pressure = ISA_P0_PA * np.power(temperature / ISA_T0_K, exponent)
    density = pressure / (ISA_GAS_CONSTANT * temperature)
    speed_of_sound = np.sqrt(ISA_GAMMA * ISA_GAS_CONSTANT * temperature)
    return density.astype(np.float32), speed_of_sound.astype(np.float32)


def fc_model_apogee(
    altitude_agl_m,
    vertical_velocity_mps,
    cd_row,
    mach_bp,
    *,
    dry_mass_kg: float,
    reference_area_m2: float,
    launch_site_altitude_m: float,
    prediction_dt_s: float,
    prediction_max_steps: int,
):
    """Vectorised ``predictApogee``. Returns (apogee_agl_m, time_to_apogee_s).

    ``cd_row`` is the Cd(Mach) row of one opening, shared by every sample.
    """
    h = np.array(altitude_agl_m, dtype=np.float32, copy=True)
    v = np.array(vertical_velocity_mps, dtype=np.float32, copy=True)
    t = np.zeros_like(h)
    dt = np.float32(prediction_dt_s)
    drag_factor = np.float32(0.5) * np.float32(reference_area_m2) / np.float32(dry_mass_kg)
    launch_alt = np.float32(launch_site_altitude_m)
    mach_bp = np.asarray(mach_bp, dtype=np.float32)
    cd_row = np.asarray(cd_row, dtype=np.float32)

    active = np.isfinite(h) & np.isfinite(v)
    for _ in range(int(prediction_max_steps)):
        if not active.any():
            break
        rho, sos = isa_atmosphere(h[active] + launch_alt)
        va = v[active]
        cd = np.interp(va / sos, mach_bp, cd_row).astype(np.float32)
        a = -drag_factor * cd * rho * va * va - FC_GRAVITY
        va = va + a * dt
        v[active] = va
        h[active] = h[active] + va * dt
        t[active] = t[active] + dt
        idx = np.flatnonzero(active)
        active[idx[va < 0.0]] = False
    valid = np.isfinite(altitude_agl_m) & np.isfinite(vertical_velocity_mps)
    h[~valid] = np.nan
    return h.astype(float), t.astype(float)


def select_opening(candidate_apogee_m: np.ndarray, opening_bp_pct, target_apogee_m: float):
    """``computeCommand`` opening selection applied to precomputed per-row apogees.

    candidate_apogee_m has shape (n_samples, n_openings). Returns (opening_pct,
    predicted_apogee_m) per sample, using the firmware rules: first row at or
    below target wins, interpolated against the previous one; all rows above
    target -> fully open; closed already below target -> closed.
    """
    opening_bp_pct = np.asarray(opening_bp_pct, dtype=float)
    n_samples, n_open = candidate_apogee_m.shape
    opening = np.full(n_samples, np.nan)
    predicted = np.full(n_samples, np.nan)
    for i in range(n_samples):
        row = candidate_apogee_m[i]
        if not np.all(np.isfinite(row)):
            continue
        opening[i] = opening_bp_pct[-1]
        predicted[i] = row[-1]
        for k in range(n_open):
            if row[k] <= target_apogee_m:
                if k == 0:
                    opening[i] = opening_bp_pct[0]
                    predicted[i] = row[0]
                else:
                    w = (row[k - 1] - target_apogee_m) / (row[k - 1] - row[k])
                    opening[i] = opening_bp_pct[k - 1] + w * (opening_bp_pct[k] - opening_bp_pct[k - 1])
                    predicted[i] = target_apogee_m
                break
    return opening, predicted


class AirbrakesTruthModel:
    """Atmosphere/drag/mass tables of the RocketPy simulation, from capture metadata."""

    def __init__(self, tables: dict[str, Any]):
        self.altitude_asl_m = np.asarray(tables["altitude_asl_m"], dtype=float)
        self.density = np.asarray(tables["density_kg_m3"], dtype=float)
        self.speed_of_sound = np.asarray(tables["speed_of_sound_m_s"], dtype=float)
        self.wind_x = np.asarray(tables["wind_x_m_s"], dtype=float)
        self.wind_y = np.asarray(tables["wind_y_m_s"], dtype=float)
        self.gravity = np.asarray(tables["gravity_m_s2"], dtype=float)
        self.mach_grid = np.asarray(tables["mach_grid"], dtype=float)
        self.body_cd = np.asarray(tables["body_cd_power_off"], dtype=float)
        self.mass_kg = float(tables["burnout_mass_kg"])
        self.reference_area_m2 = float(tables["reference_area_m2"])
        self.airbrakes_reference_area_m2 = float(tables.get("airbrakes_reference_area_m2", self.reference_area_m2))
        self.burn_out_time_s = float(tables["burn_out_capture_time_s"])
        self.elevation_m = float(tables["elevation_m"])
        self.opening_bp_pct = np.asarray(tables["airbrakes_opening_bp_pct"], dtype=float)
        self.mach_bp = np.asarray(tables["airbrakes_mach_bp"], dtype=float)
        self.cd_table = np.asarray(tables["airbrakes_cd_table"], dtype=float)

    def atmosphere(self, z_asl):
        xp = self.altitude_asl_m
        return (
            np.interp(z_asl, xp, self.density),
            np.interp(z_asl, xp, self.speed_of_sound),
            np.interp(z_asl, xp, self.wind_x),
            np.interp(z_asl, xp, self.wind_y),
            np.interp(z_asl, xp, self.gravity),
        )

    def body_cd_at(self, mach):
        return np.interp(mach, self.mach_grid, self.body_cd)

    def brake_cd_increment(self, level, mach):
        """Same as hil_rocketpy.airbrakes_drag_function: table Cd minus the closed row, zero when closed."""
        level = np.clip(np.asarray(level, dtype=float), 0.0, 1.0)
        mach = np.asarray(mach, dtype=float)
        rows = np.array([np.interp(mach, self.mach_bp, row) for row in self.cd_table])  # (n_open, n)
        opening_pct = level * 100.0
        k = np.clip(np.searchsorted(self.opening_bp_pct, opening_pct, side="right") - 1, 0, len(self.opening_bp_pct) - 2)
        lo = self.opening_bp_pct[k]
        hi = self.opening_bp_pct[k + 1]
        w = np.clip((opening_pct - lo) / (hi - lo), 0.0, 1.0)
        n = np.arange(mach.size)
        flat = rows.reshape(len(self.opening_bp_pct), -1)
        kk = np.broadcast_to(k, mach.shape).reshape(-1)
        ww = np.broadcast_to(w, mach.shape).reshape(-1)
        cd = (1.0 - ww) * flat[kk, n] + ww * flat[kk + 1, n]
        out = (cd - flat[0, n]).reshape(mach.shape)
        return np.where(level > 0.0, out, 0.0)

    def drag_acceleration(self, z_asl, vx, vy, vz, level):
        """Drag acceleration vector [m/s^2] RocketPy applies (body drag + brake increment)."""
        rho, sos, wx, wy, _g = self.atmosphere(z_asl)
        ax_, ay_, az_ = vx - wx, vy - wy, vz
        speed = np.sqrt(ax_ * ax_ + ay_ * ay_ + az_ * az_)
        mach = speed / sos
        cd_area = (self.body_cd_at(mach) * self.reference_area_m2
                   + self.brake_cd_increment(level, mach) * self.airbrakes_reference_area_m2)
        k = -0.5 * rho * speed * cd_area / self.mass_kg
        return k * ax_, k * ay_, k * az_, mach, speed

    def physics_apogee(
        self,
        start_time_s,
        z_asl,
        vx,
        vy,
        vz,
        level_schedule: Callable[[np.ndarray, np.ndarray], np.ndarray],
        *,
        dt_s: float = 0.01,
        max_time_s: float = 120.0,
    ):
        """3D point-mass coast to apogee from many start states.

        ``level_schedule(indices, t)`` returns the brake level (0..1) for the
        start states ``indices`` at absolute capture times ``t``. Returns
        (apogee_agl_m, time_to_apogee_s).
        """
        t = np.asarray(start_time_s, dtype=float).copy()
        z = np.asarray(z_asl, dtype=float).copy()
        vx = np.asarray(vx, dtype=float).copy()
        vy = np.asarray(vy, dtype=float).copy()
        vz = np.asarray(vz, dtype=float).copy()
        t0 = t.copy()
        apogee = np.full_like(z, np.nan)
        tta = np.full_like(z, np.nan)
        active = np.isfinite(z) & np.isfinite(vz)
        # Already falling: apogee is "now".
        falling = active & (vz <= 0.0)
        apogee[falling] = z[falling]
        tta[falling] = 0.0
        active &= ~falling

        for _ in range(int(max_time_s / dt_s)):
            if not active.any():
                break
            i = np.flatnonzero(active)
            level = np.broadcast_to(np.asarray(level_schedule(i, t[i]), dtype=float), i.shape)

            def accel(zz, ux, uy, uz):
                dx, dy, dz, _m, _s = self.drag_acceleration(zz, ux, uy, uz, level)
                g = np.interp(zz, self.altitude_asl_m, self.gravity)
                return dx, dy, dz - g

            # RK2 (midpoint) on velocity and position.
            ax1, ay1, az1 = accel(z[i], vx[i], vy[i], vz[i])
            hx, hy, hz = vx[i] + 0.5 * dt_s * ax1, vy[i] + 0.5 * dt_s * ay1, vz[i] + 0.5 * dt_s * az1
            zm = z[i] + 0.5 * dt_s * vz[i]
            ax2, ay2, az2 = accel(zm, hx, hy, hz)
            vz_new = vz[i] + dt_s * az2
            z_new = z[i] + dt_s * hz
            vx[i] += dt_s * ax2
            vy[i] += dt_s * ay2

            crossed = vz_new <= 0.0
            if crossed.any():
                c = i[crossed]
                # Linear interpolation of the vz = 0 instant inside this step.
                frac = vz[c] / (vz[c] - vz_new[crossed])
                apogee[c] = z[c] + frac * (z_new[crossed] - z[c])
                tta[c] = (t[c] - t0[c]) + frac * dt_s
                active[c] = False
            vz[i] = vz_new
            z[i] = z_new
            t[i] += dt_s

        return apogee - self.elevation_m, tta


# ----------------------------------------------------------------------
# AIRBRAKES CONTROLLER INTERNALS VS SIMULATOR TRUTH
# ----------------------------------------------------------------------

# One colour per entity across every airbrake-analysis figure.
AB_TRUTH_COLOR = "0.1"       # RocketPy truth (state, real apogee)
AB_FC_COLOR = "C0"           # what the FC computed/reported
AB_FC_TRUE_COLOR = "C4"      # FC model fed with the true state (perfect estimator)
AB_PHYSICS_COLOR = "C2"      # reference predictor with the physics RocketPy simulates
AB_APPLIED_COLOR = "C1"      # brake level RocketPy actually applied
AB_ERROR_COLOR = "C3"
AB_TARGET_COLOR = "C3"


def _sanitize_airbrakes_telemetry(raw_events: Any) -> list[dict[str, Any]]:
    rows = []
    for item in raw_events or []:
        if not isinstance(item, dict):
            continue
        try:
            if not np.isfinite(float(item["time_s"])):
                continue
        except (KeyError, TypeError, ValueError):
            continue
        rows.append(item)
    rows.sort(key=lambda row: float(row["time_s"]))
    return rows


def _ab_column(rows: list[dict[str, Any]], key: str) -> np.ndarray:
    return np.array([np.nan if row.get(key) is None else float(row.get(key)) for row in rows], dtype=float)


def _ab_mark_fsm(ax, fsm_events: list[tuple[float, str]], t_min: float, t_max: float, *, annotate: bool) -> None:
    """FSM transitions as recessive grey lines (labelled once, on the top axis)."""
    for event_t, state in fsm_events:
        if not (t_min <= event_t <= t_max):
            continue
        ax.axvline(event_t, color="0.6", linestyle=":", linewidth=0.9, zorder=0)
        if annotate:
            ax.annotate(state, xy=(event_t, 1.0), xycoords=("data", "axes fraction"),
                        xytext=(2, -2), textcoords="offset points", fontsize=7, color="0.35",
                        rotation=90, ha="left", va="top")


def _ab_finish_axis(ax, title: str, ylabel: str, fsm_events, t_range, *, annotate_fsm: bool = False,
                    legend_loc: str = "best") -> None:
    _ab_mark_fsm(ax, fsm_events, *t_range, annotate=annotate_fsm)
    ax.set_title(title, fontsize=10, loc="left")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.35)
    _set_legend_if_needed(ax, loc=legend_loc, fontsize=7.5, framealpha=0.85)


def _ab_stats(error: np.ndarray, unit: str) -> str:
    valid = np.isfinite(error)
    if not valid.any():
        return "no data"
    return (f"mean {np.mean(error[valid]):+.1f} {unit} · RMSE {np.sqrt(np.mean(error[valid] ** 2)):.1f} {unit}"
            f" · max |{np.max(np.abs(error[valid])):.1f}| {unit}")


def _ab_note(ax, text: str) -> None:
    ax.text(0.995, 0.95, text, transform=ax.transAxes, ha="right", va="top", fontsize=7.5,
            bbox={"boxstyle": "round,pad=0.25", "facecolor": "white", "edgecolor": "0.8", "alpha": 0.9})


def _plot_airbrakes_internals(
    hil_log: dict[str, list[Any]],
    hil_events: dict[str, list[Any]],
    metadata: dict[str, Any],
    fsm_events: list[tuple[float, str]],
    rocketpy_apogee: dict[str, float] | None,
) -> None:
    """Airbrake controller internals (what the FC computed) against the simulator.

    One figure, apogee prediction vs truth: FC predictions, the same FC model
    fed with the true state, a reference predictor with RocketPy's real
    physics, the prediction error split into estimation and model parts, and
    commanded vs ideal opening. A console summary adds input errors and timing.

    The error decomposition uses the closed-brake apogee the FC predicted
    ("apogee_closed_m"; row 0 of "candidate_apogee_m" in older captures). When
    a capture has neither (e.g. the PID law), it falls back to the Python
    replica of the FC model fed with the FC inputs.
    """

    rows = _sanitize_airbrakes_telemetry(hil_events.get("airbrakes_telemetry"))
    if len(rows) < 2:
        print("[PLOT] No airbrake controller telemetry in this capture "
              "(needs firmware that reports it, and the opening altitude reached).")
        return

    params = metadata.get("airbrakes_controller_params") or {}
    truth_tables = metadata.get("airbrakes_truth_model")
    truth = AirbrakesTruthModel(truth_tables) if truth_tables else None
    if truth is None:
        print("[PLOT] Capture has no airbrakes_truth_model: physics reference curves skipped.")

    # ---------------- FC telemetry ----------------
    t = _ab_column(rows, "time_s")
    t_vel = _ab_column(rows, "velocity_time_s")
    fc_alt = _ab_column(rows, "altitude_m")
    fc_vz = _ab_column(rows, "vertical_velocity_mps")
    fc_dep = _ab_column(rows, "deployment")
    fc_apo = _ab_column(rows, "predicted_apogee_m")
    compute_us = _ab_column(rows, "compute_us")
    armed = np.array([bool(row.get("armed")) for row in rows])
    fresh = np.array([bool(row.get("velocity_fresh")) for row in rows])

    if truth is not None:
        opening_bp = truth.opening_bp_pct
        mach_bp = truth.mach_bp
        cd_table = truth.cd_table
    else:
        opening_bp = np.array([0.0, 20.0, 40.0, 60.0, 80.0, 100.0])
        mach_bp = cd_table = None
    n_open = len(opening_bp)

    def reported_closed_apogee(row):
        value = row.get("apogee_closed_m")
        if value is None and row.get("candidate_apogee_m"):
            value = row["candidate_apogee_m"][0]
        return np.nan if value is None else float(value)

    fc_closed_reported = np.array([reported_closed_apogee(row) for row in rows], dtype=float)

    target = params.get("target_apogee_m")
    opening_alt = params.get("opening_altitude_m")
    t_apogee = rocketpy_apogee["time_s"] if rocketpy_apogee is not None else None
    real_apogee = rocketpy_apogee["z"] if rocketpy_apogee is not None else None

    # ---------------- simulator truth at each control cycle ----------------
    sim_t = _as_array(hil_log, "sim_time_s")
    z_asl = _as_array(hil_log, "z")
    elevation_m = _launch_elevation_m(metadata, z_asl)
    order = np.argsort(sim_t, kind="stable")
    sim_t, z_asl = sim_t[order], z_asl[order]
    vx_log, vy_log, vz_log = (_as_array(hil_log, k)[order] for k in ("vx", "vy", "vz"))
    applied_log = (_as_array(hil_log, "airbrakes_applied_level")[order]
                   if len(hil_log.get("airbrakes_applied_level", [])) == len(order) else None)

    def truth_at(series, times):
        return _interp_or_nan(times, sim_t, series)

    z_true = truth_at(z_asl, t)
    z_true_agl = z_true - elevation_m
    vx_true, vy_true, vz_true = truth_at(vx_log, t), truth_at(vy_log, t), truth_at(vz_log, t)

    def applied_level_at(times):
        if applied_log is None:
            return _airbrake_level_per_sample(np.asarray(times, dtype=float),
                                              _sanitize_airbrake_events(hil_events.get("airbrakes", [])))
        idx = np.clip(np.searchsorted(sim_t, times, side="right") - 1, 0, len(sim_t) - 1)
        return applied_log[idx]

    applied_at_t = applied_level_at(t)

    # ---------------- FC model: replica on FC inputs, and on the true state ----------------
    have_params = all(params.get(k) for k in ("dry_mass_kg", "reference_area_m2", "prediction_dt_s",
                                              "prediction_max_steps")) and cd_table is not None
    fc_kwargs = dict(
        dry_mass_kg=params.get("dry_mass_kg"),
        reference_area_m2=params.get("reference_area_m2"),
        launch_site_altitude_m=params.get("launch_site_altitude_m", 0.0),
        prediction_dt_s=params.get("prediction_dt_s"),
        prediction_max_steps=params.get("prediction_max_steps"),
    )
    nan_rows = np.full((len(t), n_open), np.nan)
    replica_fc_inputs = nan_rows.copy()
    fc_model_true = nan_rows.copy()
    fc_model_true_tta = nan_rows.copy()
    if have_params:
        use = armed & fresh
        for k in range(n_open):
            apo, _ = fc_model_apogee(np.where(use, fc_alt, np.nan), np.where(use, fc_vz, np.nan),
                                         cd_table[k], mach_bp, **fc_kwargs)
            replica_fc_inputs[:, k] = apo
            rising = vz_true > 0.0
            apo, tta = fc_model_apogee(np.where(rising, z_true_agl, np.nan), np.where(rising, vz_true, np.nan),
                                           cd_table[k], mach_bp, **fc_kwargs)
            fc_model_true[:, k] = apo
            fc_model_true_tta[:, k] = tta

    # What the opening-scan law would command with a perfect estimator.
    fc_true_opening, fc_true_pred = (select_opening(fc_model_true, opening_bp, target)
                                     if have_params and target
                                     else (np.full(len(t), np.nan),) * 2)
    if opening_alt is not None:
        gated = (z_true_agl < opening_alt) | ~(vz_true > 0.0)
        fc_true_opening = np.where(gated & np.isfinite(fc_true_opening), opening_bp[0], fc_true_opening)

    # ---------------- physics reference on the true state ----------------
    phys_rows = nan_rows.copy()
    phys_cmd = np.full(len(t), np.nan)
    phys_replay = np.full(len(t), np.nan)
    ideal_opening = np.full(len(t), np.nan)
    true_mach = np.full(len(t), np.nan)
    if truth is not None:
        state = (t, z_true, vx_true, vy_true, vz_true)
        for k in range(n_open):
            level_k = opening_bp[k] / 100.0
            phys_rows[:, k], _ = truth.physics_apogee(*state, lambda i, tt, lv=level_k: lv)
        phys_cmd, _ = truth.physics_apogee(*state, lambda i, tt: fc_dep[i])
        phys_replay, _ = truth.physics_apogee(*state, lambda i, tt: applied_level_at(tt))
        if target:
            ideal_opening, _ = select_opening(phys_rows, opening_bp, target)

        true_mach = truth.drag_acceleration(z_true, vx_true, vy_true, vz_true, applied_at_t)[3]

    # ---------------- prediction used for the error decomposition ----------------
    fc_apo_armed = np.where(armed, fc_apo, np.nan)
    reported = bool(np.isfinite(fc_closed_reported).any())
    fc_closed = fc_closed_reported if reported else replica_fc_inputs[:, 0]
    fc_closed_label = ("FC: apogee if closed" if reported
                       else "FC model on FC inputs: apogee if closed (Python replica)")
    fc_pred, model_true_pred, phys_pred = fc_closed, fc_model_true[:, 0], phys_rows[:, 0]
    pred_name = "closed-brake"
    # Python port check: the replica on the FC inputs must reproduce what the FC reported.
    port_both = np.isfinite(fc_closed_reported) & np.isfinite(replica_fc_inputs[:, 0])
    port_max_diff = (float(np.max(np.abs(fc_closed_reported[port_both] - replica_fc_inputs[port_both, 0])))
                     if port_both.any() else None)

    # ---------------- time window ----------------
    active_t = t[armed] if armed.any() else t
    t_lo = float(t[0]) - 1.0
    t_hi = float(max(active_t[-1], t_apogee if t_apogee is not None else active_t[-1])) + 1.5
    t_range = (t_lo, t_hi)
    in_window = (t >= t_lo) & (t <= t_hi)

    # ================= summary on the console =================
    print("\n====== AIRBRAKES CONTROLLER ANALYSIS ======")
    print(f"Control cycles recorded:   {len(t)} ({armed.sum()} armed, {(~fresh).sum()} with stale velocity)")
    if target:
        print(f"Target apogee (FC):        {target:.1f} m AGL")
    if real_apogee is not None:
        miss = f" -> miss {real_apogee - target:+.1f} m" if target else ""
        print(f"Real apogee (RocketPy):    {real_apogee:.1f} m AGL{miss}")
    if truth is not None:
        print(f"FC dry mass vs RocketPy:   {params.get('dry_mass_kg', float('nan')):.3f} kg vs {truth.mass_kg:.3f} kg")
        print(f"FC ref area vs RocketPy:   {params.get('reference_area_m2', float('nan')):.6f} vs {truth.reference_area_m2:.6f} m^2")
        print(f"FC launch alt vs elevation:{params.get('launch_site_altitude_m', float('nan')):.1f} m vs {truth.elevation_m:.1f} m")
    if port_max_diff is not None:
        print(f"Python port vs FC prediction: max |diff| {port_max_diff:.2f} m")
    if truth is not None:
        val = phys_replay - real_apogee if real_apogee is not None else np.full(len(t), np.nan)
        print(f"Physics reference check (replay applied levels vs real apogee): {_ab_stats(val[armed], 'm')}")
        print(f"{pred_name.capitalize()} apogee, FC - physics (total):  {_ab_stats((fc_pred - phys_pred)[armed], 'm')}")
        print(f"   estimation part (FC inputs vs true state):   {_ab_stats((fc_pred - model_true_pred)[armed], 'm')}")
        print(f"   model part (FC model vs physics, true state):{_ab_stats((model_true_pred - phys_pred)[armed], 'm')}")
        print(f"Commanded - ideal opening: {_ab_stats((fc_dep * 100.0 - ideal_opening)[armed], '%')}")
    print(f"Altitude input error:      {_ab_stats((fc_alt - z_true_agl)[armed], 'm')}")
    print(f"Velocity input error:      {_ab_stats((fc_vz - vz_true)[armed], 'm/s')}")
    lat = (t - t_vel)[armed]
    if lat.size:
        print(f"Controller state age:      mean {np.nanmean(lat) * 1000:.0f} ms, max {np.nanmax(lat) * 1000:.0f} ms")
    print(f"Compute time:              mean {np.nanmean(compute_us[armed]) if armed.any() else 0:.0f} us, "
          f"max {np.nanmax(compute_us) if len(compute_us) else 0:.0f} us")
    print("===========================================\n")

    # ================= FIGURE 1: apogee prediction vs truth =================
    fig1 = plt.figure(figsize=(15, 12))
    grid = fig1.add_gridspec(3, 2, height_ratios=[3.2, 1.6, 1.6], width_ratios=[1, 0.27],
                             hspace=0.28, wspace=0.03)
    ax_apo = fig1.add_subplot(grid[0, 0])
    ax_err = fig1.add_subplot(grid[1, 0], sharex=ax_apo)
    ax_dep = fig1.add_subplot(grid[2, 0], sharex=ax_apo)
    ax_txt = fig1.add_subplot(grid[:, 1])
    ax_txt.axis("off")
    fig1.suptitle("Airbrakes controller: apogee prediction vs RocketPy truth", fontsize=13)

    # Authority bands: closed .. fully open.
    if np.isfinite(replica_fc_inputs).any():
        ax_apo.fill_between(t, replica_fc_inputs[:, 0], replica_fc_inputs[:, -1], color=AB_FC_COLOR, alpha=0.12,
                            linewidth=0, label="FC model reach, closed..100% (FC inputs)")
    if np.isfinite(phys_rows).any():
        ax_apo.fill_between(t, phys_rows[:, 0], phys_rows[:, -1], color=AB_PHYSICS_COLOR, alpha=0.14,
                            linewidth=0, label="true reach, closed..100% (physics, true state)")
    ax_apo.plot(t, fc_closed, color=AB_FC_COLOR, linewidth=1.6, label=fc_closed_label)
    ax_apo.plot(t, fc_apo_armed, color=AB_FC_COLOR, linewidth=1.2, linestyle="--",
                label="FC: apogee with commanded opening")
    ax_apo.plot(t, fc_model_true[:, 0], color=AB_FC_TRUE_COLOR, linewidth=1.3,
                label="FC model on true state: if closed")
    ax_apo.plot(t, phys_rows[:, 0], color=AB_PHYSICS_COLOR, linewidth=1.6, label="physics: apogee if closed")
    ax_apo.plot(t, phys_cmd, color=AB_PHYSICS_COLOR, linewidth=1.2, linestyle="--",
                label="physics: commanded opening held")
    ax_apo.plot(t, phys_replay, color=AB_TRUTH_COLOR, linewidth=0.9, linestyle=":",
                label="physics: replay of applied levels (check)")
    if real_apogee is not None:
        ax_apo.axhline(real_apogee, color=AB_TRUTH_COLOR, linewidth=1.4,
                       label=f"real apogee {real_apogee:.0f} m")
    if target:
        ax_apo.axhline(target, color=AB_TARGET_COLOR, linewidth=1.2, linestyle="-.", label=f"target {target:.0f} m")
    finite_apo = np.concatenate([a[in_window & armed] for a in (fc_closed, phys_rows[:, 0], phys_rows[:, -1],
                                                                 replica_fc_inputs[:, -1])])
    finite_apo = finite_apo[np.isfinite(finite_apo)]
    refs = [v for v in (real_apogee, target) if v is not None]
    if finite_apo.size or refs:
        lo = min(list(finite_apo) + refs)
        hi = max(list(finite_apo) + refs)
        pad = 0.08 * (hi - lo) + 20.0
        ax_apo.set_ylim(lo - pad, hi + pad)
    _ab_mark_fsm(ax_apo, fsm_events, *t_range, annotate=True)
    ax_apo.set_title("Predicted apogee (each curve = 'if this were held from now on')", fontsize=10, loc="left")
    ax_apo.set_ylabel("Apogee AGL [m]")
    ax_apo.grid(True, alpha=0.35)

    err_total = fc_pred - phys_pred
    err_est = fc_pred - model_true_pred
    err_model = model_true_pred - phys_pred
    ax_err.axhline(0.0, color="0.4", linewidth=0.8)
    ax_err.plot(t, err_total, color=AB_ERROR_COLOR, linewidth=1.5, label="total: FC − physics")
    ax_err.plot(t, err_est, color=AB_FC_COLOR, linewidth=1.1, label="estimation: FC inputs − true state (FC model)")
    ax_err.plot(t, err_model, color=AB_FC_TRUE_COLOR, linewidth=1.1, label="model: FC model − physics (true state)")
    _ab_note(ax_err, f"total {_ab_stats(err_total[armed], 'm')}\n"
                     f"estimation {_ab_stats(err_est[armed], 'm')}\nmodel {_ab_stats(err_model[armed], 'm')}")
    _ab_finish_axis(ax_err, f"{pred_name.capitalize()} apogee prediction error, decomposed "
                            "(> 0: FC over-predicts → opens too much)",
                    "Error [m]", fsm_events, t_range, legend_loc="lower left")

    ax_dep.step(t, fc_dep * 100.0, where="post", color=AB_FC_COLOR, linewidth=1.6, label="FC commanded")
    if applied_log is not None:
        mask = (sim_t >= t_lo) & (sim_t <= t_hi)
        ax_dep.step(sim_t[mask], applied_log[mask] * 100.0, where="post", color=AB_APPLIED_COLOR, linewidth=1.0,
                    label="RocketPy applied")
    ax_dep.plot(t, fc_true_opening, color=AB_FC_TRUE_COLOR, linewidth=1.1,
                label="FC model on true state (perfect estimator)")
    ax_dep.plot(t, ideal_opening, color=AB_PHYSICS_COLOR, linewidth=1.3, linestyle="--",
                label="ideal: physics hits target if held")
    ax_dep.set_ylim(-5, 110)
    _ab_finish_axis(ax_dep, "Airbrake opening", "Opening [%]", fsm_events, t_range, legend_loc="upper left")

    ax_dep.set_xlabel("Simulation time [s]")
    ax_apo.set_xlim(*t_range)
    for ax in (ax_apo, ax_err):
        ax.tick_params(labelbottom=False)

    # Parameter / result table.
    def fmt(value, spec):
        return "n/a" if value is None or (isinstance(value, float) and not np.isfinite(value)) else format(value, spec)

    lines = ["FC PARAMETERS vs SIMULATION", ""]
    lines.append(f"target apogee     {fmt(target, '.1f')} m")
    lines.append(f"opening altitude  {fmt(opening_alt, '.1f')} m")
    if truth is not None:
        lines.append(f"dry mass  FC      {fmt(params.get('dry_mass_kg'), '.3f')} kg")
        lines.append(f"          sim     {truth.mass_kg:.3f} kg")
        lines.append(f"ref area  FC      {fmt(params.get('reference_area_m2'), '.5f')} m²")
        lines.append(f"          sim     {truth.reference_area_m2:.5f} m²")
        lines.append(f"launch alt FC     {fmt(params.get('launch_site_altitude_m'), '.1f')} m")
        lines.append(f"elevation  sim    {truth.elevation_m:.1f} m")
        if cd_table is not None:
            m_ref = float(np.nanmedian(true_mach[armed])) if armed.any() and np.isfinite(true_mach[armed]).any() else 0.3
            lines.append(f"Cd closed @M{m_ref:.2f}")
            lines.append(f"  FC table        {np.interp(m_ref, mach_bp, cd_table[0]):.3f}")
            lines.append(f"  sim body        {float(truth.body_cd_at(m_ref)):.3f}")
    lines.append(f"pred. step / max  {fmt(params.get('prediction_dt_s'), '.3f')} s / {params.get('prediction_max_steps', 'n/a')}")
    lines += ["", "RESULT", ""]
    if real_apogee is not None:
        lines.append(f"real apogee       {real_apogee:.1f} m")
        if target:
            lines.append(f"miss vs target    {real_apogee - target:+.1f} m")
    if armed.any():
        gate_i = int(np.flatnonzero(armed)[0])
        lines.append(f"armed at t        {t[gate_i]:.2f} s")
        lines.append(f"  closed pred FC  {fmt(fc_closed[gate_i], '.0f')} m")
        lines.append(f"  closed physics  {fmt(phys_rows[gate_i, 0], '.0f')} m")
        lines.append(f"  full-open phys. {fmt(phys_rows[gate_i, -1], '.0f')} m")
        lines.append(f"mean opening      {np.nanmean(fc_dep[armed]) * 100:.0f} %")
        lines.append(f"mean ideal        {np.nanmean(ideal_opening[armed]):.0f} %" if np.isfinite(ideal_opening).any()
                     else "mean ideal        n/a")
    if port_max_diff is not None:
        lines += ["", f"port check: max |FC − Python|", f"  {port_max_diff:.2f} m"]
    ax_txt.text(0.02, 0.99, "\n".join(lines), transform=ax_txt.transAxes, ha="left", va="top",
                family="monospace", fontsize=8.2,
                bbox={"boxstyle": "round,pad=0.5", "facecolor": "0.97", "edgecolor": "0.8"})
    # The apogee panel is dense: its legend lives in the side column, under the table.
    handles, labels = ax_apo.get_legend_handles_labels()
    ax_txt.legend(handles, labels, loc="upper left", bbox_to_anchor=(0.0, 0.58), fontsize=7.5,
                  title="Predicted apogee panel (top)", title_fontsize=8, frameon=True, borderaxespad=0.0)
    fig1.subplots_adjust(left=0.06, right=0.99, top=0.94, bottom=0.05)


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
    Time slider
        Jump to a specific simulation time. This pauses the replay and resumes
        from the selected sample when Space is pressed.
    Space
        Pause or resume.
    R
        Restart from the first sample.
    """
    # Reference-frame convention used by the 3D replay:
    #     S_out -> S_clean -> B -> I
    # where S_out is the saved accelerometer payload after cross-axis mixing,
    # S_clean is the ideal orthogonal sensor frame, B is the RocketPy body frame,
    # and I is the RocketPy inertial frame (+X east, +Y north, +Z up).

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
    # Inertial Z shifted so the launch pad sits at 0 (AGL), like every other plot.
    elevation_m = _launch_elevation_m(metadata, state["z"])
    z = state["z"] - elevation_m
    accel_x_g = state["accel_x_m_s2"] / G0
    accel_y_g = state["accel_y_m_s2"] / G0
    accel_z_g = state["accel_z_m_s2"] / G0
    accel_norm_g = np.sqrt(
        state["accel_x_m_s2"] ** 2
        + state["accel_y_m_s2"] ** 2
        + state["accel_z_m_s2"] ** 2
    ) / G0
    pressure_hpa = state["pressure_pa"] / 100.0

    # Each RocketPy state stores attitude as a quaternion. Convert every sample
    # to a rotation matrix whose columns are the body axes expressed in the
    # inertial frame. Multiplying by this matrix changes coordinates from:
    #
    #     B body frame -> I inertial frame
    #
    # In other words, ``rotation @ [0, 0, 1]`` is where the rocket nose points
    # in world/inertial coordinates at that sample.
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
    # Static accelerometer mounting, saved in capture metadata:
    #
    #     S_clean -> B
    #
    # ``S_clean`` is the ideal orthogonal sensor frame before cross-axis mixing.
    # This matrix never changes during flight because the sensor is bolted to
    # the rocket body.
    accelerometer_sensor_to_body = _accelerometer_sensor_to_body_matrix(
        metadata
    )
    # The transmitted accelerometer payload is ``S_out``: sensor data after
    # cross-axis output mixing. For geometry/replay we first undo that mixing:
    #
    #     S_out -> S_clean
    accelerometer_output_to_clean_sensor = (
        _accelerometer_output_to_clean_sensor_matrix(metadata)
    )
    # Launchpad diagnostic. The first calibration sample is a stationary rocket,
    # so the accelerometer should measure specific force upward: about +1 g in
    # inertial Z. Reconstruct it through the full chain:
    #
    #     S_out -> S_clean -> B -> I
    #
    # If this points downward, the capture metadata and the saved acceleration
    # payload disagree about sensor orientation or cross-axis mixing.
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
    if first_accel_inertial[2] < 0.0:
        print(
            "[REPLAY WARNING] first reconstructed accelerometer vector points "
            "down in inertial Z. For stationary calibration it should point "
            "from earth to sky. Check capture metadata and attitude."
        )

    drogue_times = _sanitize_event_times(hil_events.get("open_drogue", []))
    main_times = _sanitize_event_times(hil_events.get("open_main", []))
    fsm_events = _sanitize_fsm_events(hil_events.get("fsm_state", []))
    time_eclipses = _sanitize_time_eclipses(hil_events.get("time_eclipses", []))
    rocketpy_apogee = _apogee_agl(_sanitize_rocketpy_apogee(hil_events.get("rocketpy_apogee")), elevation_m)

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

    if rocketpy_apogee is not None:
        ax.scatter(
            rocketpy_apogee["x"],
            rocketpy_apogee["y"],
            rocketpy_apogee["z"],
            s=70,
            marker="^",
            color="tab:purple",
            label="RocketPy apogee",
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
    _mark_time_eclipses(ax_accel, time_eclipses, sim_time_s)
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
    _mark_time_eclipses(ax_pressure, time_eclipses, sim_time_s)
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
        "Replay time slider: jump to a saved sample\n"
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
    ax.set_zlabel("RocketPy inertial Z AGL [m]")
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
        bottom=0.15,
        top=0.94,
    )

    slider_ax = fig.add_axes([0.12, 0.055, 0.72, 0.03])
    time_slider = Slider(
        slider_ax,
        "Replay time [s]",
        float(sim_time_s[0]),
        float(sim_time_s[-1]),
        valinit=float(sim_time_s[0]),
        valfmt="%.3f",
    )

    positive_periods = np.diff(sim_time_s)
    positive_periods = positive_periods[positive_periods > 0]
    mean_period_s = (
        float(np.mean(positive_periods)) if len(positive_periods) else 0.05
    )
    interval_ms = max(1.0, 1000.0 * mean_period_s / playback_speed)

    animation_state = {
        "paused": False,
        "legend_visible": False,
        "updating_slider": False,
    }

    def frame_sequence_from(frame_index: int):
        return iter(range(frame_index, expected_length))

    def update(frame_index: int):
        position = np.asarray(
            [x[frame_index], y[frame_index], z[frame_index]],
            dtype=float,
        )
        rotation = rotations[frame_index]
        current_time_s = sim_time_s[frame_index]

        animation_state["updating_slider"] = True
        time_slider.set_val(float(current_time_s))
        animation_state["updating_slider"] = False

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

        # Body axes in the 3D view.
        #
        # ``rotation`` is B -> I. Its columns are the inertial directions of
        # body +X, +Y and +Z. Scaling each column gives a visible axis glyph.
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

        # Sensor axes in the 3D view.
        #
        # The sensor mounting matrix is S_clean -> B. Composing it with the
        # current attitude gives S_clean -> I, so its columns are the displayed
        # clean sensor axes in inertial coordinates.
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
        # Reconstruct the accelerometer vector from the actual payload sent to
        # the FC. The stored packet is in ``S_out`` because RocketPy has already
        # applied cross-axis output mixing. For physical interpretation:
        #
        #     1. undo cross-axis mixing:      S_out -> S_clean
        #     2. rotate through the mounting: S_clean -> B
        #     3. rotate by attitude:          B -> I
        #
        # The magenta arrow is therefore the measured specific-force vector in
        # RocketPy inertial coordinates.
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
        # the trajectory. This means arrow direction is physically meaningful,
        # while arrow length is a visualization scale, not a raw SI magnitude.
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

    def jump_to_time(target_time_s: float) -> None:
        frame_index = _nearest_sample_index(sim_time_s, target_time_s)
        replay.event_source.stop()
        animation_state["paused"] = True
        replay.frame_seq = frame_sequence_from(frame_index)
        update(frame_index)
        fig.canvas.draw_idle()

    def on_slider_changed(target_time_s: float) -> None:
        if animation_state["updating_slider"]:
            return
        jump_to_time(float(target_time_s))

    time_slider.on_changed(on_slider_changed)

    def on_key_press(event) -> None:
        key = (event.key or "").lower()

        if key == " ":
            if animation_state["paused"]:
                replay.event_source.start()
            else:
                replay.event_source.stop()
            animation_state["paused"] = not animation_state["paused"]
        elif key == "r":
            replay.frame_seq = frame_sequence_from(0)
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
    metadata: dict[str, Any] | None = None,
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
    matching hil_rocketpy.py's current capture format. ``time_eclipses`` events
    are shown as narrow shaded markers at the compacted timestamp.
    """
    if hil_events is None:
        hil_events = {}
    if metadata is None:
        metadata = {}

    sim_time_s = _as_array(hil_log, "sim_time_s")

    if len(sim_time_s) == 0:
        print("[PLOT] No HIL samples logged.")
        return

    seq = _as_array(hil_log, "seq")

    accel_x_m_s2 = _as_array(hil_log, "accel_x_m_s2")
    accel_y_m_s2 = _as_array(hil_log, "accel_y_m_s2")
    accel_z_m_s2 = _as_array(hil_log, "accel_z_m_s2")
    pressure_pa = _as_array(hil_log, "pressure_pa")
    temperature_k = _as_array(hil_log, "temperature_k")
    latitude_deg = _as_array(hil_log, "latitude_deg")
    longitude_deg = _as_array(hil_log, "longitude_deg")
    # Every altitude plotted below is AGL (launch pad = 0 m).
    altitude_asl_m = _as_array(hil_log, "altitude_m")
    elevation_m = _launch_elevation_m(metadata, altitude_asl_m)
    altitude_m = altitude_asl_m - elevation_m

    is_fred_capture = metadata.get("rocket") in {"fred", "fred_ami"}
    launch_site = (metadata.get("environment_latitude"), metadata.get("environment_longitude"))
    fred_site_map = (
        is_fred_capture
        and launch_site in FRED_LAUNCH_SITES
    )
    if is_fred_capture and not fred_site_map:
        print("[PLOT] Villafranca map skipped: capture launch coordinates differ from its calibrated origin")
    gps_x, gps_y = _lat_lon_to_local_meters(
        latitude_deg,
        longitude_deg,
        launch_site if fred_site_map else None,
    )

    accel_x_g = accel_x_m_s2 / G0
    accel_y_g = accel_y_m_s2 / G0
    accel_z_g = accel_z_m_s2 / G0
    accel_norm_g = np.sqrt(accel_x_m_s2**2 + accel_y_m_s2**2 + accel_z_m_s2**2) / G0

    drogue_times = _sanitize_event_times(hil_events.get("open_drogue", []))
    main_times = _sanitize_event_times(hil_events.get("open_main", []))
    airbrake_events = _sanitize_airbrake_events(hil_events.get("airbrakes", []))
    fsm_events = _sanitize_fsm_events(hil_events.get("fsm_state", []))
    time_eclipses = _sanitize_time_eclipses(hil_events.get("time_eclipses", []))
    rocketpy_apogee = _apogee_agl(_sanitize_rocketpy_apogee(hil_events.get("rocketpy_apogee")), elevation_m)

    apogee_idx =(_nearest_sample_index(sim_time_s, rocketpy_apogee["time_s"])
                  if rocketpy_apogee is not None else None)
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

    print(f"Launch elevation:    {elevation_m:.3f} m ASL (altitudes below are AGL)")
    print(f"Max payload altitude:{np.max(altitude_m):9.3f} m AGL")
    if rocketpy_apogee is not None:
        print(f"RocketPy apogee:     {rocketpy_apogee['z']:.3f} m AGL at capture t={rocketpy_apogee['time_s']:.3f} s")
    else:
        print("RocketPy apogee:     unavailable in this capture")
    print(f"Max |accel|:         {np.max(accel_norm_g):.3f} g")
    print(f"Drogue events:       {drogue_times}")
    print(f"Main events:         {main_times}")
    print(f"Airbrake changes:    {airbrake_events}")
    print(f"FSM transitions:     {fsm_events}")
    print(f"Time eclipses:       {time_eclipses}")
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

    if apogee_idx is not None:
        ax_3d.scatter(
            gps_x[apogee_idx],
            gps_y[apogee_idx],
            altitude_m[apogee_idx],
            s=90,
            marker="^",
            label="RocketPy apogee time",
        )
        ax_3d.text(
            gps_x[apogee_idx],
            gps_y[apogee_idx],
            altitude_m[apogee_idx],
            f" RocketPy APOGEE\nt={rocketpy_apogee['time_s']:.2f}s\ntruth alt={rocketpy_apogee['z']:.1f}m",
        )

    _mark_event_points_3d(ax_3d, sim_time_s, gps_x, gps_y, altitude_m, drogue_times, "OPEN_DROGUE", "v")
    _mark_event_points_3d(ax_3d, sim_time_s, gps_x, gps_y, altitude_m, main_times, "OPEN_MAIN", "s")
    _mark_fsm_points_3d(ax_3d, sim_time_s, gps_x, gps_y, altitude_m, fsm_events)

    ax_3d.set_title("3D GPS Payload Trajectory Sent to Flight Controller")
    ax_3d.set_xlabel("local GPS x / east [m]")
    ax_3d.set_ylabel("local GPS y / north [m]")
    ax_3d.set_zlabel("GPS altitude AGL [m]")
    _set_legend_if_needed(ax_3d)
    fig_3d.tight_layout()

    # ------------------------------------------------------------------
    # Altitude + barometer pressure together
    # ------------------------------------------------------------------
    fig_alt_pressure, ax_alt = plt.subplots(figsize=(12, 5))
    ax_pressure = ax_alt.twinx()

    ax_alt.plot(sim_time_s, altitude_m, color="C0", label="GPS altitude AGL", linewidth=1.8)
    ax_pressure.plot(sim_time_s, pressure_pa, color="C1", label="barometer pressure", linewidth=1.2)

    ax_alt.scatter(sim_time_s[0], altitude_m[0], s=50, marker="o", color="k", zorder=5, label="START")
    ax_alt.scatter(sim_time_s[-1], altitude_m[-1], s=50, marker="X", color="k", zorder=5, label="END")
    if apogee_idx is not None:
        ax_alt.scatter(rocketpy_apogee["time_s"], altitude_m[apogee_idx], s=60, marker="^",
                       color="gold", edgecolors="k", zorder=6, label="RocketPy apogee time")

    _mark_event_lines(ax_alt, drogue_times, "OPEN_DROGUE")
    _mark_event_lines(ax_alt, main_times, "OPEN_MAIN")
    _mark_fsm_event_lines(ax_alt, fsm_events)
    _mark_time_eclipses(ax_alt, time_eclipses, sim_time_s)
    _mark_fsm_points_2d(ax_alt, sim_time_s, sim_time_s, altitude_m, fsm_events, annotate=True)

    ax_alt.set_title("Altitude and Barometer Pressure Sent to FC")
    ax_alt.set_xlabel("Simulation time [s]")
    ax_alt.set_ylabel("Altitude AGL [m]", color="C0")
    ax_pressure.set_ylabel("Pressure [Pa]", color="C1")
    ax_alt.grid(True)

    handles_alt, labels_alt = ax_alt.get_legend_handles_labels()
    handles_pressure, labels_pressure = ax_pressure.get_legend_handles_labels()
    # De-duplicate while preserving order.
    combined = {}
    for handle, label in zip(handles_alt + handles_pressure, labels_alt + labels_pressure):
        if label and label not in combined:
            combined[label] = handle
    ax_alt.legend(combined.values(), combined.keys(), loc="center right", fontsize=8)
    fig_alt_pressure.tight_layout()

    # ------------------------------------------------------------------
    # GPS ground track
    # ------------------------------------------------------------------
    # fig_track, ax_track = plt.subplots(figsize=(8, 8))
    # map_image_available = fred_site_map and FRED_SITE_IMAGE.is_file()
    # if map_image_available:
    #     # The JPEG extent is calibrated to the old pad; express it relative to this capture's pad.
    #     map_x, map_y = _lat_lon_to_local_meters(
    #         np.array([FRED_SITE_ORIGIN[0]]), np.array([FRED_SITE_ORIGIN[1]]), launch_site
    #     )
    #     west, east, south, north = FRED_SITE_EXTENT_M
    #     ax_track.imshow(
    #         plt.imread(FRED_SITE_IMAGE),
    #         extent=(west + map_x[0], east + map_x[0], south + map_y[0], north + map_y[0]),
    #         origin="upper",
    #     )
    # elif fred_site_map:
    #     print(f"[PLOT] Villafranca map image not found: {FRED_SITE_IMAGE}")
    # track_line, = ax_track.plot(gps_x, gps_y, label="GPS ground track")
    # if fred_site_map:
    #     track_line.set_color("magenta")
    #     track_line.set_linewidth(2)
    #     ax_track.scatter(0, 0, s=70, marker="o", color="white",
    #                      edgecolors="black", label="Launch point")
    #     ax_track.scatter(hil_log["x"][-1], hil_log["y"][-1], s=70,
    #                      marker="X", color="red", label="RocketPy landing")
    #     if rocketpy_apogee is not None:
    #         ax_track.scatter(rocketpy_apogee["x"], rocketpy_apogee["y"], s=70,
    #                          marker="^", color="yellow", edgecolors="black",
    #                          label="RocketPy apogee")
    # else:
    #     ax_track.scatter(gps_x[0], gps_y[0], s=50, marker="o", label="START")
    #     ax_track.scatter(gps_x[-1], gps_y[-1], s=50, marker="X", label="END")
    #     if apogee_idx is not None:
    #         ax_track.scatter(gps_x[apogee_idx], gps_y[apogee_idx], s=60, marker="^", label="RocketPy apogee time")
    # _mark_event_points_2d(ax_track, sim_time_s, gps_x, gps_y, drogue_times, "OPEN_DROGUE", "v")
    # _mark_event_points_2d(ax_track, sim_time_s, gps_x, gps_y, main_times, "OPEN_MAIN", "s")
    # _mark_fsm_points_2d(ax_track, sim_time_s, gps_x, gps_y, fsm_events, annotate=True)
    # ax_track.set_title("GPS Ground Track Sent to FC" +
    #                    (" (approximate Villafranca map)" if map_image_available else ""))
    # ax_track.set_xlabel("local GPS x / east [m]")
    # ax_track.set_ylabel("local GPS y / north [m]")
    # ax_track.axis("equal")
    # ax_track.grid(True)
    # _set_legend_if_needed(ax_track)
    # fig_track.tight_layout()

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
    _mark_time_eclipses(ax_acc, time_eclipses, sim_time_s)
    ax_acc.set_title("Accelerometer Payload Sent to FC")
    ax_acc.set_xlabel("Simulation time [s]")
    ax_acc.set_ylabel("Acceleration [g]")
    ax_acc.grid(True)
    _set_legend_if_needed(ax_acc)
    fig_accel.tight_layout()

    # ------------------------------------------------------------------
    # Temperature payload
    # ------------------------------------------------------------------
    # fig_temp, ax_temp = plt.subplots(figsize=(12, 4))
    # ax_temp.plot(sim_time_s, temperature_k, label="temperature [K]")
    # _mark_event_lines(ax_temp, drogue_times, "OPEN_DROGUE")
    # _mark_event_lines(ax_temp, main_times, "OPEN_MAIN")
    # _mark_fsm_event_lines(ax_temp, fsm_events)
    # _mark_time_eclipses(ax_temp, time_eclipses, sim_time_s)
    # ax_temp.set_title("Temperature Payload Sent to FC")
    # ax_temp.set_xlabel("Simulation time [s]")
    # ax_temp.set_ylabel("Temperature [K]")
    # ax_temp.grid(True)
    # _set_legend_if_needed(ax_temp)
    # fig_temp.tight_layout()

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
        _mark_time_eclipses(ax_fsm, time_eclipses, sim_time_s)

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
    # Altitude coloured by airbrake deployment + deployment strip
    # ------------------------------------------------------------------
    if len(airbrake_events) > 0:
        _plot_airbrake_trajectory(
            sim_time_s,
            altitude_m,
            airbrake_events,
            fsm_events,
            time_eclipses,
            rocketpy_apogee,
            apogee_idx,
        )

    # ------------------------------------------------------------------
    # FC altitude estimator vs simulator truth
    # ------------------------------------------------------------------
    fc_estimates = _sanitize_fc_estimates(hil_events.get("fc_estimates"))
    if len(fc_estimates) >= 2:
        _plot_estimator_vs_truth(hil_log, metadata, fc_estimates, fsm_events, time_eclipses)
    else:
        print("[PLOT] No FC altitude estimates in this capture (needs firmware that reports them).")

    # ------------------------------------------------------------------
    # Airbrake controller internals vs simulator truth
    # ------------------------------------------------------------------
    _plot_airbrakes_internals(hil_log, hil_events, metadata, fsm_events, rocketpy_apogee)

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
            metadata=metadata,
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
