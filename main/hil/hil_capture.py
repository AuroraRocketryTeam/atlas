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
# NUMERIC HELPERS
# ----------------------------------------------------------------------

def _as_array(log: dict[str, list[Any]], key: str) -> np.ndarray:
    if key not in log:
        raise KeyError(f"Missing key in hil_log: {key}")

    return np.asarray(log[key], dtype=float)


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


def _nearest_sample_index(t: np.ndarray, target_t: float) -> int:
    return int(np.argmin(np.abs(t - target_t)))


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


# ----------------------------------------------------------------------
# PLOTTING HELPERS
# ----------------------------------------------------------------------

def _mark_event_lines(ax, event_times: list[float], label: str) -> None:
    first = True

    for event_t in event_times:
        ax.axvline(
            event_t,
            linestyle="--",
            linewidth=1,
            label=label if first else None,
        )
        first = False


def _mark_event_points_2d(
    ax,
    t: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    event_times: list[float],
    label: str,
    marker: str,
) -> None:
    first = True

    for event_t in event_times:
        idx = _nearest_sample_index(t, event_t)
        ax.scatter(
            x[idx],
            y[idx],
            s=70,
            marker=marker,
            label=label if first else None,
        )
        first = False


def _mark_event_points_3d(
    ax,
    t: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    event_times: list[float],
    label: str,
    marker: str,
) -> None:
    first = True

    for event_t in event_times:
        idx = _nearest_sample_index(t, event_t)

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
            f" {label}\nt={t[idx]:.2f}s",
        )

        first = False


def _set_legend_if_needed(ax) -> None:
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend()


# ----------------------------------------------------------------------
# MAIN ANALYSIS / PLOT API
# ----------------------------------------------------------------------

def plot_hil_log(
    hil_log: dict[str, list[Any]],
    hil_events: dict[str, list[Any]] | None = None,
    *,
    sampling_rate: float | None = None,
    show: bool = True,
) -> None:
    """
    Plot a live or saved HIL capture.

    This intentionally shows only core HIL review plots:
      - mandatory 3D trajectory with event markers and event timing
      - altitude and barometer pressure together
      - GPS ground track
      - acceleration payload
      - airbrakes command, when present

    The sampling_rate argument is accepted for API compatibility with
    hil_rocketpy.py, but no packet dt plot is generated here.
    """
    del sampling_rate  # kept only for backward-compatible calls from hil_rocketpy.py

    if hil_events is None:
        hil_events = {}

    t = _as_array(hil_log, "t")

    if len(t) == 0:
        print("[PLOT] No HIL samples logged.")
        return

    seq = _as_array(hil_log, "seq")

    ax_data = _as_array(hil_log, "ax")
    ay_data = _as_array(hil_log, "ay")
    az_data = _as_array(hil_log, "az")
    pressure = _as_array(hil_log, "p")
    lat = _as_array(hil_log, "lat")
    lon = _as_array(hil_log, "lon")
    alt = _as_array(hil_log, "alt")

    gps_x, gps_y = _lat_lon_to_local_meters(lat, lon)

    ax_g = ax_data / G0
    ay_g = ay_data / G0
    az_g = az_data / G0
    accel_norm_g = np.sqrt(ax_data**2 + ay_data**2 + az_data**2) / G0

    drogue_times = _sanitize_event_times(hil_events.get("open_drogue", []))
    main_times = _sanitize_event_times(hil_events.get("open_main", []))
    airbrake_events = _sanitize_airbrake_events(hil_events.get("airbrakes", []))

    apogee_idx = int(np.argmax(alt))
    dt = np.diff(t)

    print("\n========== HIL DATA SUMMARY ==========")
    print(f"Samples sent:        {len(t)}")
    print(f"First seq:           {seq[0]:.0f}")
    print(f"Last seq:            {seq[-1]:.0f}")
    print(f"Start time:          {t[0]:.6f} s")
    print(f"End time:            {t[-1]:.6f} s")
    print(f"Duration:            {t[-1] - t[0]:.6f} s")

    if len(dt) > 0:
        print(f"Mean dt:             {np.mean(dt):.6f} s")
        print(f"Effective rate:      {1.0 / np.mean(dt):.3f} Hz")

    print(f"Max altitude:        {np.max(alt):.3f} m")
    print(f"Apogee time:         {t[apogee_idx]:.3f} s")
    print(f"Max |accel|:         {np.max(accel_norm_g):.3f} g")
    print(f"Drogue events:       {drogue_times}")
    print(f"Main events:         {main_times}")
    print(f"Airbrake changes:    {airbrake_events}")
    print("======================================\n")

    # ------------------------------------------------------------------
    # Mandatory 3D trajectory with event timing
    # ------------------------------------------------------------------
    fig_3d = plt.figure(figsize=(10, 8))
    ax_3d = fig_3d.add_subplot(111, projection="3d")

    ax_3d.plot(gps_x, gps_y, alt, label="GPS trajectory sent to FC")

    ax_3d.scatter(gps_x[0], gps_y[0], alt[0], s=90, marker="o", label="START")
    ax_3d.text(gps_x[0], gps_y[0], alt[0], f" START\nt={t[0]:.2f}s")

    ax_3d.scatter(gps_x[-1], gps_y[-1], alt[-1], s=90, marker="X", label="END")
    ax_3d.text(gps_x[-1], gps_y[-1], alt[-1], f" END\nt={t[-1]:.2f}s")

    ax_3d.scatter(
        gps_x[apogee_idx],
        gps_y[apogee_idx],
        alt[apogee_idx],
        s=90,
        marker="^",
        label="APOGEE",
    )
    ax_3d.text(
        gps_x[apogee_idx],
        gps_y[apogee_idx],
        alt[apogee_idx],
        f" APOGEE\nt={t[apogee_idx]:.2f}s\nalt={alt[apogee_idx]:.1f}m",
    )

    _mark_event_points_3d(ax_3d, t, gps_x, gps_y, alt, drogue_times, "OPEN_DROGUE", "v")
    _mark_event_points_3d(ax_3d, t, gps_x, gps_y, alt, main_times, "OPEN_MAIN", "s")

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

    alt_line = ax_alt.plot(t, alt, label="GPS altitude", linewidth=1.8)
    pressure_line = ax_pressure.plot(t, pressure, linestyle="--", label="barometer pressure")

    ax_alt.scatter(t[0], alt[0], s=50, marker="o", label="START")
    ax_alt.scatter(t[-1], alt[-1], s=50, marker="X", label="END")
    ax_alt.scatter(t[apogee_idx], alt[apogee_idx], s=60, marker="^", label="APOGEE")

    _mark_event_lines(ax_alt, drogue_times, "OPEN_DROGUE")
    _mark_event_lines(ax_alt, main_times, "OPEN_MAIN")

    ax_alt.set_title("Altitude and Barometer Pressure Sent to FC")
    ax_alt.set_xlabel("Simulation time [s]")
    ax_alt.set_ylabel("Altitude [m]")
    ax_pressure.set_ylabel("Pressure [Pa]")
    ax_alt.grid(True)

    handles_alt, labels_alt = ax_alt.get_legend_handles_labels()
    handles_pressure, labels_pressure = ax_pressure.get_legend_handles_labels()
    ax_alt.legend(handles_alt + handles_pressure, labels_alt + labels_pressure)
    fig_alt_pressure.tight_layout()

    # ------------------------------------------------------------------
    # GPS ground track
    # ------------------------------------------------------------------
    fig_track, ax_track = plt.subplots(figsize=(8, 8))
    ax_track.plot(gps_x, gps_y, label="GPS ground track")
    ax_track.scatter(gps_x[0], gps_y[0], s=50, marker="o", label="START")
    ax_track.scatter(gps_x[-1], gps_y[-1], s=50, marker="X", label="END")
    ax_track.scatter(gps_x[apogee_idx], gps_y[apogee_idx], s=60, marker="^", label="APOGEE")
    _mark_event_points_2d(ax_track, t, gps_x, gps_y, drogue_times, "OPEN_DROGUE", "v")
    _mark_event_points_2d(ax_track, t, gps_x, gps_y, main_times, "OPEN_MAIN", "s")
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
    ax_acc.plot(t, ax_g, label="ax [g]")
    ax_acc.plot(t, ay_g, label="ay [g]")
    ax_acc.plot(t, az_g, label="az [g]")
    ax_acc.plot(t, accel_norm_g, linestyle="--", label="|a| [g]")
    _mark_event_lines(ax_acc, drogue_times, "OPEN_DROGUE")
    _mark_event_lines(ax_acc, main_times, "OPEN_MAIN")
    ax_acc.set_title("Accelerometer Payload Sent to FC")
    ax_acc.set_xlabel("Simulation time [s]")
    ax_acc.set_ylabel("Acceleration [g]")
    ax_acc.grid(True)
    _set_legend_if_needed(ax_acc)
    fig_accel.tight_layout()

    # ------------------------------------------------------------------
    # Airbrakes command level
    # ------------------------------------------------------------------
    if len(airbrake_events) > 0:
        air_t = [t[0]]
        air_lvl = [0.0]

        for event_t, lvl in airbrake_events:
            air_t.append(event_t)
            air_lvl.append(lvl)

        air_t.append(t[-1])
        air_lvl.append(air_lvl[-1])

        fig_air, ax_air = plt.subplots(figsize=(12, 4))
        ax_air.step(air_t, air_lvl, where="post")
        ax_air.set_title("Airbrakes Command Received from FC")
        ax_air.set_xlabel("Simulation time [s]")
        ax_air.set_ylabel("Deployment level")
        ax_air.set_ylim(-0.05, 1.05)
        ax_air.grid(True)
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
