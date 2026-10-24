#!/usr/bin/env python3
"""Fetch FRED's GFS forecast while online and export a HIL JSONC call.

Run this with the Python environment used for HIL, before joining Manny's Wi-Fi:

    python data_analysis/export_fred_gfs_atmosphere.py --output fred_gfs_20261025.jsonc

Paste the generated object into Environment._calls after set_date, replacing the
online GFS call. No network access is needed when HIL uses that custom atmosphere.
"""

import argparse
import json
import math
import warnings
from datetime import datetime, timezone
from pathlib import Path
from zoneinfo import ZoneInfo


FRED_CONFIG = (
    Path(__file__).resolve().parents[1]
    / "main/hil/config/fred/fred_rocketpy_config.jsonc"
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path, help="New JSONC file to write")
    args = parser.parse_args()

    try:
        import jsonc
        from rocketpy import Environment
    except ImportError as exc:
        parser.error(f"Run with the HIL Python environment (missing {exc.name})")

    with FRED_CONFIG.open(encoding="utf-8") as source:
        environment_config = jsonc.load(source)["Environment"]

    date_calls = [
        call for call in environment_config["_calls"]
        if call.get("method") == "set_date"
    ]
    if len(date_calls) != 1:
        parser.error("FRED Environment._calls must contain exactly one set_date call")

    date_args = date_calls[0]["kwargs"]
    launch_date = tuple(date_args["date"])
    launch_timezone = date_args["timezone"]
    launch_local = datetime(*launch_date, tzinfo=ZoneInfo(launch_timezone))
    launch_utc = launch_local.astimezone(timezone.utc)

    environment_args = {
        key: value for key, value in environment_config.items()
        if not key.startswith("_")
    }
    env = Environment(**environment_args)
    env.set_date(launch_date, timezone=launch_timezone)
    try:
        with warnings.catch_warnings(record=True) as forecast_warnings:
            warnings.simplefilter("always")
            env.set_atmospheric_model(type="Forecast", file="GFS")
    except Exception as exc:
        parser.exit(1, f"GFS forecast unavailable for {launch_local.isoformat()}: {exc}\n")
    for warning in forecast_warnings:
        print(f"RocketPy warning: {warning.message}")
    forecast_time_note = next(
        (str(warning.message).replace("\n", " ") for warning in forecast_warnings
         if "Exact chosen launch time" in str(warning.message)),
        "RocketPy did not report a forecast time adjustment.",
    )

    profiles = {}
    for name, function in (
        ("pressure", env.pressure),
        ("temperature", env.temperature),
        ("wind_u", env.wind_velocity_x),
        ("wind_v", env.wind_velocity_y),
    ):
        profile = [[float(height), float(value)] for height, value in function.source]
        if len(profile) < 2 or any(
            not math.isfinite(height) or not math.isfinite(value)
            or (name in ("pressure", "temperature") and value <= 0)
            for height, value in profile
        ) or any(b[0] <= a[0] for a, b in zip(profile, profile[1:])):
            parser.error(f"GFS returned an invalid {name} height profile")
        profiles[name] = profile

    launch_height_m = float(environment_config["elevation"])
    required_top_m = launch_height_m + float(environment_config["max_expected_height"])
    if any(profile[-1][0] < required_top_m for profile in profiles.values()):
        parser.error(
            f"GFS profiles do not reach the expected flight height of {required_top_m:g} m ASL"
        )

    offline_env = Environment(**environment_args)
    offline_env.set_date(launch_date, timezone=launch_timezone)
    offline_env.set_atmospheric_model(type="custom_atmosphere", **profiles)
    for source_name, attribute in (
        ("pressure", "pressure"),
        ("temperature", "temperature"),
        ("wind_u", "wind_velocity_x"),
        ("wind_v", "wind_velocity_y"),
    ):
        for height_m in (launch_height_m, required_top_m):
            original = float(getattr(env, attribute)(height_m))
            reproduced = float(getattr(offline_env, attribute)(height_m))
            if not math.isclose(original, reproduced, rel_tol=1e-8, abs_tol=1e-6):
                parser.error(f"Offline {source_name} differs from GFS at {height_m:g} m ASL")

    atmosphere_call = {
        "method": "set_atmospheric_model",
        "kwargs": {"type": "custom_atmosphere", **profiles},
    }
    header = (
        f"// GFS profile fetched {datetime.now(timezone.utc).isoformat()} for requested launch "
        f"{launch_local.isoformat()} ({launch_utc.isoformat()} UTC).\n"
        f"// {forecast_time_note}\n"
        f"// Site: {environment_config['latitude']} N, {environment_config['longitude']} E. "
        "Profiles use RocketPy's forecast height levels in metres ASL.\n"
        "// Pressure: Pa; temperature: K; wind_u: east m/s; wind_v: north m/s.\n"
        "// Paste this object after set_date in Environment._calls, with a comma between calls.\n"
    )
    try:
        with args.output.open("x", encoding="utf-8") as destination:
            destination.write(header + json.dumps(atmosphere_call, indent=2) + "\n")
    except FileExistsError:
        parser.error(f"Output already exists: {args.output}")

    print(
        f"Wrote {args.output} for {launch_local.isoformat()} "
        f"({len(profiles['pressure'])} pressure levels)"
    )


if __name__ == "__main__":
    main()
