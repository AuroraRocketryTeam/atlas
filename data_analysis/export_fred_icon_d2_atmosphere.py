#!/usr/bin/env python3
"""Export an Open-Meteo ICON-D2 forecast as an offline RocketPy atmosphere.

Run online before joining Manny's Wi-Fi:
    python data_analysis/export_fred_icon_d2_atmosphere.py --output main/hil/config/weather/forecast_now.jsonc
    python data_analysis/export_fred_icon_d2_atmosphere.py --launch-time 17:00 --output main/hil/config/weather/forecast_1700.jsonc

Replace Environment._calls with the generated JSONC array, which contains
both the launch date and atmosphere. The raw response is saved beside it.
"""

import argparse
import json
import math
import urllib.parse
import urllib.request
from datetime import datetime, time, timedelta, timezone
from pathlib import Path
from zoneinfo import ZoneInfo


FRED_CONFIG = Path(__file__).resolve().parents[1] / "main/hil/config/fred/fred_rocketpy_config.jsonc"
NEAR_GROUND = (10, 80, 120, 180)  # metres above ground
PRESSURE_LEVELS = (1000, 975, 950, 925, 900, 850)  # hPa


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path, help="New JSONC file to write")
    parser.add_argument(
        "--launch-time",
        help="Rome time as HH:MM today or YYYY-MM-DDTHH:MM; an ISO UTC offset is also accepted (default: now)",
    )
    parser.add_argument("--height-agl", type=float, help="Required profile coverage above the pad in metres")
    args = parser.parse_args()
    raw_path = args.output.with_suffix(".source.json")
    if args.output.exists() or raw_path.exists():
        parser.error(f"Output already exists: {args.output if args.output.exists() else raw_path}")

    try:
        import jsonc
        from rocketpy import Environment
    except ImportError as exc:
        parser.error(f"Run with the HIL Python environment (missing {exc.name})")

    with FRED_CONFIG.open(encoding="utf-8") as source:
        config = jsonc.load(source)["Environment"]
    date_calls = [call for call in config["_calls"] if call.get("method") == "set_date"]
    if len(date_calls) != 1:
        parser.error("FRED Environment._calls must contain exactly one set_date call")
    launch_timezone = date_calls[0]["kwargs"]["timezone"]
    zone = ZoneInfo(launch_timezone)
    if args.launch_time is None:
        launch_local = datetime.now(zone).replace(second=0, microsecond=0)
    else:
        try:
            if len(args.launch_time) == 5 and args.launch_time[2] == ":":
                launch_local = datetime.combine(
                    datetime.now(zone).date(), time.fromisoformat(args.launch_time), zone
                )
            else:
                launch_local = datetime.fromisoformat(args.launch_time.replace("Z", "+00:00"))
                launch_local = (launch_local.replace(tzinfo=zone) if launch_local.tzinfo is None
                                else launch_local.astimezone(zone))
        except ValueError:
            parser.error("--launch-time must be HH:MM or ISO date and time")
        if launch_local.second or launch_local.microsecond:
            parser.error("--launch-time must have whole-minute precision")
    launch_date = [launch_local.year, launch_local.month, launch_local.day,
                   launch_local.hour, launch_local.minute]
    launch_utc = launch_local.astimezone(timezone.utc)
    first_hour = launch_utc.replace(minute=0, second=0, microsecond=0)
    next_hour = first_hour + timedelta(hours=1)
    fraction = (launch_utc - first_hour).total_seconds() / 3600
    hour = launch_utc.strftime("%Y-%m-%dT%H:%M")
    ground = float(config["elevation"])
    top = ground + args.height_agl if args.height_agl is not None else float(config["max_expected_height"])
    if not math.isfinite(top) or top <= ground:
        parser.error("Expected flight top must be above the launch elevation")

    near = [f"{name}_{height}m" for height in NEAR_GROUND for name in ("wind_speed", "wind_direction")]
    levels = [f"{name}_{level}hPa" for level in PRESSURE_LEVELS
              for name in ("wind_speed", "wind_direction", "temperature", "geopotential_height")]
    query = urllib.parse.urlencode({
        "latitude": config["latitude"], "longitude": config["longitude"], "models": "icon_d2",
        "hourly": ",".join(("temperature_2m", "surface_pressure", *near, *levels)),
        "wind_speed_unit": "ms", "timezone": "GMT",
        "start_date": first_hour.date().isoformat(),
        "end_date": (next_hour if fraction else first_hour).date().isoformat(),
    })
    url = f"https://api.open-meteo.com/v1/forecast?{query}"
    try:
        with urllib.request.urlopen(url, timeout=60) as response:
            data = json.load(response)
    except (OSError, ValueError) as exc:
        parser.exit(1, f"ICON-D2 download failed: {exc}\n")
    hourly = data.get("hourly", {})
    try:
        i = hourly["time"].index(first_hour.strftime("%Y-%m-%dT%H:00"))
        j = hourly["time"].index(next_hour.strftime("%Y-%m-%dT%H:00")) if fraction else i
    except (KeyError, ValueError):
        parser.error(f"ICON-D2 has no hourly records around {hour} UTC")

    def value(name):
        series = hourly.get(name, [])
        if j >= len(series) or series[i] is None or series[j] is None:
            return None
        a, b = series[i], series[j]
        return a + fraction * (b - a)

    def wind_vector(suffix):
        speed = hourly.get(f"wind_speed_{suffix}", [])
        direction = hourly.get(f"wind_direction_{suffix}", [])
        if j >= min(len(speed), len(direction)) or any(
            series[index] is None for series in (speed, direction) for index in (i, j)
        ):
            return None
        vectors = [(-speed[index] * math.sin(math.radians(direction[index])),
                    -speed[index] * math.cos(math.radians(direction[index])))
                   for index in (i, j)]
        return tuple(a + fraction * (b - a) for a, b in zip(*vectors))

    units = data.get("hourly_units", {})
    expected_units = {"temperature_2m": "°C", "surface_pressure": "hPa"}
    expected_units.update({name: "m/s" if name.startswith("wind_speed") else "°" for name in near})
    expected_units.update({name: "m" if name.startswith("geopotential_height") else
                           "°C" if name.startswith("temperature") else
                           "m/s" if name.startswith("wind_speed") else "°" for name in levels})
    if any(units.get(name) != unit for name, unit in expected_units.items()):
        parser.error("ICON-D2 response has missing or unexpected units")
    if any(value(name) is None or not math.isfinite(value(name))
           for name in ("temperature_2m", "surface_pressure", *near)):
        parser.error("ICON-D2 response is missing required near-ground values")

    # Convert meteorological direction to east/north before time interpolation.
    wind = [(ground, *wind_vector("10m"))]
    wind += [(ground + h, *wind_vector(f"{h}m")) for h in NEAR_GROUND]
    pressure = [(ground, value("surface_pressure") * 100)]
    temperature = [(ground, value("temperature_2m") + 273.15)]
    for level in PRESSURE_LEVELS:
        height = value(f"geopotential_height_{level}hPa")
        if height is None or height < ground:
            continue
        pressure.append((height, level * 100))
        level_temperature = value(f"temperature_{level}hPa")
        if level_temperature is not None:
            temperature.append((height, level_temperature + 273.15))
        if height > ground + 200:
            vector = wind_vector(f"{level}hPa")
            if vector is not None:
                wind.append((height, *vector))
    wind = sorted(wind)
    profiles = {
        "pressure": sorted(pressure),
        "temperature": sorted(temperature),
        "wind_u": [(h, u) for h, u, v in wind],
        "wind_v": [(h, v) for h, u, v in wind],
    }
    profiles = {name: [[round(h, 6), round(v, 6)] for h, v in rows]
                for name, rows in profiles.items()}
    for name, rows in profiles.items():
        if (len(rows) < 2 or rows[0][0] != ground or rows[-1][0] < top
                or any(not math.isfinite(h) or not math.isfinite(v) for h, v in rows)
                or any(b[0] <= a[0] for a, b in zip(rows, rows[1:]))):
            parser.error(f"ICON-D2 returned an invalid or incomplete {name} profile")
    if any(b[1] >= a[1] for a, b in zip(profiles["pressure"], profiles["pressure"][1:])):
        parser.error("ICON-D2 pressure must decrease with height")

    environment_args = {name: value for name, value in config.items() if not name.startswith("_")}
    env = Environment(**environment_args)
    env.set_date(launch_date, timezone=launch_timezone)
    env.set_atmospheric_model(type="custom_atmosphere", **profiles)
    if not float(env.pressure(ground)) > float(env.pressure(top)) > 0:
        parser.error("RocketPy pressure is invalid across the flight zone")

    fetched = datetime.now(timezone.utc).isoformat()
    header = (
        f"// Open-Meteo ICON-D2 fetched {fetched}; launch {launch_local.isoformat()} ({hour} UTC).\n"
        f"// Hourly records {first_hour:%H:%M} and {next_hour:%H:%M} UTC; time interpolation {fraction:.2f} (wind vectors).\n"
        f"// Requested site: {config['latitude']} N, {config['longitude']} E; pad {ground:g} m ASL.\n"
        f"// Model grid: {data.get('latitude')} N, {data.get('longitude')} E; "
        f"elevation {data.get('elevation')} m. Raw response: {raw_path.name}.\n"
        "// Heights are metres ASL; pressure Pa, temperature K, east/north wind m/s; rounded to 6 decimals.\n"
        "// Replace Environment._calls with this array so the date and atmosphere stay paired.\n"
    )
    calls = [
        {"method": "set_date", "kwargs": {"date": launch_date, "timezone": launch_timezone}},
        {"method": "set_atmospheric_model", "kwargs": {"type": "custom_atmosphere", **profiles}},
    ]
    raw = {"request_url": url, "fetched_utc": fetched, "target_utc": launch_utc.isoformat(),
           "response": data}
    try:
        with raw_path.open("x", encoding="utf-8") as destination:
            json.dump(raw, destination, indent=2)
            destination.write("\n")
        with args.output.open("x", encoding="utf-8") as destination:
            destination.write(header + json.dumps(calls, indent=2) + "\n")
    except FileExistsError as exc:
        parser.error(f"Output already exists: {exc.filename}")
    print(f"Wrote {args.output} and {raw_path} for {launch_local.isoformat()} ({hour} UTC)")


if __name__ == "__main__":
    main()
