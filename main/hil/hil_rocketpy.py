import argparse
import threading
from pathlib import Path

from rocketpy import Environment, Flight, SolidMotor, RocketV2
from rocketpy import Accelerometer
from rocketpy import Barometer
from rocketpy import GnssReceiver


import hil_communication
from hil_config import load_hil_config, prepare_rocketpy_kwargs
from hil_capture import create_capture_file, save_hil_capture, plot_hil_log


class CommandState:
    def __init__(self):
        self.lock = threading.Lock()

        self.sim_time = 0
        self.open_main = False
        self.open_drogue = False
        self.airbrakes_lvl = 0.0


command_state = CommandState()


# ----------------------------------------------------------------------
# HIL DATA RECORDER
# ----------------------------------------------------------------------
hil_events = {
    "open_drogue": [],
    "open_main": [],
    "airbrakes": [],
}

hil_log = {
    # bookkeeping
    "seq": [],
    "t": [],

    # truth state from RocketPy
    "x": [],
    "y": [],
    "z": [],
    "vx": [],
    "vy": [],
    "vz": [],
    "e0": [],
    "e1": [],
    "e2": [],
    "e3": [],
    "omega1": [],
    "omega2": [],
    "omega3": [],

    # sensor data sent to the FC
    "ax": [],
    "ay": [],
    "az": [],
    "p": [],
    "lat": [],
    "lon": [],
    "alt": [],
}


# ----------------------------------------------------------------------
# BASE PATH + CLI
# ----------------------------------------------------------------------
try:
    BASE_DIR = Path(__file__).resolve().parent
except NameError:
    # If running in an environment where __file__ does not exist.
    BASE_DIR = Path(".").resolve()


def positive_int(value):
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return parsed


parser = argparse.ArgumentParser(description="RocketPy HiL simulation launcher")
parser.add_argument(
    "--rocket",
    required=True,
    help=(
        "Rocket model folder under config/. "
        "Example: --rocket fred loads config/fred/fred_rocketpy_config.json"
    ),
)
parser.add_argument(
    "--sampling-rate",
    type=positive_int,
    default=20,
    help="Sampling rate in Hz for sensors, logger, airbrakes and parachutes. Default: 20 Hz.",
)

args = parser.parse_args()

rocket_model = args.rocket.strip().lower()
sampling_rate = args.sampling_rate

CONFIG_DIR = BASE_DIR / "config" / rocket_model
CONFIG_PATH = CONFIG_DIR / f"{rocket_model}_rocketpy_config.json"

if not CONFIG_PATH.is_file():
    raise FileNotFoundError(f"Rocket config not found: {CONFIG_PATH}")

cfg = load_hil_config(CONFIG_PATH)


# ----------------------------------------------------------------------
# ENVIRONMENT
# ----------------------------------------------------------------------
env_args = prepare_rocketpy_kwargs(cfg["Environment"], CONFIG_DIR)
env = Environment(**env_args)

atmospheric_model_args = None
if "Environment.set_atmospheric_model" in cfg:
    atmospheric_model_args = prepare_rocketpy_kwargs(
        cfg["Environment.set_atmospheric_model"],
        CONFIG_DIR,
    )
    env.set_atmospheric_model(**atmospheric_model_args)

print("Environment... READY")


# ----------------------------------------------------------------------
# PARACHUTE LOGIC
# ----------------------------------------------------------------------
def simulator_check_drogue_opening(p, h, y):
    with command_state.lock:
        return command_state.open_drogue


def simulator_check_main_opening(p, h, y):
    with command_state.lock:
        return command_state.open_main


PARACHUTE_TRIGGERS = {
    "main": simulator_check_main_opening,
    "drogue": simulator_check_drogue_opening,
}


# ----------------------------------------------------------------------
# MOTOR DATA
# ----------------------------------------------------------------------
motor = SolidMotor(**prepare_rocketpy_kwargs(cfg["SolidMotor"], CONFIG_DIR))

print("Motor... READY")


# ----------------------------------------------------------------------
# ROCKET
# ----------------------------------------------------------------------
rocket = RocketV2(**prepare_rocketpy_kwargs(cfg["RocketV2"], CONFIG_DIR))

if "set_rail_buttons" in cfg:
    rocket.set_rail_buttons(**prepare_rocketpy_kwargs(cfg["set_rail_buttons"], CONFIG_DIR))

rocket.add_motor(motor, **prepare_rocketpy_kwargs(cfg["add_motor"], CONFIG_DIR))

if "add_nose" in cfg:
    rocket.add_nose(**prepare_rocketpy_kwargs(cfg["add_nose"], CONFIG_DIR))

if "add_trapezoidal_fins" in cfg:
    rocket.add_trapezoidal_fins(
        **prepare_rocketpy_kwargs(cfg["add_trapezoidal_fins"], CONFIG_DIR)
    )

if "add_tail" in cfg:
    rocket.add_tail(**prepare_rocketpy_kwargs(cfg["add_tail"], CONFIG_DIR))

for parachute_cfg in cfg.get("add_parachute", []):
    parachute_args = prepare_rocketpy_kwargs(parachute_cfg, CONFIG_DIR)
    name = parachute_args.pop("name")

    trigger_name = parachute_cfg.get("_trigger")
    if trigger_name is None:
        raise ValueError(f"Parachute {name!r} is missing required field '_trigger'")

    if trigger_name not in PARACHUTE_TRIGGERS:
        raise ValueError(f"Unknown parachute trigger: {trigger_name!r}")

    rocket.add_parachute(
        name,
        trigger=PARACHUTE_TRIGGERS[trigger_name],
        sampling_rate=sampling_rate,
        **parachute_args,
    )

print("Rocket... READY")


# ----------------------------------------------------------------------
# SENSORS
# ----------------------------------------------------------------------
accel_clean = Accelerometer(
    sampling_rate=sampling_rate,
    consider_gravity=True,
    orientation=(0, 0, 0),
    noise_density=0,
    random_walk_density=0,
    constant_bias=0,
    temperature_bias=0,
    temperature_scale_factor=0,
    cross_axis_sensitivity=0,
    name="Clean Accelerometer",
)
rocket.add_sensor(accel_clean, position=0)

barometer_clean = Barometer(
    sampling_rate=sampling_rate,
    noise_density=0,
    random_walk_density=0,
    constant_bias=0,
    temperature_bias=0,
    temperature_scale_factor=0,
    name="Clean Barometer",
)
rocket.add_sensor(barometer_clean, position=0)

gnss_clean = GnssReceiver(
    sampling_rate=sampling_rate,
    position_accuracy=0,
    altitude_accuracy=0,
    name="Clean GPS",
)
rocket.add_sensor(gnss_clean, position=0)

print("Sensors... READY")


# ----------------------------------------------------------------------
# STATE LOGGER + COMMUNICATION WITH FLIGHT CONTROLLER
# ----------------------------------------------------------------------
def on_open_main(sim_time):
    with command_state.lock:
        if command_state.sim_time <= sim_time:
            command_state.sim_time = sim_time
            if command_state.open_main == False:
                print(f"[ESP32_cmd] 'main_deployed' at t={sim_time:.3f}")
                hil_events["open_main"].append(sim_time)
            command_state.open_main = True
        else:
            print("[E]: time mismatch, overwriting with old values new stuff.")


def on_open_drogue(sim_time):
    with command_state.lock:
        if command_state.sim_time <= sim_time:
            command_state.sim_time = sim_time
            if command_state.open_drogue == False:
                print(f"[ESP32_cmd]: 'drogue_deployed' at t={sim_time:.3f}")
                hil_events["open_drogue"].append(sim_time)
            command_state.open_drogue = True
        else:
            print("[E]: time mismatch, overwriting with old values new stuff.")


def on_set_air_brakes(sim_time, lvl):
    with command_state.lock:
        if command_state.sim_time <= sim_time:
            command_state.sim_time = sim_time
            if command_state.airbrakes_lvl != lvl:
                print(f"[ESP32_cmd]: deployment_level={lvl} at t={sim_time:.3f}")
                hil_events["airbrakes"].append((sim_time, lvl))
            command_state.airbrakes_lvl = lvl
        else:
            print("[E]: time mismatch, overwriting with old values new stuff.")


def airbrakes_drag_function(level, mach):
    base_drag_added = 0.5
    return base_drag_added * level


def airbrakes_controller(time, sampling_rate, state_vector, state_history, observed_variables, interactive_objects):
    airbrake = interactive_objects
    with command_state.lock:
        airbrake.deployment_level = command_state.airbrakes_lvl
    return airbrake

seq = 0
last_sent_t = None
TIMESTAMP_EPS = 1e-9

def _safe_get(mapping, key, default=float("nan")):
    """
    Read a key from dict-like RocketPy callback data.
    Returns NaN if the key does not exist.
    """
    try:
        return mapping[key]
    except Exception:
        return default


def enqueue_data(t, state, sensors):
    global seq
    global last_sent_t

    # Drop duplicated RocketPy callbacks at the same simulated time.
    # Read main/hil/README.md "Bugs".
    if last_sent_t is not None and abs(t - last_sent_t) < TIMESTAMP_EPS:
        return

    if seq % sampling_rate == 0:
        print(f"t_sim = {t:.6f} | seq={seq}")

    # ------------------------------------------------------------------
    # Extract truth state from RocketPy
    # ------------------------------------------------------------------
    x = _safe_get(state, "x")
    y = _safe_get(state, "y")
    z = _safe_get(state, "z")

    vx = _safe_get(state, "vx")
    vy = _safe_get(state, "vy")
    vz = _safe_get(state, "vz")

    e0 = _safe_get(state, "e0")
    e1 = _safe_get(state, "e1")
    e2 = _safe_get(state, "e2")
    e3 = _safe_get(state, "e3")

    omega1 = _safe_get(state, "omega1")
    omega2 = _safe_get(state, "omega2")
    omega3 = _safe_get(state, "omega3")

    # ------------------------------------------------------------------
    # Extract sensor data sent to the FC
    # ------------------------------------------------------------------
    ax = _safe_get(sensors, "ax")
    ay = _safe_get(sensors, "ay")
    az = _safe_get(sensors, "az")
    p = _safe_get(sensors, "p")
    lat = _safe_get(sensors, "lat")
    lon = _safe_get(sensors, "lon")
    alt = _safe_get(sensors, "alt")

    # ------------------------------------------------------------------
    # Build and send the exact same payload as before
    # ------------------------------------------------------------------
    payload = hil_communication.build_payload(seq, t, ax, ay, az, p, lat, lon, alt)
    mailbox.put(payload)

    # ------------------------------------------------------------------
    # Log only packets that were actually queued
    # ------------------------------------------------------------------
    hil_log["seq"].append(seq)
    hil_log["t"].append(t)

    hil_log["x"].append(x)
    hil_log["y"].append(y)
    hil_log["z"].append(z)

    hil_log["vx"].append(vx)
    hil_log["vy"].append(vy)
    hil_log["vz"].append(vz)

    hil_log["e0"].append(e0)
    hil_log["e1"].append(e1)
    hil_log["e2"].append(e2)
    hil_log["e3"].append(e3)

    hil_log["omega1"].append(omega1)
    hil_log["omega2"].append(omega2)
    hil_log["omega3"].append(omega3)

    hil_log["ax"].append(ax)
    hil_log["ay"].append(ay)
    hil_log["az"].append(az)

    hil_log["p"].append(p)

    hil_log["lat"].append(lat)
    hil_log["lon"].append(lon)
    hil_log["alt"].append(alt)

    # Update only after the packet has actually been queued and logged.
    last_sent_t = t
    seq += 1


mailbox = hil_communication.OneSlotMailbox()
handlers_dict = {
    "on_open_drogue": on_open_drogue,
    "on_open_main": on_open_main,
    "on_set_air_brakes": on_set_air_brakes,
}
handlers = hil_communication.CommandHandlers(**handlers_dict)

state_ctrl = rocket.add_state_and_sensors_logger(callback=enqueue_data, sampling_rate=sampling_rate)
print("State Logger... READY")

rocket.add_air_brakes(
    drag_coefficient_curve=airbrakes_drag_function,
    controller_function=airbrakes_controller,
    sampling_rate=sampling_rate,
    clamp=True,
)

esp_connected = threading.Semaphore(0)
th = hil_communication.tcp_client_thread_start(esp_connected, mailbox, handlers)

print("Wait for ESP connection...", end="")
esp_connected.acquire()
print("READY")


# ----------------------------------------------------------------------
# CAPTURE METADATA
# ----------------------------------------------------------------------
def build_capture_metadata():
    flight_args = prepare_rocketpy_kwargs(cfg["Flight"], CONFIG_DIR)

    metadata = {
        "rocket": rocket_model,
        "config_path": str(CONFIG_PATH),
        "sampling_rate_hz": sampling_rate,
        "environment": env_args,
        "atmospheric_model": atmospheric_model_args,
        "flight": flight_args,
        "environment_latitude": env_args.get("latitude"),
        "environment_longitude": env_args.get("longitude"),
        "environment_elevation_m": env_args.get("elevation"),
        "rail_length_m": flight_args.get("rail_length"),
        "inclination_deg": flight_args.get("inclination"),
        "heading_deg": flight_args.get("heading"),
        "simulation_start_date_utc": env_args.get("date"),
        "notes": "HIL payload stream sent to ESP32 flight controller",
    }

    return metadata


# ----------------------------------------------------------------------
# RUN REAL-TIME SIMULATION
# ----------------------------------------------------------------------
print("Setup completed. Starting Flight...")

capture_file = create_capture_file(BASE_DIR / "hil_captures")

flight_args = prepare_rocketpy_kwargs(cfg["Flight"], CONFIG_DIR)
test_flight = Flight(
    rocket=rocket,
    environment=env,
    **flight_args,
)

reset_choice = input(
    "Send simulation reset to the flight controller before showing the plot? [y/N]: "
).strip().lower()

if reset_choice in ("y", "yes"):
    mailbox.put(hil_communication.RESET_SIMULATION)
    print("RESET_SIM requested")
else:
    print("RESET_SIM skipped")

print("Flight... COMPLETED")

save_hil_capture(
    filename=capture_file,
    hil_log=hil_log,
    hil_events=hil_events,
    metadata=build_capture_metadata(),
)

plot_hil_log(
    hil_log,
    hil_events,
    sampling_rate=sampling_rate,
)

# Keep RocketPy's built-in trajectory plot available for manual debugging.
# test_flight.plots.trajectory_3d()
