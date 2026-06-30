"""Run a RocketPy hardware-in-the-loop flight simulation.

This launcher loads a rocket HIL config, builds the RocketPy environment,
rocket, sensors, and callbacks, streams calibration and flight sensor packets
to the flight controller over TCP, accepts controller commands for actuators,
and saves a replayable HIL capture at the end of the run.
"""

import argparse
import copy
import math
import threading
from pathlib import Path
import numpy as np

from rocketpy import Environment, Flight, SolidMotor, RocketV2
from rocketpy import Accelerometer
from rocketpy import Barometer
from rocketpy import GnssReceiver

import hil_communication
from hil_config import (
    apply_configured_calls,
    load_hil_config,
    prepare_rocketpy_kwargs,
    prepare_sensor_configs,
    require_config_section,
)
from hil_utils import (
    accelerometer_body_to_clean_sensor_matrix as _utils_accelerometer_body_to_clean_sensor_matrix,
    accelerometer_cross_axis_matrix as _utils_accelerometer_cross_axis_matrix,
    matrix_like_to_numpy_3x3 as _matrix_like_to_numpy,
    numeric_array as _numeric_config_array,
    rail_body_to_inertial_matrix as _rail_body_to_inertial_matrix,
    rocketpy_constructor_orientation_from_config as _rocketpy_constructor_orientation_from_config,
    rocketpy_euler313_matrix as _rocketpy_euler313_matrix,
    rotation_matrix_to_rocketpy_quaternion as _rotation_matrix_to_rocketpy_quaternion,
)
from hil_capture import create_capture_file, save_hil_capture, plot_hil_log


class CommandState:
    def __init__(self):
        self.lock = threading.Lock()
        self.fsm_ready_event = threading.Event()
        self.reset()

    def reset(self):
        with self.lock:
            self.latest_command_sim_time_s = 0.0
            self.open_main = False
            self.open_drogue = False
            self.airbrakes_lvl = 0.0
            self.fsm_state = hil_communication.FSM_STATE_INACTIVE
            self.fsm_state_name = hil_communication.fsm_state_name(self.fsm_state)
            self._last_printed_fsm_state = None

        self.fsm_ready_event.clear()


command_state = CommandState()


# ----------------------------------------------------------------------
# PREPARE SHARED STATE AND DATA RECORDERS
# ----------------------------------------------------------------------
hil_events = {
    "open_drogue": [],
    "open_main": [],
    "airbrakes": [],
    "fsm_state": [],
}

hil_log = {
    # bookkeeping
    "seq": [],
    "sim_time_s": [],

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
    "accel_x_m_s2": [],
    "accel_y_m_s2": [],
    "accel_z_m_s2": [],
    "pressure_pa": [],
    "temperature_k": [],
    "latitude_deg": [],
    "longitude_deg": [],
    "altitude_m": [],
}


# ----------------------------------------------------------------------
# LOAD CLI ARGUMENTS AND ROCKET CONFIGURATION
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
parser.add_argument(
    "--sensor-profile",
    required=True,
    help=(
        "Exact sensor profile name from Sensors._profiles. "
        "Examples: --sensor-profile clean, --sensor-profile noisy, "
        "--sensor-profile very_noisy."
    ),
)
parser.add_argument(
    "--calibration-samples",
    type=positive_int,
    default=500,
    help="Maximum number of stationary HIL samples to send before creating Flight. Default: 500.",
)
parser.add_argument(
    "--calibration-rate",
    type=positive_int,
    default=None,
    help=(
        "Rate in Hz for the pre-flight calibration samples. "
        "Defaults to --sampling-rate."
    ),
)
parser.add_argument(
    "--no-startup-reset",
    action="store_true",
    help="Skip the startup MSG_TYPE_SIM_RESET before calibration.",
)
parser.add_argument(
    "--startup-reset-timeout",
    type=positive_int,
    default=20,
    help="Seconds to wait for the FC TCP server after startup reset. Default: 20.",
)

args = parser.parse_args()

rocket_model = args.rocket.strip().lower()
sampling_rate = args.sampling_rate
requested_sensor_profile_name = args.sensor_profile
calibration_samples = args.calibration_samples
calibration_rate = args.calibration_rate or sampling_rate
startup_reset_enabled = not args.no_startup_reset
startup_reset_timeout_s = args.startup_reset_timeout

CONFIG_DIR = BASE_DIR / "config" / rocket_model
CONFIG_PATH = CONFIG_DIR / f"{rocket_model}_rocketpy_config.jsonc"

if not CONFIG_PATH.is_file():
    raise FileNotFoundError(f"Rocket config not found: {CONFIG_PATH}")

cfg = load_hil_config(CONFIG_PATH)


# ----------------------------------------------------------------------
# BUILD ROCKETPY ENVIRONMENT
# ----------------------------------------------------------------------
environment_cfg = require_config_section(cfg, "Environment")

if (
    "set_date" in cfg
    or "set_atmospheric_model" in cfg
    or "Environment.set_date" in cfg
    or "Environment.set_atmospheric_model" in cfg
):
    raise ValueError(
        "Environment method calls must be declared inside Environment._calls."
    )

if "set_date" in environment_cfg or "set_atmospheric_model" in environment_cfg:
    raise ValueError(
        "Environment method calls must use Environment._calls, not direct "
        "Environment.set_date or Environment.set_atmospheric_model fields."
    )

environment_calls_cfg = environment_cfg.get("_calls", [])
if not isinstance(environment_calls_cfg, list):
    raise TypeError("Environment._calls must be a JSON array when provided")

environment_set_date_seen = False
for index, call_cfg in enumerate(environment_calls_cfg):
    if not isinstance(call_cfg, dict):
        raise TypeError(f"Environment._calls[{index}] must be a JSON object")

    method_name = call_cfg.get("method")
    if method_name == "set_date":
        environment_set_date_seen = True
    elif method_name == "set_atmospheric_model" and not environment_set_date_seen:
        raise ValueError(
            "Environment._calls must call set_date before set_atmospheric_model"
        )

env_args = prepare_rocketpy_kwargs(environment_cfg, CONFIG_DIR)
env = Environment(**env_args)

environment_applied_calls = apply_configured_calls(
    target=env,
    section=environment_cfg,
    config_dir=CONFIG_DIR,
    section_name="Environment",
)

atmospheric_model_args = next(
    (
        call["kwargs"]
        for call in environment_applied_calls
        if call["method"] == "set_atmospheric_model"
    ),
    None,
)
environment_set_date = next(
    (
        call["kwargs"].get("date", call["args"][0] if call["args"] else None)
        for call in environment_applied_calls
        if call["method"] == "set_date"
    ),
    None,
)

print("Environment... READY")


# ----------------------------------------------------------------------
# DEFINE CONTROLLER-DRIVEN ACTUATOR CALLBACKS
# ----------------------------------------------------------------------
def simulator_check_drogue_opening(pressure, height, state_vector):
    with command_state.lock:
        return command_state.open_drogue


def simulator_check_main_opening(pressure, height, state_vector):
    with command_state.lock:
        return command_state.open_main


PARACHUTE_TRIGGERS = {
    "main": simulator_check_main_opening,
    "drogue": simulator_check_drogue_opening,
}


# ----------------------------------------------------------------------
# MOTOR DATA
# ----------------------------------------------------------------------
motor = SolidMotor(**prepare_rocketpy_kwargs(require_config_section(cfg, "SolidMotor"), CONFIG_DIR))

print("Motor... READY")


# ----------------------------------------------------------------------
# ROCKET STRUCTURE
# ----------------------------------------------------------------------
rocket = RocketV2(**prepare_rocketpy_kwargs(require_config_section(cfg, "RocketV2"), CONFIG_DIR))

if "set_rail_buttons" in cfg:
    rocket.set_rail_buttons(**prepare_rocketpy_kwargs(cfg["set_rail_buttons"], CONFIG_DIR))

rocket.add_motor(motor, **prepare_rocketpy_kwargs(require_config_section(cfg, "add_motor"), CONFIG_DIR))

if "add_nose" in cfg:
    rocket.add_nose(**prepare_rocketpy_kwargs(cfg["add_nose"], CONFIG_DIR))

if "add_trapezoidal_fins" in cfg:
    rocket.add_trapezoidal_fins(
        **prepare_rocketpy_kwargs(cfg["add_trapezoidal_fins"], CONFIG_DIR)
    )

if "add_tail" in cfg:
    rocket.add_tail(**prepare_rocketpy_kwargs(cfg["add_tail"], CONFIG_DIR))

parachute_configs = cfg.get("add_parachute", [])
if not isinstance(parachute_configs, list):
    raise TypeError("HIL config section 'add_parachute' must be a JSON array when provided")

for parachute_cfg in parachute_configs:
    if not isinstance(parachute_cfg, dict):
        raise TypeError("Each add_parachute entry must be a JSON object")

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
SENSOR_CLASSES = {
    "Accelerometer": Accelerometer,
    "Barometer": Barometer,
    "GnssReceiver": GnssReceiver,
}

selected_sensor_profile_name, prepared_sensor_configs = prepare_sensor_configs(
    config=cfg,
    config_dir=CONFIG_DIR,
    requested_profile_name=requested_sensor_profile_name,
    default_sampling_rate_hz=sampling_rate,
    known_sensor_types=SENSOR_CLASSES.keys(),
)
selected_sensor_metadata = {}
selected_sensor_instances = {}

for prepared_sensor in prepared_sensor_configs:
    sensor_constructor_kwargs = copy.deepcopy(prepared_sensor.constructor_kwargs)
    sensor_metadata = prepared_sensor.metadata()

    # HIL config uses radians for 3-angle inertial-sensor orientations.
    # Some RocketPy versions still apply deg2rad() internally to 3-angle
    # constructor values, so pass an explicit S_clean -> B matrix instead.
    if prepared_sensor.sensor_type == "Accelerometer" and "orientation" in sensor_constructor_kwargs:
        effective_orientation_matrix = _rocketpy_constructor_orientation_from_config(
            sensor_constructor_kwargs["orientation"]
        )
        sensor_constructor_kwargs["orientation"] = effective_orientation_matrix
        sensor_metadata["orientation_convention"] = "radians"
        sensor_metadata["effective_orientation_matrix_sensor_to_body"] = (
            effective_orientation_matrix
        )

    sensor = SENSOR_CLASSES[prepared_sensor.sensor_type](
        **sensor_constructor_kwargs
    )
    rocket.add_sensor(sensor, position=prepared_sensor.position)

    selected_sensor_instances[prepared_sensor.sensor_type] = sensor
    selected_sensor_metadata[prepared_sensor.sensor_type] = sensor_metadata

print(f"Sensors... READY ({selected_sensor_profile_name})")

# ----------------------------------------------------------------------
# COMMAND HANDLERS AND TELEMETRY STREAMING
# ----------------------------------------------------------------------
def on_open_main(command_sim_time_s):
    with command_state.lock:
        if command_state.latest_command_sim_time_s <= command_sim_time_s:
            command_state.latest_command_sim_time_s = command_sim_time_s
            if command_state.open_main == False:
                print(f"[ESP32_cmd] 'main_deployed' at sim_time={command_sim_time_s:.3f}s")
                hil_events["open_main"].append(command_sim_time_s)
            command_state.open_main = True
        else:
            print("[E]: time mismatch, overwriting with old values new stuff.")


def on_open_drogue(command_sim_time_s):
    with command_state.lock:
        if command_state.latest_command_sim_time_s <= command_sim_time_s:
            command_state.latest_command_sim_time_s = command_sim_time_s
            if command_state.open_drogue == False:
                print(f"[ESP32_cmd]: 'drogue_deployed' at sim_time={command_sim_time_s:.3f}s")
                hil_events["open_drogue"].append(command_sim_time_s)
            command_state.open_drogue = True
        else:
            print("[E]: time mismatch, overwriting with old values new stuff.")


def on_set_air_brakes(command_sim_time_s, deployment_level):
    with command_state.lock:
        if command_state.latest_command_sim_time_s <= command_sim_time_s:
            command_state.latest_command_sim_time_s = command_sim_time_s
            if command_state.airbrakes_lvl != deployment_level:
                print(
                    f"[ESP32_cmd]: deployment_level={deployment_level} "
                    f"at sim_time={command_sim_time_s:.3f}s"
                )
                hil_events["airbrakes"].append((command_sim_time_s, deployment_level))
            command_state.airbrakes_lvl = deployment_level
        else:
            print("[E]: time mismatch, overwriting with old values new stuff.")


def on_fsm_state(command_sim_time_s, fsm_state):
    name = hil_communication.fsm_state_name(fsm_state)

    with command_state.lock:
        previous = command_state.fsm_state
        command_state.fsm_state = fsm_state
        command_state.fsm_state_name = name

        if previous != fsm_state:
            print(f"[ESP32_fsm]: {name} at sim_time={command_sim_time_s:.3f}s")
            hil_events["fsm_state"].append((command_sim_time_s, name))

        if fsm_state == hil_communication.FSM_STATE_READY_FOR_LAUNCH:
            command_state.fsm_ready_event.set()


def airbrakes_drag_function(level, mach):
    base_drag_added = 0.5
    return base_drag_added * level


def airbrakes_controller(controller_time_s, sampling_rate, state_vector, state_history, observed_variables, interactive_objects):
    airbrake = interactive_objects
    with command_state.lock:
        airbrake.deployment_level = command_state.airbrakes_lvl
    return airbrake

seq = 0
last_rocketpy_callback_time_s = None
TIMESTAMP_EPS = 1e-9
PREFLIGHT_CALIBRATION_DURATION_S = 0.0

def require_callback_float(mapping, key, source):
    """Read a mandatory numeric field from a RocketPy callback mapping."""
    try:
        value = mapping[key]
    except Exception as exc:
        available = ", ".join(sorted(str(item) for item in mapping.keys()))
        raise KeyError(
            f"RocketPy {source} callback is missing required field {key!r}. "
            f"Available fields: {available}"
        ) from exc

    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise TypeError(
            f"RocketPy {source} callback field {key!r} must be numeric, got {value!r}"
        ) from exc

def append_hil_log_sample(
    *,
    seq_value,
    hil_sim_time_s,
    accel_x_m_s2,
    accel_y_m_s2,
    accel_z_m_s2,
    pressure_pa,
    temperature_k,
    latitude_deg,
    longitude_deg,
    altitude_m,
    x=0.0,
    y=0.0,
    z=0.0,
    vx=0.0,
    vy=0.0,
    vz=0.0,
    e0=1.0,
    e1=0.0,
    e2=0.0,
    e3=0.0,
    omega1=0.0,
    omega2=0.0,
    omega3=0.0,
):
    """
    Log one HIL packet that was actually queued/sent to the FC.

    Truth state is stored in RocketPy inertial/body attitude variables.
    Accelerometer channels are stored exactly as payload output: ``S_out``.

    During calibration there is no RocketPy Flight state yet, so we log a
    stationary synthetic truth state. During flight, enqueue_data(...) passes
    the real RocketPy state values.
    """
    hil_log["seq"].append(seq_value)
    hil_log["sim_time_s"].append(hil_sim_time_s)

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

    hil_log["accel_x_m_s2"].append(accel_x_m_s2)
    hil_log["accel_y_m_s2"].append(accel_y_m_s2)
    hil_log["accel_z_m_s2"].append(accel_z_m_s2)

    hil_log["pressure_pa"].append(pressure_pa)
    hil_log["temperature_k"].append(temperature_k)

    hil_log["latitude_deg"].append(latitude_deg)
    hil_log["longitude_deg"].append(longitude_deg)
    hil_log["altitude_m"].append(altitude_m)

def enqueue_data(rocketpy_time_s, state, sensors):
    global seq
    global last_rocketpy_callback_time_s

    # Drop duplicated RocketPy callbacks at the same simulated time.
    # The packet timestamp sent to the FC is offset by the calibration duration,
    # but duplicate detection must stay on RocketPy's raw callback time.
    if (
        last_rocketpy_callback_time_s is not None
        and abs(rocketpy_time_s - last_rocketpy_callback_time_s) < TIMESTAMP_EPS
    ):
        return

    hil_sim_time_s = PREFLIGHT_CALIBRATION_DURATION_S + rocketpy_time_s

    if seq % sampling_rate == 0:
        print(
            f"hil_sim_time={hil_sim_time_s:.6f}s | "
            f"rocketpy_time={rocketpy_time_s:.6f}s | seq={seq}"
        )

    # Extract RocketPy truth state: inertial position/velocity + body attitude.
    x = require_callback_float(state, "x", "state")
    y = require_callback_float(state, "y", "state")
    z = require_callback_float(state, "z", "state")

    vx = require_callback_float(state, "vx", "state")
    vy = require_callback_float(state, "vy", "state")
    vz = require_callback_float(state, "vz", "state")

    e0 = require_callback_float(state, "e0", "state")
    e1 = require_callback_float(state, "e1", "state")
    e2 = require_callback_float(state, "e2", "state")
    e3 = require_callback_float(state, "e3", "state")

    omega1 = require_callback_float(state, "omega1", "state")
    omega2 = require_callback_float(state, "omega2", "state")
    omega3 = require_callback_float(state, "omega3", "state")

    # Extract sensor payload sent to the FC. Accelerometer is already S_out.
    accel_x_m_s2 = require_callback_float(sensors, "ax", "sensors")
    accel_y_m_s2 = require_callback_float(sensors, "ay", "sensors")
    accel_z_m_s2 = require_callback_float(sensors, "az", "sensors")
    pressure_pa = require_callback_float(sensors, "p", "sensors")
    latitude_deg = require_callback_float(sensors, "lat", "sensors")
    longitude_deg = require_callback_float(sensors, "lon", "sensors")
    altitude_m = require_callback_float(sensors, "alt", "sensors")
    
    # RocketPy/RocketV2 currently does not expose barometer temperature in the
    # state-and-sensors callback. If the callback provides it, use it. Otherwise
    # derive it explicitly from the configured Environment at current altitude.
    if "temperature" in sensors:
        temperature_k = require_callback_float(sensors, "temperature", "sensors")
    else:
        temperature_k = environment_temperature_at_asl_m(altitude_m) # since altitude_m could be noisy, also the temperature will be noisy.

    # Build and send the payload to the FC
    payload = hil_communication.build_sim_input_payload(
        sequence_number=seq,
        hil_sim_time_s=hil_sim_time_s,
        ax_m_s2=accel_x_m_s2,
        ay_m_s2=accel_y_m_s2,
        az_m_s2=accel_z_m_s2,
        pressure_pa=pressure_pa,
        temperature_k=temperature_k,
        latitude_deg=latitude_deg,
        longitude_deg=longitude_deg,
        altitude_m=altitude_m,
    )
    mailbox.put(payload)

    # Log only packets that were actually queued
    append_hil_log_sample(
        seq_value=seq, hil_sim_time_s=hil_sim_time_s,
        accel_x_m_s2=accel_x_m_s2,
        accel_y_m_s2=accel_y_m_s2,
        accel_z_m_s2=accel_z_m_s2,
        pressure_pa=pressure_pa,
        temperature_k=temperature_k,
        latitude_deg=latitude_deg,
        longitude_deg=longitude_deg,
        altitude_m=altitude_m,
        x=x, y=y, z=z, 
        vx=vx, vy=vy, vz=vz, 
        e0=e0, e1=e1, e2=e2, e3=e3,
        omega1=omega1, omega2=omega2, omega3=omega3,
    )

    # Update only after the packet has actually been queued and logged.
    last_rocketpy_callback_time_s = rocketpy_time_s
    seq += 1


# ----------------------------------------------------------------------
# PRE-FLIGHT CALIBRATION STREAM
# ----------------------------------------------------------------------
def require_environment_float(attribute_name):
    """Read a mandatory numeric Environment attribute with context."""
    if not hasattr(env, attribute_name):
        raise AttributeError(f"RocketPy Environment is missing required attribute {attribute_name!r}")

    value = getattr(env, attribute_name)
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise TypeError(
            f"RocketPy Environment.{attribute_name} must be numeric, got {value!r}"
        ) from exc


def environment_pressure_at_asl_m(altitude_asl_m):
    try:
        return float(env.pressure(altitude_asl_m))
    except Exception as exc:
        raise RuntimeError(
            f"RocketPy Environment.pressure failed at altitude_asl_m={altitude_asl_m!r}"
        ) from exc


def environment_temperature_at_asl_m(altitude_asl_m):
    try:
        return float(env.temperature(altitude_asl_m))
    except Exception as exc:
        raise RuntimeError(
            f"RocketPy Environment.temperature failed at altitude_asl_m={altitude_asl_m!r}"
        ) from exc


def environment_gravity_at_asl_m(altitude_asl_m):
    """Return RocketPy/Environment gravity magnitude at altitude ASL.

    Environment.gravity is a positive magnitude; the physical gravity vector in
    the inertial frame is therefore ``[0, 0, -g]``. For a stationary rocket on
    the rail, the accelerometer measures the opposite support force
    ``[0, 0, +g]``.
    """
    try:
        gravity_model = env.gravity
        if hasattr(gravity_model, "get_value_opt"):
            return float(gravity_model.get_value_opt(altitude_asl_m))
        return float(gravity_model(altitude_asl_m))
    except Exception as exc:
        raise RuntimeError(
            f"RocketPy Environment.gravity failed at altitude_asl_m={altitude_asl_m!r}"
        ) from exc


def get_launch_site_conditions():
    elevation_m = require_environment_float("elevation")

    return {
        "elevation_m": elevation_m,
        "latitude": require_environment_float("latitude"),
        "longitude": require_environment_float("longitude"),
        "pressure_pa": environment_pressure_at_asl_m(elevation_m),
        "temperature_k": environment_temperature_at_asl_m(elevation_m),
        "gravity_m_s2": environment_gravity_at_asl_m(elevation_m),
    }


# ----------------------------------------------------------------------
# CALIBRATION ATTITUDE / ACCELEROMETER GEOMETRY
# ----------------------------------------------------------------------
# Shared frame/sensor transform helpers live in hil_utils.py.
def _accelerometer_sensor_to_body_matrix():
    """Return the accelerometer mounting matrix: ``S_clean -> body``.

    Prefer RocketPy's public pure geometry matrix when available, otherwise use
    the resolved orientation metadata stored when the sensor was constructed.
    """
    accel_sensor = selected_sensor_instances.get("Accelerometer")

    if accel_sensor is not None and hasattr(accel_sensor, "rotation_sensor_to_body"):
        return _matrix_like_to_numpy(accel_sensor.rotation_sensor_to_body)

    sensor_metadata = selected_sensor_metadata.get("Accelerometer", {})

    effective_orientation_matrix = sensor_metadata.get(
        "effective_orientation_matrix_sensor_to_body"
    )
    if effective_orientation_matrix is not None:
        return _matrix_like_to_numpy(effective_orientation_matrix)

    sensor_args = sensor_metadata.get("args", {})
    orientation = sensor_args.get("orientation", [0.0, 0.0, 0.0])

    orientation_array = _numeric_config_array(orientation)
    if orientation_array.shape == (3, 3):
        return orientation_array
    if orientation_array.shape == (3,):
        return _rocketpy_euler313_matrix(orientation_array)

    raise ValueError(
        "Accelerometer orientation must be 3 Euler angles or a 3x3 matrix"
    )


def _accelerometer_cross_axis_matrix():
    """Return RocketPy-style cross-axis mixing: ``S_clean -> S_out``."""
    sensor_metadata = selected_sensor_metadata.get("Accelerometer", {})
    sensor_args = sensor_metadata.get("args", {})
    return _utils_accelerometer_cross_axis_matrix(
        sensor_args.get("cross_axis_sensitivity", 0.0)
    )


def _accelerometer_body_to_clean_sensor_matrix():
    """Return the geometric projection: ``body -> S_clean``."""
    return _utils_accelerometer_body_to_clean_sensor_matrix(
        _accelerometer_sensor_to_body_matrix()
    )


def _stationary_calibration_acceleration_sensor_frame(
    *,
    gravity_m_s2,
    rail_inclination_deg,
    rail_heading_deg,
):
    """Return stationary-pad accelerometer output and initial attitude.

    The rocket is supported by the rail. The physical gravity vector points
    down, but the accelerometer specific-force vector measured during static
    calibration points up from earth to sky: ``[0, 0, +g]`` in inertial axes.

    The calibration payload is produced through this explicit chain:

        I(+Z support) -> B -> S_clean -> S_out
    """
    body_to_inertial = _rail_body_to_inertial_matrix(
        rail_inclination_deg,
        rail_heading_deg,
    )
    body_to_clean_sensor = _accelerometer_body_to_clean_sensor_matrix()
    clean_sensor_to_output = _accelerometer_cross_axis_matrix()

    specific_force_inertial = np.asarray([0.0, 0.0, gravity_m_s2], dtype=float)
    specific_force_body = body_to_inertial.T @ specific_force_inertial  # I -> B
    specific_force_sensor_clean = body_to_clean_sensor @ specific_force_body  # B -> S_clean
    specific_force_sensor_output = clean_sensor_to_output @ specific_force_sensor_clean  # S_clean -> S_out

    # Replay check: S_out -> S_clean -> B -> I.
    reconstructed_inertial = (
        body_to_inertial
        @ _accelerometer_sensor_to_body_matrix()
        @ np.linalg.inv(clean_sensor_to_output)
        @ specific_force_sensor_output
    )

    if reconstructed_inertial[2] <= 0.0:
        raise RuntimeError(
            "Synthetic calibration accelerometer vector is not pointing upward in "
            "the inertial frame. Check rail attitude and sensor orientation transforms."
        )

    e0, e1, e2, e3 = _rotation_matrix_to_rocketpy_quaternion(body_to_inertial)

    return {
        "sensor_output_m_s2": tuple(float(component) for component in specific_force_sensor_output),
        "sensor_clean_m_s2": tuple(float(component) for component in specific_force_sensor_clean),
        "body_m_s2": tuple(float(component) for component in specific_force_body),
        "inertial_m_s2": tuple(float(component) for component in specific_force_inertial),
        "reconstructed_inertial_m_s2": tuple(float(component) for component in reconstructed_inertial),
        "attitude_quaternion": (e0, e1, e2, e3),
        "body_to_inertial": body_to_inertial,
    }


# ----------------------------------------------------------------------
# CALIBRATION SENSOR MODEL REUSE
# ----------------------------------------------------------------------
def _apply_scalar_sensor_pipeline(sensor, value):
    """
    Apply the same scalar sensor post-processing used by RocketPy sensors:
    noise/random walk/bias -> temperature drift -> quantization.

    This intentionally calls the sensor object's own methods instead of
    duplicating the noise model in HIL code.
    """
    value = float(value)
    value = sensor.apply_noise(value)
    value = sensor.apply_temperature_drift(value)
    value = sensor.quantize(value)
    return float(value)


def _apply_vector_sensor_pipeline(sensor, values):
    """
    Apply the same vector sensor post-processing used by RocketPy inertial
    sensors: temperature drift -> noise/random walk/bias -> quantization.

    Returns a plain tuple so it can be packed into the HIL protocol.
    """
    from rocketpy.mathutils.vector_matrix import Vector

    value = Vector(values)
    value = sensor.apply_temperature_drift(value)
    value = sensor.apply_noise(value)
    value = sensor.quantize(value)

    return float(value.x), float(value.y), float(value.z)


def _offset_lat_lon(latitude_deg, longitude_deg, east_m, north_m):
    """Convert local GNSS meter offsets into latitude/longitude coordinates.

    RocketPy's GNSS receiver perturbs local east/north position in meters,
    computes drift and bearing from that local vector, then projects the result
    from the launch-site latitude/longitude over the configured earth radius.
    """
    earth_radius_m = float(env.earth_radius)

    # Match RocketPy's local-vector convention: x is east, y is north. Bearing
    # is clockwise from north and then used for the great-circle projection.
    drift_m = (east_m**2 + north_m**2) ** 0.5
    bearing_rad = 2 * math.pi - math.atan2(-east_m, north_m)

    latitude_rad = math.radians(float(latitude_deg))
    longitude_rad = math.radians(float(longitude_deg))
    angular_distance = drift_m / earth_radius_m

    offset_latitude_rad = math.asin(
        math.sin(latitude_rad) * math.cos(angular_distance)
        + math.cos(latitude_rad) * math.sin(angular_distance) * math.cos(bearing_rad)
    )
    offset_longitude_rad = longitude_rad + math.atan2(
        math.sin(bearing_rad) * math.sin(angular_distance) * math.cos(latitude_rad),
        math.cos(angular_distance)
        - math.sin(latitude_rad) * math.sin(offset_latitude_rad),
    )

    offset_longitude_deg = (math.degrees(offset_longitude_rad) + 540.0) % 360.0 - 180.0

    return math.degrees(offset_latitude_rad), offset_longitude_deg


def _apply_gnss_sensor_pipeline(sensor, latitude_deg, longitude_deg, altitude_m):
    """Apply RocketPy's GNSS accuracy model to stationary calibration data."""
    # NOTE: Mirror RocketPy GnssReceiver.measure(): position_accuracy is applied to
    # local east/north meter coordinates, altitude_accuracy is applied directly
    # to altitude, then the noisy local horizontal vector is converted to GPS.
    east_m = np.random.normal(0.0, sensor.position_accuracy)
    north_m = np.random.normal(0.0, sensor.position_accuracy)
    altitude_m = np.random.normal(float(altitude_m), sensor.altitude_accuracy)
    latitude_deg, longitude_deg = _offset_lat_lon(
        latitude_deg,
        longitude_deg,
        east_m,
        north_m,
    )

    return latitude_deg, longitude_deg, altitude_m


def _apply_calibration_sensor_models(
    accel_x_m_s2,
    accel_y_m_s2,
    accel_z_m_s2,
    pressure_pa,
    temperature_k,
    latitude_deg,
    longitude_deg,
    altitude_m,
):
    """
    Pass manually generated stationary-pad calibration values through the same
    selected RocketPy sensor objects used during Flight(...).
    
    NOTE: We intentionally do not call ``sensor.measure()`` during preflight
    calibration. Calibration runs before ``Flight(...)`` exists, and avoiding
    ``measure()`` keeps this synthetic stream from mutating or conflicting with
    the sensors state that RocketPy will use during the actual flight.

    Clean profile:
        the same methods are called, but the clean sensor configuration should
        have zero noise/bias/drift and zero resolution, so values stay clean.

    Noisy profile:
        the exact configured sensor noise, bias, random walk, temperature drift
        and quantization are applied by the RocketPy sensor objects.

    Temperature is currently not a RocketPy barometer measurement in RocketV2's
    callback path. The temperature is derived from the current altitude and the 
    configured Environment, so it is already noisy if the altitude is noisy.
    """
    try:
        accel_sensor = selected_sensor_instances["Accelerometer"]
        barometer_sensor = selected_sensor_instances["Barometer"]
        gnss_sensor = selected_sensor_instances["GnssReceiver"]
    except KeyError as exc:
        raise KeyError(
            "Calibration requires Accelerometer, Barometer, and GnssReceiver profiles"
        ) from exc

    noisy_accel_x_m_s2, noisy_accel_y_m_s2, noisy_accel_z_m_s2 = _apply_vector_sensor_pipeline(
        accel_sensor,
        (accel_x_m_s2, accel_y_m_s2, accel_z_m_s2),
    )

    noisy_pressure_pa = _apply_scalar_sensor_pipeline(barometer_sensor, pressure_pa)
    
    # GNSS noise in RocketPy doesn't follow the same pipeline of the others.
    # It is NOT: temperature_drift -> noise -> quantize
    noisy_latitude_deg, noisy_longitude_deg, noisy_altitude_m = _apply_gnss_sensor_pipeline(
        gnss_sensor,
        latitude_deg,
        longitude_deg,
        altitude_m,
    )

    # NOTE: this is a fallback for RocketPy versions that do not provide temperature in the callback.
    noisy_temperature_k = environment_temperature_at_asl_m(noisy_altitude_m)

    return (
        float(noisy_accel_x_m_s2),
        float(noisy_accel_y_m_s2),
        float(noisy_accel_z_m_s2),
        float(noisy_pressure_pa),
        float(noisy_temperature_k),
        float(noisy_latitude_deg),
        float(noisy_longitude_deg),
        float(noisy_altitude_m)
    )


def run_preflight_calibration(mailbox, max_samples, rate_hz):
    """
    Send stationary pad samples before Flight(...) is created.

    This lets the FC stay in CALIBRATING and fill its own calibration/filter
    buffers using the normal MSG_TYPE_SIM_INPUT path.

    Calibration accelerometer, barometer, and GPS values are passed through the
    same RocketPy sensor objects configured for Flight(...), so clean/noisy
    behavior comes from the selected Sensors profile and is not duplicated here.
    """
    global seq
    global PREFLIGHT_CALIBRATION_DURATION_S

    if max_samples <= 0:
        PREFLIGHT_CALIBRATION_DURATION_S = 0.0
        print("[CALIBRATION] Skipped: max_samples <= 0")
        return

    sample_period_s = 1.0 / float(rate_hz)
    conditions = get_launch_site_conditions()

    launch_site_pressure_pa = conditions["pressure_pa"]
    launch_site_temperature_k = conditions["temperature_k"]
    launch_site_latitude_deg = conditions["latitude"]
    launch_site_longitude_deg = conditions["longitude"]
    launch_site_elevation_m = conditions["elevation_m"]
    launch_site_gravity_m_s2 = conditions["gravity_m_s2"]

    flight_args = prepare_rocketpy_kwargs(require_config_section(cfg, "Flight"), CONFIG_DIR)
    rail_inclination_deg = float(flight_args["inclination"])
    rail_heading_deg = float(flight_args["heading"])

    calibration_geometry = _stationary_calibration_acceleration_sensor_frame(
        gravity_m_s2=launch_site_gravity_m_s2,
        rail_inclination_deg=rail_inclination_deg,
        rail_heading_deg=rail_heading_deg,
    )

    stationary_acceleration_sensor_output_m_s2 = calibration_geometry[
        "sensor_output_m_s2"
    ]
    stationary_acceleration_sensor_clean_m_s2 = calibration_geometry[
        "sensor_clean_m_s2"
    ]
    stationary_acceleration_body_m_s2 = calibration_geometry["body_m_s2"]
    stationary_acceleration_inertial_m_s2 = calibration_geometry["inertial_m_s2"]
    reconstructed_acceleration_inertial_m_s2 = calibration_geometry[
        "reconstructed_inertial_m_s2"
    ]
    initial_attitude_quaternion = calibration_geometry["attitude_quaternion"]

    (
        stationary_accel_x_m_s2,
        stationary_accel_y_m_s2,
        stationary_accel_z_m_s2,
    ) = stationary_acceleration_sensor_output_m_s2
    initial_e0, initial_e1, initial_e2, initial_e3 = initial_attitude_quaternion

    print(
        "[CALIBRATION] Sending "
        f"up to {max_samples} stationary samples at {rate_hz} Hz "
        f"(pressure={launch_site_pressure_pa:.2f} Pa, "
        f"temperature={launch_site_temperature_k:.2f} K, "
        f"g={launch_site_gravity_m_s2:.5f} m/s^2, "
        f"lat={launch_site_latitude_deg:.7f}, "
        f"lon={launch_site_longitude_deg:.7f}, "
        f"alt={launch_site_elevation_m:.2f} m, "
        f"rail_inclination={rail_inclination_deg:.2f} deg, "
        f"rail_heading={rail_heading_deg:.2f} deg, "
        f"sensor_profile={selected_sensor_profile_name})"
    )
    print(
        "[CALIBRATION] ideal stationary accelerometer clean sensor components "
        f"aS_clean=({stationary_acceleration_sensor_clean_m_s2[0]:.6f}, "
        f"{stationary_acceleration_sensor_clean_m_s2[1]:.6f}, "
        f"{stationary_acceleration_sensor_clean_m_s2[2]:.6f}) m/s^2 "
        f"|a|={np.linalg.norm(stationary_acceleration_sensor_clean_m_s2):.6f} m/s^2"
    )
    print(
        "[CALIBRATION] ideal stationary accelerometer payload output "
        f"aS_out=({stationary_accel_x_m_s2:.6f}, "
        f"{stationary_accel_y_m_s2:.6f}, "
        f"{stationary_accel_z_m_s2:.6f}) m/s^2 "
        f"|a|={np.linalg.norm(stationary_acceleration_sensor_output_m_s2):.6f} m/s^2"
    )
    print(
        "[CALIBRATION] stationary specific force check "
        f"body=({stationary_acceleration_body_m_s2[0]:.6f}, "
        f"{stationary_acceleration_body_m_s2[1]:.6f}, "
        f"{stationary_acceleration_body_m_s2[2]:.6f}) m/s^2, "
        f"inertial=({stationary_acceleration_inertial_m_s2[0]:.6f}, "
        f"{stationary_acceleration_inertial_m_s2[1]:.6f}, "
        f"{stationary_acceleration_inertial_m_s2[2]:.6f}) m/s^2, "
        f"replay_check=({reconstructed_acceleration_inertial_m_s2[0]:.6f}, "
        f"{reconstructed_acceleration_inertial_m_s2[1]:.6f}, "
        f"{reconstructed_acceleration_inertial_m_s2[2]:.6f}) m/s^2"
    )
    print(
        "[CALIBRATION] synthetic rail attitude quaternion "
        f"e=({initial_e0:.9f}, {initial_e1:.9f}, "
        f"{initial_e2:.9f}, {initial_e3:.9f})"
    )

    sent_samples = 0

    for sample_index in range(max_samples):
        calibration_sim_time_s = sample_index * sample_period_s

        (
            sample_accel_x_m_s2,
            sample_accel_y_m_s2,
            sample_accel_z_m_s2,
            sample_pressure_pa,
            sample_temperature_k,
            sample_latitude_deg,
            sample_longitude_deg,
            sample_altitude_m,
        ) = (
            _apply_calibration_sensor_models(
                accel_x_m_s2=stationary_accel_x_m_s2,
                accel_y_m_s2=stationary_accel_y_m_s2,
                accel_z_m_s2=stationary_accel_z_m_s2,
                pressure_pa=launch_site_pressure_pa,
                temperature_k=launch_site_temperature_k,
                latitude_deg=launch_site_latitude_deg,
                longitude_deg=launch_site_longitude_deg,
                altitude_m=launch_site_elevation_m,
            )
        )

        payload = hil_communication.build_sim_input_payload(
            sequence_number=seq,
            hil_sim_time_s=calibration_sim_time_s,
            ax_m_s2=sample_accel_x_m_s2,
            ay_m_s2=sample_accel_y_m_s2,
            az_m_s2=sample_accel_z_m_s2,
            pressure_pa=sample_pressure_pa,
            temperature_k=sample_temperature_k,
            latitude_deg=sample_latitude_deg,
            longitude_deg=sample_longitude_deg,
            altitude_m=sample_altitude_m,
        )
        mailbox.put(payload)
        sent_samples += 1

        append_hil_log_sample(
            seq_value=seq,
            hil_sim_time_s=calibration_sim_time_s,
            accel_x_m_s2=sample_accel_x_m_s2,
            accel_y_m_s2=sample_accel_y_m_s2,
            accel_z_m_s2=sample_accel_z_m_s2,
            pressure_pa=sample_pressure_pa,
            temperature_k=sample_temperature_k,
            latitude_deg=sample_latitude_deg,
            longitude_deg=sample_longitude_deg,
            altitude_m=sample_altitude_m,

            # Synthetic stationary truth state for pre-Flight calibration.
            # Use the same rail attitude used to create the synthetic
            # accelerometer samples so the 3D replay is physically consistent.
            x=0.0, y=0.0, z=launch_site_elevation_m,
            vx=0.0, vy=0.0, vz=0.0,
            e0=initial_e0, e1=initial_e1, e2=initial_e2, e3=initial_e3,
            omega1=0.0, omega2=0.0, omega3=0.0,
        )

        if (
            sample_index == 0
            or (sample_index + 1) == max_samples
            or (sample_index + 1) % max(1, rate_hz) == 0
        ):
            with command_state.lock:
                fsm_name = command_state.fsm_state_name
            print(
                f"[CALIBRATION] sample {sample_index + 1}/{max_samples} "
                f"hil_sim_time={calibration_sim_time_s:.3f}s seq={seq} fsm={fsm_name} "
                f"p={sample_pressure_pa:.2f}Pa T={sample_temperature_k:.2f}K "
                f"a=({sample_accel_x_m_s2:.3f},"
                f"{sample_accel_y_m_s2:.3f},"
                f"{sample_accel_z_m_s2:.3f}) "
                f"gps=({sample_latitude_deg:.7f},"
                f"{sample_longitude_deg:.7f},"
                f"{sample_altitude_m:.2f})"
            )

        seq += 1

        if command_state.fsm_ready_event.is_set():
            print(f"[CALIBRATION] FC reported READY_FOR_LAUNCH after {sent_samples} samples")
            break

    PREFLIGHT_CALIBRATION_DURATION_S = sent_samples * sample_period_s

    if not command_state.fsm_ready_event.is_set():
        with command_state.lock:
            fsm_name = command_state.fsm_state_name
        raise TimeoutError(
            "Calibration did not complete before the configured sample limit. "
            f"Sent {sent_samples}/{max_samples} samples at {rate_hz} Hz; "
            f"last_fsm_state={fsm_name}. Increase --calibration-samples or fix the FC/mock."
        )

    print(
        "[CALIBRATION] Completed. Flight samples will start at "
        f"hil_sim_time={PREFLIGHT_CALIBRATION_DURATION_S:.3f}s"
    )


mailbox = hil_communication.OneSlotMailbox()
handlers_dict = {
    "on_open_drogue": on_open_drogue,
    "on_open_main": on_open_main,
    "on_set_air_brakes": on_set_air_brakes,
    "on_fsm_state": on_fsm_state,
}
handlers = hil_communication.CommandHandlers(**handlers_dict)

# Connect RocketPy's state/sensor callback to the HIL packet queue.
state_ctrl = rocket.add_state_and_sensors_logger(callback=enqueue_data, sampling_rate=sampling_rate)
print("State Logger... READY")

# Let FC commands set the RocketPy airbrake deployment level in real time.
rocket.add_air_brakes(
    drag_coefficient_curve=airbrakes_drag_function,
    controller_function=airbrakes_controller,
    sampling_rate=sampling_rate,
    clamp=True,
)

esp_connected = threading.Semaphore(0)
esp_reconnected = threading.Semaphore(0)

th = hil_communication.tcp_client_thread_start(
    esp_connected,
    mailbox,
    handlers,
    esp_reconnected,
)

# The simulation waits here until the flight controller TCP server is reachable.
print("Wait for ESP connection...", end="")
esp_connected.acquire()
print("READY")

# Optional startup reset gives the FC a clean state before calibration begins.
if startup_reset_enabled:
    print("[RESET] Sending startup reset to flight controller...")
    command_state.reset()
    mailbox.put(hil_communication.RESET_SIMULATION)

    print("[RESET] Waiting for flight controller to restart HIL TCP server...")
    if not esp_reconnected.acquire(timeout=startup_reset_timeout_s):
        mailbox.put(hil_communication.STOP_COMMUNICATION)
        th.join(timeout=5.0)
        raise TimeoutError(
            "Flight controller did not reconnect after startup reset "
            f"within {startup_reset_timeout_s} s"
        )
    print("[RESET] Flight controller reconnected after startup reset")
else:
    print("[RESET] Startup reset skipped by CLI")

# ----------------------------------------------------------------------
# BUILD CAPTURE METADATA
# ----------------------------------------------------------------------
def build_capture_metadata():
    flight_args = prepare_rocketpy_kwargs(require_config_section(cfg, "Flight"), CONFIG_DIR)

    metadata = {
        "rocket": rocket_model,
        "config_path": str(CONFIG_PATH),
        "sampling_rate_hz": sampling_rate,
        "environment": env_args,
        "atmospheric_model": atmospheric_model_args,
        "flight": flight_args,
        "environment_latitude": require_environment_float("latitude"),
        "environment_longitude": require_environment_float("longitude"),
        "environment_elevation_m": require_environment_float("elevation"),
        "rail_length_m": flight_args["rail_length"],
        "inclination_deg": flight_args["inclination"],
        "heading_deg": flight_args["heading"],
        "simulation_start_date_utc": environment_set_date,
        "environment_calls": environment_applied_calls,
        "sensor_profile": selected_sensor_profile_name,
        "sensors": selected_sensor_metadata,
        "startup_reset_enabled": startup_reset_enabled,
        "startup_reset_timeout_s": startup_reset_timeout_s,
        "calibration_samples": calibration_samples,
        "calibration_rate_hz": calibration_rate,
        "preflight_calibration_duration_s": PREFLIGHT_CALIBRATION_DURATION_S,
        "notes": (
            "HIL payload stream sent to ESP32 flight controller. "
            "A startup reset is sent before calibration unless disabled. "
            "Pre-flight calibration samples are sent before Flight(...) is created. "
            "Calibration accelerometer/barometer samples reuse the selected RocketPy "
            "sensor objects for noise, drift, bias and quantization. "
            "The TCP link is closed locally at the end; no final FC reset is sent. "
            "Reference frames: I(+X east,+Y north,+Z up), B(body,+Z nose), "
            "S_clean(orthogonal sensor), S_out(payload after cross-axis mixing)."
        ),
    }

    return metadata


# ----------------------------------------------------------------------
# RUN CALIBRATION, FLIGHT, CLEANUP, AND CAPTURE OUTPUT
# ----------------------------------------------------------------------
capture_file = create_capture_file(BASE_DIR / "hil_captures")

try:
    run_preflight_calibration(
        mailbox=mailbox,
        max_samples=calibration_samples,
        rate_hz=calibration_rate,
    )

    print("Setup completed. Starting Flight...")

    flight_args = prepare_rocketpy_kwargs(require_config_section(cfg, "Flight"), CONFIG_DIR)
    test_flight = Flight(
        rocket=rocket,
        environment=env,
        **flight_args,
    )

    print("Flight... COMPLETED")

finally:
    # End of Python run: do not send MSG_TYPE_SIM_RESET.
    # Just stop producing data and close the TCP socket cleanly.
    mailbox.put(hil_communication.STOP_COMMUNICATION)
    th.join(timeout=5.0)
    if th.is_alive():
        print("[TCP] Warning: communication thread did not stop within timeout")

save_hil_capture(
    filename=capture_file,
    hil_log=hil_log,
    hil_events=hil_events,
    metadata=build_capture_metadata(),
)

plot_hil_log(
    hil_log,
    hil_events,
)

# Keep RocketPy's built-in trajectory plot available for manual debugging.
# test_flight.plots.trajectory_3d()
