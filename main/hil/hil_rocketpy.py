import time
import threading
from pathlib import Path
import math

from rocketpy import Environment, Flight, SolidMotor, RocketV2
from rocketpy import Accelerometer
from rocketpy import Barometer
from rocketpy import GnssReceiver  

# communication.py
import communication

class CommandState:
    def __init__(self):
        self.lock = threading.Lock()

        self.sim_time = 0
        self.open_main = False
        self.open_drogue = False
        self.airbrakes_lvl = 0.0

command_state = CommandState()

# ----------------------------------------------------------------------
# BASE PATH
# ----------------------------------------------------------------------
try:
    BASE_DIR = Path(__file__).resolve().parent 
except NameError:
    # If running in an environment where __file__ does not exist
    BASE_DIR = Path(".").resolve()

# ----------------------------------------------------------------------
# ENVIRONMENT
# ----------------------------------------------------------------------
# Real weather data from given date
env = Environment(latitude=44.290583, longitude=12.027111, elevation=18)
env.set_date((2025, 5, 9, 12))  # Hour given in UTC time (yyyy/mm/dd/hh)

# Wind magnitude on the ground in (m/s)
wind_magnitude_ground = 8.7                          # m/s, EuRoC limit is 8.7m/s on the ground
# Heading of the wind from North in degrees
wind_heading = 315                                  # degrees from North

wind_heading_r = math.radians(wind_heading)         # tranforms into radians
s_heading = math.sin(wind_heading_r)                # sin of the wind profile
c_heading = math.cos(wind_heading_r)                # cos of the wind profile

# Creates an array of values to generate a wind profile
custom_wind_u = [
    ( 0 , wind_magnitude_ground * s_heading),
    ( 50 , 1.05 * wind_magnitude_ground * s_heading),
    ( 100 , 1.1 * wind_magnitude_ground * s_heading),
    ( 150 , 1.15 * wind_magnitude_ground * s_heading),
    ( 200 , 1.2 * wind_magnitude_ground * s_heading),
    ( 250 , 1.25 * wind_magnitude_ground * s_heading),
    ( 300 , 1.3 * wind_magnitude_ground * s_heading),
    ( 15000 , 1.3 * wind_magnitude_ground * s_heading),
]

custom_wind_v = [
    ( 0 , wind_magnitude_ground * c_heading),
    ( 50 , 1.05 * wind_magnitude_ground * c_heading),
    ( 100 , 1.1 * wind_magnitude_ground * c_heading),
    ( 150 , 1.15 * wind_magnitude_ground * c_heading),
    ( 200 , 1.2 * wind_magnitude_ground * c_heading),
    ( 250 , 1.25 * wind_magnitude_ground * c_heading),
    ( 300 , 1.3 * wind_magnitude_ground * c_heading),
    ( 15000 , 1.3 * wind_magnitude_ground * c_heading),
]

env.set_atmospheric_model(
    type="custom_atmosphere",
    wind_u=custom_wind_u,
    wind_v=custom_wind_v,
)

print("Environment... READY")

# ----------------------------------------------------------------------
# PARACHUTE LOGIC
# ----------------------------------------------------------------------

# Definition of global variables, to be used inside and outside parachute functions
global last_negative_time, apogee_detected, sampling_rate, parachute_timer
last_negative_time = None
apogee_detected = False
sampling_rate = 20
parachute_stopwatch = 0

drogue_deployed = False
main_deployed = False
deployment_level = 0.0

def simulator_check_drogue_opening(p, h, y):
    with command_state.lock:
        return command_state.open_drogue

def simulator_check_main_opening(p, h, y):
    with command_state.lock:
        return command_state.open_main

# ----------------------------------------------------------------------
# MOTOR DATA
# ----------------------------------------------------------------------
Pro75M8187 = SolidMotor(
    thrust_source=str(BASE_DIR / "Fred/SRAD_thrustcurve_BRICO_45_7mm.csv"),
    dry_mass=0.7871568876139587,
    dry_inertia=(0.66, 0.66, 0.00001),
    nozzle_radius=13.71 / 1000,
    grain_number=1,
    grain_density=0.19694061309132402/(3.14*(((0.033 / 2)**2)-((0.013 / 2)**2))*0.147),
    grain_outer_radius=0.033 / 2,
    grain_initial_inner_radius=0.013 / 2,
    grain_initial_height=0.147,
    grain_separation=3 / 1000,
    grains_center_of_mass_position=125 / 1000,
    center_of_dry_mass_position=99.41 / 1000,
    nozzle_position=0,
    burn_time=0.640,
    throat_radius=9.50 / 1000,
    coordinate_system_orientation="nozzle_to_combustion_chamber",
)

print("Motor... READY")

# ----------------------------------------------------------------------
# ROCKET
# ----------------------------------------------------------------------
Nemesis = RocketV2(
    radius=42.5 / 1000,
    mass=(2595 / 1000),  # Dry mass + ballast
    inertia=(1.49, 1.49, 0.01),
    power_off_drag=str(BASE_DIR / "Fred/FRED_v2.0_CD_power_off.csv"),
    power_on_drag=str(BASE_DIR / "Fred/FRED_v2.0_CD_power_on.csv"),
    center_of_mass_without_motor=373 / 1000,
    coordinate_system_orientation="nose_to_tail",
)

rail_buttons = Nemesis.set_rail_buttons(
    upper_button_position=424 / 1000,
    lower_button_position=625 / 1000,
    angular_position=0,
)

Nemesis.add_motor(Pro75M8187, position=0.853) # Tail position + tail length (0.810 + 0.043)

nose_cone = Nemesis.add_nose(length=0.14, kind="elliptical", position=0)

fin_set = Nemesis.add_trapezoidal_fins(
    n=3,
    root_chord=0.12,
    tip_chord=0.03,
    span=0.12,
    position=0.686,
    cant_angle=0,
    sweep_angle=30.3,
)

tail = Nemesis.add_tail(
    top_radius= 42.5 / 1000,
    bottom_radius= 30 / 1000,
    length= 43 / 1000,
    position= 810 / 1000,
)

Main = Nemesis.add_parachute(
    "Main",
    cd_s= 0.97 * 1.168,
    trigger=simulator_check_main_opening,
    sampling_rate=105,
    lag=2.1, # lag_se + lag_rec (0.1 + 2)
    noise=(0, 6.5, 0.3),
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
    name="Clean Accelerometer"  
)  
Nemesis.add_sensor(accel_clean, position=0)
  
barometer_clean = Barometer(  
    sampling_rate=sampling_rate,  
    noise_density=0,  
    random_walk_density=0,  
    constant_bias=0,  
    temperature_bias=0,  
    temperature_scale_factor=0,  
    name="Clean Barometer"  
)  
Nemesis.add_sensor(barometer_clean, position=0)

gnss_clean = GnssReceiver(  
    sampling_rate=sampling_rate,
    position_accuracy=0,
    altitude_accuracy=0,
    name="Clean GPS"  
)  
Nemesis.add_sensor(gnss_clean, position=0)
  
print("Sensors... READY")

# ----------------------------------------------------------------------
# STATE LOGGER + COMMUNICATION WITH FLIGHT CONTROLLER
# ----------------------------------------------------------------------
def on_open_main(sim_time):
    with command_state.lock:
        if command_state.sim_time <= sim_time:
            command_state.sim_time = sim_time
            if command_state.open_main == False: 
                print(f"[ESP32_cmd] 'main_deployed'") 
            command_state.open_main = True
        else:
            print("[E]: time mismatch, overwriting with old values new stuff.")

def on_open_drogue(sim_time):
    with command_state.lock:
        if command_state.sim_time <= sim_time:
            command_state.sim_time = sim_time
            if command_state.open_drogue == False: 
                print(f"[ESP32_cmd]: 'drogue_deployed'")
            command_state.open_drogue = True
        else:
            print("[E]: time mismatch, overwriting with old values new stuff.")

def on_set_air_brakes(sim_time, lvl):
    with command_state.lock:
        if command_state.sim_time <= sim_time:
            command_state.sim_time = sim_time
            if command_state.airbrakes_lvl != lvl: 
                print(f"[ESP32_cmd]: deployment_level={lvl}")
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

def enqueue_data(t, state, sensors):
    global seq
    global last_sent_t

    if last_sent_t is not None and abs(t - last_sent_t) < TIMESTAMP_EPS:
        return

    if seq % sampling_rate == 0:
        print(f"t_sim = {t:.6f} | seq={seq}")
    
    ax = sensors["ax"]
    ay = sensors["ay"]
    az = sensors["az"]
    p = sensors["p"]
    lat = sensors["lat"]
    lon = sensors["lon"]
    alt = sensors["alt"]

    payload = communication.build_payload(seq, t, ax, ay, az, p, lat, lon, alt)
    mailbox.put(payload)

    last_sent_t = t
    seq += 1

mailbox = communication.OneSlotMailbox()
handlers_dict = {
    "on_open_drogue": on_open_drogue,
    "on_open_main": on_open_main,
    "on_set_air_brakes": on_set_air_brakes
}
handlers = communication.CommandHandlers(**handlers_dict)

state_ctrl = Nemesis.add_state_and_sensors_logger(callback=enqueue_data, sampling_rate=sampling_rate)
print("State Logger... READY")

Nemesis.add_air_brakes(
    drag_coefficient_curve = airbrakes_drag_function,
    controller_function = airbrakes_controller,
    sampling_rate = sampling_rate,
    clamp=True,
)

esp_connected = threading.Semaphore(0)
th = communication.tcp_client_thread_start(esp_connected, mailbox, handlers)

print("Wait for ESP connection...", end="")
esp_connected.acquire()
print("READY")

# ----------------------------------------------------------------------
# RUN REAL-TIME SIMULATION
# ----------------------------------------------------------------------
print("Setup completed. Starting Flight...")
test_flight = Flight(
    rocket=Nemesis,
    environment=env,
    rail_length=2,
    inclination=75,
    heading=305,
    time_overshoot=False
)

reset_choice = input(
    "Send simulation reset to the flight controller before showing the plot? [y/N]: "
).strip().lower()

if reset_choice in ("y", "yes"):
    mailbox.put(communication.RESET_SIMULATION)
    print("RESET_SIM requested")
else:
    print("RESET_SIM skipped")

print("Flight... COMPLETED")

test_flight.plots.trajectory_3d()