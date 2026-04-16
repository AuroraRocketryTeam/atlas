import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import time as pytime
from pathlib import Path
import csv
import math
import pandas as pd

from rocketpy import Environment, Flight, SolidMotor, RocketV2
from rocketpy import Accelerometer
from rocketpy import Barometer
from rocketpy import GnssReceiver  

import threading
from typing import Generic, TypeVar, Optional
import serial
import struct

import communication


class CommandState:
    def __init__(self):
        self.lock = threading.Lock()

        self.sim_time = 0.0
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
env = Environment(latitude=39.3901806, longitude=-8.289189, elevation=160)
env.set_date((2025, 10, 13, 16))  # Hour given in UTC time (yyyy/mm/dd/hh)

# Real environment: De-comment the following lines to use real atmosphere data
# env.set_atmospheric_model(type="Forecast", file="GFS")

# Custom environment:De-comment the following lines to use custom atmosphere
env.set_atmospheric_model(type="custom_atmosphere",                                                                                         
     wind_u=[                                                                                                          
         (0, 0), # 10.60 m/s at 0 m                                                                                
         (4500, 0), # 10.60 m/s at 3000 m
     ],                                                                                                                
     wind_v=[                                                                                                          
         (0, 0), # -16.96 m/s at 3000 m   
         (4500, 0)
     ],)


env.max_expected_height = 4500

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
    # return the value of the flag if we need to open the drogue.
    # If the ESP32 sent a message in which DROGUE_OPEN is true, then in the callback
    # we have set the global flag to True.
    with command_state.lock:
        return command_state.open_drogue


def simulator_check_main_opening(p, h, y):
    # return the value of the flag if we need to open the drogue.
    # If the ESP32 sent a message in which MAIN_OPEN is true, then in the callback
    # we have set the global flag to True.
    with command_state.lock:
        return command_state.open_main


# ----------------------------------------------------------------------
# MOTOR DATA
# ----------------------------------------------------------------------
Pro75M8187 = SolidMotor(
    thrust_source=str(BASE_DIR / "Cesaroni_8187M1545_P.csv"),
    dry_mass=0,
    dry_inertia=(0, 0, 0),
    nozzle_radius=29 / 1000,
    grain_number=6,
    grain_density=1758.7,
    grain_outer_radius=35.9 / 1000,
    grain_initial_inner_radius=18.1 / 1000,
    grain_initial_height=156.17 / 1000,
    grain_separation=3 / 1000,
    grains_center_of_mass_position=-0.7343,
    center_of_dry_mass_position=0,
    nozzle_position=-1.296,
    burn_time=5.3,
    throat_radius=20 / 1000,
    coordinate_system_orientation="nozzle_to_combustion_chamber",
)

print("Motor... READY")

# ----------------------------------------------------------------------
# ROCKET
# ----------------------------------------------------------------------
Nemesis = RocketV2(
    radius=75 / 1000,
    mass=22.740,
    inertia=(14.304, 14.304, 0.078),
    power_off_drag=str(BASE_DIR / "Nemesis150_v4.0_RAS_CDMACH_pwrOFF.csv"),
    power_on_drag=str(BASE_DIR / "Nemesis150_v4.0_RAS_CDMACH_pwrON.csv"),
    center_of_mass_without_motor=0,
    coordinate_system_orientation="tail_to_nose",
)

rail_buttons = Nemesis.set_rail_buttons(
    upper_button_position=0.980,
    lower_button_position=-0.239,
    angular_position=0,
)

Nemesis.add_motor(Pro75M8187, position=0)

nose_cone = Nemesis.add_nose(length=0.45, kind="vonKarman", position=1.635)

fin_set = Nemesis.add_trapezoidal_fins(
    n=3,
    root_chord=0.30,
    tip_chord=0.093,
    span=0.16,
    position=-0.855,
    cant_angle=0,
    sweep_angle=58,
)

tail = Nemesis.add_tail(
    top_radius=0.075, bottom_radius=0.046, length=0.116, position=-1.155,
)

Main = Nemesis.add_parachute(
    "Main",
    cd_s=0.97 * 10.5070863,
    trigger=simulator_check_main_opening,
    sampling_rate=sampling_rate,
    lag=1.73,
    noise=(0, 6.5, 0.3),
)

Drogue = Nemesis.add_parachute(
    "Drogue",
    cd_s=0.97 * 0.6566929,
    trigger=simulator_check_drogue_opening,
    sampling_rate=sampling_rate,
    lag=1.73,
    noise=(0, 6.5, 0.3),
)

print("Rocket... READY")

# ----------------------------------------------------------------------
# SENSORS
# ----------------------------------------------------------------------
accel_clean = Accelerometer(  
    sampling_rate=50,
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
    sampling_rate=50,  
    noise_density=0,  
    random_walk_density=0,  
    constant_bias=0,  
    temperature_bias=0,  
    temperature_scale_factor=0,  
    name="Clean Barometer"  
)  
Nemesis.add_sensor(barometer_clean, position=0)

gnss_clean = GnssReceiver(  
    sampling_rate=50,
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
    print(f"[ESP32_cmd] 'main_deployed'")
    with command_state.lock:
        if command_state.sim_time <= sim_time:
            command_state.sim_time = sim_time
            command_state.open_main = True
        else:
            print("[E]: time mismatch, overwriting with old values new stuff.")

def on_open_drogue(sim_time):
    print(f"[ESP32_cmd]: 'drogue_deployed'")
    with command_state.lock:
        if command_state.sim_time <= sim_time:
            command_state.sim_time = sim_time
            command_state.open_drogue = True
        else:
            print("[E]: time mismatch, overwriting with old values new stuff.")

def on_set_air_brakes(sim_time, lvl):
    with command_state.lock:
        if command_state.sim_time <= sim_time:
            command_state.sim_time = sim_time
            if lvl > 0:
                print(f"[ESP32_cmd]: deployment_level={lvl}")
            command_state.airbrakes_lvl = lvl
        else:
            print("[E]: time mismatch, overwriting with old values new stuff.")

def airbrakes_drag_function(level, mach):
    # linear approx
    base_drag_added = 0.5
    # print(f"[py] airbrakes_drag_function: {level}")
    return base_drag_added * level

def airbrakes_controller(time, sampling_rate, state_vector, state_history, observed_variables, interactive_objects):
    airbrake = interactive_objects
    with command_state.lock:
        # print(f"[py] airbrakes_controller: {time:03.2f}, {deployment_level}")
        airbrake.deployment_level = command_state.airbrakes_lvl
    return airbrake

counter = 0
seq = 0
def enqueue_data(t, state, sensors):
    # x = state["x"]
    # y = state["y"]
    # z = state["z"]
    # vx = state["vx"]
    # vy = state["vy"]
    # vz = state["vz"]
    # e0 = state["e0"]
    # e1 = state["e1"]
    # e2 = state["e2"]
    # e3 = state["e3"]
    # omega1 = state["omega1"]
    # omega2 = state["omega2"]
    # omega3 = state["omega3"]
    global counter
    global seq
    if counter == 0:
        print(f"t_sim = {t}")
    
    if counter >= sampling_rate:
        counter = 0
        print(f"t_sim = {t}")
    
    ax = sensors["ax"]
    ay = sensors["ay"]
    az = sensors["az"]
    p = sensors["p"]
    lat = sensors["lat"]
    lon = sensors["lon"]
    alt = sensors["alt"]

    # print(f"[py]: sim_time={t}, ax={ax}, ay={ay}, az={az}, p={p}, lat={lat}, lon={lon}, alt={alt}")
    payload = communication.build_payload(seq, t, ax, ay, az, p, lat, lon, alt)
    mailbox.put(payload)

    counter += 1
    seq+=1

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
    rail_length=12,
    inclination=84,
    heading=144,
    time_overshoot=False
)

print("Flight... COMPLETED")

test_flight.plots.trajectory_3d()
