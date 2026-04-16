from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Optional, Sequence, List

# Rocket base class
from rocketpy.rocket.rocket import Rocket
from rocketpy.environment.environment import Environment
from rocketpy.control.controller import _Controller

class RocketV2(Rocket):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def add_state_and_sensors_logger(
        self,
        callback,
        sampling_rate,
        controller_name="State Logger"
    ):
        # a collection of objects that the controller function can access and potentially modify.
        _interactive_objects = None
        
        # a list of the initial values of the variables that the controller function returns.
        _initial_observed_variables = None

        # define custom controller
        def _controller_function(
            _sim_time,              # current simulation time in seconds
            _sampling_rate,         # rate at which the controller function is called
            _state,                 # [x, y, z, vx, vy, vz, e0, e1, e2, e3, wx, wy, wz]
            _state_history,         # a record of the rocket's state at each step throughout the simulation
            _observed_variables,    # a list containing the variables that the controller function returns.
            _interactive_objects,   # a list containing the objects that the controller function can interact with
            _sensors,               # list of sensors that are attached to the rocket
            # _flight = None          # flight
        ):
            if _sensors is None:
                print("_sensors is None. Skip.")
                return

            # The most recent measurements of the sensors are provided with the ``sensor.measurement`` attribute.            
            accel_clean     = _sensors[0].measurement
            barometer_clean = _sensors[1].measurement
            gnss_clean      = _sensors[2].measurement

            if accel_clean is None:
                print("accel_clean is None. Skip.")
                return

            if barometer_clean is None:
                print("barometer_clean is None. Skip.")
                return
            
            if gnss_clean is None:
                print("gnss_clean is None. Skip.")
                return

            t = _sim_time
            x, y, z = _state[0], _state[1], _state[2]
            vx, vy, vz = _state[3], _state[4], _state[5]
            e0, e1, e2, e3 = _state[6], _state[7], _state[8], _state[9]
            omega1, omega2, omega3 = _state[10], _state[11], _state[12]

            state = {
                "x": x,
                "y": y,
                "z": z,
                "vx": vx,
                "vy": vy,
                "vz": vz,
                "e0": e0,
                "e1": e1,
                "e2": e2,
                "e3": e3,
                "omega1": omega1,
                "omega2": omega2,
                "omega3": omega3
            }

            ax = accel_clean[0]
            ay = accel_clean[1]
            az = accel_clean[2]
            p = barometer_clean
            lat = gnss_clean[0]
            lon = gnss_clean[1]
            alt = gnss_clean[2]

            sensors = {
                "ax": ax,
                "ay": ay,
                "az": az,
                "p": p,
                "lat": lat,
                "lon": lon,
                "alt": alt,
            }

            callback(t, state, sensors)

        _controller = _Controller(
            interactive_objects=_interactive_objects,
            controller_function=_controller_function,
            sampling_rate=sampling_rate,
            initial_observed_variables=_initial_observed_variables,
            name=controller_name
        )

        self._add_controllers(_controller)

        return _controller
    
