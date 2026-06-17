# Rocket HIL Simulation

This document describes the current Python RocketPy hardware-in-the-loop
simulation used to drive the flight-controller HIL TCP interface.

The design goal is strict and explicit:

- The RocketPy config JSON is the single source of truth for rocket, flight,
  environment and sensor parameters.
- Constructor arguments are passed through permissively to RocketPy.
- HIL-only metadata uses keys starting with `_`.
- Mandatory fields fail fast with explicit errors.
- Sensor profiles are selected explicitly from the CLI.
- The simulator and flight controller run in lockstep: one sample sent, one
  command received.

## Main Files

- `hil_rocketpy.py`: top-level launcher. Builds RocketPy objects, starts the TCP
  communication thread, sends calibration samples, runs `Flight(...)`, saves and
  plots the capture.
- `hil_config.py`: JSON loading, formula resolution, config path normalization,
  RocketPy kwargs preparation, ordered `_calls`, and sensor profile resolution.
- `hil_communication.py`: TCP client, framing, packet encode/decode, one-slot
  lockstep mailbox, and command handlers.
- `hil_capture.py`: capture JSON creation/loading, run summary and plots.
- `mock_manny_fc_server.py`: self-contained mock flight-controller TCP server for
  testing without Manny.
- `*_rocketpy_config.json`: rocket-specific RocketPy/HIL configuration files.

## Running A Simulation

The launcher requires an explicit rocket and an explicit sensor profile:

```bash
python hil_rocketpy.py --rocket fred --sensor-profile clean
python hil_rocketpy.py --rocket fred --sensor-profile noisy
python hil_rocketpy.py --rocket fred --sensor-profile very_noisy
```

Useful options:

```bash
python hil_rocketpy.py --rocket fred --sensor-profile clean --sampling-rate 50
python hil_rocketpy.py --rocket fred --sensor-profile clean --calibration-samples 500
python hil_rocketpy.py --rocket fred --sensor-profile clean --calibration-rate 50
python hil_rocketpy.py --rocket fred --sensor-profile clean --no-startup-reset
python hil_rocketpy.py --rocket fred --sensor-profile clean --startup-reset-timeout 30
```

The flight-controller host and port are environment variables:

```bash
HIL_FC_HOST=192.168.42.1 HIL_FC_PORT=5000 python hil_rocketpy.py --rocket fred --sensor-profile clean
```

For local mock testing:

```bash
python mock_manny_fc_server.py
HIL_FC_HOST=127.0.0.1 python hil_rocketpy.py --rocket fred --sensor-profile clean --calibration-samples 300
```

The mock has no CLI arguments. Its tunables live in `MockConfig` inside
`mock_manny_fc_server.py`.

## Configuration Model

The config file is intentionally close to RocketPy naming. Sections such as
`Environment`, `SolidMotor`, `RocketV2` and `Flight` contain kwargs for the
corresponding RocketPy constructor. Rocket methods such as `add_nose` and
`add_trapezoidal_fins` use the RocketPy method names.

Example:

```python
motor = SolidMotor(**prepare_rocketpy_kwargs(require_config_section(cfg, "SolidMotor"), CONFIG_DIR))
```

Rules:

- Non-underscore keys are passed to RocketPy after mechanical JSON conversion.
- Keys starting with `_` are HIL/config metadata and are not passed to RocketPy.
- Relative file paths are resolved relative to the selected rocket config folder.
- Lists for tuple-like RocketPy parameters are converted to tuples.
- Pair-list atmospheric fields such as `wind_u`, `wind_v`, `pressure` and
  `temperature` are converted from JSON lists to tuple pairs.
- Missing mandatory sections fail immediately.

Expected mandatory sections:

- `Environment`
- `SolidMotor`
- `RocketV2`
- `add_motor`
- `Flight`
- `Sensors`

Physically optional rocket features, such as fins, tail, nose, rail buttons or
parachutes, are applied only when their section is present.

## Ordered Method Calls With `_calls`

Some RocketPy object setup cannot be represented as constructor kwargs. The main
case is `Environment`: the date must be set before loading some atmospheric
models.

The config convention for this is `_calls`:

```json
{
  "Environment": {
    "latitude": 44.290583,
    "longitude": 12.027111,
    "elevation": 18,
    "_calls": [
      {
        "method": "set_date",
        "kwargs": {
          "date": [2025, 5, 9, 12]
        }
      },
      {
        "method": "set_atmospheric_model",
        "kwargs": {
          "type": "Ensemble",
          "file": "Villafranca_ensemble_5to11may2020to2026.nc"
        }
      }
    ]
  }
}
```

The launcher does this in order:

1. Pass non-underscore keys to `Environment(**kwargs)`.
2. Apply `Environment._calls` in list order.
3. Reject direct `Environment.set_date`, direct
   `Environment.set_atmospheric_model`, top-level `set_date`, top-level
   `set_atmospheric_model`, and dotted aliases.

For ensemble or forecast-like atmospheric models, `_calls` must call `set_date`
before `set_atmospheric_model`.

## Formulas In JSON

The config supports simple numeric formulas to avoid duplicating values such as
masses, areas and positions.

Examples:

```json
"mass": "= $RocketV2._dry_mass + $RocketV2._ballast"
"position": "= $add_tail.position + $add_tail.length"
"cd_s": "= $_cd * $_area"
"wind_u": "= sin(radians($_wind_heading_deg))"
```

Reference rules:

- `$Section.key` reads from the root config.
- `$_local_key` reads from the current object first.
- Formulas must evaluate to numbers.
- Only a small whitelist of numeric functions is supported.
- Cyclic references fail during config loading.

## Sensor Profiles

Sensor profiles are defined entirely by the config file under
`Sensors._profiles`. The CLI profile name must exactly match a profile key.
There is no normalization and no compatibility alias.

Example:

```json
{
  "Sensors": {
    "_default_position": 0,
    "_profiles": {
      "clean": {
        "Accelerometer": {
          "name": "Clean Accelerometer"
        },
        "Barometer": {
          "name": "Clean Barometer"
        },
        "GnssReceiver": {
          "name": "Clean GPS"
        }
      },
      "noisy": {
        "_inherits": "clean",
        "Accelerometer": {
          "name": "Noisy Accelerometer",
          "noise_density": 0.04,
          "random_walk_density": 0.004,
          "constant_bias": 0.05
        }
      },
      "very_noisy": {
        "_inherits": "noisy",
        "Accelerometer": {
          "name": "Very Noisy Accelerometer",
          "noise_density": 0.2,
          "constant_bias": 0.25
        }
      }
    }
  }
}
```

Rules:

- `clean` is required.
- `clean` starts from built-in zero-noise defaults and may override any field.
- Non-clean profiles inherit `clean` by default.
- A profile may inherit another profile with `_inherits`.
- A profile may disable inheritance with `"_inherits": null`.
- Unknown sensor types fail fast.
- `sampling_rate` is not allowed inside a sensor profile. The launcher injects
  the CLI `--sampling-rate` into every sensor so the logger, sensors,
  parachutes and airbrakes use one coherent rate.
- Sensor `_position` overrides `Sensors._default_position`.

Currently supported sensor types:

- `Accelerometer`
- `Barometer`
- `GnssReceiver`

## Calibration Before Flight

RocketPy starts the simulated launch when `Flight(...)` is instantiated. The
flight controller, however, needs a calibration phase before launch. The launcher
therefore sends a manual stationary-pad stream before creating `Flight(...)`.

Calibration behavior:

- Samples use the configured launch-site latitude, longitude, elevation,
  pressure and temperature.
- Pressure and temperature are evaluated from the configured RocketPy
  `Environment`.
- Stationary acceleration is generated explicitly.
- Accelerometer and barometer values are passed through the selected RocketPy
  sensor objects, so clean/noisy behavior is controlled by the selected sensor
  profile.
- Calibration stops early when the FC reports `READY_FOR_LAUNCH`.
- If `READY_FOR_LAUNCH` is not received before `--calibration-samples`, the run
  fails and the communication thread is stopped.

Temperature during flight is handled explicitly:

- If RocketPy provides `temperature` in the sensor callback, that value is used.
- Otherwise the launcher evaluates `env.temperature(current_altitude_asl_m)`.
- Environment lookup failures are not hidden behind ISA fallbacks.

## Timing Names

Timing names are intentionally explicit:

- `rocketpy_time_s`: raw RocketPy callback time, starting at launch.
- `calibration_sim_time_s`: synthetic time used during calibration samples.
- `hil_sim_time_s`: time sent to the flight controller and stored in captures.
- `command_sim_time_s`: time returned by the flight controller command packet.

Flight samples are offset by the calibration duration, so `hil_sim_time_s`
continues monotonically from calibration into RocketPy flight.

## TCP Communication

The simulator and FC communicate over TCP using a simple framed binary protocol.

Frame header:

```text
magic, payload_length, message_type
```

Header format:

```text
!IHH
```

Message types:

- `MSG_TYPE_SIM_INPUT = 1`
- `MSG_TYPE_FC_COMMAND = 2`
- `MSG_TYPE_SIM_RESET = 3`

The communication thread uses a one-slot mailbox. This intentionally enforces
lockstep behavior:

1. The simulator queues one payload.
2. The TCP client sends it.
3. The TCP client waits for exactly one FC command.
4. Command handlers update parachute, airbrake and FSM state.
5. The next simulator payload may be sent.

This prevents the simulator from running ahead of the FC.

## Simulator Input Packet

Every simulator sample is sent as `MSG_TYPE_SIM_INPUT`.

Payload fields:

```text
sequence_number, hil_sim_time_s, host_unix_time_s,
ax_m_s2, ay_m_s2, az_m_s2,
pressure_pa, temperature_k,
latitude_deg, longitude_deg, altitude_m
```

Payload format:

```text
<IfIffffffff
```

Packed size: 44 bytes.

`temperature_k` is part of the current simulator packet format.

## Flight-Controller Command Packet

The FC replies with `MSG_TYPE_FC_COMMAND`.

Command fields:

```text
command_sim_time_s,
open_main,
open_drogue,
airbrakes_deployment,
fsm_state
```

Payload format:

```text
<fBBfB
```

Packed size: 11 bytes.

The command updates:

- main parachute trigger
- drogue parachute trigger
- airbrakes deployment level
- reported FC FSM state

`fsm_state` mirrors the firmware `RocketState` enum. The Python side only maps
the numeric value to a readable name for logs and plots.

Known FSM values:

```text
0  INACTIVE
1  CALIBRATING
2  READY_FOR_LAUNCH
3  LAUNCH
4  ACCELERATED_FLIGHT
5  BALLISTIC_FLIGHT
6  APOGEE
7  STABILIZATION
8  DECELERATION
9  LANDING
10 RECOVERED
```

## Reset Behavior

By default the launcher performs a startup reset:

1. Connect to the FC HIL TCP server.
2. Send `MSG_TYPE_SIM_RESET`.
3. Close the current socket.
4. Wait for the FC to restart/reopen the HIL TCP server.
5. Reconnect and start calibration.

This lets an obviously bad run be interrupted, fixed, and restarted from a known
controller state.

At the end of a Python run, the launcher does not send another FC reset. It
sends only an internal `STOP_COMMUNICATION` control message to close the local
TCP thread cleanly.

## Captures

Each run is saved under `hil_captures/` as JSON.

Top-level capture content:

- `metadata`: selected rocket, config path, rates, reset/calibration settings,
  Environment summary, Flight summary, selected sensor profile and sensor args.
- `hil_log`: every simulator packet actually queued for the FC.
- `hil_events`: commands and FSM transitions reported by the FC.

Current `hil_log` field names:

```text
seq
sim_time_s
x, y, z
vx, vy, vz
e0, e1, e2, e3
omega1, omega2, omega3
accel_x_m_s2, accel_y_m_s2, accel_z_m_s2
pressure_pa, temperature_k
latitude_deg, longitude_deg, altitude_m
```

Current `hil_events` keys:

```text
open_drogue
open_main
airbrakes
fsm_state
```

The plotter requires the current capture schema. Historical aliases such as `t`,
`ax`, `p`, `lat` and `alt` are intentionally not supported.

## Plots And Summary

After a run, `hil_capture.py` prints a summary:

- samples sent
- first/last sequence number
- start/end simulation time
- sample period and effective rate
- maximum altitude and apogee time
- maximum acceleration norm
- parachute events
- airbrakes changes
- FSM transitions

The generated plots show:

- 3D trajectory with start, end, apogee and event markers
- altitude and barometer pressure over time
- GPS ground track
- acceleration components and acceleration norm
- temperature over time
- FSM state timeline
- airbrakes deployment level

To replot a saved capture:

```bash
python hil_capture.py hil_captures/<capture>.json
python hil_capture.py hil_captures/<capture>.json --no-show
```

## Mock Manny Server

`mock_manny_fc_server.py` emulates the FC HIL TCP server when Manny is not
available.

It is intentionally self-contained:

- no argparse
- one visible `MockConfig`
- protocol constants copied locally
- deterministic FSM behavior for tests

Mock behavior:

- starts in calibration
- reports `READY_FOR_LAUNCH` after `calibration_samples_before_ready`
- computes pad altitude/pressure references from the median of calibration
  samples
- arms launch detection only after `READY_FOR_LAUNCH` has been reported
- detects launch from total accelerometer magnitude
- median-filters altitude and pressure before apogee detection
- requires consecutive filtered apogee confirmations before reporting `APOGEE`
- detects apogee only after launch and accelerated flight
- treats `LANDING` as terminal until reset
- commands airbrakes only during ascent and only inside a configured AGL band
- supports `single_main_at_apogee` and `drogue_then_main` recovery modes
- resets state and closes the connection when it receives `MSG_TYPE_SIM_RESET`

Typical local run:

```bash
python mock_manny_fc_server.py
HIL_FC_HOST=127.0.0.1 python hil_rocketpy.py --rocket fred --sensor-profile clean --calibration-samples 300
```

## Fail-Fast Policy

The current code should not hide invalid inputs with undocumented fallbacks.

Examples:

- Missing mandatory config sections fail immediately.
- Unknown sensor profiles fail immediately.
- Sensor profile names must match exactly.
- Unknown sensor types fail immediately.
- Environment method calls must use `_calls`.
- `set_atmospheric_model` must not run before `set_date` when both are needed.
- Missing RocketPy callback fields fail with the missing field name.
- Environment pressure/temperature evaluation failures propagate as errors.
- Calibration timeout is fatal.
- Protocol size/type mismatches are protocol errors.

This keeps failed HIL runs diagnosable instead of producing plausible but invalid
captures.
