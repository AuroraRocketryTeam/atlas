# HIL reference frames

This note documents the frame convention used by `hil_utils.py`, `hil_rocketpy.py`, and `hil_capture.py`.

## Frames

| Symbol | Meaning | Axes |
|---|---|---|
| `I` | RocketPy inertial frame | `+X` east, `+Y` north, `+Z` up |
| `B` | RocketPy body frame | `+Z` points toward the nose |
| `S_clean` | Ideal accelerometer sensor frame | orthogonal sensor axes before cross-axis mixing |
| `S_out` | Accelerometer payload frame | values after cross-axis mixing, exactly what is sent/logged |

## Sensor orientation convention

HIL config files store accelerometer Euler orientation as radians:

```text
orientation = [roll, pitch, roll2]  # radians, intrinsic 3-1-3
```

To remove ambiguity with RocketPy versions that internally apply `deg2rad()` to 3-angle sensor constructor values, HIL converts every 3-angle radian orientation to an explicit `3x3` matrix before constructing the RocketPy accelerometer:

```text
config radians -> S_clean->B matrix -> RocketPy constructor
```

The capture metadata keeps the original radian config and also stores:

```text
effective_orientation_matrix_sensor_to_body
```

Replay prefers this effective matrix when it is present.

## Calibration path

During pre-flight calibration the rocket is stationary on the rail. Gravity points down, but the accelerometer measures specific force upward:

```text
I(+Z support force) -> B -> S_clean -> S_out
```

So `hil_rocketpy.py` builds a synthetic stationary acceleration in inertial coordinates, rotates it into the rocket body frame, projects it into the mounted accelerometer frame, then applies cross-axis output mixing before sending the payload to the FC.

## Replay path

The 3D replay reconstructs the physical vector from the saved payload:

```text
S_out -> S_clean -> B -> I
```

So `hil_capture.py` first removes cross-axis mixing, then rotates the clean sensor components into the body frame, then applies the RocketPy attitude quaternion to draw the vector in inertial coordinates.

For old captures generated before the matrix conversion fix, `hil_capture.py` can detect the old rad-as-deg interpretation during the first stationary sample and use it only as a replay compatibility fallback.

## Important sign convention

For a stationary rocket, the reconstructed accelerometer specific-force vector should point upward in inertial coordinates:

```text
accel_I.z ~= +1 g
```

If it points downward, the issue is usually one of these transforms:

```text
sensor->body
body->inertial
S_out->S_clean
```

## Files touched in this consolidation

- `hil_utils.py`: shared frame/rotation/sensor-transform helpers.
- `hil_rocketpy.py`: uses `hil_utils.py` for calibration attitude, quaternion, mounting, and cross-axis transforms.
- `hil_capture.py`: uses `hil_utils.py` for replay attitude, mounting, and de-mixing transforms.
