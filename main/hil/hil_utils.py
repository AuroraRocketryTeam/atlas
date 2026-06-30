"""Shared utilities for RocketPy HIL frame and sensor transformations.

Reference frames used by these helpers:

    I       RocketPy inertial frame, +X east, +Y north, +Z up
    B       RocketPy body frame, body +Z points towards the nose
    S_clean Ideal orthogonal accelerometer sensor frame
    S_out   Accelerometer payload frame after cross-axis output mixing

Typical chains:

    calibration: I -> B -> S_clean -> S_out
    replay:      S_out -> S_clean -> B -> I

Important convention
--------------------
The HIL JSON configuration stores inertial-sensor Euler orientations in radians.
RocketPy's public documentation describes these angles as radians, but some
RocketPy versions internally call ``np.deg2rad(orientation)`` when a 3-element
Euler vector is passed to the sensor constructor. To avoid any ambiguity, HIL
converts 3-angle radian orientations to an explicit 3x3 rotation matrix before
creating RocketPy inertial sensors. Matrix orientations are forwarded unchanged.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np


# ----------------------------------------------------------------------
# NUMERIC CONFIG HELPERS
# ----------------------------------------------------------------------

def numeric_array(value: Any) -> np.ndarray:
    """Convert resolved config/capture numeric values to a numpy array."""
    try:
        return np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Could not convert value to a numeric array: {value!r}") from exc


def numeric_scalar(value: Any) -> float:
    """Convert one config/capture numeric value to ``float``."""
    array = numeric_array(value)
    if array.shape != ():
        raise TypeError(f"Expected a scalar numeric value, got shape {array.shape}")
    return float(array)


# ----------------------------------------------------------------------
# ROCKETPY ATTITUDE / ROTATION HELPERS
# ----------------------------------------------------------------------

def rocketpy_body_to_inertial_matrix(
    e0: float,
    e1: float,
    e2: float,
    e3: float,
) -> np.ndarray:
    """Return RocketPy's attitude matrix: ``B -> I``.

    RocketPy stores attitude as the quaternion ``(e0, e1, e2, e3)``, where
    ``e0`` is the scalar term. The returned matrix has a useful geometric
    interpretation: each column is one body-axis unit vector written in inertial
    coordinates. For example, column 2 is body ``+Z`` / nose direction in
    RocketPy's world frame.
    """
    quaternion = np.asarray([e0, e1, e2, e3], dtype=float)
    quaternion_norm = np.linalg.norm(quaternion)

    if not np.isfinite(quaternion_norm) or quaternion_norm == 0:
        return np.eye(3)

    # Normalize first so small numeric drift in saved states cannot introduce a
    # scale factor into what should be a pure rotation matrix.
    e0, e1, e2, e3 = quaternion / quaternion_norm

    return np.asarray(
        [
            [
                1 - 2 * (e2**2 + e3**2),
                2 * (e1 * e2 - e0 * e3),
                2 * (e1 * e3 + e0 * e2),
            ],
            [
                2 * (e1 * e2 + e0 * e3),
                1 - 2 * (e1**2 + e3**2),
                2 * (e2 * e3 - e0 * e1),
            ],
            [
                2 * (e1 * e3 - e0 * e2),
                2 * (e2 * e3 + e0 * e1),
                1 - 2 * (e1**2 + e2**2),
            ],
        ],
        dtype=float,
    )


def rotation_matrix_to_rocketpy_quaternion(
    body_to_inertial: Any,
) -> tuple[float, float, float, float]:
    """Convert a ``B -> I`` matrix to RocketPy ``(e0, e1, e2, e3)``.

    The branch structure is the standard numerically stable matrix-to-quaternion
    conversion: use the trace when it is comfortably positive, otherwise compute
    the component associated with the largest diagonal entry first. That avoids
    dividing by a tiny number when the attitude is close to a 180 degree turn.
    """
    matrix = np.asarray(body_to_inertial, dtype=float)
    if matrix.shape != (3, 3):
        raise ValueError("body_to_inertial must be a 3x3 matrix")

    trace = float(np.trace(matrix))

    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        e0 = 0.25 * scale
        e1 = (matrix[2, 1] - matrix[1, 2]) / scale
        e2 = (matrix[0, 2] - matrix[2, 0]) / scale
        e3 = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        e0 = (matrix[2, 1] - matrix[1, 2]) / scale
        e1 = 0.25 * scale
        e2 = (matrix[0, 1] + matrix[1, 0]) / scale
        e3 = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        e0 = (matrix[0, 2] - matrix[2, 0]) / scale
        e1 = (matrix[0, 1] + matrix[1, 0]) / scale
        e2 = 0.25 * scale
        e3 = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        e0 = (matrix[1, 0] - matrix[0, 1]) / scale
        e1 = (matrix[0, 2] + matrix[2, 0]) / scale
        e2 = (matrix[1, 2] + matrix[2, 1]) / scale
        e3 = 0.25 * scale

    quaternion = np.asarray([e0, e1, e2, e3], dtype=float)
    quaternion_norm = np.linalg.norm(quaternion)
    if not np.isfinite(quaternion_norm) or quaternion_norm == 0:
        raise ValueError("Cannot convert invalid rotation matrix to quaternion")

    quaternion /= quaternion_norm

    # q and -q represent the same attitude. Keep logs stable/readable by choosing
    # the sign with a non-negative scalar component.
    if quaternion[0] < 0.0:
        quaternion = -quaternion

    return tuple(float(component) for component in quaternion)


def rail_body_to_inertial_matrix(
    inclination_deg: float,
    heading_deg: float,
) -> np.ndarray:
    """Build the initial rail attitude matrix: ``B -> I``.

    RocketPy heading is clockwise from north. Inclination is measured from the
    horizontal plane, so 90 degrees is vertical.

    The matrix columns are the body axes expressed in inertial coordinates. We
    start from body ``+Z`` because RocketPy's body ``+Z`` points toward the nose,
    and on the pad the nose points along the launch rail.
    """
    inclination = math.radians(float(inclination_deg))
    heading = math.radians(float(heading_deg))

    # Nose/rail direction in inertial axes:
    #   +X east  gets sin(heading)
    #   +Y north gets cos(heading)
    #   +Z up    gets sin(inclination)
    body_z_inertial = np.asarray(
        [
            math.cos(inclination) * math.sin(heading),
            math.cos(inclination) * math.cos(heading),
            math.sin(inclination),
        ],
        dtype=float,
    )

    # Choose body +X as a horizontal vector perpendicular to the heading. Body
    # +Y is then whatever completes a right-handed orthonormal frame.
    body_x_inertial = np.asarray(
        [
            math.cos(heading),
            -math.sin(heading),
            0.0,
        ],
        dtype=float,
    )

    body_x_inertial /= np.linalg.norm(body_x_inertial)
    body_z_inertial /= np.linalg.norm(body_z_inertial)
    body_y_inertial = np.cross(body_z_inertial, body_x_inertial)
    body_y_inertial /= np.linalg.norm(body_y_inertial)

    # Columns are body +X/+Y/+Z expressed in inertial coordinates.
    return np.column_stack((body_x_inertial, body_y_inertial, body_z_inertial))


def rocketpy_euler313_matrix(orientation_rad: Any) -> np.ndarray:
    """Return RocketPy 3-1-3 orientation matrix: ``S_clean -> B``.

    The HIL config stores these angles in radians. This function must never call
    ``deg2rad`` on them.

    RocketPy's inertial sensor orientation uses an intrinsic 3-1-3 rotation:
    rotate about axis 3, then the new axis 1, then the new axis 3 again. The
    intermediate quaternion below is just the compact closed-form version of
    that sequence. Converting it through ``rocketpy_body_to_inertial_matrix`` is
    convenient because the same quaternion convention is used by RocketPy state.
    """
    angles = numeric_array(orientation_rad)
    if angles.shape != (3,):
        raise ValueError("RocketPy Euler 3-1-3 orientation must contain 3 angles")

    roll, pitch, roll2 = (float(v) for v in angles)

    e0 = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(roll2 / 2) - (
        math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(roll2 / 2)
    )
    e1 = math.cos(roll / 2) * math.cos(roll2 / 2) * math.sin(pitch / 2) + (
        math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(roll2 / 2)
    )
    e2 = math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(roll2 / 2) - (
        math.sin(roll / 2) * math.cos(roll2 / 2) * math.sin(pitch / 2)
    )
    e3 = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(roll2 / 2) + (
        math.cos(pitch / 2) * math.cos(roll2 / 2) * math.sin(roll / 2)
    )

    return rocketpy_body_to_inertial_matrix(e0, e1, e2, e3)


def matrix_like_to_numpy_3x3(matrix_like: Any) -> np.ndarray:
    """Convert RocketPy Matrix/list-like objects to a 3x3 numpy array.

    RocketPy matrix objects are not guaranteed to expose the same public storage
    attribute across versions, so this helper accepts common matrix containers
    as long as they clearly represent a 3x3 numeric matrix.
    """
    for attribute_name in ("components", "matrix", "data", "_components"):
        if hasattr(matrix_like, attribute_name):
            array = np.asarray(getattr(matrix_like, attribute_name), dtype=float)
            if array.shape == (3, 3):
                return array

    try:
        array = np.asarray(matrix_like, dtype=float)
        if array.shape == (3, 3):
            return array
    except (TypeError, ValueError):
        pass

    try:
        return np.asarray(
            [[float(matrix_like[i][j]) for j in range(3)] for i in range(3)],
            dtype=float,
        )
    except Exception:
        pass

    try:
        return np.asarray(
            [[float(matrix_like[i, j]) for j in range(3)] for i in range(3)],
            dtype=float,
        )
    except Exception as exc:
        raise TypeError("Could not convert matrix-like object to a 3x3 array") from exc


# ----------------------------------------------------------------------
# ACCELEROMETER ORIENTATION / CROSS-AXIS HELPERS
# ----------------------------------------------------------------------

def accelerometer_sensor_to_body_from_orientation(orientation: Any) -> np.ndarray:
    """Return accelerometer mounting matrix from config: ``S_clean -> B``.

    Supported config shapes:
      - 3 Euler angles in radians
      - explicit 3x3 matrix, already ``S_clean -> B``

    ``S_clean`` is the ideal orthogonal sensor frame before output cross-axis
    mixing. This mounting matrix describes hardware geometry only: how the
    sensor axes are bolted into the rocket body frame.
    """
    orientation_array = numeric_array(orientation)

    if orientation_array.shape == (3, 3):
        return orientation_array
    if orientation_array.shape == (3,):
        return rocketpy_euler313_matrix(orientation_array)

    raise ValueError(
        "Accelerometer orientation must be 3 Euler angles in radians or a 3x3 matrix"
    )


def rocketpy_constructor_orientation_from_config(orientation: Any) -> Any:
    """Return a RocketPy-safe orientation value from HIL config orientation.

    HIL 3-angle orientations are radians. To prevent RocketPy-version ambiguity,
    convert them to an explicit ``S_clean -> B`` matrix before passing them to the
    RocketPy sensor constructor. Explicit matrix orientations pass through.

    In short: configs stay human-friendly, but RocketPy receives the unambiguous
    matrix form.
    """
    orientation_array = numeric_array(orientation)

    if orientation_array.shape == (3, 3):
        return orientation_array.tolist()
    if orientation_array.shape == (3,):
        return rocketpy_euler313_matrix(orientation_array).tolist()

    raise ValueError(
        "Sensor orientation must be 3 Euler angles in radians or a 3x3 matrix"
    )


def accelerometer_sensor_to_body_from_metadata(
    metadata: dict[str, Any] | None,
) -> np.ndarray:
    """Return accelerometer mounting from capture metadata: ``S_clean -> B``.

    Captures store the effective matrix that was actually passed into RocketPy.
    Replay uses that matrix instead of reinterpreting config angles, so the
    saved packet stream and the visualization agree about the sensor mounting.
    """
    if not metadata:
        raise ValueError("Capture metadata is required for accelerometer orientation")

    sensor_metadata = metadata["sensors"]["Accelerometer"]

    effective = sensor_metadata.get("effective_orientation_matrix_sensor_to_body")
    if effective is None:
        raise KeyError(
            "Missing Accelerometer.effective_orientation_matrix_sensor_to_body "
            "in capture metadata"
        )

    return matrix_like_to_numpy_3x3(effective)


def accelerometer_cross_axis_matrix(cross_axis_sensitivity: Any = 0.0) -> np.ndarray:
    """Return accelerometer output-space mixing matrix: ``S_clean -> S_out``.

    Cross-axis sensitivity is not a rotation. It models one sensor channel
    leaking into the others after the clean geometric projection has already
    happened. A sensitivity of ``c`` percent means each off-diagonal entry is
    ``0.01 * c`` while the diagonal stays 1.
    """
    sensitivity = numeric_scalar(cross_axis_sensitivity)
    c = 0.01 * sensitivity

    # Rows are output channels. Columns are clean orthogonal sensor components.
    # With zero cross-axis sensitivity this is the identity matrix.
    cross_axis_matrix = np.full((3, 3), c, dtype=float)
    np.fill_diagonal(cross_axis_matrix, 1.0)
    return cross_axis_matrix


def accelerometer_cross_axis_matrix_from_metadata(
    metadata: dict[str, Any] | None,
) -> np.ndarray:
    """Return capture metadata cross-axis mixing: ``S_clean -> S_out``."""
    if not metadata:
        raise ValueError("Capture metadata is required for accelerometer cross-axis")

    sensor_args = metadata["sensors"]["Accelerometer"]["args"]
    return accelerometer_cross_axis_matrix(
        sensor_args.get("cross_axis_sensitivity", 0.0)
    )


def accelerometer_body_to_clean_sensor_matrix(
    sensor_to_body: Any,
) -> np.ndarray:
    """Return geometric projection matrix: ``B -> S_clean``.

    The mounting matrix is stored as ``S_clean -> B`` because that is what
    RocketPy expects. To project a body-frame vector onto sensor axes, invert the
    mounting matrix. For a pure rotation this is equivalent to transpose, but
    using ``inv`` keeps the intent explicit for any matrix-like input.
    """
    return np.linalg.inv(np.asarray(sensor_to_body, dtype=float))


def accelerometer_output_to_clean_sensor_matrix(
    cross_axis_matrix: Any,
) -> np.ndarray:
    """Undo output-space mixing: ``S_out -> S_clean``.

    Replay wants physical orthogonal sensor components. The saved payload is the
    mixed output vector, so the first replay step is to invert the cross-axis
    matrix before doing any geometric rotations.
    """
    return np.linalg.inv(np.asarray(cross_axis_matrix, dtype=float))


def accelerometer_output_to_clean_sensor_matrix_from_metadata(
    metadata: dict[str, Any] | None,
) -> np.ndarray:
    """Undo capture metadata output mixing: ``S_out -> S_clean``."""
    return accelerometer_output_to_clean_sensor_matrix(
        accelerometer_cross_axis_matrix_from_metadata(metadata)
    )


def accelerometer_body_to_sensor_output_matrix(
    sensor_to_body: Any,
    cross_axis_matrix: Any,
) -> np.ndarray:
    """Return calibration payload transform: ``B -> S_clean -> S_out``.

    This is the forward path used when generating synthetic calibration samples:
    first project the physical body-frame specific force onto clean sensor axes,
    then apply output cross-axis mixing to match what the FC receives.
    """
    return (
        np.asarray(cross_axis_matrix, dtype=float)
        @ accelerometer_body_to_clean_sensor_matrix(sensor_to_body)
    )


def accelerometer_output_to_body_matrix(
    sensor_to_body: Any,
    cross_axis_matrix: Any,
) -> np.ndarray:
    """Return replay reconstruction transform: ``S_out -> S_clean -> B``.

    This is the inverse of the calibration payload path, but written in the
    order replay actually performs it: unmix output channels, then rotate from
    clean sensor axes back into the rocket body frame.
    """
    return (
        np.asarray(sensor_to_body, dtype=float)
        @ accelerometer_output_to_clean_sensor_matrix(cross_axis_matrix)
    )


def accelerometer_output_to_body_matrix_from_metadata(
    metadata: dict[str, Any] | None,
) -> np.ndarray:
    """Return capture metadata reconstruction transform: ``S_out -> S_clean -> B``."""
    return accelerometer_output_to_body_matrix(
        accelerometer_sensor_to_body_from_metadata(metadata),
        accelerometer_cross_axis_matrix_from_metadata(metadata),
    )
