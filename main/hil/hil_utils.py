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
    """Return RocketPy's attitude matrix: ``B -> I``."""
    quaternion = np.asarray([e0, e1, e2, e3], dtype=float)
    quaternion_norm = np.linalg.norm(quaternion)

    if not np.isfinite(quaternion_norm) or quaternion_norm == 0:
        return np.eye(3)

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
    """Convert a ``B -> I`` matrix to RocketPy ``(e0, e1, e2, e3)``."""
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

    # q and -q represent the same attitude. Keep logs stable/readable.
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
    """
    inclination = math.radians(float(inclination_deg))
    heading = math.radians(float(heading_deg))

    body_z_inertial = np.asarray(
        [
            math.cos(inclination) * math.sin(heading),
            math.cos(inclination) * math.cos(heading),
            math.sin(inclination),
        ],
        dtype=float,
    )

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
    """Convert RocketPy Matrix/list-like objects to a 3x3 numpy array."""
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
    """Return accelerometer mounting from capture metadata: ``S_clean -> B``."""
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
    """Return accelerometer output-space mixing matrix: ``S_clean -> S_out``."""
    sensitivity = numeric_scalar(cross_axis_sensitivity)
    c = 0.01 * sensitivity

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
    """Return geometric projection matrix: ``B -> S_clean``."""
    return np.linalg.inv(np.asarray(sensor_to_body, dtype=float))


def accelerometer_output_to_clean_sensor_matrix(
    cross_axis_matrix: Any,
) -> np.ndarray:
    """Undo output-space mixing: ``S_out -> S_clean``."""
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
    """Return calibration payload transform: ``B -> S_clean -> S_out``."""
    return (
        np.asarray(cross_axis_matrix, dtype=float)
        @ accelerometer_body_to_clean_sensor_matrix(sensor_to_body)
    )


def accelerometer_output_to_body_matrix(
    sensor_to_body: Any,
    cross_axis_matrix: Any,
) -> np.ndarray:
    """Return replay reconstruction transform: ``S_out -> S_clean -> B``."""
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
