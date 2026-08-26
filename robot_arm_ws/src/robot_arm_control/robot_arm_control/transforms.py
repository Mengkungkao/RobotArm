# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Small, dependency-free rotation helpers.

The arm's user-facing API speaks roll/pitch/yaw because that is what an
operator types on a command line, while ROS messages carry quaternions.  These
functions convert between the two without pulling in a matrix library, so the
Python API works on a bare ROS installation.

Convention: intrinsic X-Y-Z (roll about X, then pitch about Y, then yaw about
Z), which is the same convention used by tf2 and by RViz.
"""

import math
from typing import Tuple

__all__ = [
    'quaternion_from_euler',
    'euler_from_quaternion',
    'normalize_quaternion',
    'quaternion_multiply',
]


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convert roll/pitch/yaw [rad] to a quaternion (x, y, z, w)."""
    half_roll, half_pitch, half_yaw = roll * 0.5, pitch * 0.5, yaw * 0.5
    sin_r, cos_r = math.sin(half_roll), math.cos(half_roll)
    sin_p, cos_p = math.sin(half_pitch), math.cos(half_pitch)
    sin_y, cos_y = math.sin(half_yaw), math.cos(half_yaw)

    return (
        sin_r * cos_p * cos_y - cos_r * sin_p * sin_y,   # x
        cos_r * sin_p * cos_y + sin_r * cos_p * sin_y,   # y
        cos_r * cos_p * sin_y - sin_r * sin_p * cos_y,   # z
        cos_r * cos_p * cos_y + sin_r * sin_p * sin_y,   # w
    )


def euler_from_quaternion(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """Convert a quaternion (x, y, z, w) to roll/pitch/yaw [rad]."""
    sin_r_cos_p = 2.0 * (w * x + y * z)
    cos_r_cos_p = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_r_cos_p, cos_r_cos_p)

    sin_p = 2.0 * (w * y - z * x)
    # Clamp: numerical noise can push this just outside [-1, 1] at gimbal lock.
    sin_p = max(-1.0, min(1.0, sin_p))
    pitch = math.asin(sin_p)

    sin_y_cos_p = 2.0 * (w * z + x * y)
    cos_y_cos_p = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_y_cos_p, cos_y_cos_p)

    return (roll, pitch, yaw)


def normalize_quaternion(x: float, y: float, z: float, w: float) -> Tuple[float, float, float, float]:
    """Return the unit quaternion; an all-zero input becomes the identity."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def quaternion_multiply(first, second):
    """Hamilton product of two (x, y, z, w) quaternions."""
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )
