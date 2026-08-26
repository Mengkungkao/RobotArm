# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Python control API for the 6-DOF robot arm (simulation and real hardware).

    from robot_arm_control import RobotArm

`RobotArm` and its companions are imported lazily, so the pure-maths helpers
in `robot_arm_control.transforms` stay usable in contexts where rclpy is not
importable - offline unit tests, tooling, documentation builds.
"""

from .transforms import (euler_from_quaternion, normalize_quaternion,
                         quaternion_from_euler, quaternion_multiply)

__all__ = [
    'RobotArm',
    'MoveResult',
    'JointStates',
    'DEFAULT_JOINT_NAMES',
    'quaternion_from_euler',
    'euler_from_quaternion',
    'normalize_quaternion',
    'quaternion_multiply',
]

__version__ = '1.0.0'

_LAZY = {'RobotArm', 'MoveResult', 'JointStates', 'DEFAULT_JOINT_NAMES'}


def __getattr__(name):
    """Import the ROS-dependent API only when it is actually used (PEP 562)."""
    if name in _LAZY:
        from . import robot_arm_api
        return getattr(robot_arm_api, name)
    raise AttributeError(f'module {__name__!r} has no attribute {name!r}')


def __dir__():
    return sorted(set(globals()) | _LAZY)
