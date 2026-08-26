#!/usr/bin/env python3
# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Minimal application built on the Python API.

Runs unchanged against Gazebo and against the physical robot:

    ros2 launch robot_arm_bringup sim.launch.py
    ros2 run robot_arm_control python_api_demo.py

    ros2 launch robot_arm_bringup real.launch.py
    ros2 run robot_arm_control python_api_demo.py
"""

import math
import sys

from robot_arm_control import RobotArm


def main() -> int:
    with RobotArm() as robot:
        if not robot.wait_for_state(timeout=15.0):
            print('No /joint_states - is the robot running?', file=sys.stderr)
            return 1

        print('enable():', robot.enable().message)

        # 1. joint space
        print('\nmove_joints([0.0, 0.5, -0.8, 0.0, 0.5, 0.0])')
        print('  ->', robot.move_joints([0.0, 0.5, -0.8, 0.0, 0.5, 0.0]).message)

        # 2. Cartesian space
        print('\nmove_to_pose(x=0.35, y=0.10, z=0.40, pitch=1.57)')
        print('  ->', robot.move_to_pose(
            x=0.35, y=0.10, z=0.40, roll=0.0, pitch=math.pi / 2, yaw=0.0).message)

        # 3. read the state back
        states = robot.get_joint_states()
        print('\njoint states:')
        for name, position in states.as_dict().items():
            print(f'  {name} = {position: .4f} rad')

        pose = robot.get_current_pose_rpy()
        if pose is not None:
            print(
                f"\ntool0 at x={pose['x']:.3f} y={pose['y']:.3f} z={pose['z']:.3f} "
                f"rpy=({pose['roll']:.3f}, {pose['pitch']:.3f}, {pose['yaw']:.3f})")

        print('\nstop():', robot.stop().message)
    return 0


if __name__ == '__main__':
    sys.exit(main())
