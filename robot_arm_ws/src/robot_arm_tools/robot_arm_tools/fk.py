# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Forward kinematics: joint angles -> tool0 pose.

    # pose of the arm right now
    ros2 run robot_arm_tools fk

    # pose the arm would have at these joint angles
    ros2 run robot_arm_tools fk -- --j1 0 --j2 0.5 --j3 -0.8 --j4 0 --j5 0.5 --j6 0
"""

import argparse
import sys

from .cli_common import (EXIT_FAILURE, EXIT_OK, JOINT_NAMES, add_common_arguments,
                         add_joint_arguments, angle_unit, collect_joint_targets,
                         format_angle, make_robot, run)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='fk',
        description='Compute the end-effector pose for a set of joint angles.')
    add_joint_arguments(parser)
    add_common_arguments(parser)
    parser.add_argument(
        '--link', default=None,
        help='link to compute the pose of (default: the end-effector frame)')
    return parser


def main(argv=None) -> int:
    arguments = build_parser().parse_args(argv if argv is not None else sys.argv[1:])

    robot = make_robot(arguments, node_name='fk_cli')
    try:
        if not robot.wait_for_state(timeout=arguments.timeout):
            print('no /joint_states: is the robot running?', file=sys.stderr)
            return EXIT_FAILURE

        current = robot.get_joint_positions()
        # With no --jN given this is simply the current state.
        joints = collect_joint_targets(arguments, current)

        unit = angle_unit(arguments.degrees)
        print(f'joint angles ({unit}):')
        for name, value in zip(JOINT_NAMES, joints):
            print(f'  {name}: {format_angle(value, arguments.degrees)}')

        pose = robot.forward_kinematics(joints, link=arguments.link)
        if pose is None:
            print('forward kinematics failed (is move_group running?)', file=sys.stderr)
            return EXIT_FAILURE

        from robot_arm_control.transforms import euler_from_quaternion
        orientation = pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w)

        link = arguments.link or robot.end_effector_frame
        print(f'\n{link} in {pose.header.frame_id}:')
        print(f'  position    x={pose.pose.position.x: .4f}  '
              f'y={pose.pose.position.y: .4f}  z={pose.pose.position.z: .4f}   [m]')
        print(f'  orientation r={roll: .4f}  p={pitch: .4f}  y={yaw: .4f}   [rad]')
        print(f'  quaternion  x={orientation.x: .4f}  y={orientation.y: .4f}  '
              f'z={orientation.z: .4f}  w={orientation.w: .4f}')
        return EXIT_OK
    finally:
        robot.shutdown()


def cli() -> int:
    return run(main)


if __name__ == '__main__':
    sys.exit(cli())
