# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Move the arm in joint space.

    ros2 run robot_arm_tools move_joint -- \
        --j1 0 --j2 0.5 --j3 -0.8 --j4 0 --j5 0.5 --j6 0

    # jog a single joint, in degrees, leaving the others where they are
    ros2 run robot_arm_tools move_joint -- --j3 -45 --degrees
"""

import argparse
import sys

from .cli_common import (add_common_arguments, add_joint_arguments, add_motion_arguments,
                         angle_unit, collect_joint_targets, EXIT_FAILURE, EXIT_OK,
                         format_angle, JOINT_NAMES, make_robot, run)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='move_joint',
        description='Plan and execute a joint-space motion (simulation or real robot).')
    add_joint_arguments(parser)
    add_motion_arguments(parser)
    add_common_arguments(parser)
    parser.add_argument(
        '--no-moveit', action='store_true',
        help='bypass MoveIt and send the target straight to the trajectory '
             'controller (no collision checking)')
    parser.add_argument(
        '--duration', type=float, default=5.0,
        help='motion duration in seconds when --no-moveit is used (default: 5)')
    return parser


def main(argv=None) -> int:
    arguments = build_parser().parse_args(argv if argv is not None else sys.argv[1:])

    robot = make_robot(arguments, use_moveit=not arguments.no_moveit, node_name='move_joint_cli')
    try:
        if not robot.wait_for_state(timeout=arguments.timeout):
            print('no /joint_states: is the robot running?', file=sys.stderr)
            return EXIT_FAILURE

        targets = collect_joint_targets(arguments, robot.get_joint_positions())

        unit = angle_unit(arguments.degrees)
        print(f'target ({unit}):')
        for name, value in zip(JOINT_NAMES, targets):
            print(f'  {name}: {format_angle(value, arguments.degrees)}')

        result = robot.move_joints(
            targets,
            wait=not arguments.no_wait,
            velocity_scaling=arguments.velocity_scaling,
            acceleration_scaling=arguments.acceleration_scaling,
            duration=arguments.duration,
        )
        print(result.message)

        if result.success and not arguments.no_wait:
            print(f'\nreached ({unit}):')
            for name, value in zip(JOINT_NAMES, robot.get_joint_positions()):
                print(f'  {name}: {format_angle(value, arguments.degrees)}')

        return EXIT_OK if result.success else EXIT_FAILURE
    finally:
        robot.shutdown()


def cli() -> int:
    return run(main)


if __name__ == '__main__':
    sys.exit(cli())
