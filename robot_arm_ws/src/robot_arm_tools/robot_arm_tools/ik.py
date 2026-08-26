# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Inverse kinematics: tool0 pose -> joint angles.

    ros2 run robot_arm_tools ik -- --x 0.35 --y 0.10 --z 0.40 --pitch 1.57

Nothing moves: this only asks MoveIt whether the pose is reachable and what
the joint angles would be.  Add --execute to move there afterwards.
"""

import argparse
import sys

from .cli_common import (EXIT_FAILURE, EXIT_OK, JOINT_NAMES, add_common_arguments,
                         add_motion_arguments, angle_unit, format_angle, make_robot, run)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='ik',
        description='Compute joint angles that put tool0 at a given pose.')
    target = parser.add_argument_group('pose target')
    target.add_argument('--x', type=float, required=True, help='target X [m]')
    target.add_argument('--y', type=float, required=True, help='target Y [m]')
    target.add_argument('--z', type=float, required=True, help='target Z [m]')
    target.add_argument('--roll', type=float, default=0.0, help='roll [rad]')
    target.add_argument('--pitch', type=float, default=0.0, help='pitch [rad]')
    target.add_argument('--yaw', type=float, default=0.0, help='yaw [rad]')
    target.add_argument(
        '--allow-collisions', action='store_true',
        help='accept solutions that are in collision')
    parser.add_argument(
        '--degrees', action='store_true', help='print joint angles in degrees')
    parser.add_argument(
        '--execute', action='store_true',
        help='move to the solution once it has been found')
    add_motion_arguments(parser)
    add_common_arguments(parser)
    return parser


def main(argv=None) -> int:
    arguments = build_parser().parse_args(argv if argv is not None else sys.argv[1:])

    robot = make_robot(arguments, node_name='ik_cli')
    try:
        if not robot.wait_for_state(timeout=arguments.timeout):
            print('no /joint_states: is the robot running?', file=sys.stderr)
            return EXIT_FAILURE

        print(
            f'pose: x={arguments.x:.3f} y={arguments.y:.3f} z={arguments.z:.3f} '
            f'rpy=({arguments.roll:.3f}, {arguments.pitch:.3f}, {arguments.yaw:.3f})')

        solution = robot.inverse_kinematics(
            x=arguments.x, y=arguments.y, z=arguments.z,
            roll=arguments.roll, pitch=arguments.pitch, yaw=arguments.yaw,
            avoid_collisions=not arguments.allow_collisions)

        if solution is None:
            # An unreachable pose is a normal answer, not a crash.
            print('no IK solution: the pose is out of reach, or in collision')
            return EXIT_FAILURE

        unit = angle_unit(arguments.degrees)
        print(f'\nsolution ({unit}):')
        for name, value in zip(JOINT_NAMES, solution):
            print(f'  {name}: {format_angle(value, arguments.degrees)}')

        if arguments.execute:
            result = robot.move_joints(
                solution,
                wait=not arguments.no_wait,
                velocity_scaling=arguments.velocity_scaling,
                acceleration_scaling=arguments.acceleration_scaling)
            print(f'\n{result.message}')
            return EXIT_OK if result.success else EXIT_FAILURE

        return EXIT_OK
    finally:
        robot.shutdown()


def cli() -> int:
    return run(main)


if __name__ == '__main__':
    sys.exit(cli())
