# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Move the end effector to a Cartesian pose.

    ros2 run robot_arm_tools move_pose -- \
        --x 0.35 --y 0.10 --z 0.40 --roll 0 --pitch 1.57 --yaw 0

    # straight-line motion instead of a free-space plan
    ros2 run robot_arm_tools move_pose -- --x 0.35 --y 0.0 --z 0.30 --linear
"""

import argparse
import sys

from .cli_common import (EXIT_FAILURE, EXIT_OK, add_common_arguments, add_motion_arguments,
                         make_robot, run)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='move_pose',
        description='Plan and execute a Cartesian motion of tool0.')
    target = parser.add_argument_group('pose target')
    target.add_argument('--x', type=float, required=True, help='target X [m]')
    target.add_argument('--y', type=float, required=True, help='target Y [m]')
    target.add_argument('--z', type=float, required=True, help='target Z [m]')
    target.add_argument('--roll', type=float, default=0.0, help='roll [rad] (default: 0)')
    target.add_argument('--pitch', type=float, default=0.0, help='pitch [rad] (default: 0)')
    target.add_argument('--yaw', type=float, default=0.0, help='yaw [rad] (default: 0)')
    target.add_argument(
        '--frame', default=None,
        help='frame the pose is given in (default: the planning frame)')
    target.add_argument(
        '--linear', action='store_true',
        help='move along a straight line instead of planning freely')
    target.add_argument(
        '--position-tolerance', type=float, default=0.005,
        help='position tolerance [m] (default: 0.005)')
    target.add_argument(
        '--orientation-tolerance', type=float, default=0.01,
        help='orientation tolerance [rad] (default: 0.01)')
    add_motion_arguments(parser)
    add_common_arguments(parser)
    return parser


def main(argv=None) -> int:
    arguments = build_parser().parse_args(argv if argv is not None else sys.argv[1:])

    robot = make_robot(arguments, node_name='move_pose_cli')
    try:
        if not robot.wait_for_state(timeout=arguments.timeout):
            print('no /joint_states: is the robot running?', file=sys.stderr)
            return EXIT_FAILURE

        print(
            f'target: x={arguments.x:.3f} y={arguments.y:.3f} z={arguments.z:.3f} '
            f'rpy=({arguments.roll:.3f}, {arguments.pitch:.3f}, {arguments.yaw:.3f})')

        if arguments.linear:
            result = robot.move_linear(
                x=arguments.x, y=arguments.y, z=arguments.z,
                roll=arguments.roll, pitch=arguments.pitch, yaw=arguments.yaw,
                wait=not arguments.no_wait)
        else:
            result = robot.move_to_pose(
                x=arguments.x, y=arguments.y, z=arguments.z,
                roll=arguments.roll, pitch=arguments.pitch, yaw=arguments.yaw,
                frame=arguments.frame,
                position_tolerance=arguments.position_tolerance,
                orientation_tolerance=arguments.orientation_tolerance,
                velocity_scaling=arguments.velocity_scaling,
                acceleration_scaling=arguments.acceleration_scaling,
                wait=not arguments.no_wait)

        print(result.message)

        if result.success and not arguments.no_wait:
            pose = robot.get_current_pose_rpy()
            if pose is not None:
                print(
                    f"reached: x={pose['x']:.3f} y={pose['y']:.3f} z={pose['z']:.3f} "
                    f"rpy=({pose['roll']:.3f}, {pose['pitch']:.3f}, {pose['yaw']:.3f})")

        return EXIT_OK if result.success else EXIT_FAILURE
    finally:
        robot.shutdown()


def cli() -> int:
    return run(main)


if __name__ == '__main__':
    sys.exit(cli())
