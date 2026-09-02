# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Software emergency stop.

    ros2 run robot_arm_tools e_stop -- --engage
    ros2 run robot_arm_tools e_stop -- --release

Engaging zeroes the commands, de-energises the drives and aborts the running
trajectory.  It is latched: releasing it clears the software latch only, and
the drives stay off until they are enabled again on purpose.

This is a SOFTWARE stop.  It does not replace a hardware e-stop chain, and it
cannot be relied on when the software itself is the thing that failed.
"""

import argparse
import sys

from .cli_common import add_common_arguments, EXIT_FAILURE, EXIT_OK, make_robot, run


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='e_stop', description='Engage or release the software emergency stop.')
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument('--engage', action='store_true', help='engage the emergency stop')
    action.add_argument('--release', action='store_true', help='release the emergency stop')
    parser.add_argument(
        '--reason', default='operator', help='reason recorded in the log')
    parser.add_argument(
        '--enable', action='store_true',
        help='re-enable the drives after releasing (use with --release)')
    add_common_arguments(parser)
    return parser


def main(argv=None) -> int:
    arguments = build_parser().parse_args(argv if argv is not None else sys.argv[1:])

    robot = make_robot(arguments, use_moveit=False, node_name='e_stop_cli')
    try:
        result = robot.set_e_stop(arguments.engage, reason=arguments.reason)
        print(result.message)
        success = result.success

        if arguments.release and arguments.enable:
            enabled = robot.enable()
            print(enabled.message)
            success = success and enabled.success

        return EXIT_OK if success else EXIT_FAILURE
    finally:
        robot.shutdown()


def cli() -> int:
    return run(main)


if __name__ == '__main__':
    sys.exit(cli())
