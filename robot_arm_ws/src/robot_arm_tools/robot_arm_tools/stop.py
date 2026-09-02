# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Stop the arm.

    ros2 run robot_arm_tools stop

Aborts the running trajectory and holds the current pose.  For an emergency
stop - which also cuts the drives and latches until released - use:

    ros2 run robot_arm_tools e_stop -- --engage
"""

import argparse
import sys

from .cli_common import add_common_arguments, EXIT_FAILURE, EXIT_OK, make_robot, run


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='stop', description='Abort the current motion and hold the pose.')
    parser.add_argument(
        '--disable', action='store_true',
        help='also de-energise the drives after stopping')
    add_common_arguments(parser)
    return parser


def main(argv=None) -> int:
    arguments = build_parser().parse_args(argv if argv is not None else sys.argv[1:])

    robot = make_robot(arguments, use_moveit=False, node_name='stop_cli')
    try:
        result = robot.stop()
        print(result.message)
        success = result.success

        if arguments.disable:
            disabled = robot.disable()
            print(disabled.message)
            success = success and disabled.success

        return EXIT_OK if success else EXIT_FAILURE
    finally:
        robot.shutdown()


def cli() -> int:
    return run(main)


if __name__ == '__main__':
    sys.exit(cli())
