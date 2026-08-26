# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Shared plumbing for the command-line tools.

Every tool follows the same contract:

  * the same connection options, so a tool can be pointed at a namespaced
    robot without editing anything;
  * degrees or radians on request - a human typing a joint angle usually
    thinks in degrees, while everything inside the stack is radians;
  * an exit code that means something: 0 success, 1 failure, 2 bad usage,
    130 interrupted.  That is what makes the tools usable from a script.

None of them know whether they are talking to Gazebo or to the real machine.
"""

import argparse
import math
import sys
from typing import List, Optional, Sequence

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

EXIT_OK = 0
EXIT_FAILURE = 1
EXIT_USAGE = 2
EXIT_INTERRUPTED = 130


def add_common_arguments(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    """Connection options shared by every tool."""
    group = parser.add_argument_group('connection')
    group.add_argument(
        '--group', default='arm', help='MoveIt planning group (default: arm)')
    group.add_argument(
        '--base-frame', default='base_link', help='planning frame (default: base_link)')
    group.add_argument(
        '--ee-frame', default='tool0', help='end-effector frame (default: tool0)')
    group.add_argument(
        '--controller', default='arm_controller',
        help='joint trajectory controller (default: arm_controller)')
    group.add_argument(
        '--timeout', type=float, default=10.0,
        help='service/action timeout in seconds (default: 10)')
    return parser


def add_motion_arguments(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    """Options shared by the tools that move the arm."""
    group = parser.add_argument_group('motion')
    group.add_argument(
        '--velocity-scaling', type=float, default=0.3,
        help='fraction of the maximum joint velocity (default: 0.3)')
    group.add_argument(
        '--acceleration-scaling', type=float, default=0.3,
        help='fraction of the maximum joint acceleration (default: 0.3)')
    group.add_argument(
        '--no-wait', action='store_true',
        help='return as soon as the goal is accepted instead of waiting for it')
    return parser


def add_joint_arguments(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    """--j1 ... --j6, plus the degrees switch."""
    group = parser.add_argument_group('joint targets')
    for index in range(1, len(JOINT_NAMES) + 1):
        group.add_argument(
            f'--j{index}', type=float, default=None,
            help=f'target for joint_{index}; omitted joints keep their current value')
    group.add_argument(
        '--degrees', action='store_true',
        help='interpret and print joint angles in degrees instead of radians')
    return parser


def collect_joint_targets(arguments, current: Optional[Sequence[float]] = None) -> List[float]:
    """
    Turn --j1 ... --j6 into a full target vector.

    A joint that was not given keeps its current value, so a one-joint jog
    does not require typing the other five.
    """
    targets: List[float] = []
    for index, name in enumerate(JOINT_NAMES):
        value = getattr(arguments, f'j{index + 1}')
        if value is None:
            if current is None or index >= len(current) or not math.isfinite(current[index]):
                raise ValueError(
                    f'no value given for {name} and the current position is unknown')
            targets.append(float(current[index]))
        else:
            targets.append(math.radians(value) if arguments.degrees else float(value))
    return targets


def format_angle(value: float, degrees: bool) -> str:
    """Format an angle for a terminal, in the unit the user asked for."""
    if not math.isfinite(value):
        return '     n/a'
    return f'{math.degrees(value):8.2f}' if degrees else f'{value:8.4f}'


def angle_unit(degrees: bool) -> str:
    return 'deg' if degrees else 'rad'


def make_robot(arguments, use_moveit: bool = True, node_name: str = 'robot_arm_cli'):
    """Build a RobotArm from the common command-line options."""
    from robot_arm_control import RobotArm
    return RobotArm(
        node_name=node_name,
        group_name=arguments.group,
        base_frame=arguments.base_frame,
        end_effector_frame=arguments.ee_frame,
        controller=arguments.controller,
        use_moveit=use_moveit,
        velocity_scaling=getattr(arguments, 'velocity_scaling', 0.3),
        acceleration_scaling=getattr(arguments, 'acceleration_scaling', 0.3),
        timeout=arguments.timeout,
    )


def run(main_function) -> int:
    """
    Wrap a tool's body with uniform error handling.

    Ctrl-C is a normal way to abandon a motion, so it exits 130 quietly
    instead of dumping a traceback at the operator.
    """
    try:
        return main_function()
    except KeyboardInterrupt:
        print('\ninterrupted', file=sys.stderr)
        return EXIT_INTERRUPTED
    except ValueError as error:
        print(f'error: {error}', file=sys.stderr)
        return EXIT_USAGE
    except Exception as error:      # noqa: BLE001 - a CLI must not show a traceback
        print(f'error: {error}', file=sys.stderr)
        return EXIT_FAILURE
