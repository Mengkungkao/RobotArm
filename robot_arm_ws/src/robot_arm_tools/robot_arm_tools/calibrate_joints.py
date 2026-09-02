# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Joint zero calibration.

    ros2 run robot_arm_tools calibrate_joints                 # guided, all joints
    ros2 run robot_arm_tools calibrate_joints -- --show       # print what is in force
    ros2 run robot_arm_tools calibrate_joints -- \
        --joint joint_3 --position 0.0                        # one joint, no prompts
    ros2 run robot_arm_tools calibrate_joints -- --save       # persist to calibration.yaml

How it works
------------
Every joint reports a raw encoder angle.  Calibration records the offset that
makes that raw angle read the TRUE joint angle at a known mechanical
reference - a hard stop, a dowel pin, a scribed mark.  The driver then applies

    q_joint = direction * (q_raw - zero_offset)

to every reading and every command, so nothing else in the stack has to know
that the encoders are not zeroed.

The drives are de-energised while calibrating: the arm must be moved by hand
(or jogged first and then disabled), and a powered joint would fight you.
"""

import argparse
import math
import sys
import time

from .cli_common import (add_common_arguments, EXIT_FAILURE, EXIT_OK, JOINT_NAMES,
                         make_robot, run)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='calibrate_joints',
        description='Teach the zero position of one or more joints.')
    parser.add_argument(
        '--joint', default=None,
        help='calibrate only this joint (default: walk through all of them)')
    parser.add_argument(
        '--position', type=float, default=None,
        help='true joint angle at the current pose [rad]; implies a non-interactive run')
    parser.add_argument(
        '--direction', type=int, choices=[-1, 1], default=None,
        help='also set the joint direction (+1 or -1)')
    parser.add_argument(
        '--degrees', action='store_true', help='interpret --position in degrees')
    parser.add_argument(
        '--show', action='store_true', help='print the calibration in force and exit')
    parser.add_argument(
        '--save', action='store_true', help='write the calibration to disk when done')
    parser.add_argument(
        '--file', default='', help='file to write to (default: the file it was loaded from)')
    parser.add_argument(
        '--hardware-node', default='/robot_arm_hardware',
        help='namespace of the hardware driver (default: /robot_arm_hardware)')
    parser.add_argument(
        '--yes', action='store_true', help='do not ask for confirmation')
    add_common_arguments(parser)
    return parser


def _wait(node, future, timeout):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.02)
    return future.result()


class CalibrationClient:
    """Thin wrapper around the driver's calibration services."""

    def __init__(self, node, namespace, timeout):
        from robot_arm_interfaces.srv import CalibrateJoint, GetCalibration, SaveCalibration
        self._node = node
        self._timeout = timeout
        self._calibrate = node.create_client(CalibrateJoint, f'{namespace}/calibrate_joint')
        self._save = node.create_client(SaveCalibration, f'{namespace}/save_calibration')
        self._get = node.create_client(GetCalibration, f'{namespace}/get_calibration')
        self._types = (CalibrateJoint, SaveCalibration, GetCalibration)

    def available(self) -> bool:
        return self._get.wait_for_service(timeout_sec=min(self._timeout, 3.0))

    def get(self):
        response = _wait(
            self._node, self._get.call_async(self._types[2].Request()), self._timeout)
        return None if response is None else response.joints

    def calibrate(self, joint, position, direction=None):
        request = self._types[0].Request()
        request.joint_name = joint
        request.known_position = float(position)
        if direction is not None:
            request.set_direction = True
            request.direction_value = int(direction)
        return _wait(self._node, self._calibrate.call_async(request), self._timeout)

    def save(self, path=''):
        request = self._types[1].Request()
        request.file_path = path
        return _wait(self._node, self._save.call_async(request), self._timeout)


def print_calibration(records) -> None:
    print(f'{"joint":<10}{"zero_offset":>14}{"dir":>6}{"min":>10}{"max":>10}{"home":>10}')
    print('-' * 60)
    for record in records:
        print(
            f'{record.name:<10}{record.zero_offset:14.6f}{record.direction:6d}'
            f'{record.min_position:10.4f}{record.max_position:10.4f}'
            f'{record.home_position:10.4f}')


def calibrate_interactively(client, robot, joints, degrees) -> bool:
    print(
        '\nGuided calibration\n'
        '------------------\n'
        'For each joint: move it to a known mechanical reference, type the true\n'
        'angle at that pose, and press Enter.  Press Enter on its own to skip a\n'
        'joint, or type q to stop.\n')

    all_ok = True
    for joint in joints:
        positions = dict(zip(robot.joint_names, robot.get_joint_positions()))
        current = positions.get(joint, float('nan'))
        shown = math.degrees(current) if degrees else current
        unit = 'deg' if degrees else 'rad'

        answer = input(
            f'{joint}: currently reads {shown:8.4f} {unit}. '
            f'True angle at this pose [{unit}] (Enter=skip, q=quit): ').strip()
        if answer.lower() in ('q', 'quit', 'exit'):
            break
        if not answer:
            print(f'  {joint} skipped')
            continue

        try:
            value = float(answer)
        except ValueError:
            print(f'  "{answer}" is not a number, {joint} skipped')
            all_ok = False
            continue

        response = client.calibrate(joint, math.radians(value) if degrees else value)
        if response is None:
            print(f'  {joint}: the driver did not answer')
            all_ok = False
        elif response.success:
            print(f'  {joint}: zero_offset = {response.zero_offset:.6f} rad')
        else:
            print(f'  {joint}: {response.message}')
            all_ok = False

    return all_ok


def main(argv=None) -> int:
    arguments = build_parser().parse_args(argv if argv is not None else sys.argv[1:])

    robot = make_robot(arguments, use_moveit=False, node_name='calibrate_joints_cli')
    try:
        client = CalibrationClient(robot.node, arguments.hardware_node, arguments.timeout)
        if not client.available():
            print(
                f'no calibration services under {arguments.hardware_node}.\n'
                'Calibration needs the real hardware driver: launch with\n'
                '  ros2 launch robot_arm_bringup real.launch.py',
                file=sys.stderr)
            return EXIT_FAILURE

        if arguments.show:
            records = client.get()
            if records is None:
                print('the driver did not answer', file=sys.stderr)
                return EXIT_FAILURE
            print_calibration(records)
            return EXIT_OK

        # Calibrating a powered joint is both useless and dangerous.
        disabled = robot.disable()
        print(f'drives: {disabled.message}')
        if not robot.wait_for_state(timeout=arguments.timeout):
            print('no /joint_states: is the driver running?', file=sys.stderr)
            return EXIT_FAILURE

        if arguments.position is not None:
            if arguments.joint is None:
                print('--position requires --joint', file=sys.stderr)
                return EXIT_FAILURE
            position = math.radians(arguments.position) if arguments.degrees \
                else arguments.position
            response = client.calibrate(arguments.joint, position, arguments.direction)
            if response is None:
                print('the driver did not answer', file=sys.stderr)
                return EXIT_FAILURE
            print(response.message)
            success = response.success
        else:
            joints = [arguments.joint] if arguments.joint else list(JOINT_NAMES)
            success = calibrate_interactively(client, robot, joints, arguments.degrees)

        records = client.get()
        if records:
            print()
            print_calibration(records)

        if arguments.save:
            if not arguments.yes:
                answer = input('\nWrite this calibration to disk? [y/N] ').strip().lower()
                if answer not in ('y', 'yes'):
                    print('not saved')
                    return EXIT_OK if success else EXIT_FAILURE
            response = client.save(arguments.file)
            if response is None:
                print('the driver did not answer', file=sys.stderr)
                return EXIT_FAILURE
            print(response.message)
            success = success and response.success
        elif success:
            print(
                '\nThe calibration is live but NOT saved. '
                'Re-run with --save to persist it.')

        return EXIT_OK if success else EXIT_FAILURE
    finally:
        robot.shutdown()


def cli() -> int:
    return run(main)


if __name__ == '__main__':
    sys.exit(cli())
