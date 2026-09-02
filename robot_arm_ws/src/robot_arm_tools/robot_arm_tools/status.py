# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
One screen with everything needed to answer "is the robot healthy?".

    ros2 run robot_arm_tools status
    ros2 run robot_arm_tools status -- --watch

Works in both modes: the sections that only exist on the real robot (drive
temperatures, bus errors) are simply left out in simulation.
"""

import argparse
import math
import sys
import time

from .cli_common import (add_common_arguments, angle_unit, EXIT_FAILURE, EXIT_OK,
                         format_angle, make_robot, run)

SAFETY_LEVELS = {0: 'OK', 1: 'WARN', 2: 'VIOLATION', 3: 'E-STOP'}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='status',
        description='Show joint states, controllers, safety and hardware status.')
    parser.add_argument('--degrees', action='store_true', help='show angles in degrees')
    parser.add_argument(
        '--watch', action='store_true', help='refresh until interrupted')
    parser.add_argument(
        '--interval', type=float, default=1.0, help='refresh interval [s] (default: 1)')
    add_common_arguments(parser)
    return parser


def _collect_controllers(node, timeout):
    """Ask the controller_manager which controllers exist and their state."""
    try:
        from controller_manager_msgs.srv import ListControllers
    except ImportError:
        return None

    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    try:
        if not client.wait_for_service(timeout_sec=min(timeout, 2.0)):
            return None
        future = client.call_async(ListControllers.Request())
        deadline = time.monotonic() + min(timeout, 3.0)
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        response = future.result()
        return None if response is None else response.controller
    finally:
        node.destroy_client(client)


def _latest_message(node, topic, message_type, timeout=1.0):
    """Grab one message from a topic, or None when nobody publishes it."""
    received = {}

    def callback(message):
        received['message'] = message

    subscription = node.create_subscription(message_type, topic, callback, 10)
    try:
        deadline = time.monotonic() + timeout
        while 'message' not in received and time.monotonic() < deadline:
            time.sleep(0.02)
        return received.get('message')
    finally:
        node.destroy_subscription(subscription)


def print_status(robot, arguments) -> bool:
    node = robot.node
    unit = angle_unit(arguments.degrees)
    healthy = True

    print('=' * 68)
    print('robot_arm status')
    print('=' * 68)

    # --- joints ------------------------------------------------------------
    states = robot.get_joint_states()
    have_states = any(math.isfinite(value) for value in states.positions)
    print(f'\njoints  (position [{unit}] / velocity / effort [Nm])')
    if not have_states:
        print('  no /joint_states received - the robot is not running')
        healthy = False
    else:
        for index, name in enumerate(states.names):
            print(
                f'  {name}: {format_angle(states.positions[index], arguments.degrees)}   '
                f'{format_angle(states.velocities[index], arguments.degrees)}   '
                f'{states.efforts[index]:8.3f}')

    # --- controllers -------------------------------------------------------
    print('\ncontrollers')
    controllers = _collect_controllers(node, arguments.timeout)
    if controllers is None:
        print('  controller_manager is not reachable')
        healthy = False
    else:
        for controller in controllers:
            marker = 'active' if controller.state == 'active' else controller.state
            print(f'  {controller.name}: {marker}  ({controller.type})')
        if not any(c.name == robot.controller and c.state == 'active' for c in controllers):
            print(f'  note: {robot.controller} is not active, motion commands will not execute')
            healthy = False

    # --- safety ------------------------------------------------------------
    print('\nsafety')
    try:
        from robot_arm_msgs.msg import SafetyStatus
        safety = _latest_message(node, '/robot_arm/safety_status', SafetyStatus, 1.5)
    except ImportError:
        safety = None

    if safety is None:
        print('  safety monitor is not publishing')
    else:
        print(f'  level: {SAFETY_LEVELS.get(safety.level, safety.level)}')
        print(f'  e-stop: {"ENGAGED" if safety.e_stop_active else "released"}')
        if safety.message:
            print(f'  message: {safety.message}')
        if safety.violating_joints:
            print(f'  violating joints: {", ".join(safety.violating_joints)}')
        if safety.level >= 2:
            healthy = False

    # --- hardware (real robot only) ----------------------------------------
    try:
        from robot_arm_msgs.msg import ArmStatus
        hardware = _latest_message(node, '/robot_arm_hardware/status', ArmStatus, 1.5)
    except ImportError:
        hardware = None

    if hardware is not None:
        print('\nhardware')
        print(f'  transport: {hardware.transport}   protocol: {hardware.protocol}')
        print(f'  connected: {hardware.connected}   drives enabled: {hardware.enabled}')
        print(f'  communication ok: {hardware.communication_ok}   '
              f'last reply: {hardware.last_comm_age:.3f} s ago')
        print(f'  errors: {hardware.read_errors} read / {hardware.write_errors} write')
        print('\n  drives (current [A] / temperature [degC] / raw encoder)')
        for joint in hardware.joints:
            print(
                f'    {joint.name}: {joint.current:7.3f}   {joint.temperature:7.2f}   '
                f'{joint.raw_encoder:12d}'
                + (f'   FAULT {joint.fault_code}' if joint.fault_code else ''))
        if not hardware.communication_ok or not hardware.connected:
            healthy = False
    else:
        print('\nhardware\n  no hardware driver detected (simulation, or the arm is offline)')

    # --- pose --------------------------------------------------------------
    pose = robot.get_current_pose_rpy()
    if pose is not None:
        print(
            f'\n{robot.end_effector_frame} in {robot.base_frame}: '
            f"x={pose['x']:.4f} y={pose['y']:.4f} z={pose['z']:.4f}  "
            f"rpy=({pose['roll']:.4f}, {pose['pitch']:.4f}, {pose['yaw']:.4f})")

    print()
    return healthy


def main(argv=None) -> int:
    arguments = build_parser().parse_args(argv if argv is not None else sys.argv[1:])

    # No MoveIt needed: status must work even when move_group is not running.
    robot = make_robot(arguments, use_moveit=False, node_name='status_cli')
    try:
        robot.wait_for_state(timeout=min(arguments.timeout, 3.0))
        if not arguments.watch:
            return EXIT_OK if print_status(robot, arguments) else EXIT_FAILURE

        while True:
            print('\033[2J\033[H', end='')      # clear the screen
            print_status(robot, arguments)
            time.sleep(max(0.1, arguments.interval))
    finally:
        robot.shutdown()


def cli() -> int:
    return run(main)


if __name__ == '__main__':
    sys.exit(cli())
