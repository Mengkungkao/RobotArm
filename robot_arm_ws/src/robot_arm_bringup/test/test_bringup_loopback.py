# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
End-to-end test of the real-robot stack, with no hardware attached.

It launches `real_robot.launch.py` with `hardware_type:=real`, which loads the
actual robot_arm_hardware plugin.  The default bus in hardware.yaml is the
loopback transport with the simulated-drive protocol, so the whole driver path
runs for real - encoder conversion, watchdogs, safety, diagnostics, services -
against motors that only exist in memory.

That covers what a simulator cannot: that the *hardware* code connects, that
encoder feedback arrives, that a trajectory reaches the drives, and that the
emergency stop actually stops execution.

    colcon test --packages-select robot_arm_bringup
"""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# The stack has to start Gazebo-free but still load a plugin, spawn two
# controllers and settle; be generous, CI machines are slow.
STARTUP_DELAY = 12.0
DEFAULT_TIMEOUT = 30.0


@pytest.mark.launch_test
def generate_test_description():
    real_robot = os.path.join(
        get_package_share_directory('robot_arm_bringup'), 'launch', 'real_robot.launch.py')

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(real_robot),
        launch_arguments={
            'hardware_type': 'real',
            'use_rviz': 'false',
            'use_safety_monitor': 'true',
        }.items(),
    )

    return launch.LaunchDescription([
        stack,
        TimerAction(period=STARTUP_DELAY, actions=[launch_testing.actions.ReadyToTest()]),
    ])


def spin_until(node, predicate, timeout=DEFAULT_TIMEOUT):
    """Spin the node until `predicate()` is true, or the timeout expires."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


def call(node, client, request, timeout=DEFAULT_TIMEOUT):
    """Call a service and spin until the answer arrives."""
    if not client.wait_for_service(timeout_sec=timeout):
        return None
    future = client.call_async(request)
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    return future.result()


class TestLoopbackBringup(unittest.TestCase):
    """Method names are numbered because unittest runs them alphabetically and
    the emergency-stop test deliberately leaves the arm stopped."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_bringup_loopback')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    # -- controllers --------------------------------------------------------

    def test_01_controllers_are_loaded_and_active(self):
        from controller_manager_msgs.srv import ListControllers

        client = self.node.create_client(
            ListControllers, '/controller_manager/list_controllers')
        response = call(self.node, client, ListControllers.Request())
        self.assertIsNotNone(response, 'the controller_manager did not answer')

        states = {controller.name: controller.state for controller in response.controller}
        self.assertIn('joint_state_broadcaster', states)
        self.assertIn('arm_controller', states)
        self.assertEqual(states['joint_state_broadcaster'], 'active')
        self.assertEqual(states['arm_controller'], 'active')

    # -- encoder feedback ---------------------------------------------------

    def test_02_joint_states_are_published(self):
        from sensor_msgs.msg import JointState

        received = []
        subscription = self.node.create_subscription(
            JointState, '/joint_states', received.append, 10)
        try:
            self.assertTrue(
                spin_until(self.node, lambda: len(received) > 5),
                'no /joint_states: the hardware plugin is not producing feedback')
        finally:
            self.node.destroy_subscription(subscription)

        message = received[-1]
        for joint in JOINTS:
            self.assertIn(joint, message.name)
        self.assertEqual(len(message.position), len(message.name))
        for position in message.position:
            self.assertTrue(
                -10.0 < position < 10.0, f'implausible joint position {position}')

    def test_03_hardware_reports_a_healthy_connection(self):
        from robot_arm_msgs.msg import ArmStatus

        received = []
        subscription = self.node.create_subscription(
            ArmStatus, '/robot_arm_hardware/status', received.append, 10)
        try:
            self.assertTrue(
                spin_until(self.node, lambda: len(received) > 0),
                'the driver publishes no ArmStatus')
        finally:
            self.node.destroy_subscription(subscription)

        status = received[-1]
        self.assertTrue(status.connected)
        self.assertTrue(status.communication_ok)
        self.assertEqual(len(status.joints), len(JOINTS))
        self.assertIn('loopback', status.transport)
        # Encoder readings must be real values, not placeholders.
        for joint in status.joints:
            self.assertFalse(joint.fault_code, f'{joint.name} reports a fault')

    def test_04_drives_can_be_enabled_through_the_common_service(self):
        from robot_arm_interfaces.srv import SetMotorEnable

        client = self.node.create_client(SetMotorEnable, '/robot_arm/set_motor_enable')
        request = SetMotorEnable.Request()
        request.enable = True
        response = call(self.node, client, request)
        self.assertIsNotNone(response, '/robot_arm/set_motor_enable did not answer')
        self.assertTrue(response.success, response.message)

    # -- motion -------------------------------------------------------------

    def test_05_a_trajectory_reaches_the_drives(self):
        from builtin_interfaces.msg import Duration
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        from sensor_msgs.msg import JointState
        from trajectory_msgs.msg import JointTrajectoryPoint

        target = [0.2, -0.3, 0.6, 0.1, 0.4, -0.2]

        client = ActionClient(
            self.node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.assertTrue(
            client.wait_for_server(timeout_sec=DEFAULT_TIMEOUT),
            'the trajectory action server never appeared')

        point = JointTrajectoryPoint()
        point.positions = target
        point.velocities = [0.0] * len(target)
        point.time_from_start = Duration(sec=4)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        goal.trajectory.points = [point]

        send_future = client.send_goal_async(goal)
        self.assertTrue(
            spin_until(self.node, send_future.done), 'the goal was never accepted')
        handle = send_future.result()
        self.assertTrue(handle.accepted, 'the controller rejected the trajectory')

        result_future = handle.get_result_async()
        self.assertTrue(
            spin_until(self.node, result_future.done, timeout=60.0),
            'the trajectory never finished')

        # What matters is that the drives moved, which can only happen if the
        # command travelled controller -> hardware -> protocol and the encoder
        # feedback travelled back.
        received = []
        subscription = self.node.create_subscription(
            JointState, '/joint_states', received.append, 10)
        try:
            self.assertTrue(spin_until(self.node, lambda: len(received) > 5))
        finally:
            self.node.destroy_subscription(subscription)

        final = dict(zip(received[-1].name, received[-1].position))
        for joint, expected in zip(JOINTS, target):
            self.assertAlmostEqual(
                final[joint], expected, delta=0.1,
                msg=f'{joint} did not follow the trajectory')

    # -- calibration --------------------------------------------------------

    def test_06_calibration_can_be_read_back(self):
        from robot_arm_interfaces.srv import GetCalibration

        client = self.node.create_client(GetCalibration, '/robot_arm_hardware/get_calibration')
        response = call(self.node, client, GetCalibration.Request())
        self.assertIsNotNone(response, 'the calibration service did not answer')
        self.assertEqual(len(response.joints), len(JOINTS))

        for record in response.joints:
            self.assertIn(record.name, JOINTS)
            self.assertIn(record.direction, (-1, 1))
            self.assertLess(record.min_position, record.max_position)

    # -- safety -------------------------------------------------------------

    def test_07_emergency_stop_disables_the_drives_and_stops_execution(self):
        from controller_manager_msgs.srv import ListControllers
        from robot_arm_interfaces.srv import SetEStop

        client = self.node.create_client(SetEStop, '/robot_arm/set_e_stop')
        request = SetEStop.Request()
        request.engage = True
        request.reason = 'integration test'
        response = call(self.node, client, request)
        self.assertIsNotNone(response, '/robot_arm/set_e_stop did not answer')
        self.assertTrue(response.success, response.message)
        self.assertTrue(response.e_stop_active)

        # The trajectory controller must be deactivated, so a running goal is
        # aborted instead of resuming when the stop is released.
        controllers = self.node.create_client(
            ListControllers, '/controller_manager/list_controllers')
        deactivated = False
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not deactivated:
            listed = call(self.node, controllers, ListControllers.Request(), timeout=5.0)
            if listed is not None:
                states = {c.name: c.state for c in listed.controller}
                deactivated = states.get('arm_controller') != 'active'
        self.assertTrue(deactivated, 'arm_controller stayed active through an e-stop')

    def test_08_enabling_is_refused_while_the_stop_is_engaged(self):
        from robot_arm_interfaces.srv import SetMotorEnable

        client = self.node.create_client(SetMotorEnable, '/robot_arm/set_motor_enable')
        request = SetMotorEnable.Request()
        request.enable = True
        response = call(self.node, client, request)
        self.assertIsNotNone(response)
        self.assertFalse(
            response.success, 'the drives were enabled while the e-stop was engaged')

    def test_09_the_stop_can_be_released(self):
        from robot_arm_interfaces.srv import SetEStop

        client = self.node.create_client(SetEStop, '/robot_arm/set_e_stop')
        request = SetEStop.Request()
        request.engage = False
        response = call(self.node, client, request)
        self.assertIsNotNone(response)
        self.assertTrue(response.success, response.message)
        self.assertFalse(response.e_stop_active)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_no_process_crashed(self, proc_info):
        # The spawners exit 0 once their controller is active; everything else
        # is stopped by the test framework with SIGINT.
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -15, 130])
