# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Python API tests.

The rotation helpers are pure functions and are tested exhaustively.  The
RobotArm tests need an rclpy context but no robot: they check the parts that
must be right before a single message is sent - argument validation, joint
ordering and the graceful behaviour when no backend is running.
"""

import math
import os
import sys

import pytest

# Allow running the file directly from the source tree as well as installed.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_arm_control.transforms import (euler_from_quaternion,      # noqa: E402
                                          normalize_quaternion,
                                          quaternion_from_euler,
                                          quaternion_multiply)


# ---------------------------------------------------------------------------
# transforms
# ---------------------------------------------------------------------------

def test_identity_rotation():
    assert quaternion_from_euler(0.0, 0.0, 0.0) == pytest.approx((0.0, 0.0, 0.0, 1.0))
    assert euler_from_quaternion(0.0, 0.0, 0.0, 1.0) == pytest.approx((0.0, 0.0, 0.0))


@pytest.mark.parametrize('rpy', [
    (0.0, 0.0, 0.0),
    (0.1, -0.2, 0.3),
    (0.0, math.pi / 2 - 1e-3, 0.0),
    (math.pi / 4, math.pi / 6, -math.pi / 3),
    (-1.2, 0.4, 2.9),
])
def test_euler_quaternion_round_trip(rpy):
    quaternion = quaternion_from_euler(*rpy)
    recovered = euler_from_quaternion(*quaternion)
    # Euler angles are not unique, so compare through the quaternion again.
    assert quaternion_from_euler(*recovered) == pytest.approx(quaternion, abs=1e-9)


def test_quaternions_are_unit_length():
    for rpy in [(0.3, 0.4, 0.5), (1.0, -1.0, 2.0)]:
        x, y, z, w = quaternion_from_euler(*rpy)
        assert math.sqrt(x * x + y * y + z * z + w * w) == pytest.approx(1.0)


def test_pitch_of_ninety_degrees():
    """The pose used throughout the docs: tool pointing straight down."""
    x, y, z, w = quaternion_from_euler(0.0, math.pi / 2, 0.0)
    assert (x, y, z, w) == pytest.approx(
        (0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4)))


def test_normalisation_handles_a_degenerate_quaternion():
    assert normalize_quaternion(0.0, 0.0, 0.0, 0.0) == (0.0, 0.0, 0.0, 1.0)
    assert normalize_quaternion(0.0, 0.0, 0.0, 5.0) == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_quaternion_multiplication_composes_rotations():
    first = quaternion_from_euler(0.0, 0.0, math.pi / 2)
    second = quaternion_from_euler(0.0, 0.0, math.pi / 2)
    combined = normalize_quaternion(*quaternion_multiply(second, first))
    expected = quaternion_from_euler(0.0, 0.0, math.pi)
    # q and -q are the same rotation.
    assert (combined == pytest.approx(expected, abs=1e-9) or
            combined == pytest.approx(tuple(-v for v in expected), abs=1e-9))


# ---------------------------------------------------------------------------
# RobotArm
# ---------------------------------------------------------------------------

@pytest.fixture(scope='module')
def ros():
    rclpy = pytest.importorskip('rclpy')
    pytest.importorskip('robot_arm_interfaces.srv')
    if not rclpy.ok():
        rclpy.init()
    yield rclpy
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def robot(ros):
    from robot_arm_control import RobotArm
    # use_moveit=False keeps the fixture usable on an installation without
    # MoveIt; the MoveIt paths are covered by the integration tests.
    arm = RobotArm(node_name='test_robot_arm_api', use_moveit=False, timeout=1.0)
    yield arm
    arm.shutdown()


def test_default_joint_names_and_frames(robot):
    assert robot.joint_names == ['joint_1', 'joint_2', 'joint_3',
                                 'joint_4', 'joint_5', 'joint_6']
    assert robot.base_frame == 'base_link'
    assert robot.end_effector_frame == 'tool0'
    assert robot.group_name == 'arm'


def test_move_joints_rejects_a_wrong_length_target(robot):
    result = robot.move_joints([0.0, 0.0, 0.0])
    assert not result
    assert 'expected 6' in result.message


def test_move_joints_rejects_non_finite_targets(robot):
    result = robot.move_joints([0.0, float('nan'), 0.0, 0.0, 0.0, 0.0])
    assert not result
    assert 'finite' in result.message


def test_joint_states_are_reported_in_canonical_order(robot):
    states = robot.get_joint_states()
    assert states.names == robot.joint_names
    # No robot is running, so every value is NaN rather than a stale zero.
    assert len(states.positions) == 6
    assert all(math.isnan(value) for value in states.positions)
    assert set(states.as_dict()) == set(robot.joint_names)


def test_cartesian_motion_requires_moveit(robot):
    result = robot.move_to_pose(x=0.35, y=0.1, z=0.4)
    assert not result
    assert 'MoveIt' in result.message


def test_calls_fail_gracefully_without_a_backend(robot):
    """Nothing may raise just because the robot is not running."""
    assert not robot.wait_for_state(timeout=0.2)
    assert not robot.enable()
    assert not robot.disable()
    assert robot.get_calibration() is None
    assert robot.inverse_kinematics(0.3, 0.0, 0.4) is None
    assert robot.forward_kinematics() is None


def test_e_stop_falls_back_to_the_topic(robot):
    """With no service running the request still reaches /e_stop."""
    result = robot.set_e_stop(True, reason='unit test')
    assert result.success
    assert '/e_stop' in result.message
