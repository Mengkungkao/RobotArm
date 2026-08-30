# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Configuration consistency tests.

A controller configured for five joints, or a safety limit wider than the
mechanical one, is the kind of mistake that only shows up when the arm is
already moving.  These checks catch it at build time.
"""

import os

import pytest
import yaml

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(os.path.dirname(HERE), 'config')


def load(name, package=None):
    if package is None:
        path = os.path.join(CONFIG, name)
    else:
        try:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory(package), 'config', name)
        except Exception:                     # not installed: use the source tree
            source_root = os.path.dirname(os.path.dirname(HERE))   # .../src
            path = os.path.join(source_root, package, 'config', name)
    if not os.path.exists(path):
        pytest.skip(f'{path} is not available')
    with open(path) as handle:
        return yaml.safe_load(handle)


# ---------------------------------------------------------------------------
# controllers.yaml
# ---------------------------------------------------------------------------

@pytest.fixture(scope='module')
def controllers():
    return load('controllers.yaml')


def test_controller_manager_declares_the_expected_controllers(controllers):
    params = controllers['controller_manager']['ros__parameters']
    assert params['update_rate'] >= 50, 'a 6-DOF arm needs at least 50 Hz'
    assert params['joint_state_broadcaster']['type'] == \
        'joint_state_broadcaster/JointStateBroadcaster'
    assert params['arm_controller']['type'] == \
        'joint_trajectory_controller/JointTrajectoryController'


def test_arm_controller_covers_every_joint(controllers):
    params = controllers['arm_controller']['ros__parameters']
    assert params['joints'] == JOINTS
    assert 'position' in params['command_interfaces']
    assert 'position' in params['state_interfaces']
    assert 'velocity' in params['state_interfaces']
    # A partial goal on a 6-DOF arm is nearly always a caller bug.
    assert params['allow_partial_joints_goal'] is False


def test_every_joint_has_trajectory_tolerances(controllers):
    constraints = controllers['arm_controller']['ros__parameters']['constraints']
    for joint in JOINTS:
        assert joint in constraints, f'{joint} has no trajectory tolerance'
        assert constraints[joint]['goal'] > 0.0
        assert constraints[joint]['trajectory'] > 0.0


def test_velocity_controller_claims_the_same_joints(controllers):
    assert controllers['velocity_controller']['ros__parameters']['joints'] == JOINTS
    assert controllers['velocity_controller']['ros__parameters']['interface_name'] == 'velocity'


# ---------------------------------------------------------------------------
# safety.yaml vs the mechanical description
# ---------------------------------------------------------------------------

@pytest.fixture(scope='module')
def safety():
    return load('safety.yaml')['safety_monitor']['ros__parameters']


@pytest.fixture(scope='module')
def robot():
    return load('robot.yaml', package='robot_arm_description')


def test_safety_supervises_every_joint(safety):
    assert safety['joints'] == JOINTS
    for joint in JOINTS:
        assert joint in safety['limits']


def test_safety_limits_are_never_wider_than_the_mechanical_limits(safety, robot):
    """The software limit may be tighter than the URDF, never looser."""
    for joint in JOINTS:
        mechanical = robot['joints'][joint]
        software = safety['limits'][joint]
        assert software['min_position'] >= mechanical['lower'] - 1e-6, \
            f'{joint}: safety min_position is below the URDF limit'
        assert software['max_position'] <= mechanical['upper'] + 1e-6, \
            f'{joint}: safety max_position is above the URDF limit'
        assert software['max_velocity'] <= mechanical['velocity'] + 1e-6, \
            f'{joint}: safety max_velocity is above the URDF limit'
        assert software['max_effort'] <= mechanical['effort'] + 1e-6, \
            f'{joint}: safety max_effort is above the URDF limit'


def test_safety_settings_are_sane(safety):
    assert safety['check_rate'] >= 10.0
    assert safety['joint_state_timeout'] > 0.0
    assert safety['warn_margin'] >= 0.0
    assert isinstance(safety['stop_controller_on_estop'], bool)


# ---------------------------------------------------------------------------
# hardware.yaml / calibration.yaml
# ---------------------------------------------------------------------------

def test_hardware_defaults_to_the_loopback_bus():
    """A fresh clone must not be able to drive a machine by accident."""
    hardware = load('hardware.yaml', package='robot_arm_hardware')['robot_arm_hardware']
    assert hardware['transport']['type'] == 'loopback'
    assert hardware['protocol']['type'] == 'loopback'


def test_every_joint_has_its_own_drive_configuration():
    hardware = load('hardware.yaml', package='robot_arm_hardware')['robot_arm_hardware']
    motor_ids = set()
    for joint in JOINTS:
        drive = hardware['joints'][joint]
        assert drive['encoder_resolution'] > 0
        assert drive['gear_ratio'] != 0
        assert drive['encoder_direction'] in (-1, 1)
        assert drive['motor_id'] not in motor_ids, 'duplicate motor id'
        motor_ids.add(drive['motor_id'])


def test_calibration_limits_stay_inside_the_mechanical_limits(robot):
    calibration = load('calibration.yaml', package='robot_arm_hardware')['calibration']
    for joint in JOINTS:
        record = calibration[joint]
        mechanical = robot['joints'][joint]
        assert record['direction'] in (-1, 1)
        assert record['min_position'] >= mechanical['lower'] - 1e-6
        assert record['max_position'] <= mechanical['upper'] + 1e-6
        assert record['min_position'] <= record['home_position'] <= record['max_position']


def test_drives_can_deliver_the_rated_joint_effort(robot):
    """The drive train and the joint limits must describe the same machine.

    Joint torque = motor current * torque constant * gear ratio.  If the
    configured effort limit needs more current than the drive is allowed to
    draw, the arm will stall on a move the planner considers perfectly valid.
    """
    hardware = load('hardware.yaml', package='robot_arm_hardware')['robot_arm_hardware']
    for joint in JOINTS:
        drive = hardware['joints'][joint]
        effort = robot['joints'][joint]['effort']
        stall_torque = drive['torque_constant'] * drive['gear_ratio'] * drive['max_current']
        assert stall_torque >= effort, (
            f'{joint}: the drive delivers {stall_torque:.1f} Nm at its current limit '
            f'but the joint is rated {effort:.1f} Nm')
        # A drive with no headroom at all cannot accelerate the joint under load.
        assert stall_torque <= effort * 3.0, (
            f'{joint}: the drive is oversized for the configured effort limit')


def test_gear_ratios_are_plausible_for_an_industrial_arm(robot):
    hardware = load('hardware.yaml', package='robot_arm_hardware')['robot_arm_hardware']
    for joint in JOINTS:
        drive = hardware['joints'][joint]
        assert 10.0 <= drive['gear_ratio'] <= 400.0, f'{joint}: implausible gear ratio'
        # The wrist axes are faster and more lightly geared than the main axes.
    assert (hardware['joints']['joint_6']['gear_ratio']
            < hardware['joints']['joint_1']['gear_ratio'])
    assert (robot['joints']['joint_6']['velocity']
            > robot['joints']['joint_1']['velocity'])


def test_control_timeouts_are_compatible_with_the_control_rate(controllers):
    hardware = load('hardware.yaml', package='robot_arm_hardware')['robot_arm_hardware']
    period = 1.0 / controllers['controller_manager']['ros__parameters']['update_rate']
    # The bus must answer well inside one control period.
    assert hardware['transport']['read_timeout_ms'] / 1000.0 < period
    assert hardware['transport']['write_timeout_ms'] / 1000.0 < period
    # The watchdogs must be longer than a control period, or they would trip
    # on the very first cycle.
    assert hardware['control']['command_timeout'] > period
    assert hardware['control']['comm_timeout'] > period
