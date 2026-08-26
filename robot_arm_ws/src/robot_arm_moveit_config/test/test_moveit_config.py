# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
MoveIt configuration tests.

A MoveIt configuration fails in ways that are hard to debug at runtime: a group
whose tip link does not exist, a named pose that misses a joint, a controller
mapping that does not match the controller that is actually running.  These
checks catch all of that at build time, without starting move_group.
"""

import os
import xml.etree.ElementTree as ET

import pytest
import yaml

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.dirname(HERE)
CONFIG = os.path.join(PACKAGE, 'config')
SRC = os.path.dirname(PACKAGE)


def load_yaml(path):
    with open(path) as handle:
        return yaml.safe_load(handle)


@pytest.fixture(scope='module')
def srdf():
    return ET.parse(os.path.join(CONFIG, 'robot_arm.srdf')).getroot()


@pytest.fixture(scope='module')
def robot_config():
    path = os.path.join(SRC, 'robot_arm_description', 'config', 'robot.yaml')
    if not os.path.exists(path):
        pytest.skip('robot_arm_description is not available')
    return load_yaml(path)


# ---------------------------------------------------------------------------
# SRDF
# ---------------------------------------------------------------------------

def test_srdf_names_match_the_urdf(srdf):
    assert srdf.get('name') == 'robot_arm'


def test_planning_group_spans_base_link_to_tool0(srdf):
    groups = {group.get('name'): group for group in srdf.findall('group')}
    assert 'arm' in groups, 'the planning group must be called "arm"'
    chain = groups['arm'].find('chain')
    assert chain is not None
    assert chain.get('base_link') == 'base_link'
    assert chain.get('tip_link') == 'tool0'


def test_named_poses_cover_every_joint(srdf):
    states = srdf.findall('group_state')
    assert states, 'at least one named pose is expected'
    for state in states:
        assert state.get('group') == 'arm'
        named = [joint.get('name') for joint in state.findall('joint')]
        assert named == JOINTS, f'named pose "{state.get("name")}" is incomplete'


def test_named_poses_are_inside_the_joint_limits(srdf, robot_config):
    limits = robot_config['joints']
    for state in srdf.findall('group_state'):
        for joint in state.findall('joint'):
            value = float(joint.get('value'))
            limit = limits[joint.get('name')]
            assert limit['lower'] <= value <= limit['upper'], \
                f'{state.get("name")}/{joint.get("name")} = {value} is outside its limits'


def test_home_pose_matches_the_simulation_start_pose(srdf, robot_config):
    """The `home` pose and the initial simulation state must agree, otherwise
    the arm jumps the first time it is asked to go home."""
    path = os.path.join(SRC, 'robot_arm_description', 'config', 'initial_positions.yaml')
    if not os.path.exists(path):
        pytest.skip('initial_positions.yaml is not available')
    initial = load_yaml(path)['initial_positions']

    home = next(s for s in srdf.findall('group_state') if s.get('name') == 'home')
    for joint in home.findall('joint'):
        assert float(joint.get('value')) == pytest.approx(initial[joint.get('name')]), \
            f'{joint.get("name")}: the home pose and initial_positions.yaml disagree'


def test_adjacent_links_have_collision_checking_disabled(srdf):
    disabled = {
        frozenset((entry.get('link1'), entry.get('link2')))
        for entry in srdf.findall('disable_collisions')
    }
    adjacent = [
        ('base_link', 'link_1'), ('link_1', 'link_2'), ('link_2', 'link_3'),
        ('link_3', 'link_4'), ('link_4', 'link_5'), ('link_5', 'link_6'),
    ]
    for pair in adjacent:
        assert frozenset(pair) in disabled, f'{pair} should not be collision checked'


def test_distant_links_are_still_collision_checked(srdf):
    """Self-collision checking is the point; over-disabling silently breaks it."""
    disabled = {
        frozenset((entry.get('link1'), entry.get('link2')))
        for entry in srdf.findall('disable_collisions')
    }
    for pair in [('base_link', 'link_6'), ('link_1', 'link_6'), ('link_2', 'link_5')]:
        assert frozenset(pair) not in disabled, f'{pair} must stay collision checked'


# ---------------------------------------------------------------------------
# kinematics / limits / controllers
# ---------------------------------------------------------------------------

def test_kinematics_solver_is_configured_for_the_arm_group():
    kinematics = load_yaml(os.path.join(CONFIG, 'kinematics.yaml'))
    assert 'arm' in kinematics
    solver = kinematics['arm']
    assert solver['kinematics_solver']
    assert solver['kinematics_solver_timeout'] > 0.0
    assert solver['kinematics_solver_search_resolution'] > 0.0


def test_joint_limits_cover_every_joint_and_match_the_urdf(robot_config):
    limits = load_yaml(os.path.join(CONFIG, 'joint_limits.yaml'))
    assert 0.0 < limits['default_velocity_scaling_factor'] <= 1.0
    assert 0.0 < limits['default_acceleration_scaling_factor'] <= 1.0

    for joint in JOINTS:
        entry = limits['joint_limits'][joint]
        assert entry['has_velocity_limits'] is True
        assert entry['max_velocity'] == pytest.approx(robot_config['joints'][joint]['velocity']), \
            f'{joint}: MoveIt and the URDF disagree about the velocity limit'
        assert entry['has_acceleration_limits'] is True
        assert entry['max_acceleration'] > 0.0


def test_moveit_executes_through_the_ros2_control_arm_controller():
    controllers = load_yaml(os.path.join(CONFIG, 'moveit_controllers.yaml'))
    manager = controllers['moveit_simple_controller_manager']
    assert manager['controller_names'] == ['arm_controller']

    arm = manager['arm_controller']
    assert arm['type'] == 'FollowJointTrajectory'
    assert arm['action_ns'] == 'follow_joint_trajectory'
    assert arm['joints'] == JOINTS

    # The name must match the controller in robot_arm_control/config/controllers.yaml,
    # otherwise planning works and execution silently does nothing.
    path = os.path.join(SRC, 'robot_arm_control', 'config', 'controllers.yaml')
    if os.path.exists(path):
        ros2_control = load_yaml(path)
        assert 'arm_controller' in ros2_control['controller_manager']['ros__parameters']
        assert ros2_control['arm_controller']['ros__parameters']['joints'] == JOINTS


def test_ompl_pipeline_is_usable():
    ompl = load_yaml(os.path.join(CONFIG, 'ompl_planning.yaml'))
    assert ompl['planning_plugin'] == 'ompl_interface/OMPLPlanner'
    assert 'RRTConnect' in ompl['planner_configs']
    assert 'RRTConnect' in ompl['arm']['planner_configs']
    assert ompl['arm']['projection_evaluator'].startswith('joints(')


def test_setup_assistant_points_at_the_shared_description():
    config = load_yaml(os.path.join(PACKAGE, '.setup_assistant'))
    urdf = config['moveit_setup_assistant_config']['urdf']
    assert urdf['package'] == 'robot_arm_description'
    assert urdf['relative_path'].endswith('robot_arm.urdf.xacro')
    # The planning frame must be base_link, so the world anchor stays out.
    assert 'use_world_frame:=false' in urdf['xacro_args']
    srdf = config['moveit_setup_assistant_config']['srdf']
    assert srdf['relative_path'] == 'config/robot_arm.srdf'


def test_rviz_configuration_loads_the_motion_planning_panel():
    config = load_yaml(os.path.join(CONFIG, 'moveit.rviz'))
    displays = config['Visualization Manager']['Displays']
    classes = [display['Class'] for display in displays]
    assert 'moveit_rviz_plugin/MotionPlanning' in classes
    assert 'moveit_rviz_plugin/Trajectory' in classes
    assert 'rviz_default_plugins/RobotModel' in classes
    assert 'rviz_default_plugins/TF' in classes
    assert 'rviz_default_plugins/InteractiveMarkers' in classes

    planning = next(d for d in displays if d['Class'] == 'moveit_rviz_plugin/MotionPlanning')
    assert planning['Planning Request']['Planning Group'] == 'arm'
