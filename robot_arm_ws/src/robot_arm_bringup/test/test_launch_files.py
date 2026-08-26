# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Structural checks on the launch files.

They load every launch description and verify the promises the documentation
makes about them - that the arguments exist, that sim and real differ only in
the backend, and that nothing starts two RViz instances or two safety monitors.
Fast, and no robot involved.
"""

import importlib.util
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
LAUNCH_DIR = os.path.join(os.path.dirname(HERE), 'launch')

LAUNCH_FILES = [
    'bringup.launch.py',
    'sim.launch.py',
    'real.launch.py',
    'real_robot.launch.py',
    'moveit.launch.py',
    'rviz.launch.py',
]


def load(name):
    pytest.importorskip('launch')
    pytest.importorskip('launch_ros')
    spec = importlib.util.spec_from_file_location(
        name.replace('.', '_'), os.path.join(LAUNCH_DIR, name))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def source(name):
    with open(os.path.join(LAUNCH_DIR, name)) as handle:
        return handle.read()


def declared_arguments(description):
    import launch
    return {
        action.name for action in description.entities
        if isinstance(action, launch.actions.DeclareLaunchArgument)
    }


@pytest.mark.parametrize('name', LAUNCH_FILES)
def test_every_launch_file_loads(name):
    module = load(name)
    assert hasattr(module, 'generate_launch_description')
    assert module.generate_launch_description() is not None


def test_bringup_offers_the_documented_switches():
    arguments = declared_arguments(load('bringup.launch.py').generate_launch_description())
    for expected in ('use_sim', 'use_rviz', 'use_moveit', 'hardware_interface',
                     'use_safety_monitor', 'planning_pipelines'):
        assert expected in arguments, f'bringup.launch.py is missing {expected}'


@pytest.mark.parametrize('name,use_sim', [('sim.launch.py', 'true'), ('real.launch.py', 'false')])
def test_sim_and_real_only_flip_use_sim(name, use_sim):
    text = source(name)
    assert f"'use_sim': '{use_sim}'" in text, \
        f'{name} must delegate to bringup.launch.py with use_sim={use_sim}'
    assert 'bringup.launch.py' in text


@pytest.mark.parametrize('name', ['sim.launch.py', 'real.launch.py'])
def test_sim_and_real_forward_the_common_arguments(name):
    module = load(name)
    for expected in ('use_rviz', 'use_moveit', 'hardware_interface', 'prefix'):
        assert expected in module.FORWARDED_ARGUMENTS, \
            f'{name} does not forward {expected} to bringup.launch.py'


def test_included_stacks_do_not_start_a_second_rviz_or_safety_monitor():
    """bringup owns RViz and the safety monitor; the backends must not add
    their own, or the user gets two windows and two e-stop owners."""
    text = source('bringup.launch.py')
    assert "'use_rviz': 'false'" in text
    assert "'use_safety_monitor': 'false'" in text


def test_real_robot_starts_the_control_stack():
    text = source('real_robot.launch.py')
    for expected in ('ros2_control_node', 'joint_state_broadcaster', 'robot_state_publisher',
                     'static_transform_publisher', 'safety_monitor'):
        assert expected in text, f'real_robot.launch.py no longer starts {expected}'
    # The planning frame must stay base_link on the real robot too.
    assert 'use_world_frame:=false' in text


def test_real_robot_defaults_to_the_real_backend():
    description = load('real_robot.launch.py').generate_launch_description()
    import launch
    defaults = {
        action.name: action.default_value[0].perform(_context())
        for action in description.entities
        if isinstance(action, launch.actions.DeclareLaunchArgument)
        and action.name == 'hardware_type'
    }
    assert defaults['hardware_type'] == 'real'


def _context():
    from launch import LaunchContext
    return LaunchContext()


def test_rviz_launch_can_run_with_and_without_moveit():
    arguments = declared_arguments(load('rviz.launch.py').generate_launch_description())
    assert 'use_moveit' in arguments
    assert 'rviz_config' in arguments


def test_moveit_launch_takes_the_backend_and_the_clock():
    arguments = declared_arguments(load('moveit.launch.py').generate_launch_description())
    assert 'hardware_type' in arguments
    assert 'use_sim_time' in arguments
    assert 'planning_pipelines' in arguments
