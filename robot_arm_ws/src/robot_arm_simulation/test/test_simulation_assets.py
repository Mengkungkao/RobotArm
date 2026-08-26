# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Simulation asset tests.

They check the parts of the simulation that can break silently: an unparsable
world, physics settings that cannot keep up with the control loop, or a launch
file that no longer offers the arguments the rest of the project passes to it.
"""

import os
import xml.etree.ElementTree as ET

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.dirname(HERE)
WORLD = os.path.join(PACKAGE, 'worlds', 'robot_arm.world')
LAUNCH = os.path.join(PACKAGE, 'launch', 'simulation.launch.py')

CONTROL_RATE = 100.0     # Hz, must match controllers.yaml


@pytest.fixture(scope='module')
def world():
    tree = ET.parse(WORLD)
    return tree.getroot()


def test_world_is_valid_sdf(world):
    assert world.tag == 'sdf'
    assert world.get('version') is not None
    assert world.find('world') is not None


def test_world_has_gravity_ground_and_light(world):
    element = world.find('world')
    gravity = element.find('gravity').text.split()
    assert float(gravity[2]) == pytest.approx(-9.81)

    includes = [item.find('uri').text for item in element.findall('include')]
    assert 'model://ground_plane' in includes
    assert 'model://sun' in includes


def test_physics_step_is_fine_enough_for_the_control_loop(world):
    physics = world.find('world').find('physics')
    step = float(physics.find('max_step_size').text)
    update_rate = float(physics.find('real_time_update_rate').text)

    control_period = 1.0 / CONTROL_RATE
    assert step <= control_period / 5.0, \
        'the physics step must be several times smaller than the control period'
    assert update_rate >= CONTROL_RATE, \
        'the physics update rate must be at least the control rate'
    assert float(physics.find('ode').find('solver').find('iters').text) >= 50, \
        'a geared arm needs a well converged solver to stay stable'


def test_launch_file_offers_the_documented_arguments():
    launch = pytest.importorskip('launch')
    pytest.importorskip('launch_ros')
    import importlib.util

    spec = importlib.util.spec_from_file_location('simulation_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    description = module.generate_launch_description()
    names = {
        action.name for action in description.entities
        if isinstance(action, launch.actions.DeclareLaunchArgument)
    }
    for expected in ('use_rviz', 'gui', 'world', 'sim_engine', 'prefix', 'controllers_file'):
        assert expected in names, f'the simulation launch file lost the {expected} argument'


def test_launch_file_uses_the_gazebo_backend_and_a_world_anchor():
    """The two things that make the simulation behave like the real robot."""
    with open(LAUNCH) as handle:
        source = handle.read()
    assert 'hardware_type:=gazebo' in source
    assert 'use_world_frame:=true' in source
    assert 'joint_state_broadcaster' in source
    assert 'arm_controller' in source
