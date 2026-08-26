# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Robot-description tests.

They expand the Xacro model in every backend mode and check the resulting
URDF the way `check_urdf` would, plus the project specific invariants:
canonical joint names, a valid single-root TF tree, complete joint limits and
the correct ros2_control plugin per mode.
"""

import os
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
LINK_NAMES = ['base_link', 'link_1', 'link_2', 'link_3', 'link_4', 'link_5', 'link_6']

PKG_SHARE = None


def _share(package):
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(package)


def _xacro_path():
    global PKG_SHARE
    if PKG_SHARE is None:
        try:
            PKG_SHARE = _share('robot_arm_description')
        except Exception:  # not installed yet: fall back to the source tree
            PKG_SHARE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(PKG_SHARE, 'urdf', 'robot_arm.urdf.xacro')


def expand(**mappings):
    """Expand the Xacro model and return the parsed URDF root element."""
    mappings = {k: str(v).lower() if isinstance(v, bool) else str(v)
                for k, v in mappings.items()}
    doc = xacro.process_file(_xacro_path(), mappings=mappings)
    return ET.fromstring(doc.toprettyxml(indent='  '))


def robot_config():
    path = os.path.join(os.path.dirname(os.path.dirname(_xacro_path())),
                        'config', 'robot.yaml')
    with open(path) as handle:
        return yaml.safe_load(handle)


# ---------------------------------------------------------------------------
# Structure
# ---------------------------------------------------------------------------

@pytest.fixture(scope='module')
def urdf():
    return expand(hardware_type='mock')


def test_model_expands(urdf):
    assert urdf.tag == 'robot'
    assert urdf.get('name') == 'robot_arm'


def test_all_links_present(urdf):
    links = {link.get('name') for link in urdf.findall('link')}
    for name in LINK_NAMES + ['tool0']:
        assert name in links, f'missing link {name}'


def test_all_joints_present_and_revolute(urdf):
    joints = {j.get('name'): j for j in urdf.findall('joint')}
    for name in JOINT_NAMES:
        assert name in joints, f'missing joint {name}'
        assert joints[name].get('type') == 'revolute'


def test_joint_limits_complete_and_match_config(urdf):
    cfg = robot_config()['joints']
    joints = {j.get('name'): j for j in urdf.findall('joint')}
    for name in JOINT_NAMES:
        limit = joints[name].find('limit')
        assert limit is not None, f'{name} has no <limit>'
        assert float(limit.get('lower')) == pytest.approx(cfg[name]['lower'])
        assert float(limit.get('upper')) == pytest.approx(cfg[name]['upper'])
        assert float(limit.get('velocity')) == pytest.approx(cfg[name]['velocity'])
        assert float(limit.get('effort')) == pytest.approx(cfg[name]['effort'])
        assert float(limit.get('lower')) < float(limit.get('upper'))


def test_joint_axes_and_dynamics(urdf):
    cfg = robot_config()['joints']
    joints = {j.get('name'): j for j in urdf.findall('joint')}
    for name in JOINT_NAMES:
        axis = joints[name].find('axis')
        assert axis is not None, f'{name} has no <axis>'
        expected = ' '.join(str(v) for v in cfg[name]['axis'])
        assert axis.get('xyz').replace('.0', '') == expected.replace('.0', '')
        dynamics = joints[name].find('dynamics')
        assert dynamics is not None, f'{name} has no <dynamics>'
        assert float(dynamics.get('damping')) >= 0.0
        assert float(dynamics.get('friction')) >= 0.0


def test_tf_tree_is_a_single_rooted_tree(urdf):
    links = {link.get('name') for link in urdf.findall('link')}
    parent_of = {}
    for joint in urdf.findall('joint'):
        child = joint.find('child').get('link')
        parent = joint.find('parent').get('link')
        assert parent in links and child in links
        assert child not in parent_of, f'{child} has more than one parent joint'
        parent_of[child] = parent

    roots = [name for name in links if name not in parent_of]
    assert roots == ['base_link'], f'expected base_link as root, got {roots}'

    # every link reaches the root in a finite number of steps -> no cycles
    for link in links:
        hops, node = 0, link
        while node in parent_of:
            node = parent_of[node]
            hops += 1
            assert hops < len(links), f'cycle detected while walking up from {link}'
        assert node == 'base_link'


def test_kinematic_chain_order(urdf):
    parent_of = {j.find('child').get('link'): (j.get('name'), j.find('parent').get('link'))
                 for j in urdf.findall('joint')}
    chain, node = [], 'tool0'
    while node in parent_of:
        joint, node = parent_of[node]
        chain.append(joint)
    chain.reverse()
    assert chain == JOINT_NAMES + ['joint_tool0']


def test_links_have_visual_collision_and_inertial(urdf):
    for link in urdf.findall('link'):
        if link.get('name') not in LINK_NAMES:
            continue  # tool0 / tool_tip are massless frames on purpose
        assert link.find('visual') is not None
        assert link.find('collision') is not None
        inertial = link.find('inertial')
        assert inertial is not None
        assert float(inertial.find('mass').get('value')) > 0.0
        inertia = inertial.find('inertia')
        for axis in ('ixx', 'iyy', 'izz'):
            assert float(inertia.get(axis)) > 0.0


def test_collision_geometry_is_primitive(urdf):
    """Primitive collision geometry keeps Gazebo and MoveIt fast."""
    for link in urdf.findall('link'):
        for collision in link.findall('collision'):
            geometry = collision.find('geometry')
            assert geometry.find('mesh') is None
            assert (geometry.find('cylinder') is not None
                    or geometry.find('box') is not None
                    or geometry.find('sphere') is not None)


def test_world_frame_is_optional(urdf):
    assert 'world' not in {link.get('name') for link in urdf.findall('link')}
    with_world = expand(hardware_type='gazebo', use_world_frame=True)
    links = {link.get('name') for link in with_world.findall('link')}
    assert 'world' in links
    world_joint = [j for j in with_world.findall('joint') if j.get('name') == 'world_joint']
    assert len(world_joint) == 1
    assert world_joint[0].get('type') == 'fixed'


def test_gripper_mount_is_optional(urdf):
    links = {link.get('name') for link in urdf.findall('link')}
    assert 'gripper_mount_link' in links
    without = expand(hardware_type='mock', use_gripper_mount=False)
    links = {link.get('name') for link in without.findall('link')}
    assert 'gripper_mount_link' not in links
    assert 'tool0' in links


# ---------------------------------------------------------------------------
# ros2_control
# ---------------------------------------------------------------------------

def _ros2_control(urdf):
    blocks = urdf.findall('ros2_control')
    assert len(blocks) == 1
    return blocks[0]


def test_ros2_control_interfaces(urdf):
    block = _ros2_control(urdf)
    joints = {j.get('name'): j for j in block.findall('joint')}
    assert sorted(joints) == JOINT_NAMES
    for joint in joints.values():
        commands = {c.get('name') for c in joint.findall('command_interface')}
        states = {s.get('name') for s in joint.findall('state_interface')}
        assert 'position' in commands
        assert 'velocity' in commands
        assert {'position', 'velocity', 'effort'} <= states


def test_mock_backend_plugin(urdf):
    plugin = _ros2_control(urdf).find('hardware/plugin').text
    assert plugin == 'mock_components/GenericSystem'


def test_gazebo_backend_plugin():
    urdf = expand(hardware_type='gazebo', sim_engine='classic')
    plugin = _ros2_control(urdf).find('hardware/plugin').text
    assert plugin == 'gazebo_ros2_control/GazeboSystem'
    # the simulator plugin must be present exactly once
    plugins = [p.get('name') for g in urdf.findall('gazebo') for p in g.findall('plugin')]
    assert plugins.count('gazebo_ros2_control') == 1


def test_real_backend_plugin_and_hardware_parameters():
    try:
        _share('robot_arm_hardware')
    except Exception:
        pytest.skip('robot_arm_hardware is not installed')

    urdf = expand(hardware_type='real')
    block = _ros2_control(urdf)
    assert block.find('hardware/plugin').text == 'robot_arm_hardware/RobotArmSystemHardware'

    params = {p.get('name'): p.text for p in block.findall('hardware/param')}
    for key in ('transport_type', 'protocol_type', 'command_timeout', 'comm_timeout'):
        assert key in params, f'hardware parameter {key} is missing'

    for joint in block.findall('joint'):
        jparams = {p.get('name'): p.text for p in joint.findall('param')}
        for key in ('motor_id', 'encoder_resolution', 'gear_ratio', 'direction',
                    'zero_offset', 'min_position', 'max_position'):
            assert key in jparams, f'{joint.get("name")}: missing {key}'
        assert int(jparams['encoder_resolution']) > 0
        assert float(jparams['gear_ratio']) != 0.0
        assert int(jparams['direction']) in (-1, 1)


def test_simulation_and_real_expose_identical_joint_interfaces():
    """The whole architecture rests on this: the controllers cannot tell the
    difference between the backends."""
    try:
        _share('robot_arm_hardware')
    except Exception:
        pytest.skip('robot_arm_hardware is not installed')

    def signature(mode):
        block = _ros2_control(expand(hardware_type=mode))
        return {
            joint.get('name'): (
                sorted(c.get('name') for c in joint.findall('command_interface')),
                sorted(s.get('name') for s in joint.findall('state_interface')),
            )
            for joint in block.findall('joint')
        }

    assert signature('gazebo') == signature('real') == signature('mock')


# ---------------------------------------------------------------------------
# Cross-check with urdfdom (the same parser check_urdf uses), when available
# ---------------------------------------------------------------------------

def test_urdfdom_parses_the_model(urdf):
    urdf_parser = pytest.importorskip('urdf_parser_py.urdf')
    doc = xacro.process_file(_xacro_path(), mappings={'hardware_type': 'mock'})
    model = urdf_parser.URDF.from_xml_string(doc.toxml())
    assert model.name == 'robot_arm'
    assert model.get_root() == 'base_link'
    chain = model.get_chain('base_link', 'tool0', links=False, fixed=False)
    assert chain == JOINT_NAMES
