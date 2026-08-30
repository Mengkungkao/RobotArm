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
MOTOR_LINKS = ['motor_1', 'motor_2', 'motor_3', 'motor_4', 'motor_5', 'motor_6']

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
# Industrial geometry: reach, the cranked elbow and the spherical wrist
# ---------------------------------------------------------------------------

def joint_origins(urdf):
    origins = {}
    for joint in urdf.findall('joint'):
        origin = joint.find('origin')
        xyz = (origin.get('xyz') if origin is not None else None) or '0 0 0'
        origins[joint.get('name')] = [float(v) for v in xyz.split()]
    return origins


def test_joint_origins_come_from_the_kinematics_block(urdf):
    """The chain is derived from `kinematics:`, so it can never drift out of
    sync with the geometry that is drawn."""
    kin = robot_config()['kinematics']
    origins = joint_origins(urdf)

    assert origins['joint_1'][2] == pytest.approx(kin['base_height'])
    assert origins['joint_2'][2] == pytest.approx(kin['shoulder_height'])
    assert origins['joint_3'][2] == pytest.approx(kin['upper_arm'])
    assert origins['joint_4'][2] == pytest.approx(kin['forearm'])
    assert origins['joint_5'][2] == pytest.approx(kin['wrist_housing'])
    assert origins['joint_6'][2] == pytest.approx(kin['wrist_offset'])
    assert origins['joint_tool0'][2] == pytest.approx(kin['flange'])


def test_elbow_is_cranked_forward(urdf):
    """The forearm is offset from the elbow axis - the signature of this class
    of industrial arm, and the thing a purely coaxial model gets wrong."""
    kin = robot_config()['kinematics']
    assert kin['elbow_offset_x'] > 0.0
    assert joint_origins(urdf)['joint_4'][0] == pytest.approx(kin['elbow_offset_x'])


def test_wrist_is_spherical(urdf):
    """Axes 4, 5 and 6 must intersect at one point - the origin of joint_5.

    That holds exactly when joint_5 and joint_6 sit on their parent's Z axis,
    and it is what keeps inverse kinematics well conditioned.
    """
    origins = joint_origins(urdf)
    for joint in ('joint_5', 'joint_6'):
        x, y, _ = origins[joint]
        assert x == pytest.approx(0.0), f'{joint} is off the wrist axis in X'
        assert y == pytest.approx(0.0), f'{joint} is off the wrist axis in Y'


def test_reach_is_that_of_the_intended_machine(urdf):
    kin = robot_config()['kinematics']
    reach = kin['upper_arm'] + kin['forearm'] + kin['wrist_housing']
    assert 0.85 < reach < 0.95, f'reach {reach:.3f} m is not an IRB-1200-class arm'
    assert kin['base_height'] + kin['shoulder_height'] == pytest.approx(0.399, abs=0.02)


def test_total_mass_is_realistic(urdf):
    """A model an order of magnitude too light will plan trajectories the real
    machine cannot follow."""
    total = sum(
        float(link.find('inertial').find('mass').get('value'))
        for link in urdf.findall('link') if link.find('inertial') is not None)
    assert 35.0 < total < 70.0, f'total mass {total:.1f} kg is not plausible'


# ---------------------------------------------------------------------------
# Drive units
# ---------------------------------------------------------------------------

def test_every_axis_has_a_drive_unit(urdf):
    links = {link.get('name') for link in urdf.findall('link')}
    for motor in MOTOR_LINKS:
        assert motor in links, f'missing {motor}'


def test_drive_units_are_attached_to_the_links_named_in_the_config(urdf):
    motors = robot_config()['motors']
    mounts = {
        j.get('name'): j.find('parent').get('link')
        for j in urdf.findall('joint') if j.get('name').endswith('_mount')
    }
    for index in range(1, 7):
        joint = f'joint_{index}'
        assert mounts[f'motor_{index}_mount'] == motors[joint]['parent']


def test_drive_units_carry_their_mass(urdf):
    """11.5 kg of drives, most of it high on the moving links: leaving it out
    makes a simulated arm accelerate in ways the real one cannot."""
    motors = robot_config()['motors']
    links = {link.get('name'): link for link in urdf.findall('link')}
    total = 0.0
    for index in range(1, 7):
        inertial = links[f'motor_{index}'].find('inertial')
        assert inertial is not None, f'motor_{index} has no mass'
        mass = float(inertial.find('mass').get('value'))
        assert mass == pytest.approx(motors[f'joint_{index}']['mass'])
        total += mass
    assert total > 5.0


def test_drive_units_have_no_collision_geometry_by_default(urdf):
    """They sit inside the arm's own envelope, so collision-checking them
    costs planning time without changing any result."""
    assert robot_config()['motors']['use_collision'] is False
    links = {link.get('name'): link for link in urdf.findall('link')}
    for index in range(1, 7):
        assert links[f'motor_{index}'].find('collision') is None


def test_drive_unit_ids_match_the_driver_configuration():
    """One set of six motors, described once for the model and once for the
    driver - the ids must agree."""
    motors = robot_config()['motors']
    try:
        share = _share('robot_arm_hardware')
    except Exception:
        pytest.skip('robot_arm_hardware is not installed')
    with open(os.path.join(share, 'config', 'hardware.yaml')) as handle:
        drives = yaml.safe_load(handle)['robot_arm_hardware']['joints']
    for joint in JOINT_NAMES:
        assert motors[joint]['motor_id'] == drives[joint]['motor_id']


def test_drive_units_can_be_turned_off():
    without = expand(hardware_type='mock', use_motors=False)
    links = {link.get('name') for link in without.findall('link')}
    assert not (links & set(MOTOR_LINKS))
    for name in LINK_NAMES:
        assert name in links


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
