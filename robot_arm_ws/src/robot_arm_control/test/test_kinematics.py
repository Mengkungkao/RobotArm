# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Kinematics and statics of the arm as described by the URDF.

These tests check that the model is a coherent machine rather than a plausible
looking pile of numbers: that the Jacobian really differentiates the pose it
claims to, that the reach is the one on the datasheet, that the wrist is
spherical, and that the drives can hold the arm and its rated payload
anywhere in the workspace inside the effort limits the description advertises.

No ROS, no simulator, no solver - just the URDF and arithmetic.
"""

import math
import os
import random
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_arm_control.kinematics import ArmModel, GRAVITY    # noqa: E402

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
PAYLOAD = 5.0          # kg, the machine's rated payload
STEP = 1e-6            # central-difference step for the Jacobian check


@pytest.fixture(scope='module')
def arm():
    xacro = pytest.importorskip('xacro')
    here = os.path.dirname(os.path.abspath(__file__))
    source = os.path.dirname(os.path.dirname(here))
    model = os.path.join(source, 'robot_arm_description', 'urdf', 'robot_arm.urdf.xacro')
    if not os.path.exists(model):
        pytest.skip('robot_arm_description is not available')
    try:
        document = xacro.process_file(model, mappings={'hardware_type': 'mock'})
    except Exception as error:      # needs an installed workspace for $(find ...)
        pytest.skip(f'cannot expand the description here: {error}')
    return ArmModel.from_string(document.toxml())


def sample(arm, count, seed=0):
    random.seed(seed)
    limits = arm.limits()
    return [[random.uniform(*limits[name]) for name in arm.joint_names] for _ in range(count)]


def position(arm, q):
    pose = arm.tool_pose(q)
    return [pose[i][3] for i in range(3)]


# ---------------------------------------------------------------------------
# Forward kinematics
# ---------------------------------------------------------------------------

def test_the_model_is_the_expected_arm(arm):
    assert arm.joint_names == JOINTS
    assert arm.root == 'base_link'
    assert arm.tip == 'tool0'


def test_pose_at_zero_is_the_sum_of_the_link_offsets(arm):
    """Straight up: the tool sits at the summed link lengths, displaced by the
    cranked elbow."""
    x, y, z = position(arm, [0.0] * 6)
    assert x == pytest.approx(0.042, abs=1e-9), 'the elbow offset is missing'
    assert y == pytest.approx(0.0, abs=1e-9)
    assert z == pytest.approx(1.380, abs=1e-9)


def test_orientation_at_zero_is_the_identity(arm):
    pose = arm.tool_pose([0.0] * 6)
    for row in range(3):
        for column in range(3):
            assert pose[row][column] == pytest.approx(1.0 if row == column else 0.0, abs=1e-12)


def test_base_rotation_sweeps_the_tool_about_the_vertical(arm):
    """joint_1 may change where the tool is, never how far away it is."""
    reference = position(arm, [0.0, 0.6, -0.9, 0.0, 0.9, 0.0])
    radius = math.hypot(reference[0], reference[1])
    for angle in (0.5, 1.7, -2.4, 3.0):
        x, y, z = position(arm, [angle, 0.6, -0.9, 0.0, 0.9, 0.0])
        assert math.hypot(x, y) == pytest.approx(radius, abs=1e-9)
        assert z == pytest.approx(reference[2], abs=1e-9)


def test_reach_is_that_of_the_intended_machine(arm):
    """Furthest the tool can get from the axis-2 height, over the workspace."""
    shoulder_height = 0.399
    furthest = 0.0
    for q in sample(arm, 4000, seed=3):
        x, y, z = position(arm, q)
        furthest = max(furthest, math.sqrt(x * x + y * y + (z - shoulder_height) ** 2))
    assert 0.94 < furthest < 1.02, f'tool reach {furthest:.3f} m is not IRB-1200 class'


def test_wrist_is_spherical(arm):
    """Turning joints 4 and 6 must not move the wrist centre - that is what
    makes the position and orientation sub-problems separable for IK."""
    base = [0.3, 0.5, -0.7, 0.0, 0.8, 0.0]
    centre = [arm.link_poses(base)['link_5'][i][3] for i in range(3)]
    for fourth in (-1.2, 0.0, 2.5):
        for sixth in (-3.0, 0.0, 1.1):
            q = list(base)
            q[3], q[5] = fourth, sixth
            moved = [arm.link_poses(q)['link_5'][i][3] for i in range(3)]
            assert math.dist(centre, moved) < 1e-9, 'the wrist centre moved'


# ---------------------------------------------------------------------------
# The Jacobian
# ---------------------------------------------------------------------------

def test_linear_jacobian_differentiates_the_tool_position(arm):
    """J[0:3] against a central difference of the pose it claims to
    differentiate.  A wrong axis, frame or lever arm shows up here."""
    for q in sample(arm, 25, seed=1):
        jacobian = arm.jacobian(q)
        for index in range(len(q)):
            forward, backward = list(q), list(q)
            forward[index] += STEP
            backward[index] -= STEP
            ahead, behind = position(arm, forward), position(arm, backward)
            for row in range(3):
                numeric = (ahead[row] - behind[row]) / (2 * STEP)
                assert jacobian[row][index] == pytest.approx(numeric, abs=1e-5)


def test_angular_jacobian_matches_the_rotation_it_produces(arm):
    """The angular rows must reproduce the rotation vector of R(q+h) R(q-h)T."""
    for q in sample(arm, 15, seed=2):
        jacobian = arm.jacobian(q)
        for index in range(len(q)):
            forward, backward = list(q), list(q)
            forward[index] += STEP
            backward[index] -= STEP
            ahead = arm.tool_pose(forward)
            behind = arm.tool_pose(backward)
            # R_ahead * R_behind^T is a small rotation; its skew part is 2h*omega.
            delta = [[sum(ahead[i][k] * behind[j][k] for k in range(3))
                      for j in range(3)] for i in range(3)]
            omega = [(delta[2][1] - delta[1][2]) / (4 * STEP),
                     (delta[0][2] - delta[2][0]) / (4 * STEP),
                     (delta[1][0] - delta[0][1]) / (4 * STEP)]
            for row in range(3):
                assert jacobian[3 + row][index] == pytest.approx(omega[row], abs=1e-4)


def test_jacobian_has_a_column_per_joint(arm):
    jacobian = arm.jacobian([0.1] * 6)
    assert len(jacobian) == 6
    assert all(len(row) == len(JOINTS) for row in jacobian)


# ---------------------------------------------------------------------------
# Statics: weights, levers and what the drives have to hold
# ---------------------------------------------------------------------------

def test_total_mass_is_that_of_the_machine(arm):
    assert arm.total_mass == pytest.approx(51.9, abs=0.5)


def test_gravity_loads_no_joint_whose_axis_is_vertical(arm):
    """joint_1 turns about gravity, so it can never be loaded by it, whatever
    the arm is holding."""
    for q in sample(arm, 200, seed=4):
        torque = arm.gravity_torque(q, payload=PAYLOAD)
        assert abs(torque[0]) < 1e-9


def test_a_balanced_arm_needs_almost_no_torque(arm):
    """Straight up, only the cranked elbow's offset mass pulls: a couple of Nm,
    not a hundred."""
    torque = arm.gravity_torque([0.0] * 6)
    assert abs(torque[1]) < 5.0
    assert abs(torque[2]) < 5.0


def test_holding_the_arm_out_sideways_costs_what_the_lever_says(arm):
    """Shoulder horizontal: torque must be about (mass beyond the shoulder) x
    (lever) x g, which is the arithmetic anybody would do by hand."""
    q = [0.0, math.pi / 2, 0.0, 0.0, 0.0, 0.0]
    torque = arm.gravity_torque(q)
    beyond_shoulder = arm.total_mass - sum(
        arm.links[name].mass for name in ('base_link', 'link_1', 'motor_1', 'motor_2'))
    poses = arm.link_poses(q)
    lever = abs(poses['tool0'][0][3] - poses['link_2'][0][3])
    plausible = beyond_shoulder * GRAVITY * lever
    assert 0.3 * plausible < abs(torque[1]) < plausible, (
        f'shoulder torque {abs(torque[1]):.1f} Nm is not in the range a '
        f'{beyond_shoulder:.1f} kg arm on a {lever:.2f} m lever implies')


def test_payload_increases_the_shoulder_torque_by_its_own_moment(arm):
    q = [0.0, math.pi / 2, 0.0, 0.0, 0.0, 0.0]
    poses = arm.link_poses(q)
    lever = abs(poses['tool0'][0][3] - poses['link_2'][0][3])
    empty = arm.gravity_torque(q)[1]
    loaded = arm.gravity_torque(q, payload=PAYLOAD)[1]
    assert abs(loaded - empty) == pytest.approx(PAYLOAD * GRAVITY * lever, rel=1e-6)


def test_the_drives_can_hold_the_rated_payload_anywhere(arm):
    """The effort limits, the masses and the dimensions have to describe one
    machine.  If a joint cannot hold its own arm plus the rated payload
    standing still, the model is not a robot that could be built."""
    efforts = arm.effort_limits()
    worst = {name: 0.0 for name in arm.joint_names}
    for q in sample(arm, 3000, seed=5):
        for name, torque in zip(arm.joint_names, arm.gravity_torque(q, payload=PAYLOAD)):
            worst[name] = max(worst[name], abs(torque))

    for name in arm.joint_names:
        assert worst[name] <= efforts[name], (
            f'{name} needs {worst[name]:.1f} Nm to hold the arm still but is '
            f'rated {efforts[name]:.1f} Nm')
        # A drive with no margin left for acceleration is not usable either.
        assert worst[name] <= 0.75 * efforts[name], (
            f'{name} spends {100 * worst[name] / efforts[name]:.0f}% of its effort '
            f'limit just holding position, leaving nothing to move with')
