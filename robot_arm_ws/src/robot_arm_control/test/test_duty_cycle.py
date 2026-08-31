# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Motion profiles and what a duty cycle costs the machine.

Every other check in this project asks whether a drive can produce a torque at
all.  These ask the question that decides whether a machine survives a shift:
what it has to produce *on average*, over a real motion, and whether that
stays under the current it can hold indefinitely rather than the peak it can
hold for a second.
"""

import importlib.util
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_arm_control.kinematics import ArmModel, trapezoidal_profile    # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.dirname(HERE)
SOURCE = os.path.dirname(PACKAGE)


def load_analyser():
    path = os.path.join(PACKAGE, 'scripts', 'analyse_duty_cycle.py')
    if not os.path.exists(path):
        pytest.skip('analyse_duty_cycle.py is not available')
    spec = importlib.util.spec_from_file_location('analyse_duty_cycle', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def arm():
    xacro = pytest.importorskip('xacro')
    model = os.path.join(SOURCE, 'robot_arm_description', 'urdf', 'robot_arm.urdf.xacro')
    hardware = os.path.join(SOURCE, 'robot_arm_hardware', 'config', 'hardware.yaml')
    if not (os.path.exists(model) and os.path.exists(hardware)):
        pytest.skip('sibling packages are not available')
    try:
        document = xacro.process_file(model, mappings={'hardware_type': 'mock'})
    except Exception as error:
        pytest.skip(f'cannot expand the description here: {error}')
    built = ArmModel.from_string(document.toxml())
    built.load_drivetrain(hardware)
    return built


@pytest.fixture(scope='module')
def limits():
    import yaml
    path = os.path.join(SOURCE, 'robot_arm_moveit_config', 'config', 'joint_limits.yaml')
    if not os.path.exists(path):
        pytest.skip('robot_arm_moveit_config is not available')
    with open(path) as handle:
        return yaml.safe_load(handle)['joint_limits']


# ---------------------------------------------------------------------------
# The motion profile
# ---------------------------------------------------------------------------

def test_profile_starts_and_ends_exactly_where_asked():
    start = [0.0, -0.4, 0.9, 0.0, 1.0, 0.0]
    end = [0.8, 0.5, -0.6, 0.3, 0.9, -0.4]
    profile = trapezoidal_profile(start, end, [1.0] * 6, [2.0] * 6)
    for actual, wanted in zip(profile[0][1], start):
        assert actual == pytest.approx(wanted, abs=1e-9)
    for actual, wanted in zip(profile[-1][1], end):
        assert actual == pytest.approx(wanted, abs=1e-9)


def test_profile_respects_every_joint_limit():
    """The path parameter is bounded by the tightest joint, so no joint may
    exceed its own velocity or acceleration."""
    start = [0.0] * 6
    end = [1.5, -1.2, 0.9, 2.0, -0.8, 3.0]
    velocity = [1.5, 1.0, 2.0, 3.0, 2.5, 4.0]
    acceleration = [5.0, 4.0, 6.0, 8.0, 7.0, 9.0]
    profile = trapezoidal_profile(start, end, velocity, acceleration)
    for _, _, qd, qdd in profile:
        for index in range(6):
            assert abs(qd[index]) <= velocity[index] * (1 + 1e-9)
            assert abs(qdd[index]) <= acceleration[index] * (1 + 1e-9)


def test_joints_start_and_finish_together():
    """A synchronised profile: no joint arrives early and waits."""
    profile = trapezoidal_profile([0.0] * 3, [2.0, 0.1, -1.0],
                                  [1.0] * 3, [2.0] * 3)
    fractions = []
    midpoint = profile[len(profile) // 2]
    for index, (start, end) in enumerate(zip(profile[0][1], profile[-1][1])):
        if abs(end - start) > 1e-9:
            fractions.append((midpoint[1][index] - start) / (end - start))
    assert max(fractions) - min(fractions) < 1e-9


def test_short_moves_never_reach_the_speed_limit():
    """A triangular profile: too short to finish accelerating."""
    profile = trapezoidal_profile([0.0], [0.01], [5.0], [1.0])
    assert max(abs(qd[0]) for _, _, qd, _ in profile) < 5.0


def test_a_move_of_nothing_is_a_single_sample():
    profile = trapezoidal_profile([0.3, 0.2], [0.3, 0.2], [1.0, 1.0], [2.0, 2.0])
    assert len(profile) == 1
    assert profile[0][2] == [0.0, 0.0]


# ---------------------------------------------------------------------------
# What the cycle costs
# ---------------------------------------------------------------------------

@pytest.fixture(scope='module')
def cycle(arm, limits):
    analyser = load_analyser()
    rows = analyser.run_cycle(arm, analyser.DEFAULT_CYCLE, 1.0, 1.0, 5.0, limits,
                              timestep=0.01)
    duration, report = analyser.summarise(arm, rows, 0.01)
    return duration, report


def test_the_rated_cycle_is_thermally_sustainable(cycle, arm):
    """The machine must be able to run its rated payload all day.

    Peak torque says a move is possible; RMS against the continuous rating
    says the arm can keep doing it.  An axis can pass every peak check in this
    project and still burn out, and this is the check that catches it.
    """
    _, report = cycle
    for name in arm.joint_names:
        entry = report[name]
        assert entry['rms_torque'] <= entry['continuous_rating'], (
            f'{name} needs {entry["rms_torque"]:.1f} Nm RMS over the cycle but can '
            f'only hold {entry["continuous_rating"]:.1f} Nm continuously')


def test_peak_torque_stays_within_what_the_drive_delivers(cycle, arm):
    _, report = cycle
    for name in arm.joint_names:
        entry = report[name]
        assert entry['peak_torque'] <= entry['peak_rating'], (
            f'{name} peaks at {entry["peak_torque"]:.1f} Nm against a '
            f'{entry["peak_rating"]:.1f} Nm drive')


def test_rms_never_exceeds_peak(cycle, arm):
    _, report = cycle
    for name in arm.joint_names:
        assert report[name]['rms_torque'] <= report[name]['peak_torque'] + 1e-9


def test_the_shoulder_is_the_hardest_worked_axis(cycle, arm):
    """It carries the whole arm on the longest lever; if some wrist axis is
    working harder, the mass distribution has gone wrong somewhere."""
    _, report = cycle
    load = {name: report[name]['rms_torque'] / report[name]['continuous_rating']
            for name in arm.joint_names}
    assert max(load, key=load.get) == 'joint_2'


def test_running_faster_costs_more(arm, limits):
    """Energy and thermal load must rise with speed.  If they do not, the
    dynamics are not seeing the acceleration."""
    analyser = load_analyser()
    results = {}
    for scale in (0.3, 1.0):
        rows = analyser.run_cycle(arm, analyser.DEFAULT_CYCLE, scale, scale, 5.0,
                                  limits, timestep=0.01)
        _, report = analyser.summarise(arm, rows, 0.01)
        results[scale] = report['joint_2']

    assert results[1.0]['peak_torque'] > results[0.3]['peak_torque']
    assert results[1.0]['rms_torque'] > results[0.3]['rms_torque']


def test_carrying_a_payload_costs_more_than_carrying_none(arm, limits):
    analyser = load_analyser()
    loaded = analyser.summarise(
        arm, analyser.run_cycle(arm, analyser.DEFAULT_CYCLE, 1.0, 1.0, 5.0,
                                limits, timestep=0.01), 0.01)[1]
    empty = analyser.summarise(
        arm, analyser.run_cycle(arm, analyser.DEFAULT_CYCLE, 1.0, 1.0, 0.0,
                                limits, timestep=0.01), 0.01)[1]
    assert loaded['joint_2']['rms_torque'] > empty['joint_2']['rms_torque']


def test_current_follows_torque_through_the_gearbox(arm):
    """Current is torque divided by kt, the ratio and the efficiency.  Getting
    the efficiency on the wrong side understates the current by 1/eta."""
    for name in arm.joint_names:
        drive = arm.drives[name]
        torque = drive.deliverable_torque()
        assert drive.current_for(torque) == pytest.approx(drive.max_current, rel=1e-9)
        assert math.isclose(
            drive.deliverable_torque(),
            drive.torque_constant * drive.gear_ratio * drive.max_current * drive.efficiency)
