# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
CLI tests.

They exercise the parts that must be right before any robot is involved:
argument parsing, unit conversion and the "fill in the joints the user did not
type" rule.  A wrong sign or a missing degree conversion here would move a
real arm to the wrong place.
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_arm_tools import cli_common     # noqa: E402
from robot_arm_tools.cli_common import (JOINT_NAMES, EXIT_FAILURE, EXIT_INTERRUPTED,  # noqa: E402
                                        EXIT_OK, EXIT_USAGE, collect_joint_targets,
                                        format_angle, run)


def parse(module_name, argv):
    module = pytest.importorskip(f'robot_arm_tools.{module_name}')
    return module.build_parser().parse_args(argv)


# ---------------------------------------------------------------------------
# joint targets
# ---------------------------------------------------------------------------

def test_all_six_joints_from_the_command_line():
    arguments = parse('move_joint', [
        '--j1', '0', '--j2', '0.5', '--j3', '-0.8',
        '--j4', '0', '--j5', '0.5', '--j6', '0'])
    targets = collect_joint_targets(arguments)
    assert targets == pytest.approx([0.0, 0.5, -0.8, 0.0, 0.5, 0.0])
    assert len(targets) == len(JOINT_NAMES)


def test_omitted_joints_keep_their_current_value():
    arguments = parse('move_joint', ['--j3', '-0.8'])
    current = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    assert collect_joint_targets(arguments, current) == pytest.approx(
        [0.1, 0.2, -0.8, 0.4, 0.5, 0.6])


def test_degrees_are_converted_to_radians():
    arguments = parse('move_joint', ['--j2', '90', '--j3', '-45', '--degrees'])
    current = [0.0] * 6
    targets = collect_joint_targets(arguments, current)
    assert targets[1] == pytest.approx(math.pi / 2)
    assert targets[2] == pytest.approx(-math.pi / 4)


def test_an_unknown_joint_value_is_a_usage_error():
    arguments = parse('move_joint', ['--j1', '0.5'])
    with pytest.raises(ValueError):
        collect_joint_targets(arguments, None)          # nothing to fall back on
    with pytest.raises(ValueError):
        collect_joint_targets(arguments, [float('nan')] * 6)


# ---------------------------------------------------------------------------
# formatting
# ---------------------------------------------------------------------------

def test_angle_formatting_follows_the_requested_unit():
    assert format_angle(math.pi, degrees=True).strip() == '180.00'
    assert format_angle(math.pi, degrees=False).strip() == '3.1416'
    assert format_angle(float('nan'), degrees=False).strip() == 'n/a'


def test_angle_unit_label():
    assert cli_common.angle_unit(True) == 'deg'
    assert cli_common.angle_unit(False) == 'rad'


# ---------------------------------------------------------------------------
# pose arguments
# ---------------------------------------------------------------------------

def test_move_pose_requires_a_position():
    module = pytest.importorskip('robot_arm_tools.move_pose')
    with pytest.raises(SystemExit):
        module.build_parser().parse_args(['--x', '0.3'])     # y and z are missing


def test_move_pose_defaults_orientation_to_zero():
    arguments = parse('move_pose', ['--x', '0.35', '--y', '0.1', '--z', '0.4'])
    assert (arguments.roll, arguments.pitch, arguments.yaw) == (0.0, 0.0, 0.0)
    assert arguments.linear is False


def test_move_pose_accepts_the_documented_example():
    arguments = parse('move_pose', [
        '--x', '0.35', '--y', '0.10', '--z', '0.40',
        '--roll', '0', '--pitch', '1.57', '--yaw', '0'])
    assert arguments.x == pytest.approx(0.35)
    assert arguments.pitch == pytest.approx(1.57)


def test_ik_shares_the_pose_arguments():
    arguments = parse('ik', ['--x', '0.35', '--y', '0.1', '--z', '0.4', '--pitch', '1.57'])
    assert arguments.execute is False
    assert arguments.allow_collisions is False


def test_e_stop_requires_an_explicit_action():
    module = pytest.importorskip('robot_arm_tools.e_stop')
    with pytest.raises(SystemExit):
        module.build_parser().parse_args([])                 # neither engage nor release
    arguments = module.build_parser().parse_args(['--engage'])
    assert arguments.engage and not arguments.release


def test_every_tool_offers_the_common_connection_options():
    for name in ('move_joint', 'move_pose', 'fk', 'ik', 'status', 'stop',
                 'e_stop', 'calibrate_joints'):
        module = pytest.importorskip(f'robot_arm_tools.{name}')
        options = {action.dest for action in module.build_parser()._actions}
        for expected in ('group', 'base_frame', 'ee_frame', 'controller', 'timeout'):
            assert expected in options, f'{name} is missing --{expected}'


# ---------------------------------------------------------------------------
# exit codes
# ---------------------------------------------------------------------------

def test_exit_codes_are_distinct_and_scriptable():
    assert (EXIT_OK, EXIT_FAILURE, EXIT_USAGE, EXIT_INTERRUPTED) == (0, 1, 2, 130)


def test_run_maps_exceptions_to_exit_codes():
    assert run(lambda: EXIT_OK) == EXIT_OK

    def raises_value_error():
        raise ValueError('bad input')
    assert run(raises_value_error) == EXIT_USAGE

    def raises_runtime_error():
        raise RuntimeError('robot on fire')
    assert run(raises_runtime_error) == EXIT_FAILURE

    def interrupted():
        raise KeyboardInterrupt()
    assert run(interrupted) == EXIT_INTERRUPTED
