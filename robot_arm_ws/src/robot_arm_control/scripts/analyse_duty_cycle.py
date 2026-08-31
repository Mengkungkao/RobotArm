#!/usr/bin/env python3
# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Torque, current, power and thermal load over a real duty cycle.

    xacro urdf/robot_arm.urdf.xacro > /tmp/arm.urdf
    ros2 run robot_arm_control analyse_duty_cycle.py /tmp/arm.urdf \\
        --hardware $(ros2 pkg prefix --share robot_arm_hardware)/config/hardware.yaml \\
        --limits   $(ros2 pkg prefix --share robot_arm_moveit_config)/config/joint_limits.yaml \\
        --csv /tmp/cycle.csv

WHY PEAK IS NOT ENOUGH
======================
Everything else in this project checks peak torque: can the drive produce it
at all.  That is the wrong question for a machine that runs all day.  A servo
is rated for a peak it can hold for a second or two and a much lower current
it can hold forever, and what decides which one applies is the RMS over the
duty cycle.  An axis can pass every peak check and still cook itself.

So this runs an actual motion - a synchronised trapezoidal profile through a
sequence of poses, carrying the payload only on the legs where the arm is
holding something - integrates the dynamics along it, and reports peak against
the peak rating and RMS against the continuous rating.  The ratio of the two
is the honest measure of how hard the cycle works the machine.

Power is mechanical shaft power plus I^2 R copper loss.  Regeneration is not
credited back: a simple drive burns it in a resistor, so counting it would
flatter the energy figure.
"""

import argparse
import csv
import math
import sys

try:
    import yaml
except ImportError:                                          # pragma: no cover
    sys.exit('needs PyYAML')

try:
    from robot_arm_control.kinematics import ArmModel, trapezoidal_profile
except ImportError:                                          # pragma: no cover
    sys.exit('needs robot_arm_control on PYTHONPATH (source the workspace)')


# A pick-and-place cycle: reach down and pick, lift, swing across, place,
# return empty.  `payload` is what the arm is holding on the way TO that pose.
DEFAULT_CYCLE = [
    ('home',  [0.0, -0.4, 0.9, 0.0, 1.0, 0.0], 0.0, 0.0),
    ('pick',  [0.0, 0.9, -0.5, 0.0, 1.1, 0.0], 0.0, 0.3),
    ('lift',  [0.0, 0.4, -0.2, 0.0, 0.9, 0.0], 1.0, 0.0),
    ('place', [1.5, 0.9, -0.5, 0.0, 1.1, 0.0], 1.0, 0.3),
    ('clear', [1.5, 0.4, -0.2, 0.0, 0.9, 0.0], 0.0, 0.0),
    ('home',  [0.0, -0.4, 0.9, 0.0, 1.0, 0.0], 0.0, 0.2),
]


def run_cycle(arm, cycle, velocity_scale, acceleration_scale, payload, limits,
              timestep=0.005):
    """Walk the cycle and return one row of state and load per sample."""
    velocity = [limits[name]['max_velocity'] * velocity_scale for name in arm.joint_names]
    acceleration = [limits[name]['max_acceleration'] * acceleration_scale
                    for name in arm.joint_names]

    rows = []
    clock = 0.0
    for index in range(1, len(cycle)):
        label, target, carrying, dwell = cycle[index]
        previous = cycle[index - 1][1]
        load = payload * carrying

        for t, q, qd, qdd in trapezoidal_profile(previous, target, velocity,
                                                 acceleration, timestep):
            torque = arm.inverse_dynamics(q, qd, qdd, payload=load)
            rows.append(sample_row(arm, clock + t, label, q, qd, torque, load))
        clock += rows[-1]['t'] - clock if rows else 0.0

        # Dwell: still holding, still drawing current against gravity.
        steps = int(dwell / timestep)
        for step in range(steps):
            torque = arm.inverse_dynamics(target, payload=load)
            rows.append(sample_row(arm, clock + step * timestep, f'{label} (hold)',
                                   target, [0.0] * len(target), torque, load))
        clock += dwell
    return rows


def sample_row(arm, time, label, q, qd, torque, payload):
    row = {'t': time, 'segment': label, 'payload': payload}
    for index, name in enumerate(arm.joint_names):
        drive = arm.drives[name]
        current = drive.current_for(torque[index])
        mechanical = torque[index] * qd[index]
        # Losses take their cut on the way in; regeneration is not credited.
        shaft = mechanical / drive.efficiency if mechanical > 0 else mechanical * drive.efficiency
        copper = current * current * drive.winding_resistance
        row[name] = {
            'q': q[index], 'qd': qd[index], 'torque': torque[index],
            'current': current, 'mechanical': mechanical,
            'electrical': max(0.0, shaft + copper), 'copper': copper,
        }
    return row


def summarise(arm, rows, timestep):
    """Peak and RMS per joint, against the peak and continuous ratings."""
    duration = rows[-1]['t'] if rows else 0.0
    report = {}
    for name in arm.joint_names:
        drive = arm.drives[name]
        torques = [row[name]['torque'] for row in rows]
        currents = [row[name]['current'] for row in rows]
        electrical = [row[name]['electrical'] for row in rows]
        count = len(torques) or 1
        report[name] = {
            'peak_torque': max(abs(v) for v in torques),
            'rms_torque': math.sqrt(sum(v * v for v in torques) / count),
            'peak_current': max(currents),
            'rms_current': math.sqrt(sum(v * v for v in currents) / count),
            'peak_power': max(electrical),
            'mean_power': sum(electrical) / count,
            'energy': sum(electrical) * timestep,
            'peak_rating': drive.deliverable_torque(),
            'continuous_rating': drive.continuous_torque(),
            'peak_current_rating': drive.max_current,
            'continuous_current_rating': drive.continuous_current,
        }
    return duration, report


def main():
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('urdf')
    parser.add_argument('--hardware', required=True, help='hardware.yaml with the drive data')
    parser.add_argument('--limits', required=True, help='MoveIt joint_limits.yaml')
    parser.add_argument('--payload', type=float, default=5.0)
    parser.add_argument('--velocity-scale', type=float, default=1.0)
    parser.add_argument('--acceleration-scale', type=float, default=1.0)
    parser.add_argument('--timestep', type=float, default=0.005)
    parser.add_argument('--csv', default=None, help='write the full time series here')
    arguments = parser.parse_args()

    arm = ArmModel.from_urdf(arguments.urdf)
    arm.load_drivetrain(arguments.hardware)
    with open(arguments.limits) as handle:
        limits = yaml.safe_load(handle)['joint_limits']

    rows = run_cycle(arm, DEFAULT_CYCLE, arguments.velocity_scale,
                     arguments.acceleration_scale, arguments.payload, limits,
                     arguments.timestep)
    duration, report = summarise(arm, rows, arguments.timestep)

    print(f'pick-and-place cycle: {duration:.2f} s, {len(rows)} samples, '
          f'{arguments.payload:.1f} kg payload')
    print(f'speed {arguments.velocity_scale:.0%} of limit, '
          f'acceleration {arguments.acceleration_scale:.0%}\n')

    print(f"{'joint':<9}{'peak Nm':>9}{'of peak':>9}{'RMS Nm':>9}{'of cont':>9}"
          f"{'RMS A':>8}{'peak W':>9}{'Wh':>8}  verdict")
    total_energy = 0.0
    hottest = None
    for name in arm.joint_names:
        entry = report[name]
        peak_use = entry['peak_torque'] / entry['peak_rating']
        thermal = entry['rms_torque'] / entry['continuous_rating']
        total_energy += entry['energy']
        if hottest is None or thermal > report[hottest]['rms_torque'] / \
                report[hottest]['continuous_rating']:
            hottest = name
        verdict = 'ok' if thermal <= 1.0 else 'OVERHEATS'
        print(f"{name:<9}{entry['peak_torque']:9.1f}{peak_use:8.0%}"
              f"{entry['rms_torque']:9.1f}{thermal:8.0%}"
              f"{entry['rms_current']:8.1f}{entry['peak_power']:9.0f}"
              f"{entry['energy'] / 3600.0:8.3f}  {verdict}")

    thermal = report[hottest]['rms_torque'] / report[hottest]['continuous_rating']
    print(f'\ncycle energy {total_energy / 3600.0:.3f} Wh, '
          f'{3600.0 / duration:.0f} cycles/hour, '
          f'{total_energy / 3600.0 * 3600.0 / duration:.0f} W average')
    print(f'thermally hardest axis: {hottest} at {thermal:.0%} of its continuous rating')

    if arguments.csv:
        with open(arguments.csv, 'w', newline='') as handle:
            fields = ['t', 'segment', 'payload']
            for name in arm.joint_names:
                fields += [f'{name}_{key}' for key in
                           ('q', 'qd', 'torque', 'current', 'mechanical', 'electrical')]
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            for row in rows:
                flat = {'t': round(row['t'], 4), 'segment': row['segment'],
                        'payload': row['payload']}
                for name in arm.joint_names:
                    for key, value in row[name].items():
                        if key != 'copper':
                            flat[f'{name}_{key}'] = round(value, 6)
                writer.writerow(flat)
        print(f'time series written to {arguments.csv}')


if __name__ == '__main__':
    main()
