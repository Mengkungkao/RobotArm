#!/usr/bin/env python3
# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Derive joint acceleration limits the drives can actually deliver.

    xacro urdf/robot_arm.urdf.xacro > /tmp/arm.urdf
    ros2 run robot_arm_description solve_acceleration_limits.py /tmp/arm.urdf

URDF has no field for acceleration, so MoveIt's joint_limits.yaml is usually
filled in by feel.  Feel is wrong here: an acceleration limit is a claim about
torque, and whether the claim holds depends on the inertia the joint sees,
which changes across the workspace, plus gravity and the payload.

Advertise an acceleration the drive cannot produce and the planner will
happily emit trajectories the arm cannot follow: it lags, the trajectory
controller reports path tolerance violations, and the simulation stops
matching the machine.

This searches for the largest limits whose worst-case torque stays inside a
fraction of each drive's effort limit, with every joint accelerating at once
in the least favourable direction - a deliberately pessimistic envelope,
because the number ends up in a file the planner trusts.

    tau = M(q) qdd + C(q, qd) qd + g(q)

is evaluated exactly, by recursive Newton-Euler, over sampled states.
"""

import argparse
import random
import sys

try:
    from robot_arm_control.kinematics import ArmModel
except ImportError:                                          # pragma: no cover
    sys.exit('needs robot_arm_control on PYTHONPATH (source the workspace)')


def worst_case_torque(arm, accelerations, payload, samples, seed, velocity_fraction=0.5):
    """Peak |torque| per joint over sampled states at these accelerations.

    `velocity_fraction` is the share of each joint's maximum speed the arm may
    already be moving at while it accelerates.  It matters more than the
    acceleration does: at full speed on every axis at once, gravity and the
    centrifugal terms alone consume 79-85% of these drives, which would leave
    no torque for acceleration at all and drive the search to nonsense.  No
    industrial arm meets that condition either - datasheet speeds are per-axis
    maxima, not a simultaneous guarantee - and trajectory ramps happen away
    from top speed, on the way out of rest and into it.
    """
    limits = arm.limits()
    names = arm.joint_names
    random.seed(seed)
    worst = {name: 0.0 for name in names}

    for _ in range(samples):
        q = [random.uniform(*limits[name]) for name in names]
        qd = [random.uniform(-1.0, 1.0) * velocity_fraction * arm.joints[name].velocity
              for name in names]
        for direction in (1.0, -1.0):
            qdd = [direction * accelerations[name] for name in names]
            for name, torque in zip(names, arm.inverse_dynamics(q, qd, qdd, payload=payload)):
                worst[name] = max(worst[name], abs(torque))
    return worst


def solve(arm, payload, headroom, samples, seed, ramp_time=0.25,
          velocity_fraction=0.5, rounds=14):
    """Largest limits that stay inside the torque budget.

    Two things bound an acceleration.  A drive cannot ramp a joint from rest
    to full speed instantly whatever the torque, so the search starts from
    `ramp_time` - a quarter second to full speed, which is the order an
    industrial arm achieves - and only ever reduces from there.

    Reduction has to be iterative rather than one division per joint: the
    joints are coupled through M(q), so accelerating joint_6 shows up as
    torque at joint_2.  Scaling each joint by its own overshoot alone lets a
    nearly inertia-free wrist axis grow without bound while loading the
    shoulder it is bolted to.
    """
    efforts = arm.effort_limits()
    ceiling = {name: arm.joints[name].velocity / ramp_time for name in arm.joint_names}
    accelerations = dict(ceiling)

    # Relax towards the fixed point from both sides: a joint under budget may
    # grow back (up to the ramp ceiling), one over budget must shrink.  A
    # reduce-only search would keep whatever over-correction the first
    # pessimistic sample caused, and answer with an arm far slower than its
    # drives allow.  The square root damps the coupling between joints.
    for _ in range(rounds):
        worst = worst_case_torque(arm, accelerations, payload, samples, seed,
                                  velocity_fraction)
        for name in arm.joint_names:
            budget = headroom * efforts[name]
            if worst[name] < 1e-9:
                accelerations[name] = ceiling[name]
                continue
            scaled = accelerations[name] * (budget / worst[name]) ** 0.5
            accelerations[name] = min(scaled, ceiling[name])

    # Whatever the search converged to, it has to be inside the budget.
    for _ in range(6):
        worst = worst_case_torque(arm, accelerations, payload, samples, seed,
                                  velocity_fraction)
        if all(worst[name] <= headroom * efforts[name] + 1e-9
               for name in arm.joint_names):
            break
        for name in arm.joint_names:
            budget = headroom * efforts[name]
            if worst[name] > budget:
                accelerations[name] *= budget / worst[name]
    return accelerations


def main():
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('urdf')
    parser.add_argument('--payload', type=float, default=5.0,
                        help='rated payload carried at the tool [kg]')
    parser.add_argument('--headroom', type=float, default=0.9,
                        help='fraction of the effort limit the peak may use')
    parser.add_argument('--samples', type=int, default=400)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--velocity-fraction', type=float, default=0.5,
                        help='share of top speed the arm may already carry while '
                             'accelerating')
    parser.add_argument('--ramp-time', type=float, default=0.25,
                        help='seconds from rest to full joint speed, the ceiling '
                             'no amount of torque can beat')
    arguments = parser.parse_args()

    arm = ArmModel.from_urdf(arguments.urdf)
    accelerations = solve(arm, arguments.payload, arguments.headroom,
                          arguments.samples, arguments.seed, arguments.ramp_time,
                          arguments.velocity_fraction)
    worst = worst_case_torque(arm, accelerations, arguments.payload,
                              arguments.samples, arguments.seed,
                              arguments.velocity_fraction)

    # Separate property: can the arm even hold full speed on every axis while
    # coasting?  If not, the velocity limits themselves are the fiction.
    coasting = worst_case_torque(arm, {name: 0.0 for name in arm.joint_names},
                                 arguments.payload, arguments.samples,
                                 arguments.seed, velocity_fraction=1.0)
    efforts = arm.effort_limits()

    print(f'# payload {arguments.payload} kg, accelerating while already moving at '
          f'{arguments.velocity_fraction:.0%} of top speed,', file=sys.stderr)
    print(f'# peak torque held under {arguments.headroom:.0%} of each effort limit; '
          f'ramp ceiling {arguments.ramp_time} s to full speed', file=sys.stderr)
    print(f"# {'joint':<9}{'accel':>9}{'peak Nm':>10}{'effort':>9}{'used':>8}"
          f"{'coasting':>12}", file=sys.stderr)
    for name in arm.joint_names:
        used = 100.0 * worst[name] / efforts[name]
        coast = 100.0 * coasting[name] / efforts[name]
        print(f'# {name:<9}{accelerations[name]:9.2f}{worst[name]:10.1f}'
              f'{efforts[name]:9.1f}{used:7.1f}%{coast:11.1f}%', file=sys.stderr)

    for name in arm.joint_names:
        # Two significant figures: the inputs do not justify more.
        print(f'{name}: {round(accelerations[name], 2)}')


if __name__ == '__main__':
    main()
