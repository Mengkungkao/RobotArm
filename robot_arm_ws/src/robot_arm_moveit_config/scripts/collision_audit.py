#!/usr/bin/env python3
# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Audit the SRDF collision matrix against the actual geometry.

    xacro urdf/robot_arm.urdf.xacro > /tmp/arm.urdf
    ros2 run robot_arm_moveit_config collision_audit.py \
        /tmp/arm.urdf config/robot_arm.srdf

Run this after ANY change to the link geometry.  A collision matrix is written
for one particular robot: a pair that could never touch on the old arm may
interpenetrate on the new one, and a disabled pair is simply not checked - the
planner will happily drive the arm through itself and nothing will complain.

METHOD AND ITS LIMIT
====================
Each collision primitive is wrapped in a capsule (segment + radius), the
configuration space is sampled uniformly within the joint limits, and the
closest approach of every non-adjacent pair is reported.

A capsule contains the primitive it wraps, so a POSITIVE clearance is a sound
proof that the pair never touches, and such a pair is safe to disable.  A
NEGATIVE clearance is not proof of the opposite: the hemispherical end caps
bulge a full radius past the flat face of a cylinder, which makes short
coaxial links - a flange sitting above a wrist housing - look like they
overlap when they are comfortably apart.  Treat negatives as "look at this
pair", not as a verdict; a deep negative, far beyond the radii involved, is a
real collision.
"""
import math
import random
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, '/home/user/RobotArm/robot_arm_ws/src/robot_arm_description/scripts')
from urdf_preview import apply, axis_angle, mul, parse_origin  # noqa: E402


def seg_seg_distance(p1, q1, p2, q2):
    """Shortest distance between two line segments (Ericson, RTCD 5.1.9)."""
    d1 = [q1[i] - p1[i] for i in range(3)]
    d2 = [q2[i] - p2[i] for i in range(3)]
    r = [p1[i] - p2[i] for i in range(3)]
    a = sum(x * x for x in d1)
    e = sum(x * x for x in d2)
    f = sum(d2[i] * r[i] for i in range(3))
    eps = 1e-12

    if a <= eps and e <= eps:
        return math.dist(p1, p2)
    if a <= eps:
        s, t = 0.0, min(1.0, max(0.0, f / e))
    else:
        c = sum(d1[i] * r[i] for i in range(3))
        if e <= eps:
            t, s = 0.0, min(1.0, max(0.0, -c / a))
        else:
            b = sum(d1[i] * d2[i] for i in range(3))
            denom = a * e - b * b
            s = min(1.0, max(0.0, (b * f - c * e) / denom)) if denom > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, min(1.0, max(0.0, -c / a))
            elif t > 1.0:
                t, s = 1.0, min(1.0, max(0.0, (b - c) / a))

    c1 = [p1[i] + d1[i] * s for i in range(3)]
    c2 = [p2[i] + d2[i] * t for i in range(3)]
    return math.dist(c1, c2)


def link_spheres(link):
    """Bounding capsules of a link's collision primitives, in link frame.

    A capsule (segment + radius) contains the primitive it wraps, so a
    positive clearance is still a sound "these never touch" - but it is far
    tighter than a bounding sphere for the long, thin shapes an arm is made
    of, where a sphere around a 391 mm forearm inevitably swallows the wrist.
    """
    out = []
    for col in link.findall('collision'):
        xyz, R = parse_origin(col)
        geo = col.find('geometry')
        if geo.find('box') is not None:
            half = [float(v) / 2 for v in geo.find('box').get('size').split()]
            axis = half.index(max(half))
            others = [half[i] for i in range(3) if i != axis]
            radius = math.hypot(*others)
            reach = max(0.0, half[axis] - 0.0)
            local = [0.0, 0.0, 0.0]
            local[axis] = reach
        elif geo.find('cylinder') is not None:
            c = geo.find('cylinder')
            radius = float(c.get('radius'))
            local = [0.0, 0.0, float(c.get('length')) / 2]
        else:
            continue
        # endpoints of the capsule segment, in link frame
        a = [sum(R[i][k] * local[k] for k in range(3)) + xyz[i] for i in range(3)]
        b = [-sum(R[i][k] * local[k] for k in range(3)) + xyz[i] for i in range(3)]
        out.append((a, b, radius))
    return out


def main(urdf_path, srdf_path, samples=20000, margin=0.02):
    root = ET.parse(urdf_path).getroot()
    links = {link.get('name'): link for link in root.findall('link')}
    joints = root.findall('joint')
    children = {}
    for j in joints:
        children.setdefault(j.find('parent').get('link'), []).append(j)
    parented = {j.find('child').get('link') for j in joints}
    base = [n for n in links if n not in parented][0]

    revolute = [j for j in joints if j.get('type') == 'revolute']
    limits = {j.get('name'): (float(j.find('limit').get('lower')),
                              float(j.find('limit').get('upper'))) for j in revolute}

    spheres = {n: link_spheres(link) for n, link in links.items()}
    checked = [n for n in links if spheres[n]]

    adjacent = set()
    for j in joints:
        adjacent.add(frozenset((j.find('parent').get('link'), j.find('child').get('link'))))

    closest = {}
    random.seed(0)
    for _ in range(samples):
        angles = {n: random.uniform(lo, hi) for n, (lo, hi) in limits.items()}
        poses = {base: ([0.0, 0.0, 0.0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]])}
        stack = [base]
        while stack:
            parent = stack.pop()
            pt, pR = poses[parent]
            for j in children.get(parent, []):
                xyz, R = parse_origin(j)
                t = apply(pR, pt, xyz)
                M = mul(pR, R)
                if j.get('type') == 'revolute':
                    a = [float(v) for v in j.find('axis').get('xyz').split()]
                    M = mul(M, axis_angle(a, angles[j.get('name')]))
                poses[j.find('child').get('link')] = (t, M)
                stack.append(j.find('child').get('link'))

        world = {}
        for n in checked:
            t, R = poses[n]
            world[n] = [(apply(R, t, a), apply(R, t, b), r) for a, b, r in spheres[n]]

        for i, a in enumerate(checked):
            for b in checked[i + 1:]:
                if frozenset((a, b)) in adjacent:
                    continue
                best = min(
                    seg_seg_distance(a1, a2, b1, b2) - ra - rb
                    for a1, a2, ra in world[a] for b1, b2, rb in world[b])
                key = frozenset((a, b))
                if key not in closest or best < closest[key]:
                    closest[key] = best

    srdf = ET.parse(srdf_path).getroot()
    disabled = {frozenset((d.get('link1'), d.get('link2'))): d.get('reason')
                for d in srdf.findall('disable_collisions')}

    print(f'{samples} random configurations, margin {margin * 1000:.0f} mm\n')
    unsafe, redundant = [], []
    for pair, gap in sorted(closest.items(), key=lambda kv: kv[1]):
        a, b = sorted(pair)
        state = disabled.get(pair)
        if state is not None and gap < margin:
            unsafe.append((a, b, gap, state))
        elif state is None and gap > 0.10:
            redundant.append((a, b, gap))

    print('DISABLED IN SRDF BUT THE LINKS CAN APPROACH:')
    for a, b, gap, reason in unsafe:
        print(f'  {a:<12} {b:<12} closest {gap * 1000:8.1f} mm   (reason="{reason}")')
    print('   none' if not unsafe else '')
    print('\nCHECKED BUT NEVER CLOSER THAN 100 mm (could be disabled):')
    for a, b, gap in redundant:
        print(f'  {a:<12} {b:<12} closest {gap * 1000:8.1f} mm')
    print('   none' if not redundant else '')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
