#!/usr/bin/env python3
# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Compute each link's real inertia tensor from the shape it is actually built of.

    xacro urdf/robot_arm.urdf.xacro > /tmp/arm.urdf
    ros2 run robot_arm_description compute_inertia.py /tmp/arm.urdf \\
        config/robot.yaml > config/inertia.yaml

WHY
===
A URDF link carries one inertial, so a link assembled from several primitives
is easy to fake with a single equivalent solid.  That is what this model used
to do, and it is wrong in ways a physics engine notices: link_3 is a forearm
tube offset 42 mm in X plus a shaft lying across Y, but a centred cylinder
gives it a symmetric tensor with every product of inertia zero.  The simulated
arm then swings about axes the real one does not, and the torques the
controller has to fight are not the torques the machine produces.

METHOD
======
Each link's visual primitives are treated as one rigid body of uniform
density.  The density is derived from the link's target mass in robot.yaml
divided by the total volume of its parts, so the masses stay the ones that
were chosen for the machine, while the distribution comes from the geometry.

For every part: the inertia of the primitive about its own centre, rotated
into the link frame (R I Rt), then shifted to the link's combined centre of
mass by the parallel axis theorem, m (|d|^2 E - d dt).  The sum is the link's
tensor about its centre of mass, products of inertia included.

Parts that overlap - a boss sunk into a beam - are counted twice in the
volume, which lowers the derived density a little.  The mass stays exact
because it is an input; only the distribution is marginally affected.
"""

import math
import sys
import xml.etree.ElementTree as ET

try:
    import yaml
except ImportError:                                          # pragma: no cover
    sys.exit('compute_inertia needs PyYAML')


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def transpose(m):
    return [[m[j][i] for j in range(3)] for i in range(3)]


def parse_origin(element):
    origin = element.find('origin') if element is not None else None
    if origin is None:
        return [0.0, 0.0, 0.0], rpy_to_matrix(0, 0, 0)
    xyz = [float(v) for v in (origin.get('xyz') or '0 0 0').split()]
    rpy = [float(v) for v in (origin.get('rpy') or '0 0 0').split()]
    return xyz, rpy_to_matrix(*rpy)


def primitive(geometry):
    """(volume, local inertia per unit mass) for a supported primitive."""
    box = geometry.find('box')
    if box is not None:
        x, y, z = [float(v) for v in box.get('size').split()]
        volume = x * y * z
        unit = [(y * y + z * z) / 12.0, (x * x + z * z) / 12.0, (x * x + y * y) / 12.0]
        return volume, unit

    cylinder = geometry.find('cylinder')
    if cylinder is not None:
        radius, length = float(cylinder.get('radius')), float(cylinder.get('length'))
        volume = math.pi * radius * radius * length
        across = (3.0 * radius * radius + length * length) / 12.0
        return volume, [across, across, radius * radius / 2.0]

    sphere = geometry.find('sphere')
    if sphere is not None:
        radius = float(sphere.get('radius'))
        unit = 2.0 * radius * radius / 5.0
        return (4.0 / 3.0) * math.pi * radius ** 3, [unit, unit, unit]

    return None, None


def link_inertia(link, target_mass):
    """Mass, centre of mass and inertia tensor of a link, from its visuals."""
    parts = []
    for visual in link.findall('visual'):
        xyz, rotation = parse_origin(visual)
        volume, unit = primitive(visual.find('geometry'))
        if volume is None or volume <= 0.0:
            continue
        parts.append((xyz, rotation, volume, unit))

    total_volume = sum(p[2] for p in parts)
    if not parts or total_volume <= 0.0:
        return None
    density = target_mass / total_volume

    centre = [sum(p[0][i] * p[2] * density for p in parts) / target_mass for i in range(3)]

    tensor = [[0.0] * 3 for _ in range(3)]
    for xyz, rotation, volume, unit in parts:
        mass = density * volume
        local = [[unit[0] * mass, 0.0, 0.0],
                 [0.0, unit[1] * mass, 0.0],
                 [0.0, 0.0, unit[2] * mass]]
        rotated = matmul(matmul(rotation, local), transpose(rotation))

        offset = [xyz[i] - centre[i] for i in range(3)]
        squared = sum(v * v for v in offset)
        for i in range(3):
            for j in range(3):
                shift = mass * ((squared if i == j else 0.0) - offset[i] * offset[j])
                tensor[i][j] += rotated[i][j] + shift

    return target_mass, centre, tensor


def target_masses(config):
    """Target mass of every link, from robot.yaml."""
    masses = dict(config['mass'])
    for joint, motor in config['motors'].items():
        if isinstance(motor, dict) and 'mass' in motor:
            masses[f"motor_{motor['motor_id']}"] = motor['mass']
    masses['gripper_mount_link'] = config['gripper_mount']['plate_mass']
    return masses


def main(urdf_path, config_path):
    root = ET.parse(urdf_path).getroot()
    with open(config_path) as handle:
        config = yaml.safe_load(handle)
    masses = target_masses(config)

    print('# ==========================================================================')
    print('#  GENERATED by robot_arm_description/scripts/compute_inertia.py')
    print('#  Do not edit by hand: re-run the script after changing geometry or mass.')
    print('#')
    print('#  Each tensor is taken about the link\'s centre of mass, in the link frame,')
    print('#  by summing the actual primitives with the parallel axis theorem.  A test')
    print('#  regenerates this file and fails if it no longer matches the geometry.')
    print('# ==========================================================================')
    print('inertial:')

    total = 0.0
    for link in root.findall('link'):
        name = link.get('name')
        if name not in masses:
            continue
        result = link_inertia(link, masses[name])
        if result is None:
            continue
        mass, centre, tensor = result
        total += mass
        # Rotating a diagonal tensor by exactly 90 degrees leaves ~1e-17
        # residue in the off-diagonals; that is float noise, not physics.
        scale = max(abs(tensor[i][i]) for i in range(3))
        for i in range(3):
            for j in range(3):
                if abs(tensor[i][j]) < 1e-9 * scale:
                    tensor[i][j] = 0.0
        print(f'  {name}:')
        print(f'    mass: {mass:.6g}')
        print('    com: [{:.6f}, {:.6f}, {:.6f}]'.format(*centre))
        print('    inertia:')
        for key, value in (('ixx', tensor[0][0]), ('ixy', tensor[0][1]), ('ixz', tensor[0][2]),
                           ('iyy', tensor[1][1]), ('iyz', tensor[1][2]), ('izz', tensor[2][2])):
            print(f'      {key}: {value:.9g}')
    print(f'\n# total mass: {total:.3f} kg')


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit('usage: compute_inertia.py <expanded.urdf> <robot.yaml>')
    main(sys.argv[1], sys.argv[2])
