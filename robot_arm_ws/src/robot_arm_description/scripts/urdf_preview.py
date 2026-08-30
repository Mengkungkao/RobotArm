#!/usr/bin/env python3
# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Render the robot model to a PNG without RViz, Gazebo or a display.

    xacro urdf/robot_arm.urdf.xacro > /tmp/arm.urdf
    ros2 run robot_arm_description urdf_preview.py /tmp/arm.urdf /tmp/arm.png
    ros2 run robot_arm_description urdf_preview.py /tmp/arm.urdf /tmp/arm.png \
        0,40,-60,0,45,0 yz

Arguments: <urdf> <output.png> [joint angles in degrees, comma separated]
[view: xz | yz | xy].

It computes forward kinematics itself and projects every visual primitive
onto the chosen plane, so it needs nothing but Pillow.  That makes it usable
in a headless CI container - to eyeball a geometry change, or to diff a
rendering against a stored one and catch a model regression that no numeric
test would notice.

It is a previewer, not a renderer: shapes are drawn as flat silhouettes with
painter ordering, and meshes are ignored.
"""
import math
import sys
import xml.etree.ElementTree as ET

try:
    from PIL import Image, ImageDraw
except ImportError:                                          # pragma: no cover
    sys.exit('urdf_preview needs Pillow:  python3 -m pip install pillow')


def rpy_to_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ]


def mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def apply(M, t, v):
    return [sum(M[i][k] * v[k] for k in range(3)) + t[i] for i in range(3)]


def axis_angle(axis, angle):
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z) or 1.0
    x, y, z = x / n, y / n, z / n
    c, s, C = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return [[x * x * C + c, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, y * y * C + c, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, z * z * C + c]]


def parse_origin(el):
    o = el.find('origin') if el is not None else None
    if o is None:
        return [0.0, 0.0, 0.0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    xyz = [float(v) for v in (o.get('xyz') or '0 0 0').split()]
    rpy = [float(v) for v in (o.get('rpy') or '0 0 0').split()]
    return xyz, rpy_to_matrix(*rpy)


def hull(points):
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2:
                (x1, y1), (x2, y2) = out[-2], out[-1]
                if (x2 - x1) * (p[1] - y1) - (y2 - y1) * (p[0] - x1) > 0:
                    break
                out.pop()
            out.append(p)
        return out[:-1]
    return half(pts) + half(reversed(pts))


def main(urdf_path, out_path, joints_deg, view='xz'):
    """Render `urdf_path` at the given joint angles [deg] into `out_path`."""
    root = ET.parse(urdf_path).getroot()
    colours = {}
    for m in root.findall('material'):
        c = m.find('color')
        if c is not None:
            r, g, b, _ = [float(v) for v in c.get('rgba').split()]
            colours[m.get('name')] = (int(r * 255), int(g * 255), int(b * 255))

    links = {link.get('name'): link for link in root.findall('link')}
    joints = root.findall('joint')
    children = {}
    for j in joints:
        children.setdefault(j.find('parent').get('link'), []).append(j)
    parented = {j.find('child').get('link') for j in joints}
    base = [n for n in links if n not in parented][0]

    angles = dict(zip(['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
                      [math.radians(a) for a in joints_deg]))

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
                M = mul(M, axis_angle(a, angles.get(j.get('name'), 0.0)))
            child = j.find('child').get('link')
            poses[child] = (t, M)
            stack.append(child)

    shapes = []
    for name, link in links.items():
        if name not in poses:
            continue
        lt, lR = poses[name]
        for vis in link.findall('visual'):
            xyz, R = parse_origin(vis)
            t = apply(lR, lt, xyz)
            M = mul(lR, R)
            mat = vis.find('material')
            colour = colours.get(mat.get('name') if mat is not None else '', (150, 150, 150))
            geo = vis.find('geometry')
            pts = []
            if geo.find('box') is not None:
                sx, sy, sz = [float(v) / 2 for v in geo.find('box').get('size').split()]
                pts = [apply(M, t, [sx * i, sy * k, sz * m])
                       for i in (-1, 1) for k in (-1, 1) for m in (-1, 1)]
            elif geo.find('cylinder') is not None:
                cyl = geo.find('cylinder')
                r, h = float(cyl.get('radius')), float(cyl.get('length')) / 2
                for sign in (-1, 1):
                    for i in range(24):
                        a = 2 * math.pi * i / 24
                        pts.append(apply(M, t, [r * math.cos(a), r * math.sin(a), sign * h]))
            if pts:
                shapes.append((colour, pts, name))

    ax = {'xz': (0, 2), 'yz': (1, 2), 'xy': (0, 1)}[view]
    proj = [[(p[ax[0]], p[ax[1]]) for p in pts] for _, pts, _ in shapes]
    flat = [p for poly in proj for p in poly]
    minx, maxx = min(p[0] for p in flat), max(p[0] for p in flat)
    miny, maxy = min(p[1] for p in flat), max(p[1] for p in flat)

    W, H, pad = 700, 900, 60
    sx = (W - 2 * pad) / max(maxx - minx, 1e-6)
    sy = (H - 2 * pad) / max(maxy - miny, 1e-6)
    s = min(sx, sy)
    ox = pad + ((W - 2 * pad) - (maxx - minx) * s) / 2
    oy = pad + ((H - 2 * pad) - (maxy - miny) * s) / 2

    def to_px(p):
        return (ox + (p[0] - minx) * s, H - (oy + (p[1] - miny) * s))

    img = Image.new('RGB', (W, H), (245, 246, 248))
    d = ImageDraw.Draw(img)
    # floor line
    d.line([to_px((minx, 0.0)), to_px((maxx, 0.0))], fill=(180, 182, 186), width=2)
    # far-to-near: sort by depth along the projection normal
    depth_axis = {0, 1, 2} - set(ax)
    da = depth_axis.pop()

    def depth(index):
        pts = shapes[index][1]
        return sum(p[da] for p in pts) / len(pts)

    # Painter's algorithm: far primitives first, near ones over the top.
    order = sorted(range(len(shapes)), key=depth)
    for i in order:
        colour, pts, _ = shapes[i]
        poly = hull([(p[ax[0]], p[ax[1]]) for p in pts])
        if len(poly) >= 3:
            d.polygon([to_px(p) for p in poly], fill=colour, outline=(40, 40, 45))
    img.save(out_path)
    print(f'{out_path}  ({len(shapes)} primitives, view={view})')


if __name__ == '__main__':
    angles = [float(a) for a in sys.argv[3].split(',')] if len(sys.argv) > 3 else [0] * 6
    view = sys.argv[4] if len(sys.argv) > 4 else 'xz'
    main(sys.argv[1], sys.argv[2], angles, view)
