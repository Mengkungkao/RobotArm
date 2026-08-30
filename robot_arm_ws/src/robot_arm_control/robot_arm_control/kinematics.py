# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Kinematics and static dynamics of the arm, straight from the URDF.

    from robot_arm_control.kinematics import ArmModel

    arm = ArmModel.from_urdf('/tmp/arm.urdf')
    pose = arm.tool_pose([0.0, 0.5, -0.8, 0.0, 0.5, 0.0])   # 4x4 homogeneous
    J = arm.jacobian(q)                                     # 6xN geometric
    tau = arm.gravity_torque(q, payload=5.0)                # Nm per joint

WHY IT EXISTS
=============
MoveIt and Gazebo each build their own kinematics from this URDF, and both are
heavy to start.  This module reads the same file with no ROS, no simulator and
no solver, which makes it possible to *check* the machine: that the Jacobian
agrees with the pose it claims to differentiate, that the reach is what the
datasheet says, and that the drives can actually hold the arm and its payload
inside the effort limits the description advertises.

Conventions: right-handed, metres, radians, Z up.  A pose is a 4x4 homogeneous
matrix as a list of rows.  The Jacobian is geometric, expressed in the base
frame, linear rows first then angular.

Pure Python on purpose - no numpy - so it runs anywhere the tests do.
"""

import math
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Sequence

__all__ = ['ArmModel', 'Joint', 'Link', 'GRAVITY']

GRAVITY = 9.80665


# ---------------------------------------------------------------------------
# Small matrix helpers.  A rotation is 3 rows of 3; a pose is 4 rows of 4.
# ---------------------------------------------------------------------------

def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> List[List[float]]:
    """Intrinsic X-Y-Z rotation, the convention URDF and tf2 both use."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def rotation_about_axis(axis: Sequence[float], angle: float) -> List[List[float]]:
    """Rodrigues' rotation about a unit axis."""
    norm = math.sqrt(sum(v * v for v in axis)) or 1.0
    x, y, z = (v / norm for v in axis)
    c, s, t = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]


def pose_from(rotation: List[List[float]], translation: Sequence[float]) -> List[List[float]]:
    return [list(rotation[i]) + [translation[i]] for i in range(3)] + [[0.0, 0.0, 0.0, 1.0]]


def compose(a: List[List[float]], b: List[List[float]]) -> List[List[float]]:
    """Matrix product of two 4x4 poses."""
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def translation_of(pose: List[List[float]]) -> List[float]:
    return [pose[i][3] for i in range(3)]


def axis_of(pose: List[List[float]], column: int) -> List[float]:
    return [pose[i][column] for i in range(3)]


def cross(a: Sequence[float], b: Sequence[float]) -> List[float]:
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def dot(a: Sequence[float], b: Sequence[float]) -> float:
    return sum(x * y for x, y in zip(a, b))


# ---------------------------------------------------------------------------
# The model
# ---------------------------------------------------------------------------

class Joint:
    """One URDF joint: its fixed offset, its axis and its limits."""

    def __init__(self, name, kind, parent, child, xyz, rpy, axis, limits):
        self.name = name
        self.kind = kind
        self.parent = parent
        self.child = child
        self.origin = pose_from(rotation_from_rpy(*rpy), xyz)
        self.axis = axis
        self.lower, self.upper, self.velocity, self.effort = limits

    @property
    def movable(self) -> bool:
        return self.kind in ('revolute', 'continuous', 'prismatic')

    def transform(self, value: float) -> List[List[float]]:
        """Fixed offset followed by the joint's own motion."""
        if self.kind == 'prismatic':
            shift = [self.axis[i] * value for i in range(3)]
            return compose(self.origin, pose_from(rotation_from_rpy(0, 0, 0), shift))
        if self.kind in ('revolute', 'continuous'):
            spin = rotation_about_axis(self.axis, value)
            return compose(self.origin, pose_from(spin, [0.0, 0.0, 0.0]))
        return self.origin


class Link:
    """One URDF link and its inertial properties."""

    def __init__(self, name, mass=0.0, com=(0.0, 0.0, 0.0)):
        self.name = name
        self.mass = mass
        self.com = list(com)


class ArmModel:
    """Forward kinematics, Jacobian and gravity torques for a serial arm."""

    def __init__(self, links: Dict[str, Link], joints: List[Joint], root: str,
                 tip: str = 'tool0'):
        self.links = links
        self.joints = {joint.name: joint for joint in joints}
        self.root = root
        self.tip = tip
        self._children: Dict[str, List[Joint]] = {}
        for joint in joints:
            self._children.setdefault(joint.parent, []).append(joint)
        self.joint_names = [j.name for j in self._chain_to(tip) if j.movable]

    # -- construction ------------------------------------------------------

    @classmethod
    def from_urdf(cls, path: str, tip: str = 'tool0') -> 'ArmModel':
        with open(path) as handle:
            return cls.from_string(handle.read(), tip=tip)

    @classmethod
    def from_string(cls, urdf: str, tip: str = 'tool0') -> 'ArmModel':
        root_element = ET.fromstring(urdf)

        links = {}
        for element in root_element.findall('link'):
            inertial = element.find('inertial')
            mass, com = 0.0, (0.0, 0.0, 0.0)
            if inertial is not None:
                mass = float(inertial.find('mass').get('value'))
                origin = inertial.find('origin')
                if origin is not None:
                    com = tuple(float(v) for v in (origin.get('xyz') or '0 0 0').split())
            links[element.get('name')] = Link(element.get('name'), mass, com)

        joints = []
        for element in root_element.findall('joint'):
            origin = element.find('origin')
            xyz = [float(v) for v in ((origin.get('xyz') if origin is not None else None)
                                      or '0 0 0').split()]
            rpy = [float(v) for v in ((origin.get('rpy') if origin is not None else None)
                                      or '0 0 0').split()]
            axis_element = element.find('axis')
            axis = [float(v) for v in ((axis_element.get('xyz')
                                        if axis_element is not None else None)
                                       or '1 0 0').split()]
            limit = element.find('limit')
            limits = (float(limit.get('lower', -math.pi)) if limit is not None else -math.pi,
                      float(limit.get('upper', math.pi)) if limit is not None else math.pi,
                      float(limit.get('velocity', 0.0)) if limit is not None else 0.0,
                      float(limit.get('effort', 0.0)) if limit is not None else 0.0)
            joints.append(Joint(
                element.get('name'), element.get('type'),
                element.find('parent').get('link'), element.find('child').get('link'),
                xyz, rpy, axis, limits))

        parented = {joint.child for joint in joints}
        roots = [name for name in links if name not in parented]
        if len(roots) != 1:
            raise ValueError(f'expected exactly one root link, found {roots}')
        return cls(links, joints, roots[0], tip=tip)

    def _chain_to(self, link: str) -> List[Joint]:
        """Joints from the root down to `link`, in order."""
        by_child = {joint.child: joint for joint in self.joints.values()}
        chain = []
        while link in by_child:
            joint = by_child[link]
            chain.append(joint)
            link = joint.parent
        chain.reverse()
        return chain

    # -- kinematics --------------------------------------------------------

    def link_poses(self, q: Sequence[float]) -> Dict[str, List[List[float]]]:
        """Pose of every link in the base frame."""
        if len(q) != len(self.joint_names):
            raise ValueError(f'expected {len(self.joint_names)} joint values, got {len(q)}')
        values = dict(zip(self.joint_names, q))

        identity = pose_from(rotation_from_rpy(0, 0, 0), [0.0, 0.0, 0.0])
        poses = {self.root: identity}
        stack = [self.root]
        while stack:
            parent = stack.pop()
            for joint in self._children.get(parent, []):
                poses[joint.child] = compose(
                    poses[parent], joint.transform(values.get(joint.name, 0.0)))
                stack.append(joint.child)
        return poses

    def tool_pose(self, q: Sequence[float]) -> List[List[float]]:
        """Pose of the tip link (tool0 by default) in the base frame."""
        return self.link_poses(q)[self.tip]

    def jacobian(self, q: Sequence[float]) -> List[List[float]]:
        """Geometric Jacobian of the tip in the base frame: 6 rows, one column
        per movable joint.  Rows 0-2 are linear, rows 3-5 angular."""
        poses = self.link_poses(q)
        tip = translation_of(poses[self.tip])

        columns = []
        for joint in self._chain_to(self.tip):
            if not joint.movable:
                continue
            # The joint frame is the parent's pose times the fixed offset; the
            # joint's own rotation does not move its own axis.
            frame = compose(poses[joint.parent], joint.origin)
            origin = translation_of(frame)
            axis = [sum(frame[i][k] * joint.axis[k] for k in range(3)) for i in range(3)]
            norm = math.sqrt(sum(v * v for v in axis)) or 1.0
            axis = [v / norm for v in axis]

            if joint.kind == 'prismatic':
                columns.append(list(axis) + [0.0, 0.0, 0.0])
            else:
                lever = [tip[i] - origin[i] for i in range(3)]
                columns.append(cross(axis, lever) + list(axis))

        return [[column[row] for column in columns] for row in range(6)]

    # -- statics -----------------------------------------------------------

    def gravity_torque(self, q: Sequence[float], payload: float = 0.0,
                       payload_link: Optional[str] = None) -> List[float]:
        """Joint torques [Nm] needed to hold the arm still against gravity.

        `payload` is a point mass at `payload_link` (the tip by default).
        """
        poses = self.link_poses(q)
        weight = [0.0, 0.0, -GRAVITY]

        # Where each mass acts, in the base frame, tagged with the link that
        # carries it so the joints below it can be charged for holding it up.
        loads = []
        for name, link in self.links.items():
            if link.mass <= 0.0 or name not in poses:
                continue
            pose = poses[name]
            centre = [sum(pose[i][k] * link.com[k] for k in range(3)) + pose[i][3]
                      for i in range(3)]
            loads.append((name, centre, link.mass))
        if payload > 0.0:
            tip = payload_link or self.tip
            loads.append((tip, translation_of(poses[tip]), payload))

        torques = []
        for joint in self._chain_to(self.tip):
            if not joint.movable:
                continue
            frame = compose(poses[joint.parent], joint.origin)
            origin = translation_of(frame)
            axis = [sum(frame[i][k] * joint.axis[k] for k in range(3)) for i in range(3)]
            norm = math.sqrt(sum(v * v for v in axis)) or 1.0
            axis = [v / norm for v in axis]

            # Only mass carried beyond this joint loads it.
            downstream = self._subtree_links(joint.child)
            total = 0.0
            for name, centre, mass in loads:
                if name not in downstream:
                    continue
                force = [mass * component for component in weight]
                lever = [centre[i] - origin[i] for i in range(3)]
                total += dot(axis, cross(lever, force))
            # The joint must supply the opposite of what gravity applies.
            torques.append(-total)
        return torques

    def _subtree_links(self, link: str) -> set:
        """Every link at or below `link` in the tree."""
        found, stack = {link}, [link]
        while stack:
            for joint in self._children.get(stack.pop(), []):
                found.add(joint.child)
                stack.append(joint.child)
        return found

    # -- properties of the machine ----------------------------------------

    @property
    def total_mass(self) -> float:
        return sum(link.mass for link in self.links.values())

    def limits(self) -> Dict[str, tuple]:
        return {name: (self.joints[name].lower, self.joints[name].upper)
                for name in self.joint_names}

    def effort_limits(self) -> Dict[str, float]:
        return {name: self.joints[name].effort for name in self.joint_names}
