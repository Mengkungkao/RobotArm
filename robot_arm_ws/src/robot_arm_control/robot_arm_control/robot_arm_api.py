# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Python API for the 6-DOF arm.

    from robot_arm_control import RobotArm

    with RobotArm() as robot:
        robot.enable()
        robot.move_joints([0.0, 0.5, -0.8, 0.0, 0.5, 0.0])
        robot.move_to_pose(x=0.35, y=0.10, z=0.40, roll=0.0, pitch=1.57, yaw=0.0)
        print(robot.get_current_pose())
        robot.stop()

The very same code drives Gazebo and the physical robot: it talks to MoveIt,
to the `arm_controller` and to the safety services, none of which know which
backend is underneath.

Planning goes through MoveIt's `/move_action`, which is plain `moveit_msgs`
over rclpy - no `moveit_py` build is required.  With `use_moveit=False` the
class still works and sends trajectories straight to the joint trajectory
controller, which is useful for tests and for a minimal installation.
"""

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from robot_arm_interfaces.srv import GetCalibration, SetEStop, SetMotorEnable

from .transforms import euler_from_quaternion, normalize_quaternion, quaternion_from_euler

__all__ = ['RobotArm', 'MoveResult', 'JointStates', 'DEFAULT_JOINT_NAMES']

DEFAULT_JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']


@dataclass
class MoveResult:
    """Outcome of a motion request."""

    success: bool
    message: str = ''
    error_code: int = 0
    planning_time: float = 0.0

    def __bool__(self) -> bool:
        return self.success


@dataclass
class JointStates:
    """Snapshot of /joint_states, ordered like `joint_names`."""

    names: List[str] = field(default_factory=list)
    positions: List[float] = field(default_factory=list)
    velocities: List[float] = field(default_factory=list)
    efforts: List[float] = field(default_factory=list)

    def as_dict(self) -> Dict[str, float]:
        """Joint name -> position, for readable printing and assertions."""
        return dict(zip(self.names, self.positions))


class RobotArm:
    """High-level control of the arm, identical in simulation and on hardware."""

    def __init__(
        self,
        node: Optional[Node] = None,
        node_name: str = 'robot_arm_api',
        joint_names: Optional[Sequence[str]] = None,
        group_name: str = 'arm',
        base_frame: str = 'base_link',
        end_effector_frame: str = 'tool0',
        controller: str = 'arm_controller',
        use_moveit: bool = True,
        velocity_scaling: float = 0.3,
        acceleration_scaling: float = 0.3,
        planning_time: float = 5.0,
        planner_id: str = '',
        timeout: float = 10.0,
    ) -> None:
        """
        Create the API.

        When `node` is None the class creates and spins its own node, so it can
        be used from a plain script.  When a node is passed in, the caller is
        responsible for spinning it (for example from inside another node).
        """
        self.joint_names = list(joint_names) if joint_names else list(DEFAULT_JOINT_NAMES)
        self.group_name = group_name
        self.base_frame = base_frame
        self.end_effector_frame = end_effector_frame
        self.controller = controller
        self.use_moveit = use_moveit
        self.velocity_scaling = velocity_scaling
        self.acceleration_scaling = acceleration_scaling
        self.planning_time = planning_time
        self.planner_id = planner_id
        self.timeout = timeout

        self._owns_node = node is None
        if self._owns_node and not rclpy.ok():
            rclpy.init()
        self._node = node if node is not None else rclpy.create_node(node_name)
        self._callback_group = ReentrantCallbackGroup()

        self._joint_state: Optional[JointState] = None
        self._joint_state_lock = threading.Lock()
        self._active_goal_handle = None

        self._node.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10,
            callback_group=self._callback_group)

        self._e_stop_publisher = self._node.create_publisher(Bool, '/e_stop', 10)

        self._enable_client = self._node.create_client(
            SetMotorEnable, '/robot_arm/set_motor_enable', callback_group=self._callback_group)
        self._e_stop_client = self._node.create_client(
            SetEStop, '/robot_arm/set_e_stop', callback_group=self._callback_group)
        self._calibration_client = self._node.create_client(
            GetCalibration, '/robot_arm/get_calibration', callback_group=self._callback_group)

        self._trajectory_client = ActionClient(
            self._node, FollowJointTrajectory,
            f'/{self.controller}/follow_joint_trajectory',
            callback_group=self._callback_group)

        self._move_group_client = None
        self._execute_client = None
        self._cartesian_client = None
        self._ik_client = None
        self._fk_client = None

        self._tf_buffer = None
        self._tf_listener = None

        self._executor = None
        self._spin_thread = None
        if self._owns_node:
            self._executor = MultiThreadedExecutor()
            self._executor.add_node(self._node)
            self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
            self._spin_thread.start()

        if self.use_moveit:
            self._setup_moveit()

    # -- lifecycle ---------------------------------------------------------

    def __enter__(self) -> 'RobotArm':
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.shutdown()

    def shutdown(self) -> None:
        """Stop spinning and destroy the node, if this object created one."""
        if not self._owns_node:
            return
        if self._executor is not None:
            self._executor.shutdown()
        if self._spin_thread is not None and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        try:
            self._node.destroy_node()
        except Exception:      # noqa: BLE001 - destroying twice must not raise
            pass

    @property
    def node(self) -> Node:
        """The underlying rclpy node, for advanced use."""
        return self._node

    # -- MoveIt plumbing ---------------------------------------------------

    def _setup_moveit(self) -> None:
        """Create the MoveIt clients.  Imported lazily so that a workspace
        without MoveIt can still use the direct trajectory path."""
        try:
            from moveit_msgs.action import ExecuteTrajectory, MoveGroup
            from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetPositionIK
        except ImportError as error:      # pragma: no cover - depends on the install
            raise ImportError(
                'moveit_msgs is not available. Install MoveIt 2 or construct '
                'RobotArm(use_moveit=False) to command the trajectory controller '
                'directly.') from error

        self._move_group_client = ActionClient(
            self._node, MoveGroup, '/move_action', callback_group=self._callback_group)
        self._execute_client = ActionClient(
            self._node, ExecuteTrajectory, '/execute_trajectory',
            callback_group=self._callback_group)
        self._cartesian_client = self._node.create_client(
            GetCartesianPath, '/compute_cartesian_path', callback_group=self._callback_group)
        self._ik_client = self._node.create_client(
            GetPositionIK, '/compute_ik', callback_group=self._callback_group)
        self._fk_client = self._node.create_client(
            GetPositionFK, '/compute_fk', callback_group=self._callback_group)

    def _ensure_tf(self) -> None:
        if self._tf_buffer is not None:
            return
        from tf2_ros import TransformListener
        from tf2_ros.buffer import Buffer
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self._node, spin_thread=False)

    # -- helpers -----------------------------------------------------------

    def _on_joint_state(self, message: JointState) -> None:
        with self._joint_state_lock:
            self._joint_state = message

    def _wait(self, future, timeout: Optional[float] = None):
        """Wait for a future while somebody else spins the node."""
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while not future.done():
            if time.monotonic() > deadline:
                return None
            if not rclpy.ok():
                return None
            time.sleep(0.005)
        return future.result()

    def _call(self, client, request, description: str, timeout: Optional[float] = None):
        """Call a service, returning None when it is unavailable or times out."""
        wait = self.timeout if timeout is None else timeout
        if not client.wait_for_service(timeout_sec=wait):
            self._node.get_logger().warning(f'{description}: service is not available')
            return None
        return self._wait(client.call_async(request), wait)

    def _log(self, message: str) -> None:
        self._node.get_logger().info(message)

    # -- state -------------------------------------------------------------

    def wait_for_state(self, timeout: float = 10.0) -> bool:
        """Block until the first /joint_states message has arrived."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._joint_state_lock:
                if self._joint_state is not None:
                    return True
            time.sleep(0.02)
        return False

    def get_joint_states(self) -> JointStates:
        """Current joint positions, velocities and efforts, in joint order."""
        with self._joint_state_lock:
            message = self._joint_state
        result = JointStates(names=list(self.joint_names))
        if message is None:
            return result

        index = {name: i for i, name in enumerate(message.name)}
        for name in self.joint_names:
            i = index.get(name)
            if i is None:
                result.positions.append(float('nan'))
                result.velocities.append(float('nan'))
                result.efforts.append(float('nan'))
                continue
            result.positions.append(
                message.position[i] if i < len(message.position) else float('nan'))
            result.velocities.append(
                message.velocity[i] if i < len(message.velocity) else float('nan'))
            result.efforts.append(message.effort[i] if i < len(message.effort) else float('nan'))
        return result

    def get_joint_positions(self) -> List[float]:
        """Joint positions [rad] in the canonical joint order."""
        return self.get_joint_states().positions

    def get_current_pose(self, frame: Optional[str] = None, reference: Optional[str] = None):
        """
        Pose of the end effector as a geometry_msgs/PoseStamped.

        Uses TF, so it works with or without MoveIt.  Returns None when the
        transform is not available yet.
        """
        frame = frame or self.end_effector_frame
        reference = reference or self.base_frame
        self._ensure_tf()

        from rclpy.duration import Duration
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            try:
                transform = self._tf_buffer.lookup_transform(
                    reference, frame, rclpy.time.Time(), timeout=Duration(seconds=0.2))
            except Exception:      # noqa: BLE001 - TF raises several types
                time.sleep(0.05)
                continue

            pose = PoseStamped()
            pose.header = transform.header
            pose.pose.position = Point(
                x=transform.transform.translation.x,
                y=transform.transform.translation.y,
                z=transform.transform.translation.z)
            pose.pose.orientation = transform.transform.rotation
            return pose

        self._node.get_logger().warning(f'No transform {reference} -> {frame}')
        return None

    def get_current_pose_rpy(self) -> Optional[Dict[str, float]]:
        """Current end-effector pose as a plain dict with roll/pitch/yaw."""
        pose = self.get_current_pose()
        if pose is None:
            return None
        roll, pitch, yaw = euler_from_quaternion(
            pose.pose.orientation.x, pose.pose.orientation.y,
            pose.pose.orientation.z, pose.pose.orientation.w)
        return {
            'x': pose.pose.position.x, 'y': pose.pose.position.y, 'z': pose.pose.position.z,
            'roll': roll, 'pitch': pitch, 'yaw': yaw,
        }

    # -- motion ------------------------------------------------------------

    def move_joints(
        self,
        positions: Sequence[float],
        wait: bool = True,
        velocity_scaling: Optional[float] = None,
        acceleration_scaling: Optional[float] = None,
        duration: float = 5.0,
        tolerance: float = 1e-3,
    ) -> MoveResult:
        """
        Move to a joint-space target [rad], one value per joint.

        With MoveIt the motion is planned and collision-checked; without it the
        values are sent to the trajectory controller as a single timed point.
        """
        if len(positions) != len(self.joint_names):
            return MoveResult(
                False,
                f'expected {len(self.joint_names)} joint values, got {len(positions)}')
        if not all(math.isfinite(value) for value in positions):
            return MoveResult(False, 'joint targets must all be finite')

        if not self.use_moveit:
            return self._send_trajectory(positions, duration=duration, wait=wait)

        from moveit_msgs.msg import Constraints, JointConstraint

        constraints = Constraints()
        for name, value in zip(self.joint_names, positions):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = float(value)
            constraint.tolerance_above = tolerance
            constraint.tolerance_below = tolerance
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)

        goal = self._build_move_group_goal(
            constraints, velocity_scaling, acceleration_scaling)
        return self._send_move_group_goal(goal, wait, 'joint-space motion')

    def move_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        quaternion: Optional[Sequence[float]] = None,
        frame: Optional[str] = None,
        wait: bool = True,
        position_tolerance: float = 0.005,
        orientation_tolerance: float = 0.01,
        velocity_scaling: Optional[float] = None,
        acceleration_scaling: Optional[float] = None,
    ) -> MoveResult:
        """
        Move `tool0` to a Cartesian target.

        Orientation is given as roll/pitch/yaw, or directly as an
        (x, y, z, w) quaternion when `quaternion` is set.
        """
        if not self.use_moveit:
            return MoveResult(
                False, 'Cartesian motion needs MoveIt; construct RobotArm(use_moveit=True)')
        if not all(math.isfinite(value) for value in (x, y, z, roll, pitch, yaw)):
            return MoveResult(False, 'pose values must all be finite')

        from moveit_msgs.msg import (BoundingVolume, Constraints, OrientationConstraint,
                                     PositionConstraint)
        from shape_msgs.msg import SolidPrimitive

        frame = frame or self.base_frame
        if quaternion is None:
            quaternion = quaternion_from_euler(roll, pitch, yaw)
        quaternion = normalize_quaternion(*quaternion)

        constraints = Constraints()

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = frame
        position_constraint.link_name = self.end_effector_frame
        position_constraint.weight = 1.0
        # A small sphere around the target is the standard way of expressing
        # "reach this point within this tolerance" to MoveIt.
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [float(position_tolerance)]
        volume = BoundingVolume()
        volume.primitives.append(primitive)
        target = Pose()
        target.position = Point(x=float(x), y=float(y), z=float(z))
        target.orientation = Quaternion(w=1.0)
        volume.primitive_poses.append(target)
        position_constraint.constraint_region = volume
        constraints.position_constraints.append(position_constraint)

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = frame
        orientation_constraint.link_name = self.end_effector_frame
        orientation_constraint.orientation = Quaternion(
            x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
        orientation_constraint.absolute_x_axis_tolerance = orientation_tolerance
        orientation_constraint.absolute_y_axis_tolerance = orientation_tolerance
        orientation_constraint.absolute_z_axis_tolerance = orientation_tolerance
        orientation_constraint.weight = 1.0
        constraints.orientation_constraints.append(orientation_constraint)

        goal = self._build_move_group_goal(
            constraints, velocity_scaling, acceleration_scaling)
        return self._send_move_group_goal(goal, wait, 'Cartesian motion')

    def move_linear(
        self,
        x: float,
        y: float,
        z: float,
        roll: Optional[float] = None,
        pitch: Optional[float] = None,
        yaw: Optional[float] = None,
        step: float = 0.005,
        jump_threshold: float = 0.0,
        min_fraction: float = 0.9,
        wait: bool = True,
    ) -> MoveResult:
        """
        Move `tool0` along a straight line to the target pose.

        Uses MoveIt's Cartesian path service; the motion is rejected when less
        than `min_fraction` of the requested path can be followed, so a partial
        move never happens silently.
        """
        if not self.use_moveit:
            return MoveResult(False, 'linear motion needs MoveIt')

        from moveit_msgs.action import ExecuteTrajectory
        from moveit_msgs.srv import GetCartesianPath

        current = self.get_current_pose()
        if current is None:
            return MoveResult(False, 'current pose is unknown, cannot plan a linear motion')

        if roll is None or pitch is None or yaw is None:
            orientation = current.pose.orientation
        else:
            quaternion = quaternion_from_euler(roll, pitch, yaw)
            orientation = Quaternion(
                x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

        target = Pose()
        target.position = Point(x=float(x), y=float(y), z=float(z))
        target.orientation = orientation

        request = GetCartesianPath.Request()
        request.header.frame_id = self.base_frame
        request.group_name = self.group_name
        request.link_name = self.end_effector_frame
        request.waypoints = [target]
        request.max_step = step
        request.jump_threshold = jump_threshold
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = self.velocity_scaling
        request.max_acceleration_scaling_factor = self.acceleration_scaling

        response = self._call(self._cartesian_client, request, 'compute_cartesian_path')
        if response is None:
            return MoveResult(False, 'compute_cartesian_path did not answer')
        if response.fraction < min_fraction:
            return MoveResult(
                False,
                f'only {response.fraction * 100:.1f}% of the straight-line path is reachable')

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.solution
        return self._send_action_goal(
            self._execute_client, goal, wait, 'linear motion',
            success_check=lambda result: result.error_code.val == 1)

    def _build_move_group_goal(self, constraints, velocity_scaling, acceleration_scaling):
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import PlanningOptions

        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = self.planning_time
        goal.request.max_velocity_scaling_factor = (
            self.velocity_scaling if velocity_scaling is None else velocity_scaling)
        goal.request.max_acceleration_scaling_factor = (
            self.acceleration_scaling if acceleration_scaling is None else acceleration_scaling)
        if self.planner_id:
            goal.request.planner_id = self.planner_id
        goal.request.goal_constraints = [constraints]

        options = PlanningOptions()
        options.plan_only = False
        options.replan = True
        options.replan_attempts = 3
        goal.planning_options = options
        return goal

    def _send_move_group_goal(self, goal, wait: bool, description: str) -> MoveResult:
        return self._send_action_goal(
            self._move_group_client, goal, wait, description,
            success_check=lambda result: result.error_code.val == 1,
            planning_time=lambda result: result.planning_time,
            error_code=lambda result: result.error_code.val)

    def _send_action_goal(
        self, client, goal, wait: bool, description: str,
        success_check=None, planning_time=None, error_code=None,
    ) -> MoveResult:
        if client is None:
            return MoveResult(False, f'{description}: MoveIt clients are not initialised')
        if not client.wait_for_server(timeout_sec=self.timeout):
            return MoveResult(False, f'{description}: action server is not available')

        send_future = client.send_goal_async(goal)
        goal_handle = self._wait(send_future)
        if goal_handle is None:
            return MoveResult(False, f'{description}: the goal was not accepted in time')
        if not goal_handle.accepted:
            return MoveResult(False, f'{description}: the goal was rejected')

        self._active_goal_handle = goal_handle
        if not wait:
            return MoveResult(True, f'{description}: goal accepted (not waiting)')

        # Execution can legitimately take much longer than a service call.
        result_response = self._wait(
            goal_handle.get_result_async(), timeout=max(self.timeout, 300.0))
        self._active_goal_handle = None
        if result_response is None:
            return MoveResult(False, f'{description}: timed out while executing')

        result = result_response.result
        success = success_check(result) if success_check else True
        return MoveResult(
            success=success,
            message=f'{description} ' + ('succeeded' if success else 'failed'),
            error_code=error_code(result) if error_code else 0,
            planning_time=planning_time(result) if planning_time else 0.0,
        )

    def _send_trajectory(
        self, positions: Sequence[float], duration: float, wait: bool
    ) -> MoveResult:
        """Send a single-point trajectory straight to the controller."""
        if not self._trajectory_client.wait_for_server(timeout_sec=self.timeout):
            return MoveResult(
                False, f'{self.controller}/follow_joint_trajectory is not available')

        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.velocities = [0.0] * len(positions)
        point.time_from_start = DurationMsg(
            sec=int(duration), nanosec=int((duration % 1.0) * 1e9))

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = list(self.joint_names)
        goal.trajectory.points = [point]

        return self._send_action_goal(
            self._trajectory_client, goal, wait, 'trajectory execution',
            success_check=lambda result: result.error_code == 0,
            error_code=lambda result: result.error_code)

    # -- kinematics --------------------------------------------------------

    def forward_kinematics(
            self,
            joint_positions: Optional[Sequence[float]] = None,
            link: Optional[str] = None):
        """
        FK through MoveIt: joint angles -> pose of `link` (default `tool0`).

        With no argument the current joint state is used.
        """
        if not self.use_moveit:
            return None
        from moveit_msgs.srv import GetPositionFK

        positions = list(joint_positions) if joint_positions is not None \
            else self.get_joint_positions()
        request = GetPositionFK.Request()
        request.header.frame_id = self.base_frame
        request.fk_link_names = [link or self.end_effector_frame]
        request.robot_state.joint_state.name = list(self.joint_names)
        request.robot_state.joint_state.position = [float(value) for value in positions]

        response = self._call(self._fk_client, request, 'compute_fk')
        if response is None or not response.pose_stamped:
            return None
        if response.error_code.val != 1:
            self._node.get_logger().warning(
                f'compute_fk failed with error code {response.error_code.val}')
            return None
        return response.pose_stamped[0]

    def inverse_kinematics(
        self,
        x: float, y: float, z: float,
        roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
        quaternion: Optional[Sequence[float]] = None,
        seed: Optional[Sequence[float]] = None,
        avoid_collisions: bool = True,
        attempts_timeout: float = 1.0,
    ) -> Optional[List[float]]:
        """
        IK through MoveIt: pose of `tool0` -> joint angles.

        Returns None when the pose is unreachable, which is a normal answer and
        not an error.
        """
        if not self.use_moveit:
            return None
        from moveit_msgs.srv import GetPositionIK

        if quaternion is None:
            quaternion = quaternion_from_euler(roll, pitch, yaw)
        quaternion = normalize_quaternion(*quaternion)

        target = PoseStamped()
        target.header.frame_id = self.base_frame
        target.pose.position = Point(x=float(x), y=float(y), z=float(z))
        target.pose.orientation = Quaternion(
            x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

        seed_positions = list(seed) if seed is not None else self.get_joint_positions()

        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group_name
        request.ik_request.ik_link_name = self.end_effector_frame
        request.ik_request.pose_stamped = target
        request.ik_request.avoid_collisions = avoid_collisions
        request.ik_request.timeout = DurationMsg(
            sec=int(attempts_timeout), nanosec=int((attempts_timeout % 1.0) * 1e9))
        request.ik_request.robot_state.joint_state.name = list(self.joint_names)
        request.ik_request.robot_state.joint_state.position = [
            0.0 if not math.isfinite(value) else float(value) for value in seed_positions]

        response = self._call(self._ik_client, request, 'compute_ik')
        if response is None or response.error_code.val != 1:
            return None

        solution = dict(zip(
            response.solution.joint_state.name, response.solution.joint_state.position))
        return [solution[name] for name in self.joint_names if name in solution] or None

    # -- safety / power ----------------------------------------------------

    def stop(self) -> MoveResult:
        """
        Stop the arm now: cancel the running goal and hold the current pose.

        Works in both modes because it acts on the controller, not on the
        backend.
        """
        cancelled = False
        if self._active_goal_handle is not None:
            try:
                self._wait(self._active_goal_handle.cancel_goal_async(), timeout=2.0)
                cancelled = True
            except Exception as error:      # noqa: BLE001
                self._node.get_logger().warning(f'Cannot cancel the goal: {error}')
            self._active_goal_handle = None

        from std_srvs.srv import Trigger
        client = self._node.create_client(
            Trigger, '/robot_arm/stop', callback_group=self._callback_group)
        response = self._call(client, Trigger.Request(), 'robot_arm/stop', timeout=2.0)
        self._node.destroy_client(client)

        if response is not None:
            return MoveResult(response.success, response.message)
        return MoveResult(cancelled, 'goal cancelled' if cancelled else 'nothing to stop')

    def enable(self) -> MoveResult:
        """Energise the drives (real robot) / activate the controller (simulation)."""
        return self._set_enabled(True)

    def disable(self) -> MoveResult:
        """De-energise the drives / deactivate the controller."""
        return self._set_enabled(False)

    def _set_enabled(self, enabled: bool) -> MoveResult:
        request = SetMotorEnable.Request()
        request.enable = enabled
        response = self._call(self._enable_client, request, 'set_motor_enable')
        if response is None:
            return MoveResult(False, '/robot_arm/set_motor_enable is not available')
        return MoveResult(response.success, response.message)

    def set_e_stop(self, engage: bool, reason: str = 'python api') -> MoveResult:
        """
        Engage or release the emergency stop.

        Falls back to the /e_stop topic when the service is not available, so
        the call still reaches the driver in a minimal setup.
        """
        request = SetEStop.Request()
        request.engage = engage
        request.reason = reason
        response = self._call(self._e_stop_client, request, 'set_e_stop', timeout=2.0)
        if response is not None:
            return MoveResult(response.success, response.message)

        message = Bool()
        message.data = engage
        self._e_stop_publisher.publish(message)
        return MoveResult(True, 'published on /e_stop (no e-stop service found)')

    def emergency_stop(self, reason: str = 'python api') -> MoveResult:
        """Engage the emergency stop."""
        return self.set_e_stop(True, reason)

    def get_calibration(self):
        """Calibration currently in force, or None outside the real robot."""
        response = self._call(
            self._calibration_client, GetCalibration.Request(), 'get_calibration', timeout=2.0)
        return None if response is None else response.joints

    def home(self, wait: bool = True) -> MoveResult:
        """
        Move to the calibrated home pose.

        The targets come from the driver's calibration when it is available
        (real robot) and default to all-zero otherwise.
        """
        calibration = self.get_calibration()
        if calibration:
            by_name = {record.name: record.home_position for record in calibration}
            targets = [by_name.get(name, 0.0) for name in self.joint_names]
        else:
            targets = [0.0] * len(self.joint_names)
        return self.move_joints(targets, wait=wait)
