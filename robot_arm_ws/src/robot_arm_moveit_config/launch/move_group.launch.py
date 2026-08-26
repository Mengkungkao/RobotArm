# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Start MoveIt's move_group for the 6-DOF arm.

    ros2 launch robot_arm_moveit_config move_group.launch.py hardware_type:=gazebo

`hardware_type` only selects which URDF variant MoveIt loads; the kinematics
and the collision geometry are identical in every backend, so the planning
behaviour does not change between simulation and hardware.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    from moveit_configs_utils import MoveItConfigsBuilder

    hardware_type = LaunchConfiguration('hardware_type').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    pipelines = LaunchConfiguration('planning_pipelines').perform(context).split()
    log_level = LaunchConfiguration('log_level').perform(context)

    moveit_config = (
        MoveItConfigsBuilder('robot_arm', package_name='robot_arm_moveit_config')
        # use_world_frame is forced to false: the model root - and therefore
        # the planning frame - must be base_link in every mode.
        .robot_description(
            mappings={'hardware_type': hardware_type, 'use_world_frame': 'false'})
        .robot_description_semantic(file_path='config/robot_arm.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=pipelines)
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
        )
        .to_moveit_configs()
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': use_sim_time},
            # Without this the first plan after start-up can be rejected
            # because the controller has not published a state yet.
            {'publish_robot_description_semantic': True},
        ],
    )

    return [move_group]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware_type', default_value='mock',
            choices=['gazebo', 'mock', 'real'],
            description='Backend variant of the URDF MoveIt loads.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use the simulator clock.'),
        DeclareLaunchArgument(
            'planning_pipelines', default_value='ompl',
            description='Space separated pipelines, e.g. '
                        '"ompl pilz_industrial_motion_planner". Pilz adds PTP/LIN/CIRC '
                        'and needs the pilz_industrial_motion_planner package.'),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='move_group log level.'),
        OpaqueFunction(function=launch_setup),
    ])
