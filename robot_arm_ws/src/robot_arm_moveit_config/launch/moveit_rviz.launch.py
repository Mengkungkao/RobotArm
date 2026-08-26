# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
RViz2 with the MoveIt MotionPlanning panel.

    ros2 launch robot_arm_moveit_config moveit_rviz.launch.py

RViz needs the same robot description, SRDF and kinematics parameters as
move_group, otherwise the interactive marker and the planning requests it
sends are built from a different model than the one that plans.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    from moveit_configs_utils import MoveItConfigsBuilder

    hardware_type = LaunchConfiguration('hardware_type').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    pipelines = LaunchConfiguration('planning_pipelines').perform(context).split()

    moveit_config = (
        MoveItConfigsBuilder('robot_arm', package_name='robot_arm_moveit_config')
        .robot_description(
            mappings={'hardware_type': hardware_type, 'use_world_frame': 'false'})
        .robot_description_semantic(file_path='config/robot_arm.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .planning_pipelines(pipelines=pipelines)
        .to_moveit_configs()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='moveit_rviz',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config').perform(context)],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': use_sim_time},
        ],
    )

    return [rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware_type', default_value='mock',
            choices=['gazebo', 'mock', 'real'],
            description='Backend variant of the URDF RViz loads.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use the simulator clock.'),
        DeclareLaunchArgument(
            'planning_pipelines', default_value='ompl',
            description='Space separated planning pipelines.'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_arm_moveit_config'), 'config', 'moveit.rviz']),
            description='RViz2 configuration file.'),
        OpaqueFunction(function=launch_setup),
    ])
