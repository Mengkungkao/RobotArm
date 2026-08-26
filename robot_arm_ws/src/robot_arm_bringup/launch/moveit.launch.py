# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
MoveIt 2 for an already running robot.

    ros2 launch robot_arm_bringup moveit.launch.py hardware_type:=gazebo

Attaches move_group (and optionally RViz) to whatever backend is running.
Useful when the robot is brought up separately, or when move_group has to be
restarted without touching the robot.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware_type = LaunchConfiguration('hardware_type')
    use_sim_time = LaunchConfiguration('use_sim_time')
    planning_pipelines = LaunchConfiguration('planning_pipelines')

    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware_type', default_value='gazebo',
            choices=['gazebo', 'mock', 'real'],
            description='Backend variant of the URDF MoveIt loads. It changes '
                        'nothing about the kinematics - only which ros2_control '
                        'section the description carries.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the simulator clock.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Also start RViz with the MotionPlanning panel.'),
        DeclareLaunchArgument(
            'planning_pipelines', default_value='ompl',
            description='Space separated MoveIt planning pipelines.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robot_arm_moveit_config'),
                    'launch', 'move_group.launch.py'])
            ]),
            launch_arguments={
                'hardware_type': hardware_type,
                'use_sim_time': use_sim_time,
                'planning_pipelines': planning_pipelines,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robot_arm_moveit_config'),
                    'launch', 'moveit_rviz.launch.py'])
            ]),
            launch_arguments={
                'hardware_type': hardware_type,
                'use_sim_time': use_sim_time,
                'planning_pipelines': planning_pipelines,
            }.items(),
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
