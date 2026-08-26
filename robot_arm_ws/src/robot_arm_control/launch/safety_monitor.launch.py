# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Start the safety monitor on its own.

It is normally started by the bringup launch files; running it separately is
useful when attaching to an already running robot.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'safety_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_arm_control'), 'config', 'safety.yaml']),
            description='Safety limits and monitor behaviour.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use the simulator clock.'),
        Node(
            package='robot_arm_control',
            executable='safety_monitor',
            name='safety_monitor',
            output='screen',
            parameters=[
                LaunchConfiguration('safety_config'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
