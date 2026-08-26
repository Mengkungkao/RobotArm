# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Run the C++ API demo against whatever backend is already running.

    ros2 launch robot_arm_bringup sim.launch.py       # or real.launch.py
    ros2 launch robot_arm_control cpp_api_demo.launch.py

MoveGroupInterface needs the robot description and the SRDF as parameters of
the calling node, so they are rebuilt here exactly as the rest of the stack
does it.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware_type = LaunchConfiguration('hardware_type')

    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution(
                [FindPackageShare('robot_arm_description'), 'urdf', 'robot_arm.urdf.xacro']),
            ' hardware_type:=', hardware_type,
            ' use_world_frame:=false',
        ]),
        value_type=str,
    )

    robot_description_semantic = ParameterValue(
        Command([
            'cat ',
            PathJoinSubstitution(
                [FindPackageShare('robot_arm_moveit_config'), 'config', 'robot_arm.srdf']),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware_type', default_value='gazebo',
            choices=['gazebo', 'mock', 'real'],
            description='Backend the running stack uses; only affects the URDF '
                        'this node loads, not which robot is driven.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the simulator clock.'),
        Node(
            package='robot_arm_control',
            executable='cpp_api_demo',
            name='cpp_api_demo',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'robot_description_semantic': robot_description_semantic},
                PathJoinSubstitution(
                    [FindPackageShare('robot_arm_moveit_config'), 'config', 'kinematics.yaml']),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
