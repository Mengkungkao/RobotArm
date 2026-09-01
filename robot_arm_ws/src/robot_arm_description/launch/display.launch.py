# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Visualise the robot description - no controllers, no simulator, no hardware.

    ros2 launch robot_arm_description display.launch.py

Joint values come from joint_state_publisher_gui sliders, so this is the
fastest way to check the kinematics, the joint limits and the TF tree
(Phase 3 of the build-up).
"""

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = get_package_share_directory('robot_arm_description')

    args = [
        DeclareLaunchArgument(
            'hardware_type', default_value='mock',
            choices=['mock', 'gazebo', 'real'],
            description='ros2_control backend written into the description.'),
        DeclareLaunchArgument(
            'prefix', default_value='',
            description='Frame/joint name prefix, e.g. "left_".'),
        DeclareLaunchArgument(
            'use_gui', default_value='true',
            description='Use joint_state_publisher_gui sliders.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz2.'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_arm_description'), 'rviz', 'view_robot.rviz']),
            description='RViz2 configuration file.'),
    ]

    # The description is expanded exactly the same way here as in the
    # simulation and real-robot launch files - only the arguments differ.
    robot_description = ParameterValue(
        Command([
            'xacro ', PathJoinSubstitution(
                [description_share, 'urdf', 'robot_arm.urdf.xacro']),
            ' hardware_type:=', LaunchConfiguration('hardware_type'),
            ' prefix:=', LaunchConfiguration('prefix'),
            ' use_world_frame:=false',
        ]),
        value_type=str,
    )

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        # The model root is base_link; `world` is provided here so that RViz
        # can always use the same fixed frame, in every mode.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base_link',
            output='log',
            arguments=['--frame-id', 'world', '--child-frame-id', 'base_link'],
        ),
        # The sliders: one per movable joint, limits read from the URDF,
        # published on /joint_states for RViz to follow.
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_gui')),
        ),
        # Same topic, no window: holds every joint at zero.  Useful over ssh,
        # and in tests.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('use_gui')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ]

    return LaunchDescription(args + nodes)
