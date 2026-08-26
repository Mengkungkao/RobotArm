# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Control stack for the PHYSICAL robot.

    ros2 launch robot_arm_bringup real_robot.launch.py

Starts robot_state_publisher, the ros2_control node with the
robot_arm_hardware driver, the controller spawners and the safety monitor.
This is the exact counterpart of robot_arm_simulation/simulation.launch.py:
same controllers, same configuration, same topics - only the ros2_control
backend differs.

    hardware_type:=real   talk to the motor controller (default)
    hardware_type:=mock   no simulator and no hardware, for offline work

Before running this against a machine, check
robot_arm_hardware/config/hardware.yaml: it ships with the loopback bus so a
fresh clone cannot move anything.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware_type = LaunchConfiguration('hardware_type')
    controllers_file = LaunchConfiguration('controllers_file')
    prefix = LaunchConfiguration('prefix')

    declared_arguments = [
        DeclareLaunchArgument(
            'hardware_type', default_value='real',
            choices=['real', 'mock'],
            description='ros2_control backend. "mock" runs the whole stack with no '
                        'hardware attached.'),
        DeclareLaunchArgument(
            'prefix', default_value='', description='Frame/joint name prefix.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Start RViz2 (bringup.launch.py starts it separately).'),
        DeclareLaunchArgument(
            'use_safety_monitor', default_value='true',
            description='Start the safety monitor. bringup.launch.py sets this to '
                        'false because it starts the monitor itself.'),
        DeclareLaunchArgument(
            'controllers_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_arm_control'), 'config', 'controllers.yaml']),
            description='ros2_control controller configuration.'),
        DeclareLaunchArgument(
            'safety_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_arm_control'), 'config', 'safety.yaml']),
            description='Safety monitor configuration.'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_arm_description'), 'rviz', 'view_robot.rviz']),
            description='RViz2 configuration file.'),
        DeclareLaunchArgument(
            'initial_controllers', default_value='arm_controller',
            description='Controller activated at start-up.'),
    ]

    # The model root is base_link, exactly as in the MoveIt configuration.
    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution(
                [FindPackageShare('robot_arm_description'), 'urdf', 'robot_arm.urdf.xacro']),
            ' hardware_type:=', hardware_type,
            ' prefix:=', prefix,
            ' use_world_frame:=false',
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # `world` exists in every mode so RViz and applications can use one fixed
    # frame; in simulation the same transform comes from the URDF anchor.
    world_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_link',
        output='log',
        arguments=['--frame-id', 'world', '--child-frame-id', 'base_link'],
    )

    # On the real robot the controller_manager is a normal node; in simulation
    # the very same manager lives inside the Gazebo plugin.
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='both',
        parameters=[{'robot_description': robot_description}, controllers_file],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            LaunchConfiguration('initial_controllers'),
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    safety_monitor = Node(
        package='robot_arm_control',
        executable='safety_monitor',
        name='safety_monitor',
        output='screen',
        parameters=[LaunchConfiguration('safety_config')],
        condition=IfCondition(LaunchConfiguration('use_safety_monitor')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription(declared_arguments + [
        robot_state_publisher,
        world_transform,
        controller_manager,
        joint_state_broadcaster_spawner,
        # The trajectory controller is only useful once joint states flow.
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner],
            )
        ),
        safety_monitor,
        rviz,
    ])
