# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Gazebo simulation of the 6-DOF arm.

    ros2 launch robot_arm_simulation simulation.launch.py

Starts Gazebo, spawns the robot, brings up ros2_control (hosted inside the
simulator by gazebo_ros2_control), activates the joint state broadcaster and
the trajectory controller, and opens RViz.

The controllers, their configuration and everything above them are exactly
what the real robot uses - only `hardware_type:=gazebo` differs.
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    world = LaunchConfiguration('world')
    sim_engine = LaunchConfiguration('sim_engine')
    prefix = LaunchConfiguration('prefix')

    declared_arguments = [
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz2 alongside the simulator.'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start the Gazebo client. Set to false for a headless run.'),
        DeclareLaunchArgument(
            'paused', default_value='false',
            description='Start the simulator paused.'),
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_arm_simulation'), 'worlds', 'robot_arm.world']),
            description='Gazebo world file.'),
        DeclareLaunchArgument(
            'sim_engine', default_value='classic',
            choices=['classic', 'ignition', 'gz'],
            description='Simulator flavour the description is generated for. '
                        'This launch file drives Gazebo Classic; the other values '
                        'exist so the same description can target newer simulators.'),
        DeclareLaunchArgument(
            'prefix', default_value='',
            description='Frame/joint name prefix.'),
        DeclareLaunchArgument(
            'spawn_x', default_value='0.0', description='Spawn position X [m].'),
        DeclareLaunchArgument(
            'spawn_y', default_value='0.0', description='Spawn position Y [m].'),
        DeclareLaunchArgument(
            'spawn_z', default_value='0.0', description='Spawn position Z [m].'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_arm_description'), 'rviz', 'view_robot.rviz']),
            description='RViz2 configuration file.'),
        DeclareLaunchArgument(
            'controllers_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_arm_control'), 'config', 'controllers.yaml']),
            description='ros2_control controller configuration.'),
    ]

    # The robot is anchored to `world` here: without that fixed joint the arm
    # would be a free-floating body in the physics engine.
    #
    # compact_xacro instead of plain xacro: gazebo_ros2_control re-passes this
    # description on a command line as `-p robot_description:=<xml>`, and rcl
    # parses that as a YAML scalar.  A pretty-printed URDF is not one - it
    # spans hundreds of lines and its comments carry ": " and "#".  The parse
    # fails, the plugin returns before creating the controller manager, and
    # every spawner then times out against a manager that never existed.
    # compact_xacro emits the same model as one comment-free line.
    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([
                FindPackagePrefix('robot_arm_description'),
                'lib', 'robot_arm_description', 'compact_xacro.py']),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('robot_arm_description'), 'urdf', 'robot_arm.urdf.xacro']),
            ' hardware_type:=gazebo',
            ' sim_engine:=', sim_engine,
            ' prefix:=', prefix,
            ' use_world_frame:=true',
            ' controllers_file:=', LaunchConfiguration('controllers_file'),
        ]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={
            'world': world,
            'gui': LaunchConfiguration('gui'),
            'pause': LaunchConfiguration('paused'),
            'verbose': 'false',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
        ],
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot_arm',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot_arm',
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z'),
        ],
    )

    # The controller_manager lives inside the Gazebo plugin, so the spawners
    # can only run once the model is in the world.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(declared_arguments + [
        gazebo,
        robot_state_publisher,
        spawn_robot,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner],
            )
        ),
        rviz,
    ])
