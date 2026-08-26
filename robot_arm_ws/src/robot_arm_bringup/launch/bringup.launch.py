# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
One entry point for the whole system.

    # simulation
    ros2 launch robot_arm_bringup bringup.launch.py \
        use_sim:=true use_rviz:=true use_moveit:=true

    # physical robot - same command, one word changed
    ros2 launch robot_arm_bringup bringup.launch.py \
        use_sim:=false use_rviz:=true use_moveit:=true

What changes between the two: which ros2_control backend is loaded.
What does not change: the controllers, the MoveIt configuration, the safety
monitor, the topics, the services and every application built on them.

    use_sim:=true|false            simulator or hardware
    hardware_interface:=sim|mock|real
                                   override the backend explicitly; `mock`
                                   runs the whole stack with neither
    use_rviz:=true|false
    use_moveit:=true|false
    use_safety_monitor:=true|false
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in ('true', '1', 'yes', 'on')


def launch_setup(context, *args, **kwargs):
    use_sim = _as_bool(LaunchConfiguration('use_sim').perform(context))
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    use_moveit = LaunchConfiguration('use_moveit').perform(context)
    use_safety_monitor = LaunchConfiguration('use_safety_monitor').perform(context)
    requested_backend = LaunchConfiguration('hardware_interface').perform(context).strip()

    # `hardware_interface` wins when it is given; otherwise use_sim decides.
    # "sim" is the user-facing name of the gazebo backend.
    if requested_backend:
        backend = requested_backend
    else:
        backend = 'sim' if use_sim else 'real'
    if backend not in ('sim', 'gazebo', 'mock', 'real'):
        raise RuntimeError(
            f"hardware_interface must be sim, mock or real (got '{backend}')")

    hardware_type = 'gazebo' if backend in ('sim', 'gazebo') else backend
    simulated = hardware_type == 'gazebo'
    use_sim_time = 'true' if simulated else 'false'

    actions = [
        LogInfo(msg=[
            '[robot_arm] backend: ', hardware_type,
            '   MoveIt: ', use_moveit,
            '   RViz: ', use_rviz,
            '   safety monitor: ', use_safety_monitor,
        ]),
    ]

    if simulated:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('robot_arm_simulation'),
                        'launch', 'simulation.launch.py'])
                ]),
                # RViz is started once, below, with the right configuration
                # for whether MoveIt is running.
                launch_arguments={
                    'use_rviz': 'false',
                    'gui': LaunchConfiguration('gazebo_gui'),
                    'world': LaunchConfiguration('world'),
                    'prefix': LaunchConfiguration('prefix'),
                }.items(),
            )
        )
    else:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('robot_arm_bringup'),
                        'launch', 'real_robot.launch.py'])
                ]),
                launch_arguments={
                    'hardware_type': hardware_type,
                    'use_rviz': 'false',
                    'use_safety_monitor': 'false',   # started here instead
                    'prefix': LaunchConfiguration('prefix'),
                }.items(),
            )
        )

    # The safety monitor is identical in both modes - that is the point of it.
    actions.append(
        Node(
            package='robot_arm_control',
            executable='safety_monitor',
            name='safety_monitor',
            output='screen',
            parameters=[
                LaunchConfiguration('safety_config'),
                {'use_sim_time': simulated},
            ],
            condition=IfCondition(use_safety_monitor),
        )
    )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robot_arm_bringup'), 'launch', 'moveit.launch.py'])
            ]),
            launch_arguments={
                'hardware_type': hardware_type,
                'use_sim_time': use_sim_time,
                'use_rviz': 'false',
                'planning_pipelines': LaunchConfiguration('planning_pipelines'),
            }.items(),
            condition=IfCondition(use_moveit),
        )
    )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robot_arm_bringup'), 'launch', 'rviz.launch.py'])
            ]),
            launch_arguments={
                'hardware_type': hardware_type,
                'use_sim_time': use_sim_time,
                # With MoveIt running, RViz gets the MotionPlanning panel.
                'use_moveit': use_moveit,
                'planning_pipelines': LaunchConfiguration('planning_pipelines'),
            }.items(),
            condition=IfCondition(use_rviz),
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim', default_value='true',
            description='true: Gazebo. false: the physical robot.'),
        DeclareLaunchArgument(
            'hardware_interface', default_value='',
            description='Override the backend: sim | mock | real. '
                        'Empty means "derive it from use_sim".'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true', description='Start RViz2.'),
        DeclareLaunchArgument(
            'use_moveit', default_value='true', description='Start MoveIt 2.'),
        DeclareLaunchArgument(
            'use_safety_monitor', default_value='true',
            description='Start the safety monitor (limits, e-stop, diagnostics).'),
        DeclareLaunchArgument(
            'prefix', default_value='', description='Frame/joint name prefix.'),
        DeclareLaunchArgument(
            'planning_pipelines', default_value='ompl',
            description='Space separated MoveIt planning pipelines.'),
        DeclareLaunchArgument(
            'gazebo_gui', default_value='true',
            description='Start the Gazebo client (simulation only).'),
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_arm_simulation'), 'worlds', 'robot_arm.world']),
            description='Gazebo world file (simulation only).'),
        DeclareLaunchArgument(
            'safety_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_arm_control'), 'config', 'safety.yaml']),
            description='Safety monitor configuration.'),
        OpaqueFunction(function=launch_setup),
    ])
