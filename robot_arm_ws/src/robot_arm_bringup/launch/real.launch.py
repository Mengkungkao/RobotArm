# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Everything, on the physical robot.

    ros2 launch robot_arm_bringup real.launch.py

Equivalent to `bringup.launch.py use_sim:=false`, and accepts the same
arguments.  MoveIt, the controllers and every application see exactly what
they saw in simulation.

Check robot_arm_hardware/config/hardware.yaml before the first run: the
default transport is the loopback bus, which drives nothing.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# Arguments forwarded verbatim to bringup.launch.py, so that
#   ros2 launch robot_arm_bringup real.launch.py use_rviz:=false
# behaves exactly like the same option on bringup.launch.py.
FORWARDED_ARGUMENTS = [
    'use_rviz',
    'use_moveit',
    'use_safety_monitor',
    'hardware_interface',
    'prefix',
    'planning_pipelines',
    'gazebo_gui',
    'world',
    'safety_config',
]


def generate_launch_description():
    bringup = PathJoinSubstitution(
        [FindPackageShare('robot_arm_bringup'), 'launch', 'bringup.launch.py'])

    # Only forward the arguments the user actually set, so bringup.launch.py
    # keeps its own defaults for the rest.
    def forward(context, *args, **kwargs):
        arguments = {'use_sim': 'false'}
        for name in FORWARDED_ARGUMENTS:
            value = context.launch_configurations.get(name, '')
            if value != '':
                arguments[name] = value
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([bringup]),
                launch_arguments=arguments.items(),
            )
        ]

    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value='',
                               description='forwarded to bringup.launch.py')
         for name in FORWARDED_ARGUMENTS]
        + [OpaqueFunction(function=forward)]
    )
