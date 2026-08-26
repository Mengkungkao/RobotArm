# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
Everything, in simulation.

    ros2 launch robot_arm_bringup sim.launch.py

Equivalent to `bringup.launch.py use_sim:=true`, and accepts the same
arguments.  The counterpart is real.launch.py - switching between them is the
only change needed to move an application from Gazebo to the machine.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# Arguments forwarded verbatim to bringup.launch.py, so that
#   ros2 launch robot_arm_bringup sim.launch.py use_rviz:=false
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
        arguments = {'use_sim': 'true'}
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
