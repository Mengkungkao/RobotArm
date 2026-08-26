# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
RViz2 on its own.

    ros2 launch robot_arm_bringup rviz.launch.py                  # robot + TF
    ros2 launch robot_arm_bringup rviz.launch.py use_moveit:=true # + MotionPlanning

With `use_moveit:=true` RViz is started from the MoveIt configuration, so it
gets the SRDF and the kinematics it needs for the interactive goal marker.
Without it, a plain viewer for the model, TF and joint states is started -
which is what you want when MoveIt is not running at all.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_moveit = LaunchConfiguration('use_moveit')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_moveit', default_value='false',
            description='Load the MoveIt MotionPlanning panel.'),
        DeclareLaunchArgument(
            'hardware_type', default_value='gazebo',
            choices=['gazebo', 'mock', 'real'],
            description='Backend variant of the URDF (MoveIt panel only).'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the simulator clock.'),
        DeclareLaunchArgument(
            'planning_pipelines', default_value='ompl',
            description='Space separated MoveIt planning pipelines.'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_arm_description'), 'rviz', 'view_robot.rviz']),
            description='Configuration used when use_moveit is false.'),

        # Plain viewer: model, TF, joint states.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=UnlessCondition(use_moveit),
        ),

        # MoveIt viewer: adds the planning scene, the trajectory preview and
        # the interactive goal marker.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robot_arm_moveit_config'),
                    'launch', 'moveit_rviz.launch.py'])
            ]),
            launch_arguments={
                'hardware_type': LaunchConfiguration('hardware_type'),
                'use_sim_time': use_sim_time,
                'planning_pipelines': LaunchConfiguration('planning_pipelines'),
            }.items(),
            condition=IfCondition(use_moveit),
        ),
    ])
