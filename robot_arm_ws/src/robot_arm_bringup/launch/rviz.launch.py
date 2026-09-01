# Copyright (c) 2026 robot_arm_ws contributors
# SPDX-License-Identifier: MIT
"""
RViz2 on its own.

    # attach to a stack that is already running (bringup, sim or real robot)
    ros2 launch robot_arm_bringup rviz.launch.py
    ros2 launch robot_arm_bringup rviz.launch.py use_moveit:=true

    # nothing else running: RViz plus the joint sliders, and a robot to move
    ros2 launch robot_arm_bringup rviz.launch.py standalone:=true

With `use_moveit:=true` RViz is started from the MoveIt configuration, so it
gets the SRDF and the kinematics it needs for the interactive goal marker.
Without it, a plain viewer for the model, TF and joint states is started -
which is what you want when MoveIt is not running at all.

`standalone:=true` additionally brings up the things that normally come from
the rest of the stack: robot_state_publisher, the world -> base_link frame,
and a joint state source - joint_state_publisher_gui by default, so the
sliders come up next to the RViz window and the arm can be posed by hand.

Leave `standalone` at false whenever controllers are running.  The
joint_state_broadcaster already owns /joint_states there, and a second
publisher on that topic makes the model twitch between two sources.
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_moveit = LaunchConfiguration('use_moveit')
    use_sim_time = LaunchConfiguration('use_sim_time')
    standalone = LaunchConfiguration('standalone')
    use_gui = LaunchConfiguration('use_gui')

    # Expanded exactly as in bringup, sim and real - only the arguments differ.
    # Used only in standalone mode; otherwise robot_state_publisher is already
    # running and publishing this on /robot_description.
    robot_description = ParameterValue(
        Command([
            'xacro ', PathJoinSubstitution(
                [FindPackageShare('robot_arm_description'),
                 'urdf', 'robot_arm.urdf.xacro']),
            ' hardware_type:=', LaunchConfiguration('hardware_type'),
            ' prefix:=', LaunchConfiguration('prefix'),
            ' use_world_frame:=false',
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_moveit', default_value='false',
            description='Load the MoveIt MotionPlanning panel.'),
        DeclareLaunchArgument(
            'hardware_type', default_value='gazebo',
            choices=['gazebo', 'mock', 'real'],
            description='Backend variant of the URDF.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the simulator clock.'),
        DeclareLaunchArgument(
            'planning_pipelines', default_value='ompl',
            description='Space separated MoveIt planning pipelines.'),
        DeclareLaunchArgument(
            'prefix', default_value='',
            description='Frame/joint name prefix, e.g. "left_".'),
        DeclareLaunchArgument(
            'standalone', default_value='false',
            description='Also start robot_state_publisher and a joint state '
                        'source, so RViz has a robot to draw with no '
                        'controllers running.  Leave false when they are.'),
        DeclareLaunchArgument(
            'use_gui', default_value='true',
            description='Standalone only: joint_state_publisher_gui sliders '
                        'rather than the headless joint_state_publisher.'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('robot_arm_description'), 'rviz', 'view_robot.rviz']),
            description='Configuration used when use_moveit is false.'),

        # --------------------------------------------------------------
        # Standalone extras.  The whole group is gated on `standalone`, so
        # none of it starts in the modes where the stack already provides
        # these - one condition, rather than one per node.
        # --------------------------------------------------------------
        GroupAction(
            condition=IfCondition(standalone),
            actions=[
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{'robot_description': robot_description,
                                 'use_sim_time': use_sim_time}],
                ),
                # The model root is base_link; `world` is provided here so
                # RViz can keep the same fixed frame it uses in every mode.
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='world_to_base_link',
                    output='log',
                    arguments=['--frame-id', 'world',
                               '--child-frame-id', 'base_link'],
                    parameters=[{'use_sim_time': use_sim_time}],
                ),
                # The sliders: one per movable joint, limits read from the
                # URDF, published on /joint_states for RViz to follow.
                Node(
                    package='joint_state_publisher_gui',
                    executable='joint_state_publisher_gui',
                    name='joint_state_publisher_gui',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                    condition=IfCondition(use_gui),
                ),
                # Same topic, no window: holds every joint at zero.  Useful
                # over ssh, and in tests.
                Node(
                    package='joint_state_publisher',
                    executable='joint_state_publisher',
                    name='joint_state_publisher',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                    condition=UnlessCondition(use_gui),
                ),
            ],
        ),

        # --------------------------------------------------------------
        # The viewer itself.
        # --------------------------------------------------------------
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
