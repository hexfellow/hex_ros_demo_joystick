#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-06-30
################################################################

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_pkg_path = FindPackageShare('hex_ros_sim_archer_y6')
    joystick_pkg_path = FindPackageShare('hex_ros_teleop_joystick')
    joy_arm_pkg_path = FindPackageShare('hex_ros_demo_joystick')
    urdf_pkg_path = FindPackageShare('hex_ros_urdf_archer_y6')

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    viewer_arg = DeclareLaunchArgument(
        name='viewer',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to turn on the MuJoCo viewer')
    rviz_arg = DeclareLaunchArgument(
        name='rviz',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to turn on RViz')
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to use simulation time')

    # ------------------------------------------------------------------
    # MuJoCo simulation
    # ------------------------------------------------------------------
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                sim_pkg_path,
                'sim_archer_y6.launch.py',
            ])),
        launch_arguments={
            'viewer': LaunchConfiguration('viewer'),
            'rviz': LaunchConfiguration("rviz"),
            'test': 'false',
        }.items(),
    )

    # ------------------------------------------------------------------
    # Joystick teleoperation
    # ------------------------------------------------------------------
    joystick_param_path = PathJoinSubstitution([
        joystick_pkg_path,
        'config',
        'ros2',
        'params.yaml',
    ])
    joystick_node = Node(
        package='hex_ros_teleop_joystick',
        executable='teleop_joystick',
        name='teleop_joystick',
        output='screen',
        emulate_tty=True,
        parameters=[
            joystick_param_path,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        remappings=[
            ('teleop_joy_state', 'teleop_joy_state'),
        ],
    )

    # ------------------------------------------------------------------
    # Joystick arm controller
    # ------------------------------------------------------------------
    joy_arm_param_path = PathJoinSubstitution([
        joy_arm_pkg_path,
        'config',
        'ros2',
        'joy_arm.yaml',
    ])
    urdf_file_path = PathJoinSubstitution([
        urdf_pkg_path,
        'urdf',
        'empty.urdf',
    ])
    joy_arm_node = Node(
        package='hex_ros_demo_joystick',
        executable='joy_arm',
        name='joy_arm',
        output='screen',
        emulate_tty=True,
        parameters=[
            joy_arm_param_path,
            {
                'model_urdf': ParameterValue(
                    urdf_file_path,
                    value_type=str,
                ),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        remappings=[
            ('manip_state', 'manip_state'),
            ('manip_ctrl', 'manip_ctrl'),
            ('teleop_joy_state', 'teleop_joy_state'),
        ],
    )

    # ------------------------------------------------------------------
    # RViz and robot state publisher
    # ------------------------------------------------------------------
    rviz_config_path = PathJoinSubstitution([
        joy_arm_pkg_path,
        'config',
        'ros2',
        'joy_arm.rviz',
    ])
    visual_urdf_path = PathJoinSubstitution([
        urdf_pkg_path,
        'urdf',
        'gr100_full.urdf',
    ])
    robot_description = ParameterValue(
        ['xacro ', visual_urdf_path],
        value_type=str,
    )
    rviz_group = GroupAction(
        actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': robot_description,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_path],
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }],
            ),
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        viewer_arg,
        rviz_arg,
        use_sim_time_arg,
        sim_launch,
        joystick_node,
        joy_arm_node,
        rviz_group,
    ])
