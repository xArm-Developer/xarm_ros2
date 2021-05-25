#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot1_ip = LaunchConfiguration('robot1_ip')
    robot2_ip = LaunchConfiguration('robot2_ip')
    report_type = LaunchConfiguration('report_type', default='normal')
    ns1 = LaunchConfiguration('ns1', default='left')
    ns2 = LaunchConfiguration('ns2', default='right')
    prefix = LaunchConfiguration('prefix', default='')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)

    dof = 7
    hw_ns = 'xarm'
    xarm_moveit_realmove_launch_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_xarm_moveit_realmove.launch.py'])),
        launch_arguments={
            'robot_ip': robot1_ip,
            'report_type': report_type,
            'prefix': prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'dof': str(dof),
            'no_gui_ctrl': 'false',
            'ros_namespace': ns1
        }.items(),
    )
    xarm_moveit_realmove_launch_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_xarm_moveit_realmove.launch.py'])),
        launch_arguments={
            'robot_ip': robot2_ip,
            'report_type': report_type,
            'prefix': prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'dof': str(dof),
            'no_gui_ctrl': 'false',
            'ros_namespace': ns2
        }.items(),
    )

    return LaunchDescription([
        xarm_moveit_realmove_launch_1,
        xarm_moveit_realmove_launch_2,
    ])
