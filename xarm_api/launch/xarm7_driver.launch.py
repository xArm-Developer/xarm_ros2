#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_ip',
            description='IP address by which the robot can be reached.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'report_type',
            default_value='normal',
            description='Tcp report type, default is normal, normal/rich/dev optional.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'hw_ns',
            default_value='xarm',
            description='The namespace of xarm_driver, default is xarm.',
        )
    )

    # Initialize Arguments
    robot_ip = LaunchConfiguration('robot_ip')
    report_type = LaunchConfiguration('report_type', default='normal')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    show_rviz = LaunchConfiguration('show_rviz', default=False)

    # robot driver launch
    # xarm_api/launch/_robot_driver.launch.py
    robot_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/_robot_driver.launch.py']),
        launch_arguments={
            'robot_ip': robot_ip,
            'report_type': report_type,
            'dof': '7',
            'hw_ns': hw_ns,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'show_rviz': show_rviz,
            'robot_type': 'xarm',
        }.items(),
    )

    return LaunchDescription(declared_arguments + [robot_driver_launch])
