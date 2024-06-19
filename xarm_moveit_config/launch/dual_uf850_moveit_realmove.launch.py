#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip_1 = LaunchConfiguration('robot_ip_1')
    robot_ip_2 = LaunchConfiguration('robot_ip_2')
    hw_ns = LaunchConfiguration('hw_ns', default='ufactory')

    # robot moveit realmove launch
    # xarm_moveit_config/launch/_dual_robot_moveit_realmove.launch.py
    robot_moveit_realmove_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_dual_robot_moveit_realmove.launch.py'])),
        launch_arguments={
            'robot_ip_1': robot_ip_1,
            'robot_ip_2': robot_ip_2,
            'dof': '6',
            'robot_type': 'uf850',
            'hw_ns': hw_ns,
            'no_gui_ctrl': 'false',
        }.items(),
    )
    
    return LaunchDescription([
        robot_moveit_realmove_launch
    ])
