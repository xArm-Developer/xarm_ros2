#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import sys
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    dof = LaunchConfiguration('dof')

    xarm_simple_planner_test_node = Node(
        name="xarm_simple_planner_joint_test",
        package="xarm_planner",
        executable="xarm_simple_planner_joint_test",
        output="screen",
        parameters=[
            {'DOF': dof},
        ],
    )

    return LaunchDescription([
        xarm_simple_planner_test_node
    ])
