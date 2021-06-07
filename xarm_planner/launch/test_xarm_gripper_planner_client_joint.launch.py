#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    xarm_planner_node_test = Node(
        name='test_xarm_gripper_planner_client_joint',
        package='xarm_planner',
        executable='test_xarm_gripper_planner_client_joint',
        output='screen',
    )
    return LaunchDescription([
        xarm_planner_node_test
    ])
