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
from launch.actions import OpaqueFunction

def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='ufactory')
    lite_gripper_node_test = Node(
        name='test_lite_gripper_realmove',
        package='xarm_planner',
        executable='test_lite_gripper_realmove',
        output='screen',
        parameters=[
            {
                'prefix': prefix,
                'hw_ns': hw_ns
            },
        ],
    )
    return [
        lite_gripper_node_test
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
    
