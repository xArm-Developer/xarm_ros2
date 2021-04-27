#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def get_xarm_robot_description(
    prefix,
    hw_ns,
    limited, 
    effort_control,
    velocity_control, 
    add_gripper,
    add_vacuum_gripper,
    dof,
    ros2_control_plugin):
    
    # robot_description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare('xarm_description'), 'urdf', 'xarm_device.urdf.xacro']),
            " ",
            "prefix:=",
            prefix,
            " ",
            "hw_ns:=",
            hw_ns,
            " ",
            "limited:=",
            limited,
            " ",
            "effort_control:=",
            effort_control,
            " ",
            "velocity_control:=",
            velocity_control,
            " ",
            "add_gripper:=",
            add_gripper,
            " ",
            "add_vacuum_gripper:=",
            add_vacuum_gripper,
            " ",
            "dof:=",
            dof,
            " ",
            "ros2_control_plugin:=",
            ros2_control_plugin,
            " ",
        ]
    )
    return {"robot_description": robot_description_content}
