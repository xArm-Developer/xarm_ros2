#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    prefix = LaunchConfiguration('prefix', default='')
    ros_namespace = LaunchConfiguration('ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    dof = LaunchConfiguration('dof', default=7)

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
            "ros_namespace:=",
            ros_namespace,
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
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # joint state publisher node
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name='joint_state_publisher',
        output='screen'
    )

    # robot state publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
        # remappings=[
        #     ('/joint_states', PathJoinSubstitution(['/', ros_namespace, 'joint_states']))
        # ]
    )

    return LaunchDescription([joint_state_publisher_node, robot_state_publisher_node])
