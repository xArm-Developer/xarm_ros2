#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import sys
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import load_python_launch_file_as_module
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    dof = LaunchConfiguration('dof', default=7)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='uf_robot_hardware/UFRobotSystemHardware')
    joint_states_remapping = LaunchConfiguration('joint_states_remapping', default='joint_states')
    xacro_file = LaunchConfiguration('xacro_file', default=PathJoinSubstitution([FindPackageShare('xarm_description'), 'urdf', 'xarm_device.urdf.xacro']))

    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')

    # robot_description
    # xarm_description/launch/lib/robot_description_lib.py
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm_description'), 'launch', 'lib', 'robot_description_lib.py'))
    get_xacro_file_content = getattr(mod, 'get_xacro_file_content')
    robot_description = {
        'robot_description': get_xacro_file_content(
            xacro_file=xacro_file, 
            arguments={
                'prefix': prefix,
                'hw_ns': hw_ns.perform(context).strip('/'),
                'limited': limited,
                'effort_control': effort_control,
                'velocity_control': velocity_control,
                'add_gripper': add_gripper,
                'add_vacuum_gripper': add_vacuum_gripper,
                'dof': dof,
                'robot_type': robot_type,
                'ros2_control_plugin': ros2_control_plugin,
                'add_other_geometry': add_other_geometry,
                'geometry_type': geometry_type,
                'geometry_mass': geometry_mass,
                'geometry_height': geometry_height,
                'geometry_radius': geometry_radius,
                'geometry_length': geometry_length,
                'geometry_width': geometry_width,
                'geometry_mesh_filename': geometry_mesh_filename,
                'geometry_mesh_origin_xyz': geometry_mesh_origin_xyz,
                'geometry_mesh_origin_rpy': geometry_mesh_origin_rpy,
                'geometry_mesh_tcp_xyz': geometry_mesh_tcp_xyz,
                'geometry_mesh_tcp_rpy': geometry_mesh_tcp_rpy,
            }
        )
    }

    # robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        remappings=[
            # ('joint_states', joint_states_remapping),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )
    return [
        robot_state_publisher_node
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
