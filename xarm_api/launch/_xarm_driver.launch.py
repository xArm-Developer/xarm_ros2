#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import yaml
from tempfile import NamedTemporaryFile
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_xarm_params(xarm_params_path, ros_namespace=''):
    if ros_namespace:
        with open(xarm_params_path, 'r') as f:
            ros2_control_params_yaml = yaml.safe_load(f)
        xarm_params_yaml = {
            ros_namespace: ros2_control_params_yaml
        }
        with NamedTemporaryFile(mode='w', prefix='launch_params_', delete=False) as h:
            yaml.dump(xarm_params_yaml, h, default_flow_style=False)
            return h.name
    return xarm_params_path


def launch_setup(context, *args, **kwargs):
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
            'dof',
            default_value='7',
            description='Degree of freedom of manipulator, defalut is 7.',
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
    dof = LaunchConfiguration('dof', default=7)
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    prefix = LaunchConfiguration('prefix', default='')
    
    xarm_params = generate_xarm_params(
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_params.yaml'),
        LaunchConfiguration('ros_namespace', default='').perform(context)
    )
    
    # xarm driver node
    xarm_driver_node = Node(
        # namespace=hw_ns,
        package='xarm_api',
        name='xarm_driver',
        executable='xarm_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            xarm_params,
            {
                'robot_ip': robot_ip,
                'report_type': report_type,
                'dof': dof,
                'add_gripper': add_gripper,
                'hw_ns': hw_ns.perform(context).strip('/'),
                'prefix': prefix.perform(context).strip('/'),
            },
        ]
    )

    return [
        xarm_driver_node
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
