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
from launch.launch_description_sources import load_python_launch_file_as_module
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_ros2_controll_params(ros2_control_params_path, ros_namespace=''):
    if ros_namespace:
        with open(ros2_control_params_path, 'r') as f:
            ros2_control_params_yaml = yaml.safe_load(f)
        ros2_control_params_yaml = {
            ros_namespace: ros2_control_params_yaml
        }
        with NamedTemporaryFile(mode='w', prefix='launch_params_', delete=False) as h:
            yaml.dump(ros2_control_params_yaml, h, default_flow_style=False)
            return h.name
    return ros2_control_params_path


def launch_setup(context, *args, **kwargs):
    prefix_1 = LaunchConfiguration('prefix_1', default='L_')
    prefix_2 = LaunchConfiguration('prefix_2', default='R_')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    dof = LaunchConfiguration('dof', default=7)
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='xarm_control/XArmHW')

    # robot_description
    xacro_file = PathJoinSubstitution([FindPackageShare('xarm_description'), 'urdf', 'dual_xarm_device.urdf.xacro'])
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm_description'), 'launch', 'lib', 'xarm_description_lib.py'))
    get_xacro_file_content = getattr(mod, 'get_xacro_file_content')
    robot_description = {
        'robot_description': get_xacro_file_content(
            xacro_file=xacro_file, 
            arguments={
                'prefix_1': prefix_1,
                'prefix_2': prefix_2,
                'hw_ns': hw_ns.perform(context).strip('/'),
                'limited': limited,
                'effort_control': effort_control,
                'velocity_control': velocity_control,
                'add_gripper': add_gripper,
                'add_vacuum_gripper': add_vacuum_gripper,
                'dof': dof,
                'ros2_control_plugin': ros2_control_plugin,
            }
        )
    }

    # ros2 control node
    ros2_control_params = generate_ros2_controll_params(
        os.path.join(get_package_share_directory('xarm_controller'), 'config', 'dual_xarm{}_controllers.yaml'.format(dof.perform(context))),
        LaunchConfiguration('ros_namespace', default='').perform(context)
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            ros2_control_params,
        ],
        output='screen',
    )
    return [ros2_control_node]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
