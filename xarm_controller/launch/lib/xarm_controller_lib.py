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
from tempfile import NamedTemporaryFile
from ament_index_python import get_package_share_directory
from ros2launch.api.api import parse_launch_arguments
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription


def get_sys_param(name, default=None):
    launch_arguments_dict = dict(parse_launch_arguments(sys.argv[4:]))
    return launch_arguments_dict.get(name, default)


def get_controller_params(dof, name='ros_namespace', default=None):
    package_path = get_package_share_directory('xarm_controller')
    controller_params = os.path.join(package_path, 'config', 'xarm{}_controllers.yaml'.format(dof))
    ros_namespace = get_sys_param(name, default=default)
    if ros_namespace:
        with open(controller_params, 'r') as f:
            controller_params_yaml = yaml.safe_load(f)
        controller_params_yaml = {
            ros_namespace: controller_params_yaml
        }
        with NamedTemporaryFile(mode='w', prefix='launch_params_', delete=False) as h:
            param_file_path = h.name
            yaml.dump(controller_params_yaml, h, default_flow_style=False)
            return param_file_path
    return controller_params


def get_xarm_ros2_control_launch_description(
    robot_ip, report_type,
    prefix, hw_ns, limited, 
    effort_control, velocity_control, 
    add_gripper, add_vacuum_gripper,
    dof='7',
    ros_namespace_name='ros_namespace', ros_namespace_default_value=''):
    ros_namespace = get_sys_param(ros_namespace_name, default=ros_namespace_default_value)
    joint_states_remapping = PathJoinSubstitution(['/', ros_namespace, hw_ns, 'joint_states'])
    controller_params = get_controller_params(dof, name=ros_namespace_name, default=ros_namespace_default_value)
    
    xarm_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_controller'), 'launch', '_xarm_ros2_control.launch.py'])),
        launch_arguments={
            'robot_ip': robot_ip,
            'report_type': report_type,
            'prefix': prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'dof': dof,
            'controller_params': controller_params,
            'joint_states_remapping': joint_states_remapping,
            'ros_namespace': ros_namespace
        }.items(),
    )
    return xarm_control_launch
