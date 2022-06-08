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
from launch.actions import OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def merge_dict(dict1, dict2):
    for k, v in dict1.items():
        try:
            if k not in dict2:
                continue
            if isinstance(v, dict):
                merge_dict(v, dict2[k])
            else:
                dict1[k] = dict2[k]
        except Exception as e:
            pass


def load_yaml(path):
    if os.path.exists(path):
        try:
            with open(path, 'r') as file:
                return yaml.safe_load(file)
        except Exception as e:
            print('load {} error, {}'.format(path, e))
    return {}


def generate_xarm_params(xarm_default_params_path, xarm_user_params_path=None, ros_namespace=''):
    if not os.path.exists(xarm_user_params_path):
        xarm_user_params_path = None
    if ros_namespace or (xarm_user_params_path is not None and xarm_default_params_path != xarm_user_params_path):
        ros2_control_params_yaml = load_yaml(xarm_default_params_path)
        ros2_control_user_params_yaml = load_yaml(xarm_user_params_path)
        # change xarm_driver to ufactory_driver
        if 'xarm_driver' in ros2_control_params_yaml and 'ufactory_driver' not in ros2_control_params_yaml:
            ros2_control_params_yaml['ufactory_driver'] = ros2_control_params_yaml.pop('xarm_driver')
        if 'xarm_driver' in ros2_control_user_params_yaml and 'ufactory_driver' not in ros2_control_user_params_yaml:
            ros2_control_user_params_yaml['ufactory_driver'] = ros2_control_user_params_yaml.pop('xarm_driver')
        merge_dict(ros2_control_params_yaml, ros2_control_user_params_yaml)
        if ros_namespace:
            xarm_params_yaml = {
                ros_namespace: ros2_control_params_yaml
            }
        else:
            xarm_params_yaml = ros2_control_params_yaml
        with NamedTemporaryFile(mode='w', prefix='launch_params_', delete=False) as h:
            yaml.dump(xarm_params_yaml, h, default_flow_style=False)
            return h.name
    return xarm_default_params_path


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
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    prefix = LaunchConfiguration('prefix', default='')
    baud_checkset = LaunchConfiguration('baud_checkset', default=True)
    default_gripper_baud = LaunchConfiguration('default_gripper_baud', default=2000000)
    
    show_rviz = LaunchConfiguration('show_rviz', default=False)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    
    xarm_params = generate_xarm_params(
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_params.yaml'),
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_user_params.yaml'),
        LaunchConfiguration('ros_namespace', default='').perform(context)
    )
    
    # xarm driver node
    xarm_driver_node = Node(
        # namespace=hw_ns,
        package='xarm_api',
        name='ufactory_driver',
        executable='xarm_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            xarm_params,
            {
                'robot_ip': robot_ip,
                'report_type': report_type,
                'dof': dof,
                'add_gripper': add_gripper if robot_type.perform(context) == 'xarm' else False,
                'hw_ns': hw_ns.perform(context).strip('/'),
                'prefix': prefix.perform(context).strip('/'),
                'baud_checkset': baud_checkset,
                'default_gripper_baud': default_gripper_baud,
            },
        ]
    )

    nodes = [
        xarm_driver_node
    ]
    if show_rviz.perform(context) == 'true':
        # xarm rviz launch
        # xarm_description/launch/_xarm_rviz_display.launch.py
        xarm_rviz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_description'), 'launch', '_xarm_rviz_display.launch.py'])),
            launch_arguments={
                'prefix': prefix,
                'hw_ns': hw_ns,
                'add_gripper': add_gripper,
                'add_vacuum_gripper': add_vacuum_gripper,
                'dof': dof,
            }.items(),
        )
        nodes.append(xarm_rviz_launch)
    return nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
