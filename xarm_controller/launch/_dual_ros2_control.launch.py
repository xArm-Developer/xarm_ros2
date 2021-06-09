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


def add_prefix_to_ros2_control_params(prefix, ros2_control_params):
    if not prefix:
        return
    for name in list(ros2_control_params.keys()):
        if name == 'controller_manager':
            continue
        ros__parameters = ros2_control_params[name].get('ros__parameters', {})
        joints = ros__parameters.get('joints', [])
        constraints = ros__parameters.get('constraints', {})
        for i, joint in enumerate(joints):
            for j, key in enumerate(constraints.keys()):
                if key == joint:
                    constraints['{}{}'.format(prefix, key)] = constraints.pop(key)
                    break
            joints[i] = '{}{}'.format(prefix, joint)
        new_name = '{}{}'.format(prefix, name)
        ros2_control_params[new_name] = ros2_control_params.pop(name)
        controller_manager_ros__parameters = ros2_control_params.get('controller_manager', {}).get('ros__parameters', {})
        if name in controller_manager_ros__parameters:
            controller_manager_ros__parameters[new_name] = controller_manager_ros__parameters.pop(name)


def generate_ros2_control_params(ros2_control_params_path_1, ros2_control_params_path_2, prefix_1='L_', prefix_2='R_', ros_namespace=''):
    with open(ros2_control_params_path_1, 'r') as f:
        ros2_control_params_yaml_1 = yaml.safe_load(f)
    with open(ros2_control_params_path_2, 'r') as f:
        ros2_control_params_yaml_2 = yaml.safe_load(f)
    add_prefix_to_ros2_control_params(prefix_1, ros2_control_params_yaml_1)
    add_prefix_to_ros2_control_params(prefix_2, ros2_control_params_yaml_2)
    ros2_control_params_yaml = {}
    ros2_control_params_yaml.update(ros2_control_params_yaml_1)
    ros2_control_params_yaml.update(ros2_control_params_yaml_2)
    ros2_control_params_yaml['controller_manager']['ros__parameters'].update(ros2_control_params_yaml_1['controller_manager']['ros__parameters'])
    ros2_control_params_yaml['controller_manager']['ros__parameters'].update(ros2_control_params_yaml_2['controller_manager']['ros__parameters'])
    if ros_namespace:
        ros2_control_params_yaml = {
            ros_namespace: ros2_control_params_yaml
        }
    with NamedTemporaryFile(mode='w', prefix='launch_params_', delete=False) as h:
        yaml.dump(ros2_control_params_yaml, h, default_flow_style=False)
        return h.name


def launch_setup(context, *args, **kwargs):
    prefix_1 = LaunchConfiguration('prefix_1', default='L_')
    prefix_2 = LaunchConfiguration('prefix_2', default='R_')
    dof = LaunchConfiguration('dof', default=7)
    dof_1 = LaunchConfiguration('dof_1', default=dof)
    dof_2 = LaunchConfiguration('dof_2', default=dof)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_gripper_1 = LaunchConfiguration('add_gripper_1', default=add_gripper)
    add_gripper_2 = LaunchConfiguration('add_gripper_2', default=add_gripper)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_vacuum_gripper_1 = LaunchConfiguration('add_vacuum_gripper_1', default=add_vacuum_gripper)
    add_vacuum_gripper_2 = LaunchConfiguration('add_vacuum_gripper_2', default=add_vacuum_gripper)
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
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
                'dof_1': dof_1,
                'dof_2': dof_2,
                'add_gripper_1': add_gripper_1,
                'add_gripper_2': add_gripper_2,
                'add_vacuum_gripper_1': add_vacuum_gripper_1,
                'add_vacuum_gripper_2': add_vacuum_gripper_2,
                'hw_ns': hw_ns.perform(context).strip('/'),
                'limited': limited,
                'effort_control': effort_control,
                'velocity_control': velocity_control,
                'ros2_control_plugin': ros2_control_plugin,
            }
        )
    }

    # ros2 control node
    ros2_control_params = generate_ros2_control_params(
        os.path.join(get_package_share_directory('xarm_controller'), 'config', 'xarm{}_controllers.yaml'.format(dof_1.perform(context))),
        os.path.join(get_package_share_directory('xarm_controller'), 'config', 'xarm{}_controllers.yaml'.format(dof_2.perform(context))),
        prefix_1.perform(context),
        prefix_2.perform(context),
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
