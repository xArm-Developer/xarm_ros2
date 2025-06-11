#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from uf_ros_lib.uf_robot_utils import  generate_robot_api_params


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
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    prefix = LaunchConfiguration('prefix', default='')
    baud_checkset = LaunchConfiguration('baud_checkset', default=True)
    default_gripper_baud = LaunchConfiguration('default_gripper_baud', default=2000000)
    joint_states_rate = LaunchConfiguration('joint_states_rate', default=-1)
    
    show_rviz = LaunchConfiguration('show_rviz', default=False)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    extra_robot_api_params_path = LaunchConfiguration('extra_robot_api_params_path', default='')
    
    robot_params = generate_robot_api_params(
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_params.yaml'),
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_user_params.yaml'),
        LaunchConfiguration('ros_namespace', default='').perform(context), node_name='ufactory_driver',
        extra_robot_api_params_path=extra_robot_api_params_path.perform(context)
    )
    
    # robot driver node
    robot_driver_node = Node(
        # namespace=hw_ns,
        package='xarm_api',
        name='ufactory_driver',
        executable='xarm_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            robot_params,
            {
                'robot_ip': robot_ip,
                'report_type': report_type,
                'dof': dof,
                'add_gripper': add_gripper if robot_type.perform(context) != 'lite' else False,
                'add_bio_gripper': add_bio_gripper,
                'hw_ns': '{}{}'.format(prefix.perform(context).strip('/'), hw_ns.perform(context).strip('/')),
                'prefix': prefix.perform(context).strip('/'),
                'baud_checkset': baud_checkset,
                'default_gripper_baud': default_gripper_baud,
                'joint_states_rate': joint_states_rate,
            },
        ]
    )

    nodes = [
        robot_driver_node
    ]
    if show_rviz.perform(context) == 'true':
        # robot rviz launch
        # xarm_description/launch/_robot_rviz_display.launch.py
        robot_rviz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_description'), 'launch', '_robot_rviz_display.launch.py'])),
            launch_arguments={
                'prefix': prefix,
                'hw_ns': hw_ns,
                'add_gripper': add_gripper,
                'add_vacuum_gripper': add_vacuum_gripper,
                'dof': dof,
            }.items(),
        )
        nodes.append(robot_rviz_launch)
    return nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
