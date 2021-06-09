#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import load_python_launch_file_as_module
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot1_ip = LaunchConfiguration('robot1_ip')
    robot2_ip = LaunchConfiguration('robot2_ip')
    report_type = LaunchConfiguration('report_type', default='normal')
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
    no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=False)

    ros2_control_plugin = 'xarm_control/XArmHW'
    controllers_name = 'controllers'
    moveit_controller_manager_key = 'moveit_simple_controller_manager'
    moveit_controller_manager_value = 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    xarm_type = 'xarm{}'.format(dof.perform(context))
    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)

    # xarm driver launch
    xarm_driver_launch_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_api'), 'launch', '_xarm_driver.launch.py'])),
        launch_arguments={
            'robot_ip': robot1_ip,
            'report_type': report_type,
            'dof': dof_1,
            'hw_ns': hw_ns,
            'add_gripper': add_gripper_1,
            'prefix': prefix_1,
        }.items(),
    )
    # xarm driver launch
    xarm_driver_launch_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_api'), 'launch', '_xarm_driver.launch.py'])),
        launch_arguments={
            'robot_ip': robot2_ip,
            'report_type': report_type,
            'dof': dof_2,
            'hw_ns': hw_ns,
            'add_gripper': add_gripper_2,
            'prefix': prefix_2,
        }.items(),
    )

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

    # robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output="screen",
        parameters=[robot_description],
        remappings=[
            # ('joint_states', joint_states_remapping),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

   # xarm moveit common launch
    xarm_moveit_common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_dual_xarm_moveit_common.launch.py'])),
        launch_arguments={
            'prefix_1': prefix_1,
            'prefix_2': prefix_2,
            'dof_1': dof_1,
            'dof_2': dof_2,
            'add_gripper_1': add_gripper_1,
            'add_gripper_2': add_gripper_2,
            'add_vacuum_gripper_1': add_vacuum_gripper_1,
            'add_vacuum_gripper_2': add_vacuum_gripper_2,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'no_gui_ctrl': no_gui_ctrl,
            'ros2_control_plugin': ros2_control_plugin,
            'controllers_name': controllers_name,
            'moveit_controller_manager_key': moveit_controller_manager_key,
            'moveit_controller_manager_value': moveit_controller_manager_value
        }.items(),
    )

    # joint state publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'source_list': [
                '{}{}/joint_states'.format(prefix_1.perform(context), hw_ns.perform(context)),
                '{}{}/joint_states'.format(prefix_2.perform(context), hw_ns.perform(context))
            ], 
        }],
        remappings=[
            ('follow_joint_trajectory', '{}xarm{}_traj_controller/follow_joint_trajectory'.format(prefix_1.perform(context), dof_1.perform(context))),
            ('follow_joint_trajectory', '{}xarm{}_traj_controller/follow_joint_trajectory'.format(prefix_2.perform(context), dof_2.perform(context))),
        ],
    )

    ros2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_controller'), 'launch', '_dual_ros2_control.launch.py'])),
        launch_arguments={
            'prefix_1': prefix_1,
            'prefix_2': prefix_2,
            'dof': dof,
            'dof_1': dof_1,
            'dof_2': dof_2,
            'add_gripper': add_gripper,
            'add_gripper_1': add_gripper_1,
            'add_gripper_2': add_gripper_2,
            'add_vacuum_gripper': add_vacuum_gripper,
            'add_vacuum_gripper_1': add_vacuum_gripper_1,
            'add_vacuum_gripper_2': add_vacuum_gripper_2,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'ros2_control_plugin': ros2_control_plugin,
        }.items(),
    )

    control_node_1 = Node(
        package='controller_manager',
        executable='spawner.py',
        output='screen',
        arguments=[
            '{}xarm{}_traj_controller'.format(prefix_1.perform(context), dof_1.perform(context)),
            '--controller-manager', '{}/controller_manager'.format(ros_namespace)
        ],
    )
    control_node_2 = Node(
        package='controller_manager',
        executable='spawner.py',
        output='screen',
        arguments=[
            '{}xarm{}_traj_controller'.format(prefix_2.perform(context), dof_2.perform(context)),
            '--controller-manager', '{}/controller_manager'.format(ros_namespace)
        ],
    )

    return [
        robot_state_publisher_node,
        xarm_moveit_common_launch,
        joint_state_publisher_node,
        ros2_launch,
        control_node_1,
        control_node_2,
        xarm_driver_launch_1,
        xarm_driver_launch_2,
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
