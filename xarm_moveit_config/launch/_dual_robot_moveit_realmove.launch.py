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
    robot_ip_1 = LaunchConfiguration('robot_ip_1')
    robot_ip_2 = LaunchConfiguration('robot_ip_2')
    report_type = LaunchConfiguration('report_type', default='normal')
    report_type_1 = LaunchConfiguration('report_type_1', default=report_type)
    report_type_2 = LaunchConfiguration('report_type_2', default=report_type)
    prefix_1 = LaunchConfiguration('prefix_1', default='L_')
    prefix_2 = LaunchConfiguration('prefix_2', default='R_')
    dof = LaunchConfiguration('dof', default=7)
    dof_1 = LaunchConfiguration('dof_1', default=dof)
    dof_2 = LaunchConfiguration('dof_2', default=dof)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    robot_type_1 = LaunchConfiguration('robot_type_1', default=robot_type)
    robot_type_2 = LaunchConfiguration('robot_type_2', default=robot_type)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_gripper_1 = LaunchConfiguration('add_gripper_1', default=add_gripper)
    add_gripper_2 = LaunchConfiguration('add_gripper_2', default=add_gripper)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_vacuum_gripper_1 = LaunchConfiguration('add_vacuum_gripper_1', default=add_vacuum_gripper)
    add_vacuum_gripper_2 = LaunchConfiguration('add_vacuum_gripper_2', default=add_vacuum_gripper)
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=False)

    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    add_other_geometry_1 = LaunchConfiguration('add_other_geometry_1', default=add_other_geometry)
    add_other_geometry_2 = LaunchConfiguration('add_other_geometry_2', default=add_other_geometry)

    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_type_1 = LaunchConfiguration('geometry_type_1', default=geometry_type)
    geometry_type_2 = LaunchConfiguration('geometry_type_2', default=geometry_type)

    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_mass_1 = LaunchConfiguration('geometry_mass_1', default=geometry_mass)
    geometry_mass_2 = LaunchConfiguration('geometry_mass_2', default=geometry_mass)

    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_height_1 = LaunchConfiguration('geometry_height_1', default=geometry_height)
    geometry_height_2 = LaunchConfiguration('geometry_height_2', default=geometry_height)

    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_radius_1 = LaunchConfiguration('geometry_radius_1', default=geometry_radius)
    geometry_radius_2 = LaunchConfiguration('geometry_radius_2', default=geometry_radius)
    
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_length_1 = LaunchConfiguration('geometry_length_1', default=geometry_length)
    geometry_length_2 = LaunchConfiguration('geometry_length_2', default=geometry_length)

    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_width_1 = LaunchConfiguration('geometry_width_1', default=geometry_width)
    geometry_width_2 = LaunchConfiguration('geometry_width_2', default=geometry_width)

    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_filename_1 = LaunchConfiguration('geometry_mesh_filename_1', default=geometry_mesh_filename)
    geometry_mesh_filename_2 = LaunchConfiguration('geometry_mesh_filename_2', default=geometry_mesh_filename)
    
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_xyz_1 = LaunchConfiguration('geometry_mesh_origin_xyz_1', default=geometry_mesh_origin_xyz)
    geometry_mesh_origin_xyz_2 = LaunchConfiguration('geometry_mesh_origin_xyz_2', default=geometry_mesh_origin_xyz)
    
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_origin_rpy_1 = LaunchConfiguration('geometry_mesh_origin_rpy_1', default=geometry_mesh_origin_rpy)
    geometry_mesh_origin_rpy_2 = LaunchConfiguration('geometry_mesh_origin_rpy_2', default=geometry_mesh_origin_rpy)
    
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_xyz_1 = LaunchConfiguration('geometry_mesh_tcp_xyz_1', default=geometry_mesh_tcp_xyz)
    geometry_mesh_tcp_xyz_2 = LaunchConfiguration('geometry_mesh_tcp_xyz_2', default=geometry_mesh_tcp_xyz)
    
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')
    geometry_mesh_tcp_rpy_1 = LaunchConfiguration('geometry_mesh_tcp_rpy_1', default=geometry_mesh_tcp_rpy)
    geometry_mesh_tcp_rpy_2 = LaunchConfiguration('geometry_mesh_tcp_rpy_2', default=geometry_mesh_tcp_rpy)

    baud_checkset = LaunchConfiguration('baud_checkset', default=True)
    baud_checkset_1 = LaunchConfiguration('baud_checkset', default=baud_checkset)
    baud_checkset_2 = LaunchConfiguration('baud_checkset', default=baud_checkset)
    default_gripper_baud = LaunchConfiguration('default_gripper_baud', default=2000000)
    default_gripper_baud_1 = LaunchConfiguration('default_gripper_baud', default=default_gripper_baud)
    default_gripper_baud_2 = LaunchConfiguration('default_gripper_baud', default=default_gripper_baud)

    ros2_control_plugin = 'uf_robot_hardware/UFRobotSystemHardware'
    controllers_name = 'controllers'
    moveit_controller_manager_key = 'moveit_simple_controller_manager'
    moveit_controller_manager_value = 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)

    # # robot driver launch
    # # xarm_api/launch/_robot_driver.launch.py
    # robot_driver_launch_1 = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_api'), 'launch', '_robot_driver.launch.py'])),
    #     launch_arguments={
    #         'robot_ip': robot_ip_1,
    #         'report_type': report_type,
    #         'dof': dof_1,
    #         'hw_ns': hw_ns,
    #         'add_gripper': add_gripper_1,
    #         'prefix': prefix_1,
    #         'baud_checkset': baud_checkset_1,
    #         'default_gripper_baud': default_gripper_baud_1,
    #         'robot_type': robot_type_1,
    #     }.items(),
    # )
    # # robot driver launch
    # # xarm_api/launch/_robot_driver.launch.py
    # robot_driver_launch_2 = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_api'), 'launch', '_robot_driver.launch.py'])),
    #     launch_arguments={
    #         'robot_ip': robot_ip_2,
    #         'report_type': report_type,
    #         'dof': dof_2,
    #         'hw_ns': hw_ns,
    #         'add_gripper': add_gripper_2,
    #         'prefix': prefix_2,
    #         'baud_checkset': baud_checkset_2,
    #         'default_gripper_baud': default_gripper_baud_2,
    #         'robot_type': robot_type_2,
    #     }.items(),
    # )

    # robot_description
    # xarm_description/launch/lib/robot_description_lib.py
    xacro_file = PathJoinSubstitution([FindPackageShare('xarm_description'), 'urdf', 'dual_xarm_device.urdf.xacro'])
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm_description'), 'launch', 'lib', 'robot_description_lib.py'))
    get_xacro_file_content = getattr(mod, 'get_xacro_file_content')
    robot_description = {
        'robot_description': get_xacro_file_content(
            xacro_file=xacro_file, 
            arguments={
                'prefix_1': prefix_1,
                'prefix_2': prefix_2,
                'dof_1': dof_1,
                'dof_2': dof_2,
                'robot_type_1': robot_type_1,
                'robot_type_2': robot_type_2,
                'add_gripper_1': add_gripper_1,
                'add_gripper_2': add_gripper_2,
                'add_vacuum_gripper_1': add_vacuum_gripper_1,
                'add_vacuum_gripper_2': add_vacuum_gripper_2,
                'hw_ns': hw_ns.perform(context).strip('/'),
                'limited': limited,
                'effort_control': effort_control,
                'velocity_control': velocity_control,
                'ros2_control_plugin': ros2_control_plugin,
                'add_other_geometry_1': add_other_geometry_1,
                'add_other_geometry_2': add_other_geometry_2,
                'geometry_type_1': geometry_type_1,
                'geometry_type_2': geometry_type_2,
                'geometry_mass_1': geometry_mass_1,
                'geometry_mass_2': geometry_mass_2,
                'geometry_height_1': geometry_height_1,
                'geometry_height_2': geometry_height_2,
                'geometry_radius_1': geometry_radius_1,
                'geometry_radius_2': geometry_radius_2,
                'geometry_length_1': geometry_length_1,
                'geometry_length_2': geometry_length_2,
                'geometry_width_1': geometry_width_1,
                'geometry_width_2': geometry_width_2,
                'geometry_mesh_filename_1': geometry_mesh_filename_1,
                'geometry_mesh_filename_2': geometry_mesh_filename_2,
                'geometry_mesh_origin_xyz_1': geometry_mesh_origin_xyz_1,
                'geometry_mesh_origin_xyz_2': geometry_mesh_origin_xyz_2,
                'geometry_mesh_origin_rpy_1': geometry_mesh_origin_rpy_1,
                'geometry_mesh_origin_rpy_2': geometry_mesh_origin_rpy_2,
                'geometry_mesh_tcp_xyz_1': geometry_mesh_tcp_xyz_1,
                'geometry_mesh_tcp_xyz_2': geometry_mesh_tcp_xyz_2,
                'geometry_mesh_tcp_rpy_1': geometry_mesh_tcp_rpy_1,
                'geometry_mesh_tcp_rpy_2': geometry_mesh_tcp_rpy_2,
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

   # robot moveit common launch
   # xarm_moveit_config/launch/_dual_robot_moveit_common.launch.py
    robot_moveit_common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_dual_robot_moveit_common.launch.py'])),
        launch_arguments={
            'prefix_1': prefix_1,
            'prefix_2': prefix_2,
            'dof_1': dof_1,
            'dof_2': dof_2,
            'robot_type_1': robot_type_1,
            'robot_type_2': robot_type_2,
            'add_gripper_1': add_gripper_1,
            'add_gripper_2': add_gripper_2,
            # 'add_gripper_1': add_gripper_1 if robot_type_1.perform(context) == 'xarm' else 'false',
            # 'add_gripper_2': add_gripper_2 if robot_type_2.perform(context) == 'xarm' else 'false',
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
            'moveit_controller_manager_value': moveit_controller_manager_value,
            'add_other_geometry_1': add_other_geometry_1,
            'add_other_geometry_2': add_other_geometry_2,
            'geometry_type_1': geometry_type_1,
            'geometry_type_2': geometry_type_2,
            'geometry_mass_1': geometry_mass_1,
            'geometry_mass_2': geometry_mass_2,
            'geometry_height_1': geometry_height_1,
            'geometry_height_2': geometry_height_2,
            'geometry_radius_1': geometry_radius_1,
            'geometry_radius_2': geometry_radius_2,
            'geometry_length_1': geometry_length_1,
            'geometry_length_2': geometry_length_2,
            'geometry_width_1': geometry_width_1,
            'geometry_width_2': geometry_width_2,
            'geometry_mesh_filename_1': geometry_mesh_filename_1,
            'geometry_mesh_filename_2': geometry_mesh_filename_2,
            'geometry_mesh_origin_xyz_1': geometry_mesh_origin_xyz_1,
            'geometry_mesh_origin_xyz_2': geometry_mesh_origin_xyz_2,
            'geometry_mesh_origin_rpy_1': geometry_mesh_origin_rpy_1,
            'geometry_mesh_origin_rpy_2': geometry_mesh_origin_rpy_2,
            'geometry_mesh_tcp_xyz_1': geometry_mesh_tcp_xyz_1,
            'geometry_mesh_tcp_xyz_2': geometry_mesh_tcp_xyz_2,
            'geometry_mesh_tcp_rpy_1': geometry_mesh_tcp_rpy_1,
            'geometry_mesh_tcp_rpy_2': geometry_mesh_tcp_rpy_2,
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
            ('follow_joint_trajectory', '{}{}{}_traj_controller/follow_joint_trajectory'.format(prefix_1.perform(context), robot_type_1.perform(context), dof_1.perform(context))),
            ('follow_joint_trajectory', '{}{}{}_traj_controller/follow_joint_trajectory'.format(prefix_2.perform(context), robot_type_2.perform(context), dof_2.perform(context))),
        ],
    )

    # ros2 control launch
    # xarm_controller/launch/_dual_ros2_control.launch.py
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_controller'), 'launch', '_dual_ros2_control.launch.py'])),
        launch_arguments={
            'prefix_1': prefix_1,
            'prefix_2': prefix_2,
            'dof': dof,
            'dof_1': dof_1,
            'dof_2': dof_2,
            'robot_type_1': robot_type_1,
            'robot_type_2': robot_type_2,
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
            'add_other_geometry_1': add_other_geometry_1,
            'add_other_geometry_2': add_other_geometry_2,
            'geometry_type_1': geometry_type_1,
            'geometry_type_2': geometry_type_2,
            'geometry_mass_1': geometry_mass_1,
            'geometry_mass_2': geometry_mass_2,
            'geometry_height_1': geometry_height_1,
            'geometry_height_2': geometry_height_2,
            'geometry_radius_1': geometry_radius_1,
            'geometry_radius_2': geometry_radius_2,
            'geometry_length_1': geometry_length_1,
            'geometry_length_2': geometry_length_2,
            'geometry_width_1': geometry_width_1,
            'geometry_width_2': geometry_width_2,
            'geometry_mesh_filename_1': geometry_mesh_filename_1,
            'geometry_mesh_filename_2': geometry_mesh_filename_2,
            'geometry_mesh_origin_xyz_1': geometry_mesh_origin_xyz_1,
            'geometry_mesh_origin_xyz_2': geometry_mesh_origin_xyz_2,
            'geometry_mesh_origin_rpy_1': geometry_mesh_origin_rpy_1,
            'geometry_mesh_origin_rpy_2': geometry_mesh_origin_rpy_2,
            'geometry_mesh_tcp_xyz_1': geometry_mesh_tcp_xyz_1,
            'geometry_mesh_tcp_xyz_2': geometry_mesh_tcp_xyz_2,
            'geometry_mesh_tcp_rpy_1': geometry_mesh_tcp_rpy_1,
            'geometry_mesh_tcp_rpy_2': geometry_mesh_tcp_rpy_2,
            'robot_ip_1': robot_ip_1,
            'robot_ip_2': robot_ip_2,
            'report_type_1': report_type_1,
            'report_type_2': report_type_2,
            'baud_checkset_1': baud_checkset_1,
            'baud_checkset_2': baud_checkset_2,
            'default_gripper_baud_1': default_gripper_baud_1,
            'default_gripper_baud_2': default_gripper_baud_2,
        }.items(),
    )

    # Load controllers
    load_controllers = []
    for controller in [
        '{}{}{}_traj_controller'.format(prefix_1.perform(context), robot_type_1.perform(context), dof_1.perform(context)),
        '{}{}{}_traj_controller'.format(prefix_2.perform(context), robot_type_2.perform(context), dof_2.perform(context)),
    ]:
        load_controllers.append(Node(
            package='controller_manager',
            executable='spawner.py',
            output='screen',
            arguments=[
                controller,
                '--controller-manager', '{}/controller_manager'.format(ros_namespace)
            ],
        ))

    return [
        robot_state_publisher_node,
        robot_moveit_common_launch,
        joint_state_publisher_node,
        ros2_control_launch,
        # robot_driver_launch_1,
        # robot_driver_launch_2,
    ] + load_controllers


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
