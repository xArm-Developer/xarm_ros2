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
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_ip_1 = LaunchConfiguration('robot_ip_1', default='')
    robot_ip_2 = LaunchConfiguration('robot_ip_2', default='')
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
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='uf_robot_hardware/UFRobotSystemHardware')

    baud_checkset = LaunchConfiguration('baud_checkset', default=True)
    baud_checkset_1 = LaunchConfiguration('baud_checkset', default=baud_checkset)
    baud_checkset_2 = LaunchConfiguration('baud_checkset', default=baud_checkset)
    default_gripper_baud = LaunchConfiguration('default_gripper_baud', default=2000000)
    default_gripper_baud_1 = LaunchConfiguration('default_gripper_baud', default=default_gripper_baud)
    default_gripper_baud_2 = LaunchConfiguration('default_gripper_baud', default=default_gripper_baud)

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

    # ros2 control params
    # xarm_controller/launch/lib/robot_controller_lib.py
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm_controller'), 'launch', 'lib', 'robot_controller_lib.py'))
    generate_dual_ros2_control_params_temp_file = getattr(mod, 'generate_dual_ros2_control_params_temp_file')
    ros2_control_params = generate_dual_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}{}_controllers.yaml'.format(robot_type_1.perform(context), dof_1.perform(context))),
        os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}{}_controllers.yaml'.format(robot_type_2.perform(context), dof_2.perform(context))),
        prefix_1=prefix_1.perform(context), 
        prefix_2=prefix_2.perform(context), 
        add_gripper_1=add_gripper_1.perform(context) in ('True', 'true'),
        add_gripper_2=add_gripper_2.perform(context) in ('True', 'true'),
        ros_namespace=LaunchConfiguration('ros_namespace', default='').perform(context),
        robot_type_1=robot_type_1.perform(context), 
        robot_type_2=robot_type_2.perform(context), 
    )

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
                'ros2_control_params': ros2_control_params,
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
            }
        )
    }

    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm_api'), 'launch', 'lib', 'robot_api_lib.py'))
    generate_robot_api_params = getattr(mod, 'generate_robot_api_params')
    robot_params = generate_robot_api_params(
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_params.yaml'),
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_user_params.yaml'),
        LaunchConfiguration('ros_namespace', default='').perform(context), node_name='ufactory_driver'
    )

    # ros2 control node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            ros2_control_params,
            robot_params,
        ],
        output='screen',
    )

    return [
        ros2_control_node
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
