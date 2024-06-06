#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import yaml
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import load_python_launch_file_as_module
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_ip = LaunchConfiguration('robot_ip', default='')
    report_type = LaunchConfiguration('report_type', default='normal')
    baud_checkset = LaunchConfiguration('baud_checkset', default=True)
    default_gripper_baud = LaunchConfiguration('default_gripper_baud', default=2000000)
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    dof = LaunchConfiguration('dof', default=7)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='uf_robot_hardware/UFRobotSystemHardware')
    xacro_file = LaunchConfiguration('xacro_file', default=PathJoinSubstitution([FindPackageShare('xarm_description'), 'urdf', 'xarm_device.urdf.xacro']))

    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)
    add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)
    model1300 = LaunchConfiguration('model1300', default=False)
    robot_sn = LaunchConfiguration('robot_sn', default='')
    attach_to = LaunchConfiguration('attach_to', default='world')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')

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

    kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')
    
    robot_description = LaunchConfiguration('robot_description', default='')
    ros2_control_params = LaunchConfiguration('ros2_control_params', default='')

    if not ros2_control_params.perform(context):
        # ros2 control params
        # xarm_controller/launch/lib/robot_controller_lib.py
        mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm_controller'), 'launch', 'lib', 'robot_controller_lib.py'))
        generate_ros2_control_params_temp_file = getattr(mod, 'generate_ros2_control_params_temp_file')
        ros2_control_params = generate_ros2_control_params_temp_file(
            os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}{}_controllers.yaml'.format(robot_type.perform(context), dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else '')),
            prefix=prefix.perform(context), 
            add_gripper=add_gripper.perform(context) in ('True', 'true'),
            add_bio_gripper=add_bio_gripper.perform(context) in ('True', 'true'),
            ros_namespace=LaunchConfiguration('ros_namespace', default='').perform(context),
            robot_type=robot_type.perform(context)
        )

    if not robot_description.perform(context):
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
                    'add_bio_gripper': add_bio_gripper,
                    'dof': dof,
                    'robot_type': robot_type,
                    'ros2_control_plugin': ros2_control_plugin,
                    'ros2_control_params': ros2_control_params,
                    'add_realsense_d435i': add_realsense_d435i,
                    'add_d435i_links': add_d435i_links,
                    'model1300': model1300,
                    'robot_sn': robot_sn,
                    'attach_to': attach_to,
                    'attach_xyz': attach_xyz,
                    'attach_rpy': attach_rpy,
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
                    'kinematics_suffix': kinematics_suffix,
                    'robot_ip': robot_ip,
                    'report_type': report_type,
                    'baud_checkset': baud_checkset,
                    'default_gripper_baud': default_gripper_baud,
                }
            )
        }
    else:
        robot_description = yaml.load(robot_description.perform(context), Loader=yaml.FullLoader)

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
