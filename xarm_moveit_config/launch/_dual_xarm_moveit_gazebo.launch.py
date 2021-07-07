#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


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

    ros2_control_plugin = 'gazebo_ros2_control/GazeboSystem'
    controllers_name = 'fake_controllers'
    moveit_controller_manager_key = 'moveit_simple_controller_manager'
    moveit_controller_manager_value = 'moveit_simple_controller_manager/MoveItSimpleControllerManager'

    # xarm moveit common launch
    # xarm_moveit_config/launch/_dual_xarm_moveit_common.launch.py
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

    # xarm gazebo launch
    # xarm_gazebo/launch/_dual_xarm_beside_table_gazebo.launch.py
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_gazebo'), 'launch', '_dual_xarm_beside_table_gazebo.launch.py'])),
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


    return [
        gazebo_launch,
        xarm_moveit_common_launch
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
