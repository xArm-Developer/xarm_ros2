#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import json
from ament_index_python import get_package_share_directory
from launch.frontend import expose
from launch.launch_description_sources import load_python_launch_file_as_module
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
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
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    add_bio_gripper_1 = LaunchConfiguration('add_bio_gripper_1', default=add_bio_gripper)
    add_bio_gripper_2 = LaunchConfiguration('add_bio_gripper_2', default=add_bio_gripper)
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='uf_robot_hardware/UFRobotFakeSystemHardware')
    node_executable = LaunchConfiguration('node_executable', default='xarm_planner_node')
    node_name = LaunchConfiguration('node_name', default=node_executable)
    node_parameters = LaunchConfiguration('node_parameters', default={})
    use_gripper_node = LaunchConfiguration('use_gripper_node', default=add_gripper)

    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)
    add_realsense_d435i_1 = LaunchConfiguration('add_realsense_d435i_1', default=add_realsense_d435i)
    add_realsense_d435i_2 = LaunchConfiguration('add_realsense_d435i_2', default=add_realsense_d435i)

    add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)
    add_d435i_links_1 = LaunchConfiguration('add_d435i_links_1', default=add_d435i_links)
    add_d435i_links_2 = LaunchConfiguration('add_d435i_links_2', default=add_d435i_links)
    
    model1300 = LaunchConfiguration('model1300', default=False)
    model1300_1 = LaunchConfiguration('model1300_1', default=model1300)
    model1300_2 = LaunchConfiguration('model1300_2', default=model1300)

    robot_sn = LaunchConfiguration('robot_sn', default='')
    robot_sn_1 = LaunchConfiguration('robot_sn_1', default=robot_sn)
    robot_sn_2 = LaunchConfiguration('robot_sn_2', default=robot_sn)

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

    kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')
    kinematics_suffix_1 = LaunchConfiguration('kinematics_suffix_1', default=kinematics_suffix)
    kinematics_suffix_2 = LaunchConfiguration('kinematics_suffix_2', default=kinematics_suffix)

    moveit_config_package_name = 'xarm_moveit_config'
    xarm_type = '{}{}'.format(robot_type_1.perform(context), dof_1.perform(context) if robot_type_1.perform(context) in ('xarm', 'lite') else '')

    # robot_description_parameters
    # xarm_moveit_config/launch/lib/robot_moveit_config_lib.py
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory(moveit_config_package_name), 'launch', 'lib', 'robot_moveit_config_lib.py'))
    get_xarm_robot_description_parameters = getattr(mod, 'get_xarm_robot_description_parameters')
    robot_description_parameters = get_xarm_robot_description_parameters(
        xacro_urdf_file=PathJoinSubstitution([FindPackageShare('xarm_description'), 'urdf', 'dual_xarm_device.urdf.xacro']),
        xacro_srdf_file=PathJoinSubstitution([FindPackageShare(moveit_config_package_name), 'srdf', 'dual_xarm.srdf.xacro']),
        urdf_arguments={
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
            'add_bio_gripper_1': add_bio_gripper_1,
            'add_bio_gripper_2': add_bio_gripper_2,
            'hw_ns': hw_ns.perform(context).strip('/'),
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'ros2_control_plugin': ros2_control_plugin,
            'add_realsense_d435i_1': add_realsense_d435i_1,
            'add_realsense_d435i_2': add_realsense_d435i_2,
            'add_d435i_links_1': add_d435i_links_1,
            'add_d435i_links_2': add_d435i_links_2,
            'model1300_1': model1300_1,
            'model1300_2': model1300_2,
            'robot_sn_1': robot_sn_1,
            'robot_sn_2': robot_sn_2,
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
            'kinematics_suffix_1': kinematics_suffix_1,
            'kinematics_suffix_2': kinematics_suffix_2,
        },
        srdf_arguments={
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
            'add_bio_gripper_1': add_bio_gripper_1,
            'add_bio_gripper_2': add_bio_gripper_2,
            'add_other_geometry_1': add_other_geometry_1,
            'add_other_geometry_2': add_other_geometry_2,
        },
        arguments={
            'context': context,
            'xarm_type': xarm_type
        }
    )

    load_yaml = getattr(mod, 'load_yaml')

    kinematics_yaml_1 = load_yaml(moveit_config_package_name, 'config', xarm_type, 'kinematics.yaml')
    joint_limits_yaml_1 = load_yaml(moveit_config_package_name, 'config', xarm_type, 'joint_limits.yaml')
    
    xarm_type = '{}{}'.format(robot_type_2.perform(context), dof_2.perform(context) if robot_type_2.perform(context) in ('xarm', 'lite') else '')
    kinematics_yaml_2 = load_yaml(moveit_config_package_name, 'config', xarm_type, 'kinematics.yaml')
    joint_limits_yaml_2 = load_yaml(moveit_config_package_name, 'config', xarm_type, 'joint_limits.yaml')
        
    if add_gripper_1.perform(context) in ('True', 'true'):
        gripper_joint_limits_yaml_1 = load_yaml(moveit_config_package_name, 'config', '{}_gripper'.format(robot_type_1.perform(context)), 'joint_limits.yaml')
        if joint_limits_yaml_1 and gripper_joint_limits_yaml_1:
            joint_limits_yaml_1['joint_limits'].update(gripper_joint_limits_yaml_1['joint_limits'])
    elif add_bio_gripper_1.perform(context) in ('True', 'true'):
        gripper_joint_limits_yaml_1 = load_yaml(moveit_config_package_name, 'config', 'bio_gripper', 'joint_limits.yaml')
        if joint_limits_yaml_1 and gripper_joint_limits_yaml_1:
            joint_limits_yaml_1['joint_limits'].update(gripper_joint_limits_yaml_1['joint_limits'])
    if add_gripper_2.perform(context) in ('True', 'true'):
        gripper_joint_limits_yaml_2 = load_yaml(moveit_config_package_name, 'config', '{}_gripper'.format(robot_type_2.perform(context)), 'joint_limits.yaml')
        if joint_limits_yaml_2 and gripper_joint_limits_yaml_2:
            joint_limits_yaml_2['joint_limits'].update(gripper_joint_limits_yaml_2['joint_limits'])
    elif add_bio_gripper_2.perform(context) in ('True', 'true'):
        gripper_joint_limits_yaml_2 = load_yaml(moveit_config_package_name, 'config', 'bio_gripper', 'joint_limits.yaml')
        if joint_limits_yaml_2 and gripper_joint_limits_yaml_2:
            joint_limits_yaml_2['joint_limits'].update(gripper_joint_limits_yaml_2['joint_limits'])
        

    add_prefix_to_moveit_params = getattr(mod, 'add_prefix_to_moveit_params')
    add_prefix_to_moveit_params(kinematics_yaml=kinematics_yaml_1, joint_limits_yaml=joint_limits_yaml_1, prefix=prefix_1.perform(context))
    add_prefix_to_moveit_params(kinematics_yaml=kinematics_yaml_2, joint_limits_yaml=joint_limits_yaml_2, prefix=prefix_2.perform(context))
    kinematics_yaml = {}
    kinematics_yaml.update(kinematics_yaml_1)
    kinematics_yaml.update(kinematics_yaml_2)
    joint_limits_yaml = {'joint_limits': {}}
    joint_limits_yaml.update(joint_limits_yaml_1['joint_limits'])
    joint_limits_yaml.update(joint_limits_yaml_2['joint_limits'])
    robot_description_parameters['robot_description_kinematics'] = kinematics_yaml
    robot_description_parameters['robot_description_planning'] = joint_limits_yaml

    try:
        xarm_planner_parameters = json.loads(node_parameters.perform(context))
    except:
        xarm_planner_parameters = {}

    xarm_planner_node = Node(
        name=node_name,
        package='xarm_planner',
        executable=node_executable,
        output='screen',
        parameters=[
            robot_description_parameters,
            {
                'robot_type_1': robot_type_1,
                'robot_type_2': robot_type_2,
                'dof_1': dof_1,
                'dof_2': dof_2,
                'prefix_1': prefix_1,
                'prefix_2': prefix_2,
            },
            xarm_planner_parameters,
        ],
    )

    nodes = [
        xarm_planner_node
    ]
    if add_gripper.perform(context) in ('True', 'true') and use_gripper_node.perform(context) in ('True', 'true'):
        xarm_gripper_planner_node = Node(
            name=node_name,
            package='xarm_planner',
            executable='xarm_gripper_planner_node',
            output='screen',
            parameters=[
                robot_description_parameters,
                {'PLANNING_GROUP': 'xarm_gripper'},
            ],
        )
        nodes.append(xarm_gripper_planner_node)
    return nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
