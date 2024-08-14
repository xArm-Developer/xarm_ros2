#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import yaml
import json
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from uf_ros_lib.moveit_configs_builder import DualMoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    dof = LaunchConfiguration('dof', default=7)
    dof_1 = LaunchConfiguration('dof_1', default=dof)
    dof_2 = LaunchConfiguration('dof_2', default=dof)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    robot_type_1 = LaunchConfiguration('robot_type_1', default=robot_type)
    robot_type_2 = LaunchConfiguration('robot_type_2', default=robot_type)
    prefix_1 = LaunchConfiguration('prefix_1', default='L_')
    prefix_2 = LaunchConfiguration('prefix_2', default='R_')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    model1300 = LaunchConfiguration('model1300', default=False)
    model1300_1 = LaunchConfiguration('model1300_1', default=model1300)
    model1300_2 = LaunchConfiguration('model1300_2', default=model1300)
    robot_sn = LaunchConfiguration('robot_sn', default='')
    robot_sn_1 = LaunchConfiguration('robot_sn_1', default=robot_sn)
    robot_sn_2 = LaunchConfiguration('robot_sn_2', default=robot_sn)
    mesh_suffix = LaunchConfiguration('mesh_suffix', default='stl')
    kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')
    kinematics_suffix_1 = LaunchConfiguration('kinematics_suffix_1', default=kinematics_suffix)
    kinematics_suffix_2 = LaunchConfiguration('kinematics_suffix_2', default=kinematics_suffix)

    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_gripper_1 = LaunchConfiguration('add_gripper_1', default=add_gripper)
    add_gripper_2 = LaunchConfiguration('add_gripper_2', default=add_gripper)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_vacuum_gripper_1 = LaunchConfiguration('add_vacuum_gripper_1', default=add_vacuum_gripper)
    add_vacuum_gripper_2 = LaunchConfiguration('add_vacuum_gripper_2', default=add_vacuum_gripper)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    add_bio_gripper_1 = LaunchConfiguration('add_bio_gripper_1', default=add_bio_gripper)
    add_bio_gripper_2 = LaunchConfiguration('add_bio_gripper_2', default=add_bio_gripper)
    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)
    add_realsense_d435i_1 = LaunchConfiguration('add_realsense_d435i_1', default=add_realsense_d435i)
    add_realsense_d435i_2 = LaunchConfiguration('add_realsense_d435i_2', default=add_realsense_d435i)
    add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)
    add_d435i_links_1 = LaunchConfiguration('add_d435i_links_1', default=add_d435i_links)
    add_d435i_links_2 = LaunchConfiguration('add_d435i_links_2', default=add_d435i_links)
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

    moveit_config_dump = LaunchConfiguration('moveit_config_dump', default='')
    moveit_config_dump = moveit_config_dump.perform(context)
    moveit_config_dict = yaml.load(moveit_config_dump, Loader=yaml.FullLoader) if moveit_config_dump else {}

    if not moveit_config_dict:
        moveit_config = DualMoveItConfigsBuilder(
            context=context,
            # controllers_name=controllers_name,
            dof_1=dof_1,
            dof_2=dof_2,
            robot_type_1=robot_type_1,
            robot_type_2=robot_type_2,
            prefix_1=prefix_1,
            prefix_2=prefix_2,
            hw_ns=hw_ns,
            limited=limited,
            effort_control=effort_control,
            velocity_control=velocity_control,
            model1300_1=model1300_1,
            model1300_2=model1300_2,
            robot_sn_1=robot_sn_1,
            robot_sn_2=robot_sn_2,
            mesh_suffix=mesh_suffix,
            kinematics_suffix_1=kinematics_suffix_1,
            kinematics_suffix_2=kinematics_suffix_2,
            # ros2_control_plugin=ros2_control_plugin,
            # ros2_control_params=ros2_control_params,
            add_gripper_1=add_gripper_1,
            add_gripper_2=add_gripper_2,
            add_vacuum_gripper_1=add_vacuum_gripper_1,
            add_vacuum_gripper_2=add_vacuum_gripper_2,
            add_bio_gripper_1=add_bio_gripper_1,
            add_bio_gripper_2=add_bio_gripper_2,
            add_realsense_d435i_1=add_realsense_d435i_1,
            add_realsense_d435i_2=add_realsense_d435i_2,
            add_d435i_links_1=add_d435i_links_1,
            add_d435i_links_2=add_d435i_links_2,
            add_other_geometry_1=add_other_geometry_1,
            add_other_geometry_2=add_other_geometry_2,
            geometry_type_1=geometry_type_1,
            geometry_type_2=geometry_type_2,
            geometry_mass_1=geometry_mass_1,
            geometry_mass_2=geometry_mass_2,
            geometry_height_1=geometry_height_1,
            geometry_height_2=geometry_height_2,
            geometry_radius_1=geometry_radius_1,
            geometry_radius_2=geometry_radius_2,
            geometry_length_1=geometry_length_1,
            geometry_length_2=geometry_length_2,
            geometry_width_1=geometry_width_1,
            geometry_width_2=geometry_width_2,
            geometry_mesh_filename_1=geometry_mesh_filename_1,
            geometry_mesh_filename_2=geometry_mesh_filename_2,
            geometry_mesh_origin_xyz_1=geometry_mesh_origin_xyz_1,
            geometry_mesh_origin_xyz_2=geometry_mesh_origin_xyz_2,
            geometry_mesh_origin_rpy_1=geometry_mesh_origin_rpy_1,
            geometry_mesh_origin_rpy_2=geometry_mesh_origin_rpy_2,
            geometry_mesh_tcp_xyz_1=geometry_mesh_tcp_xyz_1,
            geometry_mesh_tcp_xyz_2=geometry_mesh_tcp_xyz_2,
            geometry_mesh_tcp_rpy_1=geometry_mesh_tcp_rpy_1,
            geometry_mesh_tcp_rpy_2=geometry_mesh_tcp_rpy_2,
        ).to_moveit_configs()
        moveit_config_dict = moveit_config.to_dict()
    
    move_group_interface_params = {
        'robot_description': moveit_config_dict['robot_description'],
        'robot_description_semantic': moveit_config_dict['robot_description_semantic'],
        'robot_description_kinematics': moveit_config_dict['robot_description_kinematics'],
    }

    node_executable = LaunchConfiguration('node_executable', default='xarm_planner_node')
    node_name = LaunchConfiguration('node_name', default=node_executable)
    node_parameters = LaunchConfiguration('node_parameters', default={})
    use_gripper_node = LaunchConfiguration('use_gripper_node', default=add_gripper)

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
            move_group_interface_params,
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

    if use_gripper_node.perform(context) in ('True', 'true'):
        if robot_type_1.perform(context) != 'lite' and add_gripper_1.perform(context) in ('True', 'true'):
            nodes.append(Node(
                name='{}{}_gripper_planner_node'.format(prefix_1.perform(context), robot_type_1.perform(context)),
                package='xarm_planner',
                executable='xarm_gripper_planner_node',
                output='screen',
                parameters=[
                    move_group_interface_params,
                    {'PLANNING_GROUP': '{}{}_gripper'.format(prefix_1.perform(context), robot_type_1.perform(context))},
                ],
            ))
        if robot_type_2.perform(context) != 'lite' and add_gripper_2.perform(context) in ('True', 'true'):
            nodes.append(Node(
                name='{}{}_gripper_planner_node'.format(prefix_2.perform(context), robot_type_2.perform(context)),
                package='xarm_planner',
                executable='xarm_gripper_planner_node',
                output='screen',
                parameters=[
                    move_group_interface_params,
                    {'PLANNING_GROUP': '{}{}_gripper'.format(prefix_2.perform(context), robot_type_2.perform(context))},
                ],
            ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
