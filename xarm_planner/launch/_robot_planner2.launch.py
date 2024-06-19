#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import yaml
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
    dof = LaunchConfiguration('dof')
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    prefix = LaunchConfiguration('prefix', default='')
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    node_executable = LaunchConfiguration('node_executable', default='xarm_planner_node')
    node_name = LaunchConfiguration('node_name', default=node_executable)
    node_parameters = LaunchConfiguration('node_parameters', default={})
    use_gripper_node = LaunchConfiguration('use_gripper_node', default=add_gripper)
    moveit_config_dump = LaunchConfiguration('moveit_config_dump')

    moveit_config_dump = moveit_config_dump.perform(context)
    moveit_config_dict = yaml.load(moveit_config_dump, Loader=yaml.FullLoader)

    robot_description_parameters = {
        'robot_description': moveit_config_dict['robot_description'],
        'robot_description_semantic': moveit_config_dict['robot_description_semantic'],
        'robot_description_kinematics': moveit_config_dict['robot_description_kinematics'],
        'robot_description_planning': moveit_config_dict['robot_description_planning'],
    }

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
                'dof': dof,
                'robot_type': robot_type,
                'prefix': prefix
            },
            xarm_planner_parameters,
        ],
    )

    nodes = [
        xarm_planner_node
    ]
    if robot_type.perform(context) != 'lite' and add_gripper.perform(context) in ('True', 'true') and use_gripper_node.perform(context) in ('True', 'true'):
        planning_group = 'uf850_gripper' if robot_type.perform(context) == 'uf850' else 'xarm_gripper'
        xarm_gripper_planner_node = Node(
            name='xarm_gripper_planner_node',
            package='xarm_planner',
            executable='xarm_gripper_planner_node',
            output='screen',
            parameters=[
                robot_description_parameters,
                {'PLANNING_GROUP': planning_group},
            ],
        )
        nodes.append(xarm_gripper_planner_node)
    return nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
