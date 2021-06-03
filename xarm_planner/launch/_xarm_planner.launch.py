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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    dof = LaunchConfiguration('dof')
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='fake_components/GenericSystem')
    node_executable = LaunchConfiguration('node_executable', default='xarm_planner_node')
    node_name = LaunchConfiguration('node_name', default=node_executable)
    node_parameters = LaunchConfiguration('node_parameters', default={})
    use_gripper_node = LaunchConfiguration('use_gripper_node', default=add_gripper)

    moveit_config_package_name = 'xarm_moveit_config'
    xarm_type = 'xarm{}'.format(dof.perform(context))

    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory(moveit_config_package_name), 'launch', 'lib', 'xarm_moveit_config_lib.py'))
    get_xarm_robot_description_parameters = getattr(mod, 'get_xarm_robot_description_parameters')
    robot_description_parameters = get_xarm_robot_description_parameters(
        prefix, hw_ns.perform(context).strip('/'), limited, 
        effort_control, velocity_control, 
        add_gripper, add_vacuum_gripper, 
        dof, xarm_type, ros2_control_plugin,
        context=context,
    )

    try:
        xarm_planner_parameters = json.loads(node_parameters.perform(context))
    except:
        xarm_planner_parameters = {}

    xarm_planner_node = Node(
        name=node_name,
        package="xarm_planner",
        executable=node_executable,
        output="screen",
        parameters=[
            robot_description_parameters,
            {'DOF': dof},
            xarm_planner_parameters,
        ],
    )

    nodes = [
        xarm_planner_node
    ]
    if add_gripper.perform(context) == 'true' and use_gripper_node.perform(context) == 'true':
        xarm_gripper_planner_node = Node(
            name=node_name,
            package="xarm_planner",
            executable='xarm_gripper_planner_node',
            output="screen",
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
