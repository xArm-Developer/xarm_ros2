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
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def load_file(package_name, *file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, *file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, *file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, *file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def get_xarm_robot_description_semantic(srdf_path, add_gripper):
    # robot_description_semantic
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            srdf_path,
            " ",
            "add_gripper:=",
            add_gripper,
            " ",
        ]
    )
    return {"robot_description_semantic": robot_description_semantic_content}


def get_xarm_robot_description_parameters(
    prefix, hw_ns, limited, 
    effort_control, velocity_control, 
    add_gripper, add_vacuum_gripper, 
    dof, xarm_type, ros2_control_plugin='xarm_control/XArmHW',
    context=None):
    # robot_description
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm_description'), 'launch', 'lib', 'xarm_description_lib.py'))
    get_xarm_robot_description = getattr(mod, 'get_xarm_robot_description')
    robot_description = get_xarm_robot_description(
        prefix, hw_ns, limited, 
        effort_control, velocity_control, 
        add_gripper, add_vacuum_gripper, 
        dof, ros2_control_plugin
    )

    srdf_name = xarm_type
    # if context is not None and isinstance(add_gripper, LaunchConfiguration)  and add_gripper.perform(context) == 'true':
    #     srdf_name = '{}_with_gripper'.format(xarm_type)

    moveit_config_package_name = 'xarm_moveit_config'
    srdf_path = PathJoinSubstitution([FindPackageShare(moveit_config_package_name), 'srdf', '{}.srdf'.format(srdf_name)])
    robot_description_semantic = get_xarm_robot_description_semantic(srdf_path, add_gripper)
    # robot_description_semantic = {'robot_description_semantic': load_file(moveit_config_package_name, 'srdf', '{}.srdf'.format(srdf_name))}    
    robot_description_kinematics = {'robot_description_kinematics': load_yaml(moveit_config_package_name, 'config', xarm_type, 'kinematics.yaml')}
    robot_description_planning = {'robot_description_planning': load_yaml(moveit_config_package_name, 'config', xarm_type, 'joint_limits.yaml')}
    ret = {}
    ret.update(robot_description)
    ret.update(robot_description_semantic)
    ret.update(robot_description_kinematics)
    # ret.update(robot_description_planning)
    return ret
