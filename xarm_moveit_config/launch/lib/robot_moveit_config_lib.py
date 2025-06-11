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
from uf_ros_lib.uf_robot_utils import get_xacro_command, load_yaml


def load_file(package_name, *file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, *file_path)
    if not os.path.exists(absolute_file_path):
        return {}
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return {}
    except:
        return {}


def add_prefix_to_moveit_params(controllers_yaml=None, ompl_planning_yaml=None, kinematics_yaml=None, joint_limits_yaml=None, prefix=''):
    if not prefix:
        return
    if controllers_yaml:
        for i, name in enumerate(controllers_yaml['controller_names']):
            joints = controllers_yaml.get(name, {}).get('joints', [])
            for j, joint in enumerate(joints):
                joints[j] = '{}{}'.format(prefix, joint)
            controllers_yaml['controller_names'][i] = '{}{}'.format(prefix, name)
            if name in controllers_yaml:
                controllers_yaml['{}{}'.format(prefix, name)] = controllers_yaml.pop(name)
    if ompl_planning_yaml:
        for name in list(ompl_planning_yaml.keys()):
            if name != 'planner_configs':
                ompl_planning_yaml['{}{}'.format(prefix, name)] = ompl_planning_yaml.pop(name)
    if kinematics_yaml:
        for name in list(kinematics_yaml.keys()):
            kinematics_yaml['{}{}'.format(prefix, name)] = kinematics_yaml.pop(name)
    if joint_limits_yaml:
        for name in list(joint_limits_yaml['joint_limits']):
            joint_limits_yaml['joint_limits']['{}{}'.format(prefix, name)] = joint_limits_yaml['joint_limits'].pop(name)


def get_xarm_robot_description_parameters(
    xacro_urdf_file=PathJoinSubstitution([FindPackageShare('xarm_description'), 'urdf', 'xarm_device.urdf.xacro']),
    xacro_srdf_file=PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'srdf', 'xarm.srdf.xacro']),
    urdf_arguments={},
    srdf_arguments={},
    arguments={}):
    urdf_arguments['ros2_control_plugin'] = urdf_arguments.get('ros2_control_plugin', 'uf_robot_hardware/UFRobotSystemHardware')
    moveit_config_package_name = 'xarm_moveit_config'
    xarm_type = arguments.get('xarm_type', None)
    
    return {
        'robot_description': get_xacro_command(
            xacro_file=xacro_urdf_file, 
            mappings=urdf_arguments
        ),
        'robot_description_semantic': get_xacro_command(
            xacro_file=xacro_srdf_file,
            mappings=srdf_arguments
        ),
        'robot_description_planning': load_yaml(moveit_config_package_name, 'config', xarm_type, 'joint_limits.yaml'),
        'robot_description_kinematics': load_yaml(moveit_config_package_name, 'config', xarm_type, 'kinematics.yaml')
    }
