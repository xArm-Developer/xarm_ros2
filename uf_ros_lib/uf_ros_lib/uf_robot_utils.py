#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2024, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import yaml
from pathlib import Path
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from tempfile import NamedTemporaryFile
from ament_index_python import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_param_builder import load_xacro


def get_xacro_command(
    xacro_file=PathJoinSubstitution([FindPackageShare('xarm_description'), 'urdf', 'xarm_device.urdf.xacro']), 
    mappings={}):
    command = [
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        str(xacro_file) if isinstance(xacro_file, Path) else xacro_file,
        ' '
    ]
    if mappings and isinstance(mappings, dict):
        for key, val in mappings.items():
            command.extend([
                '{}:='.format(key),
                val,
                ' '
            ])
    return Command(command)


def get_xacro_content(context,
    xacro_file=Path(get_package_share_directory('xarm_description')) / 'urdf' / 'xarm_device.urdf.xacro', 
    **kwargs):
    xacro_file = Path(xacro_file.perform(context)) if isinstance(xacro_file, LaunchConfiguration) else Path(xacro_file) if isinstance(xacro_file, str) else xacro_file
    
    def get_param_str(param):
        val = param if isinstance(param, str) else 'false' if param == False else 'true' if param == True else (param.perform(context) if context is not None else param) if isinstance(param, LaunchConfiguration) else str(param)
        return val if not val else val[1:-1] if isinstance(val, str) and val[0] in ['"', '\''] and val[-1] in ['"', '\''] else val

    mappings = {}
    for k, v in kwargs.items():
        mappings[k] = get_param_str(v)
    return load_xacro(xacro_file, mappings=mappings)


def merge_dict(dict1, dict2):
    for k, v in dict1.items():
        try:
            if k not in dict2:
                continue
            if isinstance(v, dict):
                merge_dict(v, dict2[k])
            else:
                dict1[k] = dict2[k]
        except Exception as e:
            pass


def load_abspath_yaml(path):
    if path and os.path.exists(path):
        try:
            with open(path, 'r') as file:
                return yaml.safe_load(file)
        except Exception as e:
            print('load {} error, {}'.format(path, e))
    return {}


def generate_robot_api_params(default_robot_api_params_path, user_robot_api_params_path=None, ros_namespace='', node_name='ufactory_driver', extra_robot_api_params_path=None):
    if not user_robot_api_params_path or not os.path.exists(user_robot_api_params_path):
        user_robot_api_params_path = None
    if not extra_robot_api_params_path or not os.path.exists(extra_robot_api_params_path):
        extra_robot_api_params_path = None
    if ros_namespace or (user_robot_api_params_path is not None and user_robot_api_params_path != default_robot_api_params_path) or (extra_robot_api_params_path is not None and extra_robot_api_params_path != default_robot_api_params_path):
        ronbot_api_params_yaml = load_abspath_yaml(default_robot_api_params_path)
        user_params_yaml = load_abspath_yaml(user_robot_api_params_path)
        extra_params_yaml = load_abspath_yaml(extra_robot_api_params_path)
        # change xarm_driver to ufactory_driver
        if 'xarm_driver' in ronbot_api_params_yaml and node_name not in ronbot_api_params_yaml:
            ronbot_api_params_yaml[node_name] = ronbot_api_params_yaml.pop('xarm_driver')
        if 'xarm_driver' in user_params_yaml and node_name not in user_params_yaml:
            user_params_yaml[node_name] = user_params_yaml.pop('xarm_driver')
        if 'xarm_driver' in extra_params_yaml and node_name not in extra_params_yaml:
            extra_params_yaml[node_name] = extra_params_yaml.pop('xarm_driver')
        if user_params_yaml:
            merge_dict(ronbot_api_params_yaml, user_params_yaml)
        if extra_params_yaml:
            merge_dict(ronbot_api_params_yaml, extra_params_yaml)
        if ros_namespace:
            robot_params_yaml = {
                ros_namespace: ronbot_api_params_yaml
            }
        else:
            robot_params_yaml = ronbot_api_params_yaml
        with NamedTemporaryFile(mode='w', prefix='launch_params_', delete=False) as h:
            yaml.dump(robot_params_yaml, h, default_flow_style=False)
            return h.name
    return default_robot_api_params_path


def load_yaml(package_name, *file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, *file_path)
    if not os.path.exists(absolute_file_path):
        return {}
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return {}
    except:
        return {}


def add_prefix_to_ros2_control_params(prefix, ros2_control_params):
    if not prefix:
        return
    for name in list(ros2_control_params.keys()):
        if name == 'controller_manager':
            continue
        ros__parameters = ros2_control_params[name].get('ros__parameters', {})
        joints = ros__parameters.get('joints', [])
        constraints = ros__parameters.get('constraints', {})
        for i, joint in enumerate(joints):
            for j, key in enumerate(constraints.keys()):
                if key == joint:
                    constraints['{}{}'.format(prefix, key)] = constraints.pop(key)
                    break
            joints[i] = '{}{}'.format(prefix, joint)
        new_name = '{}{}'.format(prefix, name)
        ros2_control_params[new_name] = ros2_control_params.pop(name)
        controller_manager_ros__parameters = ros2_control_params.get('controller_manager', {}).get('ros__parameters', {})
        if name in controller_manager_ros__parameters:
            controller_manager_ros__parameters[new_name] = controller_manager_ros__parameters.pop(name)


def generate_ros2_control_params_temp_file(ros2_control_params_path, prefix='', add_gripper=False, add_bio_gripper=False, ros_namespace='', update_rate=None, robot_type='xarm', use_sim_time=False):
    if ros_namespace or prefix or add_gripper or add_bio_gripper or update_rate:
        with open(ros2_control_params_path, 'r') as f:
            ros2_control_params_yaml = yaml.safe_load(f)
        if update_rate is not None:
            ros2_control_params_yaml['controller_manager']['ros__parameters']['update_rate'] = update_rate
        if use_sim_time:
            ros2_control_params_yaml['controller_manager']['ros__parameters']['use_sim_time'] = True
        if add_gripper:
            gripper_control_params_path = os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}_gripper_controllers.yaml'.format(robot_type))
            # check file is exists or not
            if os.path.exists(gripper_control_params_path):
                with open(gripper_control_params_path, 'r') as f:
                    gripper_control_params_yaml = yaml.safe_load(f)
                for name, value in gripper_control_params_yaml['controller_manager']['ros__parameters'].items():
                    ros2_control_params_yaml['controller_manager']['ros__parameters'][name] = value
                    if name in gripper_control_params_yaml:
                        ros2_control_params_yaml[name] = gripper_control_params_yaml[name]
        elif add_bio_gripper:
            gripper_control_params_path = os.path.join(get_package_share_directory('xarm_controller'), 'config', 'bio_gripper_controllers.yaml')
            # check file is exists or not
            if os.path.exists(gripper_control_params_path):
                with open(gripper_control_params_path, 'r') as f:
                    gripper_control_params_yaml = yaml.safe_load(f)
                for name, value in gripper_control_params_yaml['controller_manager']['ros__parameters'].items():
                    ros2_control_params_yaml['controller_manager']['ros__parameters'][name] = value
                    if name in gripper_control_params_yaml:
                        ros2_control_params_yaml[name] = gripper_control_params_yaml[name]
                
        add_prefix_to_ros2_control_params(prefix, ros2_control_params_yaml)
        if ros_namespace:
            ros2_control_params_yaml = {
                ros_namespace: ros2_control_params_yaml
            }
        with NamedTemporaryFile(mode='w', prefix='launch_params_', delete=False) as h:
            yaml.dump(ros2_control_params_yaml, h, default_flow_style=False)
            return h.name
    return ros2_control_params_path


def generate_dual_ros2_control_params_temp_file(
    ros2_control_params_path_1, ros2_control_params_path_2, 
    prefix_1='L_', prefix_2='R_', 
    add_gripper_1=False, add_gripper_2=False, 
    add_bio_gripper_1=False, add_bio_gripper_2=False, 
    ros_namespace='', update_rate=None, use_sim_time=False,
    robot_type_1='xarm', robot_type_2='xarm'):
    with open(ros2_control_params_path_1, 'r') as f:
        ros2_control_params_yaml_1 = yaml.safe_load(f)
    with open(ros2_control_params_path_2, 'r') as f:
        ros2_control_params_yaml_2 = yaml.safe_load(f)
    if update_rate is not None:
        ros2_control_params_yaml_1['controller_manager']['ros__parameters']['update_rate'] = update_rate
        ros2_control_params_yaml_2['controller_manager']['ros__parameters']['update_rate'] = update_rate
    if use_sim_time:
        ros2_control_params_yaml_1['controller_manager']['ros__parameters']['use_sim_time'] = True
        ros2_control_params_yaml_2['controller_manager']['ros__parameters']['use_sim_time'] = True

    if add_gripper_1:
        gripper_control_params_path = os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}_gripper_controllers.yaml'.format(robot_type_1))
        # check file is exists or not
        if os.path.exists(gripper_control_params_path):
            with open(gripper_control_params_path, 'r') as f:
                gripper_control_params_yaml = yaml.safe_load(f)
            for name, value in gripper_control_params_yaml['controller_manager']['ros__parameters'].items():
                ros2_control_params_yaml_1['controller_manager']['ros__parameters'][name] = value
                if name in gripper_control_params_yaml:
                    ros2_control_params_yaml_1[name] = gripper_control_params_yaml[name]
    elif add_bio_gripper_1:
        gripper_control_params_path = os.path.join(get_package_share_directory('xarm_controller'), 'config', 'bio_gripper_controllers.yaml')
        # check file is exists or not
        if os.path.exists(gripper_control_params_path):
            with open(gripper_control_params_path, 'r') as f:
                gripper_control_params_yaml = yaml.safe_load(f)
            for name, value in gripper_control_params_yaml['controller_manager']['ros__parameters'].items():
                ros2_control_params_yaml_1['controller_manager']['ros__parameters'][name] = value
                if name in gripper_control_params_yaml:
                    ros2_control_params_yaml_1[name] = gripper_control_params_yaml[name]
        
    if add_gripper_2:
        gripper_control_params_path = os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}_gripper_controllers.yaml'.format(robot_type_2))
        # check file is exists or not
        if os.path.exists(gripper_control_params_path):
            with open(gripper_control_params_path, 'r') as f:
                gripper_control_params_yaml = yaml.safe_load(f)
            for name, value in gripper_control_params_yaml['controller_manager']['ros__parameters'].items():
                ros2_control_params_yaml_2['controller_manager']['ros__parameters'][name] = value
                if name in gripper_control_params_yaml:
                    ros2_control_params_yaml_2[name] = gripper_control_params_yaml[name]
    elif add_bio_gripper_2:
        gripper_control_params_path = os.path.join(get_package_share_directory('xarm_controller'), 'config', 'bio_gripper_controllers.yaml')
        # check file is exists or not
        if os.path.exists(gripper_control_params_path):
            with open(gripper_control_params_path, 'r') as f:
                gripper_control_params_yaml = yaml.safe_load(f)
            for name, value in gripper_control_params_yaml['controller_manager']['ros__parameters'].items():
                ros2_control_params_yaml_2['controller_manager']['ros__parameters'][name] = value
                if name in gripper_control_params_yaml:
                    ros2_control_params_yaml_2[name] = gripper_control_params_yaml[name]
        
    add_prefix_to_ros2_control_params(prefix_1, ros2_control_params_yaml_1)
    add_prefix_to_ros2_control_params(prefix_2, ros2_control_params_yaml_2)
    ros2_control_params_yaml = {}
    ros2_control_params_yaml.update(ros2_control_params_yaml_1)
    ros2_control_params_yaml.update(ros2_control_params_yaml_2)
    ros2_control_params_yaml['controller_manager']['ros__parameters'].update(ros2_control_params_yaml_1['controller_manager']['ros__parameters'])
    ros2_control_params_yaml['controller_manager']['ros__parameters'].update(ros2_control_params_yaml_2['controller_manager']['ros__parameters'])
    if ros_namespace:
        ros2_control_params_yaml = {
            ros_namespace: ros2_control_params_yaml
        }
    with NamedTemporaryFile(mode='w', prefix='launch_params_', delete=False) as h:
        yaml.dump(ros2_control_params_yaml, h, default_flow_style=False)
        return h.name