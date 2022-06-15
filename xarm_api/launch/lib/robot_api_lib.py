#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2022, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 
import os
import yaml
from tempfile import NamedTemporaryFile


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


def load_yaml(path):
    if os.path.exists(path):
        try:
            with open(path, 'r') as file:
                return yaml.safe_load(file)
        except Exception as e:
            print('load {} error, {}'.format(path, e))
    return {}


def generate_robot_api_params(robot_default_params_path, robot_user_params_path=None, ros_namespace='', node_name='ufactory_driver'):
    if not os.path.exists(robot_user_params_path):
        robot_user_params_path = None
    if ros_namespace or (robot_user_params_path is not None and robot_default_params_path != robot_user_params_path):
        ros2_control_params_yaml = load_yaml(robot_default_params_path)
        ros2_control_user_params_yaml = load_yaml(robot_user_params_path)
        # change xarm_driver to ufactory_driver
        if 'xarm_driver' in ros2_control_params_yaml and node_name not in ros2_control_params_yaml:
            ros2_control_params_yaml[node_name] = ros2_control_params_yaml.pop('xarm_driver')
        if 'xarm_driver' in ros2_control_user_params_yaml and node_name not in ros2_control_user_params_yaml:
            ros2_control_user_params_yaml[node_name] = ros2_control_user_params_yaml.pop('xarm_driver')
        merge_dict(ros2_control_params_yaml, ros2_control_user_params_yaml)
        if ros_namespace:
            xarm_params_yaml = {
                ros_namespace: ros2_control_params_yaml
            }
        else:
            xarm_params_yaml = ros2_control_params_yaml
        with NamedTemporaryFile(mode='w', prefix='launch_params_', delete=False) as h:
            yaml.dump(xarm_params_yaml, h, default_flow_style=False)
            return h.name
    return robot_default_params_path

