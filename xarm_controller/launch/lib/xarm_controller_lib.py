#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import sys
import yaml
from tempfile import NamedTemporaryFile
from ament_index_python import get_package_share_directory
from ros2launch.api.api import parse_launch_arguments


def get_sys_param(name, default=None):
    launch_arguments_dict = dict(parse_launch_arguments(sys.argv[4:]))
    return launch_arguments_dict.get(name, default)


def get_controller_params(dof, name='ros_namespace', default=None):
    package_path = get_package_share_directory('xarm_controller')
    controller_params = os.path.join(package_path, 'config', 'xarm{}_controllers.yaml'.format(dof))
    ros_namespace = get_sys_param(name, default=default)
    if ros_namespace:
        with open(controller_params, 'r') as f:
            controller_params_yaml = yaml.safe_load(f)
        controller_params_yaml = {
            ros_namespace: controller_params_yaml
        }
        with NamedTemporaryFile(mode='w', prefix='launch_params_', delete=False) as h:
            param_file_path = h.name
            yaml.dump(controller_params_yaml, h, default_flow_style=False)
            return param_file_path
    return controller_params
