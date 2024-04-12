#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2024, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from pathlib import Path
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


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
