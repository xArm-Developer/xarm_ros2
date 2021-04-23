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
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


package_path = get_package_share_directory('xarm_moveit_config')
sys.path.append(os.path.join(package_path, 'launch', 'lib'))
from xarm_moveit_config_lib import get_xarm_moveit_realmove_launch_description


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    report_type = LaunchConfiguration('report_type', default='normal')

    prefix = LaunchConfiguration('prefix', default='')
    ns = LaunchConfiguration('ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)

    dof = 5

    return get_xarm_moveit_realmove_launch_description(
        robot_ip, report_type,
        prefix, ns, limited, effort_control, velocity_control, add_gripper, add_vacuum_gripper,
        dof=str(dof), xarm_type='xarm{}'.format(dof)
    )
