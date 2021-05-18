#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import sys
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

package_path = get_package_share_directory('xarm_moveit_config')
sys.path.append(os.path.join(package_path, 'launch', 'lib'))
from xarm_moveit_config_lib import get_xarm_moveit_realmove_launch_description


def generate_launch_description():
    robot1_ip = LaunchConfiguration('robot1_ip')
    robot2_ip = LaunchConfiguration('robot2_ip')
    report_type = LaunchConfiguration('report_type', default='normal')
    ns1_default_value = 'left'
    ns2_default_value = 'right'
    ns1 = LaunchConfiguration('ns1', default=ns1_default_value)
    ns2 = LaunchConfiguration('ns2', default=ns2_default_value)
    prefix = LaunchConfiguration('prefix', default='')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    
    hw_ns = 'xarm'
    dof = 6

    xarm_moveit_realmove_launch_description_1 = get_xarm_moveit_realmove_launch_description(
        robot1_ip, report_type,
        prefix, hw_ns, limited, effort_control, velocity_control, add_gripper, add_vacuum_gripper,
        dof=str(dof), xarm_type='xarm{}'.format(dof),
        ros_namespace_name='ns1', ros_namespace_default_value=ns1_default_value
    )
    xarm_moveit_realmove_launch_description_2 = get_xarm_moveit_realmove_launch_description(
        robot2_ip, report_type,
        prefix, hw_ns, limited, effort_control, velocity_control, add_gripper, add_vacuum_gripper,
        dof=str(dof), xarm_type='xarm{}'.format(dof),
        ros_namespace_name='ns2', ros_namespace_default_value=ns2_default_value
    )
    return LaunchDescription([
        xarm_moveit_realmove_launch_description_1,
        xarm_moveit_realmove_launch_description_2,
    ])
