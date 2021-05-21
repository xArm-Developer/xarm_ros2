#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import sys
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

package_path = get_package_share_directory('xarm_description')
sys.path.append(os.path.join(package_path, 'launch', 'lib'))
from xarm_description_lib import get_xarm_robot_description

package_path = get_package_share_directory('xarm_moveit_config')
sys.path.append(os.path.join(package_path, 'launch', 'lib'))
from xarm_moveit_config_lib import get_xarm_moveit_realmove_launch_description, load_file, load_yaml


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    report_type = LaunchConfiguration('report_type', default='normal')

    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)

    dof = 7

    robot_description = get_xarm_robot_description(
        prefix, hw_ns, limited, 
        effort_control, velocity_control, 
        add_gripper, add_vacuum_gripper, 
        str(dof), 'fake_components/GenericSystem'
    )
    robot_description_semantic = {'robot_description_semantic': load_file('xarm_moveit_config', 'srdf', 'xarm{}.srdf'.format(dof))}    
    robot_description_kinematics = {'robot_description_kinematics': load_yaml('xarm_moveit_config', 'config', 'xarm{}'.format(dof), 'kinematics.yaml')}


    xarm_planner_node = Node(
        name="xarm_planner_node",
        package="xarm_planner",
        executable="xarm_planner_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics['robot_description_kinematics'],
            {'DOF': dof},
        ],
    )

    xarm_moveit_launch_description = get_xarm_moveit_realmove_launch_description(
        robot_ip, report_type,
        prefix, hw_ns, limited, effort_control, velocity_control, add_gripper, add_vacuum_gripper,
        dof=str(dof), xarm_type='xarm{}'.format(dof),
        no_gui_ctrl=True,
    )
    return LaunchDescription([
        xarm_moveit_launch_description,
        xarm_planner_node
    ])
