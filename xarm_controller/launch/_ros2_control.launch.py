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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_path = get_package_share_directory('xarm_description')
sys.path.append(os.path.join(package_path, 'launch', 'lib'))
from xarm_description_lib import get_xarm_robot_description


def generate_launch_description():
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    dof = LaunchConfiguration('dof', default=7)
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='xarm_control/XArmHW')
    controller_params = LaunchConfiguration('controller_params', default=PathJoinSubstitution([FindPackageShare('xarm_controller'), 'config', 'xarm7_controllers.yaml']))

    # robot_description
    robot_description = get_xarm_robot_description(
        prefix, hw_ns, limited, 
        effort_control, velocity_control, 
        add_gripper, add_vacuum_gripper, 
        dof, ros2_control_plugin
    )
    
    # ros2 control node
    ros2_control_node = Node(
        # namespace='/',
        package="controller_manager",
        executable="ros2_control_node",
        # name='controller_manager',
        parameters=[
            robot_description,
            controller_params,
            # {
            #     'update_rate': 48,  # Hz
            #     'joint_state_controller.type': 'joint_state_controller/JointStateController',
            #     'xarm6_traj_controller.type': 'joint_trajectory_controller/JointTrajectoryController',
            #     'xarm6_traj_controller.joints': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
            #     'xarm6_traj_controller.interface_name': 'position',
            #     # 'joint_state_controller': {
            #     #     'type': 'joint_state_controller/JointStateController'
            #     # },
            #     # 'xarm6_traj_controller': {
            #     #     'type': 'joint_trajectory_controller/JointTrajectoryController',
            #     #     'joints': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
            #     #     'interface_name': 'position'
            #     # }
            # }
        ],
        output='screen',
    )

    return LaunchDescription([ros2_control_node])
