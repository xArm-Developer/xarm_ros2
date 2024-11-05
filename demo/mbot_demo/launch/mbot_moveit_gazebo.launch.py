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
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file


def launch_setup(context, *args, **kwargs):
    dof = LaunchConfiguration('dof', default=6)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=True)
    attach_to = LaunchConfiguration('attach_to', default='base_link')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0.08"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')

    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)

    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)

    # ros2_control_plugin = 'gazebo_ros2_control/GazeboSystem'
    ros2_control_plugin = 'gz_ros2_control/GazeboSimSystem'

    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('mbot_demo'), 'config', 'ros2_controllers.yaml'),
        prefix=prefix.perform(context), 
        add_gripper=add_gripper.perform(context) in ('True', 'true'),
        add_bio_gripper=add_bio_gripper.perform(context) in ('True', 'true'),
        ros_namespace=ros_namespace,
        update_rate=1000,
        use_sim_time=True,
        robot_type=robot_type.perform(context)
    )

    pkg_path = os.path.join(get_package_share_directory('mbot_demo'))
    urdf_file = os.path.join(pkg_path, 'urdf', 'mbot_with_xarm.urdf.xacro')
    srdf_file = os.path.join(pkg_path, 'srdf', 'mbot_with_xarm.srdf.xacro')

    controllers_file = os.path.join(pkg_path, 'config', 'controllers.yaml')
    joint_limits_file = os.path.join(pkg_path, 'config', 'joint_limits.yaml')
    kinematics_file = os.path.join(pkg_path, 'config', 'kinematics.yaml')
    pipeline_filedir = os.path.join(pkg_path, 'config')

    moveit_config = (
        MoveItConfigsBuilder(
            context=context,
            dof=dof,
            robot_type=robot_type,
            prefix=prefix,
            hw_ns=hw_ns,
            limited=limited,
            attach_to=attach_to,
            attach_xyz=attach_xyz,
            attach_rpy=attach_rpy,
            ros2_control_plugin=ros2_control_plugin,
            ros2_control_params=ros2_control_params,
            add_gripper=add_gripper,
            add_vacuum_gripper=add_vacuum_gripper,
            add_bio_gripper=add_bio_gripper,
        )
        .robot_description(file_path=urdf_file)
        .robot_description_semantic(file_path=srdf_file)
        .robot_description_kinematics(file_path=kinematics_file)
        .joint_limits(file_path=joint_limits_file)
        .trajectory_execution(file_path=controllers_file)
        .planning_pipelines(config_folder=pipeline_filedir)
        .to_moveit_configs()
    )

    moveit_config_dump = yaml.dump(moveit_config.to_dict())

    # robot moveit common launch
    # xarm_moveit_config/launch/_robot_moveit_common2.launch.py
    robot_moveit_common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_robot_moveit_common2.launch.py'])),
        launch_arguments={
            'prefix': prefix,
            'attach_to': attach_to,
            'attach_xyz': attach_xyz,
            'attach_rpy': attach_rpy,
            'show_rviz': 'false',
            'use_sim_time': 'true',
            'moveit_config_dump': moveit_config_dump,
            'rviz_config': PathJoinSubstitution([FindPackageShare('mbot_demo'), 'rviz', 'moveit.rviz'])
        }.items(),
    )

    # robot gazebo launch
    # mbot_demo/launch/_robot_on_mbot_gazebo.launch.py
    robot_gazebo_launch = IncludeLaunchDescription(
        # PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('mbot_demo'), 'launch', '_robot_on_mbot_gazebo.launch.py'])),
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('mbot_demo'), 'launch', '_robot_on_mbot_gz.launch.py'])),
        launch_arguments={
            'dof': dof,
            'robot_type': robot_type,
            'prefix': prefix,
            'moveit_config_dump': moveit_config_dump,
            'show_rviz': 'true',
            'rviz_config': PathJoinSubstitution([FindPackageShare('mbot_demo'), 'rviz', 'moveit.rviz'])
        }.items(),
    )

    return [
        robot_gazebo_launch,
        robot_moveit_common_launch,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
