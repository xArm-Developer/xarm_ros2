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
from launch_ros.actions import Node
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file


def launch_setup(context, *args, **kwargs):
    dof = LaunchConfiguration('dof', default=6)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    prefix = LaunchConfiguration('prefix', default='')
    limited = LaunchConfiguration('limited', default=True)
    attach_to = LaunchConfiguration('attach_to', default='base_link')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0.08"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')

    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)

    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)

    ros2_control_plugin = 'uf_robot_hardware/UFRobotFakeSystemHardware'
    xarm_type = '{}{}'.format(robot_type.perform(context), dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else '')

    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('mbot_demo'), 'config', 'ros2_controllers.yaml'),
        prefix=prefix.perform(context), 
        add_gripper=add_gripper.perform(context) in ('True', 'true'),
        add_bio_gripper=add_bio_gripper.perform(context) in ('True', 'true'),
        ros_namespace=ros_namespace,
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
    
    # robot description launch
    # xarm_description/launch/_robot_description.launch.py
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_description'), 'launch', '_robot_description.launch.py'])),
        launch_arguments={
            'robot_description': yaml.dump(moveit_config.robot_description),
        }.items(),
    )

    # robot moveit common launch
    # xarm_moveit_config/launch/_robot_moveit_common2.launch.py
    robot_moveit_common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_robot_moveit_common2.launch.py'])),
        launch_arguments={
            'prefix': prefix,
            'attach_to': attach_to,
            'attach_xyz': attach_xyz,
            'attach_rpy': attach_rpy,
            'use_sim_time': 'false',
            'moveit_config_dump': yaml.dump(moveit_config.to_dict()),
            'rviz_config': PathJoinSubstitution([FindPackageShare('mbot_demo'), 'rviz', 'moveit.rviz'])
        }.items(),
    )

    controllers = [
        '{}{}_traj_controller'.format(prefix.perform(context), xarm_type),
        'mbot_traj_controller'
    ]
    if add_gripper.perform(context) in ('True', 'true') and robot_type.perform(context) != 'lite':
        controllers.append('{}{}_gripper_traj_controller'.format(prefix.perform(context), robot_type.perform(context)))
    elif add_bio_gripper.perform(context) in ('True', 'true') and robot_type.perform(context) != 'lite':
        controllers.append('{}bio_gripper_traj_controller'.format(prefix.perform(context)))

    # ros2 control launch
    # xarm_controller/launch/_ros2_control.launch.py
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_controller'), 'launch', '_ros2_control.launch.py'])),
        launch_arguments={
            'robot_description': yaml.dump(moveit_config.robot_description),
            'ros2_control_params': ros2_control_params,
        }.items(),
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '{}/controller_manager'.format(ros_namespace)
        ],
    )

    # Load controllers
    controller_nodes = []
    for controller in controllers:
        controller_nodes.append(Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                controller,
                '--controller-manager', '{}/controller_manager'.format(ros_namespace)
            ],
        ))

    return [
        robot_description_launch,
        robot_moveit_common_launch,
        joint_state_broadcaster,
        ros2_control_launch,
    ] + controller_nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
