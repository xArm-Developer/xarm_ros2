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
from launch.launch_description_sources import load_python_launch_file_as_module
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.actions import OpaqueFunction

    
def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration('prefix', default='')
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    dof = LaunchConfiguration('dof', default=7)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    show_rviz = LaunchConfiguration('show_rviz', default=False)

    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)
    rviz_config = LaunchConfiguration('rviz_config', default='')
    moveit_config_dump = LaunchConfiguration('moveit_config_dump', default='')

    moveit_config_dump = moveit_config_dump.perform(context)
    moveit_config_dict = yaml.load(moveit_config_dump, Loader=yaml.FullLoader) if moveit_config_dump else {}
    moveit_config_package_name = 'xarm_moveit_config'
    xarm_type = '{}{}'.format(robot_type.perform(context), dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else '')
    
    robot_description = {'robot_description': moveit_config_dict['robot_description']}

    # robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}, robot_description],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # gazebo launch
    # gazebo_ros/launch/gazebo.launch.py
    xarm_gazebo_world = PathJoinSubstitution([FindPackageShare('mbot_demo'), 'worlds', 'empty.world'])
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': ' -r -v 3 {}'.format(xarm_gazebo_world.perform(context)),
        }.items(),
    )

    # gazebo spawn entity node
    gazebo_spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'mbot_with_xarm',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # rviz with moveit configuration
    if not rviz_config.perform(context):
        rviz_config_file = PathJoinSubstitution([FindPackageShare(moveit_config_package_name), 'rviz', 'moveit.rviz'])
    else:
        rviz_config_file = rviz_config
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            {
                'robot_description': moveit_config_dict.get('robot_description', ''),
                'robot_description_semantic': moveit_config_dict.get('robot_description_semantic', ''),
                'robot_description_kinematics': moveit_config_dict.get('robot_description_kinematics', {}),
                'robot_description_planning': moveit_config_dict.get('robot_description_planning', {}),
                'planning_pipelines': moveit_config_dict.get('planning_pipelines', {}),
                'use_sim_time': True
            }
        ],
        # condition=IfCondition(show_rviz),
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # Load controllers
    controllers = [
        'joint_state_broadcaster',
        '{}{}_traj_controller'.format(prefix.perform(context), xarm_type),
        'mbot_traj_controller'
    ]
    if robot_type.perform(context) != 'lite' and add_gripper.perform(context) in ('True', 'true'):
        controllers.append('{}{}_gripper_traj_controller'.format(prefix.perform(context), robot_type.perform(context)))
    elif robot_type.perform(context) != 'lite' and add_bio_gripper.perform(context) in ('True', 'true'):
        controllers.append('{}bio_gripper_traj_controller'.format(prefix.perform(context)))
    
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
            parameters=[{'use_sim_time': True}],
        ))

    if len(controller_nodes) > 0:
        return [
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=robot_state_publisher_node,
                    on_start=gazebo_launch,
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=robot_state_publisher_node,
                    on_start=gazebo_spawn_entity_node,
                )
            ),
            RegisterEventHandler(
                condition=IfCondition(show_rviz),
                event_handler=OnProcessExit(
                    target_action=gazebo_spawn_entity_node,
                    on_exit=rviz2_node,
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gazebo_spawn_entity_node,
                    on_exit=controller_nodes,
                )
            ),
            robot_state_publisher_node
        ]
    else:
        return [
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=robot_state_publisher_node,
                    on_start=gazebo_launch,
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=robot_state_publisher_node,
                    on_start=gazebo_spawn_entity_node,
                )
            ),
            RegisterEventHandler(
                condition=IfCondition(show_rviz),
                event_handler=OnProcessExit(
                    target_action=gazebo_spawn_entity_node,
                    on_exit=rviz2_node,
                )
            ),
            robot_state_publisher_node
        ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
