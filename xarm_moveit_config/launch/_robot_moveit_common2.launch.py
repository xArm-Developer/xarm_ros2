#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration('prefix', default='')
    attach_to = LaunchConfiguration('attach_to', default='world')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')
    no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=False)
    show_rviz = LaunchConfiguration('show_rviz', default=True)
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    moveit_config_dump = LaunchConfiguration('moveit_config_dump')
    
    moveit_config_dump = moveit_config_dump.perform(context)
    moveit_config_dict = yaml.load(moveit_config_dump, Loader=yaml.FullLoader)
    moveit_config_package_name = 'xarm_moveit_config'

    # Start the actual move_group node/action server
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config_dict,
            {'use_sim_time': use_sim_time},
        ],
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution([FindPackageShare(moveit_config_package_name), 'rviz', 'planner.rviz' if no_gui_ctrl.perform(context) == 'true' else 'moveit.rviz'])
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            {
                'robot_description': moveit_config_dict['robot_description'],
                'robot_description_semantic': moveit_config_dict['robot_description_semantic'],
                'robot_description_kinematics': moveit_config_dict['robot_description_kinematics'],
                'robot_description_planning': moveit_config_dict['robot_description_planning'],
                'planning_pipelines': moveit_config_dict['planning_pipelines'],
                'use_sim_time': use_sim_time
            }
        ],
        condition=IfCondition(show_rviz),
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    xyz = attach_xyz.perform(context)[1:-1].split(' ')
    rpy = attach_rpy.perform(context)[1:-1].split(' ')
    args = xyz + rpy + [attach_to.perform(context), '{}link_base'.format(prefix.perform(context))]

    # Static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=args,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    robot_planner_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_planner'), 'launch', '_robot_planner.launch.py'])),
        condition=IfCondition(no_gui_ctrl),
        launch_arguments={
            'moveit_config_dump': moveit_config_dump,
        }.items(),
    )

    return [
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=rviz2_node,
            on_exit=[EmitEvent(event=Shutdown())]
        )),
        rviz2_node,
        static_tf,
        move_group_node,
        robot_planner_node_launch
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
