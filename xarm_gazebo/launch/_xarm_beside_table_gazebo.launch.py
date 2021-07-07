#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import load_python_launch_file_as_module
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.actions import OpaqueFunction

    
def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    dof = LaunchConfiguration('dof', default=7)
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='gazebo_ros2_control/GazeboSystem')
    
    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')
    
    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)

    # ros2 control params
    # xarm_controller/launch/lib/xarm_controller_lib.py
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm_controller'), 'launch', 'lib', 'xarm_controller_lib.py'))
    generate_ros2_control_params_temp_file = getattr(mod, 'generate_ros2_control_params_temp_file')
    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'), 'config', 'xarm{}_controllers.yaml'.format(dof.perform(context))),
        prefix=prefix.perform(context), 
        add_gripper=add_gripper.perform(context) in ('True', 'true'),
        ros_namespace=LaunchConfiguration('ros_namespace', default='').perform(context),
        update_rate=1000,
    )

    # robot_description
    # xarm_description/launch/lib/xarm_description_lib.py
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm_description'), 'launch', 'lib', 'xarm_description_lib.py'))
    get_xacro_file_content = getattr(mod, 'get_xacro_file_content')
    robot_description = {
        'robot_description': get_xacro_file_content(
            xacro_file=PathJoinSubstitution([FindPackageShare('xarm_description'), 'urdf', 'xarm_device.urdf.xacro']), 
            arguments={
                'prefix': prefix,
                'dof': dof,
                'add_gripper': add_gripper,
                'add_vacuum_gripper': add_vacuum_gripper,
                'hw_ns': hw_ns.perform(context).strip('/'),
                'limited': limited,
                'effort_control': effort_control,
                'velocity_control': velocity_control,
                'ros2_control_plugin': ros2_control_plugin,
                'ros2_control_params': ros2_control_params,
                'add_other_geometry': add_other_geometry,
                'geometry_type': geometry_type,
                'geometry_mass': geometry_mass,
                'geometry_height': geometry_height,
                'geometry_radius': geometry_radius,
                'geometry_length': geometry_length,
                'geometry_width': geometry_width,
                'geometry_mesh_filename': geometry_mesh_filename,
                'geometry_mesh_origin_xyz': geometry_mesh_origin_xyz,
                'geometry_mesh_origin_rpy': geometry_mesh_origin_rpy,
                'geometry_mesh_tcp_xyz': geometry_mesh_tcp_xyz,
                'geometry_mesh_tcp_rpy': geometry_mesh_tcp_rpy,
            }
        ),
    }

    # robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # gazebo launch
    # gazebo_ros/launch/gazebo.launch.py
    xarm_gazebo_world = PathJoinSubstitution([FindPackageShare('xarm_gazebo'), 'worlds', 'table.world'])
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])),
        launch_arguments={
            'world': xarm_gazebo_world
        }.items(),
    )

    # gazebo spawn entity node
    gazebo_spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'xarm{}'.format(dof.perform(context)),
            '-x', '-0.2',
            '-y', '-0.5',
            '-z', '1.021',
            '-Y', '1.571',
        ],
    )

    # Load controllers
    controllers = [
        'joint_state_controller',
        '{}xarm{}_traj_controller'.format(prefix.perform(context), dof.perform(context)),
    ]
    if add_gripper.perform(context) in ('True', 'true'):
        controllers.append('{}xarm_gripper_traj_controller'.format(prefix.perform(context)))
    load_controllers = []
    for controller in controllers:
        load_controllers.append(Node(
            package='controller_manager',
            executable='spawner.py',
            output='screen',
            arguments=[
                controller,
                '--controller-manager', '{}/controller_manager'.format(ros_namespace)
            ],
        ))

    return [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gazebo_spawn_entity_node,
                on_exit=load_controllers,
            )
        ),
        gazebo_launch,
        robot_state_publisher_node,
        gazebo_spawn_entity_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
