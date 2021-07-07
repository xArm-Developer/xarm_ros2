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
    prefix_1 = LaunchConfiguration('prefix_1', default='L_')
    prefix_2 = LaunchConfiguration('prefix_2', default='R_')
    dof = LaunchConfiguration('dof', default=7)
    dof_1 = LaunchConfiguration('dof_1', default=dof)
    dof_2 = LaunchConfiguration('dof_2', default=dof)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_gripper_1 = LaunchConfiguration('add_gripper_1', default=add_gripper)
    add_gripper_2 = LaunchConfiguration('add_gripper_2', default=add_gripper)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_vacuum_gripper_1 = LaunchConfiguration('add_vacuum_gripper_1', default=add_vacuum_gripper)
    add_vacuum_gripper_2 = LaunchConfiguration('add_vacuum_gripper_2', default=add_vacuum_gripper)
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='gazebo_ros2_control/GazeboSystem')
    
    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    add_other_geometry_1 = LaunchConfiguration('add_other_geometry_1', default=add_other_geometry)
    add_other_geometry_2 = LaunchConfiguration('add_other_geometry_2', default=add_other_geometry)

    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_type_1 = LaunchConfiguration('geometry_type_1', default=geometry_type)
    geometry_type_2 = LaunchConfiguration('geometry_type_2', default=geometry_type)

    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_mass_1 = LaunchConfiguration('geometry_mass_1', default=geometry_mass)
    geometry_mass_2 = LaunchConfiguration('geometry_mass_2', default=geometry_mass)

    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_height_1 = LaunchConfiguration('geometry_height_1', default=geometry_height)
    geometry_height_2 = LaunchConfiguration('geometry_height_2', default=geometry_height)

    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_radius_1 = LaunchConfiguration('geometry_radius_1', default=geometry_radius)
    geometry_radius_2 = LaunchConfiguration('geometry_radius_2', default=geometry_radius)
    
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_length_1 = LaunchConfiguration('geometry_length_1', default=geometry_length)
    geometry_length_2 = LaunchConfiguration('geometry_length_2', default=geometry_length)

    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_width_1 = LaunchConfiguration('geometry_width_1', default=geometry_width)
    geometry_width_2 = LaunchConfiguration('geometry_width_2', default=geometry_width)

    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_filename_1 = LaunchConfiguration('geometry_mesh_filename_1', default=geometry_mesh_filename)
    geometry_mesh_filename_2 = LaunchConfiguration('geometry_mesh_filename_2', default=geometry_mesh_filename)
    
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_xyz_1 = LaunchConfiguration('geometry_mesh_origin_xyz_1', default=geometry_mesh_origin_xyz)
    geometry_mesh_origin_xyz_2 = LaunchConfiguration('geometry_mesh_origin_xyz_2', default=geometry_mesh_origin_xyz)
    
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_origin_rpy_1 = LaunchConfiguration('geometry_mesh_origin_rpy_1', default=geometry_mesh_origin_rpy)
    geometry_mesh_origin_rpy_2 = LaunchConfiguration('geometry_mesh_origin_rpy_2', default=geometry_mesh_origin_rpy)
    
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_xyz_1 = LaunchConfiguration('geometry_mesh_tcp_xyz_1', default=geometry_mesh_tcp_xyz)
    geometry_mesh_tcp_xyz_2 = LaunchConfiguration('geometry_mesh_tcp_xyz_2', default=geometry_mesh_tcp_xyz)
    
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')
    geometry_mesh_tcp_rpy_1 = LaunchConfiguration('geometry_mesh_tcp_rpy_1', default=geometry_mesh_tcp_rpy)
    geometry_mesh_tcp_rpy_2 = LaunchConfiguration('geometry_mesh_tcp_rpy_2', default=geometry_mesh_tcp_rpy)

    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)

    # ros2 control params
    # xarm_controller/launch/lib/xarm_controller_lib.py
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm_controller'), 'launch', 'lib', 'xarm_controller_lib.py'))
    generate_dual_ros2_control_params_temp_file = getattr(mod, 'generate_dual_ros2_control_params_temp_file')
    ros2_control_params = generate_dual_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'), 'config', 'xarm{}_controllers.yaml'.format(dof_1.perform(context))),
        os.path.join(get_package_share_directory('xarm_controller'), 'config', 'xarm{}_controllers.yaml'.format(dof_2.perform(context))),
        prefix_1=prefix_1.perform(context), 
        prefix_2=prefix_2.perform(context), 
        add_gripper_1=add_gripper_1.perform(context) in ('True', 'true'),
        add_gripper_2=add_gripper_2.perform(context) in ('True', 'true'),
        ros_namespace=ros_namespace,
        update_rate=1000,
    )

    # robot_description
    # xarm_description/launch/lib/xarm_description_lib.py
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm_description'), 'launch', 'lib', 'xarm_description_lib.py'))
    get_xacro_file_content = getattr(mod, 'get_xacro_file_content')
    robot_description = {
        'robot_description': get_xacro_file_content(
            xacro_file=PathJoinSubstitution([FindPackageShare('xarm_description'), 'urdf', 'dual_xarm_device.urdf.xacro']), 
            arguments={
                'prefix_1': prefix_1,
                'prefix_2': prefix_2,
                'dof_1': dof_1,
                'dof_2': dof_2,
                'add_gripper_1': add_gripper_1,
                'add_gripper_2': add_gripper_2,
                'add_vacuum_gripper_1': add_vacuum_gripper_1,
                'add_vacuum_gripper_2': add_vacuum_gripper_2,
                'hw_ns': hw_ns.perform(context).strip('/'),
                'limited': limited,
                'effort_control': effort_control,
                'velocity_control': velocity_control,
                'ros2_control_plugin': ros2_control_plugin,
                'ros2_control_params': ros2_control_params,
                'add_other_geometry_1': add_other_geometry_1,
                'add_other_geometry_2': add_other_geometry_2,
                'geometry_type_1': geometry_type_1,
                'geometry_type_2': geometry_type_2,
                'geometry_mass_1': geometry_mass_1,
                'geometry_mass_2': geometry_mass_2,
                'geometry_height_1': geometry_height_1,
                'geometry_height_2': geometry_height_2,
                'geometry_radius_1': geometry_radius_1,
                'geometry_radius_2': geometry_radius_2,
                'geometry_length_1': geometry_length_1,
                'geometry_length_2': geometry_length_2,
                'geometry_width_1': geometry_width_1,
                'geometry_width_2': geometry_width_2,
                'geometry_mesh_filename_1': geometry_mesh_filename_1,
                'geometry_mesh_filename_2': geometry_mesh_filename_2,
                'geometry_mesh_origin_xyz_1': geometry_mesh_origin_xyz_1,
                'geometry_mesh_origin_xyz_2': geometry_mesh_origin_xyz_2,
                'geometry_mesh_origin_rpy_1': geometry_mesh_origin_rpy_1,
                'geometry_mesh_origin_rpy_2': geometry_mesh_origin_rpy_2,
                'geometry_mesh_tcp_xyz_1': geometry_mesh_tcp_xyz_1,
                'geometry_mesh_tcp_xyz_2': geometry_mesh_tcp_xyz_2,
                'geometry_mesh_tcp_rpy_1': geometry_mesh_tcp_rpy_1,
                'geometry_mesh_tcp_rpy_2': geometry_mesh_tcp_rpy_2,
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
            'world': xarm_gazebo_world,
            # 'pause': 'true'
        }.items(),
    )

    # gazebo spawn entity node
    gazebo_spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'dual_xarm',
            '-x', '0.5',
            '-y', '-0.5',
            '-z', '1.021',
            '-Y', '1.571',
        ],
    )

    # Load controllers
    controllers = [
        'joint_state_controller',
        '{}xarm{}_traj_controller'.format(prefix_1.perform(context), dof_1.perform(context)),
        '{}xarm{}_traj_controller'.format(prefix_2.perform(context), dof_2.perform(context)),
    ]
    if add_gripper_1.perform(context) in ('True', 'true'):
        controllers.append('{}xarm_gripper_traj_controller'.format(prefix_1.perform(context)))
    if add_gripper_2.perform(context) in ('True', 'true'):
        controllers.append('{}xarm_gripper_traj_controller'.format(prefix_2.perform(context)))
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
