#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import yaml
from pathlib import Path
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.actions import OpaqueFunction
from uf_ros_lib.uf_robot_utils import get_xacro_content, generate_dual_ros2_control_params_temp_file

    
def launch_setup(context, *args, **kwargs):
    prefix_1 = LaunchConfiguration('prefix_1', default='L_')
    prefix_2 = LaunchConfiguration('prefix_2', default='R_')
    dof = LaunchConfiguration('dof', default=7)
    dof_1 = LaunchConfiguration('dof_1', default=dof)
    dof_2 = LaunchConfiguration('dof_2', default=dof)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    robot_type_1 = LaunchConfiguration('robot_type_1', default=robot_type)
    robot_type_2 = LaunchConfiguration('robot_type_2', default=robot_type)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_gripper_1 = LaunchConfiguration('add_gripper_1', default=add_gripper)
    add_gripper_2 = LaunchConfiguration('add_gripper_2', default=add_gripper)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_vacuum_gripper_1 = LaunchConfiguration('add_vacuum_gripper_1', default=add_vacuum_gripper)
    add_vacuum_gripper_2 = LaunchConfiguration('add_vacuum_gripper_2', default=add_vacuum_gripper)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    add_bio_gripper_1 = LaunchConfiguration('add_bio_gripper_1', default=add_bio_gripper)
    add_bio_gripper_2 = LaunchConfiguration('add_bio_gripper_2', default=add_bio_gripper)
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='gazebo_ros2_control/GazeboSystem')
    
    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)
    add_realsense_d435i_1 = LaunchConfiguration('add_realsense_d435i_1', default=add_realsense_d435i)
    add_realsense_d435i_2 = LaunchConfiguration('add_realsense_d435i_2', default=add_realsense_d435i)

    add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)
    add_d435i_links_1 = LaunchConfiguration('add_d435i_links_1', default=add_d435i_links)
    add_d435i_links_2 = LaunchConfiguration('add_d435i_links_2', default=add_d435i_links)
    
    model1300 = LaunchConfiguration('model1300', default=False)
    model1300_1 = LaunchConfiguration('model1300_1', default=model1300)
    model1300_2 = LaunchConfiguration('model1300_2', default=model1300)

    robot_sn = LaunchConfiguration('robot_sn', default='')
    robot_sn_1 = LaunchConfiguration('robot_sn_1', default=robot_sn)
    robot_sn_2 = LaunchConfiguration('robot_sn_2', default=robot_sn)

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

    kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')
    kinematics_suffix_1 = LaunchConfiguration('kinematics_suffix_1', default=kinematics_suffix)
    kinematics_suffix_2 = LaunchConfiguration('kinematics_suffix_2', default=kinematics_suffix)

    load_controller = LaunchConfiguration('load_controller', default=False)
    show_rviz = LaunchConfiguration('show_rviz', default=False)
    no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=False)

    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)
    moveit_config_dump = LaunchConfiguration('moveit_config_dump', default='')

    moveit_config_dump = moveit_config_dump.perform(context)
    moveit_config_dict = yaml.load(moveit_config_dump, Loader=yaml.FullLoader) if moveit_config_dump else {}
    moveit_config_package_name = 'xarm_moveit_config'
    xarm_type_1 = '{}{}'.format(robot_type_1.perform(context), dof_1.perform(context) if robot_type_1.perform(context) in ('xarm', 'lite') else '')
    xarm_type_2 = '{}{}'.format(robot_type_2.perform(context), dof_2.perform(context) if robot_type_2.perform(context) in ('xarm', 'lite') else '')

    if not moveit_config_dict:
        # ros2 control params
        ros2_control_params = generate_dual_ros2_control_params_temp_file(
            os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}_controllers.yaml'.format(xarm_type_1)),
            os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}_controllers.yaml'.format(xarm_type_2)),
            prefix_1=prefix_1.perform(context), 
            prefix_2=prefix_2.perform(context), 
            add_gripper_1=add_gripper_1.perform(context) in ('True', 'true'),
            add_gripper_2=add_gripper_2.perform(context) in ('True', 'true'),
            add_bio_gripper_1=add_bio_gripper_1.perform(context) in ('True', 'true'),
            add_bio_gripper_2=add_bio_gripper_2.perform(context) in ('True', 'true'),
            ros_namespace=ros_namespace,
            update_rate=1000,
            robot_type_1=robot_type_1.perform(context), 
            robot_type_2=robot_type_2.perform(context), 
        )

        # robot_description
        robot_description = {
            'robot_description': get_xacro_content(
                context,
                xacro_file=Path(get_package_share_directory('xarm_description')) / 'urdf' / 'dual_xarm_device.urdf.xacro',
                dof_1=dof_1,
                dof_2=dof_2,
                robot_type_1=robot_type_1,
                robot_type_2=robot_type_2,
                prefix_1=prefix_1,
                prefix_2=prefix_2,
                hw_ns=hw_ns,
                limited=limited,
                effort_control=effort_control,
                velocity_control=velocity_control,
                model1300_1=model1300_1,
                model1300_2=model1300_2,
                robot_sn_1=robot_sn_1,
                robot_sn_2=robot_sn_2,
                kinematics_suffix_1=kinematics_suffix_1,
                kinematics_suffix_2=kinematics_suffix_2,
                ros2_control_plugin=ros2_control_plugin,
                ros2_control_params=ros2_control_params,
                add_gripper_1=add_gripper_1,
                add_gripper_2=add_gripper_2,
                add_vacuum_gripper_1=add_vacuum_gripper_1,
                add_vacuum_gripper_2=add_vacuum_gripper_2,
                add_bio_gripper_1=add_bio_gripper_1,
                add_bio_gripper_2=add_bio_gripper_2,
                add_realsense_d435i_1=add_realsense_d435i_1,
                add_realsense_d435i_2=add_realsense_d435i_2,
                add_d435i_links_1=add_d435i_links_1,
                add_d435i_links_2=add_d435i_links_2,
                add_other_geometry_1=add_other_geometry_1,
                add_other_geometry_2=add_other_geometry_2,
                geometry_type_1=geometry_type_1,
                geometry_type_2=geometry_type_2,
                geometry_mass_1=geometry_mass_1,
                geometry_mass_2=geometry_mass_2,
                geometry_height_1=geometry_height_1,
                geometry_height_2=geometry_height_2,
                geometry_radius_1=geometry_radius_1,
                geometry_radius_2=geometry_radius_2,
                geometry_length_1=geometry_length_1,
                geometry_length_2=geometry_length_2,
                geometry_width_1=geometry_width_1,
                geometry_width_2=geometry_width_2,
                geometry_mesh_filename_1=geometry_mesh_filename_1,
                geometry_mesh_filename_2=geometry_mesh_filename_2,
                geometry_mesh_origin_xyz_1=geometry_mesh_origin_xyz_1,
                geometry_mesh_origin_xyz_2=geometry_mesh_origin_xyz_2,
                geometry_mesh_origin_rpy_1=geometry_mesh_origin_rpy_1,
                geometry_mesh_origin_rpy_2=geometry_mesh_origin_rpy_2,
                geometry_mesh_tcp_xyz_1=geometry_mesh_tcp_xyz_1,
                geometry_mesh_tcp_xyz_2=geometry_mesh_tcp_xyz_2,
                geometry_mesh_tcp_rpy_1=geometry_mesh_tcp_rpy_1,
                geometry_mesh_tcp_rpy_2=geometry_mesh_tcp_rpy_2,
            )
        }
        moveit_config_dict = robot_description
    else:
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
    xarm_gazebo_world = PathJoinSubstitution([FindPackageShare('xarm_gazebo'), 'worlds', 'table.world'])
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])),
        launch_arguments={
            'world': xarm_gazebo_world,
            'server_required': 'true',
            'gui_required': 'true',
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
            '-entity', 'DUAL_UF_ROBOT',
            '-x', '0.5',
            '-y', '-0.54' if robot_type.perform(context) == 'uf850' else '-0.5',
            '-z', '1.021',
            '-Y', '1.571',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution([FindPackageShare(moveit_config_package_name), 'rviz', 'dual_planner.rviz' if no_gui_ctrl.perform(context) == 'true' else 'dual_moveit.rviz'])
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
        '{}{}_traj_controller'.format(prefix_1.perform(context), xarm_type_1),
        '{}{}_traj_controller'.format(prefix_2.perform(context), xarm_type_2),
    ]
    # check robot_type is not lite
    if robot_type_1.perform(context) != 'lite' and add_gripper_1.perform(context) in ('True', 'true'):
        controllers.append('{}{}_gripper_traj_controller'.format(prefix_1.perform(context), robot_type_1.perform(context)))
    elif robot_type_1.perform(context) != 'lite' and add_bio_gripper_1.perform(context) in ('True', 'true'):
        controllers.append('{}bio_gripper_traj_controller'.format(prefix_1.perform(context)))
    # check robot_type is not lite
    if robot_type_2.perform(context) != 'lite' and add_gripper_2.perform(context) in ('True', 'true'):
        controllers.append('{}{}_gripper_traj_controller'.format(prefix_2.perform(context), robot_type_2.perform(context)))
    elif robot_type_2.perform(context) != 'lite' and add_bio_gripper_2.perform(context) in ('True', 'true'):
        controllers.append('{}bio_gripper_traj_controller'.format(prefix_2.perform(context)))
    
    controller_nodes = []
    if load_controller.perform(context) in ('True', 'true'):
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
            robot_state_publisher_node,
            # gazebo_launch,
            # gazebo_spawn_entity_node,
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
            robot_state_publisher_node,
            # gazebo_launch,
            # gazebo_spawn_entity_node,
        ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
