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
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


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
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=False)
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='uf_robot_hardware/UFRobotFakeSystemHardware')
    controllers_name = LaunchConfiguration('controllers_name', default='fake_controllers')
    moveit_controller_manager_key = LaunchConfiguration('moveit_controller_manager_key', default='moveit_fake_controller_manager')
    moveit_controller_manager_value = LaunchConfiguration('moveit_controller_manager_value', default='moveit_fake_controller_manager/MoveItFakeControllerManager')

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

    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    moveit_config_package_name = 'xarm_moveit_config'
    xarm_type = '{}{}'.format(robot_type_1.perform(context), dof_1.perform(context) if robot_type_1.perform(context) in ('xarm', 'lite') else '')

    # robot_description_parameters
    # xarm_moveit_config/launch/lib/robot_moveit_config_lib.py
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory(moveit_config_package_name), 'launch', 'lib', 'robot_moveit_config_lib.py'))
    get_xarm_robot_description_parameters = getattr(mod, 'get_xarm_robot_description_parameters')
    robot_description_parameters = get_xarm_robot_description_parameters(
        xacro_urdf_file=PathJoinSubstitution([FindPackageShare('xarm_description'), 'urdf', 'dual_xarm_device.urdf.xacro']),
        xacro_srdf_file=PathJoinSubstitution([FindPackageShare(moveit_config_package_name), 'srdf', 'dual_xarm.srdf.xacro']),
        urdf_arguments={
            'prefix_1': prefix_1,
            'prefix_2': prefix_2,
            'dof_1': dof_1,
            'dof_2': dof_2,
            'robot_type_1': robot_type_1,
            'robot_type_2': robot_type_2,
            'add_gripper_1': add_gripper_1,
            'add_gripper_2': add_gripper_2,
            'add_vacuum_gripper_1': add_vacuum_gripper_1,
            'add_vacuum_gripper_2': add_vacuum_gripper_2,
            'add_bio_gripper_1': add_bio_gripper_1,
            'add_bio_gripper_2': add_bio_gripper_2,
            'hw_ns': hw_ns.perform(context).strip('/'),
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'ros2_control_plugin': ros2_control_plugin,
            'add_realsense_d435i_1': add_realsense_d435i_1,
            'add_realsense_d435i_2': add_realsense_d435i_2,
            'add_d435i_links_1': add_d435i_links_1,
            'add_d435i_links_2': add_d435i_links_2,
            'model1300_1': model1300_1,
            'model1300_2': model1300_2,
            'robot_sn_1': robot_sn_1,
            'robot_sn_2': robot_sn_2,
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
            'kinematics_suffix_1': kinematics_suffix_1,
            'kinematics_suffix_2': kinematics_suffix_2,
        },
        srdf_arguments={
            'prefix_1': prefix_1,
            'prefix_2': prefix_2,
            'dof_1': dof_1,
            'dof_2': dof_2,
            'robot_type_1': robot_type_1,
            'robot_type_2': robot_type_2,
            'add_gripper_1': add_gripper_1,
            'add_gripper_2': add_gripper_2,
            'add_vacuum_gripper_1': add_vacuum_gripper_1,
            'add_vacuum_gripper_2': add_vacuum_gripper_2,
            'add_bio_gripper_1': add_bio_gripper_1,
            'add_bio_gripper_2': add_bio_gripper_2,
            'add_other_geometry_1': add_other_geometry_1,
            'add_other_geometry_2': add_other_geometry_2,
        },
        arguments={
            'context': context,
            'xarm_type': xarm_type
        }
    )

    load_yaml = getattr(mod, 'load_yaml')

    controllers_yaml_1 = load_yaml(moveit_config_package_name, 'config', xarm_type, '{}.yaml'.format(controllers_name.perform(context)))
    ompl_planning_yaml_1 = load_yaml(moveit_config_package_name, 'config', xarm_type, 'ompl_planning.yaml')
    kinematics_yaml_1 = load_yaml(moveit_config_package_name, 'config', xarm_type, 'kinematics.yaml')
    joint_limits_yaml_1 = load_yaml(moveit_config_package_name, 'config', xarm_type, 'joint_limits.yaml')
    
    xarm_type = '{}{}'.format(robot_type_2.perform(context), dof_2.perform(context) if robot_type_2.perform(context) in ('xarm', 'lite') else '')
    controllers_yaml_2 = load_yaml(moveit_config_package_name, 'config', xarm_type, '{}.yaml'.format(controllers_name.perform(context)))
    ompl_planning_yaml_2 = load_yaml(moveit_config_package_name, 'config', xarm_type, 'ompl_planning.yaml')
    kinematics_yaml_2 = load_yaml(moveit_config_package_name, 'config', xarm_type, 'kinematics.yaml')
    joint_limits_yaml_2 = load_yaml(moveit_config_package_name, 'config', xarm_type, 'joint_limits.yaml')
    
        
    if add_gripper_1.perform(context) in ('True', 'true'):
        gripper_controllers_yaml_1 = load_yaml(moveit_config_package_name, 'config', '{}_gripper'.format(robot_type_1.perform(context)), '{}.yaml'.format(controllers_name.perform(context)))
        gripper_ompl_planning_yaml_1 = load_yaml(moveit_config_package_name, 'config', '{}_gripper'.format(robot_type_1.perform(context)), 'ompl_planning.yaml')
        # gripper_kinematics_yaml_1 = load_yaml(moveit_config_package_name, 'config', '{}_gripper'.format(robot_type_1.perform(context)), 'kinematics.yaml')
        gripper_joint_limits_yaml_1 = load_yaml(moveit_config_package_name, 'config', '{}_gripper'.format(robot_type_1.perform(context)), 'joint_limits.yaml')

        if gripper_controllers_yaml_1 and 'controller_names' in gripper_controllers_yaml_1:
            for name in gripper_controllers_yaml_1['controller_names']:
                if name not in gripper_controllers_yaml_1:
                    continue
                if name not in controllers_yaml_1['controller_names']:
                    controllers_yaml_1['controller_names'].append(name)
                controllers_yaml_1[name] = gripper_controllers_yaml_1[name]
        if gripper_ompl_planning_yaml_1:
            ompl_planning_yaml_1.update(gripper_ompl_planning_yaml_1)
        # kinematics_yaml_1.update(gripper_kinematics_yaml_1)
        if joint_limits_yaml_1 and gripper_joint_limits_yaml_1:
            joint_limits_yaml_1['joint_limits'].update(gripper_joint_limits_yaml_1['joint_limits'])
    elif add_bio_gripper_1.perform(context) in ('True', 'true'):
        gripper_controllers_yaml_1 = load_yaml(moveit_config_package_name, 'config', 'bio_gripper', '{}.yaml'.format(controllers_name.perform(context)))
        gripper_ompl_planning_yaml_1 = load_yaml(moveit_config_package_name, 'config', 'bio_gripper', 'ompl_planning.yaml')
        # gripper_kinematics_yaml_1 = load_yaml(moveit_config_package_name, 'config', 'bio_gripper', 'kinematics.yaml')
        gripper_joint_limits_yaml_1 = load_yaml(moveit_config_package_name, 'config', 'bio_gripper', 'joint_limits.yaml')

        if gripper_controllers_yaml_1 and 'controller_names' in gripper_controllers_yaml_1:
            for name in gripper_controllers_yaml_1['controller_names']:
                if name not in gripper_controllers_yaml_1:
                    continue
                if name not in controllers_yaml_1['controller_names']:
                    controllers_yaml_1['controller_names'].append(name)
                controllers_yaml_1[name] = gripper_controllers_yaml_1[name]
        if gripper_ompl_planning_yaml_1:
            ompl_planning_yaml_1.update(gripper_ompl_planning_yaml_1)
        # kinematics_yaml_1.update(gripper_kinematics_yaml_1)
        if joint_limits_yaml_1 and gripper_joint_limits_yaml_1:
            joint_limits_yaml_1['joint_limits'].update(gripper_joint_limits_yaml_1['joint_limits'])
    
    if add_gripper_2.perform(context) in ('True', 'true'):
        gripper_controllers_yaml_2 = load_yaml(moveit_config_package_name, 'config', '{}_gripper'.format(robot_type_2.perform(context)), '{}.yaml'.format(controllers_name.perform(context)))
        gripper_ompl_planning_yaml_2 = load_yaml(moveit_config_package_name, 'config', '{}_gripper'.format(robot_type_2.perform(context)), 'ompl_planning.yaml')
        gripper_joint_limits_yaml_2 = load_yaml(moveit_config_package_name, 'config', '{}_gripper'.format(robot_type_2.perform(context)), 'joint_limits.yaml')

        if gripper_controllers_yaml_2 and 'controller_names' in gripper_controllers_yaml_2:
            for name in gripper_controllers_yaml_2['controller_names']:
                if name not in gripper_controllers_yaml_2:
                    continue
                if name not in controllers_yaml_2['controller_names']:
                    controllers_yaml_2['controller_names'].append(name)
                controllers_yaml_2[name] = gripper_controllers_yaml_2[name]
        if gripper_ompl_planning_yaml_2:
            ompl_planning_yaml_2.update(gripper_ompl_planning_yaml_2)
        if joint_limits_yaml_2 and gripper_joint_limits_yaml_2:
            joint_limits_yaml_2['joint_limits'].update(gripper_joint_limits_yaml_2['joint_limits'])
    elif add_bio_gripper_2.perform(context) in ('True', 'true'):
        gripper_controllers_yaml_2 = load_yaml(moveit_config_package_name, 'config', 'bio_gripper', '{}.yaml'.format(controllers_name.perform(context)))
        gripper_ompl_planning_yaml_2 = load_yaml(moveit_config_package_name, 'config', 'bio_gripper', 'ompl_planning.yaml')
        gripper_joint_limits_yaml_2 = load_yaml(moveit_config_package_name, 'config', 'bio_gripper', 'joint_limits.yaml')

        if gripper_controllers_yaml_2 and 'controller_names' in gripper_controllers_yaml_2:
            for name in gripper_controllers_yaml_2['controller_names']:
                if name not in gripper_controllers_yaml_2:
                    continue
                if name not in controllers_yaml_2['controller_names']:
                    controllers_yaml_2['controller_names'].append(name)
                controllers_yaml_2[name] = gripper_controllers_yaml_2[name]
        if gripper_ompl_planning_yaml_2:
            ompl_planning_yaml_2.update(gripper_ompl_planning_yaml_2)
        if joint_limits_yaml_2 and gripper_joint_limits_yaml_2:
            joint_limits_yaml_2['joint_limits'].update(gripper_joint_limits_yaml_2['joint_limits'])

    add_prefix_to_moveit_params = getattr(mod, 'add_prefix_to_moveit_params')
    add_prefix_to_moveit_params(
        controllers_yaml=controllers_yaml_1, ompl_planning_yaml=ompl_planning_yaml_1, 
        kinematics_yaml=kinematics_yaml_1, joint_limits_yaml=joint_limits_yaml_1, 
        prefix=prefix_1.perform(context))
    add_prefix_to_moveit_params(
        controllers_yaml=controllers_yaml_2, ompl_planning_yaml=ompl_planning_yaml_2, 
        kinematics_yaml=kinematics_yaml_2, joint_limits_yaml=joint_limits_yaml_2, 
        prefix=prefix_2.perform(context))
    controllers_yaml = {}
    controllers_yaml.update(controllers_yaml_1)
    controllers_yaml.update(controllers_yaml_2)
    controllers_yaml['controller_names'].extend(controllers_yaml_1['controller_names'])
    ompl_planning_yaml = {}
    ompl_planning_yaml.update(ompl_planning_yaml_1)
    ompl_planning_yaml.update(ompl_planning_yaml_2)
    kinematics_yaml = {}
    kinematics_yaml.update(kinematics_yaml_1)
    kinematics_yaml.update(kinematics_yaml_2)
    robot_description_parameters['robot_description_kinematics'] = kinematics_yaml
    if 'robot_description_planning' in robot_description_parameters:
        joint_limits_yaml = {'joint_limits': {}}
        joint_limits_yaml['joint_limits'].update(joint_limits_yaml_1['joint_limits'])
        joint_limits_yaml['joint_limits'].update(joint_limits_yaml_2['joint_limits'])
        robot_description_parameters['robot_description_planning'] = joint_limits_yaml

    # Planning Configuration
    ompl_planning_pipeline_config = {
        'default_planning_pipeline': 'ompl',
        'planning_pipelines': ['ompl'],
    }
    if os.environ.get('ROS_DISTRO', '') > 'iron':
        ompl_planning_pipeline_config['ompl'] = {
            'planning_plugins': ['ompl_interface/OMPLPlanner'],
            'request_adapters': [
                'default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision',
            ],
            'response_adapters': [
                'default_planning_response_adapters/AddTimeOptimalParameterization',
                'default_planning_response_adapters/ValidateSolution',
                'default_planning_response_adapters/DisplayMotionPath',
            ],
        }
    else:
        ompl_planning_pipeline_config['ompl'] = {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            'start_state_max_bounds_error': 0.1,
        }
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    # Moveit controllers Configuration
    moveit_controllers = {
        moveit_controller_manager_key.perform(context): controllers_yaml,
        'moveit_controller_manager': moveit_controller_manager_value.perform(context),
    }

    # Trajectory Execution Configuration
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        'trajectory_execution.execution_duration_monitoring': False
    }
    
    plan_execution = {
        'plan_execution.record_trajectory_state_frequency': 10.0,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        # "planning_scene_monitor_options": {
        #     "name": "planning_scene_monitor",
        #     "robot_description": "robot_description",
        #     "joint_state_topic": "/joint_states",
        #     "attached_collision_object_topic": "/move_group/planning_scene_monitor",
        #     "publish_planning_scene_topic": "/move_group/publish_planning_scene",
        #     "monitored_planning_scene_topic": "/move_group/monitored_planning_scene",
        #     "wait_for_initial_state_timeout": 10.0,
        # },
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description_parameters,
            ompl_planning_pipeline_config,
            trajectory_execution,
            plan_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': use_sim_time},
        ],
    )

    # rviz with moveit configuration
    # rviz_config_file = PathJoinSubstitution([FindPackageShare(moveit_config_package_name), 'config', xarm_type, 'planner.rviz' if no_gui_ctrl.perform(context) == 'true' else 'moveit.rviz'])
    rviz_config_file = PathJoinSubstitution([FindPackageShare(moveit_config_package_name), 'rviz', 'dual_planner.rviz' if no_gui_ctrl.perform(context) == 'true' else 'dual_moveit.rviz'])
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description_parameters,
            ompl_planning_pipeline_config,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )
    
    link_base_1 = '{}link_base'.format(prefix_1.perform(context))
    link_base_2 = '{}link_base'.format(prefix_2.perform(context))

    # Static TF
    static_tf_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='{}static_transform_publisher'.format(prefix_1.perform(context)),
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', link_base_1],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    static_tf_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='{}static_transform_publisher'.format(prefix_2.perform(context)),
        output='screen',
        arguments=['0.0', '1.0', '0.0', '0.0', '0.0', '0.0', 'world', link_base_2],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return [
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=rviz2_node,
            on_exit=[EmitEvent(event=Shutdown())]
        )),
        rviz2_node,
        static_tf_1,
        static_tf_2,
        move_group_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
