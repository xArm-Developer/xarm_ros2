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
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    dof = LaunchConfiguration('dof', default=7)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=False)
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='uf_robot_hardware/UFRobotFakeSystemHardware')
    controllers_name = LaunchConfiguration('controllers_name', default='fake_controllers')
    moveit_controller_manager_key = LaunchConfiguration('moveit_controller_manager_key', default='moveit_fake_controller_manager')
    moveit_controller_manager_value = LaunchConfiguration('moveit_controller_manager_value', default='moveit_fake_controller_manager/MoveItFakeControllerManager')

    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)
    add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)
    model1300 = LaunchConfiguration('model1300', default=False)

    attach_to = LaunchConfiguration('attach_to', default='world')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')

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

    kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')

    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    moveit_config_package_name = 'xarm_moveit_config'
    xarm_type = '{}{}'.format(robot_type.perform(context), dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else '')

    # robot_description_parameters
    # xarm_moveit_config/launch/lib/robot_moveit_config_lib.py
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory(moveit_config_package_name), 'launch', 'lib', 'robot_moveit_config_lib.py'))
    get_xarm_robot_description_parameters = getattr(mod, 'get_xarm_robot_description_parameters')
    robot_description_parameters = get_xarm_robot_description_parameters(
        xacro_urdf_file=PathJoinSubstitution([FindPackageShare('xarm_description'), 'urdf', 'xarm_device.urdf.xacro']),
        xacro_srdf_file=PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'srdf', 'xarm.srdf.xacro']),
        urdf_arguments={
            'prefix': prefix,
            'hw_ns': hw_ns.perform(context).strip('/'),
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'add_bio_gripper': add_bio_gripper,
            'dof': dof,
            'robot_type': robot_type,
            'ros2_control_plugin': ros2_control_plugin,
            'add_realsense_d435i': add_realsense_d435i,
            'add_d435i_links': add_d435i_links,
            'model1300': model1300,
            'attach_to': attach_to,
            'attach_xyz': attach_xyz,
            'attach_rpy': attach_rpy,
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
            'kinematics_suffix': kinematics_suffix,
        },
        srdf_arguments={
            'prefix': prefix,
            'dof': dof,
            'robot_type': robot_type,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'add_bio_gripper': add_bio_gripper,
            'add_other_geometry': add_other_geometry,
        },
        arguments={
            'context': context,
            'xarm_type': xarm_type,
        }
    )

    load_yaml = getattr(mod, 'load_yaml')
    controllers_yaml = load_yaml(moveit_config_package_name, 'config', xarm_type, '{}.yaml'.format(controllers_name.perform(context)))
    ompl_planning_yaml = load_yaml(moveit_config_package_name, 'config', xarm_type, 'ompl_planning.yaml')
    kinematics_yaml = robot_description_parameters['robot_description_kinematics']
    joint_limits_yaml = robot_description_parameters.get('robot_description_planning', None)

    if add_gripper.perform(context) in ('True', 'true'):
        gripper_controllers_yaml = load_yaml(moveit_config_package_name, 'config', '{}_gripper'.format(robot_type.perform(context)), '{}.yaml'.format(controllers_name.perform(context)))
        gripper_ompl_planning_yaml = load_yaml(moveit_config_package_name, 'config', '{}_gripper'.format(robot_type.perform(context)), 'ompl_planning.yaml')
        gripper_joint_limits_yaml = load_yaml(moveit_config_package_name, 'config', '{}_gripper'.format(robot_type.perform(context)), 'joint_limits.yaml')

        if gripper_controllers_yaml and 'controller_names' in gripper_controllers_yaml:
            for name in gripper_controllers_yaml['controller_names']:
                if name in gripper_controllers_yaml:
                    if name not in controllers_yaml['controller_names']:
                        controllers_yaml['controller_names'].append(name)
                    controllers_yaml[name] = gripper_controllers_yaml[name]
        if gripper_ompl_planning_yaml:
            ompl_planning_yaml.update(gripper_ompl_planning_yaml)
        if joint_limits_yaml and gripper_joint_limits_yaml:
            joint_limits_yaml['joint_limits'].update(gripper_joint_limits_yaml['joint_limits'])
    elif add_bio_gripper.perform(context) in ('True', 'true'):
        gripper_controllers_yaml = load_yaml(moveit_config_package_name, 'config', 'bio_gripper', '{}.yaml'.format(controllers_name.perform(context)))
        gripper_ompl_planning_yaml = load_yaml(moveit_config_package_name, 'config', 'bio_gripper', 'ompl_planning.yaml')
        gripper_joint_limits_yaml = load_yaml(moveit_config_package_name, 'config', 'bio_gripper', 'joint_limits.yaml')

        if gripper_controllers_yaml and 'controller_names' in gripper_controllers_yaml:
            for name in gripper_controllers_yaml['controller_names']:
                if name in gripper_controllers_yaml:
                    if name not in controllers_yaml['controller_names']:
                        controllers_yaml['controller_names'].append(name)
                    controllers_yaml[name] = gripper_controllers_yaml[name]
        if gripper_ompl_planning_yaml:
            ompl_planning_yaml.update(gripper_ompl_planning_yaml)
        if joint_limits_yaml and gripper_joint_limits_yaml:
            joint_limits_yaml['joint_limits'].update(gripper_joint_limits_yaml['joint_limits'])

    add_prefix_to_moveit_params = getattr(mod, 'add_prefix_to_moveit_params')
    add_prefix_to_moveit_params(
        controllers_yaml=controllers_yaml, ompl_planning_yaml=ompl_planning_yaml, 
        kinematics_yaml=kinematics_yaml, joint_limits_yaml=joint_limits_yaml, 
        prefix=prefix.perform(context))

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

    # sensor_manager_parameters = {
    #     'sensors': ['ros'],
    #     'octomap_resolution': 0.02,
    #     'ros.sensor_plugin': 'occupancy_map_monitor/PointCloudOctomapUpdater',
    #     'ros.point_cloud_topic': '/camera/depth/color/points',
    #     'ros.max_range': 2.0,
    #     'ros.point_subsample': 1,
    #     'ros.padding_offset': 0.1,
    #     'ros.padding_scale': 1.0,
    #     'ros.max_update_rate': 1.0,
    #     'ros.filtered_cloud_topic': 'filtered_cloud',
    # }

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
            # sensor_manager_parameters,
            {'use_sim_time': use_sim_time},
        ],
    )

    # rviz with moveit configuration
    # rviz_config_file = PathJoinSubstitution([FindPackageShare(moveit_config_package_name), 'config', xarm_type, 'planner.rviz' if no_gui_ctrl.perform(context) == 'true' else 'moveit.rviz'])
    rviz_config_file = PathJoinSubstitution([FindPackageShare(moveit_config_package_name), 'rviz', 'planner.rviz' if no_gui_ctrl.perform(context) == 'true' else 'moveit.rviz'])
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

    xyz = attach_xyz.perform(context)[1:-1].split(' ')
    rpy = attach_rpy.perform(context)[1:-1].split(' ')
    args = xyz + rpy + ['world', '{}link_base'.format(prefix.perform(context))]

    # Static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        # arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'link_base'],
        arguments=args,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return [
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=rviz2_node,
            on_exit=[EmitEvent(event=Shutdown())]
        )),
        rviz2_node,
        static_tf,
        move_group_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
