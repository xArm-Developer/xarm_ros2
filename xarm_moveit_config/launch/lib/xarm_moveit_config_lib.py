#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import sys
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.substitution import Substitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from ros2launch.api.api import parse_launch_arguments
import xacro

package_path = get_package_share_directory('xarm_description')
sys.path.append(os.path.join(package_path, 'launch', 'lib'))
from xarm_description_lib import get_xarm_robot_description

package_path = get_package_share_directory('xarm_controller')
sys.path.append(os.path.join(package_path, 'launch', 'lib'))
from xarm_controller_lib import get_sys_param, get_controller_params


def load_file(package_name, *file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, *file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, *file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, *file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def get_xarm_moveit_common_launch_entities(
    prefix, hw_ns, limited, 
    effort_control, velocity_control, 
    add_gripper, add_vacuum_gripper, dof,
    ros2_control_plugin,
    xarm_type='xarm7',
    no_gui_ctrl=False,
    controllers_name='fake_controllers',
    moveit_controller_manager_key='moveit_fake_controller_manager', 
    moveit_controller_manager_value='moveit_fake_controller_manager/MoveItFakeControllerManager'):

    # robot_description
    robot_description = get_xarm_robot_description(
        prefix, hw_ns, limited, 
        effort_control, velocity_control, 
        add_gripper, add_vacuum_gripper, 
        dof, ros2_control_plugin
    )

    moveit_config_package_name = 'xarm_moveit_config'

    robot_description_semantic = {'robot_description_semantic': load_file(moveit_config_package_name, 'srdf', '{}.srdf'.format(xarm_type))}    
    robot_description_kinematics = {'robot_description_kinematics': load_yaml(moveit_config_package_name, 'config', xarm_type, 'kinematics.yaml')}
    robot_description_planning = {'robot_description_planning': load_yaml(moveit_config_package_name, 'config', xarm_type, 'joint_limits.yaml')}

    # Planning Configuration
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_pipeline_config['move_group'].update(load_yaml(moveit_config_package_name, 'config', xarm_type, 'ompl_planning.yaml'))

    # Moveit controllers Configuration
    moveit_controllers = {
        moveit_controller_manager_key: load_yaml(moveit_config_package_name, 'config', xarm_type, '{}.yaml'.format(controllers_name)),
        'moveit_controller_manager': moveit_controller_manager_value,
    }

    # Trajectory Execution Configuration
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        'trajectory_execution.execution_duration_monitoring': False
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
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            # robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution([FindPackageShare(moveit_config_package_name), 'config', xarm_type, 'planner.rviz' if no_gui_ctrl else 'moveit.rviz'])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            # robot_description_planning,
            ompl_planning_pipeline_config,
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "link_base"],
    )

    return [
        rviz_node,
        static_tf,
        move_group_node,
    ]


def get_xarm_moveit_fake_launch_description(
    prefix, hw_ns, limited, 
    effort_control, velocity_control, 
    add_gripper, add_vacuum_gripper, 
    dof='7', xarm_type='xarm7',
    no_gui_ctrl=False,
    ros_namespace_name='ros_namespace', ros_namespace_default_value=''):

    ros2_control_plugin = 'fake_components/GenericSystem'
    controllers_name = 'fake_controllers'
    moveit_controller_manager_key = 'moveit_fake_controller_manager'
    moveit_controller_manager_value = 'moveit_fake_controller_manager/MoveItFakeControllerManager'

    ros_namespace = get_sys_param(ros_namespace_name, default=ros_namespace_default_value)

    # robot description launch
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_description'), 'launch', '_xarm_robot_description.launch.py'])),
        launch_arguments={
            'prefix': prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'dof': dof,
            'ros2_control_plugin': ros2_control_plugin,
            'joint_states_remapping': 'joint_states',
            'ros_namespace': ros_namespace,
        }.items(),
    )

    entities = get_xarm_moveit_common_launch_entities(
        prefix, hw_ns, limited, effort_control, velocity_control, add_gripper, add_vacuum_gripper, dof,
        ros2_control_plugin,
        xarm_type=xarm_type,
        no_gui_ctrl=no_gui_ctrl,
        controllers_name=controllers_name,
        moveit_controller_manager_key=moveit_controller_manager_key, 
        moveit_controller_manager_value=moveit_controller_manager_value,
    )

    # joint state publisher node
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name='joint_state_publisher',
        output='screen',
        parameters=[{"source_list": ["fake_controller_joint_states"]}],
    )
    return LaunchDescription([
        robot_description_launch, 
        joint_state_publisher_node
    ] + entities)


def get_xarm_moveit_realmove_launch_description(
    robot_ip, report_type,
    prefix, hw_ns, limited, 
    effort_control, velocity_control, 
    add_gripper, add_vacuum_gripper, 
    dof='7', xarm_type='xarm7',
    no_gui_ctrl=False,
    ros_namespace_name='ros_namespace', ros_namespace_default_value=''):

    ros2_control_plugin = 'xarm_control/XArmHW'
    controllers_name = 'controllers'
    moveit_controller_manager_key = 'moveit_simple_controller_manager'
    moveit_controller_manager_value = 'moveit_simple_controller_manager/MoveItSimpleControllerManager'

    ros_namespace = get_sys_param(ros_namespace_name, default=ros_namespace_default_value)
    joint_states_remapping = PathJoinSubstitution(['/', ros_namespace, hw_ns, 'joint_states'])
    controller_params = get_controller_params(dof, name=ros_namespace_name, default=ros_namespace_default_value)

    # xarm driver launch
    xarm_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_api'), 'launch', '_xarm_driver.launch.py'])),
        launch_arguments={
            'robot_ip': robot_ip,
            'report_type': report_type,
            'dof': dof,
            'hw_ns': hw_ns,
            'ros_namespace': ros_namespace,
        }.items(),
    )

    # robot description launch
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_description'), 'launch', '_xarm_robot_description.launch.py'])),
        launch_arguments={
            'prefix': prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'dof': dof,
            'ros2_control_plugin': ros2_control_plugin,
            # 'joint_states_remapping': PathJoinSubstitution([hw_ns, 'joint_states']),
            'joint_states_remapping': joint_states_remapping,
            'ros_namespace': ros_namespace,
        }.items(),
    )

    entities = get_xarm_moveit_common_launch_entities(
        prefix, hw_ns, limited, effort_control, velocity_control, add_gripper, add_vacuum_gripper, dof,
        ros2_control_plugin,
        xarm_type=xarm_type,
        no_gui_ctrl=no_gui_ctrl,
        controllers_name=controllers_name,
        moveit_controller_manager_key=moveit_controller_manager_key, 
        moveit_controller_manager_value=moveit_controller_manager_value,
    )
    
    launch_arguments_dict = dict(parse_launch_arguments(sys.argv[4:]))
    hw_ns_str = launch_arguments_dict.get('hw_ns', 'xarm')

    # joint state publisher node
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name='joint_state_publisher',
        output='screen',
        parameters=[{'source_list': ['{}/joint_states'.format(hw_ns_str)]}],
        remappings=[
            ('follow_joint_trajectory', '{}_traj_controller/follow_joint_trajectory'.format(xarm_type)),
            # ('tf_static', 'xarm/tf_static'),
            # ('xarm/source_list', 'source_list')
        ],
        # parameters=[{"source_list": [PathJoinSubstitution([hw_ns, 'joint_states'])]}],
        # remappings=[
        #     # ('/follow_joint_trajectory', PathJoinSubstitution([hw_ns, '{}_traj_controller'.format(xarm_type), 'follow_joint_trajectory'])),
        #     ('/follow_joint_trajectory', 'xarm/{}_traj_controller/follow_joint_trajectory'.format(xarm_type)),
        #     # ('/tf_static', PathJoinSubstitution([hw_ns, 'tf_static'])),
        #     # (PathJoinSubstitution([hw_ns, 'source_list']), '/source_list'),
        # ]
    )

    # controller_params = PathJoinSubstitution([FindPackageShare('xarm_controller'), 'config', '{}_controllers.yaml'.format(xarm_type)])
    # controller_params = get_controller_params(dof)
    # ros2 control launch
    ros2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_controller'), 'launch', '_ros2_control.launch.py'])),
        launch_arguments={
            'prefix': prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'dof': dof,
            'ros2_control_plugin': ros2_control_plugin,
            'controller_params': controller_params,
            'ros_namespace': ros_namespace,
        }.items(),
    )

    # Load controllers
    controllers = []
    # for controller in ['{}_traj_controller'.format(xarm_type)]:
    #     controllers.append(ExecuteProcess(
    #         cmd=["ros2 run controller_manager spawner.py {}".format(controller)],
    #         shell=True,
    #         output="screen",
    #     ))

    # ros_namespace = launch_arguments_dict.get('ros_namespace', '')
    control_node = Node(
        package="controller_manager",
        executable="spawner.py",
        output='screen',
        arguments=[
            '{}_traj_controller'.format(xarm_type),
            '--controller-manager', '{}/controller_manager'.format(ros_namespace)
        ],
    )
    

    return LaunchDescription([
        xarm_driver_launch,
        robot_description_launch,
        ros2_launch,
        joint_state_publisher_node,
    ] + entities + controllers + [
        control_node
    ])
