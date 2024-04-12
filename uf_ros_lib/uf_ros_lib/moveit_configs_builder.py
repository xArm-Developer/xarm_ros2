
#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2024, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 
"""Simplify loading moveit config parameters.

xarm_moveit_config/
    config/xxx/
        kinematics.yaml -> IK solver's parameters
        joint_limits.yaml -> Overriding position/velocity/acceleration limits from the URDF file
        *_planning.yaml -> planning pipelines parameters
        *_controllers.yaml -> trajectory execution manager's parameters
        ...

MoveItConfigsBuilder(
    context: launch.launch_context.LaunchContext,
    controllers_name: 'fake_controllers' or 'controllers',
    **kwargs
)
    context: instance of launch.launch_context.LaunchContext
    controllers_name: 'controllers' if realmove else 'fake_controllers' 
    **kwargs: 
        robot_type='xarm'
        dof=7
        prefix=''
        hw_ns='xarm'
        limited=False
        effort_control=False
        velocity_control=False
        add_gripper=False
        add_vacuum_gripper=False
        add_bio_gripper=False,
        ros2_control_plugin='uf_robot_hardware/UFRobotSystemHardware'
        add_realsense_d435i=False
        add_d435i_links=True
        model1300=False
        attach_to='world'
        attach_xyz='0 0 0'
        attach_rpy='0 0 0'
        add_other_geometry=False
        geometry_type='box'
        geometry_mass=0.1
        geometry_height=0.1
        geometry_radius=0.1
        geometry_length=0.1
        geometry_width=0.1
        geometry_mesh_filename=''
        geometry_mesh_origin_xyz='0 0 0'
        geometry_mesh_origin_rpy='0 0 0'
        geometry_mesh_tcp_xyz='0 0 0'
        geometry_mesh_tcp_rpy='0 0 0'
        kinematics_suffix=''

DualMoveItConfigsBuilder(
    context: launch.launch_context.LaunchContext,
    controllers_name: 'fake_controllers' or 'controllers',
    **kwargs
)
    context: instance of launch.launch_context.LaunchContext
    controllers_name: 'controllers' if realmove else 'fake_controllers' 
    **kwargs:
        prefix_1='L_'
        prefix_2='R_'
        dof_1=7
        dof_2=7
        robot_type_1='xarm'
        robot_type_2='xarm'
        hw_ns='xarm'
        limited=False
        effort_control=False
        velocity_control=False
        ros2_control_plugin='uf_robot_hardware/UFRobotSystemHardware'
        add_gripper_1=False
        add_gripper_2=False
        add_vacuum_gripper_1=False
        add_vacuum_gripper_2=False
        add_bio_gripper_1=False
        add_bio_gripper_2=False
        model1300_1=False
        model1300_2=False
        robot_sn_1=''
        robot_sn_2=''
        kinematics_suffix_1=''
        kinematics_suffix_2=''
        add_realsense_d435i_1=False
        add_realsense_d435i_2=False
        add_d435i_links_1=True
        add_d435i_links_2=True
        add_other_geometry_1=False
        add_other_geometry_2=False
        geometry_type_1='box'
        geometry_type_2='box'
        geometry_mass_1=0.1
        geometry_mass_2=0.1
        geometry_height_1=0.1
        geometry_height_2=0.1
        geometry_radius_1=0.1
        geometry_radius_2=0.1
        geometry_length_1=0.1
        geometry_length_2=0.1
        geometry_width_1=0.1
        geometry_width_2=0.1
        geometry_mesh_filename_1=''
        geometry_mesh_filename_2=''
        geometry_mesh_origin_xyz_1='0 0 0'
        geometry_mesh_origin_xyz_2='0 0 0'
        geometry_mesh_origin_rpy_1='0 0 0'
        geometry_mesh_origin_rpy_2='0 0 0'
        geometry_mesh_tcp_xyz_1='0 0 0'
        geometry_mesh_tcp_xyz_2='0 0 0'
        geometry_mesh_tcp_rpy_1='0 0 0'
        geometry_mesh_tcp_rpy_2='0 0 0'

Example:
    moveit_configs = MoveItConfigsBuilder(
        context=context,
        controllers_name='fake_controllers',
        ...).to_moveit_configs()
    ...
    moveit_configs.robot_description
    moveit_configs.robot_description_semantic
    moveit_configs.robot_description_kinematics
    moveit_configs.planning_pipelines
    moveit_configs.trajectory_execution
    moveit_configs.planning_scene_monitor
    moveit_configs.joint_limits
    moveit_configs.to_dict()
"""

import os
import re
from pathlib import Path
from dataclasses import dataclass
from ament_index_python import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder, load_yaml, load_xacro
from launch_param_builder.utils import ParameterBuilderFileNotFoundError
from uf_ros_lib.uf_robot_utils import get_xacro_command
from uf_ros_lib.parameter_descriptions import YamlParameterValue
from uf_ros_lib.substitutions.common import CommonYAML, DualCommonYAML
from uf_ros_lib.substitutions.joint_limits import JointLimitsYAML, DualJointLimitsYAML
from uf_ros_lib.substitutions.kinematics import KinematicsYAML, DualKinematicsYAML
from uf_ros_lib.substitutions.controllers import ControllersYAML, DualControllersYAML
from uf_ros_lib.substitutions.planning_pipelines import get_pattern_matches, PlanningPipelinesYAML, DualPlanningPipelinesYAML

try:
    from moveit_configs_utils import MoveItConfigs
except Exception as e:
    @dataclass
    class MoveItConfigs:
        """Class containing MoveIt related parameters."""

        # A pathlib Path to the moveit config package
        package_path = None
        # A dictionary that has the contents of the URDF file.
        robot_description = {}
        # A dictionary that has the contents of the SRDF file.
        robot_description_semantic = {}
        # A dictionary IK solver specific parameters.
        robot_description_kinematics = {}
        # A dictionary that contains the planning pipelines parameters.
        planning_pipelines = {}
        # A dictionary contains parameters for trajectory execution & moveit controller managers.
        trajectory_execution = {}
        # A dictionary that has the planning scene monitor's parameters.
        planning_scene_monitor = {}
        # A dictionary that has the sensor 3d configuration parameters.
        sensors_3d = {}
        # A dictionary containing move_group's non-default capabilities.
        move_group_capabilities = {}
        # A dictionary containing the overridden position/velocity/acceleration limits.
        joint_limits = {}
        # A dictionary containing MoveItCpp related parameters.
        moveit_cpp = {}
        # A dictionary containing the cartesian limits for the Pilz planner.
        pilz_cartesian_limits = {}

        def to_dict(self):
            parameters = {}
            parameters.update(self.robot_description)
            parameters.update(self.robot_description_semantic)
            parameters.update(self.robot_description_kinematics)
            parameters.update(self.planning_pipelines)
            parameters.update(self.trajectory_execution)
            parameters.update(self.planning_scene_monitor)
            parameters.update(self.sensors_3d)
            parameters.update(self.joint_limits)
            parameters.update(self.moveit_cpp)
            # Update robot_description_planning with pilz cartesian limits
            if self.pilz_cartesian_limits:
                parameters["robot_description_planning"].update(
                    self.pilz_cartesian_limits["robot_description_planning"]
                )
            return parameters


class MoveItConfigsBuilder(ParameterBuilder):
    __moveit_configs = None
    __urdf_package = ''
    # Relative path of the URDF file w.r.t. __urdf_package
    __urdf_file_path = ''
    # Relative path of the SRDF file  w.r.t. xarm_moveit_config
    __srdf_file_path = ''
    # String specify the parameter name that the robot description will be loaded to, it will also be used as a prefix
    # for "_planning", "_semantic", and "_kinematics"
    __robot_description = ''

    def __init__(
        self,
        context=None,
        controllers_name='fake_controllers',
        **kwargs
    ):
        super().__init__('xarm_moveit_config')
        self.__moveit_configs = MoveItConfigs()

        self.__context = context

        def get_param_str(name, default_val):
            val = kwargs.get(name, default_val)
            return val if isinstance(val, str) else 'false' if val == False else 'true' if val == True else (val.perform(context) if context is not None else val) if isinstance(val, LaunchConfiguration) else str(val)

        def get_list_param_str(name, default_val):
            val = get_param_str(name, default_val)
            return val[1:-1] if context is not None and isinstance(val, str) and val[0] in ['"', '\''] and val[-1] in ['"', '\''] else val

        prefix = get_param_str('prefix', '')
        hw_ns = get_param_str('hw_ns', 'xarm')
        limited = get_param_str('limited', False)
        effort_control = get_param_str('effort_control', False)
        velocity_control = get_param_str('velocity_control', False)
        ros2_control_plugin = get_param_str('ros2_control_plugin', 'uf_robot_hardware/UFRobotSystemHardware')
        ros2_control_params = get_param_str('ros2_control_params', '')
        mesh_suffix = get_param_str('mesh_suffix', 'stl')

        attach_to = get_param_str('attach_to', 'world')
        attach_xyz = get_list_param_str('attach_xyz', '0 0 0')
        attach_rpy = get_list_param_str('attach_rpy', '0 0 0')

        add_gripper = get_param_str('add_gripper', False)
        add_vacuum_gripper = get_param_str('add_vacuum_gripper', False)
        add_bio_gripper = get_param_str('add_bio_gripper', False)
        model1300 = get_param_str('model1300', False)
        dof = get_param_str('dof', 7)
        robot_ip = get_param_str('robot_ip', '')
        robot_type = get_param_str('robot_type', 'xarm')
        robot_sn = get_param_str('robot_sn', '')
        report_type = get_param_str('report_type', 'normal')
        baud_checkset = get_param_str('baud_checkset', True)
        default_gripper_baud = get_param_str('default_gripper_baud', 2000000)
        kinematics_suffix = get_param_str('kinematics_suffix', '')
        add_realsense_d435i = get_param_str('add_realsense_d435i', False)
        add_d435i_links = get_param_str('add_d435i_links', True)
        use_gazebo_camera = get_param_str('use_gazebo_camera', False)
        add_other_geometry = get_param_str('add_other_geometry', False)
        geometry_type = get_param_str('geometry_type', 'box')
        geometry_mass = get_param_str('geometry_mass', 0.1)
        geometry_height = get_param_str('geometry_height', 0.1)
        geometry_radius = get_param_str('geometry_radius', 0.1)
        geometry_length = get_param_str('geometry_length', 0.1)
        geometry_width = get_param_str('geometry_width', 0.1)
        geometry_mesh_filename = get_param_str('geometry_mesh_filename', '')
        geometry_mesh_origin_xyz = get_list_param_str('geometry_mesh_origin_xyz', '0 0 0')
        geometry_mesh_origin_rpy = get_list_param_str('geometry_mesh_origin_rpy', '0 0 0')
        geometry_mesh_tcp_xyz = get_list_param_str('geometry_mesh_tcp_xyz', '0 0 0')
        geometry_mesh_tcp_rpy = get_list_param_str('geometry_mesh_tcp_rpy', '0 0 0')

        self.__prefix = prefix
        self.__robot_dof = dof
        self.__robot_type = robot_type
        self.__add_gripper = add_gripper
        self.__add_bio_gripper = add_bio_gripper
        self.__controllers_name = (controllers_name.perform(context) if context is not None else controllers_name) if isinstance(controllers_name, LaunchConfiguration) else controllers_name

        self.__urdf_xacro_args = {
            'prefix': prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'ros2_control_plugin': ros2_control_plugin,
            'ros2_control_params': ros2_control_params,
            'mesh_suffix': mesh_suffix,

            'attach_to': attach_to,
            'attach_xyz': attach_xyz,
            'attach_rpy': attach_rpy,

            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'add_bio_gripper': add_bio_gripper,
            'model1300': model1300,
            'dof': dof,
            'robot_ip': robot_ip,
            'robot_type': robot_type,
            'robot_sn': robot_sn,
            'report_type': report_type,
            'baud_checkset': baud_checkset,
            'default_gripper_baud': default_gripper_baud,
            'kinematics_suffix': kinematics_suffix,
            'add_realsense_d435i': add_realsense_d435i,
            'add_d435i_links': add_d435i_links,
            'use_gazebo_camera': use_gazebo_camera,
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
        self.__srdf_xacro_args = {
            'prefix': prefix,
            'dof': dof,
            'robot_type': robot_type,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'add_bio_gripper': add_bio_gripper,
            'add_other_geometry': add_other_geometry,
        }

        self.__urdf_package = Path(get_package_share_directory('xarm_description'))
        self.__urdf_file_path = Path('urdf/xarm_device.urdf.xacro')
        self.__srdf_file_path = Path('srdf/xarm.srdf.xacro')

        self.__robot_description = 'robot_description'

    def robot_description(
        self,
        file_path = None,
        mappings = None,
    ):
        """Load robot description.

        :param file_path: Absolute or relative path to the URDF file (w.r.t. xarm_moveit_config).
        :param mappings: mappings to be passed when loading the xacro file.
        :return: Instance of MoveItConfigsBuilder with robot_description loaded.
        """
        if file_path is None:
            robot_description_file_path = self.__urdf_package / self.__urdf_file_path
        else:
            robot_description_file_path = self._package_path / file_path
        mappings = mappings or self.__urdf_xacro_args
        
        if (mappings is None) or all(
            (isinstance(key, str) and isinstance(value, str))
            for key, value in mappings.items()
        ):
            try:
                self.__moveit_configs.robot_description = {
                    self.__robot_description: load_xacro(
                        robot_description_file_path,
                        mappings=mappings,
                    )
                }
            except ParameterBuilderFileNotFoundError as e:
                logging.warning('\x1b[33;21m{}\x1b[0m'.format(e))
                logging.warning('\x1b[33;21mThe robot description will be loaded from /robot_description topic \x1b[0m')
        else:
            self.__moveit_configs.robot_description = {
                self.__robot_description: get_xacro_command(
                    str(robot_description_file_path), mappings=mappings
                )
            }

        return self

    def robot_description_semantic(
        self,
        file_path = None,
        mappings = None,
    ):
        """Load semantic robot description.

        :param file_path: Absolute or relative path to the SRDF file (w.r.t. xarm_moveit_config).
        :param mappings: mappings to be passed when loading the xacro file.
        :return: Instance of MoveItConfigsBuilder with robot_description_semantic loaded.
        """
        key = self.__robot_description + '_semantic'
        file_path = self._package_path / (file_path or self.__srdf_file_path)
        mappings = mappings or self.__srdf_xacro_args
        
        if (mappings is None) or all(
            (isinstance(key, str) and isinstance(value, str))
            for key, value in mappings.items()
        ):
            self.__moveit_configs.robot_description_semantic = {
                key: load_xacro(file_path, mappings=mappings)
            }
        else:
            self.__moveit_configs.robot_description_semantic = {
                key: get_xacro_command(str(file_path), mappings=mappings)
            }
        
        return self

    def robot_description_kinematics(self, file_path = None):
        """Load IK solver parameters.

        :param file_path: Absolute or relative path to the kinematics yaml file (w.r.t. xarm_moveit_config).
        :return: Instance of MoveItConfigsBuilder with robot_description_kinematics loaded.
        """
        key = self.__robot_description + '_kinematics'

        params = [self.__prefix, self.__robot_type, self.__robot_dof]
        if all(isinstance(value, str) for value in params):
            robot_name = '{}{}'.format(self.__robot_type, self.__robot_dof if self.__robot_type == 'xarm' else '6' if self.__robot_type == 'lite' else '')
            
            if file_path is None:
                file_path = self._package_path / 'config' / robot_name / 'kinematics.yaml'
                kinematics_yaml = load_yaml(file_path)
            else:
                file_path = self._package_path / file_path
                kinematics_yaml = load_yaml(file_path) if file_path else {}
            if kinematics_yaml and self.__prefix:
                for name in list(kinematics_yaml.keys()):
                    kinematics_yaml['{}{}'.format(self.__prefix, name)] = kinematics_yaml.pop(name)
            self.__moveit_configs.robot_description_kinematics = {
                key: kinematics_yaml
            }
        else:
            self.__moveit_configs.robot_description_kinematics = {
                key: YamlParameterValue(
                    KinematicsYAML(file_path, package_path=self._package_path, 
                        prefix=self.__prefix, robot_type=self.__robot_type, robot_dof=self.__robot_dof
                    ), value_type=str)
            }

        return self

    def joint_limits(self, file_path = None):
        """Load joint limits overrides.

        :param file_path: Absolute or relative path to the joint limits yaml file (w.r.t. xarm_moveit_config).
        :return: Instance of MoveItConfigsBuilder with robot_description_planning loaded.
        """
        key = self.__robot_description + '_planning'

        params = [self.__prefix, self.__robot_type, self.__robot_dof, self.__add_gripper, self.__add_bio_gripper]
        if all(isinstance(value, str) for value in params):
            robot_name = '{}{}'.format(self.__robot_type, self.__robot_dof if self.__robot_type == 'xarm' else '6' if self.__robot_type == 'lite' else '')
            
            if file_path is None:
                file_path = self._package_path / 'config' / robot_name / 'joint_limits.yaml'
                joint_limits = load_yaml(file_path)
            else:
                file_path = self._package_path / file_path
                joint_limits = load_yaml(file_path) if file_path else {}
            if self.__add_gripper in ('True', 'true'):
                gripper_joint_limits_yaml = load_yaml(self._package_path / 'config' / '{}_gripper'.format(self.__robot_type) / 'joint_limits.yaml')
                if gripper_joint_limits_yaml and 'joint_limits' in gripper_joint_limits_yaml:
                    joint_limits['joint_limits'].update(gripper_joint_limits_yaml['joint_limits'])
            elif self.__add_bio_gripper in ('True', 'true'):
                gripper_joint_limits_yaml = load_yaml(self._package_path / 'config' / 'bio_gripper' / 'joint_limits.yaml')
                if gripper_joint_limits_yaml and 'joint_limits' in gripper_joint_limits_yaml:
                    joint_limits['joint_limits'].update(gripper_joint_limits_yaml['joint_limits'])
            if joint_limits and self.__prefix:
                for name in list(joint_limits['joint_limits']):
                    joint_limits['joint_limits']['{}{}'.format(self.__prefix, name)] = joint_limits['joint_limits'].pop(name)
            self.__moveit_configs.joint_limits = {
                key: joint_limits
            }
        else:
            self.__moveit_configs.joint_limits = {
                key: YamlParameterValue(
                    JointLimitsYAML(file_path, package_path=self._package_path, 
                        prefix=self.__prefix, robot_type=self.__robot_type, robot_dof=self.__robot_dof,
                        add_gripper=self.__add_gripper, add_bio_gripper=self.__add_bio_gripper
                ), value_type=str)
            }

        return self

    def moveit_cpp(self, file_path = None):
        """Load MoveItCpp parameters.

        :param file_path: Absolute or relative path to the MoveItCpp yaml file (w.r.t. xarm_moveit_config).
        :return: Instance of MoveItConfigsBuilder with moveit_cpp loaded.
        """
        params = [self.__robot_type, self.__robot_dof]
        if all(isinstance(value, str) for value in params):
            robot_name = '{}{}'.format(self.__robot_type, self.__robot_dof if self.__robot_type == 'xarm' else '6' if self.__robot_type == 'lite' else '')
            
            if file_path is None:
                file_path = self._package_path / 'config' / robot_name / 'moveit_cpp.yaml'
                moveit_cpp = load_yaml(file_path)
            else:
                file_path = self._package_path / file_path
                moveit_cpp = load_yaml(file_path) if file_path else {}
            self.__moveit_configs.moveit_cpp = moveit_cpp

        return self

    def trajectory_execution(
        self,
        file_path = None,
        controllers_name = None,
        moveit_manage_controllers = False,
    ):
        """Load trajectory execution and moveit controller managers' parameters

        :param file_path: Absolute or relative path to the controllers yaml file (w.r.t. xarm_moveit_config).
        :param moveit_manage_controllers: Whether trajectory execution manager is allowed to switch controllers' states.
        :return: Instance of MoveItConfigsBuilder with trajectory_execution loaded.
        """
        controllers_name = controllers_name if controllers_name else self.__controllers_name

        params = [self.__prefix, self.__robot_type, self.__robot_dof, self.__add_gripper, self.__add_bio_gripper, controllers_name]
        if all(isinstance(value, str) for value in params):
            robot_name = '{}{}'.format(self.__robot_type, self.__robot_dof if self.__robot_type == 'xarm' else '6' if self.__robot_type == 'lite' else '')
            controllers_name = controllers_name if controllers_name.endswith('.yaml') else '{}.yaml'.format(controllers_name)
            if file_path is None:
                file_path = self._package_path / 'config' / robot_name / controllers_name
                controllers_yaml = load_yaml(file_path)
                if self.__add_gripper in ('True', 'true'):
                    gripper_controllers_yaml = load_yaml(self._package_path / 'config' / '{}_gripper'.format(self.__robot_type) / controllers_name)
                    if gripper_controllers_yaml and 'controller_names' in gripper_controllers_yaml:
                        for name in gripper_controllers_yaml['controller_names']:
                            if name in gripper_controllers_yaml:
                                if name not in controllers_yaml['controller_names']:
                                    controllers_yaml['controller_names'].append(name)
                                controllers_yaml[name] = gripper_controllers_yaml[name]
                elif self.__add_bio_gripper in ('True', 'true'):
                    gripper_controllers_yaml = load_yaml(self._package_path / 'config' / 'bio_gripper' / controllers_name)
                    if gripper_controllers_yaml and 'controller_names' in gripper_controllers_yaml:
                        for name in gripper_controllers_yaml['controller_names']:
                            if name in gripper_controllers_yaml:
                                if name not in controllers_yaml['controller_names']:
                                    controllers_yaml['controller_names'].append(name)
                                controllers_yaml[name] = gripper_controllers_yaml[name]
            else:
                file_path = self._package_path / file_path
                controllers_yaml = load_yaml(file_path) if file_path else {}

            if controllers_yaml and self.__prefix:
                for i, name in enumerate(controllers_yaml['controller_names']):
                    joints = controllers_yaml.get(name, {}).get('joints', [])
                    for j, joint in enumerate(joints):
                        joints[j] = '{}{}'.format(self.__prefix, joint)
                    controllers_yaml['controller_names'][i] = '{}{}'.format(self.__prefix, name)
                    if name in controllers_yaml:
                        controllers_yaml['{}{}'.format(self.__prefix, name)] = controllers_yaml.pop(name)

            self.__moveit_configs.trajectory_execution = {
                'moveit_manage_controllers': moveit_manage_controllers,
                'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
                'moveit_simple_controller_manager': controllers_yaml,
                'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                'trajectory_execution.allowed_goal_duration_margin': 0.5,
                'trajectory_execution.allowed_start_tolerance': 0.01,
                'trajectory_execution.execution_duration_monitoring': False
            }
        else:
            self.__moveit_configs.trajectory_execution = {
                'moveit_manage_controllers': moveit_manage_controllers,
                'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
                'moveit_simple_controller_manager': YamlParameterValue(
                    ControllersYAML(file_path, package_path=self._package_path, 
                        prefix=self.__prefix, robot_type=self.__robot_type, robot_dof=self.__robot_dof,
                        add_gripper=self.__add_gripper, add_bio_gripper=self.__add_bio_gripper, controllers_name=controllers_name
                    ), value_type=str),
                'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                'trajectory_execution.allowed_goal_duration_margin': 0.5,
                'trajectory_execution.allowed_start_tolerance': 0.01,
                'trajectory_execution.execution_duration_monitoring': False
            }
        return self

    def planning_scene_monitor(
        self,
        publish_planning_scene = True,
        publish_geometry_updates = True,
        publish_state_updates = True,
        publish_transforms_updates = True,
        publish_robot_description = False,
        publish_robot_description_semantic = False,
    ):
        self.__moveit_configs.planning_scene_monitor = {
            # TODO: Fix parameter namespace upstream -- see planning_scene_monitor.cpp:262
            # 'planning_scene_monitor': {
            'publish_planning_scene': publish_planning_scene,
            'publish_geometry_updates': publish_geometry_updates,
            'publish_state_updates': publish_state_updates,
            'publish_transforms_updates': publish_transforms_updates,
            'publish_robot_description': publish_robot_description,
            'publish_robot_description_semantic': publish_robot_description_semantic,
            # }
        }
        return self

    def sensors_3d(self, file_path = None):
        """Load sensors_3d parameters.

        :param file_path: Absolute or relative path to the sensors_3d yaml file (w.r.t. xarm_moveit_config).
        :return: Instance of MoveItConfigsBuilder with robot_description_planning loaded.
        """
        params = [self.__robot_type, self.__robot_dof]
        if all(isinstance(value, str) for value in params):
            robot_name = '{}{}'.format(self.__robot_type, self.__robot_dof if self.__robot_type == 'xarm' else '6' if self.__robot_type == 'lite' else '')
            
            if file_path is None:
                file_path = self._package_path / 'config' / robot_name / 'sensors_3d.yaml'
            else:
                file_path = self._package_path / file_path
            if file_path and file_path.exists():
                sensors_data = load_yaml(file_path)
                # TODO(mikeferguson): remove the second part of this check once
                # https://github.com/ros-planning/moveit_resources/pull/141 has made through buildfarm
                if sensors_data and len(sensors_data['sensors']) > 0 and sensors_data['sensors'][0]:
                    self.__moveit_configs.sensors_3d = sensors_data
        
        return self

    def planning_pipelines(
        self,
        default_planning_pipeline = None,
        pipelines = None,
        load_all = True,
    ):
        """Load planning pipelines parameters.

        :param default_planning_pipeline: Name of the default planning pipeline.
        :param pipelines: List of the planning pipelines to be loaded.
        :param load_all: Only used if pipelines is None.
                         If true, loads all pipelines defined in config package AND this package.
                         If false, only loads the pipelines defined in config package.
        :return: Instance of MoveItConfigsBuilder with planning_pipelines loaded.
        """
        params = [self.__prefix, self.__robot_type, self.__robot_dof, self.__add_gripper, self.__add_bio_gripper]
        if all(isinstance(value, str) for value in params):
            robot_name = '{}{}'.format(self.__robot_type, self.__robot_dof if self.__robot_type == 'xarm' else '6' if self.__robot_type == 'lite' else '')
            config_folder = self._package_path / 'config' / robot_name
            if pipelines is None:
                planning_pattern = re.compile('^(.*)_planning.yaml$')
                pipelines = get_pattern_matches(config_folder, planning_pattern)
            pipelines = list(set(pipelines))
             # Define default pipeline as needed
            if not default_planning_pipeline:
                if not pipelines or 'ompl' in pipelines:
                    default_planning_pipeline = 'ompl'
                else:
                    default_planning_pipeline = pipelines[0]

            if default_planning_pipeline not in pipelines:
                raise RuntimeError(
                    'default_planning_pipeline: `{}` doesn\'t name any of the input pipelines `{}`'.format(default_planning_pipeline, ','.join(pipelines))
                )

            self.__moveit_configs.planning_pipelines = {
                'planning_pipelines': pipelines,
                'default_planning_pipeline': default_planning_pipeline,
            }
            
            for pipeline in pipelines:
                filename = pipeline + '_planning.yaml'
                parameter_file = config_folder / filename
                
                planning_yaml = load_yaml(parameter_file)
                
                if self.__add_gripper in ('True', 'true'):
                    parameter_file = self._package_path / 'config' / '{}_gripper'.format(self.__robot_type) / filename
                    if parameter_file.exists():
                        gripper_planning_yaml = load_yaml(parameter_file)
                        if gripper_planning_yaml:
                            planning_yaml.update(gripper_planning_yaml)
                elif self.__add_bio_gripper in ('True', 'true'):
                    parameter_file = self._package_path / 'config' / 'bio_gripper' / filename
                    if parameter_file.exists():
                        gripper_planning_yaml = load_yaml(parameter_file)
                        if gripper_planning_yaml:
                            planning_yaml.update(gripper_planning_yaml)
                if planning_yaml and self.__prefix:
                    for name in list(planning_yaml.keys()):
                        if pipeline == 'ompl' and name != 'planner_configs':
                            planning_yaml['{}{}'.format(self.__prefix, name)] = planning_yaml.pop(name)
                self.__moveit_configs.planning_pipelines[pipeline] = planning_yaml
            # Special rule to add ompl planner_configs
            if 'ompl' in self.__moveit_configs.planning_pipelines:
                ompl_config = self.__moveit_configs.planning_pipelines['ompl']
                if os.environ.get('ROS_DISTRO', '') > 'iron':
                    ompl_config.update({
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
                    })
                else:
                    ompl_config.update({
                        'planning_plugin': 'ompl_interface/OMPLPlanner',
                        'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
                        'start_state_max_bounds_error': 0.1,
                    })
        else:
            pipelines = list(set(pipelines)) if pipelines else ['ompl']
            default_planning_pipeline = default_planning_pipeline if default_planning_pipeline else 'ompl'
            if default_planning_pipeline not in pipelines:
                raise RuntimeError(
                    'default_planning_pipeline: `{}` doesn\'t name any of the input pipelines `{}`'.format(default_planning_pipeline, ','.join(pipelines))
                )
            self.__moveit_configs.planning_pipelines = {
                'planning_pipelines': pipelines,
                'default_planning_pipeline': default_planning_pipeline,
            }
            for pipeline in pipelines:
                self.__moveit_configs.planning_pipelines[pipeline] = YamlParameterValue(
                    PlanningPipelinesYAML(pipeline, package_path=self._package_path, 
                        prefix=self.__prefix, robot_type=self.__robot_type, robot_dof=self.__robot_dof,
                        add_gripper=self.__add_gripper, add_bio_gripper=self.__add_bio_gripper
                    ), value_type=str)
        
        return self

    def pilz_cartesian_limits(self, file_path = None):
        """Load cartesian limits.

        :param file_path: Absolute or relative path to the cartesian limits file (w.r.t. xarm_moveit_config).
        :return: Instance of MoveItConfigsBuilder with pilz_cartesian_limits loaded.
        """
        params = [self.__robot_type, self.__robot_dof]
        if all(isinstance(value, str) for value in params):
            robot_name = '{}{}'.format(self.__robot_type, self.__robot_dof if self.__robot_type == 'xarm' else '6' if self.__robot_type == 'lite' else '')
            deprecated_path = self._package_path / 'config' / robot_name / 'cartesian_limits.yaml'
            if deprecated_path.exists():
                logging.warning('\x1b[33;21mcartesian_limits.yaml is deprecated, please rename to pilz_cartesian_limits.yaml\x1b[0m')
            if file_path is None:
                file_path = self._package_path / 'config' / robot_name / 'pilz_cartesian_limits.yaml'
            else:
                file_path = self._package_path / file_path
            key = self.__robot_description + '_planning'
            if file_path.exists():
                self.__moveit_configs.pilz_cartesian_limits = {
                    key: load_yaml(file_path)
                }
        else:
            key = self.__robot_description + '_planning'
            self.__moveit_configs.pilz_cartesian_limits = {
                key: YamlParameterValue(
                    CommonYAML('pilz_cartesian_limits.yaml', package_path=self._package_path, 
                        robot_type=self.__robot_type, robot_dof=self.__robot_dof,
                ), value_type=str)
            }

        return self

    def to_moveit_configs(self):
        """Get MoveIt configs from xarm_moveit_config.

        :return: An MoveItConfigs instance with all parameters loaded.
        """
        if not self.__moveit_configs.robot_description:
            self.robot_description()
        if not self.__moveit_configs.robot_description_semantic:
            self.robot_description_semantic()
        if not self.__moveit_configs.robot_description_kinematics:
            self.robot_description_kinematics()
        if not self.__moveit_configs.planning_pipelines:
            self.planning_pipelines()
        if not self.__moveit_configs.trajectory_execution:
            self.trajectory_execution()
        if not self.__moveit_configs.planning_scene_monitor:
            self.planning_scene_monitor()
        if not self.__moveit_configs.sensors_3d:
            self.sensors_3d()
        if not self.__moveit_configs.joint_limits:
            self.joint_limits()
        # TODO(JafarAbdi): We should have a default moveit_cpp.yaml as port of a moveit config package
        # if not self.__moveit_configs.moveit_cpp:
        #     self.moveit_cpp()
        if 'pilz_industrial_motion_planner' in self.__moveit_configs.planning_pipelines:
            if not self.__moveit_configs.pilz_cartesian_limits:
                self.pilz_cartesian_limits()
        return self.__moveit_configs

    def to_dict(self, include_moveit_configs = True):
        """Get loaded parameters from xarm_moveit_config as a dictionary.

        :param include_moveit_configs: Whether to include the MoveIt config parameters or
                                       only the ones from ParameterBuilder
        :return: Dictionary with all parameters loaded.
        """
        parameters = self._parameters
        if include_moveit_configs:
            parameters.update(self.to_moveit_configs().to_dict())
        return parameters


class DualMoveItConfigsBuilder(ParameterBuilder):
    __moveit_configs = None
    __urdf_package = ''
    # Relative path of the URDF file w.r.t. __urdf_package
    __urdf_file_path = ''
    # Relative path of the SRDF file  w.r.t. xarm_moveit_config
    __srdf_file_path = ''
    # String specify the parameter name that the robot description will be loaded to, it will also be used as a prefix
    # for "_planning", "_semantic", and "_kinematics"
    __robot_description = ''

    def __init__(
        self,
        context=None,
        controllers_name='fake_controllers',
        **kwargs
    ):
        super().__init__('xarm_moveit_config')
        self.__moveit_configs = MoveItConfigs()

        self.__context = context

        def get_param_str(name, default_val):
            val = kwargs.get(name, default_val)
            return val if isinstance(val, str) else 'false' if val == False else 'true' if val == True else (val.perform(context) if context is not None else val) if isinstance(val, LaunchConfiguration) else str(val)
        
        def get_list_param_str(name, default_val):
            val = get_param_str(name, default_val)
            return val[1:-1] if context is not None and isinstance(val, str) and val[0] in ['"', '\''] and val[-1] in ['"', '\''] else val

        hw_ns = get_param_str('hw_ns', 'xarm')
        limited = get_param_str('limited', False)
        effort_control = get_param_str('effort_control', False)
        velocity_control = get_param_str('velocity_control', False)
        ros2_control_plugin = get_param_str('ros2_control_plugin', 'uf_robot_hardware/UFRobotSystemHardware')
        ros2_control_params = get_param_str('ros2_control_params', '')
        mesh_suffix = get_param_str('mesh_suffix', 'stl')

        prefix_1 = get_param_str('prefix_1', 'L_')
        prefix_2 = get_param_str('prefix_2', 'R_')
        add_gripper = get_param_str('add_gripper', False)
        add_gripper_1 = get_param_str('add_gripper_1', add_gripper)
        add_gripper_2 = get_param_str('add_gripper_2', add_gripper)
        add_vacuum_gripper = get_param_str('add_vacuum_gripper', False)
        add_vacuum_gripper_1 = get_param_str('add_vacuum_gripper_1', add_vacuum_gripper)
        add_vacuum_gripper_2 = get_param_str('add_vacuum_gripper_2', add_vacuum_gripper)
        add_bio_gripper = get_param_str('add_bio_gripper', False)
        add_bio_gripper_1 = get_param_str('add_bio_gripper_1', add_bio_gripper)
        add_bio_gripper_2 = get_param_str('add_bio_gripper_2', add_bio_gripper)
        model1300 = get_param_str('model1300', False)
        model1300_1 = get_param_str('model1300_1', model1300)
        model1300_2 = get_param_str('model1300_2', model1300)
        dof = get_param_str('dof', 7)
        dof_1 = get_param_str('dof_1', dof)
        dof_2 = get_param_str('dof_2', dof)
        robot_ip_1 = get_param_str('robot_ip_1', '')
        robot_ip_2 = get_param_str('robot_ip_2', '')
        robot_type = get_param_str('robot_type', 'xarm')
        robot_type_1 = get_param_str('robot_type_1', robot_type)
        robot_type_2 = get_param_str('robot_type_2', robot_type)
        robot_sn = get_param_str('robot_sn', '')
        robot_sn_1 = get_param_str('robot_sn_1', robot_sn)
        robot_sn_2 = get_param_str('robot_sn_2', robot_sn)
        report_type = get_param_str('report_type', 'normal')
        report_type_1 = get_param_str('report_type_1', report_type)
        report_type_2 = get_param_str('report_type_2', report_type)
        baud_checkset = get_param_str('baud_checkset', True)
        baud_checkset_1 = get_param_str('baud_checkset_1', baud_checkset)
        baud_checkset_2 = get_param_str('baud_checkset_2', baud_checkset)
        default_gripper_baud = get_param_str('default_gripper_baud', 2000000)
        default_gripper_baud_1 = get_param_str('default_gripper_baud_1', default_gripper_baud)
        default_gripper_baud_2 = get_param_str('default_gripper_baud_2', default_gripper_baud)
        kinematics_suffix = get_param_str('kinematics_suffix', '')
        kinematics_suffix_1 = get_param_str('kinematics_suffix_1', kinematics_suffix)
        kinematics_suffix_2 = get_param_str('kinematics_suffix_2', kinematics_suffix)
        add_realsense_d435i = get_param_str('add_realsense_d435i', False)
        add_realsense_d435i_1 = get_param_str('add_realsense_d435i_1', add_realsense_d435i)
        add_realsense_d435i_2 = get_param_str('add_realsense_d435i_2', add_realsense_d435i)
        add_d435i_links = get_param_str('add_d435i_links', True)
        add_d435i_links_1 = get_param_str('add_d435i_links_1', add_d435i_links)
        add_d435i_links_2 = get_param_str('add_d435i_links_2', add_d435i_links)
        add_other_geometry = get_param_str('add_other_geometry', False)
        add_other_geometry_1 = get_param_str('add_other_geometry_1', add_other_geometry)
        add_other_geometry_2 = get_param_str('add_other_geometry_2', add_other_geometry)
        geometry_type = get_param_str('geometry_type', 'box')
        geometry_type_1 = get_param_str('geometry_type_1', geometry_type)
        geometry_type_2 = get_param_str('geometry_type_2', geometry_type)
        geometry_mass = get_param_str('geometry_mass', 0.1)
        geometry_mass_1 = get_param_str('geometry_mass_1', geometry_mass)
        geometry_mass_2 = get_param_str('geometry_mass_2', geometry_mass)
        geometry_height = get_param_str('geometry_height', 0.1)
        geometry_height_1 = get_param_str('geometry_height_1', geometry_height)
        geometry_height_2 = get_param_str('geometry_height_2', geometry_height)
        geometry_radius = get_param_str('geometry_radius', 0.1)
        geometry_radius_1 = get_param_str('geometry_radius_1', geometry_radius)
        geometry_radius_2 = get_param_str('geometry_radius_2', geometry_radius)
        geometry_length = get_param_str('geometry_length', 0.1)
        geometry_length_1 = get_param_str('geometry_length_1', geometry_length)
        geometry_length_2 = get_param_str('geometry_length_2', geometry_length)
        geometry_width = get_param_str('geometry_width', 0.1)
        geometry_width_1 = get_param_str('geometry_width_1', geometry_width)
        geometry_width_2 = get_param_str('geometry_width_2', geometry_width)
        geometry_mesh_filename = get_param_str('geometry_mesh_filename', '')
        geometry_mesh_filename_1 = get_param_str('geometry_mesh_filename_1', geometry_mesh_filename)
        geometry_mesh_filename_2 = get_param_str('geometry_mesh_filename_2', geometry_mesh_filename)
        geometry_mesh_origin_xyz = get_list_param_str('geometry_mesh_origin_xyz', '0 0 0')
        geometry_mesh_origin_xyz_1 = get_list_param_str('geometry_mesh_origin_xyz_1', geometry_mesh_origin_xyz)
        geometry_mesh_origin_xyz_2 = get_list_param_str('geometry_mesh_origin_xyz_2', geometry_mesh_origin_xyz)
        geometry_mesh_origin_rpy = get_list_param_str('geometry_mesh_origin_rpy', '0 0 0')
        geometry_mesh_origin_rpy_1 = get_list_param_str('geometry_mesh_origin_rpy_1', geometry_mesh_origin_rpy)
        geometry_mesh_origin_rpy_2 = get_list_param_str('geometry_mesh_origin_rpy_2', geometry_mesh_origin_rpy)
        geometry_mesh_tcp_xyz = get_list_param_str('geometry_mesh_tcp_xyz', '0 0 0')
        geometry_mesh_tcp_xyz_1 = get_list_param_str('geometry_mesh_tcp_xyz_1', geometry_mesh_tcp_xyz)
        geometry_mesh_tcp_xyz_2 = get_list_param_str('geometry_mesh_tcp_xyz_2', geometry_mesh_tcp_xyz)
        geometry_mesh_tcp_rpy = get_list_param_str('geometry_mesh_tcp_rpy', '0 0 0')
        geometry_mesh_tcp_rpy_1 = get_list_param_str('geometry_mesh_tcp_rpy_1', geometry_mesh_tcp_rpy)
        geometry_mesh_tcp_rpy_2 = get_list_param_str('geometry_mesh_tcp_rpy_2', geometry_mesh_tcp_rpy)
        
        self.__prefix_1 = prefix_1
        self.__prefix_2 = prefix_2
        self.__robot_dof_1 = dof_1
        self.__robot_dof_2 = dof_2
        self.__robot_type_1 = robot_type_1
        self.__robot_type_2 = robot_type_2
        self.__add_gripper_1 = add_gripper_1
        self.__add_gripper_2 = add_gripper_2
        self.__add_bio_gripper_1 = add_bio_gripper_1
        self.__add_bio_gripper_2 = add_bio_gripper_2
        self.__controllers_name = (controllers_name.perform(context) if context is not None else controllers_name) if isinstance(controllers_name, LaunchConfiguration) else controllers_name
        
        self.__urdf_xacro_args = {
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'ros2_control_plugin': ros2_control_plugin,
            'ros2_control_params': ros2_control_params,
            'mesh_suffix': mesh_suffix,
            'prefix_1': prefix_1,
            'prefix_2': prefix_2,
            'add_gripper_1': add_gripper_1,
            'add_gripper_2': add_gripper_2,
            'add_vacuum_gripper_1': add_vacuum_gripper_1,
            'add_vacuum_gripper_2': add_vacuum_gripper_2,
            'add_bio_gripper_1': add_bio_gripper_1,
            'add_bio_gripper_2': add_bio_gripper_2,
            'model1300_1': model1300_1,
            'model1300_2': model1300_2,
            'dof_1': dof_1,
            'dof_2': dof_2,
            'robot_ip_1': robot_ip_1,
            'robot_ip_2': robot_ip_2,
            'robot_type_1': robot_type_1,
            'robot_type_2': robot_type_2,
            'robot_sn_1': robot_sn_1,
            'robot_sn_2': robot_sn_2,
            'report_type_1': report_type_1,
            'report_type_2': report_type_2,
            'baud_checkset_1': baud_checkset_1,
            'baud_checkset_2': baud_checkset_2,
            'default_gripper_baud_1': default_gripper_baud_1,
            'default_gripper_baud_2': default_gripper_baud_2,
            'kinematics_suffix_1': kinematics_suffix_1,
            'kinematics_suffix_2': kinematics_suffix_2,
            'add_realsense_d435i_1': add_realsense_d435i_1,
            'add_realsense_d435i_2': add_realsense_d435i_2,
            'add_d435i_links_1': add_d435i_links_1,
            'add_d435i_links_2': add_d435i_links_2,
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
        self.__srdf_xacro_args = {
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
        }

        self.__urdf_package = Path(get_package_share_directory('xarm_description'))
        self.__urdf_file_path = Path('urdf/dual_xarm_device.urdf.xacro')
        self.__srdf_file_path = Path('srdf/dual_xarm.srdf.xacro')

        self.__robot_description = 'robot_description'

    def robot_description(
        self,
        file_path = None,
        mappings = None,
    ):
        """Load robot description.

        :param file_path: Absolute or relative path to the URDF file (w.r.t. xarm_moveit_config).
        :param mappings: mappings to be passed when loading the xacro file.
        :return: Instance of MoveItConfigsBuilder with robot_description loaded.
        """
        if file_path is None:
            robot_description_file_path = self.__urdf_package / self.__urdf_file_path
        else:
            robot_description_file_path = self._package_path / file_path
        mappings = mappings or self.__urdf_xacro_args
        
        if (mappings is None) or all(
            (isinstance(key, str) and isinstance(value, str))
            for key, value in mappings.items()
        ):
            try:
                self.__moveit_configs.robot_description = {
                    self.__robot_description: load_xacro(
                        robot_description_file_path,
                        mappings=mappings,
                    )
                }
            except ParameterBuilderFileNotFoundError as e:
                logging.warning('\x1b[33;21m{}\x1b[0m'.format(e))
                logging.warning('\x1b[33;21mThe robot description will be loaded from /robot_description topic \x1b[0m')
        else:
            self.__moveit_configs.robot_description = {
                self.__robot_description: get_xacro_command(
                    str(robot_description_file_path), mappings=mappings
                )
            }

        return self

    def robot_description_semantic(
        self,
        file_path = None,
        mappings = None,
    ):
        """Load semantic robot description.

        :param file_path: Absolute or relative path to the SRDF file (w.r.t. xarm_moveit_config).
        :param mappings: mappings to be passed when loading the xacro file.
        :return: Instance of MoveItConfigsBuilder with robot_description_semantic loaded.
        """
        key = self.__robot_description + '_semantic'
        file_path = self._package_path / (file_path or self.__srdf_file_path)
        mappings = mappings or self.__srdf_xacro_args
        
        if (mappings is None) or all(
            (isinstance(key, str) and isinstance(value, str))
            for key, value in mappings.items()
        ):
            self.__moveit_configs.robot_description_semantic = {
                key: load_xacro(file_path, mappings=mappings)
            }
        else:
            self.__moveit_configs.robot_description_semantic = {
                key: get_xacro_command(str(file_path), mappings=mappings)
            }
        
        return self

    def robot_description_kinematics(self, file_path = None):
        """Load IK solver parameters.

        :param file_path: Absolute or relative path to the kinematics yaml file (w.r.t. xarm_moveit_config).
        :return: Instance of MoveItConfigsBuilder with robot_description_kinematics loaded.
        """
        key = self.__robot_description + '_kinematics'

        params = [self.__prefix_1, self.__prefix_2, self.__robot_type_1, self.__robot_dof_2, self.__robot_dof_1, self.__robot_dof_2]
        if all(isinstance(value, str) for value in params):
            robot_name_1 = '{}{}'.format(self.__robot_type_1, self.__robot_dof_1 if self.__robot_type_1 == 'xarm' else '6' if self.__robot_type_1 == 'lite' else '')
            robot_name_2 = '{}{}'.format(self.__robot_type_2, self.__robot_dof_2 if self.__robot_type_2 == 'xarm' else '6' if self.__robot_type_2 == 'lite' else '')
            
            if file_path is None:
                file_path_1 = self._package_path / 'config' / robot_name_1 / 'kinematics.yaml'
                file_path_2 = self._package_path / 'config' / robot_name_2 / 'kinematics.yaml'
                kinematics_yaml_1 = load_yaml(file_path_1)
                kinematics_yaml_2 = load_yaml(file_path_2)
            else:
                file_path_1 = self._package_path / file_path
                file_path_2 = self._package_path / file_path
                kinematics_yaml_1 = load_yaml(file_path_1) if file_path_1 else {}
                kinematics_yaml_2 = load_yaml(file_path_2) if file_path_2 else {}
            if kinematics_yaml_1 and self.__prefix_1:
                for name in list(kinematics_yaml_1.keys()):
                    kinematics_yaml_1['{}{}'.format(self.__prefix_1, name)] = kinematics_yaml_1.pop(name)
            if kinematics_yaml_2 and self.__prefix_2:
                for name in list(kinematics_yaml_2.keys()):
                    kinematics_yaml_2['{}{}'.format(self.__prefix_2, name)] = kinematics_yaml_2.pop(name)

            kinematics_yaml = {}
            kinematics_yaml.update(kinematics_yaml_1)
            kinematics_yaml.update(kinematics_yaml_2)
            self.__moveit_configs.robot_description_kinematics = {
                key: kinematics_yaml
            }
        else:
            self.__moveit_configs.robot_description_kinematics = {
                key: YamlParameterValue(
                    DualKinematicsYAML(file_path, package_path=self._package_path, 
                        prefix_1=self.__prefix_1, prefix_2=self.__prefix_2, 
                        robot_type_1=self.__robot_type_1, robot_type_2=self.__robot_type_2,
                        robot_dof_1=self.__robot_dof_1, robot_dof_2=self.__robot_dof_2
                    ), value_type=str)
            }

        return self

    def joint_limits(self, file_path = None):
        """Load joint limits overrides.

        :param file_path: Absolute or relative path to the joint limits yaml file (w.r.t. xarm_moveit_config).
        :return: Instance of MoveItConfigsBuilder with robot_description_planning loaded.
        """
        key = self.__robot_description + '_planning'
        params = [self.__prefix_1, self.__prefix_2, self.__robot_type_1, self.__robot_dof_2, self.__robot_dof_1, self.__robot_dof_2, self.__add_gripper_1, self.__add_gripper_2, self.__add_bio_gripper_1, self.__add_bio_gripper_2]

        if all(isinstance(value, str) for value in params):
            robot_name_1 = '{}{}'.format(self.__robot_type_1, self.__robot_dof_1 if self.__robot_type_1 == 'xarm' else '6' if self.__robot_type_1 == 'lite' else '')
            robot_name_2 = '{}{}'.format(self.__robot_type_2, self.__robot_dof_2 if self.__robot_type_2 == 'xarm' else '6' if self.__robot_type_2 == 'lite' else '')
            
            if file_path is None:
                file_path_1 = self._package_path / 'config' / robot_name_1 / 'joint_limits.yaml'
                file_path_2 = self._package_path / 'config' / robot_name_2 / 'joint_limits.yaml'
                joint_limits_1 = load_yaml(file_path_1)
                joint_limits_2 = load_yaml(file_path_2)
            else:
                file_path_1 = self._package_path / file_path
                file_path_2 = self._package_path / file_path
                joint_limits_1 = load_yaml(file_path_1) if file_path_1 else {}
                joint_limits_2 = load_yaml(file_path_2) if file_path_2 else {}
            if self.__add_gripper_1 in ('True', 'true'):
                gripper_joint_limits_yaml = load_yaml(self._package_path / 'config' / '{}_gripper'.format(self.__robot_type_1) / 'joint_limits.yaml')
                if gripper_joint_limits_yaml and 'joint_limits' in gripper_joint_limits_yaml:
                    joint_limits_1['joint_limits'].update(gripper_joint_limits_yaml['joint_limits'])
            elif self.__add_bio_gripper_1 in ('True', 'true'):
                gripper_joint_limits_yaml = load_yaml(self._package_path / 'config' / 'bio_gripper' / 'joint_limits.yaml')
                if gripper_joint_limits_yaml and 'joint_limits' in gripper_joint_limits_yaml:
                    joint_limits_1['joint_limits'].update(gripper_joint_limits_yaml['joint_limits'])
            if joint_limits_1 and self.__prefix_1:
                for name in list(joint_limits_1['joint_limits']):
                    joint_limits_1['joint_limits']['{}{}'.format(self.__prefix_1, name)] = joint_limits_1['joint_limits'].pop(name)
            
            if self.__add_gripper_2 in ('True', 'true'):
                gripper_joint_limits_yaml = load_yaml(self._package_path / 'config' / '{}_gripper'.format(self.__robot_type_1) / 'joint_limits.yaml')
                if gripper_joint_limits_yaml and 'joint_limits' in gripper_joint_limits_yaml:
                    joint_limits_2['joint_limits'].update(gripper_joint_limits_yaml['joint_limits'])
            elif self.__add_bio_gripper_2 in ('True', 'true'):
                gripper_joint_limits_yaml = load_yaml(self._package_path / 'config' / 'bio_gripper' / 'joint_limits.yaml')
                if gripper_joint_limits_yaml and 'joint_limits' in gripper_joint_limits_yaml:
                    joint_limits_2['joint_limits'].update(gripper_joint_limits_yaml['joint_limits'])
            if joint_limits_2 and self.__prefix_2:
                for name in list(joint_limits_2['joint_limits']):
                    joint_limits_2['joint_limits']['{}{}'.format(self.__prefix_2, name)] = joint_limits_2['joint_limits'].pop(name)
            
            joint_limits = {'joint_limits': {}}
            joint_limits['joint_limits'].update(joint_limits_1['joint_limits'])
            joint_limits['joint_limits'].update(joint_limits_2['joint_limits'])
            self.__moveit_configs.joint_limits = {
                key: joint_limits
            }
        else:
            self.__moveit_configs.joint_limits = {
                key: YamlParameterValue(
                    DualJointLimitsYAML(file_path, package_path=self._package_path, 
                        prefix_1=self.__prefix_1, prefix_2=self.__prefix_2, 
                        robot_type_1=self.__robot_type_1, robot_type_2=self.__robot_type_2, 
                        robot_dof_1=self.__robot_dof_1, robot_dof_2=self.__robot_dof_2,
                        add_gripper_1=self.__add_gripper_1, add_gripper_2=self.__add_gripper_2, 
                        add_bio_gripper_1=self.__add_bio_gripper_1, add_bio_gripper_2=self.__add_bio_gripper_2,
                ), value_type=str)
            }

        return self

    def moveit_cpp(self, file_path = None):
        """Load MoveItCpp parameters.

        :param file_path: Absolute or relative path to the MoveItCpp yaml file (w.r.t. xarm_moveit_config).
        :return: Instance of MoveItConfigsBuilder with moveit_cpp loaded.
        """
        params = [self.__robot_type_1, self.__robot_type_2, self.__robot_dof_1, self.__robot_dof_2]
        if all(isinstance(value, str) for value in params):
            robot_name_1 = '{}{}'.format(self.__robot_type_1, self.__robot_dof_1 if self.__robot_type_1 == 'xarm' else '6' if self.__robot_type_1 == 'lite' else '')
            robot_name_2 = '{}{}'.format(self.__robot_type_2, self.__robot_dof_2 if self.__robot_type_2 == 'xarm' else '6' if self.__robot_type_2 == 'lite' else '')
            
            if file_path is None:
                file_path_1 = self._package_path / 'config' / robot_name_1 / 'moveit_cpp.yaml'
                file_path_2 = self._package_path / 'config' / robot_name_2 / 'moveit_cpp.yaml'
                moveit_cpp = load_yaml(file_path_1)
                moveit_cpp_2 = load_yaml(file_path_2)
                moveit_cpp.update(moveit_cpp_2)
            else:
                file_path = self._package_path / file_path
                moveit_cpp = load_yaml(file_path) if file_path and file_path.exists() else {}
            self.__moveit_configs.moveit_cpp = moveit_cpp
        
        return self

    def trajectory_execution(
        self,
        file_path = None,
        controllers_name = None,
        moveit_manage_controllers = False,
    ):
        """Load trajectory execution and moveit controller managers' parameters

        :param file_path: Absolute or relative path to the controllers yaml file (w.r.t. xarm_moveit_config).
        :param moveit_manage_controllers: Whether trajectory execution manager is allowed to switch controllers' states.
        :return: Instance of MoveItConfigsBuilder with trajectory_execution loaded.
        """
        controllers_name = controllers_name if controllers_name else self.__controllers_name
        
        params = [self.__prefix_1, self.__prefix_2, self.__robot_type_1, self.__robot_dof_2, self.__robot_dof_1, self.__robot_dof_2, self.__add_gripper_1, self.__add_gripper_2, self.__add_bio_gripper_1, self.__add_bio_gripper_2, controllers_name]
        if all(isinstance(value, str) for value in params):
            robot_name_1 = '{}{}'.format(self.__robot_type_1, self.__robot_dof_1 if self.__robot_type_1 == 'xarm' else '6' if self.__robot_type_1 == 'lite' else '')
            robot_name_2 = '{}{}'.format(self.__robot_type_2, self.__robot_dof_2 if self.__robot_type_2 == 'xarm' else '6' if self.__robot_type_2 == 'lite' else '')
            controllers_name = controllers_name if controllers_name.endswith('.yaml') else '{}.yaml'.format(controllers_name)
            if file_path is None:
                file_path_1 = self._package_path / 'config' / robot_name_1 / controllers_name
                file_path_2 = self._package_path / 'config' / robot_name_2 / controllers_name
                controllers_yaml_1 = load_yaml(file_path_1)
                controllers_yaml_2 = load_yaml(file_path_2)
                if self.__add_gripper_1 in ('True', 'true'):
                    gripper_controllers_yaml = load_yaml(self._package_path / 'config' / '{}_gripper'.format(self.__robot_type_1) / controllers_name)
                    if gripper_controllers_yaml and 'controller_names' in gripper_controllers_yaml:
                        for name in gripper_controllers_yaml['controller_names']:
                            if name in gripper_controllers_yaml:
                                if name not in controllers_yaml_1['controller_names']:
                                    controllers_yaml_1['controller_names'].append(name)
                                controllers_yaml_1[name] = gripper_controllers_yaml[name]
                elif self.__add_bio_gripper_1 in ('True', 'true'):
                    gripper_controllers_yaml = load_yaml(self._package_path / 'config' / 'bio_gripper' / controllers_name)
                    if gripper_controllers_yaml and 'controller_names' in gripper_controllers_yaml:
                        for name in gripper_controllers_yaml['controller_names']:
                            if name in gripper_controllers_yaml:
                                if name not in controllers_yaml_1['controller_names']:
                                    controllers_yaml_1['controller_names'].append(name)
                                controllers_yaml_1[name] = gripper_controllers_yaml[name]
                
                if self.__add_gripper_2 in ('True', 'true'):
                    gripper_controllers_yaml = load_yaml(self._package_path / 'config' / '{}_gripper'.format(self.__robot_type_2) / controllers_name)
                    if gripper_controllers_yaml and 'controller_names' in gripper_controllers_yaml:
                        for name in gripper_controllers_yaml['controller_names']:
                            if name in gripper_controllers_yaml:
                                if name not in controllers_yaml_2['controller_names']:
                                    controllers_yaml_2['controller_names'].append(name)
                                controllers_yaml_2[name] = gripper_controllers_yaml[name]
                elif self.__add_bio_gripper_2 in ('True', 'true'):
                    gripper_controllers_yaml = load_yaml(self._package_path / 'config' / 'bio_gripper' / controllers_name)
                    if gripper_controllers_yaml and 'controller_names' in gripper_controllers_yaml:
                        for name in gripper_controllers_yaml['controller_names']:
                            if name in gripper_controllers_yaml:
                                if name not in controllers_yaml_2['controller_names']:
                                    controllers_yaml_2['controller_names'].append(name)
                                controllers_yaml_2[name] = gripper_controllers_yaml[name]
            else:
                file_path_1 = self._package_path / file_path
                file_path_2 = self._package_path / file_path
                controllers_yaml_1 = load_yaml(file_path_1) if file_path_1 else {}
                controllers_yaml_2 = load_yaml(file_path_2) if file_path_2 else {}

            if controllers_yaml_1 and self.__prefix_1:
                for i, name in enumerate(controllers_yaml_1['controller_names']):
                    joints = controllers_yaml_1.get(name, {}).get('joints', [])
                    for j, joint in enumerate(joints):
                        joints[j] = '{}{}'.format(self.__prefix_1, joint)
                    controllers_yaml_1['controller_names'][i] = '{}{}'.format(self.__prefix_1, name)
                    if name in controllers_yaml_1:
                        controllers_yaml_1['{}{}'.format(self.__prefix_1, name)] = controllers_yaml_1.pop(name)
            if controllers_yaml_2 and self.__prefix_2:
                for i, name in enumerate(controllers_yaml_2['controller_names']):
                    joints = controllers_yaml_2.get(name, {}).get('joints', [])
                    for j, joint in enumerate(joints):
                        joints[j] = '{}{}'.format(self.__prefix_2, joint)
                    controllers_yaml_2['controller_names'][i] = '{}{}'.format(self.__prefix_2, name)
                    if name in controllers_yaml_2:
                        controllers_yaml_2['{}{}'.format(self.__prefix_2, name)] = controllers_yaml_2.pop(name)

            controllers_yaml = {}
            controllers_yaml.update(controllers_yaml_1)
            controllers_yaml['controller_names'].extend(controllers_yaml_2['controller_names'])
            controllers_yaml_2.pop('controller_names', [])
            controllers_yaml.update(controllers_yaml_2)

            self.__moveit_configs.trajectory_execution = {
                'moveit_manage_controllers': moveit_manage_controllers,
                'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
                'moveit_simple_controller_manager': controllers_yaml,
                'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                'trajectory_execution.allowed_goal_duration_margin': 0.5,
                'trajectory_execution.allowed_start_tolerance': 0.01,
                'trajectory_execution.execution_duration_monitoring': False
            }
        else:
            self.__moveit_configs.trajectory_execution = {
                'moveit_manage_controllers': moveit_manage_controllers,
                'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
                'moveit_simple_controller_manager': YamlParameterValue(
                    DualControllersYAML(file_path, package_path=self._package_path, 
                        prefix_1=self.__prefix_1, prefix_2=self.__prefix_2, 
                        robot_type_1=self.__robot_type_1, robot_type_2=self.__robot_type_2, 
                        robot_dof_1=self.__robot_dof_1, robot_dof_2=self.__robot_dof_2,
                        add_gripper_1=self.__add_gripper_1, add_gripper_2=self.__add_gripper_2, 
                        add_bio_gripper_1=self.__add_bio_gripper_1, add_bio_gripper_2=self.__add_bio_gripper_2, 
                        controllers_name=controllers_name
                    ), value_type=str),
                'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                'trajectory_execution.allowed_goal_duration_margin': 0.5,
                'trajectory_execution.allowed_start_tolerance': 0.01,
                'trajectory_execution.execution_duration_monitoring': False
            }
        return self

    def planning_scene_monitor(
        self,
        publish_planning_scene = True,
        publish_geometry_updates = True,
        publish_state_updates = True,
        publish_transforms_updates = True,
        publish_robot_description = False,
        publish_robot_description_semantic = False,
    ):
        self.__moveit_configs.planning_scene_monitor = {
            # TODO: Fix parameter namespace upstream -- see planning_scene_monitor.cpp:262
            # 'planning_scene_monitor': {
            'publish_planning_scene': publish_planning_scene,
            'publish_geometry_updates': publish_geometry_updates,
            'publish_state_updates': publish_state_updates,
            'publish_transforms_updates': publish_transforms_updates,
            'publish_robot_description': publish_robot_description,
            'publish_robot_description_semantic': publish_robot_description_semantic,
            # }
        }
        return self

    def sensors_3d(self, file_path = None):
        """Load sensors_3d parameters.

        :param file_path: Absolute or relative path to the sensors_3d yaml file (w.r.t. xarm_moveit_config).
        :return: Instance of MoveItConfigsBuilder with robot_description_planning loaded.
        """
        params = [self.__robot_type_1, self.__robot_type_2, self.__robot_dof_1, self.__robot_dof_2]
        if all(isinstance(value, str) for value in params):
            robot_name_1 = '{}{}'.format(self.__robot_type_1, self.__robot_dof_1 if self.__robot_type_1 == 'xarm' else '6' if self.__robot_type_1 == 'lite' else '')
            robot_name_2 = '{}{}'.format(self.__robot_type_2, self.__robot_dof_2 if self.__robot_type_2 == 'xarm' else '6' if self.__robot_type_2 == 'lite' else '')
            
            if file_path is None:
                file_path_1 = self._package_path / 'config' / robot_name_1 / 'sensors_3d.yaml'
                file_path_2 = self._package_path / 'config' / robot_name_2 / 'sensors_3d.yaml'
                sensors_data = {}
                if file_path_1.exists():
                    sensors_data_1 = load_yaml(file_path_1)
                    sensors_data.update(sensors_data_1)
                if file_path_2.exists():
                    sensors_data_2 = load_yaml(file_path_2)
                    sensors_data.update(sensors_data_2)
            else:
                file_path = self._package_path / file_path
                sensors_data = load_yaml(file_path) if file_path and file_path.exists() else {}

            # TODO(mikeferguson): remove the second part of this check once
            # https://github.com/ros-planning/moveit_resources/pull/141 has made through buildfarm
            if sensors_data and len(sensors_data['sensors']) > 0 and sensors_data['sensors'][0]:
                self.__moveit_configs.sensors_3d = sensors_data
        
        return self

    def planning_pipelines(
        self,
        default_planning_pipeline = None,
        pipelines = None,
        load_all = True,
    ):
        """Load planning pipelines parameters.

        :param default_planning_pipeline: Name of the default planning pipeline.
        :param pipelines: List of the planning pipelines to be loaded.
        :param load_all: Only used if pipelines is None.
                         If true, loads all pipelines defined in config package AND this package.
                         If false, only loads the pipelines defined in config package.
        :return: Instance of MoveItConfigsBuilder with planning_pipelines loaded.
        """
        params = [self.__prefix_1, self.__prefix_2, self.__robot_type_1, self.__robot_dof_2, self.__robot_dof_1, self.__robot_dof_2, self.__add_gripper_1, self.__add_gripper_2, self.__add_bio_gripper_1, self.__add_bio_gripper_2]
        if all(isinstance(value, str) for value in params):
            robot_name_1 = '{}{}'.format(self.__robot_type_1, self.__robot_dof_1 if self.__robot_type_1 == 'xarm' else '6' if self.__robot_type_1 == 'lite' else '')
            robot_name_2 = '{}{}'.format(self.__robot_type_2, self.__robot_dof_2 if self.__robot_type_2 == 'xarm' else '6' if self.__robot_type_2 == 'lite' else '')
            config_folder_1 = self._package_path / 'config' / robot_name_1
            config_folder_2 = self._package_path / 'config' / robot_name_2
            if pipelines is None:
                planning_pattern = re.compile('^(.*)_planning.yaml$')
                pipelines_1 = get_pattern_matches(config_folder_1, planning_pattern)
                pipelines_2 = get_pattern_matches(config_folder_2, planning_pattern)
                pipelines = []
                pipelines.extend(pipelines_1)
                pipelines.extend(pipelines_2)
                pipelines = list(set(pipelines))
            else:
                pipelines = list(set(pipelines))
                pipelines_1 = pipelines
                pipelines_2 = pipelines
             # Define default pipeline as needed
            if not default_planning_pipeline:
                if not pipelines or 'ompl' in pipelines:
                    default_planning_pipeline = 'ompl'
                else:
                    default_planning_pipeline = pipelines[0]

            if default_planning_pipeline not in pipelines:
                raise RuntimeError(
                    'default_planning_pipeline: `{}` doesn\'t name any of the input pipelines `{}`'.format(default_planning_pipeline, ','.join(pipelines))
                )

            self.__moveit_configs.planning_pipelines = {
                'planning_pipelines': pipelines,
                'default_planning_pipeline': default_planning_pipeline,
            }
            
            for pipeline in pipelines:
                filename = pipeline + '_planning.yaml'
                planning_yaml = {}
                planning_yaml_1 = {}
                planning_yaml_2 = {}
                if pipeline in pipelines_1:
                    parameter_file = config_folder_1 / filename
                    planning_yaml_1 = load_yaml(parameter_file)
                    if self.__add_gripper_1 in ('True', 'true'):
                        parameter_file = self._package_path / 'config' / '{}_gripper'.format(self.__robot_type_1) / filename
                        if parameter_file.exists():
                            gripper_planning_yaml = load_yaml(parameter_file)
                            if gripper_planning_yaml:
                                planning_yaml_1.update(gripper_planning_yaml)
                    elif self.__add_bio_gripper_1 in ('True', 'true'):
                        parameter_file = self._package_path / 'config' / 'bio_gripper' / filename
                        if parameter_file.exists():
                            gripper_planning_yaml = load_yaml(parameter_file)
                            if gripper_planning_yaml:
                                planning_yaml_1.update(gripper_planning_yaml)
                    if planning_yaml_1 and self.__prefix_1:
                        for name in list(planning_yaml_1.keys()):
                            if pipeline == 'ompl' and name != 'planner_configs':
                                planning_yaml_1['{}{}'.format(self.__prefix_1, name)] = planning_yaml_1.pop(name)

                if pipeline in pipelines_2:
                    parameter_file = config_folder_2 / filename
                    planning_yaml_2 = load_yaml(parameter_file)
                                
                    if self.__add_gripper_2 in ('True', 'true'):
                        parameter_file = self._package_path / 'config' / '{}_gripper'.format(self.__robot_type_2) / filename
                        if parameter_file.exists():
                            gripper_planning_yaml = load_yaml(parameter_file)
                            if gripper_planning_yaml:
                                planning_yaml_2.update(gripper_planning_yaml)
                    elif self.__add_bio_gripper_2 in ('True', 'true'):
                        parameter_file = self._package_path / 'config' / 'bio_gripper' / filename
                        if parameter_file.exists():
                            gripper_planning_yaml = load_yaml(parameter_file)
                            if gripper_planning_yaml:
                                planning_yaml_2.update(gripper_planning_yaml)
                    if planning_yaml_2 and self.__prefix_2:
                        for name in list(planning_yaml_2.keys()):
                            if pipeline == 'ompl' and name != 'planner_configs':
                                planning_yaml_2['{}{}'.format(self.__prefix_2, name)] = planning_yaml_2.pop(name)
                
                planning_yaml.update(planning_yaml_1)
                planning_yaml.update(planning_yaml_2)
                self.__moveit_configs.planning_pipelines[pipeline] = planning_yaml
            
            # Special rule to add ompl planner_configs
            if 'ompl' in self.__moveit_configs.planning_pipelines:
                ompl_config = self.__moveit_configs.planning_pipelines['ompl']
                if os.environ.get('ROS_DISTRO', '') > 'iron':
                    ompl_config.update({
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
                    })
                else:
                    ompl_config.update({
                        'planning_plugin': 'ompl_interface/OMPLPlanner',
                        'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
                        'start_state_max_bounds_error': 0.1,
                    })
        else:
            pipelines = list(set(pipelines)) if pipelines else ['ompl']
            default_planning_pipeline = default_planning_pipeline if default_planning_pipeline else 'ompl'
            if default_planning_pipeline not in pipelines:
                raise RuntimeError(
                    'default_planning_pipeline: `{}` doesn\'t name any of the input pipelines `{}`'.format(default_planning_pipeline, ','.join(pipelines))
                )
            self.__moveit_configs.planning_pipelines = {
                'planning_pipelines': pipelines,
                'default_planning_pipeline': default_planning_pipeline,
            }
            for pipeline in pipelines:
                self.__moveit_configs.planning_pipelines[pipeline] = YamlParameterValue(
                    DualPlanningPipelinesYAML(pipeline, package_path=self._package_path, 
                        prefix_1=self.__prefix_1, prefix_2=self.__prefix_2, 
                        robot_type_1=self.__robot_type_1, robot_type_2=self.__robot_type_2, 
                        robot_dof_1=self.__robot_dof_1, robot_dof_2=self.__robot_dof_2,
                        add_gripper_1=self.__add_gripper_1, add_gripper_2=self.__add_gripper_2, 
                        add_bio_gripper_1=self.__add_bio_gripper_1, add_bio_gripper_2=self.__add_bio_gripper_2, 
                    ), value_type=str)
        
        return self

    def pilz_cartesian_limits(self, file_path = None):
        """Load cartesian limits.

        :param file_path: Absolute or relative path to the cartesian limits file (w.r.t. xarm_moveit_config).
        :return: Instance of MoveItConfigsBuilder with pilz_cartesian_limits loaded.
        """
        params = [self.__robot_type_1, self.__robot_type_2, self.__robot_dof_1, self.__robot_dof_2]
        if all(isinstance(value, str) for value in params):
            robot_name_1 = '{}{}'.format(self.__robot_type_1, self.__robot_dof_1 if self.__robot_type_1 == 'xarm' else '6' if self.__robot_type_1 == 'lite' else '')
            robot_name_2 = '{}{}'.format(self.__robot_type_2, self.__robot_dof_2 if self.__robot_type_2 == 'xarm' else '6' if self.__robot_type_2 == 'lite' else '')
            
            deprecated_path_1 = self._package_path / 'config' / robot_name_1 / 'cartesian_limits.yaml'
            deprecated_path_2 = self._package_path / 'config' / robot_name_2 / 'cartesian_limits.yaml'
            if deprecated_path_1.exists() or deprecated_path_2.exists():
                logging.warning('\x1b[33;21mcartesian_limits.yaml is deprecated, please rename to pilz_cartesian_limits.yaml\x1b[0m')
            if file_path is None:
                file_path_1 = self._package_path / 'config' / robot_name_1 / 'pilz_cartesian_limits.yaml'
                file_path_2 = self._package_path / 'config' / robot_name_2 / 'pilz_cartesian_limits.yaml'
                pilz_cartesian_limits = {}
                if file_path_1.exists():
                    pilz_cartesian_limits_1 = load_yaml(file_path_1)
                    pilz_cartesian_limits.update(pilz_cartesian_limits_1)
                if file_path_2.exists():
                    pilz_cartesian_limits_2 = load_yaml(file_path_2)
                    pilz_cartesian_limits.update(pilz_cartesian_limits_2)
            else:
                file_path = self._package_path / file_path
                pilz_cartesian_limits = load_yaml(file_path) if file_path and file_path.exists() else {}
            
            key = self.__robot_description + '_planning'
            self.__moveit_configs.pilz_cartesian_limits = {
                key: pilz_cartesian_limits
            }
        else:
            key = self.__robot_description + '_planning'
            self.__moveit_configs.pilz_cartesian_limits = {
                key: YamlParameterValue(
                    DualCommonYAML('pilz_cartesian_limits.yaml', package_path=self._package_path, 
                        prefix_1=self.__prefix_1, prefix_2=self.__prefix_2, 
                        robot_type_1=self.__robot_type_1, robot_type_2=self.__robot_type_2, 
                        robot_dof_1=self.__robot_dof_1, robot_dof_2=self.__robot_dof_2,
                ), value_type=str)
            }
        
        return self

    def to_moveit_configs(self):
        """Get MoveIt configs from xarm_moveit_config.

        :return: An MoveItConfigs instance with all parameters loaded.
        """
        if not self.__moveit_configs.robot_description:
            self.robot_description()
        if not self.__moveit_configs.robot_description_semantic:
            self.robot_description_semantic()
        if not self.__moveit_configs.robot_description_kinematics:
            self.robot_description_kinematics()
        if not self.__moveit_configs.planning_pipelines:
            self.planning_pipelines()
        if not self.__moveit_configs.trajectory_execution:
            self.trajectory_execution()
        if not self.__moveit_configs.planning_scene_monitor:
            self.planning_scene_monitor()
        if not self.__moveit_configs.sensors_3d:
            self.sensors_3d()
        if not self.__moveit_configs.joint_limits:
            self.joint_limits()
        # TODO(JafarAbdi): We should have a default moveit_cpp.yaml as port of a moveit config package
        # if not self.__moveit_configs.moveit_cpp:
        #     self.moveit_cpp()
        if 'pilz_industrial_motion_planner' in self.__moveit_configs.planning_pipelines:
            if not self.__moveit_configs.pilz_cartesian_limits:
                self.pilz_cartesian_limits()
        return self.__moveit_configs

    def to_dict(self, include_moveit_configs = True):
        """Get loaded parameters from xarm_moveit_config as a dictionary.

        :param include_moveit_configs: Whether to include the MoveIt config parameters or
                                       only the ones from ParameterBuilder
        :return: Dictionary with all parameters loaded.
        """
        parameters = self._parameters
        if include_moveit_configs:
            parameters.update(self.to_moveit_configs().to_dict())
        return parameters