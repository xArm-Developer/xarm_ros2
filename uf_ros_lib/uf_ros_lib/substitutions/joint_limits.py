#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2024, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 
import yaml
from launch.frontend import expose_substitution
from launch.substitution import Substitution
from launch_param_builder import load_yaml
from .common import BaseYamlSubstitution


@expose_substitution("joint-limits")
class JointLimitsYAML(BaseYamlSubstitution):
    """Substitution that can access load joint_limits file with mappings involving any subsititutable."""

    def __init__(self, file_path, package_path=None, 
        prefix='', robot_type='xarm', robot_dof=7, 
        add_gripper=False, add_bio_gripper=False):
        super().__init__()
        self.__package_path = package_path
        self.__file_path = file_path
        self.__prefix = prefix
        self.__robot_type = robot_type
        self.__robot_dof = robot_dof
        self.__add_gripper = add_gripper
        self.__add_bio_gripper = add_bio_gripper

    @classmethod
    def parse(cls, data):
        """Parse `JointLimitsYAML` substitution."""
        if len(data) != 1:
            raise TypeError('JointLimitsYAML substitution expects 1 argument')
        kwargs = {}
        kwargs["file_path"] = data[0]
        return cls, kwargs

    def describe(self):
        """Return a description of this substitution as a string."""
        return 'JointLimitsYAML(file_path={}, package_path={}, prefix={}, robot_type={}, robot_dof={}, add_gripper={}, add_bio_gripper={})'.format(
            self.get_var_describe(self.__file_path), 
            self.get_var_describe(self.__package_path),
            self.get_var_describe(self.__prefix),
            self.get_var_describe(self.__robot_type),
            self.get_var_describe(self.__robot_dof),
            self.get_var_describe(self.__add_gripper),
            self.get_var_describe(self.__add_bio_gripper)
        )

    def perform(self, context):
        """
        Perform the substitution by retrieving the mappings and context.
        """
        prefix = self.get_var_perform(self.__prefix, context)
        robot_type = self.get_var_perform(self.__robot_type, context)
        robot_dof = self.get_var_perform(self.__robot_dof, context)
        add_gripper = self.get_var_perform(self.__add_gripper, context).lower() == 'true'
        add_bio_gripper = self.get_var_perform(self.__add_bio_gripper, context).lower() == 'true'

        robot_name = '{}{}'.format(robot_type, robot_dof if robot_type == 'xarm' else '6' if robot_type == 'lite' else '')
        file_path = self.__file_path if self.__file_path else (self.__package_path / 'config' / robot_name / 'joint_limits.yaml')

        joint_limits = load_yaml(file_path)
        joint_limits = joint_limits if joint_limits else {}
        if robot_type != 'lite' and add_gripper:
            gripper_joint_limits_yaml = load_yaml(self.__package_path / 'config' / '{}_gripper'.format(robot_type) / 'joint_limits.yaml')
            if gripper_joint_limits_yaml and 'joint_limits' in gripper_joint_limits_yaml:
                joint_limits['joint_limits'].update(gripper_joint_limits_yaml['joint_limits'])
        elif robot_type != 'lite' and add_bio_gripper:
            gripper_joint_limits_yaml = load_yaml(self.__package_path / 'config' / 'bio_gripper' / 'joint_limits.yaml')
            if gripper_joint_limits_yaml and 'joint_limits' in gripper_joint_limits_yaml:
                joint_limits['joint_limits'].update(gripper_joint_limits_yaml['joint_limits'])
        if prefix:
            for name in list(joint_limits['joint_limits']):
                joint_limits['joint_limits']['{}{}'.format(prefix, name)] = joint_limits['joint_limits'].pop(name)
        
        return yaml.dump(joint_limits)


@expose_substitution("dual-joint-limits")
class DualJointLimitsYAML(BaseYamlSubstitution):
    """Substitution that can access load joint_limits file with mappings involving any subsititutable."""

    def __init__(self, file_path, package_path=None, 
        prefix_1='L_', prefix_2='R_',
        robot_type_1='xarm', robot_type_2='xarm',
        robot_dof_1=7, robot_dof_2=7,
        add_gripper_1=False, add_gripper_2=False, 
        add_bio_gripper_1=False, add_bio_gripper_2=False):
        super().__init__()
        self.__package_path = package_path
        self.__file_path = file_path
        self.__prefix_1 = prefix_1
        self.__prefix_2 = prefix_2
        self.__robot_type_1 = robot_type_1
        self.__robot_type_2 = robot_type_2
        self.__robot_dof_1 = robot_dof_1
        self.__robot_dof_2 = robot_dof_2
        self.__add_gripper_1 = add_gripper_1
        self.__add_gripper_2 = add_gripper_2
        self.__add_bio_gripper_1 = add_bio_gripper_1
        self.__add_bio_gripper_2 = add_bio_gripper_2

    @classmethod
    def parse(cls, data):
        """Parse `DualJointLimitsYAML` substitution."""
        if len(data) != 1:
            raise TypeError('DualJointLimitsYAML substitution expects 1 argument')
        kwargs = {}
        kwargs["file_path"] = data[0]
        return cls, kwargs

    def describe(self):
        """Return a description of this substitution as a string."""
        return 'DualJointLimitsYAML(file_path={}, package_path={}, prefix_1={}, prefix_2={}, robot_type_1={}, robot_type_2={}, robot_dof_1={}, robot_dof_2={}, add_gripper_1={}, add_gripper_2={}, add_bio_gripper_1={}, add_bio_gripper_2={})'.format(
            self.get_var_describe(self.__file_path),
            self.get_var_describe(self.__package_path),
            self.get_var_describe(self.__prefix_1),
            self.get_var_describe(self.__prefix_2),
            self.get_var_describe(self.__robot_type_1),
            self.get_var_describe(self.__robot_type_2),
            self.get_var_describe(self.__robot_dof_1),
            self.get_var_describe(self.__robot_dof_2),
            self.get_var_describe(self.__add_gripper_1),
            self.get_var_describe(self.__add_gripper_2),
            self.get_var_describe(self.__add_bio_gripper_1),
            self.get_var_describe(self.__add_bio_gripper_2)
        )

    def perform(self, context):
        """
        Perform the substitution by retrieving the mappings and context.
        """
        prefix_1 = self.get_var_perform(self.__prefix_1, context)
        prefix_2 = self.get_var_perform(self.__prefix_2, context)
        robot_type_1 = self.get_var_perform(self.__robot_type_1, context)
        robot_type_2 = self.get_var_perform(self.__robot_type_2, context)
        robot_dof_1 = self.get_var_perform(self.__robot_dof_1, context)
        robot_dof_2 = self.get_var_perform(self.__robot_dof_2, context)
        add_gripper_1 = self.get_var_perform(self.__add_gripper_1, context).lower() == 'true'
        add_gripper_2 = self.get_var_perform(self.__add_gripper_2, context).lower() == 'true'
        add_bio_gripper_1 = self.get_var_perform(self.__add_bio_gripper_1, context).lower() == 'true'
        add_bio_gripper_2 = self.get_var_perform(self.__add_bio_gripper_2, context).lower() == 'true'

        robot_name_1 = '{}{}'.format(robot_type_1, robot_dof_1 if robot_type_1 == 'xarm' else '6' if robot_type_1 == 'lite' else '')
        robot_name_2 = '{}{}'.format(robot_type_2, robot_dof_2 if robot_type_2 == 'xarm' else '6' if robot_type_2 == 'lite' else '')

        file_path_1 = self.__file_path if self.__file_path else (self.__package_path / 'config' / robot_name_1 / 'joint_limits.yaml')
        file_path_2 = self.__file_path if self.__file_path else (self.__package_path / 'config' / robot_name_2 / 'joint_limits.yaml')

        joint_limits_1 = load_yaml(file_path_1)
        joint_limits_1 = joint_limits_1 if joint_limits_1 else {}
        if robot_type_1 != 'lite' and add_gripper_1:
            gripper_joint_limits_yaml = load_yaml(self.__package_path / 'config' / '{}_gripper'.format(robot_type_1) / 'joint_limits.yaml')
            if gripper_joint_limits_yaml and 'joint_limits' in gripper_joint_limits_yaml:
                joint_limits_1['joint_limits'].update(gripper_joint_limits_yaml['joint_limits'])
        elif robot_type_1 != 'lite' and add_bio_gripper_1:
            gripper_joint_limits_yaml = load_yaml(self.__package_path / 'config' / 'bio_gripper' / 'joint_limits.yaml')
            if gripper_joint_limits_yaml and 'joint_limits' in gripper_joint_limits_yaml:
                joint_limits_1['joint_limits'].update(gripper_joint_limits_yaml['joint_limits'])
        if prefix_1:
            for name in list(joint_limits_1['joint_limits']):
                joint_limits_1['joint_limits']['{}{}'.format(prefix_1, name)] = joint_limits_1['joint_limits'].pop(name)
        
        joint_limits_2 = load_yaml(file_path_2)
        joint_limits_2 = joint_limits_2 if joint_limits_2 else {}
        if robot_type_2 != 'lite' and add_gripper_2:
            gripper_joint_limits_yaml = load_yaml(self.__package_path / 'config' / '{}_gripper'.format(robot_type_2) / 'joint_limits.yaml')
            if gripper_joint_limits_yaml and 'joint_limits' in gripper_joint_limits_yaml:
                joint_limits_2['joint_limits'].update(gripper_joint_limits_yaml['joint_limits'])
        elif robot_type_2 != 'lite' and add_bio_gripper_2:
            gripper_joint_limits_yaml = load_yaml(self.__package_path / 'config' / 'bio_gripper' / 'joint_limits.yaml')
            if gripper_joint_limits_yaml and 'joint_limits' in gripper_joint_limits_yaml:
                joint_limits_2['joint_limits'].update(gripper_joint_limits_yaml['joint_limits'])
        if prefix_2:
            for name in list(joint_limits_2['joint_limits']):
                joint_limits_2['joint_limits']['{}{}'.format(prefix_2, name)] = joint_limits_2['joint_limits'].pop(name)
        
        joint_limits = {'joint_limits': {}}
        joint_limits['joint_limits'].update(joint_limits_1['joint_limits'])
        joint_limits['joint_limits'].update(joint_limits_2['joint_limits'])

        return yaml.dump(joint_limits)
