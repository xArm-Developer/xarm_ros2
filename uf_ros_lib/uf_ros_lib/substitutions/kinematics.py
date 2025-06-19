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


@expose_substitution("kinematics")
class KinematicsYAML(BaseYamlSubstitution):
    """Substitution that can access load Kinematics file with mappings involving any subsititutable."""

    def __init__(self, file_path, package_path=None, prefix='', robot_type='xarm', robot_dof=7):
        super().__init__()
        self.__file_path = file_path
        self.__package_path = package_path
        self.__prefix = prefix
        self.__robot_type = robot_type
        self.__robot_dof = robot_dof

    @classmethod
    def parse(cls, data):
        """Parse `KinematicsYAML` substitution."""
        if len(data) != 1:
            raise TypeError('KinematicsYAML substitution expects 1 argument')
        kwargs = {}
        kwargs["file_path"] = data[0]
        return cls, kwargs

    def describe(self):
        """Return a description of this substitution as a string."""
        return 'KinematicsYAML(file_path={}, package_path={}, prefix={}, robot_type={}, robot_dof={})'.format(
            self.get_var_describe(self.__file_path), 
            self.get_var_describe(self.__package_path),
            self.get_var_describe(self.__prefix),
            self.get_var_describe(self.__robot_type),
            self.get_var_describe(self.__robot_dof)
        )

    def perform(self, context):
        """
        Perform the substitution by retrieving the mappings and context.
        """
        prefix = self.get_var_perform(self.__prefix, context)
        robot_type = self.get_var_perform(self.__robot_type, context)
        robot_dof = self.get_var_perform(self.__robot_dof, context)

        robot_name = '{}{}'.format(robot_type, robot_dof if robot_type == 'xarm' else '6' if robot_type == 'lite' else '')
        
        file_path = self.__file_path if self.__file_path else (self.__package_path / 'config' / robot_name / 'kinematics.yaml')

        kinematics_yaml = load_yaml(file_path)
        kinematics_yaml = kinematics_yaml if kinematics_yaml else {}
        if prefix:
            for name in list(kinematics_yaml.keys()):
                kinematics_yaml['{}{}'.format(prefix, name)] = kinematics_yaml.pop(name)
        
        return yaml.dump(kinematics_yaml)


@expose_substitution("dual-kinematics")
class DualKinematicsYAML(BaseYamlSubstitution):
    """Substitution that can access load Kinematics file with mappings involving any subsititutable."""

    def __init__(self, file_path, package_path=None, 
        prefix_1='L_', prefix_2='R_',
        robot_type_1='xarm', robot_type_2='xarm',
        robot_dof_1=7, robot_dof_2=7):
        super().__init__()
        self.__file_path = file_path
        self.__package_path = package_path
        self.__prefix_1 = prefix_1
        self.__prefix_2 = prefix_2
        self.__robot_type_1 = robot_type_1
        self.__robot_type_2 = robot_type_2
        self.__robot_dof_1 = robot_dof_1
        self.__robot_dof_2 = robot_dof_2

    @classmethod
    def parse(cls, data):
        """Parse `DualKinematicsYAML` substitution."""
        if len(data) != 1:
            raise TypeError('DualKinematicsYAML substitution expects 1 argument')
        kwargs = {}
        kwargs["file_path"] = data[0]
        return cls, kwargs

    def describe(self):
        """Return a description of this substitution as a string."""
        return 'DualKinematicsYAML(file_path={}, package_path={}, prefix_1={}, prefix_2={}, robot_type_1={}, robot_type_2={}, robot_dof_1={}, robot_dof_2={})'.format(
            self.get_var_describe(self.__file_path), 
            self.get_var_describe(self.__package_path),
            self.get_var_describe(self.__prefix_1),
            self.get_var_describe(self.__prefix_2),
            self.get_var_describe(self.__robot_type_1),
            self.get_var_describe(self.__robot_type_2),
            self.get_var_describe(self.__robot_dof_1),
            self.get_var_describe(self.__robot_dof_2)
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

        robot_name_1 = '{}{}'.format(robot_type_1, robot_dof_1 if robot_type_1 == 'xarm' else '6' if robot_type_1 == 'lite' else '')
        robot_name_2 = '{}{}'.format(robot_type_2, robot_dof_2 if robot_type_2 == 'xarm' else '6' if robot_type_2 == 'lite' else '')
        
        file_path_1 = self.__file_path if self.__file_path else (self.__package_path / 'config' / robot_name_1 / 'kinematics.yaml')
        file_path_2 = self.__file_path if self.__file_path else (self.__package_path / 'config' / robot_name_2 / 'kinematics.yaml')

        kinematics_yaml_1 = load_yaml(file_path_1)
        kinematics_yaml_1 = kinematics_yaml_1 if kinematics_yaml_1 else {}
        if prefix_1:
            for name in list(kinematics_yaml_1.keys()):
                kinematics_yaml_1['{}{}'.format(prefix_1, name)] = kinematics_yaml_1.pop(name)
        kinematics_yaml_2 = load_yaml(file_path_2)
        kinematics_yaml_2 = kinematics_yaml_2 if kinematics_yaml_2 else {}
        if prefix_2:
            for name in list(kinematics_yaml_2.keys()):
                kinematics_yaml_2['{}{}'.format(prefix_2, name)] = kinematics_yaml_2.pop(name)
        
        kinematics_yaml = {}
        kinematics_yaml.update(kinematics_yaml_1)
        kinematics_yaml.update(kinematics_yaml_2)
        
        return yaml.dump(kinematics_yaml)
