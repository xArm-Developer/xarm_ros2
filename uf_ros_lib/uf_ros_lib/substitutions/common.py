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
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions


class BaseYamlSubstitution(Substitution):
    @classmethod
    def get_var_describe(cls, var):
        return var.describe() if isinstance(var, Substitution) else var
    
    @classmethod
    def get_var_perform(cls, var, context):
        return perform_substitutions(context, normalize_to_list_of_substitutions(var)) if isinstance(var, Substitution) else var


@expose_substitution("common-yaml")
class CommonYAML(BaseYamlSubstitution):
    """Substitution that can access load common yaml file with mappings involving any subsititutable."""

    def __init__(self, file_name, package_path=None, robot_type='xarm', robot_dof=7):
        super().__init__()
        self.__file_name = file_name
        self.__package_path = package_path
        self.__robot_type = robot_type
        self.__robot_dof = robot_dof
    
    @classmethod
    def parse(cls, data):
        """Parse `CommonYAML` substitution."""
        if len(data) != 1:
            raise TypeError('CommonYAML substitution expects 1 argument')
        kwargs = {}
        kwargs["file_name"] = data[0]
        return cls, kwargs

    def describe(self):
        """Return a description of this substitution as a string."""
        return 'CommonYAML(file_name={}, package_path={}, robot_type={}, robot_dof={})'.format(
            self.get_var_describe(self.__file_name), 
            self.get_var_describe(self.__package_path),
            self.get_var_describe(self.__robot_type),
            self.get_var_describe(self.__robot_dof)
        )

    def perform(self, context):
        """
        Perform the substitution by retrieving the mappings and context.
        """
        robot_type = self.get_var_perform(self.__robot_type, context)
        robot_dof = self.get_var_perform(self.__robot_dof, context)
        filename = self.get_var_perform(self.__file_name, context)

        robot_name = '{}{}'.format(robot_type, robot_dof if robot_type == 'xarm' else '6' if robot_type == 'lite' else '')        
        file_path = self.__file_path if self.__file_path else (self.__package_path / 'config' / robot_name / filename)
        
        return yaml.dump(load_yaml(file_path))


@expose_substitution("dual-common-yaml")
class DualCommonYAML(BaseYamlSubstitution):
    """Substitution that can access load common yaml file with mappings involving any subsititutable."""

    def __init__(self, file_name, package_path=None, robot_type_1='xarm', robot_type_2='xarm', robot_dof_1=7, robot_dof_2=7):
        super().__init__()
        self.__file_name = file_name
        self.__package_path = package_path
        self.__robot_type_1 = robot_type_1
        self.__robot_type_2 = robot_type_2
        self.__robot_dof_1 = robot_dof_1
        self.__robot_dof_2 = robot_dof_2
    
    @classmethod
    def parse(cls, data):
        """Parse `DualCommonYAML` substitution."""
        if len(data) != 1:
            raise TypeError('DualCommonYAML substitution expects 1 argument')
        kwargs = {}
        kwargs["file_name"] = data[0]
        return cls, kwargs

    def describe(self):
        """Return a description of this substitution as a string."""
        return 'DualCommonYAML(file_name={}, package_path={}, robot_type_1={}, robot_type_2={}, robot_dof_1={}, robot_dof_2={})'.format(
            self.get_var_describe(self.__file_name), 
            self.get_var_describe(self.__package_path),
            self.get_var_describe(self.__robot_type_1),
            self.get_var_describe(self.__robot_type_2),
            self.get_var_describe(self.__robot_dof_1),
            self.get_var_describe(self.__robot_dof_2)
        )

    def perform(self, context):
        """
        Perform the substitution by retrieving the mappings and context.
        """
        robot_type_1 = self.get_var_perform(self.__robot_type_1, context)
        robot_type_2 = self.get_var_perform(self.__robot_type_2, context)
        robot_dof_1 = self.get_var_perform(self.__robot_dof_1, context)
        robot_dof_2 = self.get_var_perform(self.__robot_dof_2, context)
        filename = self.get_var_perform(self.__file_name, context)

        robot_name = '{}{}'.format(robot_type_1, robot_dof_1 if robot_type_1 == 'xarm' else '6' if robot_type_1 == 'lite' else '')        
        robot_name = '{}{}'.format(robot_type_2, robot_dof_2 if robot_type_2 == 'xarm' else '6' if robot_type_2 == 'lite' else '')        
        file_path_1 = self.__file_path if self.__file_path else (self.__package_path / 'config' / robot_name_1 / filename)
        file_path_2 = self.__file_path if self.__file_path else (self.__package_path / 'config' / robot_name_2 / filename)
        
        yaml_1 = load_yaml(file_path_1)
        yaml_2 = load_yaml(file_path_2)
        yaml_1.update(yaml_2)

        return yaml.dump(yaml_1)