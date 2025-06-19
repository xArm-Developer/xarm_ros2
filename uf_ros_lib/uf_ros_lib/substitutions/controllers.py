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

@expose_substitution("controllers")
class ControllersYAML(BaseYamlSubstitution):
    """Substitution that can access load controllers file with mappings involving any subsititutable."""

    def __init__(self, file_path, package_path=None, 
        prefix='', robot_type='xarm', robot_dof=7, 
        add_gripper=False, add_bio_gripper=False, 
        controllers_name=''):
        super().__init__()
        self.__file_path = file_path
        self.__package_path = package_path
        
        self.__prefix = prefix
        self.__robot_type = robot_type
        self.__robot_dof = robot_dof
        self.__add_gripper = add_gripper
        self.__add_bio_gripper = add_bio_gripper
        self.__controllers_name = controllers_name

    @classmethod
    def parse(cls, data):
        """Parse `ControllersYAML` substitution."""
        if len(data) != 1:
            raise TypeError('ControllersYAML substitution expects 1 argument')
        kwargs = {}
        kwargs["file_path"] = data[0]
        return cls, kwargs

    def describe(self):
        """Return a description of this substitution as a string."""
        return 'ControllersYAML(file_path={}, package_path={}, prefix={}, robot_type={}, add_gripper={}, add_bio_gripper={}, controllers_name={})'.format(
            self.get_var_describe(self.__file_path),
            self.get_var_describe(self.__package_path),
            self.get_var_describe(self.__prefix),
            self.get_var_describe(self.__robot_type),
            self.get_var_describe(self.__robot_dof),
            self.get_var_describe(self.__add_gripper),
            self.get_var_describe(self.__add_bio_gripper),
            self.get_var_describe(self.__controllers_name)
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
        controllers_name = self.get_var_perform(self.__controllers_name, context)
   
        robot_name = '{}{}'.format(robot_type, robot_dof if robot_type == 'xarm' else '6' if robot_type == 'lite' else '')

        controllers_name = controllers_name if controllers_name.endswith('.yaml') else '{}.yaml'.format(controllers_name)
        file_path = self.__file_path if self.__file_path else (self.__package_path / 'config' / robot_name / controllers_name)

        controllers_yaml = load_yaml(file_path)
        controllers_yaml = controllers_yaml if controllers_yaml else {}

        if robot_type != 'lite' and add_gripper:
            gripper_controllers_yaml = load_yaml(self.__package_path / 'config' / '{}_gripper'.format(robot_type) / controllers_name)
            if gripper_controllers_yaml:
                for name in gripper_controllers_yaml['controller_names']:
                    if name in gripper_controllers_yaml:
                        if name not in controllers_yaml['controller_names']:
                            controllers_yaml['controller_names'].append(name)
                        controllers_yaml[name] = gripper_controllers_yaml[name]
        elif robot_type != 'lite' and add_bio_gripper:
            gripper_controllers_yaml = load_yaml(self.__package_path / 'config' / 'bio_gripper' / controllers_name)
            if gripper_controllers_yaml:
                for name in gripper_controllers_yaml['controller_names']:
                    if name in gripper_controllers_yaml:
                        if name not in controllers_yaml['controller_names']:
                            controllers_yaml['controller_names'].append(name)
                        controllers_yaml[name] = gripper_controllers_yaml[name]
        if controllers_yaml and prefix:
            for i, name in enumerate(controllers_yaml['controller_names']):
                joints = controllers_yaml.get(name, {}).get('joints', [])
                for j, joint in enumerate(joints):
                    joints[j] = '{}{}'.format(prefix, joint)
                controllers_yaml['controller_names'][i] = '{}{}'.format(prefix, name)
                if name in controllers_yaml:
                    controllers_yaml['{}{}'.format(prefix, name)] = controllers_yaml.pop(name)
        
        return yaml.dump(controllers_yaml)


@expose_substitution("dual-controllers")
class DualControllersYAML(BaseYamlSubstitution):
    """Substitution that can access load controllers file with mappings involving any subsititutable."""

    def __init__(self, file_path, package_path=None, 
        prefix_1='L_', prefix_2='R_',
        robot_type_1='xarm', robot_type_2='xarm',
        robot_dof_1=7, robot_dof_2=7,
        add_gripper_1=False, add_gripper_2=False, 
        add_bio_gripper_1=False, add_bio_gripper_2=False, 
        controllers_name=''):
        super().__init__()
        self.__file_path = file_path
        self.__package_path = package_path
        self.__controllers_name = controllers_name
        
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
        """Parse `DualControllersYAML` substitution."""
        if len(data) != 1:
            raise TypeError('DualControllersYAML substitution expects 1 argument')
        kwargs = {}
        kwargs["file_path"] = data[0]
        return cls, kwargs

    def describe(self):
        """Return a description of this substitution as a string."""
        return 'DualControllersYAML(file_path={}, package_path={}, prefix_1={}, prefix_2={}, robot_type_1={}, robot_type_2={}, robot_dof_1={}, robot_dof_2={}, add_gripper_1={}, add_gripper_2={}, add_bio_gripper_1={}, add_bio_gripper_2={}, controllers_name={})'.format(
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
            self.get_var_describe(self.__add_bio_gripper_2),
            self.get_var_describe(self.__controllers_name)
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
        controllers_name = self.get_var_perform(self.__controllers_name, context)
   
        robot_name_1 = '{}{}'.format(robot_type_1, robot_dof_1 if robot_type_1 == 'xarm' else '6' if robot_type_1 == 'lite' else '')
        robot_name_2 = '{}{}'.format(robot_type_2, robot_dof_2 if robot_type_2 == 'xarm' else '6' if robot_type_2 == 'lite' else '')

        controllers_name = controllers_name if controllers_name.endswith('.yaml') else '{}.yaml'.format(controllers_name)
        file_path_1 = self.__file_path if self.__file_path else (self.__package_path / 'config' / robot_name_1 / controllers_name)
        file_path_2 = self.__file_path if self.__file_path else (self.__package_path / 'config' / robot_name_2 / controllers_name)

        controllers_yaml_1 = load_yaml(file_path_1)
        controllers_yaml_1 = controllers_yaml_1 if controllers_yaml_1 else {}

        if robot_type_1 != 'lite' and add_gripper_1:
            gripper_controllers_yaml = load_yaml(self.__package_path / 'config' / '{}_gripper'.format(robot_type_1) / controllers_name)
            if gripper_controllers_yaml:
                for name in gripper_controllers_yaml['controller_names']:
                    if name in gripper_controllers_yaml:
                        if name not in controllers_yaml_1['controller_names']:
                            controllers_yaml_1['controller_names'].append(name)
                        controllers_yaml_1[name] = gripper_controllers_yaml[name]
        elif robot_type_1 != 'lite' and add_bio_gripper_1:
            gripper_controllers_yaml = load_yaml(self.__package_path / 'config' / 'bio_gripper' / controllers_name)
            if gripper_controllers_yaml:
                for name in gripper_controllers_yaml['controller_names']:
                    if name in gripper_controllers_yaml:
                        if name not in controllers_yaml_1['controller_names']:
                            controllers_yaml_1['controller_names'].append(name)
                        controllers_yaml_1[name] = gripper_controllers_yaml[name]
        if controllers_yaml_1 and prefix_1:
            for i, name in enumerate(controllers_yaml_1['controller_names']):
                joints = controllers_yaml_1.get(name, {}).get('joints', [])
                for j, joint in enumerate(joints):
                    joints[j] = '{}{}'.format(prefix_1, joint)
                controllers_yaml_1['controller_names'][i] = '{}{}'.format(prefix_1, name)
                if name in controllers_yaml_1:
                    controllers_yaml_1['{}{}'.format(prefix_1, name)] = controllers_yaml_1.pop(name)
        
        controllers_yaml_2 = load_yaml(file_path_2)
        controllers_yaml_2 = controllers_yaml_2 if controllers_yaml_2 else {}

        if robot_type_2 != 'lite' and add_gripper_2:
            gripper_controllers_yaml = load_yaml(self.__package_path / 'config' / '{}_gripper'.format(robot_type_1) / controllers_name)
            if gripper_controllers_yaml:
                for name in gripper_controllers_yaml['controller_names']:
                    if name in gripper_controllers_yaml:
                        if name not in controllers_yaml_2['controller_names']:
                            controllers_yaml_2['controller_names'].append(name)
                        controllers_yaml_2[name] = gripper_controllers_yaml[name]
        elif robot_type_2 != 'lite' and add_bio_gripper_2:
            gripper_controllers_yaml = load_yaml(self.__package_path / 'config' / 'bio_gripper' / controllers_name)
            if gripper_controllers_yaml:
                for name in gripper_controllers_yaml['controller_names']:
                    if name in gripper_controllers_yaml:
                        if name not in controllers_yaml_2['controller_names']:
                            controllers_yaml_2['controller_names'].append(name)
                        controllers_yaml_2[name] = gripper_controllers_yaml[name]
        if controllers_yaml_2 and prefix_2:
            for i, name in enumerate(controllers_yaml_2['controller_names']):
                joints = controllers_yaml_2.get(name, {}).get('joints', [])
                for j, joint in enumerate(joints):
                    joints[j] = '{}{}'.format(prefix_2, joint)
                controllers_yaml_2['controller_names'][i] = '{}{}'.format(prefix_2, name)
                if name in controllers_yaml_2:
                    controllers_yaml_2['{}{}'.format(prefix_2, name)] = controllers_yaml_2.pop(name)
        
        controllers_yaml = {}
        controllers_yaml.update(controllers_yaml_1)
        controllers_yaml['controller_names'].extend(controllers_yaml_2['controller_names'])
        controllers_yaml_2.pop('controller_names', [])
        controllers_yaml.update(controllers_yaml_2)
        
        return yaml.dump(controllers_yaml)
