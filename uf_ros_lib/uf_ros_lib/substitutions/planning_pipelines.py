#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2024, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 
import os
import re
import yaml
from launch.frontend import expose_substitution
from launch.substitution import Substitution
from launch_param_builder import load_yaml
from .common import BaseYamlSubstitution


def get_pattern_matches(folder, pattern):
    """Given all the files in the folder, find those that match the pattern.

    If there are groups defined, the groups are returned. Otherwise the path to the matches are returned.
    """
    matches = []
    if not folder.exists():
        return matches
    for child in folder.iterdir():
        if not child.is_file():
            continue
        m = pattern.search(child.name)
        if m:
            groups = m.groups()
            if groups:
                matches.append(groups[0])
            else:
                matches.append(child)
    return matches


@expose_substitution("planning-pipelines")
class PlanningPipelinesYAML(BaseYamlSubstitution):
    """Substitution that can access load planning_pipelines file with mappings involving any subsititutable."""

    def __init__(self, pipeline, package_path=None, config_folder=None,
        prefix='', robot_type='xarm', robot_dof=7, 
        add_gripper=False, add_bio_gripper=False):
        super().__init__()
        self.__pipeline = pipeline
        self.__package_path = package_path
        self.__config_folder = config_folder
        self.__prefix = prefix
        self.__robot_type = robot_type
        self.__robot_dof = robot_dof
        self.__add_gripper = add_gripper
        self.__add_bio_gripper = add_bio_gripper

    @classmethod
    def parse(cls, data):
        """Parse `PlanningPipelinesYAML` substitution."""
        if len(data) != 1:
            raise TypeError('PlanningPipelinesYAML substitution expects 1 argument')
        kwargs = {}
        kwargs["pipeline"] = data[0]
        return cls, kwargs

    def describe(self):
        """Return a description of this substitution as a string."""
        return 'PlanningPipelinesYAML({}, package_path={}, prefix={}, robot_type={}, robot_dof={}, add_gripper={}, add_bio_gripper={})'.format(
            self.get_var_describe(self.__pipeline), 
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
        pipeline = self.get_var_perform(self.__pipeline, context)
        prefix = self.get_var_perform(self.__prefix, context)
        robot_type = self.get_var_perform(self.__robot_type, context)
        robot_dof = self.get_var_perform(self.__robot_dof, context)
        add_gripper = self.get_var_perform(self.__add_gripper, context).lower() == 'true'
        add_bio_gripper = self.get_var_perform(self.__add_bio_gripper, context).lower() == 'true'

        robot_name = '{}{}'.format(robot_type, robot_dof if robot_type == 'xarm' else '6' if robot_type == 'lite' else '')

        filename = pipeline + '_planning.yaml'
        parameter_file = self.__package_path / 'config' / 'moveit_configs' / filename
        if parameter_file.exists():
            planning_yaml = load_yaml(parameter_file)
            planning_yaml = planning_yaml if planning_yaml else {}
        else:
            planning_yaml = {}

        if self.__config_folder is None:
            parameter_file = self.__package_path / 'config' / robot_name / filename
        else:
            parameter_file = self.__package_path / self.__config_folder / filename
        pipeline_planning_yaml = load_yaml(parameter_file)
        pipeline_planning_yaml = pipeline_planning_yaml if pipeline_planning_yaml else {}

        if robot_type != 'lite' and add_gripper:
            parameter_file = self.__package_path / 'config' / '{}_gripper'.format(robot_type) / filename
            if parameter_file.exists():
                gripper_planning_yaml = load_yaml(parameter_file)
                if gripper_planning_yaml:
                    pipeline_planning_yaml.update(gripper_planning_yaml)
        elif robot_type != 'lite' and add_bio_gripper:
            parameter_file = self.__package_path / 'config' / 'bio_gripper' / filename
            if parameter_file.exists():
                gripper_planning_yaml = load_yaml(parameter_file)
                if gripper_planning_yaml:
                    pipeline_planning_yaml.update(gripper_planning_yaml)
        if pipeline_planning_yaml and prefix:
            for name in list(pipeline_planning_yaml.keys()):
                if pipeline == 'ompl' and name != 'planner_configs' and name not in planning_yaml:
                    pipeline_planning_yaml['{}{}'.format(prefix, name)] = pipeline_planning_yaml.pop(name)
        planning_yaml.update(pipeline_planning_yaml)
        if pipeline == 'ompl' and 'planner_configs' not in planning_yaml:
            parameter_file = self.__package_path / 'config' / 'moveit_configs' / 'ompl_defaults.yaml'
            planning_yaml.update(load_yaml(parameter_file))
        # if pipeline == 'ompl':
        #     if os.environ.get('ROS_DISTRO', '') > 'iron':
        #         planning_yaml.update({
        #             'planning_plugins': ['ompl_interface/OMPLPlanner'],
        #             'request_adapters': [
        #                 'default_planning_request_adapters/ResolveConstraintFrames',
        #                 'default_planning_request_adapters/ValidateWorkspaceBounds',
        #                 'default_planning_request_adapters/CheckStartStateBounds',
        #                 'default_planning_request_adapters/CheckStartStateCollision',
        #             ],
        #             'response_adapters': [
        #                 'default_planning_response_adapters/AddTimeOptimalParameterization',
        #                 'default_planning_response_adapters/ValidateSolution',
        #                 'default_planning_response_adapters/DisplayMotionPath',
        #             ],
        #         })
        #     else:
        #         planning_yaml.update({
        #             'planning_plugin': 'ompl_interface/OMPLPlanner',
        #             'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
        #             'start_state_max_bounds_error': 0.1,
        #         })

        return yaml.dump(planning_yaml)


@expose_substitution("dual-planning-pipelines")
class DualPlanningPipelinesYAML(BaseYamlSubstitution):
    """Substitution that can access load planning_pipelines file with mappings involving any subsititutable."""

    def __init__(self, pipeline, package_path=None, 
        prefix_1='L_', prefix_2='R_',
        robot_type_1='xarm', robot_type_2='xarm',
        robot_dof_1=7, robot_dof_2=7,
        add_gripper_1=False, add_gripper_2=False, 
        add_bio_gripper_1=False, add_bio_gripper_2=False):
        super().__init__()
        self.__pipeline = pipeline
        self.__package_path = package_path        
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
        """Parse `DualPlanningPipelinesYAML` substitution."""
        if len(data) != 1:
            raise TypeError('DualPlanningPipelinesYAML substitution expects 1 argument')
        kwargs = {}
        kwargs["pipeline"] = data[0]
        return cls, kwargs

    def describe(self):
        """Return a description of this substitution as a string."""
        return 'DualPlanningPipelinesYAML(pipeline={}, package_path={}, prefix_1={}, prefix_2={}, robot_type_1={}, robot_type_2={}, robot_dof_1={}, robot_dof_2={}, add_gripper_1={}, add_gripper_2={}, add_bio_gripper_1={}, add_bio_gripper_2={})'.format(
            self.get_var_describe(self.__pipeline),
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
        pipeline = self.get_var_perform(self.__pipeline, context)
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

        filename = pipeline + '_planning.yaml'
        file_path_1 = self.__package_path / 'config' / robot_name_1 / filename
        file_path_2 = self.__package_path / 'config' / robot_name_2 / filename

        parameter_file = self.__package_path / 'config' / 'moveit_configs' / filename
        if parameter_file.exists():
            planning_yaml = load_yaml(parameter_file)
            planning_yaml = planning_yaml if planning_yaml else {}
        else:
            planning_yaml = {}

        # if pipeline == 'ompl':
        #     if os.environ.get('ROS_DISTRO', '') > 'iron':
        #         planning_yaml = {
        #             'planning_plugins': ['ompl_interface/OMPLPlanner'],
        #             'request_adapters': [
        #                 'default_planning_request_adapters/ResolveConstraintFrames',
        #                 'default_planning_request_adapters/ValidateWorkspaceBounds',
        #                 'default_planning_request_adapters/CheckStartStateBounds',
        #                 'default_planning_request_adapters/CheckStartStateCollision',
        #             ],
        #             'response_adapters': [
        #                 'default_planning_response_adapters/AddTimeOptimalParameterization',
        #                 'default_planning_response_adapters/ValidateSolution',
        #                 'default_planning_response_adapters/DisplayMotionPath',
        #             ],
        #         }
        #     else:
        #         planning_yaml = {
        #             'planning_plugin': 'ompl_interface/OMPLPlanner',
        #             'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
        #             'start_state_max_bounds_error': 0.1,
        #         }
        # else:
        #     planning_yaml = {}

        planning_yaml_1 = load_yaml(file_path_1)
        planning_yaml_1 = planning_yaml_1 if planning_yaml_1 else {}
        if robot_type_1 != 'lite' and add_gripper_1:
            parameter_file = self.__package_path / 'config' / '{}_gripper'.format(robot_type_1) / filename
            if parameter_file.exists():
                gripper_planning_yaml = load_yaml(parameter_file)
                if gripper_planning_yaml:
                    planning_yaml_1.update(gripper_planning_yaml)
        elif robot_type_1 != 'lite' and add_bio_gripper_1:
            parameter_file = self.__package_path / 'config' / 'bio_gripper' / filename
            if parameter_file.exists():
                gripper_planning_yaml = load_yaml(parameter_file)
                if gripper_planning_yaml:
                    planning_yaml_1.update(gripper_planning_yaml)
        if planning_yaml_1 and prefix_1:
            for name in list(planning_yaml_1.keys()):
                if pipeline == 'ompl' and name != 'planner_configs' and name not in planning_yaml:
                    planning_yaml_1['{}{}'.format(prefix_1, name)] = planning_yaml_1.pop(name)
        
        planning_yaml_2 = load_yaml(file_path_2)
        planning_yaml_2 = planning_yaml_2 if planning_yaml_2 else {}
        if robot_type_2 != 'lite' and add_gripper_2:
            parameter_file = self.__package_path / 'config' / '{}_gripper'.format(robot_type_2) / filename
            if parameter_file.exists():
                gripper_planning_yaml = load_yaml(parameter_file)
                if gripper_planning_yaml:
                    planning_yaml_2.update(gripper_planning_yaml)
        elif robot_type_2 != 'lite' and add_bio_gripper_2:
            parameter_file = self.__package_path / 'config' / 'bio_gripper' / filename
            if parameter_file.exists():
                gripper_planning_yaml = load_yaml(parameter_file)
                if gripper_planning_yaml:
                    planning_yaml_2.update(gripper_planning_yaml)
        if planning_yaml_2 and prefix_2:
            for name in list(planning_yaml_2.keys()):
                if pipeline == 'ompl' and name != 'planner_configs' and name not in planning_yaml:
                    planning_yaml_2['{}{}'.format(prefix_2, name)] = planning_yaml_2.pop(name)

        planning_yaml.update(planning_yaml_1)
        planning_yaml.update(planning_yaml_2)
        if pipeline == 'ompl' and 'planner_configs' not in planning_yaml:
            parameter_file = self.__package_path / 'config' / 'moveit_configs' / 'ompl_defaults.yaml'
            planning_yaml.update(load_yaml(parameter_file))

        return yaml.dump(planning_yaml)
