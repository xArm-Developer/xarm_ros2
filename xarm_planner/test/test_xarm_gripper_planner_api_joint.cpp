/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/
 
#include "xarm_planner/xarm_planner.h"

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[xarm_simple_planner_test] Ctrl-C caught, exit process...\n");
    exit(-1);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_xarm_gripper_planner_api_joint", node_options);
    RCLCPP_INFO(node->get_logger(), "test_xarm_gripper_planner_api_joint start");

    std::string robot_type;
    node->get_parameter_or("robot_type", robot_type, std::string("xarm"));
    std::string group_name = robot_type + "_gripper";

    RCLCPP_INFO(node->get_logger(), "namespace: %s, group_name: %s", node->get_namespace(), group_name.c_str());

    signal(SIGINT, exit_sig_handler);

    xarm_planner::XArmPlanner planner(node, group_name);

    std::vector<double> tar_joint1(6, 0.0);  // open
    std::vector<double> tar_joint2(6, 0.85); // close

    while (rclcpp::ok())
    {
        planner.planJointTarget(tar_joint1);
        planner.executePath();

        planner.planJointTarget(tar_joint2);
        planner.executePath();
    }

    RCLCPP_INFO(node->get_logger(), "test_xarm_gripper_planner_api_joint over");
    return 0;
}