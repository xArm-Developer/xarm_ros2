/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/
 
#include "xarm_planner/xarm_planner.h"

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[test_xarm_planner_api_joint] Ctrl-C caught, exit process...\n");
    exit(-1);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_xarm_planner_api_joint", node_options);
    RCLCPP_INFO(node->get_logger(), "test_xarm_planner_api_joint start");

    signal(SIGINT, exit_sig_handler);

    int dof;
    node->get_parameter_or("dof", dof, 7);
    std::string robot_type;
    node->get_parameter_or("robot_type", robot_type, std::string("xarm"));
    std::string group_name = robot_type;
    if (robot_type == "xarm" || robot_type == "lite")
        group_name = robot_type + std::to_string(dof);
    std::string prefix;
    node->get_parameter_or("prefix", prefix, std::string(""));
    if (prefix != "") {
        group_name = prefix + group_name;
    }

    RCLCPP_INFO(node->get_logger(), "namespace: %s, group_name: %s", node->get_namespace(), group_name.c_str());

    xarm_planner::XArmPlanner planner(node, group_name);

    std::vector<double> tar_joint1;
    std::vector<double> tar_joint2;
    std::vector<double> tar_joint3;
    
    switch (dof) {
    case 5:
        {
            tar_joint1 = {1.570796, -1.570796, -1.047198, 2.792527, -1.570796};
            tar_joint2 = {0, 0, 0, 0, 0};
            tar_joint3 = {-1.570796, -1.570796, -1.047198, -0.349066, 2.617994};
        }
        break;
    case 6:
        {
            tar_joint1 = {1.570796, -1.570796, -1.047198, 2.967060, 2.792527, -3.124139};
            tar_joint2 = {0, 0, 0, 0, 0, 0};
            tar_joint3 = {-1.570796, -1.570796, -1.047198, -2.967060, -0.349066, 3.124139};
        }
        break;
    case 7:
        {
            tar_joint1 = {1.570796, -1.570796, -1.570796, 1.396263, 2.967060, 2.792527, -1.570796};
            tar_joint2 = {0, 0, 0, 0, 0, 0, 0};
            tar_joint3 = {-1.570796, -1.570796, 1.570796, 1.396263, -2.967060, -0.349066, 2.617994};
        }
        break;
    default:
        {
            RCLCPP_INFO(node->get_logger(), "param dof error");
            exit(1);
        }
        break;
    }

    while (rclcpp::ok())
    {
        planner.planJointTarget(tar_joint2);
        planner.executePath();

        planner.planJointTarget(tar_joint1);
        planner.executePath();

        planner.planJointTarget(tar_joint2);
        planner.executePath();
        
        planner.planJointTarget(tar_joint3);
        planner.executePath();
    }

    RCLCPP_INFO(node->get_logger(), "test_xarm_planner_api_joint over");
    return 0;
}