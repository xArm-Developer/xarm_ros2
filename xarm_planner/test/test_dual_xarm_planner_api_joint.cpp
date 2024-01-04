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

    int dof_1, dof_2;
    node->get_parameter_or("dof_1", dof_1, 7);
    node->get_parameter_or("dof_2", dof_2, 7);
    std::string robot_type_1;
    std::string robot_type_2;
    std::string prefix_1;
    std::string prefix_2;
    node->get_parameter_or("robot_type_1", robot_type_1, std::string("xarm"));
    node->get_parameter_or("robot_type_2", robot_type_2, std::string("xarm"));
    node->get_parameter_or("prefix_1", prefix_1, std::string("L_"));
    node->get_parameter_or("prefix_2", prefix_2, std::string("R_"));
    std::string group_name_L = robot_type_1;
    std::string group_name_R = robot_type_2;
    if (robot_type_1 == "xarm" || robot_type_1 == "lite") {
        group_name_L = robot_type_1 + std::to_string(dof_1);
    }
    if (robot_type_2 == "xarm" || robot_type_2 == "lite") {
        group_name_R = robot_type_2 + std::to_string(dof_2);
    }
    group_name_L = prefix_1 + group_name_L;
    group_name_R = prefix_2 + group_name_R;
    

    RCLCPP_INFO(node->get_logger(), "namespace: %s, group_name_L: %s, group_name_R: %s", node->get_namespace(), group_name_L.c_str(), group_name_R.c_str());

    xarm_planner::XArmPlanner planner_L(node, group_name_L);
    xarm_planner::XArmPlanner planner_R(node, group_name_R);

    std::vector<double> tar_joint1_L, tar_joint1_R;
    std::vector<double> tar_joint2_L, tar_joint2_R;
    std::vector<double> tar_joint3_L, tar_joint3_R;
    
    switch (dof_1) {
        case 5:
        {
            tar_joint1_L = {1.570796, -1.570796, -1.047198, 2.792527, -1.570796};
            tar_joint2_L = {0, 0, 0, 0, 0};
            tar_joint3_L = {-1.570796, -1.570796, -1.047198, -0.349066, 2.617994};
            break;
        }
        case 6:
        {
            tar_joint1_L = {1.570796, -1.570796, -1.047198, 2.967060, 2.792527, -3.124139};
            tar_joint2_L = {0, 0, 0, 0, 0, 0};
            tar_joint3_L = {-1.570796, -1.570796, -1.047198, -2.967060, -0.349066, 3.124139};
            break;
        }
        case 7:
        {
            tar_joint1_L = {1.570796, -1.570796, -1.570796, 1.396263, 2.967060, 2.792527, -1.570796};
            tar_joint2_L = {0, 0, 0, 0, 0, 0, 0};
            tar_joint3_L = {-1.570796, -1.570796, 1.570796, 1.396263, -2.967060, -0.349066, 2.617994};
            break;
        }
        default:
        {
            RCLCPP_INFO(node->get_logger(), "param dof error");
            exit(1);
        }
    }
    switch (dof_2) {
        case 5:
        {
            tar_joint1_R = {1.570796, -1.570796, -1.047198, 2.792527, -1.570796};
            tar_joint2_R = {0, 0, 0, 0, 0};
            tar_joint3_R = {-1.570796, -1.570796, -1.047198, -0.349066, 2.617994};
            break;
        }
        case 6:
        {
            tar_joint1_R = {1.570796, -1.570796, -1.047198, 2.967060, 2.792527, -3.124139};
            tar_joint2_R = {0, 0, 0, 0, 0, 0};
            tar_joint3_R = {-1.570796, -1.570796, -1.047198, -2.967060, -0.349066, 3.124139};
            break;
        }
        case 7:
        {
            tar_joint1_R = {1.570796, -1.570796, -1.570796, 1.396263, 2.967060, 2.792527, -1.570796};
            tar_joint2_R = {0, 0, 0, 0, 0, 0, 0};
            tar_joint3_R = {-1.570796, -1.570796, 1.570796, 1.396263, -2.967060, -0.349066, 2.617994};
            break;
        }
        default:
        {
            RCLCPP_INFO(node->get_logger(), "param dof error");
            exit(1);
        }
    }

    int cnt = 0;
    while (rclcpp::ok())
    {
        cnt++;
        planner_L.planJointTarget(tar_joint2_L);
        planner_R.planJointTarget(tar_joint2_R);
        RCLCPP_INFO(node->get_logger(), "L execute, %d", cnt);
        planner_L.executePath(false);
        RCLCPP_INFO(node->get_logger(), "R execute, %d", cnt);
        planner_R.executePath(false);

        planner_L.planJointTarget(tar_joint1_L);
        planner_R.planJointTarget(tar_joint1_R);
        RCLCPP_INFO(node->get_logger(), "L execute, %d", cnt);
        planner_L.executePath(false);
        RCLCPP_INFO(node->get_logger(), "R execute, %d", cnt);
        planner_R.executePath(false);

        planner_L.planJointTarget(tar_joint2_L);
        planner_R.planJointTarget(tar_joint2_R);
        RCLCPP_INFO(node->get_logger(), "L execute, %d", cnt);
        planner_L.executePath(false);
        RCLCPP_INFO(node->get_logger(), "R execute, %d", cnt);
        planner_R.executePath(false);
        
        planner_L.planJointTarget(tar_joint3_L);
        planner_R.planJointTarget(tar_joint3_R);
        RCLCPP_INFO(node->get_logger(), "L execute, %d", cnt);
        planner_L.executePath(false);
        RCLCPP_INFO(node->get_logger(), "R execute, %d", cnt);
        planner_R.executePath(false);
    }

    RCLCPP_INFO(node->get_logger(), "test_xarm_planner_api_joint over");
    return 0;
}