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
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("xarm_planner_test_pose", node_options);
    RCLCPP_INFO(node->get_logger(), "xarm_planner_test_pose start");
    int dof;
    node->get_parameter_or("DOF", dof, 7);
    RCLCPP_INFO(node->get_logger(), "namespace: %s, dof: %d", node->get_namespace(), dof);

    signal(SIGINT, exit_sig_handler);

    std::string group_name = "xarm" + std::to_string(dof);
    xarm_planner::XArmPlanner planner(node, group_name);

    geometry_msgs::msg::Pose target_pose1;
    target_pose1.position.x = 0.3;
	target_pose1.position.y = -0.1;
	target_pose1.position.z = 0.2;
	target_pose1.orientation.x = 1;
	target_pose1.orientation.y = 0;
	target_pose1.orientation.z = 0;
	target_pose1.orientation.w = 0;

    geometry_msgs::msg::Pose target_pose2;
    target_pose2.position.x = 0.3;
	target_pose2.position.y = 0.1;
	target_pose2.position.z = 0.2;
	target_pose2.orientation.x = 1;
	target_pose2.orientation.y = 0;
	target_pose2.orientation.z = 0;
	target_pose2.orientation.w = 0;

    geometry_msgs::msg::Pose target_pose3;
    target_pose3.position.x = 0.3;
	target_pose3.position.y = 0.1;
	target_pose3.position.z = 0.4;
	target_pose3.orientation.x = 1;
	target_pose3.orientation.y = 0;
	target_pose3.orientation.z = 0;
	target_pose3.orientation.w = 0;

    geometry_msgs::msg::Pose target_pose4;
    target_pose4.position.x = 0.3;
	target_pose4.position.y = -0.1;
	target_pose4.position.z = 0.4;
	target_pose4.orientation.x = 1;
	target_pose4.orientation.y = 0;
	target_pose4.orientation.z = 0;
	target_pose4.orientation.w = 0;

    while (rclcpp::ok())
    {
        planner.planPoseTarget(target_pose1);
        planner.executePath();

        planner.planPoseTarget(target_pose2);
        planner.executePath();

        planner.planPoseTarget(target_pose3);
        planner.executePath();
        
        planner.planPoseTarget(target_pose4);
        planner.executePath();
    }

    RCLCPP_INFO(node->get_logger(), "xarm_planner_test_pose over");
    return 0;
}