/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include "xarm_planner/xarm_planner.h"
#include <std_msgs/msg/bool.hpp>
#include <xarm_msgs/srv/plan_pose.hpp>
#include <xarm_msgs/srv/plan_joint.hpp>
#include <xarm_msgs/srv/plan_exec.hpp>
#include <xarm_msgs/srv/plan_single_straight.hpp>

#define BIND_CLS_CB(func) std::bind(func, this, std::placeholders::_1, std::placeholders::_2)

class XArmPlannerRunner
{
public:
    XArmPlannerRunner(rclcpp::Node::SharedPtr& node);
    ~XArmPlannerRunner() {};

private:
    bool do_joint_plan(const std::shared_ptr<xarm_msgs::srv::PlanJoint::Request> req, std::shared_ptr<xarm_msgs::srv::PlanJoint::Response> res);
    bool exec_plan_cb(const std::shared_ptr<xarm_msgs::srv::PlanExec::Request> req, std::shared_ptr<xarm_msgs::srv::PlanExec::Response> res);
    
private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<xarm_planner::XArmPlanner> xarm_planner_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr exec_plan_sub_;

    rclcpp::Service<xarm_msgs::srv::PlanExec>::SharedPtr exec_plan_server_;
    rclcpp::Service<xarm_msgs::srv::PlanJoint>::SharedPtr joint_plan_server_;
};

XArmPlannerRunner::XArmPlannerRunner(rclcpp::Node::SharedPtr& node)
    : node_(node)
{
    std::string group_name;   
    node_->get_parameter_or("PLANNING_GROUP", group_name, std::string("xarm_gripper"));

    RCLCPP_INFO(node_->get_logger(), "namespace: %s, group_name: %s", node->get_namespace(), group_name.c_str());

    xarm_planner_ = std::make_shared<xarm_planner::XArmPlanner>(group_name);

    exec_plan_server_ = node_->create_service<xarm_msgs::srv::PlanExec>("xarm_gripper_exec_plan", BIND_CLS_CB(&XArmPlannerRunner::exec_plan_cb));
    joint_plan_server_ = node_->create_service<xarm_msgs::srv::PlanJoint>("xarm_gripper_joint_plan", BIND_CLS_CB(&XArmPlannerRunner::do_joint_plan));
}

bool XArmPlannerRunner::do_joint_plan(const std::shared_ptr<xarm_msgs::srv::PlanJoint::Request> req, std::shared_ptr<xarm_msgs::srv::PlanJoint::Response> res)
{
    bool success = xarm_planner_->planJointTarget(req->target);
    res->success = success;
    return success;
}

bool XArmPlannerRunner::exec_plan_cb(const std::shared_ptr<xarm_msgs::srv::PlanExec::Request> req, std::shared_ptr<xarm_msgs::srv::PlanExec::Response> res)
{
    bool success = xarm_planner_->executePath(req->wait);
    res->success = success;
    return success;
}

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[xarm_gripper_planner_node] Ctrl-C caught, exit process...\n");
    exit(-1);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("xarm_gripper_planner_node", node_options);
    RCLCPP_INFO(node->get_logger(), "xarm_gripper_planner_node start");
    signal(SIGINT, exit_sig_handler);

    XArmPlannerRunner xarm_planner_runner(node);
    
    rclcpp::spin(node);
    rclcpp::shutdown();

    RCLCPP_INFO(node->get_logger(), "xarm_gripper_planner_node over");
    return 0;
}