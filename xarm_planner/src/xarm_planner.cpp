/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/
 
#include "xarm_planner/xarm_planner.h"

namespace xarm_planner
{
const double jump_threshold = 0.0;
const double eef_step = 0.005;
const double maxV_scale_factor = 0.3;

XArmPlanner::XArmPlanner(const rclcpp::Node::SharedPtr& node, const std::string& group_name)
    : node_(node)
{
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name);
    // rclcpp::executors::SingleThreadedExecutor executor;
    // executor.add_node(node_);
    // std::thread([&executor]() { executor.spin(); }).detach();
    RCLCPP_INFO(node_->get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(node_->get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());
    RCLCPP_INFO(node_->get_logger(), "Available Planning Groups:");
    std::copy(move_group_->getJointModelGroupNames().begin(), move_group_->getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
}

bool XArmPlanner::planJointTarget(const std::vector<double>& joint_target)
{
    bool success = move_group_->setJointValueTarget(joint_target);
    if (!success)
        RCLCPP_WARN(node_->get_logger(), "setJointValueTarget: out of bounds");
    success = (move_group_->plan(xarm_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
        RCLCPP_ERROR(node_->get_logger(), "planJointTarget: plan failed");
    return success;
}

bool XArmPlanner::planPoseTarget(const geometry_msgs::msg::Pose& pose_target)
{
    bool success = move_group_->setPoseTarget(pose_target);
    if (!success)
        RCLCPP_WARN(node_->get_logger(), "setPoseTarget: out of bounds");
    success = (move_group_->plan(xarm_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
        RCLCPP_ERROR(node_->get_logger(), "planPoseTarget: plan failed");
    return success;
}

bool XArmPlanner::planPoseTargets(const std::vector<geometry_msgs::msg::Pose>& pose_target_vector)
{
    bool success = move_group_->setPoseTargets(pose_target_vector);
    if (!success)
        RCLCPP_WARN(node_->get_logger(), "setPoseTargets: out of bounds");
    success = (move_group_->plan(xarm_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
        RCLCPP_ERROR(node_->get_logger(), "planPoseTargets: plan failed");
    return success;
}

bool XArmPlanner::planCartesianPath(const std::vector<geometry_msgs::msg::Pose>& pose_target_vector)
{   
    move_group_->setMaxVelocityScalingFactor(maxV_scale_factor);
    moveit_msgs::msg::RobotTrajectory trajectory;
    
    double fraction = move_group_->computeCartesianPath(pose_target_vector, eef_step, jump_threshold, trajectory);
    bool success = true;
    if(fraction < 0.9) {
        RCLCPP_ERROR(node_->get_logger(), "planCartesianPath: plan failed, fraction=%lf", fraction);
        return false;
    }
    xarm_plan_.trajectory_ = trajectory;
    return true;
}

bool XArmPlanner::executePath(bool wait)
{
    moveit::planning_interface::MoveItErrorCode code;
    if (wait)
        code =  move_group_->execute(xarm_plan_);
    else
        code =  move_group_->asyncExecute(xarm_plan_);
    bool success = (code == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
        RCLCPP_ERROR(node_->get_logger(), "executePath: execute failed, wait=%d, MoveItErrorCode=%d", wait, code);
    return success;
}
}
