/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/

/*
 * 本头文件相对官方原版的修改：
 *   新增两个成员 display_planned_path_pub_ / display_planned_path_pub2_，
 *   用于向 RViz 重新发布限速后的笛卡尔轨迹（详见 xarm_planner.cpp
 *   文件头部的“修改说明”，修改 3）。
 *   其余接口签名与官方完全一致，不影响已有调用方。
 */

#ifndef __XARM_PLANNER_H__
#define __XARM_PLANNER_H__

#include <signal.h>
#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
// #include <moveit_visual_tools/moveit_visual_tools.h>

#include <std_msgs/msg/bool.hpp>
#include <xarm_msgs/srv/plan_pose.hpp>
#include <xarm_msgs/srv/plan_joint.hpp>
#include <xarm_msgs/srv/plan_exec.hpp>
#include <xarm_msgs/srv/plan_single_straight.hpp>


namespace xarm_planner
{
    class XArmPlanner
    {
    public:
        XArmPlanner(const rclcpp::Node::SharedPtr& node, const std::string& group_name);
        XArmPlanner(const std::string& group_name);
        ~XArmPlanner() {};

        bool planJointTarget(const std::vector<double>& joint_target);
        bool planPoseTarget(const geometry_msgs::msg::Pose& pose_target);
        bool planPoseTargets(const std::vector<geometry_msgs::msg::Pose>& pose_target_vector);
        bool planCartesianPath(const std::vector<geometry_msgs::msg::Pose>& pose_target_vector);

        bool executePath(bool wait = true);
    private:
        void init(const std::string& group_name);

        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
        // 【修改 3】RViz 轨迹显示发布器（官方原版没有这两个成员）
        // 笛卡尔轨迹时间参数化完成后，把带时序的轨迹重新发布给 RViz，
        // 解决笛卡尔轨迹在 RViz 中一闪而过的问题。两个话题分别对应
        // RViz 中 MotionPlanning 和 Trajectory 两个显示组件（见 xarm_planner.cpp init()）。
        rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_planned_path_pub_;
        rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_planned_path_pub2_;
        moveit::planning_interface::MoveGroupInterface::Plan xarm_plan_;
        moveit_msgs::msg::RobotTrajectory trajectory_;
        bool is_trajectory_;
    };
}

#endif // __XARM_PLANNER_H__
