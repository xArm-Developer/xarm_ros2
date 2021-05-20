/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/
 
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

#define BIND_CLS_CB(func) std::bind(func, this, std::placeholders::_1, std::placeholders::_2)

const double jump_threshold = 0.0;
const double eef_step = 0.005;
const double maxV_scale_factor = 0.3; // check!!

// namespace rvt = rviz_visual_tools;

class XArmSimplePlanner
{
public:
    XArmSimplePlanner(rclcpp::Node::SharedPtr& node);
     ~XArmSimplePlanner() {};

private:
    void init();
    bool do_pose_plan(const std::shared_ptr<xarm_msgs::srv::PlanPose::Request> req, std::shared_ptr<xarm_msgs::srv::PlanPose::Response> res);
    bool do_joint_plan(const std::shared_ptr<xarm_msgs::srv::PlanJoint::Request> req, std::shared_ptr<xarm_msgs::srv::PlanJoint::Response> res);
    bool do_single_cartesian_plan(const std::shared_ptr<xarm_msgs::srv::PlanSingleStraight::Request> req, std::shared_ptr<xarm_msgs::srv::PlanSingleStraight::Response> res);
    bool exec_plan_cb(const std::shared_ptr<xarm_msgs::srv::PlanExec::Request> req, std::shared_ptr<xarm_msgs::srv::PlanExec::Response> res);
    void execute_plan_topic(const std_msgs::msg::Bool::SharedPtr exec);
    void show_trail(bool plan_result);

private:
    rclcpp::Node::SharedPtr node_;
    // moveit_visual_tools::MoveItVisualTools *visual_tools;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::MoveGroupInterface::Plan my_xarm_plan_;

    rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_path_pub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr exec_plan_sub_;

    rclcpp::Service<xarm_msgs::srv::PlanExec>::SharedPtr exec_plan_server_;
    rclcpp::Service<xarm_msgs::srv::PlanPose>::SharedPtr pose_plan_server_;
    rclcpp::Service<xarm_msgs::srv::PlanJoint>::SharedPtr joint_plan_server_;
    rclcpp::Service<xarm_msgs::srv::PlanSingleStraight>::SharedPtr single_straight_plan_server_;
};

XArmSimplePlanner::XArmSimplePlanner(rclcpp::Node::SharedPtr& node)
{
    node_ = node;
    int dof;
    node->get_parameter_or("DOF", dof, 7);
    std::string PLANNING_GROUP = "xarm" + std::to_string(dof);
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, PLANNING_GROUP);
    init();
}

void XArmSimplePlanner::init()
{
    // move_group_->getJoints();
    // display_path_pub_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>("display_planned_path", 1);

    RCLCPP_INFO(node_->get_logger(), "[move_group_planner] Reference frame: %s", move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(node_->get_logger(), "[move_group_planner] End effector link: %s", move_group_->getEndEffectorLink().c_str());

    exec_plan_server_ = node_->create_service<xarm_msgs::srv::PlanExec>("xarm_exec_plan", BIND_CLS_CB(&XArmSimplePlanner::exec_plan_cb));
    pose_plan_server_ = node_->create_service<xarm_msgs::srv::PlanPose>("xarm_pose_plan", BIND_CLS_CB(&XArmSimplePlanner::do_pose_plan));
    joint_plan_server_ = node_->create_service<xarm_msgs::srv::PlanJoint>("xarm_joint_plan", BIND_CLS_CB(&XArmSimplePlanner::do_joint_plan));
    single_straight_plan_server_ = node_->create_service<xarm_msgs::srv::PlanSingleStraight>("xarm_straight_plan", BIND_CLS_CB(&XArmSimplePlanner::do_single_cartesian_plan));
    exec_plan_sub_ = node_->create_subscription<std_msgs::msg::Bool>("xarm_planner_exec", 10, std::bind(&XArmSimplePlanner::execute_plan_topic, this, std::placeholders::_1));

    // visual_tools = new moveit_visual_tools::MoveItVisualTools("link_base");
    // Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    // text_pose.translation().z() = 0.8;
    // visual_tools->publishText(text_pose, "xArm Planner Demo", rvt::WHITE, rvt::XLARGE);
    // visual_tools->trigger();
}

bool XArmSimplePlanner::do_pose_plan(const std::shared_ptr<xarm_msgs::srv::PlanPose::Request> req, std::shared_ptr<xarm_msgs::srv::PlanPose::Response> res)
{
    move_group_->setPoseTarget(req->target);
    RCLCPP_INFO(node_->get_logger(), "xarm_planner received new target: [ position: (%f, %f, %f), orientation: (%f, %f, %f, %f)", \
        req->target.position.x, req->target.position.y, req->target.position.z, req->target.orientation.x, \
        req->target.orientation.y, req->target.orientation.z, req->target.orientation.w);
    bool success = (move_group_->plan(my_xarm_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    res->success = success;
    RCLCPP_INFO(node_->get_logger(), "[move_group_planner] This plan (pose goal) %s", success ? "SUCCEEDED" : "FAILED");
    
    show_trail(success);

    return success;
}

bool XArmSimplePlanner::do_joint_plan(const std::shared_ptr<xarm_msgs::srv::PlanJoint::Request> req, std::shared_ptr<xarm_msgs::srv::PlanJoint::Response> res)
{
    RCLCPP_INFO(node_->get_logger(), "move_group_planner received new plan Request");
    move_group_->setJointValueTarget(req->target);
    
    bool success = (move_group_->plan(my_xarm_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    res->success = success;
    RCLCPP_INFO(node_->get_logger(), "[move_group_planner] This plan (joint goal) %s", success ? "SUCCEEDED" : "FAILED");
    show_trail(success);
    return success;
}

bool XArmSimplePlanner::do_single_cartesian_plan(const std::shared_ptr<xarm_msgs::srv::PlanSingleStraight::Request> req, std::shared_ptr<xarm_msgs::srv::PlanSingleStraight::Response> res)
{
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(req->target);
    move_group_->setMaxVelocityScalingFactor(maxV_scale_factor);
    moveit_msgs::msg::RobotTrajectory trajectory;
    
    double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    bool success = true;
    if(fraction<0.9)
        success = false;
    else
    {
        my_xarm_plan_.trajectory_ = trajectory;
    }
    fprintf(stderr, "[XArmSimplePlanner::do_single_cartesian_plan(): ] Coverage: %lf\n", fraction);

    res->success = success;
    show_trail(success);
    
    return success;
}

bool XArmSimplePlanner::exec_plan_cb(const std::shared_ptr<xarm_msgs::srv::PlanExec::Request> req, std::shared_ptr<xarm_msgs::srv::PlanExec::Response> res)
{
    if(req->exec)
    {
        RCLCPP_INFO(node_->get_logger(), "Received Execution Service Request, wait=%d", req->wait);
        bool finish_ok;
        if (req->wait)
            finish_ok = (move_group_->execute(my_xarm_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS); /* return after execution finish */
        else
            finish_ok = (move_group_->asyncExecute(my_xarm_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        res->success = finish_ok;
        return finish_ok;
    }
    res->success = false;
    return false;
}

void XArmSimplePlanner::execute_plan_topic(const std_msgs::msg::Bool::SharedPtr exec)
{
    if(exec->data)
    { 
        RCLCPP_INFO(node_->get_logger(), "Received Execution Command !!!!!");
        move_group_->asyncExecute(my_xarm_plan_); /* return without waiting for finish */
    }
}

void XArmSimplePlanner::show_trail(bool plan_result)
{
    if(plan_result)
    {
        // RCLCPP_INFO(node_->get_logger(), "[xarm_planner] Visualizing plan as trajectory line");
        
        // visual_tools->deleteAllMarkers();
        // const moveit::core::JointModelGroup* joint_model_group = move_group_->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
        // visual_tools->publishTrajectoryLine(my_xarm_plan_.trajectory_, joint_model_group);
        // visual_tools->trigger();
    }
}

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[xarm_simple_planner] Ctrl-C caught, exit process...\n");
    exit(-1);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("xarm_simple_planner");

    RCLCPP_INFO(node->get_logger(), "namespace: %s", node->get_namespace());

    node->declare_parameter("DOF");

    RCLCPP_INFO(node->get_logger(), "xarm_simple_planner start");
    XArmSimplePlanner xarm_simple_planner(node);

    signal(SIGINT, exit_sig_handler);
    rclcpp::spin(node);
    rclcpp::shutdown();

    RCLCPP_INFO(node->get_logger(), "xarm_simple_planner over");
}