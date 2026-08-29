/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/

/*
 * ============================================================================
 * ★★★ 本文件相对 UFACTORY 官方原版的全部修改说明 ★★★
 * ============================================================================
 * 修改背景：
 *   真机实验时发现机械臂运动（尤其是笛卡尔直线运动）速度严重超速，是由于笛卡尔直线运动不受限速约束，
 *   存在安全隐患，需要对 xarm_planner 的全部运动接口做统一限速。
 *
 * 修改 1：调低速度/加速度缩放系数（第 30~36 行附近）
 *   - 官方原值: max_velocity_scaling_factor = 0.3
 *              max_acceleration_scaling_factor = 0.1
 *   - 现改为:   0.05 / 0.02（真机安全速度，可按需调整）
 *   - 实际速度 ≈ joint_limits.yaml 中 max_velocity(2.14 rad/s) × 缩放系数，
 *     即约 0.11 rad/s（约 6°/s）。
 *   - 这两个常量对 OMPL 规划（plan_pose/plan_joint）和笛卡尔规划都生效，
 *     是全局唯一的速度调节入口，调速度只改这里即可。
 *
 * 修改 2：笛卡尔路径手动时间参数化（planCartesianPath 函数内，【限速核心】）
 *   - 问题：MoveIt2 Humble 的 GetCartesianPath 服务没有速度缩放字段，
 *     setMaxVelocityScalingFactor 对 computeCartesianPath 完全不生效，
 *     笛卡尔轨迹会按 joint_limits.yaml 满速执行（这就是改小系数后
 *     笛卡尔运动仍然飞快的原因）。
 *   - 修复：computeCartesianPath 成功后，用 TOTG
 *     (TimeOptimalTrajectoryGeneration) 按上述缩放系数手动重新做时间参数化。
 *   - 【重要禁忌】起始状态从轨迹首点构造，绝不能用
 *     move_group_->getCurrentState()！后者会在服务回调内部发起阻塞式服务调用，
 *     在 rclcpp 单线程执行器下会死锁（表现为客户端服务调用永久卡住）。
 *
 * 修改 3：向 RViz 重新发布限速后的笛卡尔轨迹（init + planCartesianPath）
 *   - 问题：move_group 规划成功后会自动把轨迹发布到 display_planned_path 供
 *     RViz 显示，但笛卡尔轨迹无时间参数化（time_from_start 全为 0），
 *     RViz 播放时一闪而过。
 *   - 修复：时间参数化成功后，把带时序的轨迹按 DisplayTrajectory 格式重新发布。
 *   - 因此 RViz 中会先闪现一下（move_group 发的无时序版），再出现慢速轨迹，
 *     这是预期行为，已确认保留。
 *
 * 修改 4：新增头文件与 CMake 依赖（见文件头部 include 和 CMakeLists.txt）
 *   - 新增 include: robot_trajectory.h / conversions.h /
 *     time_optimal_trajectory_generation.h（均来自 moveit_core）
 *   - CMakeLists.txt 新增 find_package(moveit_core) 并加入 ament_target_dependencies
 *
 * 修改任何内容后必须执行：
 *   cd xarm_ws && colcon build --packages-select xarm_planner
 *   然后重启 launch（xarm_planner_node）才会生效。
 * ============================================================================
 */

#include "xarm_planner/xarm_planner.h"
// ---- 【修改 4】以下三个头文件均为本次修改新增，服务于笛卡尔路径限速与 RViz 显示 ----
#include <moveit/robot_trajectory/robot_trajectory.h>                       // RobotTrajectory 对象，供 TOTG 处理
#include <moveit/robot_state/conversions.h>                                 // robotStateToRobotStateMsg，构造 RViz 显示消息的起始状态
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h> // TOTG 时间参数化算法

namespace xarm_planner
{
const double jump_threshold = 0.0;
const double eef_step = 0.005;
// ------------------------------------------------------------
// 【修改 1】全局速度/加速度缩放系数（调速度只需改这两个常量）
// 官方原值 0.3 / 0.1，真机上过快存在安全隐患，已调低。
// 生效范围：
//   - OMPL 规划 (xarm_pose_plan / xarm_joint_plan)：通过
//     init() 中 setMaxVelocityScalingFactor 传给规划流水线；
//   - 笛卡尔规划 (xarm_straight_plan)：官方接口会忽略该系数，
//     靠下方 planCartesianPath 中手动 TOTG 传入，见【修改 2】。
// 实际速度 ≈ xarm_moveit_config/config/xarm7/joint_limits.yaml 的
// max_velocity(2.14 rad/s) × 本系数。觉得太慢/太快时在此调整，
// 然后重新编译本包并重启 launch。
// ------------------------------------------------------------
const double max_velocity_scaling_factor = 0.1;      // 官方原值 0.3
const double max_acceleration_scaling_factor = 0.05;  // 官方原值 0.1

XArmPlanner::XArmPlanner(const rclcpp::Node::SharedPtr& node, const std::string& group_name)
    : node_(node)
{
    init(group_name);
}

XArmPlanner::XArmPlanner(const std::string& group_name)
{
    node_ = rclcpp::Node::make_shared("xarm_planner_move_group_node");
    init(group_name);
}

void XArmPlanner::init(const std::string& group_name) 
{
    is_trajectory_ = false;
    // ------------------------------------------------------------
    // 【修改 3】创建 RViz 轨迹显示发布器（官方原版没有这段）
    // 目的：把限速后的笛卡尔轨迹重新发布给 RViz，解决笛卡尔轨迹在
    // RViz 中一闪而过的问题（原因见文件头部修改说明）。
    // 两个话题对应 RViz 配置中两个不同的显示组件（见项目内
    // xarm7_grasp/rviz/xarm7_grasp_vision.rviz）：
    //   MotionPlanning 显示: /move_group/display_planned_path（相对话题解析）
    //   Trajectory 显示:    /display_planned_path（绝对话题）
    // 两个都发布，确保任一组件都能看到限速后的轨迹。
    // 注意：必须用绝对话题名（带开头 /），否则 RViz 订阅不到。
    // ------------------------------------------------------------
    display_planned_path_pub_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
        "/move_group/display_planned_path", 10);
    display_planned_path_pub2_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
        "/display_planned_path", 10);
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name);
    RCLCPP_INFO(node_->get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(node_->get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());
    RCLCPP_INFO(node_->get_logger(), "Available Planning Groups:");
    std::copy(move_group_->getJointModelGroupNames().begin(), move_group_->getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
    // 【修改 1 生效点】把缩放系数设置给 MoveGroupInterface，
    // 对 OMPL 规划生效（笛卡尔规划不走这条路，见 planCartesianPath）
    move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor);
    move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);
}

bool XArmPlanner::planJointTarget(const std::vector<double>& joint_target)
{
    bool success = move_group_->setJointValueTarget(joint_target);
    if (!success)
        RCLCPP_WARN(node_->get_logger(), "setJointValueTarget: out of bounds");
    success = (move_group_->plan(xarm_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success)
        RCLCPP_ERROR(node_->get_logger(), "planJointTarget: plan failed");
    is_trajectory_ = false;
    return success;
}

bool XArmPlanner::planPoseTarget(const geometry_msgs::msg::Pose& pose_target)
{
    bool success = move_group_->setPoseTarget(pose_target);
    if (!success)
        RCLCPP_WARN(node_->get_logger(), "setPoseTarget: out of bounds");
    success = (move_group_->plan(xarm_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success)
        RCLCPP_ERROR(node_->get_logger(), "planPoseTarget: plan failed");
    is_trajectory_ = false;
    return success;
}

bool XArmPlanner::planPoseTargets(const std::vector<geometry_msgs::msg::Pose>& pose_target_vector)
{
    bool success = move_group_->setPoseTargets(pose_target_vector);
    if (!success)
        RCLCPP_WARN(node_->get_logger(), "setPoseTargets: out of bounds");
    success = (move_group_->plan(xarm_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success)
        RCLCPP_ERROR(node_->get_logger(), "planPoseTargets: plan failed");
    is_trajectory_ = false;
    return success;
}

bool XArmPlanner::planCartesianPath(const std::vector<geometry_msgs::msg::Pose>& pose_target_vector)
{   
    // moveit_msgs::msg::RobotTrajectory trajectory_;
    
    double fraction = move_group_->computeCartesianPath(pose_target_vector, eef_step, jump_threshold, trajectory_);
    bool success = true;
    if(fraction < 0.9) {
        RCLCPP_ERROR(node_->get_logger(), "planCartesianPath: plan failed, fraction=%lf", fraction);
        return false;
    }

    // ============================================================
    // 【修改 2】笛卡尔轨迹手动时间参数化（限速核心，官方原版没有以下整段）
    //
    // 原因：MoveGroupInterface::computeCartesianPath 调用的 GetCartesianPath
    // 服务没有速度/加速度缩放字段，setMaxVelocityScalingFactor 设置的系数
    // 对笛卡尔路径完全不生效，轨迹会按 joint_limits.yaml 满速执行。
    // 这里仿照 RViz 内部的做法，手动用 TOTG 按缩放系数重新做时间参数化。
    //
    // 【重要禁忌】起始状态从轨迹首点构造，绝不能用 move_group_->getCurrentState()！
    // 后者会在服务回调内部发起阻塞式服务调用，在单线程执行器下会死锁，
    // 表现为客户端调用 xarm_straight_plan 服务永久卡住（已踩坑验证）。
    // ============================================================
    try {
        const auto& jt = trajectory_.joint_trajectory;

        // 步骤 1：构造轨迹起始状态（从轨迹第一个点逐关节赋值，纯本地计算，无阻塞）
        moveit::core::RobotState start_state(move_group_->getRobotModel());
        start_state.setToDefaultValues();
        if (!jt.points.empty()) {
            for (size_t i = 0; i < jt.joint_names.size() && i < jt.points[0].positions.size(); ++i)
                start_state.setVariablePosition(jt.joint_names[i], jt.points[0].positions[i]);
        }
        start_state.update();

        // 步骤 2：把消息格式轨迹转为内部 RobotTrajectory 对象，供时间参数化处理
        robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(), move_group_->getName());
        rt.setRobotTrajectoryMsg(start_state, trajectory_);

        // 步骤 3：用 TOTG 按【修改 1】的缩放系数重新计算每个轨迹点的时间戳，
        // 成功后回写 trajectory_，后续 executePath 执行的就是限速版轨迹。
        // 若失败/异常，保留原始时序（不阻塞规划流程，仅打印警告）。
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        bool time_ok = totg.computeTimeStamps(rt, max_velocity_scaling_factor, max_acceleration_scaling_factor);
        if (time_ok) {
            rt.getRobotTrajectoryMsg(trajectory_);
            RCLCPP_INFO(node_->get_logger(),
                "planCartesianPath: time parameterization success, vel_scale=%.2f, acc_scale=%.2f",
                max_velocity_scaling_factor, max_acceleration_scaling_factor);

            // 【修改 3】把限速后的带时序轨迹重新发布给 RViz 显示。
            // move_group 在规划完成时已自动发布过一版无时序轨迹（time_from_start 全为 0，
            // RViz 播放一闪而过），这里发布限速版本会将其替换为慢速播放。
            // 因此 RViz 中会先闪现一下再出现慢速轨迹，属预期行为。
            moveit_msgs::msg::DisplayTrajectory display_traj;
            display_traj.model_id = move_group_->getRobotModel()->getName();
            display_traj.trajectory.push_back(trajectory_);
            moveit::core::robotStateToRobotStateMsg(start_state, display_traj.trajectory_start);
            display_planned_path_pub_->publish(display_traj);   // → MotionPlanning 显示组件
            display_planned_path_pub2_->publish(display_traj);  // → Trajectory 显示组件
        } else {
            RCLCPP_WARN(node_->get_logger(), "planCartesianPath: time parameterization failed, keep original timing");
        }
    } catch (const std::exception& e) {
        // 异常保护：时间参数化出错时回退为原始轨迹时序，不让节点崩溃或服务卡死
        RCLCPP_ERROR(node_->get_logger(),
            "planCartesianPath: time parameterization exception: %s, keep original timing", e.what());
    }

    is_trajectory_ = true;
    // https://github.com/ros-planning/moveit2/commit/8bfe782d6254997d185644fa3eb358d2b79d69b2
    // (struct Plan) trajectory_ => trajectory
    // xarm_plan_.trajectory_ = trajectory;
    return true;
}

bool XArmPlanner::executePath(bool wait)
{
    moveit::core::MoveItErrorCode code;
    if (wait)
        code = is_trajectory_ ? move_group_->execute(trajectory_) : move_group_->execute(xarm_plan_);
    else
        code =  is_trajectory_ ? move_group_->asyncExecute(trajectory_) : move_group_->asyncExecute(xarm_plan_);
    bool success = (code == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success)
        RCLCPP_ERROR(node_->get_logger(), "executePath: execute failed, wait=%d, MoveItErrorCode=%d", wait, code.val);
    return success;
}
}
