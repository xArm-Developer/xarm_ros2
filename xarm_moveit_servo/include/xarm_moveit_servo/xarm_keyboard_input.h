/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#ifndef __XARM_KEYBOARD_INPUT_H__
#define __XARM_KEYBOARD_INPUT_H__

#include <termios.h>

#include <unistd.h>
#include <cstring>
#include <string>
#include <memory>
#include <stdexcept>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>


class KeyboardReader
{
public:
    KeyboardReader() : k_fd_(0)
    {
        tcgetattr(k_fd_, &k_old_termios_);
        struct termios k_termios;
        memcpy(&k_termios, &k_old_termios_, sizeof(struct termios));
        k_termios.c_lflag &= ~(ICANON | ECHO);
        // Setting a new line, then end of file
        k_termios.c_cc[VEOL] = 1;
        k_termios.c_cc[VEOF] = 2;
        // k_termios.c_ispeed = 30;
        // k_termios.c_ospeed = 30;
        tcsetattr(k_fd_, TCSANOW, &k_termios);
    }
    void readOne(char *c)
    {
        int rc = read(k_fd_, c, 1);
        if (rc < 0)
        {
            throw std::runtime_error("keyboard read failed");
        }
    }
    void shutdown()
    {
        tcsetattr(k_fd_, TCSANOW, &k_old_termios_);
    } 

private:
    int k_fd_;
    struct termios k_old_termios_;
};


class KeyboardServoPub
{
public:
    KeyboardServoPub(rclcpp::Node::SharedPtr& node);
    void keyLoop();

private:
    template <typename T>
    void _declare_or_get_param(T& output_value, const std::string& param_name, const T default_value = T{});
    void spin();

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_arm1_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_arm2_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr collision_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr elevator_cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr drivetrain_cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_left_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_right_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
    rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_input_arm1_;
    rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_input_arm2_;
    std::shared_ptr<moveit_msgs::srv::ServoCommandType::Request> switch_request_;

    int dof_;
    int ros_queue_size_;
    int arm1_command_type_;
    int arm2_command_type_;

    std::string cartesian_command_in_topic_;
    std::string joint_command_in_topic_;
    std::string elevator_cmd_vel_topic_;
    std::string drivetrain_cmd_vel_topic_;
    std::string gripper_left_topic_;
    std::string gripper_right_topic_;

    std::string robot_link_command_frame_;
    std::string ee_frame_name_;

    std::string planning_frame_;
    std::string arm1_ns_;
    std::string arm2_ns_;
    std::string arm1_planning_frame_;
    std::string arm2_planning_frame_;
    std::string servo_srv_ns_;

    std::string joint_prefix_;

    double joint_vel_cmd_;
    double linear_pos_cmd_;
    double elevator_vel_step_;
    double drivetrain_linear_vel_;
    double drivetrain_angular_vel_;
    double gripper_step_;
    
    // Gripper state tracking for incremental control
    double left_gripper_width_;
    double right_gripper_width_;

    rclcpp::Node::SharedPtr node_;
    void _switch_command_type(int arm_idx, int command_type);
    void publish_twist_for_arm(int arm_idx, double dx, double dy, double dz);
    void publish_elevator_velocity(double vz);
    void publish_drivetrain_velocity(double linear_x, double linear_y, double angular_z);
    void publish_gripper_width(int arm_idx, double width);
    void close_gripper_slightly(int arm_idx);
    void open_gripper_slightly(int arm_idx);
};


#endif // __XARM_KEYBOARD_INPUT_H__
