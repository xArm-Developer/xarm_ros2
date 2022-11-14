/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#ifndef __XARM_KEYBOARD_INPUT_H__
#define __XARM_KEYBOARD_INPUT_H__

#include <termios.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>


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

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr collision_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

    int dof_;
    int ros_queue_size_;

    std::string cartesian_command_in_topic_;
    std::string joint_command_in_topic_;

    std::string robot_link_command_frame_;
    std::string ee_frame_name_;

    std::string planning_frame_;

    double joint_vel_cmd_;
    double linear_pos_cmd_;

    rclcpp::Node::SharedPtr node_;
};


#endif // __XARM_KEYBOARD_INPUT_H__
