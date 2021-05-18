/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
           Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "xarm_api/xarm_msgs.h"


void exit_sig_handler(int signum)
{
    fprintf(stderr, "[xarm_driver] Ctrl-C caught, exit process...\n");
    exit(-1);
}

void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr states)
{
    std::string pos_str = "[ ";
    for (int i = 0; i < states->position.size(); i++) { 
        pos_str += std::to_string(states->position[i]); 
        pos_str += " ";
    }
    pos_str += "]";
    RCLCPP_INFO(rclcpp::get_logger("joint_states"), "positon: %s", pos_str.c_str());
}

void xarm_states_callback(const xarm_msgs::msg::RobotMsg::SharedPtr states)
{
    RCLCPP_INFO(rclcpp::get_logger("xarm_states"), "state: %d, error: %d", states->state, states->err);
}

void xarm_cgpio_states_callback(const xarm_msgs::msg::CIOState::SharedPtr states)
{
    RCLCPP_INFO(rclcpp::get_logger("xarm_cgpio_states"), "callback");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string hw_ns = "xarm";
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_xarm_states", hw_ns);

    RCLCPP_INFO(rclcpp::get_logger("test_xarm_states"), "namespace: %s", node->get_namespace());
    RCLCPP_INFO(rclcpp::get_logger("test_xarm_states"), "test_xarm_states start");

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>("joint_states", 100, joint_states_callback);
    rclcpp::Subscription<xarm_msgs::msg::RobotMsg>::SharedPtr xarm_state_sub = node->create_subscription<xarm_msgs::msg::RobotMsg>("xarm_states", 100, xarm_states_callback);
    rclcpp::Subscription<xarm_msgs::msg::CIOState>::SharedPtr cgpio_sub = node->create_subscription<xarm_msgs::msg::CIOState>("xarm_cgpio_states", 100, xarm_cgpio_states_callback);

    signal(SIGINT, exit_sig_handler);
    rclcpp::spin(node);
    rclcpp::shutdown();

    RCLCPP_INFO(rclcpp::get_logger("test_xarm_states"), "test_xarm_states over");
    return 0;
}
