/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
           Vinman <vinman.cub@gmail.com>
 ============================================================================*/
 
#include <signal.h>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <xarm_api/xarm_ros_client.h>


void exit_sig_handler(int signum)
{
    fprintf(stderr, "[xarm_driver] Ctrl-C caught, exit process...\n");
    exit(-1);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string hw_ns = "xarm";
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("xarm_ros_client");
    RCLCPP_INFO(rclcpp::get_logger("test_xarm_velo_move"), "test_xarm_velo_move start");

    signal(SIGINT, exit_sig_handler);

    xarm_api::XArmROSClient client;
    client.init(node, hw_ns);
    client.motion_enable(true);
    client.set_mode(4);
	client.set_state(0);

    int ret;

    std::vector<float> jnt_v = { 1, 0, 0, 0, 0, 0, 0 };
    ret = client.vc_set_joint_velocity(jnt_v);
    RCLCPP_INFO(rclcpp::get_logger("test_xarm_velo_move"), "velo_move_joint: %d", ret);
    rclcpp::sleep_for(std::chrono::seconds(2));
    jnt_v[0] = -1;
    ret = client.vc_set_joint_velocity(jnt_v);
    RCLCPP_INFO(rclcpp::get_logger("test_xarm_velo_move"), "velo_move_joint: %d", ret);
    rclcpp::sleep_for(std::chrono::seconds(2));
    // stop
    jnt_v[0] = 0;
    ret = client.vc_set_joint_velocity(jnt_v);
    RCLCPP_INFO(rclcpp::get_logger("test_xarm_velo_move"), "velo_move_joint: %d", ret);

    std::vector<float> line_v = { 100, 0, 0, 0, 0, 0};
    client.set_mode(5);
	client.set_state(0);
    ret = client.vc_set_cartesian_velocity(line_v);
    RCLCPP_INFO(rclcpp::get_logger("test_xarm_velo_move"), "velo_move_line: %d", ret);
    rclcpp::sleep_for(std::chrono::seconds(2));
    line_v[0] = -100;
    ret = client.vc_set_cartesian_velocity(line_v);
    RCLCPP_INFO(rclcpp::get_logger("test_xarm_velo_move"), "velo_move_line: %d", ret);
    rclcpp::sleep_for(std::chrono::seconds(2));
    // stop
    line_v[0] = 0;
    ret = client.vc_set_cartesian_velocity(line_v);
    RCLCPP_INFO(rclcpp::get_logger("test_xarm_velo_move"), "velo_move_line: %d", ret);

    RCLCPP_INFO(rclcpp::get_logger("test_xarm_velo_move"), "test_xarm_velo_move over");
    return 0;
}