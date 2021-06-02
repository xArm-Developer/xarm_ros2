/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
           Vinman <vinman.cub@gmail.com>
 ============================================================================*/
 
#include <signal.h>
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
    RCLCPP_INFO(rclcpp::get_logger("test_xarm_ros_client"), "test_xarm_ros_client start");

    signal(SIGINT, exit_sig_handler);

    xarm_api::XArmROSClient client;
    client.init(node, hw_ns);
    client.motion_enable(true);
    client.set_mode(0);
	client.set_state(0);

    RCLCPP_INFO(rclcpp::get_logger("test_xarm_ros_client"), "test_xarm_ros_client over");
    return 0;
}