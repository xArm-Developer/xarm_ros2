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
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_xarm_velo_move", "xarm");
    RCLCPP_INFO(rclcpp::get_logger("test_xarm_velo_move"), "namespace: %s", node->get_namespace());
    RCLCPP_INFO(rclcpp::get_logger("test_xarm_velo_move"), "test_xarm_velo_move start");

    signal(SIGINT, exit_sig_handler);

    xarm_api::XArmROSClient client;
    client.init(node);
    client.motionEnable(1);
    client.setMode(4);
	client.setState(0);

    int ret;

    std::vector<float> jnt_v = { 1, 0, 0, 0, 0, 0, 0 };
    ret = client.veloMoveJoint(jnt_v);
    RCLCPP_INFO(rclcpp::get_logger("test_xarm_velo_move"), "velo_move_joint: %d", ret);
    rclcpp::sleep_for(std::chrono::seconds(2));
    jnt_v[0] = -1;
    ret = client.veloMoveJoint(jnt_v);
    RCLCPP_INFO(rclcpp::get_logger("test_xarm_velo_move"), "velo_move_joint: %d", ret);
    rclcpp::sleep_for(std::chrono::seconds(2));
    // stop
    jnt_v[0] = 0;
    ret = client.veloMoveJoint(jnt_v);
    RCLCPP_INFO(rclcpp::get_logger("test_xarm_velo_move"), "velo_move_joint: %d", ret);

    std::vector<float> line_v = { 100, 0, 0, 0, 0, 0};
    client.setMode(5);
    client.setState(0);
    ret = client.veloMoveLine(line_v);
    RCLCPP_INFO(rclcpp::get_logger("test_xarm_velo_move"), "velo_move_line: %d", ret);
    rclcpp::sleep_for(std::chrono::seconds(2));
    line_v[0] = -100;
    ret = client.veloMoveLine(line_v);
    RCLCPP_INFO(rclcpp::get_logger("test_xarm_velo_move"), "velo_move_line: %d", ret);
    rclcpp::sleep_for(std::chrono::seconds(2));
    // stop
    line_v[0] = 0;
    ret = client.veloMoveLine(line_v);
    RCLCPP_INFO(rclcpp::get_logger("test_xarm_velo_move"), "velo_move_line: %d", ret);

    RCLCPP_INFO(rclcpp::get_logger("test_xarm_velo_move"), "test_xarm_velo_move over");
    return 0;
}