/* Copyright 2023 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#include <signal.h>
#include <thread>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <xarm_msgs/srv/call.hpp>

#define SERVICE_CALL_FAILED 999

std::shared_ptr<rclcpp::Node> node;

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[test_lite_gripper_node] Ctrl-C caught, exit process...\n");
    exit(-1);
}

template<typename ServiceT, typename SharedRequest = typename ServiceT::Request::SharedPtr>
int call_request(std::shared_ptr<ServiceT> client, SharedRequest req)
{
    bool is_try_again = false;
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            exit(1);
        }
        if (!is_try_again) {
            is_try_again = true;
            RCLCPP_WARN(node->get_logger(), "service %s not available, waiting ...", client->get_service_name());
        }
    }
    auto result_future = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service %s", client->get_service_name());
        return SERVICE_CALL_FAILED;
    }
    auto res = result_future.get();
    RCLCPP_INFO(node->get_logger(), "call service %s, ret=%d", client->get_service_name(), res->ret);
    return res->ret;
}

int main(int argc, char** argv)
{	
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node = rclcpp::Node::make_shared("test_lite_gripper_node", node_options);
    RCLCPP_INFO(node->get_logger(), "test_lite_gripper_node start");
    RCLCPP_INFO(node->get_logger(), "namespace: %s", node->get_namespace());

    std::string prefix;
    std::string hw_ns;
    node->get_parameter_or("prefix", prefix, std::string(""));
    node->get_parameter_or("hw_ns", hw_ns, std::string("ufactory"));
    std::string open_lite6_gripper_service = prefix + hw_ns + "/open_lite6_gripper";
    std::string close_lite6_gripper_service = prefix + hw_ns + "/close_lite6_gripper";

    RCLCPP_INFO(node->get_logger(), "open_lite_gripper_service: %s", open_lite6_gripper_service.c_str());
    RCLCPP_INFO(node->get_logger(), "close_lite_gripper_service: %s", close_lite6_gripper_service.c_str());
    
    signal(SIGINT, exit_sig_handler);

    rclcpp::Client<xarm_msgs::srv::Call>::SharedPtr open_client_ = node->create_client<xarm_msgs::srv::Call>(open_lite6_gripper_service);
    rclcpp::Client<xarm_msgs::srv::Call>::SharedPtr close_client_ = node->create_client<xarm_msgs::srv::Call>(close_lite6_gripper_service);

    std::shared_ptr<xarm_msgs::srv::Call::Request> call_req = std::make_shared<xarm_msgs::srv::Call::Request>();;
    
    while (rclcpp::ok())
    {
        call_request(open_client_, call_req);
        rclcpp::sleep_for(std::chrono::seconds(1));
        call_request(close_client_, call_req);
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    RCLCPP_INFO(node->get_logger(), "test_lite_gripper_node over");
    return 0;
}