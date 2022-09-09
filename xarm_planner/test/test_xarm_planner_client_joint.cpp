/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#include <signal.h>
#include <thread>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <xarm_msgs/srv/plan_pose.hpp>
#include <xarm_msgs/srv/plan_joint.hpp>
#include <xarm_msgs/srv/plan_exec.hpp>
#include <xarm_msgs/srv/plan_single_straight.hpp>

#define SERVICE_CALL_FAILED 999

std::shared_ptr<rclcpp::Node> node;

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[test_xarm_planner_node_joint] Ctrl-C caught, exit process...\n");
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
    RCLCPP_INFO(node->get_logger(), "call service %s, success=%d", client->get_service_name(), res->success);
    return res->success;
}

int main(int argc, char** argv)
{	
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node = rclcpp::Node::make_shared("test_xarm_planner_node_joint", node_options);
    RCLCPP_INFO(node->get_logger(), "test_xarm_planner_node_joint start");
    // node->declare_parameter("dof");
    int dof;
    node->get_parameter_or("dof", dof, 7);
    RCLCPP_INFO(node->get_logger(), "namespace: %s, dof: %d", node->get_namespace(), dof);
    
    signal(SIGINT, exit_sig_handler);

    rclcpp::Client<xarm_msgs::srv::PlanJoint>::SharedPtr joint_plan_client_ = node->create_client<xarm_msgs::srv::PlanJoint>("xarm_joint_plan");
    rclcpp::Client<xarm_msgs::srv::PlanExec>::SharedPtr exec_plan_client_ = node->create_client<xarm_msgs::srv::PlanExec>("xarm_exec_plan");

    std::shared_ptr<xarm_msgs::srv::PlanJoint::Request> joint_plan_req = std::make_shared<xarm_msgs::srv::PlanJoint::Request>();;
    std::shared_ptr<xarm_msgs::srv::PlanExec::Request> exec_plan_req = std::make_shared<xarm_msgs::srv::PlanExec::Request>();;

    exec_plan_req->wait = true;

    std::vector<double> tar_joint1;
    std::vector<double> tar_joint2;
    std::vector<double> tar_joint3;
    
    switch (dof) {
    case 5:
        {
            tar_joint1 = {1.570796, -1.570796, -1.047198, 2.792527, -1.570796};
            tar_joint2 = {0, 0, 0, 0, 0};
            tar_joint3 = {-1.570796, -1.570796, -1.047198, -0.349066, 2.617994};
        }
        break;
    case 6:
        {
            tar_joint1 = {1.570796, -1.570796, -1.047198, 2.967060, 2.792527, -3.124139};
            tar_joint2 = {0, 0, 0, 0, 0, 0};
            tar_joint3 = {-1.570796, -1.570796, -1.047198, -2.967060, -0.349066, 3.124139};
        }
        break;
    case 7:
        {
            tar_joint1 = {1.570796, -1.570796, -1.570796, 1.396263, 2.967060, 2.792527, -1.570796};
            tar_joint2 = {0, 0, 0, 0, 0, 0, 0};
            tar_joint3 = {-1.570796, -1.570796, 1.570796, 1.396263, -2.967060, -0.349066, 2.617994};
        }
        break;
    default:
        {
            RCLCPP_INFO(node->get_logger(), "param dof error");
            exit(1);
        }
        break;
    }
    
    while (rclcpp::ok())
    {
        joint_plan_req->target = tar_joint2;
        call_request(joint_plan_client_, joint_plan_req);
        call_request(exec_plan_client_, exec_plan_req);
        // rclcpp::sleep_for(std::chrono::seconds(3));

        joint_plan_req->target = tar_joint1;
        call_request(joint_plan_client_, joint_plan_req);
        call_request(exec_plan_client_, exec_plan_req);
        // rclcpp::sleep_for(std::chrono::seconds(3));

        joint_plan_req->target = tar_joint2;
        call_request(joint_plan_client_, joint_plan_req);
        call_request(exec_plan_client_, exec_plan_req);
        // rclcpp::sleep_for(std::chrono::seconds(3));

        joint_plan_req->target = tar_joint3;
        call_request(joint_plan_client_, joint_plan_req);
        call_request(exec_plan_client_, exec_plan_req);
        // rclcpp::sleep_for(std::chrono::seconds(3));
    }

    RCLCPP_INFO(node->get_logger(), "test_xarm_planner_node_joint over");
    return 0;
}