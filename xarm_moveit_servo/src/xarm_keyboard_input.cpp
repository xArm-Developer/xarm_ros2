/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#include <signal.h>
#include <stdio.h>
#include <thread>
#include <unistd.h>
#include "xarm_moveit_servo/xarm_keyboard_input.h"


// Define used keys
#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_PERIOD 0x2E
#define KEYCODE_SEMICOLON 0x3B
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_7 0x37
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_E 0x65
#define KEYCODE_R 0x72

KeyboardReader keyboard_reader_;


KeyboardServoPub::KeyboardServoPub(rclcpp::Node::SharedPtr& node)
: dof_(6), ros_queue_size_(10),
  // make these RELATIVE so they resolve under your node's namespace (e.g., /arm1/...)
  cartesian_command_in_topic_("cmd_ee"),
  joint_command_in_topic_("joint_delta"),
  // leave frames as-is; your launch/YAML can override them
  robot_link_command_frame_("link_base"),
  ee_frame_name_("link_eef"),
  planning_frame_("link_base"),
  joint_vel_cmd_(1.0),
  linear_pos_cmd_(0.5)
{
    node_ = node;
    // init parameter from node
    // before reading params
    joint_prefix_ = "arm1_";  // default; override per-namespace in launch

    // after your other _declare_or_get_param(...) calls
    _declare_or_get_param<std::string>(joint_prefix_, "joint_prefix", joint_prefix_);
    _declare_or_get_param<int>(dof_, "dof", dof_);
    _declare_or_get_param<int>(ros_queue_size_, "ros_queue_size", ros_queue_size_);
    _declare_or_get_param<std::string>(cartesian_command_in_topic_, "moveit_servo.cartesian_command_in_topic", cartesian_command_in_topic_);
    _declare_or_get_param<std::string>(joint_command_in_topic_, "moveit_servo.joint_command_in_topic", joint_command_in_topic_);
    _declare_or_get_param<std::string>(robot_link_command_frame_, "moveit_servo.robot_link_command_frame", robot_link_command_frame_);
    _declare_or_get_param<std::string>(ee_frame_name_, "moveit_servo.ee_frame_name", ee_frame_name_);
    _declare_or_get_param<std::string>(planning_frame_, "moveit_servo.planning_frame", planning_frame_);

    if (cartesian_command_in_topic_.rfind("~/", 0) == 0) {
        cartesian_command_in_topic_ = cartesian_command_in_topic_.substr(2);
    }
    if (joint_command_in_topic_.rfind("~/", 0) == 0) {
        joint_command_in_topic_ = joint_command_in_topic_.substr(2); // <- fixed length bug
    }

    // Setup pub/sub
    twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
        cartesian_command_in_topic_, rclcpp::SensorDataQoS());
    joint_pub_ = node_->create_publisher<control_msgs::msg::JointJog>(
        joint_command_in_topic_, ros_queue_size_);
    // collision_pub_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);

    // Create a service client to start the ServoServer
    // --- params for service namespace + optional start ---
    servo_srv_ns_ = "servo_node";        // becomes /<arm_ns>/servo_node/... under your namespace
    bool try_start_service = false;

    _declare_or_get_param<std::string>(servo_srv_ns_, "servo_srv_ns", servo_srv_ns_);
    _declare_or_get_param<bool>(try_start_service, "try_start_service", try_start_service);

    // --- build clients against your ServoNode ---
    switch_input_ = node_->create_client<moveit_msgs::srv::ServoCommandType>(
        servo_srv_ns_ + std::string("/switch_command_type"));

    // (optional) start service; many Servo builds don’t expose this
    if (try_start_service) {
    servo_start_client_ = node_->create_client<std_srvs::srv::Trigger>(
        servo_srv_ns_ + std::string("/start_servo"));
    if (servo_start_client_->wait_for_service(std::chrono::seconds(1))) {
        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        servo_start_client_->async_send_request(req);
    } else {
        RCLCPP_WARN(node_->get_logger(), "Start service not available at %s",
                    (servo_srv_ns_ + "/start_servo").c_str());
    }
    }

    // init switching state
    command_type_ = -1;
    switch_request_ = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();

    RCLCPP_INFO(node_->get_logger(),
    "Servo services under ns: %s  (switch: %s)",
    servo_srv_ns_.c_str(), (servo_srv_ns_ + "/switch_command_type").c_str());
}

template <typename T>
void KeyboardServoPub::_declare_or_get_param(T& output_value, const std::string& param_name, const T default_value)
{
    try
    {
        if (node_->has_parameter(param_name))
        {
            node_->get_parameter<T>(param_name, output_value);
        }
        else
        {
            output_value = node_->declare_parameter<T>(param_name, default_value);
        }
    }
    catch (const rclcpp::exceptions::InvalidParameterTypeException& e)
    {
        RCLCPP_WARN_STREAM(node_->get_logger(), "InvalidParameterTypeException(" << param_name << "): " << e.what());
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Error getting parameter \'" << param_name << "\', check parameter type in YAML file");
        throw e;
    }

    RCLCPP_INFO_STREAM(node_->get_logger(), "Found parameter - " << param_name << ": " << output_value);
}

void KeyboardServoPub::spin()
{
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node_);
  }
}

void KeyboardServoPub::_switch_command_type(int command_type)
{
    if (command_type == command_type_) return;

    if (!switch_input_->wait_for_service(std::chrono::seconds(0))) {
        RCLCPP_WARN(node_->get_logger(),
                    "Service unavailable: %s",
                    (servo_srv_ns_ + "/switch_command_type").c_str());
        return;
    }
    switch (command_type) {
        case 0:
        {
            switch_request_->command_type = moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG;
            auto result = switch_input_->async_send_request(switch_request_);
            if (result.get()->success)
            {
                command_type_ = command_type;
                RCLCPP_INFO_STREAM(node_->get_logger(), "Switched to input type: JOINT_JOG");
            }
            else
            {
                RCLCPP_WARN_STREAM(node_->get_logger(), "Could not switch input to: JOINT_JOG");
            }

            // switch_request_->command_type = moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG;
            // auto result_future = switch_input_->async_send_request(switch_request_);
            // RCLCPP_INFO_STREAM(node_->get_logger(), "11Switched to input type: JOINT_JOG");
            // if (rclcpp::spin_until_future_complete(node_, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
            //     auto result = result_future.get();
            //     if (result->success) {
            //         command_type_ = command_type;
            //         RCLCPP_INFO(node_->get_logger(), "Switched to input type: JOINT_JOG");
            //     } else {
            //         RCLCPP_WARN(node_->get_logger(), "Could not switch input to: JOINT_JOG");
            //     }
            // }
            // else {
            //     RCLCPP_ERROR(node_->get_logger(), "Failed to call service /servo_server/switch_command_type");
            // }
            break;
        }
        case 1:
        {
            switch_request_->command_type = moveit_msgs::srv::ServoCommandType::Request::TWIST;
            auto result = switch_input_->async_send_request(switch_request_);
            if (result.get()->success)
            {
                command_type_ = command_type;
                RCLCPP_INFO_STREAM(node_->get_logger(), "Switched to input type: TWIST");
            }
            else
            {
                RCLCPP_WARN_STREAM(node_->get_logger(), "Could not switch input to: TWIST");
            }

            // switch_request_->command_type = moveit_msgs::srv::ServoCommandType::Request::TWIST;
            // auto result_future = switch_input_->async_send_request(switch_request_);
            // RCLCPP_INFO_STREAM(node_->get_logger(), "11Switched to input type: TWIST");
            // if (rclcpp::spin_until_future_complete(node_, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
            //     auto result = result_future.get();
            //     if (result->success) {
            //         command_type_ = command_type;
            //         RCLCPP_INFO(node_->get_logger(), "Switched to input type: TWIST");
            //     } else {
            //         RCLCPP_WARN(node_->get_logger(), "Could not switch input to: TWIST");
            //     }
            // }
            // else {
            //     RCLCPP_ERROR(node_->get_logger(), "Failed to call service /servo_server/switch_command_type");
            // }
            break;
        }
        case 2:
        {
            switch_request_->command_type = moveit_msgs::srv::ServoCommandType::Request::POSE;
            auto result = switch_input_->async_send_request(switch_request_);
            if (result.get()->success)
            {
                command_type_ = command_type;
                RCLCPP_INFO_STREAM(node_->get_logger(), "Switched to input type: POSE");
            }
            else
            {
                RCLCPP_WARN_STREAM(node_->get_logger(), "Could not switch input to: POSE");
            }

            // switch_request_->command_type = moveit_msgs::srv::ServoCommandType::Request::POSE;
            // auto result_future = switch_input_->async_send_request(switch_request_);
            // RCLCPP_INFO_STREAM(node_->get_logger(), "11Switched to input type: POSE");
            // if (rclcpp::spin_until_future_complete(node_, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
            //     auto result = result_future.get();
            //     if (result->success) {
            //         command_type_ = command_type;
            //         RCLCPP_INFO(node_->get_logger(), "Switched to input type: POSE");
            //     } else {
            //         RCLCPP_WARN(node_->get_logger(), "Could not switch input to: POSE");
            //     }
            // }
            // else {
            //     RCLCPP_ERROR(node_->get_logger(), "Failed to call service /servo_server/switch_command_type");
            // }
            break;
        }
        default:
            break;
    }
}

void KeyboardServoPub::keyLoop()
{
    char c;
    bool publish_twist = false;
    bool publish_joint = false;

    std::thread{ std::bind(&KeyboardServoPub::spin, this) }.detach();

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys and the '.' and ';' keys to Cartesian jog");
    puts("Use 'W' to Cartesian jog in the world frame, and 'E' for the End-Effector frame");
    puts("Use 1|2|3|4|5|6|7 keys to joint jog. 'R' to reverse the direction of jogging.");
    puts("'Q' to quit.");

    switch_request_ = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
    
    for (;;) {
        try {
            keyboard_reader_.readOne(&c);
        }
        catch (const std::runtime_error&) {
            perror("read():");
            return;
        }
        RCLCPP_DEBUG(node_->get_logger(), "value: 0x%02X", c);

        auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

        // Use read key-press
        switch (c)
        {
        case KEYCODE_LEFT:
            RCLCPP_DEBUG(node_->get_logger(), "LEFT");
            twist_msg->twist.linear.y = linear_pos_cmd_;
            publish_twist = true;
            break;
        case KEYCODE_RIGHT:
            RCLCPP_DEBUG(node_->get_logger(), "RIGHT");
            twist_msg->twist.linear.y = -linear_pos_cmd_;
            publish_twist = true;
            break;
        case KEYCODE_UP:
            RCLCPP_DEBUG(node_->get_logger(), "UP");
            twist_msg->twist.linear.x = linear_pos_cmd_;
            publish_twist = true;
            break;
        case KEYCODE_DOWN:
            RCLCPP_DEBUG(node_->get_logger(), "DOWN");
            twist_msg->twist.linear.x = -linear_pos_cmd_;
            publish_twist = true;
            break;
        case KEYCODE_PERIOD:
            RCLCPP_DEBUG(node_->get_logger(), "PERIOD");
            twist_msg->twist.linear.z = -linear_pos_cmd_;
            publish_twist = true;
            break;
        case KEYCODE_SEMICOLON:
            RCLCPP_DEBUG(node_->get_logger(), "SEMICOLON");
            twist_msg->twist.linear.z = linear_pos_cmd_;
            publish_twist = true;
            break;
        case KEYCODE_E:
            RCLCPP_DEBUG(node_->get_logger(), "E");
            planning_frame_ = ee_frame_name_;
            break;
        case KEYCODE_W:
            RCLCPP_DEBUG(node_->get_logger(), "W");
            planning_frame_ = robot_link_command_frame_;
            break;
        case KEYCODE_1:
            RCLCPP_DEBUG(node_->get_logger(), "1");
            joint_msg->joint_names.push_back(joint_prefix_ + "joint1");
            joint_msg->velocities.push_back(joint_vel_cmd_);
            publish_joint = true;
            break;
        case KEYCODE_2:
            RCLCPP_DEBUG(node_->get_logger(), "2");
            joint_msg->joint_names.push_back(joint_prefix_ + "joint2");
            joint_msg->velocities.push_back(joint_vel_cmd_);
            publish_joint = true;
            break;
        case KEYCODE_3:
            RCLCPP_DEBUG(node_->get_logger(), "3");
            joint_msg->joint_names.push_back(joint_prefix_ + "joint3");
            joint_msg->velocities.push_back(joint_vel_cmd_);
            publish_joint = true;
            break;
        case KEYCODE_4:
            RCLCPP_DEBUG(node_->get_logger(), "4");
            joint_msg->joint_names.push_back(joint_prefix_ + "joint4");
            joint_msg->velocities.push_back(joint_vel_cmd_);
            publish_joint = true;
            break;
        case KEYCODE_5:
            RCLCPP_DEBUG(node_->get_logger(), "5");
            joint_msg->joint_names.push_back(joint_prefix_ + "joint5");
            joint_msg->velocities.push_back(joint_vel_cmd_);
            publish_joint = true;
            break;
        case KEYCODE_6:
            RCLCPP_DEBUG(node_->get_logger(), "6");
            joint_msg->joint_names.push_back(joint_prefix_ + "joint6");
            joint_msg->velocities.push_back(joint_vel_cmd_);
            publish_joint = true;
            break;
        case KEYCODE_7:
            RCLCPP_DEBUG(node_->get_logger(), "7");
            joint_msg->joint_names.push_back("joint7");
            joint_msg->velocities.push_back(joint_vel_cmd_);
            publish_joint = true;
            break;
        case KEYCODE_R:
            RCLCPP_DEBUG(node_->get_logger(), "R");
            joint_vel_cmd_ *= -1;
            break;
        case KEYCODE_Q:
            RCLCPP_DEBUG(node_->get_logger(), "quit");
            return;
        }
        
        // If a key requiring a publish was pressed, publish the message now
        if (publish_twist)
        {
            _switch_command_type(1);
            twist_msg->header.stamp = node_->now();
            twist_msg->header.frame_id = planning_frame_;
            twist_pub_->publish(std::move(twist_msg));
            publish_twist = false;
        }
        else if (publish_joint)
        {
            _switch_command_type(0);
            joint_msg->header.stamp = node_->now();
            joint_msg->header.frame_id = "joint";
            joint_pub_->publish(std::move(joint_msg));
            publish_joint = false;
        }
    }
}

void exit_sig_handler(int sig)
{
  (void)sig;
  keyboard_reader_.shutdown();
  rclcpp::shutdown();
  exit(-1);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("xarm_moveit_servo_keyboard_node", node_options);

    RCLCPP_INFO(node->get_logger(), "namespace: %s", node->get_namespace());

    KeyboardServoPub keyboard_servo_pub(node);
    signal(SIGINT, exit_sig_handler);
    keyboard_servo_pub.keyLoop();
    keyboard_reader_.shutdown();
    
    // rclcpp::spin(node);
    rclcpp::shutdown();

    RCLCPP_INFO(node->get_logger(), "xarm_moveit_servo_keyboard_node over");

    return 0;
}
