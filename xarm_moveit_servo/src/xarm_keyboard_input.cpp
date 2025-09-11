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
#include <chrono>


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
#define KEYCODE_8 0x38
#define KEYCODE_9 0x39
#define KEYCODE_0 0x30

#define KEYCODE_A       0x61
#define KEYCODE_S       0x73
#define KEYCODE_D       0x64
#define KEYCODE_I       0x69
#define KEYCODE_J       0x6A
#define KEYCODE_K       0x6B
#define KEYCODE_L       0x6C
#define KEYCODE_U       0x75
#define KEYCODE_O       0x6F
#define KEYCODE_MINUS   0x2D
#define KEYCODE_EQUAL   0x3D
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
    _declare_or_get_param<std::string>(arm1_ns_, "arm1_ns", "arm1");
    _declare_or_get_param<std::string>(arm2_ns_, "arm2_ns", "arm2");
    _declare_or_get_param<std::string>(arm1_planning_frame_, "arm1_planning_frame", "arm1_link_base");
    _declare_or_get_param<std::string>(arm2_planning_frame_, "arm2_planning_frame", "arm2_link_base");
    _declare_or_get_param<std::string>(joint_prefix_, "joint_prefix", joint_prefix_);
    _declare_or_get_param<int>(dof_, "dof", dof_);
    _declare_or_get_param<int>(ros_queue_size_, "ros_queue_size", ros_queue_size_);
    _declare_or_get_param<std::string>(cartesian_command_in_topic_, "moveit_servo.cartesian_command_in_topic", cartesian_command_in_topic_);
    _declare_or_get_param<std::string>(joint_command_in_topic_, "moveit_servo.joint_command_in_topic", joint_command_in_topic_);
    _declare_or_get_param<std::string>(robot_link_command_frame_, "moveit_servo.robot_link_command_frame", robot_link_command_frame_);
    _declare_or_get_param<std::string>(ee_frame_name_, "moveit_servo.ee_frame_name", ee_frame_name_);
    _declare_or_get_param<std::string>(planning_frame_, "moveit_servo.planning_frame", planning_frame_);
    _declare_or_get_param<std::string>(elevator_cmd_vel_topic_, "elevator_cmd_vel_topic", "/elevator/cmd_vel");
    _declare_or_get_param<double>(elevator_vel_step_, "elevator_vel_step", 0.10);
    

    if (cartesian_command_in_topic_.rfind("~/", 0) == 0) {
        cartesian_command_in_topic_ = cartesian_command_in_topic_.substr(2);
    }
    if (joint_command_in_topic_.rfind("~/", 0) == 0) {
        joint_command_in_topic_ = joint_command_in_topic_.substr(2); // <- fixed length bug
    }

    // Setup pub/sub
    const auto arm1_twist_topic = "/" + arm1_ns_ + "/" + cartesian_command_in_topic_;
    const auto arm2_twist_topic = "/" + arm2_ns_ + "/" + cartesian_command_in_topic_;
    twist_pub_arm1_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(arm1_twist_topic, rclcpp::SensorDataQoS());
    twist_pub_arm2_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(arm2_twist_topic, rclcpp::SensorDataQoS());
    const auto arm1_joint_topic = "/" + arm1_ns_ + "/" + joint_command_in_topic_; // e.g. /arm1/joint_delta
    joint_pub_ = node_->create_publisher<control_msgs::msg::JointJog>(arm1_joint_topic, ros_queue_size_);
    elevator_cmd_vel_pub_ = node_->create_publisher<std_msgs::msg::Float64>(elevator_cmd_vel_topic_, 10);
    // collision_pub_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);

    // Create a service client to start the ServoServer
    // --- params for service namespace + optional start ---
    servo_srv_ns_ = "servo_node";        // becomes /<arm_ns>/servo_node/... under your namespace
    bool try_start_service = false;

    _declare_or_get_param<std::string>(servo_srv_ns_, "servo_srv_ns", servo_srv_ns_);
    _declare_or_get_param<bool>(try_start_service, "try_start_service", try_start_service);

    // --- build clients against your ServoNode ---
    switch_input_arm1_ = node_->create_client<moveit_msgs::srv::ServoCommandType>(
        "/" + arm1_ns_ + "/" + servo_srv_ns_ + "/switch_command_type");
    switch_input_arm2_ = node_->create_client<moveit_msgs::srv::ServoCommandType>(
        "/" + arm2_ns_ + "/" + servo_srv_ns_ + "/switch_command_type");

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
    arm1_command_type_ = -1;
    arm2_command_type_ = -1;

    RCLCPP_INFO(node_->get_logger(),
        "Twist pubs: %s, %s | Servo switch: /%s/%s/switch_command_type , /%s/%s/switch_command_type",
        arm1_twist_topic.c_str(), arm2_twist_topic.c_str(),
        arm1_ns_.c_str(), servo_srv_ns_.c_str(),
        arm2_ns_.c_str(), servo_srv_ns_.c_str());
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

void KeyboardServoPub::_switch_command_type(int arm_idx, int command_type)
{
  // Select per-arm state + client
  int& last_type = (arm_idx == 1) ? arm1_command_type_ : arm2_command_type_;
  auto& client   = (arm_idx == 1) ? switch_input_arm1_  : switch_input_arm2_;
  const char* arm_label = (arm_idx == 1) ? "arm1" : "arm2";

  if (command_type == last_type) return;

  if (!client || !client->wait_for_service(std::chrono::seconds(0))) {
    RCLCPP_WARN(node_->get_logger(),
                "[%s] switch_command_type service unavailable: /%s/%s/switch_command_type",
                arm_label,
                (arm_idx == 1 ? arm1_ns_.c_str() : arm2_ns_.c_str()),
                servo_srv_ns_.c_str());
    return;
  }

  // Build request locally (avoid sharing a mutable member across calls)
  auto req = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
  switch (command_type) {
    case 0: req->command_type = moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG; break;
    case 1: req->command_type = moveit_msgs::srv::ServoCommandType::Request::TWIST;     break;
    case 2: req->command_type = moveit_msgs::srv::ServoCommandType::Request::POSE;      break;
    default: return;
  }

  auto future = client->async_send_request(req);
  try {
    auto resp = future.get();
    if (resp && resp->success) {
      last_type = command_type;
      RCLCPP_INFO(node_->get_logger(), "[%s] Switched input to %s",
                  arm_label,
                  command_type == 0 ? "JOINT_JOG" :
                  command_type == 1 ? "TWIST"     : "POSE");
    } else {
      RCLCPP_WARN(node_->get_logger(), "[%s] switch_command_type call returned !success", arm_label);
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(node_->get_logger(), "[%s] switch_command_type exception: %s", arm_label, e.what());
  }
}

void KeyboardServoPub::publish_elevator_velocity(double vz)
{
  if (!elevator_cmd_vel_pub_) return;
  std_msgs::msg::Float64 msg;
  msg.data = vz;  // + = up, - = down (adjust if your axis is inverted)
  elevator_cmd_vel_pub_->publish(msg);
}
// NEW: publish one TwistStamped for a chosen arm (translation only)
void KeyboardServoPub::publish_twist_for_arm(int arm_idx, double dx, double dy, double dz)
{
  // Ensure the target Servo is listening for TWIST
  _switch_command_type(arm_idx, 1); // 1 = TWIST

  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = node_->now();
  msg.header.frame_id = (arm_idx == 1) ? arm1_planning_frame_ : arm2_planning_frame_;
  msg.twist.linear.x = dx;
  msg.twist.linear.y = dy;
  msg.twist.linear.z = dz;

  if (arm_idx == 1) {
    twist_pub_arm1_->publish(msg);
  } else {
    twist_pub_arm2_->publish(msg);
  }
}

void KeyboardServoPub::keyLoop()
{
    char c;
    bool publish_joint = false;

    std::thread{ std::bind(&KeyboardServoPub::spin, this) }.detach();

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Arm1 (WASD = X/Y, Q/E = Z)");
    puts("Arm2 (IJKL = X/Y, U/O = Z)");
    puts("Joint jog: 1..6 (prefix from joint_prefix), 'R' flips direction");
    puts("Arrow Up/Down = Elevator velocity (+/-)");

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

        auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

        // Use read key-press
        switch (c)
        {
        case KEYCODE_W:  publish_twist_for_arm(1, +linear_pos_cmd_,  0.0,              0.0); break;
        case KEYCODE_S:  publish_twist_for_arm(1, -linear_pos_cmd_,  0.0,              0.0); break;
        case KEYCODE_A:  publish_twist_for_arm(1,  0.0,             +linear_pos_cmd_,  0.0); break;
        case KEYCODE_D:  publish_twist_for_arm(1,  0.0,             -linear_pos_cmd_,  0.0); break;
        case KEYCODE_Q:  publish_twist_for_arm(1,  0.0,              0.0,             +linear_pos_cmd_); break;
        case KEYCODE_E:  publish_twist_for_arm(1,  0.0,              0.0,             -linear_pos_cmd_); break;

        case KEYCODE_I:  publish_twist_for_arm(2, +linear_pos_cmd_,  0.0,              0.0); break;
        case KEYCODE_K:  publish_twist_for_arm(2, -linear_pos_cmd_,  0.0,              0.0); break;
        case KEYCODE_J:  publish_twist_for_arm(2,  0.0,             +linear_pos_cmd_,  0.0); break;
        case KEYCODE_L:  publish_twist_for_arm(2,  0.0,             -linear_pos_cmd_,  0.0); break;
        case KEYCODE_U:  publish_twist_for_arm(2,  0.0,              0.0,             +linear_pos_cmd_); break;
        case KEYCODE_O:  publish_twist_for_arm(2,  0.0,              0.0,             -linear_pos_cmd_); break;
        case KEYCODE_UP:
          publish_elevator_velocity(+elevator_vel_step_);
          break;
        case KEYCODE_DOWN:
          publish_elevator_velocity(-elevator_vel_step_);
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
        }
        
        // If a key requiring a publish was pressed, publish the message now
        if (publish_joint)
        {
          _switch_command_type(1, 0);  // arm 1, JOINT_JOG
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
