/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#include <signal.h>
#include <stdio.h>
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
cartesian_command_in_topic_("/servo_server/delta_twist_cmds"), 
joint_command_in_topic_("/servo_server/delta_joint_cmds"), 
robot_link_command_frame_("link_base"), 
ee_frame_name_("link_eef"),
planning_frame_("link_base"),
joint_vel_cmd_(1.0),
linear_pos_cmd_(0.5)
{
    node_ = node;
    // init parameter from node
    _declare_or_get_param<int>(dof_, "dof", dof_);
    _declare_or_get_param<int>(ros_queue_size_, "ros_queue_size", ros_queue_size_);
    _declare_or_get_param<std::string>(cartesian_command_in_topic_, "moveit_servo.cartesian_command_in_topic", cartesian_command_in_topic_);
    _declare_or_get_param<std::string>(joint_command_in_topic_, "moveit_servo.joint_command_in_topic", joint_command_in_topic_);
    _declare_or_get_param<std::string>(robot_link_command_frame_, "moveit_servo.robot_link_command_frame", robot_link_command_frame_);
    _declare_or_get_param<std::string>(ee_frame_name_, "moveit_servo.ee_frame_name", ee_frame_name_);
    _declare_or_get_param<std::string>(planning_frame_, "moveit_servo.planning_frame", planning_frame_);

    if (cartesian_command_in_topic_.rfind("~/", 0) == 0) {
        cartesian_command_in_topic_ = "/servo_server/" + cartesian_command_in_topic_.substr(2, cartesian_command_in_topic_.length());
    }
    if (joint_command_in_topic_.rfind("~/", 0) == 0) {
        joint_command_in_topic_ = "/servo_server/" + joint_command_in_topic_.substr(2, cartesian_command_in_topic_.length());
    }

    // Setup pub/sub
    twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(cartesian_command_in_topic_, ros_queue_size_);
    joint_pub_ = node_->create_publisher<control_msgs::msg::JointJog>(joint_command_in_topic_, ros_queue_size_);
    // collision_pub_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);

    // Create a service client to start the ServoServer
    servo_start_client_ = node_->create_client<std_srvs::srv::Trigger>("/servo_server/start_servo");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    
    RCLCPP_INFO(node_->get_logger(), "\n*********dof=%d, ros_queue_size=%d\n", dof_, ros_queue_size_);
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
            joint_msg->joint_names.push_back("joint1");
            joint_msg->velocities.push_back(joint_vel_cmd_);
            publish_joint = true;
            break;
        case KEYCODE_2:
            RCLCPP_DEBUG(node_->get_logger(), "2");
            joint_msg->joint_names.push_back("joint2");
            joint_msg->velocities.push_back(joint_vel_cmd_);
            publish_joint = true;
            break;
        case KEYCODE_3:
            RCLCPP_DEBUG(node_->get_logger(), "3");
            joint_msg->joint_names.push_back("joint3");
            joint_msg->velocities.push_back(joint_vel_cmd_);
            publish_joint = true;
            break;
        case KEYCODE_4:
            RCLCPP_DEBUG(node_->get_logger(), "4");
            joint_msg->joint_names.push_back("joint4");
            joint_msg->velocities.push_back(joint_vel_cmd_);
            publish_joint = true;
            break;
        case KEYCODE_5:
            RCLCPP_DEBUG(node_->get_logger(), "5");
            joint_msg->joint_names.push_back("joint5");
            joint_msg->velocities.push_back(joint_vel_cmd_);
            publish_joint = true;
            break;
        case KEYCODE_6:
            RCLCPP_DEBUG(node_->get_logger(), "6");
            joint_msg->joint_names.push_back("joint6");
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
            twist_msg->header.stamp = node_->now();
            twist_msg->header.frame_id = planning_frame_;
            twist_pub_->publish(std::move(twist_msg));
            publish_twist = false;
        }
        else if (publish_joint)
        {
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
