/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#include "xarm_moveit_servo/xarm_joystick_input.h"


namespace xarm_moveit_servo
{

enum JOYSTICK_TYPE
{
    JOYSTICK_XBOX360_WIRED = 1,
    JOYSTICK_XBOX360_WIRELESS = 2,
    JOYSTICK_SPACEMOUSE_WIRELESS = 3
};

enum XBOX360_WIRED_CONTROLLER_AXIS
{
    XBOX360_WIRED_LEFT_STICK_LR = 0,
    XBOX360_WIRED_LEFT_STICK_FB = 1,
    XBOX360_WIRED_LEFT_TRIGGER = 2,
    XBOX360_WIRED_RIGHT_STICK_LR = 3,
    XBOX360_WIRED_RIGHT_STICK_FB = 4,
    XBOX360_WIRED_RIGHT_TRIGGER = 5,
    XBOX360_WIRED_CROSS_KEY_LR = 6,
    XBOX360_WIRED_CROSS_KEY_FB = 7
};

enum XBOX360_WIRELESS_CONTROLLER_AXIS
{
    XBOX360_WIRELESS_LEFT_STICK_LR = 0,
    XBOX360_WIRELESS_LEFT_STICK_FB = 1,
    XBOX360_WIRELESS_RIGHT_STICK_LR = 2,
    XBOX360_WIRELESS_RIGHT_STICK_FB = 3,
    XBOX360_WIRELESS_LEFT_TRIGGER = 4,
    XBOX360_WIRELESS_RIGHT_TRIGGER = 5,
    XBOX360_WIRELESS_CROSS_KEY_LR = 6,
    XBOX360_WIRELESS_CROSS_KEY_FB = 7
};

enum XBOX360_CONTROLLER_BUTTON
{
    XBOX360_BTN_A = 0,
    XBOX360_BTN_B = 1,
    XBOX360_BTN_X = 2,
    XBOX360_BTN_Y = 3,
    XBOX360_BTN_LB = 4,
    XBOX360_BTN_RB = 5,
    XBOX360_BTN_BACK = 6,
    XBOX360_BTN_START = 7,
    XBOX360_BTN_POWER = 8,
    XBOX360_BTN_STICK_LEFT = 9,
    XBOX360_BTN_STICK_RIGHT = 10
};

enum SPACEMOUSE_WIRELESS_AXIS
{
    SPM_STICK_Y = 0,
    SPM_STICK_X = 1,
    SPM_STICK_Z = 2,
    SPM_STICK_PITCH = 3,
    SPM_STICK_ROLL = 4,
    SPM_STICK_YAW = 5
};

enum SPACEMOUSE_WIRELESS_BUTTON
{
    SPM_BTN_LEFT = 0,
    SPM_BTN_RIGHT = 1
};

JoyToServoPub::JoyToServoPub(const rclcpp::NodeOptions& options)
  : Node("joy_to_twist_publisher", options), 
    dof_(7), ros_queue_size_(10), joystick_type_(JOYSTICK_XBOX360_WIRED), initialized_status_(10),
    joy_topic_("/joy"),
    cartesian_command_in_topic_("/servo_server/delta_twist_cmds"), 
    joint_command_in_topic_("/servo_server/delta_joint_cmds"), 
    robot_link_command_frame_("link_base"), 
    ee_frame_name_("link_eef"),
    planning_frame_("link_base")
{
    // init parameter from node
    _declare_or_get_param<int>(dof_, "dof", dof_);
    _declare_or_get_param<int>(ros_queue_size_, "ros_queue_size", ros_queue_size_);
    _declare_or_get_param<int>(joystick_type_, "joystick_type", joystick_type_);
    _declare_or_get_param<std::string>(joy_topic_, "joy_topic", joy_topic_);
    _declare_or_get_param<std::string>(cartesian_command_in_topic_, "moveit_servo.cartesian_command_in_topic", cartesian_command_in_topic_);
    _declare_or_get_param<std::string>(joint_command_in_topic_, "moveit_servo.joint_command_in_topic", joint_command_in_topic_);
    _declare_or_get_param<std::string>(robot_link_command_frame_, "moveit_servo.robot_link_command_frame", robot_link_command_frame_);
    _declare_or_get_param<std::string>(ee_frame_name_, "moveit_servo.ee_frame_name", ee_frame_name_);
    _declare_or_get_param<std::string>(planning_frame_, "moveit_servo.planning_frame", planning_frame_);

    if (cartesian_command_in_topic_.rfind("~/", 0) == 0) {
        cartesian_command_in_topic_ = "/servo_server/" + cartesian_command_in_topic_.substr(2, cartesian_command_in_topic_.length());
    }
    if (joint_command_in_topic_.rfind("~/", 0) == 0) {
        joint_command_in_topic_ = "/servo_server/" + joint_command_in_topic_.substr(2, joint_command_in_topic_.length());
    }

    // Setup pub/sub
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(joy_topic_, ros_queue_size_, std::bind(&JoyToServoPub::_joy_callback, this, std::placeholders::_1));

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(cartesian_command_in_topic_, ros_queue_size_);
    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(joint_command_in_topic_, ros_queue_size_);
    // collision_pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);

    // Create a service client to start the ServoServer
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_server/start_servo");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
}

template <typename T>
void JoyToServoPub::_declare_or_get_param(T& output_value, const std::string& param_name, const T default_value)
{
    try
    {
        if (this->has_parameter(param_name))
        {
            this->get_parameter<T>(param_name, output_value);
        }
        else
        {
            output_value = this->declare_parameter<T>(param_name, default_value);
        }
    }
    catch (const rclcpp::exceptions::InvalidParameterTypeException& e)
    {
        RCLCPP_WARN_STREAM(this->get_logger(), "InvalidParameterTypeException(" << param_name << "): " << e.what());
        RCLCPP_ERROR_STREAM(this->get_logger(), "Error getting parameter \'" << param_name << "\', check parameter type in YAML file");
        throw e;
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Found parameter - " << param_name << ": " << output_value);
}

void JoyToServoPub::_filter_twist_msg(std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist, double val)
{
    if (abs(twist->twist.linear.x) < val) {
        twist->twist.linear.x = 0;
    }
    if (abs(twist->twist.linear.y) < val) {
        twist->twist.linear.y = 0;
    }
    if (abs(twist->twist.linear.z) < val) {
        twist->twist.linear.z = 0;
    }
    if (abs(twist->twist.angular.x) < val) {
        twist->twist.angular.x = 0;
    }
    if (abs(twist->twist.angular.y) < val) {
        twist->twist.angular.y = 0;
    }
    if (abs(twist->twist.angular.z) < val) {
        twist->twist.angular.z = 0;
    }
}

bool JoyToServoPub::_convert_xbox360_joy_to_cmd(
    const std::vector<float>& axes, const std::vector<int>& buttons,
    std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist,
    std::unique_ptr<control_msgs::msg::JointJog>& joint)
{
    // xbox360 wired axis
    int left_stick_lr = XBOX360_WIRED_LEFT_STICK_LR;
    int left_stick_fb = XBOX360_WIRED_LEFT_STICK_FB;
    int right_stick_lr = XBOX360_WIRED_RIGHT_STICK_LR;
    int right_stick_fb = XBOX360_WIRED_RIGHT_STICK_FB;
    int left_trigger = XBOX360_WIRED_LEFT_TRIGGER;
    int right_trigger = XBOX360_WIRED_RIGHT_TRIGGER;
    int cross_key_lr = XBOX360_WIRED_CROSS_KEY_LR;
    int cross_key_fb = XBOX360_WIRED_CROSS_KEY_FB;
    
    if (joystick_type_ == JOYSTICK_XBOX360_WIRELESS) {
        // xbox360 wireless axis
        left_stick_lr = XBOX360_WIRELESS_LEFT_STICK_LR;
        left_stick_fb = XBOX360_WIRELESS_LEFT_STICK_FB;
        right_stick_lr = XBOX360_WIRELESS_RIGHT_STICK_LR;
        right_stick_fb = XBOX360_WIRELESS_RIGHT_STICK_FB;
        left_trigger = XBOX360_WIRELESS_LEFT_TRIGGER;
        right_trigger = XBOX360_WIRELESS_RIGHT_TRIGGER;
        cross_key_lr = XBOX360_WIRELESS_CROSS_KEY_LR;
        cross_key_fb = XBOX360_WIRELESS_CROSS_KEY_FB;
    }

    if (buttons[XBOX360_BTN_BACK] && planning_frame_ == ee_frame_name_) {
        planning_frame_ = robot_link_command_frame_;
    }
    else if (buttons[XBOX360_BTN_START] && planning_frame_ == robot_link_command_frame_) {
        planning_frame_ = ee_frame_name_;
    }
    
    if (buttons[XBOX360_BTN_A] || buttons[XBOX360_BTN_B] 
        || buttons[XBOX360_BTN_X] || buttons[XBOX360_BTN_Y] 
        || axes[cross_key_lr] || axes[cross_key_fb])
    {
        // Map the D_PAD to the proximal joints
        joint->joint_names.push_back("joint1");
        joint->velocities.push_back(axes[cross_key_lr] * 1);
        joint->joint_names.push_back("joint2");
        joint->velocities.push_back(axes[cross_key_fb] * 1);

        // Map the diamond to the distal joints
        joint->joint_names.push_back("joint" + std::to_string(dof_));
        joint->velocities.push_back((buttons[XBOX360_BTN_B] - buttons[XBOX360_BTN_X]) * 1);
        joint->joint_names.push_back("joint" + std::to_string(dof_ - 1));
        joint->velocities.push_back((buttons[XBOX360_BTN_Y] - buttons[XBOX360_BTN_A]) * 1);
        return false;
    }

    // The bread and butter: map buttons to twist commands
    twist->twist.linear.x = axes[left_stick_fb];
    twist->twist.linear.y = axes[left_stick_lr];
    twist->twist.linear.z = -1 * (axes[left_trigger] - axes[right_trigger]);
    twist->twist.angular.y = axes[right_stick_fb];
    twist->twist.angular.x = axes[right_stick_lr];
    twist->twist.angular.z = buttons[XBOX360_BTN_LB] - buttons[XBOX360_BTN_RB];

    return true;
}

bool JoyToServoPub::_convert_spacemouse_wireless_joy_to_cmd(const std::vector<float>& axes, const std::vector<int>& buttons,
    std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist)
{
    twist->twist.linear.x = axes[SPM_STICK_X];
    twist->twist.linear.y = axes[SPM_STICK_Y];
    twist->twist.linear.z = axes[SPM_STICK_Z];
    
    twist->twist.angular.x = axes[SPM_STICK_ROLL];
    twist->twist.angular.y = axes[SPM_STICK_PITCH];
    twist->twist.angular.z = axes[SPM_STICK_YAW];

    if (buttons[SPM_BTN_LEFT]) {
        twist->twist.angular.x = 0;
        twist->twist.angular.y = 0;
        twist->twist.angular.z = 0;
    }
    if (buttons[SPM_BTN_RIGHT]) {
        twist->twist.linear.x = 0;
        twist->twist.linear.y = 0;
        twist->twist.linear.z = 0;
    }
    return true;
}

void JoyToServoPub::_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // std::string axes_str = "[ ";
    // for (int i = 0; i < msg->axes.size(); i++) { 
    //     axes_str += std::to_string(msg->axes[i]); 
    //     axes_str += " ";
    // }
    // axes_str += "]";
    // RCLCPP_INFO(this->get_logger(), "axes_str: %s", axes_str.c_str());
    // return;

    // Create the messages we might publish
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    if (dof_ == 7 && initialized_status_) {
        initialized_status_ -= 1;
        joint_msg->joint_names.push_back("joint1");
        joint_msg->velocities.push_back(initialized_status_ > 0 ? 0.01 : 0);

        joint_msg->header.stamp = this->now();
        joint_msg->header.frame_id = "joint1";
        joint_pub_->publish(std::move(joint_msg));

        return;
    }

    bool pub_twist = false;

    switch (joystick_type_) {
        case JOYSTICK_XBOX360_WIRED: // xbox360 wired
        case JOYSTICK_XBOX360_WIRELESS: // xbox360 wireless
            if (msg->axes.size() != 8 || msg->buttons.size() != 11)
                return;
            pub_twist = _convert_xbox360_joy_to_cmd(msg->axes, msg->buttons, twist_msg, joint_msg);
            break;
        case JOYSTICK_SPACEMOUSE_WIRELESS: // spacemouse wireless
            if (msg->axes.size() != 6 || msg->buttons.size() != 2)
                return;
            pub_twist = _convert_spacemouse_wireless_joy_to_cmd(msg->axes, msg->buttons, twist_msg);
            break;
        default:
            return;
    }
    if (pub_twist) {
        // publish the TwistStamped
        _filter_twist_msg(twist_msg, 0.2);
        // RCLCPP_INFO(this->get_logger(), "linear=[%f, %f, %f], angular=[%f, %f, %f]", 
        //     twist_msg->twist.linear.x, twist_msg->twist.linear.y, twist_msg->twist.linear.z,
        //     twist_msg->twist.angular.x, twist_msg->twist.angular.y, twist_msg->twist.angular.z);
        twist_msg->header.frame_id = planning_frame_;
        twist_msg->header.stamp = this->now();
        twist_pub_->publish(std::move(twist_msg));
    }
    else {
        // publish the JointJog
        joint_msg->header.stamp = this->now();
        joint_msg->header.frame_id = "joint";
        joint_pub_->publish(std::move(joint_msg));
    }
}

}

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(xarm_moveit_servo::JoyToServoPub)
