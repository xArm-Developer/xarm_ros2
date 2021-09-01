/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#include "xarm_moveit_servo/xarm_joystick.h"


namespace xarm_moveit_servo
{
// Enums for button names -> axis/button array index
// For XBOX 1 controller
enum Axis
{
    LEFT_STICK_X = 0,
    LEFT_STICK_Y = 1,
    LEFT_TRIGGER = 2,
    RIGHT_STICK_X = 3,
    RIGHT_STICK_Y = 4,
    RIGHT_TRIGGER = 5,
    D_PAD_X = 6,
    D_PAD_Y = 7
};
enum Button
{
    A = 0,
    B = 1,
    X = 2,
    Y = 3,
    LEFT_BUMPER = 4,
    RIGHT_BUMPER = 5,
    CHANGE_VIEW = 6,
    MENU = 7,
    HOME = 8,
    LEFT_STICK_CLICK = 9,
    RIGHT_STICK_CLICK = 10
};

// Some axes have offsets (e.g. the default trigger position is 1.0 not 0)
// This will map the default values for the axes
std::map<Axis, double> AXIS_DEFAULTS = { { LEFT_TRIGGER, 1.0 }, { RIGHT_TRIGGER, 1.0 } };
std::map<Button, double> BUTTON_DEFAULTS;
    
JoyToServoPub::JoyToServoPub(const rclcpp::NodeOptions& options)
: Node("joy_to_twist_publisher", options), 
dof_(7), ros_queue_size_(10), is_initialized_(false),
joy_topic_("/joy"),
cartesian_command_in_topic_("/servo_server/delta_twist_cmds"), 
joint_command_in_topic_("/servo_server/delta_joint_cmds"), 
robot_link_command_frame_("link_base"), 
ee_frame_name_("link_eef"),
planning_frame_("link_base")
{
    // init parameter from node
    declareOrGetParam<int>(dof_, "dof", dof_);
    declareOrGetParam<int>(ros_queue_size_, "ros_queue_size", ros_queue_size_);
    declareOrGetParam<std::string>(joy_topic_, "joy_topic", joy_topic_);
    declareOrGetParam<std::string>(cartesian_command_in_topic_, "moveit_servo.cartesian_command_in_topic", cartesian_command_in_topic_);
    declareOrGetParam<std::string>(joint_command_in_topic_, "moveit_servo.joint_command_in_topic", joint_command_in_topic_);
    declareOrGetParam<std::string>(robot_link_command_frame_, "moveit_servo.robot_link_command_frame", robot_link_command_frame_);
    declareOrGetParam<std::string>(ee_frame_name_, "moveit_servo.ee_frame_name", ee_frame_name_);
    declareOrGetParam<std::string>(planning_frame_, "moveit_servo.planning_frame", planning_frame_);

    if (cartesian_command_in_topic_.rfind("~/", 0) == 0) {
        cartesian_command_in_topic_ = "/servo_server/" + cartesian_command_in_topic_.substr(2, cartesian_command_in_topic_.length());
    }
    if (joint_command_in_topic_.rfind("~/", 0) == 0) {
        joint_command_in_topic_ = "/servo_server/" + joint_command_in_topic_.substr(2, cartesian_command_in_topic_.length());
    }

    // Setup pub/sub
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(joy_topic_, ros_queue_size_, std::bind(&JoyToServoPub::joyCB, this, std::placeholders::_1));

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(cartesian_command_in_topic_, ros_queue_size_);
    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(joint_command_in_topic_, ros_queue_size_);
    // collision_pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);

    // Create a service client to start the ServoServer
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_server/start_servo");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
}

template <typename T>
void JoyToServoPub::declareOrGetParam(T& output_value, const std::string& param_name, const T default_value)
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

bool JoyToServoPub::convertJoyToCmd(const std::vector<float>& axes, const std::vector<int>& buttons,
                     std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist,
                     std::unique_ptr<control_msgs::msg::JointJog>& joint)
{
    // Give joint jogging priority because it is only buttons
    // If any joint jog command is requested, we are only publishing joint commands
    if (buttons[A] || buttons[B] || buttons[X] || buttons[Y] || axes[D_PAD_X] || axes[D_PAD_Y])
    {
        // Map the D_PAD to the proximal joints
        joint->joint_names.push_back("joint1");
        joint->velocities.push_back(axes[D_PAD_X] * 1);
        joint->joint_names.push_back("joint2");
        joint->velocities.push_back(axes[D_PAD_Y] * 1);

        // Map the diamond to the distal joints
        joint->joint_names.push_back("joint" + std::to_string(dof_));
        joint->velocities.push_back((buttons[B] - buttons[X]) * 1);
        joint->joint_names.push_back("joint" + std::to_string(dof_ - 1));
        joint->velocities.push_back((buttons[Y] - buttons[A]) * 1);
        return false;
    }

    // The bread and butter: map buttons to twist commands
    twist->twist.linear.z = axes[RIGHT_STICK_Y];
    twist->twist.linear.y = axes[RIGHT_STICK_X];

    double lin_x_right = -0.5 * (axes[RIGHT_TRIGGER] - AXIS_DEFAULTS.at(RIGHT_TRIGGER));
    double lin_x_left = 0.5 * (axes[LEFT_TRIGGER] - AXIS_DEFAULTS.at(LEFT_TRIGGER));
    twist->twist.linear.x = lin_x_right + lin_x_left;

    twist->twist.angular.y = axes[LEFT_STICK_Y];
    twist->twist.angular.x = -axes[LEFT_STICK_X];

    double roll_positive = buttons[RIGHT_BUMPER];
    double roll_negative = -1 * (buttons[LEFT_BUMPER]);
    twist->twist.angular.z = roll_positive + roll_negative;

    if (abs(twist->twist.linear.x) < 0.05) {
        twist->twist.linear.x = 0;
    }
    if (abs(twist->twist.linear.y) < 0.05) {
        twist->twist.linear.y = 0;
    }
    if (abs(twist->twist.linear.z) < 0.05) {
        twist->twist.linear.z = 0;
    }
    if (abs(twist->twist.angular.x) < 0.05) {
        twist->twist.angular.x = 0;
    }
    if (abs(twist->twist.angular.y) < 0.05) {
        twist->twist.angular.y = 0;
    }
    if (abs(twist->twist.angular.z) < 0.05) {
        twist->twist.angular.z = 0;
    }

    // RCLCPP_INFO(this->get_logger(), "linear=[%f, %f, %f], angular=[%f, %f, %f]", 
    //     twist->twist.linear.x, twist->twist.linear.y, twist->twist.linear.z,
    //     twist->twist.angular.x, twist->twist.angular.y, twist->twist.angular.z);

    return true;
}

void JoyToServoPub::joyCB(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // Create the messages we might publish
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    if (msg->buttons[CHANGE_VIEW] && planning_frame_ == ee_frame_name_) {
        planning_frame_ = robot_link_command_frame_;
    }
    else if (msg->buttons[MENU] && planning_frame_ == robot_link_command_frame_) {
        planning_frame_ = ee_frame_name_;
    }

    if (convertJoyToCmd(msg->axes, msg->buttons, twist_msg, joint_msg))
    {
        // publish the TwistStamped
        twist_msg->header.frame_id = planning_frame_;
        twist_msg->header.stamp = this->now();
        twist_pub_->publish(std::move(twist_msg));
    }
    else
    {
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
