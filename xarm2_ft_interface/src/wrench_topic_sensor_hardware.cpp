// Copyright (c) 2026 Hivebotics
// BSD License

#include "xarm2_ft_interface/wrench_topic_sensor_hardware.hpp"

#include <chrono>
#include <limits>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/qos.hpp>

namespace xarm2_ft_interface
{

hardware_interface::CallbackReturn WrenchTopicSensorHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read the wrench topic name from URDF <param> (default: /ft_sensor/wrench)
  if (info_.hardware_parameters.count("wrench_topic")) {
    wrench_topic_ = info_.hardware_parameters.at("wrench_topic");
  } else {
    wrench_topic_ = "/ft_sensor/wrench";
  }

  // Validate that exactly one sensor is declared
  if (info_.sensors.size() != 1) {
    RCLCPP_FATAL(
      rclcpp::get_logger("WrenchTopicSensorHardware"),
      "Expected exactly 1 sensor in URDF, got %zu", info_.sensors.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  sensor_name_ = info_.sensors[0].name;

  // Validate that the sensor has exactly 6 state interfaces
  if (info_.sensors[0].state_interfaces.size() != 6) {
    RCLCPP_FATAL(
      rclcpp::get_logger("WrenchTopicSensorHardware"),
      "Expected 6 state interfaces for sensor '%s', got %zu",
      sensor_name_.c_str(), info_.sensors[0].state_interfaces.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize state arrays to zero
  hw_sensor_states_.fill(0.0);
  latest_wrench_.fill(0.0);

  RCLCPP_INFO(
    rclcpp::get_logger("WrenchTopicSensorHardware"),
    "Initialized FT sensor hardware interface for sensor '%s', topic: '%s'",
    sensor_name_.c_str(), wrench_topic_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WrenchTopicSensorHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Create an internal ROS node for the subscription
  rclcpp::NodeOptions node_options;
  node_options.arguments({"--ros-args", "-r", "__node:=ft_sensor_hardware_node"});
  node_ = rclcpp::Node::make_shared("_ft_sensor_hw", "", node_options);

  // Declare a ROS parameter so the topic can be overridden at runtime
  node_->declare_parameter<std::string>("wrench_topic", wrench_topic_);
  wrench_topic_ = node_->get_parameter("wrench_topic").as_string();

  // Create subscription with SensorDataQoS for best-effort / volatile compatibility
  wrench_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
    wrench_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&WrenchTopicSensorHardware::wrench_callback, this, std::placeholders::_1));

  RCLCPP_INFO(
    node_->get_logger(),
    "FT sensor hardware configured — subscribing to '%s'", wrench_topic_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WrenchTopicSensorHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset sensor values on activation
  hw_sensor_states_.fill(0.0);
  {
    std::lock_guard<std::mutex> lock(wrench_mutex_);
    latest_wrench_.fill(0.0);
  }
  data_received_ = false;

  RCLCPP_INFO(
    rclcpp::get_logger("WrenchTopicSensorHardware"),
    "FT sensor hardware activated, waiting for data on '%s'", wrench_topic_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WrenchTopicSensorHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("WrenchTopicSensorHardware"),
    "FT sensor hardware deactivated");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WrenchTopicSensorHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  wrench_sub_.reset();
  node_.reset();

  RCLCPP_INFO(
    rclcpp::get_logger("WrenchTopicSensorHardware"),
    "FT sensor hardware cleaned up");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
WrenchTopicSensorHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Export the 6 FT state interfaces in the order expected by the controller:
  // force.x, force.y, force.z, torque.x, torque.y, torque.z
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(sensor_name_, "force.x", &hw_sensor_states_[0]));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(sensor_name_, "force.y", &hw_sensor_states_[1]));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(sensor_name_, "force.z", &hw_sensor_states_[2]));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(sensor_name_, "torque.x", &hw_sensor_states_[3]));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(sensor_name_, "torque.y", &hw_sensor_states_[4]));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(sensor_name_, "torque.z", &hw_sensor_states_[5]));

  return state_interfaces;
}

hardware_interface::return_type WrenchTopicSensorHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Process any pending subscription callbacks
  if (node_) {
    rclcpp::spin_some(node_);
  }

  // Copy the latest received wrench values to the state interface storage
  {
    std::lock_guard<std::mutex> lock(wrench_mutex_);
    hw_sensor_states_ = latest_wrench_;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type WrenchTopicSensorHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // No-op: this is a read-only sensor interface
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::CommandInterface>
WrenchTopicSensorHardware::export_command_interfaces()
{
  // No command interfaces — read-only sensor
  return {};
}

void WrenchTopicSensorHardware::wrench_callback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(wrench_mutex_);
  latest_wrench_[0] = msg->wrench.force.x;
  latest_wrench_[1] = msg->wrench.force.y;
  latest_wrench_[2] = msg->wrench.force.z;
  latest_wrench_[3] = msg->wrench.torque.x;
  latest_wrench_[4] = msg->wrench.torque.y;
  latest_wrench_[5] = msg->wrench.torque.z;

  if (!data_received_.exchange(true)) {
    RCLCPP_INFO(
      node_->get_logger(),
      "FT sensor: first wrench data received on '%s'", wrench_topic_.c_str());
  }
}

}  // namespace xarm2_ft_interface

PLUGINLIB_EXPORT_CLASS(
  xarm2_ft_interface::WrenchTopicSensorHardware,
  hardware_interface::SystemInterface)
