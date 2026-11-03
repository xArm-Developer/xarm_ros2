// Copyright (c) 2026 Hivebotics
// BSD License

#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

namespace xarm2_ft_interface
{

/**
 * @brief A ros2_control SensorInterface that subscribes to a WrenchStamped topic
 *        and exposes force/torque values as hardware state interfaces.
 *
 * This provides a unified FT sensor hardware interface for both simulation
 * (Ignition bridge publishes WrenchStamped) and real hardware (xArm API
 * publishes WrenchStamped on ufactory/uf_ftsensor_ext_states).
 *
 * URDF ros2_control parameters:
 *   - wrench_topic (string): The topic to subscribe to. Default: "/ft_sensor/wrench"
 *
 * The sensor must be declared in the URDF with name "force_torque_sensor" and
 * the 6 standard state interfaces: force.x, force.y, force.z, torque.x, torque.y, torque.z
 *
 * NOTE: Uses SystemInterface (not SensorInterface) so that ign_ros2_control's
 * Gazebo plugin can load it via its standard hardware fallback mechanism.
 * No command interfaces are exported — this is effectively read-only.
 */
class WrenchTopicSensorHardware : public hardware_interface::SystemInterface
{
public:
  WrenchTopicSensorHardware() = default;
  ~WrenchTopicSensorHardware() override = default;

  // Lifecycle
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  // Interface export
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Read from the subscribed topic
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // No-op write (read-only sensor)
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

  // Internal ROS node for subscribing
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;

  // Topic name (read from URDF param, overridable as ROS param)
  std::string wrench_topic_;

  // Sensor name from URDF
  std::string sensor_name_;

  // State interface storage: [force.x, force.y, force.z, torque.x, torque.y, torque.z]
  std::array<double, 6> hw_sensor_states_{};

  // Latest received values (written by callback, read by read())
  std::array<double, 6> latest_wrench_{};
  std::mutex wrench_mutex_;

  // Flag to track if we've received at least one message
  std::atomic<bool> data_received_{false};
};

}  // namespace xarm2_ft_interface
