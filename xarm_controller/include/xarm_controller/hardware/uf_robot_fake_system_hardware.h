/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
           Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#ifndef __UF_ROBOT_FAKE_SYSTEM_HARDWARE_INTERFACE_H__
#define __UF_ROBOT_FAKE_SYSTEM_HARDWARE_INTERFACE_H__

#include <vector>
#include <thread>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/visibility_control.h"


namespace uf_robot_hardware
{
    class UFRobotFakeSystemHardware : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(UFRobotFakeSystemHardware)

        hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type start() override;

        hardware_interface::return_type stop() override;

        hardware_interface::return_type read() override;

        hardware_interface::return_type write() override;

        hardware_interface::status get_status() const final
        {
            return status_;
        }
        std::string get_name() const final
        {
            return info_.name;
        }

    protected:
        hardware_interface::HardwareInfo info_;
        hardware_interface::status status_;
    
    private:
        std::vector<double> position_cmds_;
        std::vector<double> velocity_cmds_;
        std::vector<double> position_states_;
        std::vector<double> velocity_states_;

        std::shared_ptr<rclcpp::Node> node_;
        std::thread node_thread_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        sensor_msgs::msg::JointState joint_state_msg_;
    };
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(uf_robot_hardware::UFRobotFakeSystemHardware, hardware_interface::SystemInterface)

#endif // __UF_ROBOT_FAKE_SYSTEM_HARDWARE_INTERFACE_H__