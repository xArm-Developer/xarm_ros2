/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
           Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#ifndef __XARM_HARDWARE_INTERFACE_H__
#define __XARM_HARDWARE_INTERFACE_H__

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
#include "xarm_api/xarm_ros_client.h"


namespace xarm_control
{
    class XArmHW : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(XArmHW)

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
        int curr_state_;
		int curr_mode_;
		int curr_err_;
        bool initial_write_;
        bool velocity_control_;
        std::vector<float> prev_cmds_float_;
        std::vector<float> cmds_float_;
        std::vector<double> position_cmds_;
        std::vector<double> velocity_cmds_;
        std::vector<double> position_states_;
        std::vector<double> velocity_states_;

        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<rclcpp::Node> client_node_;
        xarm_api::XArmROSClient xarm_client_;

        rclcpp::Time cur_time_;
        rclcpp::Time last_time_;

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        rclcpp::Subscription<xarm_msgs::msg::RobotMsg>::SharedPtr xarm_state_sub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trajectory_execution_event_sub_;

        void _receive_event(const std_msgs::msg::String::SharedPtr event);
        void _joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr states);
        void _xarm_states_callback(const xarm_msgs::msg::RobotMsg::SharedPtr states);
        bool _check_cmds_is_change(std::vector<float> prev, std::vector<float> cur, double threshold = 0.0001);
    };
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(xarm_control::XArmHW, hardware_interface::SystemInterface)

#endif // __XARM_HARDWARE_INTERFACE_H__