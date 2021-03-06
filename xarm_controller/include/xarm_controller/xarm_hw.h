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
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/load_start_controller.hpp"
#include "controller_manager_msgs/srv/reload_controller_libraries.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
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
        int read_code_;
        int write_code_;

        std::string hw_ns_;

        std::vector<float> prev_cmds_float_;
        std::vector<float> cmds_float_;
        std::vector<double> position_cmds_;
        std::vector<double> velocity_cmds_;
        std::vector<double> position_states_;
        std::vector<double> velocity_states_;

        bool velocity_control_;
        bool initialized_;
        bool read_ready_;
        bool reload_controller_;

        long int read_cnts_;
        long int read_failed_cnts_;
        double read_max_time_;
        double read_total_time_;
        std::vector<float> prev_read_angles_;
        std::vector<float> curr_read_angles_;
        
        rclcpp::Time prev_read_time_;
        rclcpp::Time curr_read_time_;
        rclcpp::Time curr_write_time_;
        rclcpp::Time prev_write_time_;

        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<rclcpp::Node> client_node_;
        xarm_api::XArmROSClient xarm_client_;

        std::shared_ptr<controller_manager_msgs::srv::ListControllers::Request> req_list_controller_;
	    std::shared_ptr<controller_manager_msgs::srv::ListControllers::Response> res_list_controller_;
        std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request> req_switch_controller_;
        std::shared_ptr<controller_manager_msgs::srv::SwitchController::Response> res_switch_controller_;

        rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr client_list_controller_;
        rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr client_switch_controller_;

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        rclcpp::Subscription<xarm_msgs::msg::RobotMsg>::SharedPtr xarm_state_sub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trajectory_execution_event_sub_;

        void _receive_event(const std_msgs::msg::String::SharedPtr event);
        void _joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr states);
        void _xarm_states_callback(const xarm_msgs::msg::RobotMsg::SharedPtr states);
        bool _check_cmds_is_change(std::vector<float> prev, std::vector<float> cur, double threshold = 0.0001);
        bool _check_cmds_is_change(std::vector<double> prev, std::vector<double> cur, double threshold = 0.0001);
        bool _xarm_is_ready_read(void);
        bool _xarm_is_ready_write(void);

        bool _need_reset(void);

        void _reload_controller(void);

        template<typename ServiceT, typename SharedRequest = typename ServiceT::Request::SharedPtr, typename SharedResponse = typename ServiceT::Response::SharedPtr>
        int _call_request(std::shared_ptr<ServiceT> client, SharedRequest req, SharedResponse& res);

    };
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(xarm_control::XArmHW, hardware_interface::SystemInterface)

#endif // __XARM_HARDWARE_INTERFACE_H__