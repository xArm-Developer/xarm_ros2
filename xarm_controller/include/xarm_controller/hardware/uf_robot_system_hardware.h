/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
           Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#ifndef __UF_ROBOT_SYSTEM_HARDWARE_INTERFACE_H__
#define __UF_ROBOT_SYSTEM_HARDWARE_INTERFACE_H__

#include <vector>
#include <thread>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
// #include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/visibility_control.h"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "xarm_api/xarm_driver.h"


namespace uf_robot_hardware
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class UFRobotSystemHardware : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(UFRobotSystemHardware)

        CallbackReturn on_init(const hardware_interface::HardwareInfo& info) final;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

        CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) final;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) final;

        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration &period) final;
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration &period) final;

        // hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
        //                                                             const std::vector<std::string>& stop_interfaces) final;

        // hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
        //                                                             const std::vector<std::string>& stop_interfaces) final;

    protected:
        hardware_interface::HardwareInfo info_;
    
    private:
        int read_code_;
        int write_code_;

        std::string robot_ip_;

        float prev_cmds_float_[7];
		float cmds_float_[7];
        std::vector<double> position_cmds_;
        std::vector<double> velocity_cmds_;
        std::vector<double> position_states_;
        std::vector<double> velocity_states_;

        bool velocity_control_;
        bool initialized_;
        bool read_ready_;
        bool reactivate_controller_later_;

        long int read_cnts_;
        long int read_failed_cnts_;
        double read_max_time_;
        double read_total_time_;
        
        float prev_read_position_[7];
		float curr_read_position_[7];
		float curr_read_velocity_[7];
		float curr_read_effort_[7];
        
        rclcpp::Time prev_read_time_;
        rclcpp::Time curr_read_time_;
        rclcpp::Time curr_write_time_;
        rclcpp::Time prev_write_time_;

        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<rclcpp::Node> hw_node_;
        xarm_api::XArmDriver xarm_driver_;
        sensor_msgs::msg::JointState *joint_state_msg_;

        std::shared_ptr<controller_manager_msgs::srv::ListControllers::Request> req_list_controller_;
	    std::shared_ptr<controller_manager_msgs::srv::ListControllers::Response> res_list_controller_;
        std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request> req_switch_controller_;
        std::shared_ptr<controller_manager_msgs::srv::SwitchController::Response> res_switch_controller_;

        rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr client_list_controller_;
        rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr client_switch_controller_;

        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr update_goal_state_pub_;
        std_msgs::msg::Empty update_goal_state_msg_;

        bool _check_cmds_is_change(float *prev, float *cur, double threshold = 0.0001);
        bool _xarm_is_ready_read(void);
        bool _xarm_is_ready_write(void);
        bool _firmware_version_is_ge(int major, int minor, int revision);

        bool _need_reset(void);

        void _deactivate_controller(void);
        void _activate_controller(void);

        void _init_ufactory_driver(void);

        template<typename ServiceT, typename SharedRequest = typename ServiceT::Request::SharedPtr, typename SharedResponse = typename ServiceT::Response::SharedPtr>
        int _call_request(std::shared_ptr<ServiceT> client, SharedRequest req, SharedResponse& res);

    };
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(uf_robot_hardware::UFRobotSystemHardware, hardware_interface::SystemInterface)

#endif // __UF_ROBOT_SYSTEM_HARDWARE_INTERFACE_H__