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
#include <future>
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
// #include "hardware_interface/visibility_control.h"
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

        // Cartesian TCP command/state, SI units (m, rad). The arm is always in
        // XARM_MODE::POSE, so motion is commanded with set_position(), not servoj.
        std::vector<double> tcp_cmds_;
        std::vector<double> tcp_states_;
        std::vector<double> prev_tcp_cmds_;
        // Commanded pose held on force-compliant axes. The controller applies the force
        // correction as an offset on top of the command, so a compliant axis must be
        // re-commanded to a fixed standoff or the command chases its own output.
        double tcp_standoff_[6];
        // Optional `speed` command interface on the same gpio: the set_position() speed
        // for the next motion, mm/s. 0 or NaN means "use tcp_speed_", so a URDF that
        // does not declare it, or a controller that never writes it, keeps the launch
        // time speed.
        double tcp_speed_cmd_;
        // tcp/servo: 0 asks for POSE, 1 for SERVO, NaN leaves the launch param in charge.
        // The state is the mode the arm reports.
        double servo_cmd_;
        double servo_state_;
        // 1 while the controller error code is non-zero, 0 while it is clear, NaN while the
        // driver is not connected. Optional `fault` state interface on the tcp gpio.
        double fault_state_;

        // End-effector force/torque, N and Nm, ordered fx fy fz tx ty tz
        std::vector<double> ft_states_;

        // Runtime force control, exposed as a second gpio component so the loop can be
        // armed, disarmed and retuned mid operation without relaunching. Command order
        // is FT_CMD_NAMES, state order is FT_CFG_STATE_NAMES.
        std::vector<double> ft_cmds_;
        std::vector<double> ft_cfg_states_;
        bool ft_armed_;
        bool prev_want_armed_;
        bool has_ft_gpio_;
        // "<ft gpio name>/", used to keep the force control controller out of the
        // deactivate/reactivate cycle.
        std::string ft_iface_prefix_;

        // False on hardware with no physical F/T sensor -- gates every SDK call that
        // talks to it. Distinct from ft_sensor_mode_ == 0, just an unarmed force loop.
        bool has_ft_sensor_;

        // Force control configuration (see set_ft_sensor_force_parameters)
        int ft_sensor_mode_;
        int ft_coord_;
        int ft_c_axis_[6];
        float ft_f_ref_[6];
        float ft_limits_[6];
        float ft_kp_[6];
        float ft_ki_[6];
        float ft_kd_[6];
        float ft_xe_limit_[6];
        bool ft_zero_on_activate_;

        // Homing, performed on activation before the force loop is armed
        bool home_on_activate_;
        bool has_home_pose_;
        bool has_home_joints_;
        float home_pose_[6];     // mm, rad
        float home_joints_[7];   // rad
        float home_speed_;       // mm/s
        float home_acc_;         // mm/s^2
        float home_joint_speed_; // rad/s
        float home_joint_acc_;   // rad/s^2

        // set_position() parameters used by write()
        float tcp_speed_;   // mm/s, the fallback for tcp_speed_cmd_
        float tcp_acc_;     // mm/s^2
        float tcp_radius_;  // mm, <0 disables blending
        int cmd_queue_max_; // skip write() when the controller cache is at least this deep

        bool velocity_control_;
        // set_servo_cartesian()/XARM_MODE::SERVO instead of set_position()/XARM_MODE::POSE --
        // no motion queue, no backlog/jitter under continuous streaming, costs the ft gpio.
        bool cartesian_servo_mode_;
        bool initialized_;
        bool read_ready_;
        bool reactivate_controller_later_;

        long int read_cnts_;
        long int read_failed_cnts_;
        double read_max_time_;
        double read_total_time_;
        
        // float prev_read_position_[7];
		// float curr_read_position_[7];
		// float curr_read_velocity_[7];
		// float curr_read_effort_[7];
        
        // rclcpp::Time prev_read_time_;
        // rclcpp::Time curr_read_time_;
        rclcpp::Time curr_write_time_;
        rclcpp::Time prev_write_time_;
        rclcpp::Time prev_rearm_time_;
        rclcpp::Time prev_ft_retry_time_;
        rclcpp::Time prev_servo_switch_time_;
        rclcpp::Time servo_settle_until_;
        std::future<bool> servo_switch_future_;
        bool servo_switch_target_;

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

        // Parse a space separated list of n floats. Returns false if str is empty or
        // does not hold exactly n values, leaving out untouched.
        bool _parse_vecn(const std::string& str, float *out, int n);
        void _init_ft_params(void);
        int _setup_ft_sensor(void);
        // Push whatever the ft gpio command interfaces currently hold to the arm.
        // Called from write(); every call it makes is skipped unless something changed.
        void _apply_ft_commands(void);
        bool _apply_servo_command(void);
        bool _idle_for_mode_switch(void);
        // Switch the onboard force loop on or off. Returns true if the arm ended up in
        // the requested state.
        bool _arm_ft(bool on);
        void _refresh_ft_cfg_states(void);
        int _go_home(void);
        bool _tcp_cmd_is_valid(void);
        bool _tcp_cmds_is_change(double threshold = 0.00001);

        // True while the arm is in state 5 (CONFIG_CHANGED), which every force config
        // change causes. Re-arms the arm and tells write() to skip the cycle.
        bool _handle_config_changed(void);
        // A controller that only claims ft gpio interfaces commands no motion.
        bool _is_ft_controller(const controller_manager_msgs::msg::ControllerState& c);

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