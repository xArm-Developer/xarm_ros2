/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
           Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#ifndef __XARM_DRIVER_H
#define __XARM_DRIVER_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <control_msgs/action/gripper_command.hpp>

#include "xarm_msgs.h"
#include "xarm/wrapper/xarm_api.h"

namespace xarm_api
{
    class XArmDriver
    {
    public:
        XArmDriver() {};
        ~XArmDriver();
        void init(rclcpp::Node::SharedPtr& node, std::string &server_ip, bool in_ros_control = false);

        void pub_robot_msg(xarm_msgs::msg::RobotMsg &rm_msg);
        void pub_joint_state(sensor_msgs::msg::JointState &js_msg);
        void pub_cgpio_state(xarm_msgs::msg::CIOState &cio_msg);
        void pub_ftsensor_ext_state(geometry_msgs::msg::WrenchStamped &wrench_msg);
        void pub_ftsensor_raw_state(geometry_msgs::msg::WrenchStamped &wrench_msg);
        
        bool is_connected(void);
        std::string controller_error_interpreter(int err=-1);

        rclcpp::Logger get_logger() { return node_->get_logger(); }

        sensor_msgs::msg::JointState* get_joint_states();

    private:
        void _report_connect_changed_callback(bool connected, bool reported);
        void _report_data_callback(XArmReportData *report_data_ptr);
        bool _get_wait_param(void);

        void _init_xarm_gripper(void);
        inline float _xarm_gripper_pos_convert(float pos, bool reversed = false);
        rclcpp_action::GoalResponse _handle_xarm_gripper_action_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const control_msgs::action::GripperCommand::Goal> goal);
        rclcpp_action::CancelResponse _handle_xarm_gripper_action_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle);
        void _handle_xarm_gripper_action_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle);
        void _xarm_gripper_action_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle);
        void _pub_xarm_gripper_joint_states(float pos);

        void _init_bio_gripper(void);
        inline float _bio_gripper_pos_convert(float pos, bool reversed = false);
        rclcpp_action::GoalResponse _handle_bio_gripper_action_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const control_msgs::action::GripperCommand::Goal> goal);
        rclcpp_action::CancelResponse _handle_bio_gripper_action_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle);
        void _handle_bio_gripper_action_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle);
        void _bio_gripper_action_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle);
        void _pub_bio_gripper_joint_states(float pos);

        template<typename ServiceT, typename CallbackT>
    	typename rclcpp::Service<ServiceT>::SharedPtr _create_service(const std::string & service_name, CallbackT && callback);

        void _init_publisher(void);
        void _init_service(void);
        void _init_subscription(void);
        bool _firmware_version_is_ge(int major, int minor, int revision);

    public:
        XArmAPI *arm;
        int curr_state;
        int curr_err;
        int curr_cmdnum;
        int curr_mode;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Node::SharedPtr hw_node_;

        int dof_;
        int joint_states_rate_;
        bool in_ros_control_;
        int vacuum_gripper_hardware_version_;
        std::string report_type_;
        std::vector<std::string> joint_names_;
        sensor_msgs::msg::JointState joint_state_msg_;
        geometry_msgs::msg::WrenchStamped ftsensor_msg_;
        xarm_msgs::msg::RobotMsg xarm_state_msg_;
        xarm_msgs::msg::CIOState cgpio_state_msg_;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        rclcpp::Publisher<xarm_msgs::msg::RobotMsg>::SharedPtr robot_state_pub_;
        rclcpp::Publisher<xarm_msgs::msg::CIOState>::SharedPtr cgpio_state_pub_;
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr ftsensor_ext_state_pub_;
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr ftsensor_raw_state_pub_;

        bool xarm_gripper_init_loop_;
        int xarm_gripper_speed_;
        int xarm_gripper_max_pos_;
        int xarm_gripper_frequency_;
        int xarm_gripper_threshold_;
        int xarm_gripper_threshold_times_;
        sensor_msgs::msg::JointState xarm_gripper_joint_state_msg_;
        control_msgs::action::GripperCommand::Feedback::SharedPtr xarm_gripper_feedback_;
        control_msgs::action::GripperCommand::Result::SharedPtr xarm_gripper_result_;
        rclcpp_action::Server<control_msgs::action::GripperCommand>::SharedPtr xarm_gripper_action_server_;

        bool bio_gripper_init_loop_;
        int bio_gripper_speed_;
        int bio_gripper_max_pos_;
        int bio_gripper_min_pos_;
        int bio_gripper_frequency_;
        int bio_gripper_threshold_;
        int bio_gripper_threshold_times_;
        sensor_msgs::msg::JointState bio_gripper_joint_state_msg_;
        control_msgs::action::GripperCommand::Feedback::SharedPtr bio_gripper_feedback_;
        control_msgs::action::GripperCommand::Result::SharedPtr bio_gripper_result_;
        rclcpp_action::Server<control_msgs::action::GripperCommand>::SharedPtr bio_gripper_action_server_;
    
    private:
        bool service_debug_;
        // Call
        rclcpp::Service<xarm_msgs::srv::Call>::SharedPtr service_clean_error_;
        rclcpp::Service<xarm_msgs::srv::Call>::SharedPtr service_clean_warn_;
        rclcpp::Service<xarm_msgs::srv::Call>::SharedPtr service_clean_conf_;
        rclcpp::Service<xarm_msgs::srv::Call>::SharedPtr service_save_conf_;
        rclcpp::Service<xarm_msgs::srv::Call>::SharedPtr service_reload_dynamics_;
        rclcpp::Service<xarm_msgs::srv::Call>::SharedPtr service_set_counter_reset_;
        rclcpp::Service<xarm_msgs::srv::Call>::SharedPtr service_set_counter_increase_;
        rclcpp::Service<xarm_msgs::srv::Call>::SharedPtr service_clean_gripper_error_;
        rclcpp::Service<xarm_msgs::srv::Call>::SharedPtr service_clean_bio_gripper_error_;
        rclcpp::Service<xarm_msgs::srv::Call>::SharedPtr service_start_record_trajectory_;
        rclcpp::Service<xarm_msgs::srv::Call>::SharedPtr service_stop_record_trajectory_;
        rclcpp::Service<xarm_msgs::srv::Call>::SharedPtr service_ft_sensor_set_zero_;
        rclcpp::Service<xarm_msgs::srv::Call>::SharedPtr service_set_linear_track_stop_;
        rclcpp::Service<xarm_msgs::srv::Call>::SharedPtr service_clean_linear_track_error_;
        rclcpp::Service<xarm_msgs::srv::Call>::SharedPtr service_open_lite6_gripper_;
        rclcpp::Service<xarm_msgs::srv::Call>::SharedPtr service_close_lite6_gripper_;
        rclcpp::Service<xarm_msgs::srv::Call>::SharedPtr service_stop_lite6_gripper_;
        bool _clean_error(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res);
        bool _clean_warn(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res);
        bool _clean_conf(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res);
        bool _save_conf(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res);
        bool _reload_dynamics(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res);
        bool _set_counter_reset(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res);
        bool _set_counter_increase(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res);
        bool _clean_gripper_error(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res); 
        bool _clean_bio_gripper_error(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res);
        bool _start_record_trajectory(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res);
        bool _stop_record_trajectory(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res);
        bool _ft_sensor_set_zero(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res);
        bool _set_linear_track_stop(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res);
        bool _clean_linear_track_error(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res);
        bool _open_lite6_gripper(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res);
        bool _close_lite6_gripper(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res);
        bool _stop_lite6_gripper(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res);

        // GetInt16
        rclcpp::Service<xarm_msgs::srv::GetInt16>::SharedPtr service_get_state_;
        rclcpp::Service<xarm_msgs::srv::GetInt16>::SharedPtr service_get_cmdnum_;
        rclcpp::Service<xarm_msgs::srv::GetInt16>::SharedPtr service_get_vacuum_gripper_;
        rclcpp::Service<xarm_msgs::srv::GetInt16>::SharedPtr service_get_gripper_err_code_;
        rclcpp::Service<xarm_msgs::srv::GetInt16>::SharedPtr service_get_bio_gripper_status_;
        rclcpp::Service<xarm_msgs::srv::GetInt16>::SharedPtr service_get_bio_gripper_error_;
        rclcpp::Service<xarm_msgs::srv::GetInt16>::SharedPtr service_get_reduced_mode_;
        rclcpp::Service<xarm_msgs::srv::GetInt16>::SharedPtr service_get_report_tau_or_i_;
        rclcpp::Service<xarm_msgs::srv::GetInt16>::SharedPtr service_ft_sensor_app_get_;
        rclcpp::Service<xarm_msgs::srv::GetInt16>::SharedPtr service_get_ft_sensor_error_;
        rclcpp::Service<xarm_msgs::srv::GetInt16>::SharedPtr service_get_trajectory_rw_status_;
        rclcpp::Service<xarm_msgs::srv::GetInt16>::SharedPtr service_get_linear_track_pos_;
        rclcpp::Service<xarm_msgs::srv::GetInt16>::SharedPtr service_get_linear_track_status_;
        rclcpp::Service<xarm_msgs::srv::GetInt16>::SharedPtr service_get_linear_track_error_;
        rclcpp::Service<xarm_msgs::srv::GetInt16>::SharedPtr service_get_linear_track_is_enabled_;
        rclcpp::Service<xarm_msgs::srv::GetInt16>::SharedPtr service_get_linear_track_on_zero_;
        rclcpp::Service<xarm_msgs::srv::GetInt16>::SharedPtr service_get_linear_track_sci_;
        bool _get_state(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res);
        bool _get_cmdnum(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res);
        bool _get_vacuum_gripper(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res);
        bool _get_gripper_err_code(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res);
        bool _get_bio_gripper_status(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res);
        bool _get_bio_gripper_error(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res);
        bool _get_reduced_mode(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res);
        bool _get_report_tau_or_i(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res);
        bool _ft_sensor_app_get(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res);
        bool _get_ft_sensor_error(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res);
        bool _get_trajectory_rw_status(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res);
        bool _get_linear_track_pos(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res);
        bool _get_linear_track_status(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res);
        bool _get_linear_track_error(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res);
        bool _get_linear_track_is_enabled(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res);
        bool _get_linear_track_on_zero(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res);
        bool _get_linear_track_sci(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res);

        // GetInt16List
        rclcpp::Service<xarm_msgs::srv::GetInt16List>::SharedPtr service_get_err_warn_code_;
        rclcpp::Service<xarm_msgs::srv::GetInt16List>::SharedPtr service_get_linear_track_sco_;
        bool _get_err_warn_code(const std::shared_ptr<xarm_msgs::srv::GetInt16List::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16List::Response> res);
        bool _get_linear_track_sco(const std::shared_ptr<xarm_msgs::srv::GetInt16List::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16List::Response> res);

        // SetInt16
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr service_set_mode_;
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr service_set_state_;
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr service_set_collision_sensitivity_;
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr service_set_teach_sensitivity_;
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr service_set_gripper_mode_;
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr service_set_gripper_enable_;
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr service_set_bio_gripper_speed_;
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr service_set_collision_rebound_;
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr service_set_fence_mode_;
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr service_set_reduced_mode_;
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr service_set_self_collision_detection_;
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr service_set_simulation_robot_;
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr service_set_baud_checkset_enable_;
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr service_set_report_tau_or_i_;
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr service_ft_sensor_enable_;
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr service_ft_sensor_app_set_;
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr service_set_linear_track_enable_;
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr service_set_linear_track_speed_;
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr service_set_cartesian_velo_continuous_;
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr service_set_allow_approx_motion_;
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr service_set_only_check_type_;     
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr service_config_tgpio_reset_when_stop_;     
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr service_config_cgpio_reset_when_stop_;     
        bool _set_mode(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
        bool _set_state(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
        bool _set_collision_sensitivity(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
        bool _set_teach_sensitivity(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
        bool _set_gripper_mode(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
        bool _set_gripper_enable(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
        bool _set_bio_gripper_speed(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
        bool _set_collision_rebound(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
        bool _set_fence_mode(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
        bool _set_reduced_mode(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
        bool _set_self_collision_detection(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
        bool _set_simulation_robot(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
        bool _set_baud_checkset_enable(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
        bool _set_report_tau_or_i(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
        bool _ft_sensor_enable(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
        bool _ft_sensor_app_set(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
        bool _set_linear_track_enable(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
        bool _set_linear_track_speed(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
        bool _set_cartesian_velo_continuous(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
        bool _set_allow_approx_motion(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
        bool _set_only_check_type(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
        bool _config_tgpio_reset_when_stop(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
        bool _config_cgpio_reset_when_stop(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);

        // SetInt16ById
        rclcpp::Service<xarm_msgs::srv::SetInt16ById>::SharedPtr service_motion_enable_;
        rclcpp::Service<xarm_msgs::srv::SetInt16ById>::SharedPtr service_set_servo_attach_;
        rclcpp::Service<xarm_msgs::srv::SetInt16ById>::SharedPtr service_set_servo_detach_;
        bool _motion_enable(const std::shared_ptr<xarm_msgs::srv::SetInt16ById::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16ById::Response> res);
        bool _set_servo_attach(const std::shared_ptr<xarm_msgs::srv::SetInt16ById::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16ById::Response> res);
        bool _set_servo_detach(const std::shared_ptr<xarm_msgs::srv::SetInt16ById::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16ById::Response> res);
        
        // SetInt16List
        rclcpp::Service<xarm_msgs::srv::SetInt16List>::SharedPtr service_set_reduced_tcp_boundary_;
        bool _set_reduced_tcp_boundary(const std::shared_ptr<xarm_msgs::srv::SetInt16List::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16List::Response> res);
        
        // GetInt32
        rclcpp::Service<xarm_msgs::srv::GetInt32>::SharedPtr service_get_tgpio_modbus_baudrate_;
        bool _get_tgpio_modbus_baudrate(const std::shared_ptr<xarm_msgs::srv::GetInt32::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt32::Response> res);

        // GetInt32ByType
        rclcpp::Service<xarm_msgs::srv::GetInt32ByType>::SharedPtr service_get_checkset_default_baud_;
        bool _get_checkset_default_baud(const std::shared_ptr<xarm_msgs::srv::GetInt32ByType::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt32ByType::Response> res);

        // SetInt32
        rclcpp::Service<xarm_msgs::srv::SetInt32>::SharedPtr service_set_tgpio_modbus_baudrate_;
        bool _set_tgpio_modbus_baudrate(const std::shared_ptr<xarm_msgs::srv::SetInt32::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt32::Response> res);

        // SetInt32ByType
        rclcpp::Service<xarm_msgs::srv::SetInt32ByType>::SharedPtr service_set_checkset_default_baud_;
        bool _set_checkset_default_baud(const std::shared_ptr<xarm_msgs::srv::SetInt32ByType::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt32ByType::Response> res);

        // GetFloat32
        rclcpp::Service<xarm_msgs::srv::GetFloat32>::SharedPtr service_get_gripper_position_;
        bool _get_gripper_position(const std::shared_ptr<xarm_msgs::srv::GetFloat32::Request> req, std::shared_ptr<xarm_msgs::srv::GetFloat32::Response> res);

        // GetFloat32List
        rclcpp::Service<xarm_msgs::srv::GetFloat32List>::SharedPtr service_get_position_;
        rclcpp::Service<xarm_msgs::srv::GetFloat32List>::SharedPtr service_get_servo_angle_;
        rclcpp::Service<xarm_msgs::srv::GetFloat32List>::SharedPtr service_get_position_aa_;
        rclcpp::Service<xarm_msgs::srv::GetFloat32List>::SharedPtr service_get_ft_sensor_data_;
        bool _get_position(const std::shared_ptr<xarm_msgs::srv::GetFloat32List::Request> req, std::shared_ptr<xarm_msgs::srv::GetFloat32List::Response> res);
        bool _get_servo_angle(const std::shared_ptr<xarm_msgs::srv::GetFloat32List::Request> req, std::shared_ptr<xarm_msgs::srv::GetFloat32List::Response> res);
        bool _get_position_aa(const std::shared_ptr<xarm_msgs::srv::GetFloat32List::Request> req, std::shared_ptr<xarm_msgs::srv::GetFloat32List::Response> res);
        bool _get_ft_sensor_data(const std::shared_ptr<xarm_msgs::srv::GetFloat32List::Request> req, std::shared_ptr<xarm_msgs::srv::GetFloat32List::Response> res);
        
        // SetFloat32
        rclcpp::Service<xarm_msgs::srv::SetFloat32>::SharedPtr service_set_pause_time_;
        rclcpp::Service<xarm_msgs::srv::SetFloat32>::SharedPtr service_set_tcp_jerk_;
        rclcpp::Service<xarm_msgs::srv::SetFloat32>::SharedPtr service_set_tcp_maxacc_;
        rclcpp::Service<xarm_msgs::srv::SetFloat32>::SharedPtr service_set_joint_jerk_;
        rclcpp::Service<xarm_msgs::srv::SetFloat32>::SharedPtr service_set_joint_maxacc_;
        rclcpp::Service<xarm_msgs::srv::SetFloat32>::SharedPtr service_set_gripper_speed_;
        rclcpp::Service<xarm_msgs::srv::SetFloat32>::SharedPtr service_set_reduced_max_tcp_speed_;
        rclcpp::Service<xarm_msgs::srv::SetFloat32>::SharedPtr service_set_reduced_max_joint_speed_;
        bool _set_pause_time(const std::shared_ptr<xarm_msgs::srv::SetFloat32::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32::Response> res);
        bool _set_tcp_jerk(const std::shared_ptr<xarm_msgs::srv::SetFloat32::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32::Response> res);
        bool _set_tcp_maxacc(const std::shared_ptr<xarm_msgs::srv::SetFloat32::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32::Response> res);
        bool _set_joint_jerk(const std::shared_ptr<xarm_msgs::srv::SetFloat32::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32::Response> res);
        bool _set_joint_maxacc(const std::shared_ptr<xarm_msgs::srv::SetFloat32::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32::Response> res);
        bool _set_gripper_speed(const std::shared_ptr<xarm_msgs::srv::SetFloat32::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32::Response> res);
        bool _set_reduced_max_tcp_speed(const std::shared_ptr<xarm_msgs::srv::SetFloat32::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32::Response> res);
        bool _set_reduced_max_joint_speed(const std::shared_ptr<xarm_msgs::srv::SetFloat32::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32::Response> res);

        // SetFloat32List
        rclcpp::Service<xarm_msgs::srv::SetFloat32List>::SharedPtr service_set_gravity_direction_;
        rclcpp::Service<xarm_msgs::srv::SetFloat32List>::SharedPtr service_set_tcp_offset_;
        rclcpp::Service<xarm_msgs::srv::SetFloat32List>::SharedPtr service_set_world_offset_;
        rclcpp::Service<xarm_msgs::srv::SetFloat32List>::SharedPtr service_set_reduced_joint_range_;
        bool _set_gravity_direction(const std::shared_ptr<xarm_msgs::srv::SetFloat32List::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32List::Response> res);
        bool _set_tcp_offset(const std::shared_ptr<xarm_msgs::srv::SetFloat32List::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32List::Response> res);
        bool _set_world_offset(const std::shared_ptr<xarm_msgs::srv::SetFloat32List::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32List::Response> res);
        bool _set_reduced_joint_range(const std::shared_ptr<xarm_msgs::srv::SetFloat32List::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32List::Response> res);

        // SetTcpLoad
        rclcpp::Service<xarm_msgs::srv::SetTcpLoad>::SharedPtr service_set_tcp_load_;
        bool _set_tcp_load(const std::shared_ptr<xarm_msgs::srv::SetTcpLoad::Request> req, std::shared_ptr<xarm_msgs::srv::SetTcpLoad::Response> res);

        // MoveCartesian
        rclcpp::Service<xarm_msgs::srv::MoveCartesian>::SharedPtr service_set_position_;
        rclcpp::Service<xarm_msgs::srv::MoveCartesian>::SharedPtr service_set_tool_position_;
        rclcpp::Service<xarm_msgs::srv::MoveCartesian>::SharedPtr service_set_position_aa_;
        rclcpp::Service<xarm_msgs::srv::MoveCartesian>::SharedPtr service_set_servo_cartesian_;
        rclcpp::Service<xarm_msgs::srv::MoveCartesian>::SharedPtr service_set_servo_cartesian_aa_;
        bool _set_position(const std::shared_ptr<xarm_msgs::srv::MoveCartesian::Request> req, std::shared_ptr<xarm_msgs::srv::MoveCartesian::Response> res);
        bool _set_tool_position(const std::shared_ptr<xarm_msgs::srv::MoveCartesian::Request> req, std::shared_ptr<xarm_msgs::srv::MoveCartesian::Response> res);
        bool _set_position_aa(const std::shared_ptr<xarm_msgs::srv::MoveCartesian::Request> req, std::shared_ptr<xarm_msgs::srv::MoveCartesian::Response> res);
        bool _set_servo_cartesian(const std::shared_ptr<xarm_msgs::srv::MoveCartesian::Request> req, std::shared_ptr<xarm_msgs::srv::MoveCartesian::Response> res);
        bool _set_servo_cartesian_aa(const std::shared_ptr<xarm_msgs::srv::MoveCartesian::Request> req, std::shared_ptr<xarm_msgs::srv::MoveCartesian::Response> res);

        // MoveJoint
        rclcpp::Service<xarm_msgs::srv::MoveJoint>::SharedPtr service_set_servo_angle_;
        rclcpp::Service<xarm_msgs::srv::MoveJoint>::SharedPtr service_set_servo_angle_j_;
        bool _set_servo_angle(const std::shared_ptr<xarm_msgs::srv::MoveJoint::Request> req, std::shared_ptr<xarm_msgs::srv::MoveJoint::Response> res);
        bool _set_servo_angle_j(const std::shared_ptr<xarm_msgs::srv::MoveJoint::Request> req, std::shared_ptr<xarm_msgs::srv::MoveJoint::Response> res);

        // MoveCircle
        rclcpp::Service<xarm_msgs::srv::MoveCircle>::SharedPtr service_move_circle_;
        bool _move_circle(const std::shared_ptr<xarm_msgs::srv::MoveCircle::Request> req, std::shared_ptr<xarm_msgs::srv::MoveCircle::Response> res);

        // MoveHome
        rclcpp::Service<xarm_msgs::srv::MoveHome>::SharedPtr service_move_gohome_;
        bool _move_gohome(const std::shared_ptr<xarm_msgs::srv::MoveHome::Request> req, std::shared_ptr<xarm_msgs::srv::MoveHome::Response> res);

        // MoveVelocity
        rclcpp::Service<xarm_msgs::srv::MoveVelocity>::SharedPtr service_vc_set_joint_velocity_;
        rclcpp::Service<xarm_msgs::srv::MoveVelocity>::SharedPtr service_vc_set_cartesian_velocity_;
        rclcpp::Subscription<xarm_msgs::msg::MoveVelocity>::SharedPtr subscription_vc_set_joint_velocity_;
        rclcpp::Subscription<xarm_msgs::msg::MoveVelocity>::SharedPtr subscription_vc_set_cartesian_velocity_;
        bool _vc_set_joint_velocity(const std::shared_ptr<xarm_msgs::srv::MoveVelocity::Request> req, std::shared_ptr<xarm_msgs::srv::MoveVelocity::Response> res);
        void _vc_set_joint_velocity_topic_callback(const xarm_msgs::msg::MoveVelocity::SharedPtr msg);
        bool _vc_set_cartesian_velocity(const std::shared_ptr<xarm_msgs::srv::MoveVelocity::Request> req, std::shared_ptr<xarm_msgs::srv::MoveVelocity::Response> res);
        void _vc_set_cartesian_velocity_topic_callback(const xarm_msgs::msg::MoveVelocity::SharedPtr msg);

        // GetDigitalIO
        rclcpp::Service<xarm_msgs::srv::GetDigitalIO>::SharedPtr service_get_tgpio_digital_;
        rclcpp::Service<xarm_msgs::srv::GetDigitalIO>::SharedPtr service_get_cgpio_digital_;
        bool _get_tgpio_digital(const std::shared_ptr<xarm_msgs::srv::GetDigitalIO::Request> req, std::shared_ptr<xarm_msgs::srv::GetDigitalIO::Response> res);
        bool _get_cgpio_digital(const std::shared_ptr<xarm_msgs::srv::GetDigitalIO::Request> req, std::shared_ptr<xarm_msgs::srv::GetDigitalIO::Response> res);

        // GetAnalogIO
        rclcpp::Service<xarm_msgs::srv::GetAnalogIO>::SharedPtr service_get_tgpio_analog_;
        rclcpp::Service<xarm_msgs::srv::GetAnalogIO>::SharedPtr service_get_cgpio_analog_;
        bool _get_tgpio_analog(const std::shared_ptr<xarm_msgs::srv::GetAnalogIO::Request> req, std::shared_ptr<xarm_msgs::srv::GetAnalogIO::Response> res);
        bool _get_cgpio_analog(const std::shared_ptr<xarm_msgs::srv::GetAnalogIO::Request> req, std::shared_ptr<xarm_msgs::srv::GetAnalogIO::Response> res);

        // SetDigitalIO
        rclcpp::Service<xarm_msgs::srv::SetDigitalIO>::SharedPtr service_set_tgpio_digital_;
        rclcpp::Service<xarm_msgs::srv::SetDigitalIO>::SharedPtr service_set_cgpio_digital_;
        rclcpp::Service<xarm_msgs::srv::SetDigitalIO>::SharedPtr service_set_tgpio_digital_with_xyz_;
        rclcpp::Service<xarm_msgs::srv::SetDigitalIO>::SharedPtr service_set_cgpio_digital_with_xyz_;
        bool _set_tgpio_digital(const std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Request> req, std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Response> res);
        bool _set_cgpio_digital(const std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Request> req, std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Response> res);
        bool _set_tgpio_digital_with_xyz(const std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Request> req, std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Response> res);
        bool _set_cgpio_digital_with_xyz(const std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Request> req, std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Response> res);

        // SetAnalogIO
        rclcpp::Service<xarm_msgs::srv::SetAnalogIO>::SharedPtr service_set_cgpio_analog_;
        rclcpp::Service<xarm_msgs::srv::SetAnalogIO>::SharedPtr service_set_cgpio_analog_with_xyz_;
        bool _set_cgpio_analog(const std::shared_ptr<xarm_msgs::srv::SetAnalogIO::Request> req, std::shared_ptr<xarm_msgs::srv::SetAnalogIO::Response> res);
        bool _set_cgpio_analog_with_xyz(const std::shared_ptr<xarm_msgs::srv::SetAnalogIO::Request> req, std::shared_ptr<xarm_msgs::srv::SetAnalogIO::Response> res);

        // VacuumGripperCtrl
        rclcpp::Service<xarm_msgs::srv::VacuumGripperCtrl>::SharedPtr service_set_vacuum_gripper_;
        bool _set_vacuum_gripper(const std::shared_ptr<xarm_msgs::srv::VacuumGripperCtrl::Request> req, std::shared_ptr<xarm_msgs::srv::VacuumGripperCtrl::Response> res);

        // GripperMove
        rclcpp::Service<xarm_msgs::srv::GripperMove>::SharedPtr service_set_gripper_position_;
        bool _set_gripper_position(const std::shared_ptr<xarm_msgs::srv::GripperMove::Request> req, std::shared_ptr<xarm_msgs::srv::GripperMove::Response> res);
        
        // BioGripperEnable
        rclcpp::Service<xarm_msgs::srv::BioGripperEnable>::SharedPtr service_set_bio_gripper_enable_;
        bool _set_bio_gripper_enable(const std::shared_ptr<xarm_msgs::srv::BioGripperEnable::Request> req, std::shared_ptr<xarm_msgs::srv::BioGripperEnable::Response> res);
        
        // BioGripperCtrl
        rclcpp::Service<xarm_msgs::srv::BioGripperCtrl>::SharedPtr service_open_bio_gripper_;
        rclcpp::Service<xarm_msgs::srv::BioGripperCtrl>::SharedPtr service_close_bio_gripper_;
        bool _open_bio_gripper(const std::shared_ptr<xarm_msgs::srv::BioGripperCtrl::Request> req, std::shared_ptr<xarm_msgs::srv::BioGripperCtrl::Response> res);
        bool _close_bio_gripper(const std::shared_ptr<xarm_msgs::srv::BioGripperCtrl::Request> req, std::shared_ptr<xarm_msgs::srv::BioGripperCtrl::Response> res);         
        
        // RobotiqReset
        rclcpp::Service<xarm_msgs::srv::RobotiqReset>::SharedPtr service_robotiq_reset_;
        bool _robotiq_reset(const std::shared_ptr<xarm_msgs::srv::RobotiqReset::Request> req, std::shared_ptr<xarm_msgs::srv::RobotiqReset::Response> res);
        
        // RobotiqActivate
        rclcpp::Service<xarm_msgs::srv::RobotiqActivate>::SharedPtr service_robotiq_set_activate_;
        bool _robotiq_set_activate(const std::shared_ptr<xarm_msgs::srv::RobotiqActivate::Request> req, std::shared_ptr<xarm_msgs::srv::RobotiqActivate::Response> res);
        
        // RobotiqMove
        rclcpp::Service<xarm_msgs::srv::RobotiqMove>::SharedPtr service_robotiq_set_position_;
        rclcpp::Service<xarm_msgs::srv::RobotiqMove>::SharedPtr service_robotiq_open_;
        rclcpp::Service<xarm_msgs::srv::RobotiqMove>::SharedPtr service_robotiq_close_;
        bool _robotiq_set_position(const std::shared_ptr<xarm_msgs::srv::RobotiqMove::Request> req, std::shared_ptr<xarm_msgs::srv::RobotiqMove::Response> res);
        bool _robotiq_open(const std::shared_ptr<xarm_msgs::srv::RobotiqMove::Request> req, std::shared_ptr<xarm_msgs::srv::RobotiqMove::Response> res);
        bool _robotiq_close(const std::shared_ptr<xarm_msgs::srv::RobotiqMove::Request> req, std::shared_ptr<xarm_msgs::srv::RobotiqMove::Response> res);
        
        // RobotiqGetStatus
        rclcpp::Service<xarm_msgs::srv::RobotiqGetStatus>::SharedPtr service_robotiq_get_status_;
        bool _robotiq_get_status(const std::shared_ptr<xarm_msgs::srv::RobotiqGetStatus::Request> req, std::shared_ptr<xarm_msgs::srv::RobotiqGetStatus::Response> res);

        // SetModbusTimeout
        rclcpp::Service<xarm_msgs::srv::SetModbusTimeout>::SharedPtr service_set_tgpio_modbus_timeout_;
        bool _set_tgpio_modbus_timeout(const std::shared_ptr<xarm_msgs::srv::SetModbusTimeout::Request> req, std::shared_ptr<xarm_msgs::srv::SetModbusTimeout::Response> res);

        // GetSetModbusData
        rclcpp::Service<xarm_msgs::srv::GetSetModbusData>::SharedPtr service_getset_tgpio_modbus_data_;
        bool _getset_tgpio_modbus_data(const std::shared_ptr<xarm_msgs::srv::GetSetModbusData::Request> req, std::shared_ptr<xarm_msgs::srv::GetSetModbusData::Response> res);
        
        // TrajCtrl
        rclcpp::Service<xarm_msgs::srv::TrajCtrl>::SharedPtr service_save_record_trajectory_;
        rclcpp::Service<xarm_msgs::srv::TrajCtrl>::SharedPtr service_load_trajectory_;
        bool _save_record_trajectory(const std::shared_ptr<xarm_msgs::srv::TrajCtrl::Request> req, std::shared_ptr<xarm_msgs::srv::TrajCtrl::Response> res);
        bool _load_trajectory(const std::shared_ptr<xarm_msgs::srv::TrajCtrl::Request> req, std::shared_ptr<xarm_msgs::srv::TrajCtrl::Response> res);
        
        // TrajPlay
        rclcpp::Service<xarm_msgs::srv::TrajPlay>::SharedPtr service_playback_trajectory_;
        bool _playback_trajectory(const std::shared_ptr<xarm_msgs::srv::TrajPlay::Request> req, std::shared_ptr<xarm_msgs::srv::TrajPlay::Response> res);
    
        // IdenLoad
        rclcpp::Service<xarm_msgs::srv::IdenLoad>::SharedPtr service_iden_tcp_load_;
        rclcpp::Service<xarm_msgs::srv::IdenLoad>::SharedPtr service_ft_sensor_iden_load_;
        bool _iden_tcp_load(const std::shared_ptr<xarm_msgs::srv::IdenLoad::Request> req, std::shared_ptr<xarm_msgs::srv::IdenLoad::Response> res);
        bool _ft_sensor_iden_load(const std::shared_ptr<xarm_msgs::srv::IdenLoad::Request> req, std::shared_ptr<xarm_msgs::srv::IdenLoad::Response> res);

        // FtCaliLoad
        rclcpp::Service<xarm_msgs::srv::FtCaliLoad>::SharedPtr service_ft_sensor_cali_load_;
        bool _ft_sensor_cali_load(const std::shared_ptr<xarm_msgs::srv::FtCaliLoad::Request> req, std::shared_ptr<xarm_msgs::srv::FtCaliLoad::Response> res);
    
        // FtForceConfig
        rclcpp::Service<xarm_msgs::srv::FtForceConfig>::SharedPtr service_config_force_control_;
        bool _config_force_control(const std::shared_ptr<xarm_msgs::srv::FtForceConfig::Request> req, std::shared_ptr<xarm_msgs::srv::FtForceConfig::Response> res);
    
        // FtForcePid
        rclcpp::Service<xarm_msgs::srv::FtForcePid>::SharedPtr service_set_force_control_pid_;
        bool _set_force_control_pid(const std::shared_ptr<xarm_msgs::srv::FtForcePid::Request> req, std::shared_ptr<xarm_msgs::srv::FtForcePid::Response> res);
    
        // FtImpedance
        rclcpp::Service<xarm_msgs::srv::FtImpedance>::SharedPtr service_set_impedance_;
        rclcpp::Service<xarm_msgs::srv::FtImpedance>::SharedPtr service_set_impedance_mbk_;
        rclcpp::Service<xarm_msgs::srv::FtImpedance>::SharedPtr service_set_impedance_config_;
        bool _set_impedance(const std::shared_ptr<xarm_msgs::srv::FtImpedance::Request> req, std::shared_ptr<xarm_msgs::srv::FtImpedance::Response> res);
        bool _set_impedance_mbk(const std::shared_ptr<xarm_msgs::srv::FtImpedance::Request> req, std::shared_ptr<xarm_msgs::srv::FtImpedance::Response> res);
        bool _set_impedance_config(const std::shared_ptr<xarm_msgs::srv::FtImpedance::Request> req, std::shared_ptr<xarm_msgs::srv::FtImpedance::Response> res);

        // LinearTrackBackOrigin
        rclcpp::Service<xarm_msgs::srv::LinearTrackBackOrigin>::SharedPtr service_set_linear_track_back_origin_;
        bool _set_linear_track_back_origin(const std::shared_ptr<xarm_msgs::srv::LinearTrackBackOrigin::Request> req, std::shared_ptr<xarm_msgs::srv::LinearTrackBackOrigin::Response> res);

        // LinearTrackSetPos
        rclcpp::Service<xarm_msgs::srv::LinearTrackSetPos>::SharedPtr service_set_linear_track_pos_;
        bool _set_linear_track_pos(const std::shared_ptr<xarm_msgs::srv::LinearTrackSetPos::Request> req, std::shared_ptr<xarm_msgs::srv::LinearTrackSetPos::Response> res);
    };
}

#endif // __XARM_DRIVER_H