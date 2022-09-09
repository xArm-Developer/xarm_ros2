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
        void init(rclcpp::Node::SharedPtr& node, std::string &server_ip);

        void pub_robot_msg(xarm_msgs::msg::RobotMsg &rm_msg);
        void pub_joint_state(sensor_msgs::msg::JointState &js_msg);
        void pub_cgpio_state(xarm_msgs::msg::CIOState &cio_msg);
        void pub_ftsensor_ext_state(geometry_msgs::msg::WrenchStamped &wrench_msg);
        void pub_ftsensor_raw_state(geometry_msgs::msg::WrenchStamped &wrench_msg);
        
        bool is_connected(void);
        std::string controller_error_interpreter(int err=-1);

        rclcpp::Logger get_logger() { return node_->get_logger(); }

    private:
        void _report_connect_changed_callback(bool connected, bool reported);
        void _report_data_callback(XArmReportData *report_data_ptr);
        bool _get_wait_param(void);

        void _init_gripper(void);
        inline float _gripper_pos_convert(float pos, bool reversed = false);
        rclcpp_action::GoalResponse _handle_gripper_action_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const control_msgs::action::GripperCommand::Goal> goal);
        rclcpp_action::CancelResponse _handle_gripper_action_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle);
        void _handle_gripper_action_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle);
        void _gripper_action_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle);
        void _pub_gripper_joint_states(float pos);

        template<typename ServiceT, typename CallbackT>
    	typename rclcpp::Service<ServiceT>::SharedPtr _create_service(const std::string & service_name, CallbackT && callback);

        void _init_publisher(void);
        void _init_service(void);

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

        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sleep_sub_;

        bool gripper_init_loop_;
        int gripper_speed_;
        int gripper_max_pos_;
        int gripper_frequency_;
        int gripper_threshold_;
        int gripper_threshold_times_;
        sensor_msgs::msg::JointState gripper_joint_state_msg_;
        control_msgs::action::GripperCommand::Feedback::SharedPtr gripper_feedback_;
        control_msgs::action::GripperCommand::Result::SharedPtr gripper_result_;
        rclcpp_action::Server<control_msgs::action::GripperCommand>::SharedPtr gripper_action_server_;
    
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

        // GetInt16
        rclcpp::Service<xarm_msgs::srv::GetInt16>::SharedPtr service_get_state_;
        rclcpp::Service<xarm_msgs::srv::GetInt16>::SharedPtr service_get_cmdnum_;
        rclcpp::Service<xarm_msgs::srv::GetInt16>::SharedPtr service_get_vacuum_gripper_;
        rclcpp::Service<xarm_msgs::srv::GetInt16>::SharedPtr service_get_gripper_err_code_;
        rclcpp::Service<xarm_msgs::srv::GetInt16>::SharedPtr service_get_bio_gripper_status_;
        rclcpp::Service<xarm_msgs::srv::GetInt16>::SharedPtr service_get_bio_gripper_error_;
        bool _get_state(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res);
        bool _get_cmdnum(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res);
        bool _get_vacuum_gripper(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res);
        bool _get_gripper_err_code(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res);
        bool _get_bio_gripper_status(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res);
        bool _get_bio_gripper_error(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res);

        // GetInt16List
        rclcpp::Service<xarm_msgs::srv::GetInt16List>::SharedPtr service_get_err_warn_code_;
        bool _get_err_warn_code(const std::shared_ptr<xarm_msgs::srv::GetInt16List::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16List::Response> res);

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
        bool _get_position(const std::shared_ptr<xarm_msgs::srv::GetFloat32List::Request> req, std::shared_ptr<xarm_msgs::srv::GetFloat32List::Response> res);
        bool _get_servo_angle(const std::shared_ptr<xarm_msgs::srv::GetFloat32List::Request> req, std::shared_ptr<xarm_msgs::srv::GetFloat32List::Response> res);
        bool _get_position_aa(const std::shared_ptr<xarm_msgs::srv::GetFloat32List::Request> req, std::shared_ptr<xarm_msgs::srv::GetFloat32List::Response> res);
        
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
        bool _vc_set_joint_velocity(const std::shared_ptr<xarm_msgs::srv::MoveVelocity::Request> req, std::shared_ptr<xarm_msgs::srv::MoveVelocity::Response> res);
        bool _vc_set_cartesian_velocity(const std::shared_ptr<xarm_msgs::srv::MoveVelocity::Request> req, std::shared_ptr<xarm_msgs::srv::MoveVelocity::Response> res);

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
    };
}

#endif // __XARM_DRIVER_H