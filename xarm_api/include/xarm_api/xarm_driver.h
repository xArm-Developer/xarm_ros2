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

        // provide a list of services:
        bool MotionCtrlCB(const std::shared_ptr<xarm_msgs::srv::SetAxis::Request> req, std::shared_ptr<xarm_msgs::srv::SetAxis::Response> res);
        bool SetModeCB(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
        bool SetStateCB(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
        bool SetTCPOffsetCB(const std::shared_ptr<xarm_msgs::srv::TCPOffset::Request> req, std::shared_ptr<xarm_msgs::srv::TCPOffset::Response> res);
        bool SetLoadCB(const std::shared_ptr<xarm_msgs::srv::SetLoad::Request> req, std::shared_ptr<xarm_msgs::srv::SetLoad::Response> res);
        bool SetDigitalIOCB(const std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Request> req, std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Response> res);
        bool GetDigitalIOCB(const std::shared_ptr<xarm_msgs::srv::GetDigitalIO::Request> req, std::shared_ptr<xarm_msgs::srv::GetDigitalIO::Response> res);
        bool GetAnalogIOCB(const std::shared_ptr<xarm_msgs::srv::GetAnalogIO::Request> req, std::shared_ptr<xarm_msgs::srv::GetAnalogIO::Response> res);
        bool ClearErrCB(const std::shared_ptr<xarm_msgs::srv::ClearErr::Request> req, std::shared_ptr<xarm_msgs::srv::ClearErr::Response> res);
        bool MoveitClearErrCB(const std::shared_ptr<xarm_msgs::srv::ClearErr::Request> req, std::shared_ptr<xarm_msgs::srv::ClearErr::Response> res);
        bool GetErrCB(const std::shared_ptr<xarm_msgs::srv::GetErr::Request> req, std::shared_ptr<xarm_msgs::srv::GetErr::Response> res);
        bool GoHomeCB(const std::shared_ptr<xarm_msgs::srv::Move::Request> req, std::shared_ptr<xarm_msgs::srv::Move::Response> res);
        bool MoveJointCB(const std::shared_ptr<xarm_msgs::srv::Move::Request> req, std::shared_ptr<xarm_msgs::srv::Move::Response> res);
        bool MoveJointbCB(const std::shared_ptr<xarm_msgs::srv::Move::Request> req, std::shared_ptr<xarm_msgs::srv::Move::Response> res);
        bool MoveLinebCB(const std::shared_ptr<xarm_msgs::srv::Move::Request> req, std::shared_ptr<xarm_msgs::srv::Move::Response> res);
        bool MoveLineCB(const std::shared_ptr<xarm_msgs::srv::Move::Request> req, std::shared_ptr<xarm_msgs::srv::Move::Response> res);
        bool MoveLineToolCB(const std::shared_ptr<xarm_msgs::srv::Move::Request> req, std::shared_ptr<xarm_msgs::srv::Move::Response> res);
        bool MoveServoJCB(const std::shared_ptr<xarm_msgs::srv::Move::Request> req, std::shared_ptr<xarm_msgs::srv::Move::Response> res);
        bool MoveServoCartCB(const std::shared_ptr<xarm_msgs::srv::Move::Request> req, std::shared_ptr<xarm_msgs::srv::Move::Response> res);
        bool MoveLineAACB(const std::shared_ptr<xarm_msgs::srv::MoveAxisAngle::Request> req, std::shared_ptr<xarm_msgs::srv::MoveAxisAngle::Response> res);
        bool MoveServoCartAACB(const std::shared_ptr<xarm_msgs::srv::MoveAxisAngle::Request> req, std::shared_ptr<xarm_msgs::srv::MoveAxisAngle::Response> res);
        bool GripperConfigCB(const std::shared_ptr<xarm_msgs::srv::GripperConfig::Request> req, std::shared_ptr<xarm_msgs::srv::GripperConfig::Response> res);
        bool GripperMoveCB(const std::shared_ptr<xarm_msgs::srv::GripperMove::Request> req, std::shared_ptr<xarm_msgs::srv::GripperMove::Response> res);
        bool GripperStateCB(const std::shared_ptr<xarm_msgs::srv::GripperState::Request> req, std::shared_ptr<xarm_msgs::srv::GripperState::Response> res);
        bool VacuumGripperCB(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
        bool SetModbusCB(const std::shared_ptr<xarm_msgs::srv::SetToolModbus::Request> req, std::shared_ptr<xarm_msgs::srv::SetToolModbus::Response> res);
        bool ConfigModbusCB(const std::shared_ptr<xarm_msgs::srv::ConfigToolModbus::Request> req, std::shared_ptr<xarm_msgs::srv::ConfigToolModbus::Response> res);
        bool SetControllerDOutCB(const std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Request> req, std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Response> res);
        bool GetControllerDInCB(const std::shared_ptr<xarm_msgs::srv::GetControllerDigitalIO::Request> req, std::shared_ptr<xarm_msgs::srv::GetControllerDigitalIO::Response> res);
        bool SetControllerAOutCB(const std::shared_ptr<xarm_msgs::srv::SetControllerAnalogIO::Request> req, std::shared_ptr<xarm_msgs::srv::SetControllerAnalogIO::Response> res);
        bool GetControllerAInCB(const std::shared_ptr<xarm_msgs::srv::GetAnalogIO::Request> req, std::shared_ptr<xarm_msgs::srv::GetAnalogIO::Response> res);
        bool VeloMoveJointCB(const std::shared_ptr<xarm_msgs::srv::MoveVelo::Request> req, std::shared_ptr<xarm_msgs::srv::MoveVelo::Response> res);
        bool VeloMoveLineVCB(const std::shared_ptr<xarm_msgs::srv::MoveVelo::Request> req, std::shared_ptr<xarm_msgs::srv::MoveVelo::Response> res);
        bool SetMaxJAccCB(const std::shared_ptr<xarm_msgs::srv::SetFloat32::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32::Response> res);
        bool SetMaxLAccCB(const std::shared_ptr<xarm_msgs::srv::SetFloat32::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32::Response> res);

        void SleepTopicCB(const std_msgs::msg::Float32::SharedPtr msg);

        void pub_robot_msg(xarm_msgs::msg::RobotMsg &rm_msg);
        void pub_joint_state(sensor_msgs::msg::JointState &js_msg);
        void pub_cgpio_state(xarm_msgs::msg::CIOState &cio_msg);

        rclcpp::Logger get_logger() { return node_->get_logger(); }

    private:
        void _report_connect_changed_callback(bool connected, bool reported);
        void _report_data_callback(XArmReportData *report_data_ptr);
        bool _get_wait_param(void);

        void _init_gripper();
        inline float _gripper_pos_convert(float pos, bool reversed = false);
        rclcpp_action::GoalResponse _handle_gripper_action_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const control_msgs::action::GripperCommand::Goal> goal);
        rclcpp_action::CancelResponse _handle_gripper_action_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle);
        void _handle_gripper_action_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle);
        void _gripper_action_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle);
        void _pub_gripper_joint_states(float pos);

    public:
        XArmAPI *arm;

    private:
        std::string report_type_;
        int dof_;
        int curr_state_;
        int curr_err_;

        rclcpp::Node::SharedPtr node_;
        rclcpp::Node::SharedPtr hw_node_;
        rclcpp::Service<xarm_msgs::srv::SetAxis>::SharedPtr motion_ctrl_service_;
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr set_mode_service_;
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr set_state_service_;
        rclcpp::Service<xarm_msgs::srv::TCPOffset>::SharedPtr set_tcp_offset_service_;
        rclcpp::Service<xarm_msgs::srv::SetLoad>::SharedPtr set_load_service_;
        rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr go_home_service_;
        rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr move_joint_service_;
        rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr move_jointb_service_;
        rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr move_lineb_service_;
        rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr move_line_service_;
        rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr move_line_tool_service_;
        rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr move_servoj_service_;
        rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr move_servo_cart_service_;
        rclcpp::Service<xarm_msgs::srv::ClearErr>::SharedPtr clear_err_service_;
        rclcpp::Service<xarm_msgs::srv::ClearErr>::SharedPtr moveit_clear_err_service_;
        rclcpp::Service<xarm_msgs::srv::GetErr>::SharedPtr get_err_service_;
        rclcpp::Service<xarm_msgs::srv::MoveAxisAngle>::SharedPtr move_line_aa_service_;
        rclcpp::Service<xarm_msgs::srv::MoveAxisAngle>::SharedPtr move_servo_cart_aa_service_;
        rclcpp::Service<xarm_msgs::srv::SetDigitalIO>::SharedPtr set_end_io_service_;
        rclcpp::Service<xarm_msgs::srv::GetDigitalIO>::SharedPtr get_digital_in_service_;
        rclcpp::Service<xarm_msgs::srv::GetAnalogIO>::SharedPtr get_analog_in_service_;
        rclcpp::Service<xarm_msgs::srv::ConfigToolModbus>::SharedPtr config_modbus_service_;
        rclcpp::Service<xarm_msgs::srv::SetToolModbus>::SharedPtr set_modbus_service_;
        rclcpp::Service<xarm_msgs::srv::GripperConfig>::SharedPtr gripper_config_service_;
        rclcpp::Service<xarm_msgs::srv::GripperMove>::SharedPtr gripper_move_service_;
        rclcpp::Service<xarm_msgs::srv::GripperState>::SharedPtr gripper_state_service_;
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr set_vacuum_gripper_service_;
        rclcpp::Service<xarm_msgs::srv::SetDigitalIO>::SharedPtr set_controller_dout_service_;
        rclcpp::Service<xarm_msgs::srv::GetControllerDigitalIO>::SharedPtr get_controller_din_service_;
        rclcpp::Service<xarm_msgs::srv::SetControllerAnalogIO>::SharedPtr set_controller_aout_service_;
        rclcpp::Service<xarm_msgs::srv::GetAnalogIO>::SharedPtr get_controller_ain_service_;
        rclcpp::Service<xarm_msgs::srv::MoveVelo>::SharedPtr vc_set_jointv_service_;
        rclcpp::Service<xarm_msgs::srv::MoveVelo>::SharedPtr vc_set_linev_service_;
        rclcpp::Service<xarm_msgs::srv::SetFloat32>::SharedPtr set_max_jacc_service_;
        rclcpp::Service<xarm_msgs::srv::SetFloat32>::SharedPtr set_max_lacc_service_;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        rclcpp::Publisher<xarm_msgs::msg::RobotMsg>::SharedPtr robot_state_pub_;
        rclcpp::Publisher<xarm_msgs::msg::CIOState>::SharedPtr cgpio_state_pub_;

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
    
    // private:
    //     // SetInt16
    //     bool _set_mode(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
    //     bool _set_state(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
    //     bool _set_collision_sensitivity(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
    //     bool _set_teach_sensitivity(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
    //     bool _shutdown_system(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);

    //     // Call
    //     bool _clean_error(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res);
    //     bool _clean_warn(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res);
    //     bool _clean_conf(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res);
    //     bool _save_conf(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res);
    //     bool _reload_dynamics(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res);
    //     bool _set_counter_reset(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res);
    //     bool _set_counter_increase(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res);

    //     // SetInt16ById
    //     bool _motion_enable(const std::shared_ptr<xarm_msgs::srv::SetInt16ById::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16ById::Response> res);
    //     bool _set_servo_attach(const std::shared_ptr<xarm_msgs::srv::SetInt16ById::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16ById::Response> res);
    //     bool _set_servo_detach(const std::shared_ptr<xarm_msgs::srv::SetInt16ById::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16ById::Response> res);
        
    //     // GetString
    //     bool _get_version(const std::shared_ptr<xarm_msgs::srv::GetString::Request> req, std::shared_ptr<xarm_msgs::srv::GetString::Response> res);
    //     bool _get_robot_sn(const std::shared_ptr<xarm_msgs::srv::GetString::Request> req, std::shared_ptr<xarm_msgs::srv::GetString::Response> res);
        
    //     // GetInt16
    //     bool _get_state(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res);
    //     bool _get_cmdnum(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res);

    //     // SetFloat32
    //     bool _set_pause_time(const std::shared_ptr<xarm_msgs::srv::SetFloat32::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32::Response> res);
    //     bool _set_tcp_jerk(const std::shared_ptr<xarm_msgs::srv::SetFloat32::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32::Response> res);
    //     bool _set_tcp_maxacc(const std::shared_ptr<xarm_msgs::srv::SetFloat32::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32::Response> res);
    //     bool _set_joint_jerk(const std::shared_ptr<xarm_msgs::srv::SetFloat32::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32::Response> res);
    //     bool _set_joint_maxacc(const std::shared_ptr<xarm_msgs::srv::SetFloat32::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32::Response> res);

    //     //GetInt16List
    //     bool _get_err_warn_code(const std::shared_ptr<xarm_msgs::srv::GetInt16List::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16List::Response> res);
        
    //     // GetFloat32List
    //     bool _get_position(const std::shared_ptr<xarm_msgs::srv::GetFloat32List::Request> req, std::shared_ptr<xarm_msgs::srv::GetFloat32List::Response> res);
    //     bool _get_servo_angle(const std::shared_ptr<xarm_msgs::srv::GetFloat32List::Request> req, std::shared_ptr<xarm_msgs::srv::GetFloat32List::Response> res);
    //     bool _get_position_aa(const std::shared_ptr<xarm_msgs::srv::GetFloat32List::Request> req, std::shared_ptr<xarm_msgs::srv::GetFloat32List::Response> res);
        
    //     // SetFloat32List
    //     bool _set_gravity_direction(const std::shared_ptr<xarm_msgs::srv::SetFloat32List::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32List::Response> res);
    //     bool _set_tcp_load(const std::shared_ptr<xarm_msgs::srv::SetFloat32List::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32List::Response> res);
    //     bool _set_tcp_offset(const std::shared_ptr<xarm_msgs::srv::SetFloat32List::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32List::Response> res);
    //     bool _set_world_offset(const std::shared_ptr<xarm_msgs::srv::SetFloat32List::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32List::Response> res);

    //     // MoveLine
    //     bool _move_line()
    //     bool _move_line_aa()

    //     // MoveArcLine
    //     bool _move_arc_line()
        
    //     // MoveJoint
    //     // MoveArcJoint
    //     // MoveCircle
    //     // MoveGoHome


    //     // Move
    //     bool _set_position(const std::shared_ptr<xarm_msgs::srv::Move::Request> req, std::shared_ptr<xarm_msgs::srv::Move::Response> res);
    //     bool _set_tool_position(const std::shared_ptr<xarm_msgs::srv::Move::Request> req, std::shared_ptr<xarm_msgs::srv::Move::Response> res);
    //     bool _set_servo_angle(const std::shared_ptr<xarm_msgs::srv::Move::Request> req, std::shared_ptr<xarm_msgs::srv::Move::Response> res);
    //     bool _set_servo_angle_j(const std::shared_ptr<xarm_msgs::srv::Move::Request> req, std::shared_ptr<xarm_msgs::srv::Move::Response> res);

    //     // MoveAA
    //     bool _set_position_aa(const std::shared_ptr<xarm_msgs::srv::MoveAA::Request> req, std::shared_ptr<xarm_msgs::srv::MoveAA::Response> res);
    //     bool _set_servo_cartesian_aa(const std::shared_ptr<xarm_msgs::srv::MoveAA::Request> req, std::shared_ptr<xarm_msgs::srv::MoveAA::Response> res);

    //     bool _get_pose_offset();
        
        
        
    //     bool _set_servo_cartesian();
        
    //     bool _move_circle();
    //     bool _move_gohome();
    //     bool _reset();
    //     bool _vc_set_joint_velocity();
    //     bool _vc_set_cartesian_velocity();
    //     bool _emergency_stop();
        
    //     bool _get_gripper_version();
    //     bool _set_gripper_enable();
    //     bool _set_gripper_mode();
    //     bool _get_gripper_position();
    //     bool _set_gripper_position();
    //     bool _set_gripper_speed();
    //     bool _get_gripper_err_code();
    //     bool _clean_gripper_error();

    //     bool _get_tgpio_version();
    //     bool _get_tgpio_digital();
    //     bool _set_tgpio_digital();
    //     bool _get_tgpio_analog();
    //     bool _set_tgpio_digital_with_xyz();
    //     bool _config_tgpio_reset_when_stop();
    //     bool _get_cgpio_digital();
    //     bool _get_cgpio_analog();
    //     bool _set_cgpio_digital();
    //     bool _set_cgpio_analog();
    //     bool _set_cgpio_digital_input_function();
    //     bool _set_cgpio_digital_output_function();
    //     bool _get_cgpio_state();
    //     bool _set_cgpio_digital_with_xyz();
    //     bool _set_cgpio_analog_with_xyz();
    //     bool _config_cgpio_reset_when_stop();

    //     bool _get_vacuum_gripper(); // get_suction_cup
    //     bool _set_vacuum_gripper(); // set_suction_cup

    //     bool _get_reduced_mode();
    //     bool _set_reduced_mode();
    //     bool _set_reduced_max_tcp_speed();
    //     bool _set_reduced_max_joint_speed();
    //     bool _set_reduced_tcp_boundary();
    //     bool _set_reduced_joint_range();
    //     bool _set_fence_mode();
    //     bool _set_collision_rebound();
    //     bool _get_reduced_states();

    //     bool _start_record_trajectory();
    //     bool _stop_record_trajectory();
    //     bool _save_record_trajectory();
    //     bool _load_trajectory();
    //     bool _playback_trajectory();

    //     bool _robotiq_reset();
    //     bool _robotiq_set_activate();
    //     bool _robotiq_set_position();
    //     bool _robotiq_open();
    //     bool _robotiq_close();
    //     bool _robotiq_get_status();

    //     bool _set_bio_gripper_enable();
    //     bool _set_bio_gripper_speed();
    //     bool _open_bio_gripper();
    //     bool _close_bio_gripper();
    //     bool _get_bio_gripper_status();
    //     bool _get_bio_gripper_error();
    //     bool _clean_bio_gripper_error();

    //     bool _set_tgpio_modbus_timeout();
    //     bool _set_tgpio_modbus_baudrate();
    //     bool _get_tgpio_modbus_baudrate();
    //     bool _getset_tgpio_modbus_data();
    };
}

#endif // __XARM_DRIVER_H