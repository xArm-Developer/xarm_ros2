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
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

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
    
    public:
        XArmAPI *arm;

    private:
        std::string report_type_;
        int dof_;
        int curr_state_;
        int curr_err_;

        rclcpp::Node::SharedPtr node_;
        rclcpp::Service<xarm_msgs::srv::SetAxis>::SharedPtr motion_ctrl_server_;
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr set_mode_server_;
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr set_state_server_;
        rclcpp::Service<xarm_msgs::srv::TCPOffset>::SharedPtr set_tcp_offset_server_;
        rclcpp::Service<xarm_msgs::srv::SetLoad>::SharedPtr set_load_server_;
        rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr go_home_server_;
        rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr move_joint_server_;
        rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr move_jointb_server_;
        rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr move_lineb_server_;
        rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr move_line_server_;
        rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr move_line_tool_server_;
        rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr move_servoj_server_;
        rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr move_servo_cart_server_;
        rclcpp::Service<xarm_msgs::srv::ClearErr>::SharedPtr clear_err_server_;
        rclcpp::Service<xarm_msgs::srv::ClearErr>::SharedPtr moveit_clear_err_server_;
        rclcpp::Service<xarm_msgs::srv::GetErr>::SharedPtr get_err_server_;
        rclcpp::Service<xarm_msgs::srv::MoveAxisAngle>::SharedPtr move_line_aa_server_;
        rclcpp::Service<xarm_msgs::srv::MoveAxisAngle>::SharedPtr move_servo_cart_aa_server_;
        rclcpp::Service<xarm_msgs::srv::SetDigitalIO>::SharedPtr set_end_io_server_;
        rclcpp::Service<xarm_msgs::srv::GetDigitalIO>::SharedPtr get_digital_in_server_;
        rclcpp::Service<xarm_msgs::srv::GetAnalogIO>::SharedPtr get_analog_in_server_;
        rclcpp::Service<xarm_msgs::srv::ConfigToolModbus>::SharedPtr config_modbus_server_;
        rclcpp::Service<xarm_msgs::srv::SetToolModbus>::SharedPtr set_modbus_server_;
        rclcpp::Service<xarm_msgs::srv::GripperConfig>::SharedPtr gripper_config_server_;
        rclcpp::Service<xarm_msgs::srv::GripperMove>::SharedPtr gripper_move_server_;
        rclcpp::Service<xarm_msgs::srv::GripperState>::SharedPtr gripper_state_server_;
        rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr set_vacuum_gripper_server_;
        rclcpp::Service<xarm_msgs::srv::SetDigitalIO>::SharedPtr set_controller_dout_server_;
        rclcpp::Service<xarm_msgs::srv::GetControllerDigitalIO>::SharedPtr get_controller_din_server_;
        rclcpp::Service<xarm_msgs::srv::SetControllerAnalogIO>::SharedPtr set_controller_aout_server_;
        rclcpp::Service<xarm_msgs::srv::GetAnalogIO>::SharedPtr get_controller_ain_server_;
        rclcpp::Service<xarm_msgs::srv::MoveVelo>::SharedPtr vc_set_jointv_server_;
        rclcpp::Service<xarm_msgs::srv::MoveVelo>::SharedPtr vc_set_linev_server_;
        rclcpp::Service<xarm_msgs::srv::SetFloat32>::SharedPtr set_max_jacc_server_;
        rclcpp::Service<xarm_msgs::srv::SetFloat32>::SharedPtr set_max_lacc_server_;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        rclcpp::Publisher<xarm_msgs::msg::RobotMsg>::SharedPtr robot_state_pub_;
        rclcpp::Publisher<xarm_msgs::msg::CIOState>::SharedPtr cgpio_state_pub_;

        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sleep_sub_;

    };
}

#endif // __XARM_DRIVER_H