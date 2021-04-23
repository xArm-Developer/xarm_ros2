/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason <jason@ufactory.cc>
           Vinman <vinman@ufactory.cc> 
 ============================================================================*/
 
#ifndef __XARM_ROS_CLIENT_H__
#define __XARM_ROS_CLIENT_H__

#include <rclcpp/rclcpp.hpp>
#include "xarm_msgs.h"
#include "visibility_control.h"

namespace xarm_api
{

class XARM_API_PUBLIC XArmROSClient
{
public:
	XArmROSClient();
	void init(rclcpp::Node::SharedPtr& node);
	~XArmROSClient();

	int motionEnable(short en);
	int setState(short state);
	int setMode(short mode);
	int clearErr(void);
	int getErr(int *err);
	int setTCPOffset(const std::vector<float>& tcp_offset);
	int setLoad(float mass, const std::vector<float>& center_of_mass);
	int setServoJ(const std::vector<float>& joint_cmd);
	int setServoCartisian(const std::vector<float>& cart_cmd);
	int goHome(float jnt_vel_rad, float jnt_acc_rad=15);
	int moveJoint(const std::vector<float>& joint_cmd, float jnt_vel_rad, float jnt_acc_rad=15);
	int moveLine(const std::vector<float>& cart_cmd, float cart_vel_mm, float cart_acc_mm=500);
	int moveLineB(int num_of_pnts, const std::vector<float> cart_cmds[], float cart_vel_mm, float cart_acc_mm=500, float radii=0);
	int getGripperState(float *curr_pulse, int *curr_err);
	int gripperConfig(float pulse_vel);
	int gripperMove(float pulse);
	
	int config_tool_modbus(int baud_rate, int time_out_ms);
	int send_tool_modbus(unsigned char* data, int send_len, unsigned char* recv_data=NULL, int recv_len=0);

	int veloMoveJoint(const std::vector<float>& jnt_v, bool is_sync = true);
	int veloMoveLine(const std::vector<float>& line_v, bool is_tool_coord = false);

private:
	template<typename ServiceT, typename SharedRequest = typename ServiceT::Request::SharedPtr>
	int _call_request(std::shared_ptr<ServiceT> client, SharedRequest req);

	template<typename ServiceT, typename SharedRequest = typename ServiceT::Request::SharedPtr, typename SharedResponse = typename ServiceT::Response::SharedPtr>
	int _call_request(std::shared_ptr<ServiceT> client, SharedRequest req, SharedResponse res);

private:
	rclcpp::Client<xarm_msgs::srv::SetAxis>::SharedPtr motion_ctrl_client_;
	rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr set_mode_client_;
	rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr set_state_client_;
	rclcpp::Client<xarm_msgs::srv::TCPOffset>::SharedPtr set_tcp_offset_client_;
	rclcpp::Client<xarm_msgs::srv::SetLoad>::SharedPtr set_load_client_;
	rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr go_home_client_;
	rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr move_joint_client_;
	rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr move_jointb_client_;
	rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr move_lineb_client_;
	rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr move_line_client_;
	rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr move_line_tool_client_;
	rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr move_servoj_client_;
	rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr move_servo_cart_client_;
	rclcpp::Client<xarm_msgs::srv::ClearErr>::SharedPtr clear_err_client_;
	rclcpp::Client<xarm_msgs::srv::GetErr>::SharedPtr get_err_client_;
	rclcpp::Client<xarm_msgs::srv::MoveAxisAngle>::SharedPtr move_line_aa_client_;
	rclcpp::Client<xarm_msgs::srv::MoveAxisAngle>::SharedPtr move_servo_cart_aa_client_;
	rclcpp::Client<xarm_msgs::srv::SetDigitalIO>::SharedPtr set_end_io_client_;
	rclcpp::Client<xarm_msgs::srv::GetDigitalIO>::SharedPtr get_digital_in_client_;
	rclcpp::Client<xarm_msgs::srv::GetAnalogIO>::SharedPtr get_analog_in_client_;
	rclcpp::Client<xarm_msgs::srv::ConfigToolModbus>::SharedPtr config_modbus_client_;
	rclcpp::Client<xarm_msgs::srv::SetToolModbus>::SharedPtr set_modbus_client_;
	rclcpp::Client<xarm_msgs::srv::GripperConfig>::SharedPtr gripper_config_client_;
	rclcpp::Client<xarm_msgs::srv::GripperMove>::SharedPtr gripper_move_client_;
	rclcpp::Client<xarm_msgs::srv::GripperState>::SharedPtr gripper_state_client_;
	rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr set_vacuum_gripper_client_;
	rclcpp::Client<xarm_msgs::srv::SetDigitalIO>::SharedPtr set_controller_dout_client_;
	rclcpp::Client<xarm_msgs::srv::GetControllerDigitalIO>::SharedPtr get_controller_din_client_;
	rclcpp::Client<xarm_msgs::srv::SetControllerAnalogIO>::SharedPtr set_controller_aout_client_;
	rclcpp::Client<xarm_msgs::srv::GetAnalogIO>::SharedPtr get_controller_ain_client_;
	rclcpp::Client<xarm_msgs::srv::MoveVelo>::SharedPtr velo_move_joint_client_;
	rclcpp::Client<xarm_msgs::srv::MoveVelo>::SharedPtr velo_move_line_client_;
	rclcpp::Client<xarm_msgs::srv::SetFloat32>::SharedPtr set_max_jacc_client_;
	rclcpp::Client<xarm_msgs::srv::SetFloat32>::SharedPtr set_max_lacc_client_;

	std::shared_ptr<xarm_msgs::srv::SetAxis::Request> set_axis_req_;
    std::shared_ptr<xarm_msgs::srv::SetInt16::Request> set_int16_req_;
    std::shared_ptr<xarm_msgs::srv::TCPOffset::Request> offset_req_;
    std::shared_ptr<xarm_msgs::srv::SetLoad::Request> set_load_req_;
    std::shared_ptr<xarm_msgs::srv::ClearErr::Request> clear_err_req_;
    std::shared_ptr<xarm_msgs::srv::GetErr::Request> get_err_req_;
    std::shared_ptr<xarm_msgs::srv::Move::Request> move_req_;
    std::shared_ptr<xarm_msgs::srv::Move::Request> servoj_req_;
    std::shared_ptr<xarm_msgs::srv::Move::Request> servo_cart_req_;
    std::shared_ptr<xarm_msgs::srv::ConfigToolModbus::Request> cfg_modbus_req_;
    std::shared_ptr<xarm_msgs::srv::SetToolModbus::Request> set_modbus_req_;
    std::shared_ptr<xarm_msgs::srv::GripperConfig::Request> gripper_config_req_;
    std::shared_ptr<xarm_msgs::srv::GripperMove::Request> gripper_move_req_;
    std::shared_ptr<xarm_msgs::srv::GripperState::Request> gripper_state_req_;
    std::shared_ptr<xarm_msgs::srv::MoveVelo::Request> move_velo_req_;

    rclcpp::Node::SharedPtr node_;
};

} 


#endif // __XARM_ROS_CLIENT_H__