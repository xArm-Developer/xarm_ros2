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
#define TIMEOUT_10 10
#define NO_TIMEOUT -1
typedef unsigned int u32;
typedef float fp32;

class XARM_API_PUBLIC XArmROSClient
{
public:
	XArmROSClient();
	void init(rclcpp::Node::SharedPtr& node, std::string hw_ns = "xarm");
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

	template<typename ServiceT>
	typename rclcpp::Client<ServiceT>::SharedPtr _create_client(const std::string & service_name);

private:
	// rclcpp::Client<xarm_msgs::srv::SetAxis>::SharedPtr motion_ctrl_client_;
	// rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr set_mode_client_;
	// rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr set_state_client_;
	// rclcpp::Client<xarm_msgs::srv::TCPOffset>::SharedPtr set_tcp_offset_client_;
	// rclcpp::Client<xarm_msgs::srv::SetLoad>::SharedPtr set_load_client_;
	// rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr go_home_client_;
	// rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr move_joint_client_;
	// rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr move_jointb_client_;
	// rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr move_lineb_client_;
	// rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr move_line_client_;
	// rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr move_line_tool_client_;
	// rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr move_servoj_client_;
	// rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr move_servo_cart_client_;
	// rclcpp::Client<xarm_msgs::srv::ClearErr>::SharedPtr clear_err_client_;
	// rclcpp::Client<xarm_msgs::srv::GetErr>::SharedPtr get_err_client_;
	// rclcpp::Client<xarm_msgs::srv::MoveAxisAngle>::SharedPtr move_line_aa_client_;
	// rclcpp::Client<xarm_msgs::srv::MoveAxisAngle>::SharedPtr move_servo_cart_aa_client_;
	// rclcpp::Client<xarm_msgs::srv::SetDigitalIO>::SharedPtr set_end_io_client_;
	// rclcpp::Client<xarm_msgs::srv::GetDigitalIO>::SharedPtr get_digital_in_client_;
	// rclcpp::Client<xarm_msgs::srv::GetAnalogIO>::SharedPtr get_analog_in_client_;
	// rclcpp::Client<xarm_msgs::srv::ConfigToolModbus>::SharedPtr config_modbus_client_;
	// rclcpp::Client<xarm_msgs::srv::SetToolModbus>::SharedPtr set_modbus_client_;
	// rclcpp::Client<xarm_msgs::srv::GripperConfig>::SharedPtr gripper_config_client_;
	// rclcpp::Client<xarm_msgs::srv::GripperMove>::SharedPtr gripper_move_client_;
	// rclcpp::Client<xarm_msgs::srv::GripperState>::SharedPtr gripper_state_client_;
	// rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr set_vacuum_gripper_client_;
	// rclcpp::Client<xarm_msgs::srv::SetDigitalIO>::SharedPtr set_controller_dout_client_;
	// rclcpp::Client<xarm_msgs::srv::GetControllerDigitalIO>::SharedPtr get_controller_din_client_;
	// rclcpp::Client<xarm_msgs::srv::SetControllerAnalogIO>::SharedPtr set_controller_aout_client_;
	// rclcpp::Client<xarm_msgs::srv::GetAnalogIO>::SharedPtr get_controller_ain_client_;
	// rclcpp::Client<xarm_msgs::srv::MoveVelo>::SharedPtr velo_move_joint_client_;
	// rclcpp::Client<xarm_msgs::srv::MoveVelo>::SharedPtr velo_move_line_client_;
	// rclcpp::Client<xarm_msgs::srv::SetFloat32>::SharedPtr set_max_jacc_client_;
	// rclcpp::Client<xarm_msgs::srv::SetFloat32>::SharedPtr set_max_lacc_client_;

	// std::shared_ptr<xarm_msgs::srv::SetAxis::Request> set_axis_req_;
    // std::shared_ptr<xarm_msgs::srv::SetInt16::Request> set_int16_req_;
    // std::shared_ptr<xarm_msgs::srv::TCPOffset::Request> offset_req_;
    // std::shared_ptr<xarm_msgs::srv::SetLoad::Request> set_load_req_;
    // std::shared_ptr<xarm_msgs::srv::ClearErr::Request> clear_err_req_;
    // std::shared_ptr<xarm_msgs::srv::GetErr::Request> get_err_req_;
    // std::shared_ptr<xarm_msgs::srv::Move::Request> move_req_;
    // std::shared_ptr<xarm_msgs::srv::Move::Request> servoj_req_;
    // std::shared_ptr<xarm_msgs::srv::Move::Request> servo_cart_req_;
    // std::shared_ptr<xarm_msgs::srv::ConfigToolModbus::Request> cfg_modbus_req_;
    // std::shared_ptr<xarm_msgs::srv::SetToolModbus::Request> set_modbus_req_;
    // std::shared_ptr<xarm_msgs::srv::GripperConfig::Request> gripper_config_req_;
    // std::shared_ptr<xarm_msgs::srv::GripperMove::Request> gripper_move_req_;
    // std::shared_ptr<xarm_msgs::srv::GripperState::Request> gripper_state_req_;
    // std::shared_ptr<xarm_msgs::srv::MoveVelo::Request> move_velo_req_;

	std::string hw_ns_;
    rclcpp::Node::SharedPtr node_;

public:
	// Call
	int clean_error();
	int clean_warn();
	int clean_conf();
	int save_conf();
	int reload_dynamics();
	int set_counter_reset();
	int set_counter_increase();
	int clean_gripper_error();
	int clean_bio_gripper_error();
	int start_record_trajectory();
	int stop_record_trajectory();

	// GetInt16
	int get_state(int *state);
	int get_cmdnum(int *cmdnum);
	int get_vacuum_gripper(int *status);
	int get_gripper_err_code(int *err);
	int get_bio_gripper_status(int *status);
	int get_bio_gripper_error(int *err);

	//GetErrWarn
	int get_err_warn_code(int err_warn[2]);

	// SetInt16
	int set_mode(int mode);
	int set_state(int state);
	int set_collision_sensitivity(int sensitivity);
	int set_teach_sensitivity(int sensitivity);
	int set_gripper_mode(int mode);
	int set_gripper_enable(bool enable);
	int set_tgpio_modbus_timeout(int timeout);
	int set_bio_gripper_speed(int speed);

	// SetInt16ById
	int motion_enable(bool enable, int servo_id = 8);
	int set_servo_attach(int servo_id);
	int set_servo_detach(int servo_id);

	// GetInt32
	int get_tgpio_modbus_baudrate(int *baudrate);

	// SetInt32
	int set_tgpio_modbus_baudrate(int baudrate);

	// GetFloat32
	int get_gripper_position(fp32 *pos);

	// GetFloat32List
	int get_position(fp32 pose[6]);
	int get_servo_angle(fp32 angles[7]);
	int get_position_aa(fp32 pose[6]);

	// SetFloat32
	int set_pause_time(fp32 sltime);
	int set_tcp_jerk(fp32 jerk);
	int set_tcp_maxacc(fp32 maxacc);
	int set_joint_jerk(fp32 jerk);
	int set_joint_maxacc(fp32 maxacc);
	int set_gripper_speed(fp32 speed);

	// SetFloat32List
	int set_gravity_direction(const std::vector<fp32>& gravity_dir);
	int set_tcp_load(fp32 weight, const std::vector<fp32>& center_of_gravity);
	int set_tcp_offset(const std::vector<fp32>& offset);
	int set_world_offset(const std::vector<fp32>& offset);

	// MoveCartesian
	int set_position(const std::vector<fp32>& pose, fp32 radius = -1, fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool wait = false, fp32 timeout = NO_TIMEOUT);
	int set_position(const std::vector<fp32>& pose, fp32 radius = -1, bool wait = false, fp32 timeout = NO_TIMEOUT);
	int set_position(const std::vector<fp32>& pose, bool wait = false, fp32 timeout = NO_TIMEOUT);
	int set_tool_position(const std::vector<fp32>& pose, fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool wait = false, fp32 timeout = NO_TIMEOUT);
	int set_tool_position(const std::vector<fp32>& pose, bool wait = false, fp32 timeout = NO_TIMEOUT);
	int set_position_aa(const std::vector<fp32>& pose, fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool is_tool_coord = false, bool relative = false, bool wait = false, fp32 timeout = NO_TIMEOUT);
	int set_position_aa(const std::vector<fp32>& pose, bool is_tool_coord = false, bool relative = false, bool wait = false, fp32 timeout = NO_TIMEOUT);
	int set_servo_cartesian(const std::vector<fp32>& pose, fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool is_tool_coord = false);	
	int set_servo_cartesian_aa(const std::vector<fp32>& pose, fp32 speed = 0, fp32 acc = 0, bool is_tool_coord = false, bool relative = false);
	int set_servo_cartesian_aa(const std::vector<fp32>& pose, bool is_tool_coord = false, bool relative = false);

	// MoveJoint
	int set_servo_angle(const std::vector<fp32>& angles, fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool wait = false, fp32 timeout = NO_TIMEOUT, fp32 radius = -1);
	int set_servo_angle(const std::vector<fp32>& angles, bool wait = false, fp32 timeout = NO_TIMEOUT, fp32 radius = -1);
	int set_servo_angle_j(const std::vector<fp32>& angles, fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0);

	// MoveCircle
	int move_circle(const std::vector<fp32>& pose1, const std::vector<fp32>& pose2, fp32 percent, fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool wait = false, fp32 timeout = NO_TIMEOUT);

	// MoveHome
	int move_gohome(fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool wait = false, fp32 timeout = NO_TIMEOUT);
	int move_gohome(bool wait = false, fp32 timeout = NO_TIMEOUT);

	// MoveVelocity
	int vc_set_joint_velocity(const std::vector<fp32>& speeds, bool is_sync = true);
	int vc_set_cartesian_velocity(const std::vector<fp32>& speeds, bool is_tool_coord = false);

	// GetDigitalIO
	int get_tgpio_digital(int *io0_value, int *io1_value);
	int get_cgpio_digital(int *digitals, int *digitals2 = NULL);

	// GetAnalogIO
	int get_tgpio_analog(int ionum, fp32 *value);
	int get_cgpio_analog(int ionum, fp32 *value);

	// SetDigitalIO
	int set_tgpio_digital(int ionum, int value, fp32 delay_sec=0);
	int set_cgpio_digital(int ionum, int value, fp32 delay_sec=0);
	int set_tgpio_digital_with_xyz(int ionum, int value, const std::vector<fp32>& xyz, fp32 tol_r);
	int set_cgpio_digital_with_xyz(int ionum, int value, const std::vector<fp32>& xyz, fp32 tol_r);

	// SetAnalogIO
	int set_cgpio_analog(int ionum, fp32 value);
	int set_cgpio_analog_with_xyz(int ionum, fp32 value, const std::vector<fp32>& xyz, fp32 tol_r);

	// VacuumGripperCtrl
	int set_vacuum_gripper(bool on, bool wait = false, float timeout = 3, float delay_sec = 0);
	
	// GripperMove
	int set_gripper_position(fp32 pos, bool wait = false, fp32 timeout = 10);

	// BioGripperEnable
	int set_bio_gripper_enable(bool enable, bool wait = true, fp32 timeout = 3);

	// BioGripperCtrl
	int open_bio_gripper(int speed = 0, bool wait = true, fp32 timeout = 5);
	int open_bio_gripper(bool wait = true, fp32 timeout = 5);
	int close_bio_gripper(int speed = 0, bool wait = true, fp32 timeout = 5);
	int close_bio_gripper(bool wait = true, fp32 timeout = 5);

	// RobotiqReset
	int robotiq_reset(unsigned char ret_data[6] = NULL);
	
	// RobotiqActivate
	int robotiq_set_activate(bool wait = true, fp32 timeout = 3, unsigned char ret_data[6] = NULL);
	int robotiq_set_activate(bool wait = true, unsigned char ret_data[6] = NULL);
	int robotiq_set_activate(unsigned char ret_data[6] = NULL);

	// RobotiqMove
	int robotiq_set_position(unsigned char pos, unsigned char speed = 0xFF, unsigned char force = 0xFF, bool wait = true, fp32 timeout = 5, unsigned char ret_data[6] = NULL);
	int robotiq_set_position(unsigned char pos, bool wait = true, fp32 timeout = 5, unsigned char ret_data[6] = NULL);
	int robotiq_set_position(unsigned char pos, bool wait = true, unsigned char ret_data[6] = NULL);
	int robotiq_set_position(unsigned char pos, unsigned char ret_data[6] = NULL);
	int robotiq_open(unsigned char speed = 0xFF, unsigned char force = 0xFF, bool wait = true, fp32 timeout = 5, unsigned char ret_data[6] = NULL);
	int robotiq_open(bool wait = true, fp32 timeout = 5, unsigned char ret_data[6] = NULL);
	int robotiq_open(bool wait = true, unsigned char ret_data[6] = NULL);
	int robotiq_open(unsigned char ret_data[6] = NULL);
	int robotiq_close(unsigned char speed = 0xFF, unsigned char force = 0xFF, bool wait = true, fp32 timeout = 5, unsigned char ret_data[6] = NULL);
	int robotiq_close(bool wait = true, fp32 timeout = 5, unsigned char ret_data[6] = NULL);
	int robotiq_close(bool wait = true, unsigned char ret_data[6] = NULL);
	int robotiq_close(unsigned char ret_data[6] = NULL);

	// RobotiqGetStatus
	int robotiq_get_status(unsigned char ret_data[9], unsigned char number_of_registers = 3);

	// GetSetModbusData
	int getset_tgpio_modbus_data(const std::vector<unsigned char>& modbus_data, int modbus_length, std::vector<unsigned char> ret_data, int ret_length);

	// TrajCtrl
	int save_record_trajectory(std::string& filename, float timeout = 10);
	int load_trajectory(std::string& filename, float timeout = 10);

	// TrajPlay
	int playback_trajectory(int times = 1, bool wait = false, int double_speed = 1, std::string filename = "");

private:
	// Call
	std::shared_ptr<xarm_msgs::srv::Call::Request> req_call_;
	rclcpp::Client<xarm_msgs::srv::Call>::SharedPtr client_clean_error_;
	rclcpp::Client<xarm_msgs::srv::Call>::SharedPtr client_clean_warn_;
	rclcpp::Client<xarm_msgs::srv::Call>::SharedPtr client_clean_conf_;
	rclcpp::Client<xarm_msgs::srv::Call>::SharedPtr client_save_conf_;
	rclcpp::Client<xarm_msgs::srv::Call>::SharedPtr client_reload_dynamics_;
	rclcpp::Client<xarm_msgs::srv::Call>::SharedPtr client_set_counter_reset_;
	rclcpp::Client<xarm_msgs::srv::Call>::SharedPtr client_set_counter_increase_;
	rclcpp::Client<xarm_msgs::srv::Call>::SharedPtr client_clean_gripper_error_;
	rclcpp::Client<xarm_msgs::srv::Call>::SharedPtr client_clean_bio_gripper_error_;
	rclcpp::Client<xarm_msgs::srv::Call>::SharedPtr client_start_record_trajectory_;
	rclcpp::Client<xarm_msgs::srv::Call>::SharedPtr client_stop_record_trajectory_;

	// GetInt16
	std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req_get_int16_;
	std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res_get_int16_;
	rclcpp::Client<xarm_msgs::srv::GetInt16>::SharedPtr client_get_state_;
	rclcpp::Client<xarm_msgs::srv::GetInt16>::SharedPtr client_get_cmdnum_;
	rclcpp::Client<xarm_msgs::srv::GetInt16>::SharedPtr client_get_vacuum_gripper_;
	rclcpp::Client<xarm_msgs::srv::GetInt16>::SharedPtr client_get_gripper_err_code_;
	rclcpp::Client<xarm_msgs::srv::GetInt16>::SharedPtr client_get_bio_gripper_status_;
	rclcpp::Client<xarm_msgs::srv::GetInt16>::SharedPtr client_get_bio_gripper_error_;
	
	// GetErrWarn
	std::shared_ptr<xarm_msgs::srv::GetErrWarn::Request> req_get_err_warn_;
	std::shared_ptr<xarm_msgs::srv::GetErrWarn::Response> res_get_err_warn_;
	rclcpp::Client<xarm_msgs::srv::GetErrWarn>::SharedPtr client_get_err_warn_code_;
	
	// SetInt16
	std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req_set_int16_;
	rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr client_set_mode_;
	rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr client_set_state_;
	rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr client_set_collision_sensitivity_;
	rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr client_set_teach_sensitivity_;
	rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr client_set_gripper_mode_;
	rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr client_set_gripper_enable_;
	rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr client_set_tgpio_modbus_timeout_;
	rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr client_set_bio_gripper_speed_;
	
	// SetInt16ById
	std::shared_ptr<xarm_msgs::srv::SetInt16ById::Request> req_set_int16_by_id_;
	rclcpp::Client<xarm_msgs::srv::SetInt16ById>::SharedPtr client_motion_enable_;
	rclcpp::Client<xarm_msgs::srv::SetInt16ById>::SharedPtr client_set_servo_attach_;
	rclcpp::Client<xarm_msgs::srv::SetInt16ById>::SharedPtr client_set_servo_detach_;
	
	// GetInt32
	std::shared_ptr<xarm_msgs::srv::GetInt32::Request> req_get_int32_;
	std::shared_ptr<xarm_msgs::srv::GetInt32::Response> res_get_int32_;
	rclcpp::Client<xarm_msgs::srv::GetInt32>::SharedPtr client_get_tgpio_modbus_baudrate_;
	
	// SetInt32
	std::shared_ptr<xarm_msgs::srv::SetInt32::Request> req_set_int32_;
	rclcpp::Client<xarm_msgs::srv::SetInt32>::SharedPtr client_set_tgpio_modbus_baudrate_;

	// GetFloat32
	std::shared_ptr<xarm_msgs::srv::GetFloat32::Request> req_get_float32_;
	std::shared_ptr<xarm_msgs::srv::GetFloat32::Response> res_get_float32_;
	rclcpp::Client<xarm_msgs::srv::GetFloat32>::SharedPtr client_get_gripper_position_;

	// GetFloat32List
	std::shared_ptr<xarm_msgs::srv::GetFloat32List::Request> req_get_float32_list_;
	std::shared_ptr<xarm_msgs::srv::GetFloat32List::Response> res_get_float32_list_;
	rclcpp::Client<xarm_msgs::srv::GetFloat32List>::SharedPtr client_get_position_;
	rclcpp::Client<xarm_msgs::srv::GetFloat32List>::SharedPtr client_get_servo_angle_;
	rclcpp::Client<xarm_msgs::srv::GetFloat32List>::SharedPtr client_get_position_aa_;

	// SetFloat32
	std::shared_ptr<xarm_msgs::srv::SetFloat32::Request> req_set_float32_;
	rclcpp::Client<xarm_msgs::srv::SetFloat32>::SharedPtr client_set_pause_time_;
	rclcpp::Client<xarm_msgs::srv::SetFloat32>::SharedPtr client_set_tcp_jerk_;
	rclcpp::Client<xarm_msgs::srv::SetFloat32>::SharedPtr client_set_tcp_maxacc_;
	rclcpp::Client<xarm_msgs::srv::SetFloat32>::SharedPtr client_set_joint_jerk_;
	rclcpp::Client<xarm_msgs::srv::SetFloat32>::SharedPtr client_set_joint_maxacc_;
	rclcpp::Client<xarm_msgs::srv::SetFloat32>::SharedPtr client_set_gripper_speed_;

	// SetFloat32List
	std::shared_ptr<xarm_msgs::srv::SetFloat32List::Request> req_set_float32_list_;
	rclcpp::Client<xarm_msgs::srv::SetFloat32List>::SharedPtr client_set_gravity_direction_;
	rclcpp::Client<xarm_msgs::srv::SetFloat32List>::SharedPtr client_set_tcp_offset_;
	rclcpp::Client<xarm_msgs::srv::SetFloat32List>::SharedPtr client_set_world_offset_;

	// SetTcpLoad
	std::shared_ptr<xarm_msgs::srv::SetTcpLoad::Request> req_set_tcp_load_;
	rclcpp::Client<xarm_msgs::srv::SetTcpLoad>::SharedPtr client_set_tcp_load_;

	// MoveCartesian
	std::shared_ptr<xarm_msgs::srv::MoveCartesian::Request> req_move_cartesian_;
	rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedPtr client_set_position_;
	rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedPtr client_set_tool_position_;
	rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedPtr client_set_position_aa_;
	rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedPtr client_set_servo_cartesian_;
	rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedPtr client_set_servo_cartesian_aa_;

	// MoveJoint
	std::shared_ptr<xarm_msgs::srv::MoveJoint::Request> req_move_joint_;
	rclcpp::Client<xarm_msgs::srv::MoveJoint>::SharedPtr client_set_servo_angle_;
	rclcpp::Client<xarm_msgs::srv::MoveJoint>::SharedPtr client_set_servo_angle_j_;

	// MoveCircle
	std::shared_ptr<xarm_msgs::srv::MoveCircle::Request> req_move_circle_;
	rclcpp::Client<xarm_msgs::srv::MoveCircle>::SharedPtr client_move_circle_;

	// MoveHome
	std::shared_ptr<xarm_msgs::srv::MoveHome::Request> req_move_home_;
	rclcpp::Client<xarm_msgs::srv::MoveHome>::SharedPtr client_move_gohome_;

	// MoveVelocity
	std::shared_ptr<xarm_msgs::srv::MoveVelocity::Request> req_move_velocity_;
	rclcpp::Client<xarm_msgs::srv::MoveVelocity>::SharedPtr client_vc_set_joint_velocity_;
	rclcpp::Client<xarm_msgs::srv::MoveVelocity>::SharedPtr client_vc_set_cartesian_velocity_;

	// GetDigitalIO
	std::shared_ptr<xarm_msgs::srv::GetDigitalIO::Request> req_get_digital_io_;
	std::shared_ptr<xarm_msgs::srv::GetDigitalIO::Response> res_get_digital_io_;
	rclcpp::Client<xarm_msgs::srv::GetDigitalIO>::SharedPtr client_get_tgpio_digital_;
	rclcpp::Client<xarm_msgs::srv::GetDigitalIO>::SharedPtr client_get_cgpio_digital_;

	// GetAnalogIO
	std::shared_ptr<xarm_msgs::srv::GetAnalogIO::Request> req_get_analog_io_;
	std::shared_ptr<xarm_msgs::srv::GetAnalogIO::Response> res_get_analog_io_;
	rclcpp::Client<xarm_msgs::srv::GetAnalogIO>::SharedPtr client_get_tgpio_analog_;
	rclcpp::Client<xarm_msgs::srv::GetAnalogIO>::SharedPtr client_get_cgpio_analog_;

	// SetDigitalIO
	std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Request> req_set_digital_io_;
	rclcpp::Client<xarm_msgs::srv::SetDigitalIO>::SharedPtr client_set_tgpio_digital_;
	rclcpp::Client<xarm_msgs::srv::SetDigitalIO>::SharedPtr client_set_cgpio_digital_;
	rclcpp::Client<xarm_msgs::srv::SetDigitalIO>::SharedPtr client_set_tgpio_digital_with_xyz_;
	rclcpp::Client<xarm_msgs::srv::SetDigitalIO>::SharedPtr client_set_cgpio_digital_with_xyz_;

	// SetAnalogIO
	std::shared_ptr<xarm_msgs::srv::SetAnalogIO::Request> req_set_analog_io_;
	rclcpp::Client<xarm_msgs::srv::SetAnalogIO>::SharedPtr client_set_cgpio_analog_;
	rclcpp::Client<xarm_msgs::srv::SetAnalogIO>::SharedPtr client_set_cgpio_analog_with_xyz_;

	// VacuumGripperCtrl
	std::shared_ptr<xarm_msgs::srv::VacuumGripperCtrl::Request> req_vacuum_gripper_ctrl_;
	rclcpp::Client<xarm_msgs::srv::VacuumGripperCtrl>::SharedPtr client_set_vacuum_gripper_;

	// GripperMove
	std::shared_ptr<xarm_msgs::srv::GripperMove::Request> req_gripper_move_;
	rclcpp::Client<xarm_msgs::srv::GripperMove>::SharedPtr client_set_gripper_position_;

	// BioGripperEnable
	std::shared_ptr<xarm_msgs::srv::BioGripperEnable::Request> req_bio_gripper_enable_;
	rclcpp::Client<xarm_msgs::srv::BioGripperEnable>::SharedPtr client_set_bio_gripper_enable_;

	// BioGripperCtrl
	std::shared_ptr<xarm_msgs::srv::BioGripperCtrl::Request> req_bio_gripper_ctrl_;
	rclcpp::Client<xarm_msgs::srv::BioGripperCtrl>::SharedPtr client_open_bio_gripper_;
	rclcpp::Client<xarm_msgs::srv::BioGripperCtrl>::SharedPtr client_close_bio_gripper_;

	// RobotiqReset
	std::shared_ptr<xarm_msgs::srv::RobotiqReset::Request> req_robotiq_reset_;
	std::shared_ptr<xarm_msgs::srv::RobotiqReset::Response> res_robotiq_reset_;
	rclcpp::Client<xarm_msgs::srv::RobotiqReset>::SharedPtr client_robotiq_reset_;

	// RobotiqActivate
	std::shared_ptr<xarm_msgs::srv::RobotiqActivate::Request> req_robotiq_activate_;
	std::shared_ptr<xarm_msgs::srv::RobotiqActivate::Response> res_robotiq_activate_;
	rclcpp::Client<xarm_msgs::srv::RobotiqActivate>::SharedPtr client_robotiq_set_activate_;

	// RobotiqMove
	std::shared_ptr<xarm_msgs::srv::RobotiqMove::Request> req_robotiq_move_;
	std::shared_ptr<xarm_msgs::srv::RobotiqMove::Response> res_robotiq_move_;
	rclcpp::Client<xarm_msgs::srv::RobotiqMove>::SharedPtr client_robotiq_set_position_;

	// RobotiqGetStatus
	std::shared_ptr<xarm_msgs::srv::RobotiqGetStatus::Request> req_robotiq_get_status_;
	std::shared_ptr<xarm_msgs::srv::RobotiqGetStatus::Response> res_robotiq_get_status_;
	rclcpp::Client<xarm_msgs::srv::RobotiqGetStatus>::SharedPtr client_robotiq_get_status_;

	// GetSetModbusData
	std::shared_ptr<xarm_msgs::srv::GetSetModbusData::Request> req_getset_modbus_data_;
	std::shared_ptr<xarm_msgs::srv::GetSetModbusData::Response> res_getset_modbus_data_;
	rclcpp::Client<xarm_msgs::srv::GetSetModbusData>::SharedPtr client_getset_tgpio_modbus_data_;

	// TrajCtrl
	std::shared_ptr<xarm_msgs::srv::TrajCtrl::Request> req_traj_ctrl_;
	rclcpp::Client<xarm_msgs::srv::TrajCtrl>::SharedPtr client_save_record_trajectory_;
	rclcpp::Client<xarm_msgs::srv::TrajCtrl>::SharedPtr client_load_trajectory_;

	// TrajPlay
	std::shared_ptr<xarm_msgs::srv::TrajPlay::Request> req_traj_play_;
	rclcpp::Client<xarm_msgs::srv::TrajPlay>::SharedPtr client_playback_trajectory_;
};

} 


#endif // __XARM_ROS_CLIENT_H__