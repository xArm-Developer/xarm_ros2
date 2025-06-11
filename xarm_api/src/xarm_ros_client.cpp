/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
           Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#include "xarm_api/xarm_ros_client.h"

#define SERVICE_CALL_FAILED 999

namespace xarm_api{

XArmROSClient::XArmROSClient(){}
XArmROSClient::~XArmROSClient(){}

void XArmROSClient::init(rclcpp::Node::SharedPtr& node, std::string hw_ns)
{   
    node_ = node;
    hw_ns_ = hw_ns;
    RCLCPP_INFO(node_->get_logger(), "namespace: %s", node->get_namespace());

    req_call_ = std::make_shared<xarm_msgs::srv::Call::Request>();
    req_get_int16_ = std::make_shared<xarm_msgs::srv::GetInt16::Request>();
    res_get_int16_ = std::make_shared<xarm_msgs::srv::GetInt16::Response>();
    req_get_int16_list_ = std::make_shared<xarm_msgs::srv::GetInt16List::Request>();
    res_get_int16_list_ = std::make_shared<xarm_msgs::srv::GetInt16List::Response>();
    req_set_int16_ = std::make_shared<xarm_msgs::srv::SetInt16::Request>();
	req_set_int16_by_id_ = std::make_shared<xarm_msgs::srv::SetInt16ById::Request>();
	req_set_int16_list_ = std::make_shared<xarm_msgs::srv::SetInt16List::Request>();
	req_get_int32_ = std::make_shared<xarm_msgs::srv::GetInt32::Request>();
	res_get_int32_ = std::make_shared<xarm_msgs::srv::GetInt32::Response>();
    req_get_int32_by_type_ = std::make_shared<xarm_msgs::srv::GetInt32ByType::Request>();
	res_get_int32_by_type_ = std::make_shared<xarm_msgs::srv::GetInt32ByType::Response>();
	req_set_int32_ = std::make_shared<xarm_msgs::srv::SetInt32::Request>();
	req_set_int32_by_type_ = std::make_shared<xarm_msgs::srv::SetInt32ByType::Request>();
	req_get_float32_ = std::make_shared<xarm_msgs::srv::GetFloat32::Request>();
	res_get_float32_ = std::make_shared<xarm_msgs::srv::GetFloat32::Response>();
	req_get_float32_list_ = std::make_shared<xarm_msgs::srv::GetFloat32List::Request>();
	res_get_float32_list_ = std::make_shared<xarm_msgs::srv::GetFloat32List::Response>();
	req_set_float32_ = std::make_shared<xarm_msgs::srv::SetFloat32::Request>();
	req_set_float32_list_ = std::make_shared<xarm_msgs::srv::SetFloat32List::Request>();
	req_set_tcp_load_ = std::make_shared<xarm_msgs::srv::SetTcpLoad::Request>();
	req_move_cartesian_ = std::make_shared<xarm_msgs::srv::MoveCartesian::Request>();
	req_move_joint_ = std::make_shared<xarm_msgs::srv::MoveJoint::Request>();
	req_move_circle_ = std::make_shared<xarm_msgs::srv::MoveCircle::Request>();
	req_move_home_ = std::make_shared<xarm_msgs::srv::MoveHome::Request>();
	req_move_velocity_ = std::make_shared<xarm_msgs::srv::MoveVelocity::Request>();
	req_get_digital_io_ = std::make_shared<xarm_msgs::srv::GetDigitalIO::Request>();
	res_get_digital_io_ = std::make_shared<xarm_msgs::srv::GetDigitalIO::Response>();
	req_get_analog_io_ = std::make_shared<xarm_msgs::srv::GetAnalogIO::Request>();
	res_get_analog_io_ = std::make_shared<xarm_msgs::srv::GetAnalogIO::Response>();
	req_set_digital_io_ = std::make_shared<xarm_msgs::srv::SetDigitalIO::Request>();
	req_set_analog_io_ = std::make_shared<xarm_msgs::srv::SetAnalogIO::Request>();
	req_vacuum_gripper_ctrl_ = std::make_shared<xarm_msgs::srv::VacuumGripperCtrl::Request>();
	req_gripper_move_ = std::make_shared<xarm_msgs::srv::GripperMove::Request>();
	req_bio_gripper_enable_ = std::make_shared<xarm_msgs::srv::BioGripperEnable::Request>();
	req_bio_gripper_ctrl_ = std::make_shared<xarm_msgs::srv::BioGripperCtrl::Request>();
	req_robotiq_reset_ = std::make_shared<xarm_msgs::srv::RobotiqReset::Request>();
	res_robotiq_reset_ = std::make_shared<xarm_msgs::srv::RobotiqReset::Response>();
	req_robotiq_activate_ = std::make_shared<xarm_msgs::srv::RobotiqActivate::Request>();
	res_robotiq_activate_ = std::make_shared<xarm_msgs::srv::RobotiqActivate::Response>();
	req_robotiq_move_ = std::make_shared<xarm_msgs::srv::RobotiqMove::Request>();
	res_robotiq_move_ = std::make_shared<xarm_msgs::srv::RobotiqMove::Response>();
	req_robotiq_get_status_ = std::make_shared<xarm_msgs::srv::RobotiqGetStatus::Request>();
	res_robotiq_get_status_ = std::make_shared<xarm_msgs::srv::RobotiqGetStatus::Response>();
	req_getset_modbus_data_ = std::make_shared<xarm_msgs::srv::GetSetModbusData::Request>();
	res_getset_modbus_data_ = std::make_shared<xarm_msgs::srv::GetSetModbusData::Response>();
	req_traj_ctrl_ = std::make_shared<xarm_msgs::srv::TrajCtrl::Request>();
	req_traj_play_ = std::make_shared<xarm_msgs::srv::TrajPlay::Request>();

    client_clean_error_ = _create_client<xarm_msgs::srv::Call>("clean_error");
    client_clean_warn_ = _create_client<xarm_msgs::srv::Call>("clean_warn");
    client_clean_conf_ = _create_client<xarm_msgs::srv::Call>("clean_conf");
    client_save_conf_ = _create_client<xarm_msgs::srv::Call>("save_conf");
    client_reload_dynamics_ = _create_client<xarm_msgs::srv::Call>("reload_dynamics");
    client_set_counter_reset_ = _create_client<xarm_msgs::srv::Call>("set_counter_reset");
    client_set_counter_increase_ = _create_client<xarm_msgs::srv::Call>("set_counter_increase");
    client_clean_gripper_error_ = _create_client<xarm_msgs::srv::Call>("clean_gripper_error");
    client_clean_bio_gripper_error_ = _create_client<xarm_msgs::srv::Call>("clean_bio_gripper_error");
    client_start_record_trajectory_ = _create_client<xarm_msgs::srv::Call>("start_record_trajectory");
    client_stop_record_trajectory_ = _create_client<xarm_msgs::srv::Call>("stop_record_trajectory");

    client_get_state_ = _create_client<xarm_msgs::srv::GetInt16>("get_state");
    client_get_cmdnum_ = _create_client<xarm_msgs::srv::GetInt16>("get_cmdnum");
    client_get_vacuum_gripper_ = _create_client<xarm_msgs::srv::GetInt16>("get_vacuum_gripper");
    client_get_gripper_err_code_ = _create_client<xarm_msgs::srv::GetInt16>("get_gripper_err_code");
    client_get_bio_gripper_status_ = _create_client<xarm_msgs::srv::GetInt16>("get_bio_gripper_status");
    client_get_bio_gripper_error_ = _create_client<xarm_msgs::srv::GetInt16>("get_bio_gripper_error");
    
    client_get_err_warn_code_ = _create_client<xarm_msgs::srv::GetInt16List>("get_err_warn_code");
    
    client_set_mode_ = _create_client<xarm_msgs::srv::SetInt16>("set_mode");
    client_set_state_ = _create_client<xarm_msgs::srv::SetInt16>("set_state");
    client_set_collision_sensitivity_ = _create_client<xarm_msgs::srv::SetInt16>("set_collision_sensitivity");
    client_set_teach_sensitivity_ = _create_client<xarm_msgs::srv::SetInt16>("set_teach_sensitivity");
    client_set_gripper_mode_ = _create_client<xarm_msgs::srv::SetInt16>("set_gripper_mode");
    client_set_gripper_enable_ = _create_client<xarm_msgs::srv::SetInt16>("set_gripper_enable");
    client_set_tgpio_modbus_timeout_ = _create_client<xarm_msgs::srv::SetInt16>("set_tgpio_modbus_timeout");
    client_set_bio_gripper_speed_ = _create_client<xarm_msgs::srv::SetInt16>("set_bio_gripper_speed");
    client_set_collision_rebound_ = _create_client<xarm_msgs::srv::SetInt16>("set_collision_rebound");
    client_set_fence_mode_ = _create_client<xarm_msgs::srv::SetInt16>("set_fence_mode");
    client_set_reduced_mode_ = _create_client<xarm_msgs::srv::SetInt16>("set_reduced_mode");
    client_set_self_collision_detection_ = _create_client<xarm_msgs::srv::SetInt16>("set_self_collision_detection");
    client_set_simulation_robot_ = _create_client<xarm_msgs::srv::SetInt16>("set_simulation_robot");
    client_set_baud_checkset_enable_ = _create_client<xarm_msgs::srv::SetInt16>("set_baud_checkset_enable");

    client_motion_enable_ = _create_client<xarm_msgs::srv::SetInt16ById>("motion_enable");
    client_set_servo_attach_ = _create_client<xarm_msgs::srv::SetInt16ById>("set_servo_attach");
    client_set_servo_detach_ = _create_client<xarm_msgs::srv::SetInt16ById>("set_servo_detach");
    
    client_set_reduced_tcp_boundary_ = _create_client<xarm_msgs::srv::SetInt16List>("set_reduced_tcp_boundary");
    
    client_get_tgpio_modbus_baudrate_ = _create_client<xarm_msgs::srv::GetInt32>("get_tgpio_modbus_baudrate");
    
    client_get_checkset_default_baud_ = _create_client<xarm_msgs::srv::GetInt32ByType>("get_checkset_default_baud");

    client_set_tgpio_modbus_baudrate_ = _create_client<xarm_msgs::srv::SetInt32>("set_tgpio_modbus_baudrate");

    client_set_checkset_default_baud_ = _create_client<xarm_msgs::srv::SetInt32ByType>("set_checkset_default_baud");
    
    client_get_gripper_position_ = _create_client<xarm_msgs::srv::GetFloat32>("get_gripper_position");
    
    client_get_position_ = _create_client<xarm_msgs::srv::GetFloat32List>("get_position");
    client_get_servo_angle_ = _create_client<xarm_msgs::srv::GetFloat32List>("get_servo_angle");
    client_get_position_aa_ = _create_client<xarm_msgs::srv::GetFloat32List>("get_position_aa");

    client_set_pause_time_ = _create_client<xarm_msgs::srv::SetFloat32>("set_pause_time");
    client_set_tcp_jerk_ = _create_client<xarm_msgs::srv::SetFloat32>("set_tcp_jerk");
    client_set_tcp_maxacc_ = _create_client<xarm_msgs::srv::SetFloat32>("set_tcp_maxacc");
    client_set_joint_jerk_ = _create_client<xarm_msgs::srv::SetFloat32>("set_joint_jerk");
    client_set_joint_maxacc_ = _create_client<xarm_msgs::srv::SetFloat32>("set_joint_maxacc");
    client_set_gripper_speed_ = _create_client<xarm_msgs::srv::SetFloat32>("set_gripper_speed");
    client_set_reduced_max_tcp_speed_ = _create_client<xarm_msgs::srv::SetFloat32>("set_reduced_max_tcp_speed");
    client_set_reduced_max_joint_speed_ = _create_client<xarm_msgs::srv::SetFloat32>("set_reduced_max_joint_speed");
    
    client_set_gravity_direction_ = _create_client<xarm_msgs::srv::SetFloat32List>("set_gravity_direction");
    client_set_tcp_offset_ = _create_client<xarm_msgs::srv::SetFloat32List>("set_tcp_offset");
    client_set_world_offset_ = _create_client<xarm_msgs::srv::SetFloat32List>("set_world_offset");
    client_set_reduced_joint_range_ = _create_client<xarm_msgs::srv::SetFloat32List>("set_reduced_joint_range");

    client_set_tcp_load_ = _create_client<xarm_msgs::srv::SetTcpLoad>("set_tcp_load");

    client_set_position_ = _create_client<xarm_msgs::srv::MoveCartesian>("set_position");
    client_set_tool_position_ = _create_client<xarm_msgs::srv::MoveCartesian>("set_tool_position");
    client_set_position_aa_ = _create_client<xarm_msgs::srv::MoveCartesian>("set_position_aa");
    client_set_servo_cartesian_ = _create_client<xarm_msgs::srv::MoveCartesian>("set_servo_cartesian");
    client_set_servo_cartesian_aa_ = _create_client<xarm_msgs::srv::MoveCartesian>("set_servo_cartesian_aa");

    client_set_servo_angle_ = _create_client<xarm_msgs::srv::MoveJoint>("set_servo_angle");
    client_set_servo_angle_j_ = _create_client<xarm_msgs::srv::MoveJoint>("set_servo_angle_j");

    client_move_circle_ = _create_client<xarm_msgs::srv::MoveCircle>("move_circle");
    
    client_move_gohome_ = _create_client<xarm_msgs::srv::MoveHome>("move_gohome");
    
    client_vc_set_joint_velocity_ = _create_client<xarm_msgs::srv::MoveVelocity>("vc_set_joint_velocity");
    client_vc_set_cartesian_velocity_ = _create_client<xarm_msgs::srv::MoveVelocity>("vc_set_cartesian_velocity");
    
    client_get_tgpio_digital_ = _create_client<xarm_msgs::srv::GetDigitalIO>("get_tgpio_digital");
    client_get_cgpio_digital_ = _create_client<xarm_msgs::srv::GetDigitalIO>("get_cgpio_digital");

    client_get_tgpio_analog_ = _create_client<xarm_msgs::srv::GetAnalogIO>("get_tgpio_analog");
    client_get_cgpio_analog_ = _create_client<xarm_msgs::srv::GetAnalogIO>("get_cgpio_analog");
    
    client_set_tgpio_digital_ = _create_client<xarm_msgs::srv::SetDigitalIO>("set_tgpio_digital");
    client_set_cgpio_digital_ = _create_client<xarm_msgs::srv::SetDigitalIO>("set_cgpio_digital");
    client_set_tgpio_digital_with_xyz_ = _create_client<xarm_msgs::srv::SetDigitalIO>("set_tgpio_digital_with_xyz");
    client_set_cgpio_digital_with_xyz_ = _create_client<xarm_msgs::srv::SetDigitalIO>("set_cgpio_digital_with_xyz");

    client_set_cgpio_analog_ = _create_client<xarm_msgs::srv::SetAnalogIO>("set_cgpio_analog");
    client_set_cgpio_analog_with_xyz_ = _create_client<xarm_msgs::srv::SetAnalogIO>("set_cgpio_analog_with_xyz");

    client_set_vacuum_gripper_ = _create_client<xarm_msgs::srv::VacuumGripperCtrl>("set_vacuum_gripper");

    client_set_gripper_position_ = _create_client<xarm_msgs::srv::GripperMove>("set_gripper_position");
    
    client_set_bio_gripper_enable_ = _create_client<xarm_msgs::srv::BioGripperEnable>("set_bio_gripper_enable");
    
    client_open_bio_gripper_ = _create_client<xarm_msgs::srv::BioGripperCtrl>("open_bio_gripper");
    client_close_bio_gripper_ = _create_client<xarm_msgs::srv::BioGripperCtrl>("close_bio_gripper");
    
    client_robotiq_reset_ = _create_client<xarm_msgs::srv::RobotiqReset>("robotiq_reset");

    client_robotiq_set_activate_ = _create_client<xarm_msgs::srv::RobotiqActivate>("robotiq_set_activate");

    client_robotiq_set_position_ = _create_client<xarm_msgs::srv::RobotiqMove>("robotiq_set_position");

    client_robotiq_get_status_ = _create_client<xarm_msgs::srv::RobotiqGetStatus>("robotiq_get_status");

    client_getset_tgpio_modbus_data_ = _create_client<xarm_msgs::srv::GetSetModbusData>("getset_tgpio_modbus_data");

    client_save_record_trajectory_ = _create_client<xarm_msgs::srv::TrajCtrl>("save_record_trajectory");
    client_load_trajectory_ = _create_client<xarm_msgs::srv::TrajCtrl>("load_trajectory");
    
    client_playback_trajectory_ = _create_client<xarm_msgs::srv::TrajPlay>("playback_trajectory");

    std::thread th([this]() -> void {
        RCLCPP_INFO(node_->get_logger(), "ＳＰＩＮ () !!!!! **************************************");
        rclcpp::spin(node_);
    });
    th.detach();
}

template<typename ServiceT>
typename rclcpp::Client<ServiceT>::SharedPtr XArmROSClient::_create_client(const std::string & service_name)
{
    return node_->create_client<ServiceT>(hw_ns_ + "/" + service_name);
}

template<typename ServiceT, typename SharedRequest>
int XArmROSClient::_call_request(std::shared_ptr<ServiceT> client, SharedRequest req)
{
    bool is_try_again = false;
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            exit(1);
        }
        if (!is_try_again) {
            is_try_again = true;
            RCLCPP_WARN(node_->get_logger(), "service %s not available, waiting ...", client->get_service_name());
        }
    }
    auto result_future = client->async_send_request(req);
    // new 20240808 comment out
    // if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::FutureReturnCode::SUCCESS)
    // {
    //     RCLCPP_ERROR(node_->get_logger(), "Failed to call service %s", client->get_service_name());
    //     return SERVICE_CALL_FAILED;
    // }
    auto res = result_future.get();
    if (res->message.size() != 0)
        RCLCPP_DEBUG(node_->get_logger(), "call service %s, ret=%d, message(%s)", client->get_service_name(), res->ret, res->message.c_str());
    else
        RCLCPP_DEBUG(node_->get_logger(), "call service %s, ret=%d", client->get_service_name(), res->ret);
    return res->ret;
}

template<typename ServiceT, typename SharedRequest, typename SharedResponse>
int XArmROSClient::_call_request(std::shared_ptr<ServiceT> client, SharedRequest req, SharedResponse& res)
{
    bool is_try_again = false;
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            exit(1);
        }
        if (!is_try_again) {
            is_try_again = true;
            RCLCPP_WARN(node_->get_logger(), "service %s not available, waiting ...", client->get_service_name());
        }
    }
    auto result_future = client->async_send_request(req);
    // new 20240808 comment out
    // if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::FutureReturnCode::SUCCESS)
    // {
    //     RCLCPP_ERROR(node_->get_logger(), "Failed to call service %s", client->get_service_name());
    //     return SERVICE_CALL_FAILED;
    // }
    res = result_future.get();
    if (res->message.size() != 0)
        RCLCPP_DEBUG(node_->get_logger(), "call service %s, ret=%d, message(%s)", client->get_service_name(), res->ret, res->message.c_str());
    else
        RCLCPP_DEBUG(node_->get_logger(), "call service %s, ret=%d", client->get_service_name(), res->ret);
    return res->ret;
}

// Call
int XArmROSClient::clean_error()
{
    return _call_request(client_clean_error_, req_call_);
}

int XArmROSClient::clean_warn()
{
    return _call_request(client_clean_warn_, req_call_);
}

int XArmROSClient::clean_conf()
{
    return _call_request(client_clean_conf_, req_call_);
}
int XArmROSClient::save_conf()
{
    return _call_request(client_save_conf_, req_call_);
}

int XArmROSClient::reload_dynamics()
{
    return _call_request(client_reload_dynamics_, req_call_);
}

int XArmROSClient::set_counter_reset()
{
    return _call_request(client_set_counter_reset_, req_call_);
}

int XArmROSClient::set_counter_increase()
{
    return _call_request(client_set_counter_increase_, req_call_);
}

int XArmROSClient::clean_gripper_error()
{
    return _call_request(client_clean_gripper_error_, req_call_);
}

int XArmROSClient::clean_bio_gripper_error()
{
    return _call_request(client_clean_bio_gripper_error_, req_call_);
}

int XArmROSClient::start_record_trajectory()
{
    return _call_request(client_start_record_trajectory_, req_call_);
}

int XArmROSClient::stop_record_trajectory()
{
    return _call_request(client_stop_record_trajectory_, req_call_);
}

// GetInt16
int XArmROSClient::get_state(int *state)
{
    int ret = _call_request(client_get_state_, req_get_int16_, res_get_int16_);
    *state = res_get_int16_->data;
    return ret;
}

int XArmROSClient::get_cmdnum(int *cmdnum)
{
    int ret = _call_request(client_get_cmdnum_, req_get_int16_, res_get_int16_);
    *cmdnum = res_get_int16_->data;
    return ret;
}
int XArmROSClient::get_vacuum_gripper(int *status)
{
    int ret = _call_request(client_get_vacuum_gripper_, req_get_int16_, res_get_int16_);
    *status = res_get_int16_->data;
    return ret;
}
int XArmROSClient::get_gripper_err_code(int *err)
{
    int ret = _call_request(client_get_gripper_err_code_, req_get_int16_, res_get_int16_);
    *err = res_get_int16_->data;
    return ret;
}
int XArmROSClient::get_bio_gripper_status(int *status)
{
    int ret = _call_request(client_get_bio_gripper_status_, req_get_int16_, res_get_int16_);
    *status = res_get_int16_->data;
    return ret;
}
int XArmROSClient::get_bio_gripper_error(int *err)
{
    int ret = _call_request(client_get_bio_gripper_error_, req_get_int16_, res_get_int16_);
    *err = res_get_int16_->data;
    return ret;
}

// GetInt16List
int XArmROSClient::get_err_warn_code(std::vector<int>& err_warn)
{
    int ret = _call_request(client_get_err_warn_code_, req_get_int16_list_, res_get_int16_list_);
    err_warn.resize(2);
    // err_warn.swap(res_get_int16_list_->datas);
    err_warn.assign(res_get_int16_list_->datas.begin(), res_get_int16_list_->datas.end());
    res_get_int16_list_->datas.clear();
    return ret;
}

// SetInt16
int XArmROSClient::set_mode(int mode)
{
    req_set_int16_->data = mode;
    return _call_request(client_set_mode_, req_set_int16_);
}

int XArmROSClient::set_state(int state)
{
    req_set_int16_->data = state;
    return _call_request(client_set_state_, req_set_int16_);
}

int XArmROSClient::set_collision_sensitivity(int sensitivity)
{
    req_set_int16_->data = sensitivity;
    return _call_request(client_set_collision_sensitivity_, req_set_int16_);
}

int XArmROSClient::set_teach_sensitivity(int sensitivity)
{
    req_set_int16_->data = sensitivity;
    return _call_request(client_set_teach_sensitivity_, req_set_int16_);
}

int XArmROSClient::set_gripper_mode(int mode)
{
    req_set_int16_->data = mode;
    return _call_request(client_set_gripper_mode_, req_set_int16_);
}

int XArmROSClient::set_gripper_enable(bool enable)
{
    req_set_int16_->data = (int)enable;
    return _call_request(client_set_gripper_enable_, req_set_int16_);
}

int XArmROSClient::set_tgpio_modbus_timeout(int timeout)
{
    req_set_int16_->data = timeout;
    return _call_request(client_set_tgpio_modbus_timeout_, req_set_int16_);
}

int XArmROSClient::set_bio_gripper_speed(int speed)
{
    req_set_int16_->data = speed;
    return _call_request(client_set_bio_gripper_speed_, req_set_int16_);
}

int XArmROSClient::set_collision_rebound(bool on)
{
    req_set_int16_->data = (int)on;
    return _call_request(client_set_collision_rebound_, req_set_int16_);
}

int XArmROSClient::set_fence_mode(bool on)
{
    req_set_int16_->data = (int)on;
    return _call_request(client_set_fence_mode_, req_set_int16_);
}

int XArmROSClient::set_reduced_mode(bool on)
{
    req_set_int16_->data = (int)on;
    return _call_request(client_set_reduced_mode_, req_set_int16_);
}

int XArmROSClient::set_self_collision_detection(bool on)
{
    req_set_int16_->data = (int)on;
    return _call_request(client_set_self_collision_detection_, req_set_int16_);
}

int XArmROSClient::set_simulation_robot(bool on)
{
    req_set_int16_->data = (int)on;
    return _call_request(client_set_simulation_robot_, req_set_int16_);
}

int XArmROSClient::set_baud_checkset_enable(bool enable)
{
    req_set_int16_->data = (int)enable;
    return _call_request(client_set_baud_checkset_enable_, req_set_int16_);
}

// SetInt16ById
int XArmROSClient::motion_enable(bool enable, int servo_id)
{
    req_set_int16_by_id_->id = servo_id;
    req_set_int16_by_id_->data = (int)enable;
    return _call_request(client_motion_enable_, req_set_int16_by_id_);
}

int XArmROSClient::set_servo_attach(int servo_id)
{
    req_set_int16_by_id_->id = servo_id;
    return _call_request(client_set_servo_attach_, req_set_int16_by_id_);
}

int XArmROSClient::set_servo_detach(int servo_id)
{
    req_set_int16_by_id_->id = servo_id;
    return _call_request(client_set_servo_detach_, req_set_int16_by_id_);
}

// SetInt32ByType
int XArmROSClient::set_checkset_default_baud(int type, int baud)
{
    req_set_int32_by_type_->type = type;
    req_set_int32_by_type_->data = baud;
    return _call_request(client_set_checkset_default_baud_, req_set_int32_by_type_);
}

// SetInt16List
int XArmROSClient::set_reduced_tcp_boundary(const std::vector<int>& boundary)
{
    req_set_int16_list_->datas.resize(6);
    // req_set_int16_list_->datas.swap(boundary);
    req_set_int16_list_->datas.assign(boundary.begin(), boundary.end());
    return _call_request(client_set_reduced_tcp_boundary_, req_set_int16_list_);
}

// GetInt32
int XArmROSClient::get_tgpio_modbus_baudrate(int *baudrate)
{
    int ret = _call_request(client_get_tgpio_modbus_baudrate_, req_get_int32_, res_get_int32_);
    *baudrate = res_get_int32_->data;
    return ret;
}

// GetInt32ByType
int XArmROSClient::get_checkset_default_baud(int type, int *baud)
{
    req_get_int32_by_type_->type = type;
    int ret = _call_request(client_get_checkset_default_baud_, req_get_int32_by_type_, res_get_int32_by_type_);
    *baud = res_get_int32_by_type_->data;
    return ret;
}

// SetInt32
int XArmROSClient::set_tgpio_modbus_baudrate(int baudrate)
{
    req_set_int32_->data = baudrate;
    return _call_request(client_set_tgpio_modbus_baudrate_, req_set_int32_);
}

// GetFloat32
int XArmROSClient::get_gripper_position(fp32 *pos)
{
    int ret = _call_request(client_get_gripper_position_, req_get_float32_, res_get_float32_);
    *pos = res_get_float32_->data;
    return ret;
}

// GetFloat32List
int XArmROSClient::get_position(std::vector<fp32>& pose)
{
    int ret = _call_request(client_get_position_, req_get_float32_list_, res_get_float32_list_);
    pose.resize(6);
    pose.swap(res_get_float32_list_->datas);
    // pose.assign(res_get_float32_list_->datas.begin(), res_get_float32_list_->datas.end());
    res_get_float32_list_->datas.clear();
    return ret;
}
int XArmROSClient::get_servo_angle(std::vector<fp32>& angles)
{
    int ret = _call_request(client_get_servo_angle_, req_get_float32_list_, res_get_float32_list_);
    angles.resize(7);
    angles.swap(res_get_float32_list_->datas);
    // angles.assign(res_get_float32_list_->datas.begin(), res_get_float32_list_->datas.end());
    res_get_float32_list_->datas.clear();
    return ret;
}

int XArmROSClient::get_position_aa(std::vector<fp32>& pose)
{
    int ret = _call_request(client_get_position_aa_, req_get_float32_list_, res_get_float32_list_);
    pose.resize(6);
    pose.swap(res_get_float32_list_->datas);
    // pose.assign(res_get_float32_list_->datas.begin(), res_get_float32_list_->datas.end());
    res_get_float32_list_->datas.clear();
    return ret;
}

// SetFloat32
int XArmROSClient::set_pause_time(fp32 sltime)
{
    req_set_float32_->data = sltime;
    return _call_request(client_set_pause_time_, req_set_float32_);
}

int XArmROSClient::set_tcp_jerk(fp32 jerk)
{
    req_set_float32_->data = jerk;
    return _call_request(client_set_tcp_jerk_, req_set_float32_);
}

int XArmROSClient::set_tcp_maxacc(fp32 maxacc)
{
    req_set_float32_->data = maxacc;
    return _call_request(client_set_tcp_maxacc_, req_set_float32_);
}

int XArmROSClient::set_joint_jerk(fp32 jerk)
{
    req_set_float32_->data = jerk;
    return _call_request(client_set_joint_jerk_, req_set_float32_);
}

int XArmROSClient::set_joint_maxacc(fp32 maxacc)
{
    req_set_float32_->data = maxacc;
    return _call_request(client_set_joint_maxacc_, req_set_float32_);
}

int XArmROSClient::set_gripper_speed(fp32 speed)
{
    req_set_float32_->data = speed;
    return _call_request(client_set_gripper_speed_, req_set_float32_);
}

int XArmROSClient::set_reduced_max_tcp_speed(fp32 speed)
{
    req_set_float32_->data = speed;
    return _call_request(client_set_reduced_max_tcp_speed_, req_set_float32_);
}

int XArmROSClient::set_reduced_max_joint_speed(fp32 speed)
{
    req_set_float32_->data = speed;
    return _call_request(client_set_reduced_max_joint_speed_, req_set_float32_);
}

// SetFloat32List
int XArmROSClient::set_gravity_direction(const std::vector<fp32>& gravity_dir)
{
    req_set_float32_list_->datas = gravity_dir;
    return _call_request(client_set_gravity_direction_, req_set_float32_list_);
}

int XArmROSClient::set_tcp_load(fp32 weight, const std::vector<fp32>& center_of_gravity)
{
    req_set_tcp_load_->weight = weight;
    req_set_tcp_load_->center_of_gravity = center_of_gravity;
    return _call_request(client_set_tcp_load_, req_set_tcp_load_);
}

int XArmROSClient::set_tcp_offset(const std::vector<fp32>& offset)
{
    req_set_float32_list_->datas = offset;
    return _call_request(client_set_tcp_offset_, req_set_float32_list_);
}

int XArmROSClient::set_world_offset(const std::vector<fp32>& offset)
{
    req_set_float32_list_->datas = offset;
    return _call_request(client_set_world_offset_, req_set_float32_list_);
}

int XArmROSClient::set_reduced_joint_range(const std::vector<fp32>& jrange)
{
    req_set_float32_list_->datas = jrange;
    return _call_request(client_set_reduced_joint_range_, req_set_float32_list_);
}

// MoveCartesian
int XArmROSClient::set_position(const std::vector<fp32>& pose, fp32 radius, fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout)
{
    req_move_cartesian_->pose = pose;
    req_move_cartesian_->radius = radius;
    req_move_cartesian_->speed = speed;
    req_move_cartesian_->acc = acc;
    req_move_cartesian_->mvtime = mvtime;
    req_move_cartesian_->wait = wait;
    req_move_cartesian_->timeout = timeout;
    return _call_request(client_set_position_, req_move_cartesian_);
}

int XArmROSClient::set_position(const std::vector<fp32>& pose, fp32 radius, bool wait, fp32 timeout)
{
    return set_position(pose, radius, 0, 0, 0, wait, timeout);
}

int XArmROSClient::set_position(const std::vector<fp32>& pose, bool wait, fp32 timeout)
{
    return set_position(pose, -1, 0, 0, 0, wait, timeout);
}

int XArmROSClient::set_tool_position(const std::vector<fp32>& pose, fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout)
{
    req_move_cartesian_->pose = pose;
    req_move_cartesian_->speed = speed;
    req_move_cartesian_->acc = acc;
    req_move_cartesian_->mvtime = mvtime;
    req_move_cartesian_->wait = wait;
    req_move_cartesian_->timeout = timeout;
    return _call_request(client_set_tool_position_, req_move_cartesian_);
}

int XArmROSClient::set_tool_position(const std::vector<fp32>& pose, bool wait, fp32 timeout)
{
    return set_tool_position(pose, 0, 0, 0, wait, timeout);
}

int XArmROSClient::set_position_aa(const std::vector<fp32>& pose, fp32 speed, fp32 acc, fp32 mvtime, bool is_tool_coord, bool relative, bool wait, fp32 timeout)
{
    req_move_cartesian_->pose = pose;
    req_move_cartesian_->speed = speed;
    req_move_cartesian_->acc = acc;
    req_move_cartesian_->mvtime = mvtime;
    req_move_cartesian_->is_tool_coord = is_tool_coord;
    req_move_cartesian_->relative = relative;
    req_move_cartesian_->wait = wait;
    req_move_cartesian_->timeout = timeout;
    return _call_request(client_set_position_aa_, req_move_cartesian_);
}

int XArmROSClient::set_position_aa(const std::vector<fp32>& pose, bool is_tool_coord, bool relative, bool wait, fp32 timeout)
{
    return set_position_aa(pose, 0, 0, 0, is_tool_coord, relative, wait, timeout);
}

int XArmROSClient::set_servo_cartesian(const std::vector<fp32>& pose, fp32 speed, fp32 acc, fp32 mvtime, bool is_tool_coord)
{
    req_move_cartesian_->pose = pose;
    req_move_cartesian_->speed = speed;
    req_move_cartesian_->acc = acc;
    req_move_cartesian_->mvtime = mvtime;
    req_move_cartesian_->is_tool_coord = is_tool_coord;
    return _call_request(client_set_servo_cartesian_, req_move_cartesian_);

}

int XArmROSClient::set_servo_cartesian_aa(const std::vector<fp32>& pose, fp32 speed, fp32 acc, bool is_tool_coord, bool relative)
{
    req_move_cartesian_->pose = pose;
    req_move_cartesian_->speed = speed;
    req_move_cartesian_->acc = acc;
    req_move_cartesian_->is_tool_coord = is_tool_coord;
    req_move_cartesian_->relative = relative;
    return _call_request(client_set_servo_cartesian_aa_, req_move_cartesian_);
}

int XArmROSClient::set_servo_cartesian_aa(const std::vector<fp32>& pose, bool is_tool_coord, bool relative)
{
    return set_servo_cartesian(pose, 0, 0, is_tool_coord, relative);
}

// MoveJoint
int XArmROSClient::set_servo_angle(const std::vector<fp32>& angles, fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout, fp32 radius)
{
    req_move_joint_->angles = angles;
    req_move_joint_->speed = speed;
    req_move_joint_->acc = acc;
    req_move_joint_->mvtime = mvtime;
    req_move_joint_->wait = wait;
    req_move_joint_->timeout = timeout;
    req_move_joint_->radius = radius;
    return _call_request(client_set_servo_angle_, req_move_joint_);
}

int XArmROSClient::set_servo_angle(const std::vector<fp32>& angles, bool wait, fp32 timeout, fp32 radius)
{
    return set_servo_angle(angles, 0, 0, 0, wait, timeout, radius);
}

int XArmROSClient::set_servo_angle_j(const std::vector<fp32>& angles, fp32 speed, fp32 acc, fp32 mvtime)
{
    req_move_joint_->angles = angles;
    req_move_joint_->speed = speed;
    req_move_joint_->acc = acc;
    req_move_joint_->mvtime = mvtime;
    return _call_request(client_set_servo_angle_j_, req_move_joint_);
}

// MoveCircle
int XArmROSClient::move_circle(const std::vector<fp32>& pose1, const std::vector<fp32>& pose2, fp32 percent, fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout)
{
    req_move_circle_->pose1 = pose1;
    req_move_circle_->pose2 = pose2;
    req_move_circle_->percent = percent;
    req_move_circle_->speed = speed;
    req_move_circle_->acc = acc;
    req_move_circle_->mvtime = mvtime;
    req_move_circle_->wait = wait;
    req_move_circle_->timeout = timeout;
    return _call_request(client_move_circle_, req_move_circle_);
}

// MoveHome
int XArmROSClient::move_gohome(fp32 speed, fp32 acc, fp32 mvtime, bool wait, fp32 timeout)
{
    req_move_home_->speed = speed;
    req_move_home_->acc = acc;
    req_move_home_->mvtime = mvtime;
    req_move_home_->wait = wait;
    req_move_home_->timeout = timeout;
    return _call_request(client_move_gohome_, req_move_home_);
}
int XArmROSClient::move_gohome(bool wait, fp32 timeout)
{
    return move_gohome(0, 0, 0, wait, timeout);
}

// MoveVelocity
int XArmROSClient::vc_set_joint_velocity(const std::vector<fp32>& speeds, bool is_sync, float duration)
{
    req_move_velocity_->speeds = speeds;
    req_move_velocity_->is_sync = is_sync;
    req_move_velocity_->duration = duration;
    return _call_request(client_vc_set_joint_velocity_, req_move_velocity_);
}

int XArmROSClient::vc_set_cartesian_velocity(const std::vector<fp32>& speeds, bool is_tool_coord, float duration)
{
    req_move_velocity_->speeds = speeds;
    req_move_velocity_->is_tool_coord = is_tool_coord;
    req_move_velocity_->duration = duration;
    return _call_request(client_vc_set_cartesian_velocity_, req_move_velocity_);
}

// GetDigitalIO
int XArmROSClient::get_tgpio_digital(std::vector<int>& digitals)
{
    int ret = _call_request(client_get_tgpio_digital_, req_get_digital_io_, res_get_digital_io_);
    digitals.resize(2);
    // digitals.swap(res_get_digital_io_->digitals);
    digitals.assign(res_get_digital_io_->digitals.begin(), res_get_digital_io_->digitals.end());
    res_get_digital_io_->digitals.clear();
    return ret;
}

int XArmROSClient::get_cgpio_digital(std::vector<int>& digitals)
{
    int ret = _call_request(client_get_cgpio_digital_, req_get_digital_io_, res_get_digital_io_);
    digitals.resize(16);
    // digitals.swap(res_get_digital_io_->digitals);
    digitals.assign(res_get_digital_io_->digitals.begin(), res_get_digital_io_->digitals.end());
    res_get_digital_io_->digitals.clear();
    return ret;
}
// int XArmROSClient::get_tgpio_digital(int *io0_value, int *io1_value)
// {
//     int ret = _call_request(client_get_tgpio_digital_, req_get_digital_io_, res_get_digital_io_);
//     *io0_value = res_get_digital_io_->digitals[0];
//     *io1_value = res_get_digital_io_->digitals[1];
//     return ret;
// }

// int XArmROSClient::get_cgpio_digital(int *digitals, int *digitals2)
// {
//     int ret = _call_request(client_get_cgpio_digital_, req_get_digital_io_, res_get_digital_io_);
//     *digitals = res_get_digital_io_->digitals[0];
//     for (int i = 0; i < 8; i++) {
//         digitals[i] = res_get_digital_io_->digitals[i];
//         if (digitals2 != NULL)
//             digitals2[i] = res_get_digital_io_->digitals[i + 8];
//     }
//     return ret;
// }

// GetAnalogIO
int XArmROSClient::get_tgpio_analog(int ionum, fp32 *value)
{
    req_get_analog_io_->ionum = ionum;
    int ret = _call_request(client_get_tgpio_analog_, req_get_analog_io_, res_get_analog_io_);
    *value = res_get_analog_io_->data;
    return ret;
}

int XArmROSClient::get_cgpio_analog(int ionum, fp32 *value)
{
    req_get_analog_io_->ionum = ionum;
    int ret = _call_request(client_get_cgpio_analog_, req_get_analog_io_, res_get_analog_io_);
    *value = res_get_analog_io_->data;
    return ret;
}

// SetDigitalIO
int XArmROSClient::set_tgpio_digital(int ionum, int value, fp32 delay_sec)
{
    req_set_digital_io_->ionum = ionum;
    req_set_digital_io_->value = value;
    req_set_digital_io_->delay_sec = delay_sec;
    return _call_request(client_set_tgpio_digital_, req_set_digital_io_);
}

int XArmROSClient::set_cgpio_digital(int ionum, int value, fp32 delay_sec)
{
    req_set_digital_io_->ionum = ionum;
    req_set_digital_io_->value = value;
    req_set_digital_io_->delay_sec = delay_sec;
    return _call_request(client_set_cgpio_digital_, req_set_digital_io_);
}

int XArmROSClient::set_tgpio_digital_with_xyz(int ionum, int value, const std::vector<fp32>& xyz, fp32 tol_r)
{
    req_set_digital_io_->ionum = ionum;
    req_set_digital_io_->value = value;
    req_set_digital_io_->xyz = xyz;
    req_set_digital_io_->tol_r = tol_r;
    return _call_request(client_set_tgpio_digital_with_xyz_, req_set_digital_io_);
}

int XArmROSClient::set_cgpio_digital_with_xyz(int ionum, int value, const std::vector<fp32>& xyz, fp32 tol_r)
{
    req_set_digital_io_->ionum = ionum;
    req_set_digital_io_->value = value;
    req_set_digital_io_->xyz = xyz;
    req_set_digital_io_->tol_r = tol_r;
    return _call_request(client_set_cgpio_digital_with_xyz_, req_set_digital_io_);
}

// SetAnalogIO
int XArmROSClient::set_cgpio_analog(int ionum, fp32 value)
{
    req_set_analog_io_->ionum = ionum;
    req_set_analog_io_->value = value;
    return _call_request(client_set_cgpio_analog_, req_set_analog_io_);
}

int XArmROSClient::set_cgpio_analog_with_xyz(int ionum, fp32 value, const std::vector<fp32>& xyz, fp32 tol_r)
{
    req_set_analog_io_->ionum = ionum;
    req_set_analog_io_->value = value;
    req_set_analog_io_->xyz = xyz;
    req_set_analog_io_->tol_r = tol_r;
    return _call_request(client_set_cgpio_analog_with_xyz_, req_set_analog_io_);
}

// VacuumGripperCtrl
int XArmROSClient::set_vacuum_gripper(bool on, bool wait, float timeout, float delay_sec)
{
    req_vacuum_gripper_ctrl_->on = on;
    req_vacuum_gripper_ctrl_->wait = wait;
    req_vacuum_gripper_ctrl_->timeout = timeout;
    req_vacuum_gripper_ctrl_->delay_sec = delay_sec;
    return _call_request(client_set_vacuum_gripper_, req_vacuum_gripper_ctrl_);
}

// GripperMove
int XArmROSClient::set_gripper_position(fp32 pos, bool wait, fp32 timeout)
{
    req_gripper_move_->pos = pos;
    req_gripper_move_->wait = wait;
    req_gripper_move_->timeout = timeout;
    return _call_request(client_set_gripper_position_, req_gripper_move_);
}

// BioGripperEnable
int XArmROSClient::set_bio_gripper_enable(bool enable, bool wait, fp32 timeout)
{
    req_bio_gripper_enable_->enable = enable;
    req_bio_gripper_enable_->wait = wait;
    req_bio_gripper_enable_->timeout = timeout;
    return _call_request(client_set_bio_gripper_enable_, req_bio_gripper_enable_);
}

// BioGripperCtrl
int XArmROSClient::open_bio_gripper(int speed, bool wait, fp32 timeout)
{
    req_bio_gripper_ctrl_->speed = speed;
    req_bio_gripper_ctrl_->wait = wait;
    req_bio_gripper_ctrl_->timeout = timeout;
    return _call_request(client_open_bio_gripper_, req_bio_gripper_ctrl_);
}

int XArmROSClient::open_bio_gripper(bool wait, fp32 timeout)
{
    return open_bio_gripper(0, wait, timeout);
}

int XArmROSClient::close_bio_gripper(int speed, bool wait, fp32 timeout)
{
    req_bio_gripper_ctrl_->speed = speed;
    req_bio_gripper_ctrl_->wait = wait;
    req_bio_gripper_ctrl_->timeout = timeout;
    return _call_request(client_close_bio_gripper_, req_bio_gripper_ctrl_);
}

int XArmROSClient::close_bio_gripper(bool wait, fp32 timeout)
{
    return close_bio_gripper(0, wait, timeout);
}

// RobotiqReset
int XArmROSClient::robotiq_reset()
{
    return _call_request(client_robotiq_reset_, req_robotiq_reset_, res_robotiq_reset_);
}
// int XArmROSClient::robotiq_reset(std::vector<unsigned char>& ret_data)
// {
//     int ret = _call_request(client_robotiq_reset_, req_robotiq_reset_, res_robotiq_reset_);
//     ret_data.resize(6);
//     ret_data.assign(res_robotiq_reset_->ret_data.begin(), res_robotiq_reset_->ret_data.end());
//     return ret;
// }

// RobotiqActivate
int XArmROSClient::robotiq_set_activate(bool wait, fp32 timeout)
{
    req_robotiq_activate_->wait = wait;
    req_robotiq_activate_->timeout = timeout;
    return  _call_request(client_robotiq_set_activate_, req_robotiq_activate_, res_robotiq_activate_);
}
// int XArmROSClient::robotiq_set_activate(bool wait, fp32 timeout, unsigned char ret_data[6])
// {
//     req_robotiq_activate_->wait = wait;
//     req_robotiq_activate_->timeout = timeout;
//     int ret = _call_request(client_robotiq_set_activate_, req_robotiq_activate_, res_robotiq_activate_);
//     if (ret_data != NULL) {
//         for (int i = 0; i < 6; i++) {
//             ret_data[i] = res_robotiq_activate_->ret_data[i];
//         }
//     }
//     return ret;
// }

// int XArmROSClient::robotiq_set_activate(bool wait, unsigned char ret_data[6])
// {
//     return robotiq_set_activate(wait, 3, ret_data);
// }

// int XArmROSClient::robotiq_set_activate(unsigned char ret_data[6])
// {
//     return robotiq_set_activate(true, 3, ret_data);
// }

// RobotiqMove
int XArmROSClient::robotiq_set_position(unsigned char pos, unsigned char speed, unsigned char force, bool wait, fp32 timeout)
{
    req_robotiq_move_->pos = pos;
    req_robotiq_move_->speed = speed;
    req_robotiq_move_->force = force;
    req_robotiq_move_->wait = wait;
    req_robotiq_move_->timeout = timeout;
    return _call_request(client_robotiq_set_position_, req_robotiq_move_, res_robotiq_move_);
}

int XArmROSClient::robotiq_set_position(unsigned char pos, bool wait, fp32 timeout)
{
    return robotiq_set_position(pos, 0xFF, 0xFF, wait, timeout);
}

int XArmROSClient::robotiq_open(unsigned char speed, unsigned char force, bool wait, fp32 timeout)
{
    return robotiq_set_position(0x00, speed, force, wait, timeout);
}

int XArmROSClient::robotiq_open(bool wait, fp32 timeout)
{
    return robotiq_open(0xFF, 0xFF, wait, timeout);
}

int XArmROSClient::robotiq_close(unsigned char speed, unsigned char force, bool wait, fp32 timeout)
{
    return robotiq_set_position(0xFF, speed, force, wait, timeout);
}

int XArmROSClient::robotiq_close(bool wait, fp32 timeout)
{
    return robotiq_close(0xFF, 0xFF, wait, timeout);
}
// int XArmROSClient::robotiq_set_position(unsigned char pos, unsigned char speed, unsigned char force, bool wait, fp32 timeout, unsigned char ret_data[6])
// {
//     req_robotiq_move_->pos = pos;
//     req_robotiq_move_->speed = speed;
//     req_robotiq_move_->force = force;
//     req_robotiq_move_->wait = wait;
//     req_robotiq_move_->timeout = timeout;
//     int ret = _call_request(client_robotiq_set_position_, req_robotiq_move_, res_robotiq_move_);
//     if (ret_data != NULL) {
//         for (int i = 0; i < 6; i++) {
//             ret_data[i] = res_robotiq_move_->ret_data[i];
//         }
//     }
//     return ret;
// }

// int XArmROSClient::robotiq_set_position(unsigned char pos, bool wait, fp32 timeout, unsigned char ret_data[6])
// {
//     return robotiq_set_position(pos, 0xFF, 0xFF, wait, timeout, ret_data);
// }

// int XArmROSClient::robotiq_set_position(unsigned char pos, bool wait, unsigned char ret_data[6])
// {
//     return robotiq_set_position(pos, wait, 5, ret_data);
// }

// int XArmROSClient::robotiq_set_position(unsigned char pos, unsigned char ret_data[6])
// {
//     return robotiq_set_position(pos, true, ret_data);
// }

// int XArmROSClient::robotiq_open(unsigned char speed, unsigned char force, bool wait, fp32 timeout, unsigned char ret_data[6])
// {
//     return robotiq_set_position(0x00, speed, force, wait, timeout, ret_data);
// }

// int XArmROSClient::robotiq_open(bool wait, fp32 timeout, unsigned char ret_data[6])
// {
//     return robotiq_set_position(0x00, wait, timeout, ret_data);
// }

// int XArmROSClient::robotiq_open(bool wait, unsigned char ret_data[6])
// {
//     return robotiq_open(wait, 5, ret_data);
// }

// int XArmROSClient::robotiq_open(unsigned char ret_data[6])
// {
//     return robotiq_open(true, ret_data);
// }

// int XArmROSClient::robotiq_close(unsigned char speed, unsigned char force, bool wait, fp32 timeout, unsigned char ret_data[6])
// {
//     return robotiq_set_position(0xFF, speed, force, wait, timeout, ret_data);
// }

// int XArmROSClient::robotiq_close(bool wait, fp32 timeout, unsigned char ret_data[6])
// {
//     return robotiq_set_position(0xFF, wait, timeout, ret_data);
// }

// int XArmROSClient::robotiq_close(bool wait, unsigned char ret_data[6])
// {
//     return robotiq_close(wait, 5, ret_data);
// }

// int XArmROSClient::robotiq_close(unsigned char ret_data[6])
// {
//     return robotiq_close(true, ret_data);
// }

// RobotiqGetStatus
int XArmROSClient::robotiq_get_status(std::vector<unsigned char>& ret_data, unsigned char number_of_registers)
{
    req_robotiq_get_status_->number_of_registers = number_of_registers;
    int ret = _call_request(client_robotiq_get_status_, req_robotiq_get_status_, res_robotiq_get_status_);
    ret_data.resize(9);
    ret_data.swap(res_robotiq_get_status_->ret_data);
    // ret_data.assign(res_robotiq_get_status_->ret_data.begin(), res_robotiq_get_status_->ret_data.end());
    res_robotiq_get_status_->ret_data.clear();
    return ret;
}

// GetSetModbusData
int XArmROSClient::getset_tgpio_modbus_data(const std::vector<unsigned char>& modbus_data, int modbus_length, std::vector<unsigned char>& ret_data, int ret_length)
{
    req_getset_modbus_data_->modbus_data = modbus_data;
    req_getset_modbus_data_->modbus_length = modbus_length;
    req_getset_modbus_data_->ret_length = ret_length;
    int ret = _call_request(client_getset_tgpio_modbus_data_, req_getset_modbus_data_, res_getset_modbus_data_);
    ret_data.resize(ret_length);
    ret_data.swap(res_getset_modbus_data_->ret_data);
    // ret_data.assign(res_getset_modbus_data_->ret_data.begin(), res_getset_modbus_data_->ret_data.end());
    res_getset_modbus_data_->ret_data.clear();
    return ret;
}

// TrajCtrl
int XArmROSClient::save_record_trajectory(std::string& filename, float timeout)
{
    req_traj_ctrl_->filename = filename;
    req_traj_ctrl_->timeout = timeout;
    return _call_request(client_save_record_trajectory_, req_traj_ctrl_);
}

int XArmROSClient::load_trajectory(std::string& filename, float timeout)
{
    req_traj_ctrl_->filename = filename;
    req_traj_ctrl_->timeout = timeout;
    return _call_request(client_load_trajectory_, req_traj_ctrl_);
}

// TrajPlay
int XArmROSClient::playback_trajectory(int times, bool wait, int double_speed, std::string filename)
{
    req_traj_play_->filename = filename;
    req_traj_play_->times = times;
    req_traj_play_->wait = wait;
    req_traj_play_->double_speed = double_speed;
    return _call_request(client_playback_trajectory_, req_traj_play_);
}


}