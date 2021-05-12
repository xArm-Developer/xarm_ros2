/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
           Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#include "xarm_api/xarm_ros_client.h"

#define SERVICE_CALL_FAILED 999
#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("xarm_ros_client"), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("xarm_ros_client"), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("xarm_ros_client"), __VA_ARGS__)

namespace xarm_api{

XArmROSClient::XArmROSClient(){}
XArmROSClient::~XArmROSClient(){}

void XArmROSClient::init(rclcpp::Node::SharedPtr& node, std::string hw_ns_)
{   
    node_ = node;
    ROS_INFO("namespace: %s", node->get_namespace());
    // std::string client_ns = "";
    // client_ns.assign(node->get_namespace());
    // if (client_ns.rfind("/") != client_ns.length()-1)
    //     client_ns += "/";

    // std::string hw_ns = "xarm/";
    std::string hw_ns = hw_ns_ + "/";
    // std::string hw_ns = "";

    motion_ctrl_client_ = node->create_client<xarm_msgs::srv::SetAxis>(hw_ns + "motion_ctrl");
    set_mode_client_ = node->create_client<xarm_msgs::srv::SetInt16>(hw_ns + "set_mode");
    set_state_client_ = node->create_client<xarm_msgs::srv::SetInt16>(hw_ns + "set_state");
    set_tcp_offset_client_ = node->create_client<xarm_msgs::srv::TCPOffset>(hw_ns + "set_tcp_offset");
    set_load_client_ = node->create_client<xarm_msgs::srv::SetLoad>(hw_ns + "set_load");
    go_home_client_ = node->create_client<xarm_msgs::srv::Move>(hw_ns + "go_home");
    move_joint_client_ = node->create_client<xarm_msgs::srv::Move>(hw_ns + "move_joint");
    move_jointb_client_ = node->create_client<xarm_msgs::srv::Move>(hw_ns + "move_jointb");
    move_lineb_client_ = node->create_client<xarm_msgs::srv::Move>(hw_ns + "move_lineb");
    move_line_client_ = node->create_client<xarm_msgs::srv::Move>(hw_ns + "move_line");
    move_line_tool_client_ = node->create_client<xarm_msgs::srv::Move>(hw_ns + "move_line_tool");
    move_servoj_client_ = node->create_client<xarm_msgs::srv::Move>(hw_ns + "move_servoj");
    move_servo_cart_client_ = node->create_client<xarm_msgs::srv::Move>(hw_ns + "move_servo_cart");
    clear_err_client_ = node->create_client<xarm_msgs::srv::ClearErr>(hw_ns + "clear_err");
    get_err_client_ = node->create_client<xarm_msgs::srv::GetErr>(hw_ns + "get_err");
    move_line_aa_client_ = node->create_client<xarm_msgs::srv::MoveAxisAngle>(hw_ns + "move_line_aa");
    move_servo_cart_aa_client_ = node->create_client<xarm_msgs::srv::MoveAxisAngle>(hw_ns + "move_servo_cart_aa");
    set_end_io_client_ = node->create_client<xarm_msgs::srv::SetDigitalIO>(hw_ns + "set_digital_out");
    get_digital_in_client_ = node->create_client<xarm_msgs::srv::GetDigitalIO>(hw_ns + "get_digital_in");
    get_analog_in_client_ = node->create_client<xarm_msgs::srv::GetAnalogIO>(hw_ns + "get_analog_in");
    config_modbus_client_ = node->create_client<xarm_msgs::srv::ConfigToolModbus>(hw_ns + "config_tool_modbus");
    set_modbus_client_ = node->create_client<xarm_msgs::srv::SetToolModbus>(hw_ns + "set_tool_modbus");
    gripper_config_client_ = node->create_client<xarm_msgs::srv::GripperConfig>(hw_ns + "gripper_config");
    gripper_move_client_ = node->create_client<xarm_msgs::srv::GripperMove>(hw_ns + "gripper_move");
    gripper_state_client_ = node->create_client<xarm_msgs::srv::GripperState>(hw_ns + "gripper_state");
    set_vacuum_gripper_client_ = node->create_client<xarm_msgs::srv::SetInt16>(hw_ns + "vacuum_gripper_set");
    set_controller_dout_client_ = node->create_client<xarm_msgs::srv::SetDigitalIO>(hw_ns + "set_controller_dout");
    get_controller_din_client_ = node->create_client<xarm_msgs::srv::GetControllerDigitalIO>(hw_ns + "get_controller_din");
    set_controller_aout_client_ = node->create_client<xarm_msgs::srv::SetControllerAnalogIO>(hw_ns + "set_controller_aout");
    get_controller_ain_client_ = node->create_client<xarm_msgs::srv::GetAnalogIO>(hw_ns + "get_controller_ain");
    velo_move_joint_client_ = node->create_client<xarm_msgs::srv::MoveVelo>(hw_ns + "velo_move_joint");
    velo_move_line_client_ = node->create_client<xarm_msgs::srv::MoveVelo>(hw_ns + "velo_move_line");
    set_max_jacc_client_ = node->create_client<xarm_msgs::srv::SetFloat32>(hw_ns + "set_max_acc_joint");
    set_max_lacc_client_ = node->create_client<xarm_msgs::srv::SetFloat32>(hw_ns + "set_max_acc_line");

    // while (!motion_ctrl_client_->wait_for_service(std::chrono::seconds(2))) {
    //     if (!rclcpp::ok()) {
    //         ROS_ERROR("Interrupted while waiting for the service. Exiting.");
    //         exit(1);
    //     }
    //     ROS_INFO("service not available, waiting again...");
    // }

    set_axis_req_ = std::make_shared<xarm_msgs::srv::SetAxis::Request>();
    set_int16_req_ = std::make_shared<xarm_msgs::srv::SetInt16::Request>();
    offset_req_ = std::make_shared<xarm_msgs::srv::TCPOffset::Request>();
    set_load_req_ = std::make_shared<xarm_msgs::srv::SetLoad::Request>();
    clear_err_req_ = std::make_shared<xarm_msgs::srv::ClearErr::Request>();
    get_err_req_ = std::make_shared<xarm_msgs::srv::GetErr::Request>();
    move_req_ = std::make_shared<xarm_msgs::srv::Move::Request>();
    servoj_req_ = std::make_shared<xarm_msgs::srv::Move::Request>();
    servo_cart_req_ = std::make_shared<xarm_msgs::srv::Move::Request>();
    cfg_modbus_req_ = std::make_shared<xarm_msgs::srv::ConfigToolModbus::Request>();
    set_modbus_req_ = std::make_shared<xarm_msgs::srv::SetToolModbus::Request>();
    gripper_config_req_ = std::make_shared<xarm_msgs::srv::GripperConfig::Request>();
    gripper_move_req_ = std::make_shared<xarm_msgs::srv::GripperMove::Request>();
    gripper_state_req_ = std::make_shared<xarm_msgs::srv::GripperState::Request>();
    move_velo_req_ = std::make_shared<xarm_msgs::srv::MoveVelo::Request>();
}

template<typename ServiceT, typename SharedRequest = typename ServiceT::Request::SharedPtr>
int XArmROSClient::_call_request(std::shared_ptr<ServiceT> client, SharedRequest req)
{
    bool is_try_again = false;
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            ROS_ERROR("Interrupted while waiting for the service. Exiting.");
            exit(1);
        }
        if (!is_try_again) {
            is_try_again = true;
            ROS_WARN("service %s not available, waiting ...", client->get_service_name());
        }
    }
    auto result_future = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
        ROS_ERROR("Failed to call service %s", client->get_service_name());
        return SERVICE_CALL_FAILED;
    }
    auto res = result_future.get();
    // ROS_INFO("call service %s, ret=%d, message=%s", client->get_service_name(), res->ret, res->message.c_str());
    return res->ret;
}

template<typename ServiceT, typename SharedRequest = typename ServiceT::Request::SharedPtr, typename SharedResponse = typename ServiceT::Response::SharedPtr>
int XArmROSClient::_call_request(std::shared_ptr<ServiceT> client, SharedRequest req, SharedResponse res)
{
    auto result_future = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
        ROS_ERROR("Failed to call service %s", client->get_service_name());
        return SERVICE_CALL_FAILED;
    }
    res = result_future.get();
    // ROS_INFO("%s", res->message.c_str());
    return res->ret;
}

int XArmROSClient::motionEnable(short en)
{
    set_axis_req_->id = 8;
    set_axis_req_->data = en;
    return _call_request(motion_ctrl_client_, set_axis_req_);
}

int XArmROSClient::setState(short state)
{
	set_int16_req_->data = state;
    return _call_request(set_state_client_, set_int16_req_);
}

int XArmROSClient::setMode(short mode)
{
	set_int16_req_->data = mode;
    return _call_request(set_mode_client_, set_int16_req_);
}

int XArmROSClient::clearErr()
{
    return _call_request(clear_err_client_, clear_err_req_);
}

int XArmROSClient::getErr(int *err)
{
    std::shared_ptr<xarm_msgs::srv::GetErr::Response> res = std::make_shared<xarm_msgs::srv::GetErr::Response>();
    int ret = _call_request(get_err_client_, get_err_req_, res);
    if (ret != SERVICE_CALL_FAILED) {
        *err = res->err;
    }
    return ret;
}

int XArmROSClient::setServoJ(const std::vector<float>& joint_cmd)
{
	servoj_req_->mvvelo = 0;
    servoj_req_->mvacc = 0;
    servoj_req_->mvtime = 0;
    servoj_req_->pose = joint_cmd;
    return _call_request(move_servoj_client_, servoj_req_);
}

int XArmROSClient::setServoCartisian(const std::vector<float>& cart_cmd)
{
    servo_cart_req_->mvvelo = 0;
    servo_cart_req_->mvacc = 0;
    servo_cart_req_->mvtime = 0;
    servo_cart_req_->pose = cart_cmd;
    return _call_request(move_servo_cart_client_, servo_cart_req_);
}

int XArmROSClient::setTCPOffset(const std::vector<float>& tcp_offset)
{
    if(tcp_offset.size() != 6)
    {
        ROS_ERROR("Set tcp offset service parameter should be 6-element Cartesian offset!");
        return 1;
    }
    
    offset_req_->x = tcp_offset[0];
    offset_req_->y = tcp_offset[1];
    offset_req_->z = tcp_offset[2];
    offset_req_->roll = tcp_offset[3];
    offset_req_->pitch = tcp_offset[4];
    offset_req_->yaw = tcp_offset[5];
    return _call_request(set_tcp_offset_client_, offset_req_);
}

int XArmROSClient::setLoad(float mass, const std::vector<float>& center_of_mass)
{
    set_load_req_->mass = mass;
    set_load_req_->xc = center_of_mass[0];
    set_load_req_->yc = center_of_mass[1];
    set_load_req_->zc = center_of_mass[2];
    return _call_request(set_load_client_, set_load_req_);
}

int XArmROSClient::goHome(float jnt_vel_rad, float jnt_acc_rad)
{
    move_req_->mvvelo = jnt_vel_rad;
    move_req_->mvacc = jnt_acc_rad;
    move_req_->mvtime = 0;
    return _call_request(go_home_client_, move_req_);
}

int XArmROSClient::moveJoint(const std::vector<float>& joint_cmd, float jnt_vel_rad, float jnt_acc_rad)
{
    move_req_->mvvelo = jnt_vel_rad;
    move_req_->mvacc = jnt_acc_rad;
    move_req_->mvtime = 0;
    move_req_->pose = joint_cmd;
    return _call_request(move_joint_client_, move_req_);
}

int XArmROSClient::moveLine(const std::vector<float>& cart_cmd, float cart_vel_mm, float cart_acc_mm)
{
    move_req_->mvvelo = cart_vel_mm;
    move_req_->mvacc = cart_acc_mm;
    move_req_->mvtime = 0;
    move_req_->pose = cart_cmd;
    return _call_request(move_line_client_, move_req_);
}

int XArmROSClient::moveLineB(int num_of_pnts, const std::vector<float> cart_cmds[], float cart_vel_mm, float cart_acc_mm, float radii)
{
    move_req_->mvvelo = cart_vel_mm;
    move_req_->mvacc = cart_acc_mm;
    move_req_->mvtime = 0;
    move_req_->mvradii = radii;
    int ret = 0;
    for(int i=0; i<num_of_pnts; i++)
    {
        move_req_->pose = cart_cmds[i];
        ret = _call_request(move_lineb_client_, move_req_);
        if (ret) break;
    }
    return ret;
}

int XArmROSClient::config_tool_modbus(int baud_rate, int time_out_ms)
{
    cfg_modbus_req_->baud_rate = baud_rate;
    cfg_modbus_req_->timeout_ms = time_out_ms;
    return _call_request(config_modbus_client_, cfg_modbus_req_);
}

int XArmROSClient::send_tool_modbus(unsigned char* data, int send_len, unsigned char* recv_data, int recv_len)
{
    for(int i=0; i<send_len; i++)
    {
        set_modbus_req_->send_data.push_back(data[i]);
    }
    set_modbus_req_->respond_len = recv_len;
    int ret = _call_request(set_modbus_client_, set_modbus_req_);
    set_modbus_req_->send_data.clear();
    return ret;
}

int XArmROSClient::gripperMove(float pulse)
{
    gripper_move_req_->pulse_pos = pulse;
    return _call_request(gripper_move_client_, gripper_move_req_);
}

int XArmROSClient::gripperConfig(float pulse_vel)
{
    gripper_config_req_->pulse_vel = pulse_vel;
    return _call_request(gripper_config_client_, gripper_config_req_);
}

int XArmROSClient::getGripperState(float *curr_pulse, int *curr_err)
{
    std::shared_ptr<xarm_msgs::srv::GripperState::Response> res = std::make_shared<xarm_msgs::srv::GripperState::Response>();
    int ret = _call_request(gripper_state_client_, gripper_state_req_, res);
    if (ret != SERVICE_CALL_FAILED) {
        *curr_pulse = res->curr_pos;
        *curr_err = res->err_code;
    }
    return ret;
}

int XArmROSClient::veloMoveJoint(const std::vector<float>& jnt_v, bool is_sync) 
{
    move_velo_req_->velocities = jnt_v;
    move_velo_req_->jnt_sync = is_sync ? 1 : 0;
    return _call_request(velo_move_joint_client_, move_velo_req_);
}

int XArmROSClient::veloMoveLine(const std::vector<float>& line_v, bool is_tool_coord)
{
    move_velo_req_->velocities = line_v;
    move_velo_req_->coord = is_tool_coord ? 1 : 0;
    return _call_request(velo_move_line_client_, move_velo_req_);
}

}