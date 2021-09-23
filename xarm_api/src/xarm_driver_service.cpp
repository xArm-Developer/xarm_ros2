/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/
#include "xarm_api/xarm_driver.h"

#define PARAM_ERROR 997
#define BIND_CLS_CB(func) std::bind(func, this, std::placeholders::_1, std::placeholders::_2)

namespace xarm_api
{
    template<typename ServiceT, typename CallbackT>
    typename rclcpp::Service<ServiceT>::SharedPtr XArmDriver::_create_service(const std::string & service_name, CallbackT && callback)
    {
        bool enable;
        node_->get_parameter_or("services." + service_name, enable, false);
        if (service_debug_ || enable) {
            auto service = hw_node_->create_service<ServiceT>(service_name, BIND_CLS_CB(callback));
            RCLCPP_DEBUG(node_->get_logger(), "create_service: %s", service->get_service_name());
            return service;
        }
        return NULL;
    }

    void XArmDriver::_init_service()
    {
        node_->get_parameter_or("services.debug", service_debug_, false);

        // Call
        service_clean_error_ = _create_service<xarm_msgs::srv::Call>("clean_error", &XArmDriver::_clean_error);
        service_clean_warn_ = _create_service<xarm_msgs::srv::Call>("clean_warn", &XArmDriver::_clean_warn);
        service_clean_conf_ = _create_service<xarm_msgs::srv::Call>("clean_conf", &XArmDriver::_clean_conf);
        service_save_conf_ = _create_service<xarm_msgs::srv::Call>("save_conf", &XArmDriver::_save_conf);
        service_reload_dynamics_ = _create_service<xarm_msgs::srv::Call>("reload_dynamics", &XArmDriver::_reload_dynamics);
        service_set_counter_reset_ = _create_service<xarm_msgs::srv::Call>("set_counter_reset", &XArmDriver::_set_counter_reset);
        service_set_counter_increase_ = _create_service<xarm_msgs::srv::Call>("set_counter_increase", &XArmDriver::_set_counter_increase);
        service_clean_gripper_error_ = _create_service<xarm_msgs::srv::Call>("clean_gripper_error", &XArmDriver::_clean_gripper_error);
        service_clean_bio_gripper_error_ = _create_service<xarm_msgs::srv::Call>("clean_bio_gripper_error", &XArmDriver::_clean_bio_gripper_error);
        service_start_record_trajectory_ = _create_service<xarm_msgs::srv::Call>("start_record_trajectory", &XArmDriver::_start_record_trajectory);
        service_stop_record_trajectory_ = _create_service<xarm_msgs::srv::Call>("stop_record_trajectory", &XArmDriver::_stop_record_trajectory);

        // GetInt16
        service_get_state_ = _create_service<xarm_msgs::srv::GetInt16>("get_state", &XArmDriver::_get_state);
        service_get_cmdnum_ = _create_service<xarm_msgs::srv::GetInt16>("get_cmdnum", &XArmDriver::_get_cmdnum);
        service_get_vacuum_gripper_ = _create_service<xarm_msgs::srv::GetInt16>("get_vacuum_gripper", &XArmDriver::_get_vacuum_gripper);
        service_get_gripper_err_code_ = _create_service<xarm_msgs::srv::GetInt16>("get_gripper_err_code", &XArmDriver::_get_gripper_err_code);
        service_get_bio_gripper_status_ = _create_service<xarm_msgs::srv::GetInt16>("get_bio_gripper_status", &XArmDriver::_get_bio_gripper_status);
        service_get_bio_gripper_error_ = _create_service<xarm_msgs::srv::GetInt16>("get_bio_gripper_error", &XArmDriver::_get_bio_gripper_error);
        
        // GetInt16List
        service_get_err_warn_code_ = _create_service<xarm_msgs::srv::GetInt16List>("get_err_warn_code", &XArmDriver::_get_err_warn_code);

        // SetInt16
        service_set_mode_ = _create_service<xarm_msgs::srv::SetInt16>("set_mode", &XArmDriver::_set_mode);
        service_set_state_ = _create_service<xarm_msgs::srv::SetInt16>("set_state", &XArmDriver::_set_state);
        service_set_collision_sensitivity_ = _create_service<xarm_msgs::srv::SetInt16>("set_collision_sensitivity", &XArmDriver::_set_collision_sensitivity);
        service_set_teach_sensitivity_ = _create_service<xarm_msgs::srv::SetInt16>("set_teach_sensitivity", &XArmDriver::_set_teach_sensitivity);
        service_set_gripper_mode_ = _create_service<xarm_msgs::srv::SetInt16>("set_gripper_mode", &XArmDriver::_set_gripper_mode);
        service_set_gripper_enable_ = _create_service<xarm_msgs::srv::SetInt16>("set_gripper_enable", &XArmDriver::_set_gripper_enable);
        service_set_tgpio_modbus_timeout_ = _create_service<xarm_msgs::srv::SetInt16>("set_tgpio_modbus_timeout", &XArmDriver::_set_tgpio_modbus_timeout);
        service_set_bio_gripper_speed_ = _create_service<xarm_msgs::srv::SetInt16>("set_bio_gripper_speed", &XArmDriver::_set_bio_gripper_speed);
        service_set_collision_rebound_ = _create_service<xarm_msgs::srv::SetInt16>("set_collision_rebound", &XArmDriver::_set_collision_rebound);
        service_set_fence_mode_ = _create_service<xarm_msgs::srv::SetInt16>("set_fence_mode", &XArmDriver::_set_fence_mode);
        service_set_reduced_mode_ = _create_service<xarm_msgs::srv::SetInt16>("set_reduced_mode", &XArmDriver::_set_reduced_mode);
        service_set_self_collision_detection_ = _create_service<xarm_msgs::srv::SetInt16>("set_self_collision_detection", &XArmDriver::_set_self_collision_detection);
        service_set_simulation_robot_ = _create_service<xarm_msgs::srv::SetInt16>("set_simulation_robot", &XArmDriver::_set_simulation_robot);
        
        // SetInt16ById
        service_motion_enable_= _create_service<xarm_msgs::srv::SetInt16ById>("motion_enable", &XArmDriver::_motion_enable);
        service_set_servo_attach_ = _create_service<xarm_msgs::srv::SetInt16ById>("set_servo_attach", &XArmDriver::_set_servo_attach);
        service_set_servo_detach_ = _create_service<xarm_msgs::srv::SetInt16ById>("set_servo_detach", &XArmDriver::_set_servo_detach);
        
        // SetInt16List        
        service_set_reduced_tcp_boundary_ = _create_service<xarm_msgs::srv::SetInt16List>("set_reduced_tcp_boundary", &XArmDriver::_set_reduced_tcp_boundary);

        // GetInt32
        service_get_tgpio_modbus_baudrate_ = _create_service<xarm_msgs::srv::GetInt32>("get_tgpio_modbus_baudrate", &XArmDriver::_get_tgpio_modbus_baudrate);

        // SetInt32
        service_set_tgpio_modbus_baudrate_ = _create_service<xarm_msgs::srv::SetInt32>("set_tgpio_modbus_baudrate", &XArmDriver::_set_tgpio_modbus_baudrate);
        
        // GetFloat32
        service_get_gripper_position_ = _create_service<xarm_msgs::srv::GetFloat32>("get_gripper_position", &XArmDriver::_get_gripper_position);
    
        // GetFloat32List
        service_get_position_ = _create_service<xarm_msgs::srv::GetFloat32List>("get_position", &XArmDriver::_get_position);
        service_get_servo_angle_ = _create_service<xarm_msgs::srv::GetFloat32List>("get_servo_angle", &XArmDriver::_get_servo_angle);
        service_get_position_aa_ = _create_service<xarm_msgs::srv::GetFloat32List>("get_position_aa", &XArmDriver::_get_position_aa);
    
        // SetFloat32
        service_set_pause_time_ = _create_service<xarm_msgs::srv::SetFloat32>("set_pause_time", &XArmDriver::_set_pause_time);
        service_set_tcp_jerk_ = _create_service<xarm_msgs::srv::SetFloat32>("set_tcp_jerk", &XArmDriver::_set_tcp_jerk);
        service_set_tcp_maxacc_ = _create_service<xarm_msgs::srv::SetFloat32>("set_tcp_maxacc", &XArmDriver::_set_tcp_maxacc);
        service_set_joint_jerk_ = _create_service<xarm_msgs::srv::SetFloat32>("set_joint_jerk", &XArmDriver::_set_joint_jerk);
        service_set_joint_maxacc_ = _create_service<xarm_msgs::srv::SetFloat32>("set_joint_maxacc", &XArmDriver::_set_joint_maxacc);
        service_set_gripper_speed_ = _create_service<xarm_msgs::srv::SetFloat32>("set_gripper_speed", &XArmDriver::_set_gripper_speed);
        service_set_reduced_max_tcp_speed_ = _create_service<xarm_msgs::srv::SetFloat32>("set_reduced_max_tcp_speed", &XArmDriver::_set_reduced_max_tcp_speed);
        service_set_reduced_max_joint_speed_ = _create_service<xarm_msgs::srv::SetFloat32>("set_reduced_max_joint_speed", &XArmDriver::_set_reduced_max_joint_speed);
    
        // SetFloat32List
        service_set_gravity_direction_ = _create_service<xarm_msgs::srv::SetFloat32List>("set_gravity_direction", &XArmDriver::_set_gravity_direction);
        service_set_tcp_offset_ = _create_service<xarm_msgs::srv::SetFloat32List>("set_tcp_offset", &XArmDriver::_set_tcp_offset);
        service_set_world_offset_ = _create_service<xarm_msgs::srv::SetFloat32List>("set_world_offset", &XArmDriver::_set_world_offset);
        service_set_reduced_joint_range_ = _create_service<xarm_msgs::srv::SetFloat32List>("set_reduced_joint_range", &XArmDriver::_set_reduced_joint_range);

        // SetTcpLoad
        service_set_tcp_load_ = _create_service<xarm_msgs::srv::SetTcpLoad>("set_tcp_load", &XArmDriver::_set_tcp_load);
        
        // MoveCartesian
        service_set_position_ = _create_service<xarm_msgs::srv::MoveCartesian>("set_position", &XArmDriver::_set_position);
        service_set_tool_position_ = _create_service<xarm_msgs::srv::MoveCartesian>("set_tool_position", &XArmDriver::_set_tool_position);
        service_set_position_aa_ = _create_service<xarm_msgs::srv::MoveCartesian>("set_position_aa", &XArmDriver::_set_position_aa);
        service_set_servo_cartesian_ = _create_service<xarm_msgs::srv::MoveCartesian>("set_servo_cartesian", &XArmDriver::_set_servo_cartesian);
        service_set_servo_cartesian_aa_ = _create_service<xarm_msgs::srv::MoveCartesian>("set_servo_cartesian_aa", &XArmDriver::_set_servo_cartesian_aa);
        
        // MoveJoint
        service_set_servo_angle_ = _create_service<xarm_msgs::srv::MoveJoint>("set_servo_angle", &XArmDriver::_set_servo_angle);
        service_set_servo_angle_j_ = _create_service<xarm_msgs::srv::MoveJoint>("set_servo_angle_j", &XArmDriver::_set_servo_angle_j);
        
        // MoveCircle
        service_move_circle_ = _create_service<xarm_msgs::srv::MoveCircle>("move_circle", &XArmDriver::_move_circle);
        
        // MoveHome
        service_move_gohome_ = _create_service<xarm_msgs::srv::MoveHome>("move_gohome", &XArmDriver::_move_gohome);
        
        // MoveVelocity
        service_vc_set_joint_velocity_ = _create_service<xarm_msgs::srv::MoveVelocity>("vc_set_joint_velocity", &XArmDriver::_vc_set_joint_velocity);
        service_vc_set_cartesian_velocity_ = _create_service<xarm_msgs::srv::MoveVelocity>("vc_set_cartesian_velocity", &XArmDriver::_vc_set_cartesian_velocity);
        
        // GetDigitalIO
        service_get_tgpio_digital_ = _create_service<xarm_msgs::srv::GetDigitalIO>("get_tgpio_digital", &XArmDriver::_get_tgpio_digital);
        service_get_cgpio_digital_ = _create_service<xarm_msgs::srv::GetDigitalIO>("get_cgpio_digital", &XArmDriver::_get_cgpio_digital);
        
        // GetAnalogIO
        service_get_tgpio_analog_ = _create_service<xarm_msgs::srv::GetAnalogIO>("get_tgpio_analog", &XArmDriver::_get_tgpio_analog);
        service_get_cgpio_analog_ = _create_service<xarm_msgs::srv::GetAnalogIO>("get_cgpio_analog", &XArmDriver::_get_cgpio_analog);
        
        // SetDigitalIO
        service_set_tgpio_digital_ = _create_service<xarm_msgs::srv::SetDigitalIO>("set_tgpio_digital", &XArmDriver::_set_tgpio_digital);
        service_set_cgpio_digital_ = _create_service<xarm_msgs::srv::SetDigitalIO>("set_cgpio_digital", &XArmDriver::_set_cgpio_digital);
        service_set_tgpio_digital_with_xyz_ = _create_service<xarm_msgs::srv::SetDigitalIO>("set_tgpio_digital_with_xyz", &XArmDriver::_set_tgpio_digital_with_xyz);
        service_set_cgpio_digital_with_xyz_ = _create_service<xarm_msgs::srv::SetDigitalIO>("set_cgpio_digital_with_xyz", &XArmDriver::_set_cgpio_digital_with_xyz);
        
        // SetAnalogIO
        service_set_cgpio_analog_ = _create_service<xarm_msgs::srv::SetAnalogIO>("set_cgpio_analog", &XArmDriver::_set_cgpio_analog);
        service_set_cgpio_analog_with_xyz_ = _create_service<xarm_msgs::srv::SetAnalogIO>("set_cgpio_analog_with_xyz", &XArmDriver::_set_cgpio_analog_with_xyz);
        
        // VacuumGripperCtrl
        service_set_vacuum_gripper_ = _create_service<xarm_msgs::srv::VacuumGripperCtrl>("set_vacuum_gripper", &XArmDriver::_set_vacuum_gripper);
        
        // GripperMove
        service_set_gripper_position_ = _create_service<xarm_msgs::srv::GripperMove>("set_gripper_position", &XArmDriver::_set_gripper_position);
        
        // BioGripperEnable
        service_set_bio_gripper_enable_ = _create_service<xarm_msgs::srv::BioGripperEnable>("set_bio_gripper_enable", &XArmDriver::_set_bio_gripper_enable);
        
        // BioGripperCtrl
        service_open_bio_gripper_ = _create_service<xarm_msgs::srv::BioGripperCtrl>("open_bio_gripper", &XArmDriver::_open_bio_gripper);
        service_close_bio_gripper_ = _create_service<xarm_msgs::srv::BioGripperCtrl>("close_bio_gripper", &XArmDriver::_close_bio_gripper);
        
        // RobotiqReset
        service_robotiq_reset_ = _create_service<xarm_msgs::srv::RobotiqReset>("robotiq_reset", &XArmDriver::_robotiq_reset);
        
        // RobotiqActivate
        service_robotiq_set_activate_ = _create_service<xarm_msgs::srv::RobotiqActivate>("robotiq_set_activate", &XArmDriver::_robotiq_set_activate);
        
        // RobotiqMove
        service_robotiq_set_position_ = _create_service<xarm_msgs::srv::RobotiqMove>("robotiq_set_position", &XArmDriver::_robotiq_set_position);
        service_robotiq_open_ = _create_service<xarm_msgs::srv::RobotiqMove>("robotiq_open", &XArmDriver::_robotiq_open);
        service_robotiq_close_ = _create_service<xarm_msgs::srv::RobotiqMove>("robotiq_close", &XArmDriver::_robotiq_close);
        
        // RobotiqGetStatus
        service_robotiq_get_status_ = _create_service<xarm_msgs::srv::RobotiqGetStatus>("robotiq_get_status", &XArmDriver::_robotiq_get_status);
        
        // GetSetModbusData
        service_getset_tgpio_modbus_data_ = _create_service<xarm_msgs::srv::GetSetModbusData>("getset_tgpio_modbus_data", &XArmDriver::_getset_tgpio_modbus_data);
        
        // TrajCtrl
        service_save_record_trajectory_ = _create_service<xarm_msgs::srv::TrajCtrl>("save_record_trajectory", &XArmDriver::_save_record_trajectory);
        service_load_trajectory_ = _create_service<xarm_msgs::srv::TrajCtrl>("load_trajectory", &XArmDriver::_load_trajectory);
        
        // TrajPlay
        service_playback_trajectory_ = _create_service<xarm_msgs::srv::TrajPlay>("playback_trajectory", &XArmDriver::_playback_trajectory);
    }

    bool XArmDriver::_clean_error(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res) 
    {
        res->ret = arm->clean_error();
        return res->ret >= 0;
    }

    bool XArmDriver::_clean_warn(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res)
    {
        res->ret = arm->clean_warn();
        return res->ret >= 0;
    }

    bool XArmDriver::_clean_conf(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res)
    {
        res->ret = arm->clean_conf();
        return res->ret >= 0;
    }

    bool XArmDriver::_save_conf(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res)
    {
        res->ret = arm->save_conf();
        return res->ret >= 0;
    }

    bool XArmDriver::_reload_dynamics(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res)
    {
        res->ret = arm->reload_dynamics();
        return res->ret >= 0;
    }

    bool XArmDriver::_set_counter_reset(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res)
    {
        res->ret = arm->set_counter_reset();
        return res->ret >= 0;
    }
    bool XArmDriver::_set_counter_increase(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res)
    {
        res->ret = arm->set_counter_increase();
        return res->ret >= 0;
    }

    bool XArmDriver::_clean_gripper_error(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res)
    {
        res->ret = arm->clean_gripper_error();
        return res->ret >= 0;
    }

    bool XArmDriver::_clean_bio_gripper_error(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res)
    {
        res->ret = arm->clean_bio_gripper_error();
        return res->ret >= 0;
    }
    
    bool XArmDriver::_start_record_trajectory(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res)
    {
        res->ret = arm->start_record_trajectory();
        return res->ret >= 0;
    }

    bool XArmDriver::_stop_record_trajectory(const std::shared_ptr<xarm_msgs::srv::Call::Request> req, std::shared_ptr<xarm_msgs::srv::Call::Response> res)
    {
        res->ret = arm->stop_record_trajectory();
        return res->ret >= 0;
    }

    bool XArmDriver::_get_state(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res)
    {
        res->ret = arm->get_state((int *)&res->data);
        res->message = "data=" + std::to_string(res->data);
        return res->ret >= 0;
    }
    
    bool XArmDriver::_get_cmdnum(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res)
    {
        res->ret = arm->get_cmdnum((int *)&res->data);
        res->message = "data=" + std::to_string(res->data);
        return res->ret >= 0; 
    }

    bool XArmDriver::_get_vacuum_gripper(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res)
    {
        res->ret = arm->get_vacuum_gripper((int *)&res->data);
        res->message = "data=" + std::to_string(res->data);
        return res->ret >= 0;
    }

    bool XArmDriver::_get_gripper_err_code(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res)
    {
        res->ret = arm->get_gripper_err_code((int *)&res->data);
        res->message = "data=" + std::to_string(res->data);
        return res->ret >= 0;
    }

    bool XArmDriver::_get_bio_gripper_status(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res)
    {
        res->ret = arm->get_bio_gripper_status((int *)&res->data);
        res->message = "data=" + std::to_string(res->data);
        return res->ret >= 0;
    }

    bool XArmDriver::_get_bio_gripper_error(const std::shared_ptr<xarm_msgs::srv::GetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16::Response> res)
    {
        res->ret = arm->get_bio_gripper_error((int *)&res->data);
        res->message = "data=" + std::to_string(res->data);
        return res->ret >= 0;
    }

    bool XArmDriver::_get_err_warn_code(const std::shared_ptr<xarm_msgs::srv::GetInt16List::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt16List::Response> res)
    {
        int err_warn[2];
        res->ret = arm->get_err_warn_code(err_warn);
        res->datas.resize(2);
        res->datas[0] = err_warn[0];
        res->datas[1]= err_warn[1];
        res->message = "datas=[ " + std::to_string(res->datas[0]) + ", " + std::to_string(res->datas[1]) + " ]";
        return res->ret >= 0;
    }

    bool XArmDriver::_set_mode(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res)
    {
        arm->set_state(XARM_STATE::STOP);
        sleep_milliseconds(10);
        res->ret = arm->set_mode(req->data);
        res->message = "data=" + std::to_string(req->data);
        return res->ret >= 0;
    }

    bool XArmDriver::_set_state(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res)
    {
        res->ret = arm->set_state(req->data);
        res->message = "data=" + std::to_string(req->data);
        return res->ret >= 0;
    }

    bool XArmDriver::_set_collision_sensitivity(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res)
    {
        res->ret = arm->set_collision_sensitivity(req->data);
        res->message = "data=" + std::to_string(req->data);
        return res->ret >= 0;
    }

    bool XArmDriver::_set_teach_sensitivity(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res)
    {
        res->ret = arm->set_teach_sensitivity(req->data);
        res->message = "data=" + std::to_string(req->data);
        return res->ret >= 0;
    }

    bool XArmDriver::_set_gripper_mode(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res)
    {
        res->ret = arm->set_gripper_mode(req->data);
        res->message = "data=" + std::to_string(req->data);
        return res->ret >= 0; 
    }
    bool XArmDriver::_set_gripper_enable(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res)
    {
        res->ret = arm->set_gripper_enable(req->data);
        res->message = "data=" + std::to_string(req->data);
        return res->ret >= 0; 
    }

    bool XArmDriver::_set_tgpio_modbus_timeout(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res)
    {
        res->ret = arm->set_tgpio_modbus_timeout(req->data);
        res->message = "data=" + std::to_string(req->data);
        return res->ret >= 0;  
    }

    bool XArmDriver::_set_bio_gripper_speed(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res)
    {
        res->ret = arm->set_bio_gripper_speed(req->data);
        res->message = "data=" + std::to_string(req->data);
        return res->ret >= 0; 
    }

    bool XArmDriver::_set_collision_rebound(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res)
    {
        res->ret = arm->set_collision_rebound(req->data);
        res->message = "data=" + std::to_string(req->data);
        return res->ret >= 0; 
    }

    bool XArmDriver::_set_fence_mode(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res)
    {
        res->ret = arm->set_fence_mode(req->data);
        res->message = "data=" + std::to_string(req->data);
        return res->ret >= 0; 
    }

    bool XArmDriver::_set_reduced_mode(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res)
    {
        res->ret = arm->set_reduced_mode(req->data);
        res->message = "data=" + std::to_string(req->data);
        return res->ret >= 0; 
    }

    bool XArmDriver::_set_self_collision_detection(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res)
    {
        res->ret = arm->set_self_collision_detection(req->data);
        res->message = "data=" + std::to_string(req->data);
        return res->ret >= 0; 
    }

    bool XArmDriver::_set_simulation_robot(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res)
    {
        res->ret = arm->set_simulation_robot(req->data);
        res->message = "data=" + std::to_string(req->data);
        return res->ret >= 0; 
    }

    bool XArmDriver::_motion_enable(const std::shared_ptr<xarm_msgs::srv::SetInt16ById::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16ById::Response> res)
    {
        res->ret = arm->motion_enable(req->data, req->id);
        res->message = "id=" + std::to_string(req->id) + ", data=" + std::to_string(req->data);
        return res->ret >= 0;
    }

    bool XArmDriver::_set_servo_attach(const std::shared_ptr<xarm_msgs::srv::SetInt16ById::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16ById::Response> res)
    {
        res->ret = arm->set_servo_attach(req->id);
        res->message = "id=" + std::to_string(req->id);
        return res->ret >= 0;
    }

    bool XArmDriver::_set_servo_detach(const std::shared_ptr<xarm_msgs::srv::SetInt16ById::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16ById::Response> res)
    {
        res->ret = arm->set_servo_detach(req->id);
        res->message = "id=" + std::to_string(req->id);
        return res->ret >= 0; 
    }

    bool XArmDriver::_set_reduced_tcp_boundary(const std::shared_ptr<xarm_msgs::srv::SetInt16List::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16List::Response> res)
    {
        if (req->datas.size() < 6) {
            res->ret = PARAM_ERROR;
            return false;
        }
        int boundary[6];
        for (int i = 0; i < 6; i++) {
            boundary[i] = req->datas[i];
        }
        res->ret = arm->set_reduced_tcp_boundary(boundary);
        std::string tmp = "";
        for (int i = 0; i < 6; i++) {
            tmp += (i == 0 ? "" : ", ") + std::to_string(boundary[i]);
        }
        res->message = "datas=[ " + tmp + " ]";
        return res->ret >= 0; 
    }

    bool XArmDriver::_get_tgpio_modbus_baudrate(const std::shared_ptr<xarm_msgs::srv::GetInt32::Request> req, std::shared_ptr<xarm_msgs::srv::GetInt32::Response> res)
    {
        res->ret = arm->get_tgpio_modbus_baudrate(&res->data);
        res->message = "data=" + std::to_string(res->data);
        return res->ret >= 0;
    }

    bool XArmDriver::_set_tgpio_modbus_baudrate(const std::shared_ptr<xarm_msgs::srv::SetInt32::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt32::Response> res)
    {
        res->ret = arm->set_tgpio_modbus_baudrate(req->data);
        res->message = "data=" + std::to_string(req->data);
        return res->ret >= 0;
    }

    bool XArmDriver::_get_gripper_position(const std::shared_ptr<xarm_msgs::srv::GetFloat32::Request> req, std::shared_ptr<xarm_msgs::srv::GetFloat32::Response> res)
    {
        res->ret = arm->get_gripper_position(&res->data);
        res->message = "data=" + std::to_string(res->data);
        return res->ret >= 0; 
    }

    bool XArmDriver::_get_position(const std::shared_ptr<xarm_msgs::srv::GetFloat32List::Request> req, std::shared_ptr<xarm_msgs::srv::GetFloat32List::Response> res)
    {
        res->datas.resize(6);
        res->ret = arm->get_position(&res->datas[0]);
        std::string tmp = "";
        for (int i = 0; i < res->datas.size(); i++) {
            tmp += (i == 0 ? "" : ", ") + std::to_string(res->datas[i]);
        }
        res->message = "datas=[ " + tmp + " ]";
        return res->ret >= 0; 
    }

    bool XArmDriver::_get_servo_angle(const std::shared_ptr<xarm_msgs::srv::GetFloat32List::Request> req, std::shared_ptr<xarm_msgs::srv::GetFloat32List::Response> res)
    {
        res->datas.resize(7);
        res->ret = arm->get_servo_angle(&res->datas[0]);
        std::string tmp = "";
        for (int i = 0; i < res->datas.size(); i++) {
            tmp += (i == 0 ? "" : ", ") + std::to_string(res->datas[i]);
        }
        res->message = "datas=[ " + tmp + " ]";
        return res->ret >= 0;
    }

    bool XArmDriver::_get_position_aa(const std::shared_ptr<xarm_msgs::srv::GetFloat32List::Request> req, std::shared_ptr<xarm_msgs::srv::GetFloat32List::Response> res)
    {
        res->datas.resize(6);
        res->ret = arm->get_position_aa(&res->datas[0]);
        std::string tmp = "";
        for (int i = 0; i < res->datas.size(); i++) {
            tmp += (i == 0 ? "" : ", ") + std::to_string(res->datas[i]);
        }
        res->message = "datas=[ " + tmp + " ]";
        return res->ret >= 0; 
    }

    bool XArmDriver::_set_pause_time(const std::shared_ptr<xarm_msgs::srv::SetFloat32::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32::Response> res)
    {
        res->ret = arm->set_pause_time(req->data);
        res->message = "data=" + std::to_string(req->data);
        return res->ret >= 0; 
    }

    bool XArmDriver::_set_tcp_jerk(const std::shared_ptr<xarm_msgs::srv::SetFloat32::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32::Response> res)
    {
        res->ret = arm->set_tcp_jerk(req->data);
        res->message = "data=" + std::to_string(req->data);
        return res->ret >= 0; 
    }

    bool XArmDriver::_set_tcp_maxacc(const std::shared_ptr<xarm_msgs::srv::SetFloat32::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32::Response> res)
    {
        res->ret = arm->set_tcp_maxacc(req->data);
        res->message = "data=" + std::to_string(req->data);
        return res->ret >= 0;  
    }

    bool XArmDriver::_set_joint_jerk(const std::shared_ptr<xarm_msgs::srv::SetFloat32::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32::Response> res)
    {
        res->ret = arm->set_joint_jerk(req->data);
        res->message = "data=" + std::to_string(req->data);
        return res->ret >= 0;  
    }

    bool XArmDriver::_set_joint_maxacc(const std::shared_ptr<xarm_msgs::srv::SetFloat32::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32::Response> res)
    {
        res->ret = arm->set_joint_maxacc(req->data);
        res->message = "data=" + std::to_string(req->data);
        return res->ret >= 0;  
    }

    bool XArmDriver::_set_gripper_speed(const std::shared_ptr<xarm_msgs::srv::SetFloat32::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32::Response> res)
    {
        res->ret = arm->set_gripper_speed(req->data);
        res->message = "data=" + std::to_string(req->data);
        return res->ret >= 0;  
    }

    bool XArmDriver::_set_reduced_max_tcp_speed(const std::shared_ptr<xarm_msgs::srv::SetFloat32::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32::Response> res)
    {
        res->ret = arm->set_reduced_max_tcp_speed(req->data);
        res->message = "data=" + std::to_string(req->data);
        return res->ret >= 0;  
    }

    bool XArmDriver::_set_reduced_max_joint_speed(const std::shared_ptr<xarm_msgs::srv::SetFloat32::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32::Response> res)
    {
        res->ret = arm->set_reduced_max_joint_speed(req->data);
        res->message = "data=" + std::to_string(req->data);
        return res->ret >= 0;  
    }

    bool XArmDriver::_set_gravity_direction(const std::shared_ptr<xarm_msgs::srv::SetFloat32List::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32List::Response> res)
    {
        if (req->datas.size() < 3) {
            res->ret = PARAM_ERROR;
            return false;
        }
            
        res->ret = arm->set_gravity_direction(&req->datas[0]);
        std::string tmp = "";
        for (int i = 0; i < 3; i++) {
            tmp += (i == 0 ? "" : ", ") + std::to_string(req->datas[i]);
        }
        res->message = "datas=[ " + tmp + " ]";
        return res->ret >= 0; 
    }

    bool XArmDriver::_set_tcp_offset(const std::shared_ptr<xarm_msgs::srv::SetFloat32List::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32List::Response> res)
    {
        if (req->datas.size() < 6) {
            res->ret = PARAM_ERROR;
            return false;
        }
        res->ret = arm->set_tcp_offset(&req->datas[0]);
        std::string tmp = "";
        for (int i = 0; i < 6; i++) {
            tmp += (i == 0 ? "" : ", ") + std::to_string(req->datas[i]);
        }
        res->message = "datas=[ " + tmp + " ]";
        return res->ret >= 0; 
    }

    bool XArmDriver::_set_world_offset(const std::shared_ptr<xarm_msgs::srv::SetFloat32List::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32List::Response> res)
    {
        if (req->datas.size() < 6) {
            res->ret = PARAM_ERROR;
            return false;
        }
        res->ret = arm->set_world_offset(&req->datas[0]);
        std::string tmp = "";
        for (int i = 0; i < 6; i++) {
            tmp += (i == 0 ? "" : ", ") + std::to_string(req->datas[i]);
        }
        res->message = "datas=[ " + tmp + " ]";
        return res->ret >= 0; 
    }

    bool XArmDriver::_set_reduced_joint_range(const std::shared_ptr<xarm_msgs::srv::SetFloat32List::Request> req, std::shared_ptr<xarm_msgs::srv::SetFloat32List::Response> res)
    {
        if (req->datas.size() < dof_ * 2) {
            res->ret = PARAM_ERROR;
            return false;
        }
        float jrange[14] = { 0 };
        for (int i = 0; i < std::min((int)req->datas.size(), 14); i++) {
            jrange[i] = req->datas[i];
        }
        res->ret = arm->set_reduced_joint_range(jrange);
        std::string tmp = "";
        for (int i = 0; i < dof_ * 2; i++) {
            tmp += (i == 0 ? "" : ", ") + std::to_string(jrange[i]);
        }
        res->message = "datas=[ " + tmp + " ]";
        return res->ret >= 0;
    }

    bool XArmDriver::_set_tcp_load(const std::shared_ptr<xarm_msgs::srv::SetTcpLoad::Request> req, std::shared_ptr<xarm_msgs::srv::SetTcpLoad::Response> res)
    {
        if (req->center_of_gravity.size() < 3) {
            res->ret = PARAM_ERROR;
            return false;
        }
        res->ret = arm->set_tcp_load(req->weight, &req->center_of_gravity[0]);
        std::string tmp = "";
        for (int i = 0; i < 3; i++) {
            tmp += (i == 0 ? "" : ", ") + std::to_string(req->center_of_gravity[i]);
        }
        res->message = "weight=" + std::to_string(req->weight) + ", center_of_gravity=[ " + tmp + " ]";
        return res->ret >= 0;
    }

    bool XArmDriver::_set_position(const std::shared_ptr<xarm_msgs::srv::MoveCartesian::Request> req, std::shared_ptr<xarm_msgs::srv::MoveCartesian::Response> res)
    {
        if (req->pose.size() < 6) {
            res->ret = PARAM_ERROR;
            return false;
        }
        res->ret = arm->set_position(&req->pose[0], req->radius, req->speed, req->acc, req->mvtime, req->wait, req->timeout);
        return res->ret >= 0;
    }

    bool XArmDriver::_set_tool_position(const std::shared_ptr<xarm_msgs::srv::MoveCartesian::Request> req, std::shared_ptr<xarm_msgs::srv::MoveCartesian::Response> res)
    {
        if (req->pose.size() < 6) {
            res->ret = PARAM_ERROR;
            return false;
        }
        res->ret = arm->set_tool_position(&req->pose[0], req->speed, req->acc, req->mvtime, req->wait, req->timeout);
        return res->ret >= 0;
    }

    bool XArmDriver::_set_position_aa(const std::shared_ptr<xarm_msgs::srv::MoveCartesian::Request> req, std::shared_ptr<xarm_msgs::srv::MoveCartesian::Response> res)
    {
        if (req->pose.size() < 6) {
            res->ret = PARAM_ERROR;
            return false;
        }
        res->ret = arm->set_position_aa(&req->pose[0], req->speed, req->acc, req->mvtime, req->is_tool_coord, req->relative, req->wait, req->timeout);
        return res->ret >= 0;
    }

    bool XArmDriver::_set_servo_cartesian(const std::shared_ptr<xarm_msgs::srv::MoveCartesian::Request> req, std::shared_ptr<xarm_msgs::srv::MoveCartesian::Response> res)
    {
        if (req->pose.size() < 6) {
            res->ret = PARAM_ERROR;
            return false;
        }
        res->ret = arm->set_servo_cartesian(&req->pose[0], req->speed, req->acc, req->mvtime, req->is_tool_coord);
        return res->ret >= 0;
    }

    bool XArmDriver::_set_servo_cartesian_aa(const std::shared_ptr<xarm_msgs::srv::MoveCartesian::Request> req, std::shared_ptr<xarm_msgs::srv::MoveCartesian::Response> res)
    {
        if (req->pose.size() < 6) {
            res->ret = PARAM_ERROR;
            return false;
        }
        res->ret = arm->set_servo_cartesian_aa(&req->pose[0], req->speed, req->acc, req->is_tool_coord, req->relative);
        return res->ret >= 0;
    }

    bool XArmDriver::_set_servo_angle(const std::shared_ptr<xarm_msgs::srv::MoveJoint::Request> req, std::shared_ptr<xarm_msgs::srv::MoveJoint::Response> res)
    {
        if (req->angles.size() < dof_) {
            res->ret = PARAM_ERROR;
            return false;
        }
        float angles[7] = { 0 };
        for (int i = 0; i < std::min((int)req->angles.size(), 7); i++) {
            angles[i] = req->angles[i];
        }
        res->ret = arm->set_servo_angle(angles, req->speed, req->acc, req->mvtime, req->wait, req->timeout, req->radius);
        return res->ret >= 0;
    }

    bool XArmDriver::_set_servo_angle_j(const std::shared_ptr<xarm_msgs::srv::MoveJoint::Request> req, std::shared_ptr<xarm_msgs::srv::MoveJoint::Response> res)
    {
        if (req->angles.size() < dof_) {
            res->ret = PARAM_ERROR;
            return false;
        }
        float angles[7] = { 0 };
        for (int i = 0; i < std::min((int)req->angles.size(), 7); i++) {
            angles[i] = req->angles[i];
        }
        res->ret = arm->set_servo_angle_j(angles, req->speed, req->acc, req->mvtime);
        return res->ret >= 0;
    }

    bool XArmDriver::_move_circle(const std::shared_ptr<xarm_msgs::srv::MoveCircle::Request> req, std::shared_ptr<xarm_msgs::srv::MoveCircle::Response> res)
    {
        if (req->pose1.size() < 6 || req->pose2.size() < 6) {
            res->ret = PARAM_ERROR;
            return false;
        }
        res->ret = arm->move_circle(&req->pose1[0], &req->pose2[0], req->percent, req->speed, req->acc, req->mvtime, req->wait, req->timeout);
        return res->ret >= 0;
    }

    bool XArmDriver::_move_gohome(const std::shared_ptr<xarm_msgs::srv::MoveHome::Request> req, std::shared_ptr<xarm_msgs::srv::MoveHome::Response> res)
    {
        res->ret = arm->move_gohome(req->speed, req->acc, req->mvtime, req->wait, req->timeout);
        return res->ret >= 0;
    }

    bool XArmDriver::_vc_set_joint_velocity(const std::shared_ptr<xarm_msgs::srv::MoveVelocity::Request> req, std::shared_ptr<xarm_msgs::srv::MoveVelocity::Response> res)
    {
        if (req->speeds.size() < dof_) {
            res->ret = PARAM_ERROR;
            return false;
        }
        float speeds[7] = { 0 };
        for (int i = 0; i < std::min((int)req->speeds.size(), 7); i++) {
            speeds[i] = req->speeds[i];
        }
        res->ret = arm->vc_set_joint_velocity(speeds, req->is_sync, req->duration);
        return res->ret >= 0;
    }

    bool XArmDriver::_vc_set_cartesian_velocity(const std::shared_ptr<xarm_msgs::srv::MoveVelocity::Request> req, std::shared_ptr<xarm_msgs::srv::MoveVelocity::Response> res)
    {
        if (req->speeds.size() < 6) {
            res->ret = PARAM_ERROR;
            return false;
        }
        res->ret = arm->vc_set_cartesian_velocity(&req->speeds[0], req->is_tool_coord, req->duration);
        return res->ret >= 0;
    }

    bool XArmDriver::_get_tgpio_digital(const std::shared_ptr<xarm_msgs::srv::GetDigitalIO::Request> req, std::shared_ptr<xarm_msgs::srv::GetDigitalIO::Response> res)
    {
        res->digitals.resize(2);
        res->ret = arm->get_tgpio_digital((int*)&res->digitals[0], (int*)&res->digitals[1]);
        res->message = "digitals=[ " + std::to_string(res->digitals[0]) + ", " + std::to_string(res->digitals[1]) + " ]";
        return res->ret >= 0;  
    }

    bool XArmDriver::_get_cgpio_digital(const std::shared_ptr<xarm_msgs::srv::GetDigitalIO::Request> req, std::shared_ptr<xarm_msgs::srv::GetDigitalIO::Response> res)
    {
        int digitals[8];
        int digitals2[8];
        res->ret = arm->get_cgpio_digital(digitals, digitals2);
        res->digitals.resize(16);
        for (int i = 0; i < 8; i++) {
            res->digitals[i] = digitals[i];
            res->digitals[i + 8] = digitals2[i];
        }
        std::string tmp = "";
        for (int i = 0; i < res->digitals.size(); i++) {
            tmp += (i == 0 ? "" : ", ") + std::to_string(res->digitals[i]);
        }
        res->message = "digitals=[ " + tmp + " ]";
        return res->ret >= 0; 
    }

    bool XArmDriver::_get_tgpio_analog(const std::shared_ptr<xarm_msgs::srv::GetAnalogIO::Request> req, std::shared_ptr<xarm_msgs::srv::GetAnalogIO::Response> res)
    {
        res->ret = arm->get_tgpio_analog(req->ionum, &res->data);
        res->message = "ionum=" + std::to_string(req->ionum) + ", data= " + std::to_string(res->data);
        return res->ret >= 0;
    }

    bool XArmDriver::_get_cgpio_analog(const std::shared_ptr<xarm_msgs::srv::GetAnalogIO::Request> req, std::shared_ptr<xarm_msgs::srv::GetAnalogIO::Response> res)
    {
        res->ret = arm->get_cgpio_analog(req->ionum, &res->data);
        res->message = "ionum=" + std::to_string(req->ionum) + ", data= " + std::to_string(res->data);
        return res->ret >= 0; 
    }

    bool XArmDriver::_set_tgpio_digital(const std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Request> req, std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Response> res)
    {
        res->ret = arm->set_tgpio_digital(req->ionum, req->value, req->delay_sec);
        res->message = "ionum=" + std::to_string(req->ionum) + ", value= " + std::to_string(req->value) + ", delay_sec=" + std::to_string(req->delay_sec);
        return res->ret >= 0; 
    }

    bool XArmDriver::_set_cgpio_digital(const std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Request> req, std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Response> res)
    {
        res->ret = arm->set_cgpio_digital(req->ionum, req->value, req->delay_sec);
        res->message = "ionum=" + std::to_string(req->ionum) + ", value= " + std::to_string(req->value) + ", delay_sec=" + std::to_string(req->delay_sec);
        return res->ret >= 0; 
    }

    bool XArmDriver::_set_tgpio_digital_with_xyz(const std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Request> req, std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Response> res)
    {
        res->ret = arm->set_tgpio_digital_with_xyz(req->ionum, req->value, &req->xyz[0], req->tol_r);
        return res->ret >= 0;
    }

    bool XArmDriver::_set_cgpio_digital_with_xyz(const std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Request> req, std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Response> res)
    {
        res->ret = arm->set_cgpio_digital_with_xyz(req->ionum, req->value, &req->xyz[0], req->tol_r);
        return res->ret >= 0;
    }

    bool XArmDriver::_set_cgpio_analog(const std::shared_ptr<xarm_msgs::srv::SetAnalogIO::Request> req, std::shared_ptr<xarm_msgs::srv::SetAnalogIO::Response> res)
    {
        res->ret = arm->set_cgpio_analog(req->ionum, req->value);
        res->message = "ionum=" + std::to_string(req->ionum) + ", value= " + std::to_string(req->value);
        return res->ret >= 0;
    }

    bool XArmDriver::_set_cgpio_analog_with_xyz(const std::shared_ptr<xarm_msgs::srv::SetAnalogIO::Request> req, std::shared_ptr<xarm_msgs::srv::SetAnalogIO::Response> res)
    {
        res->ret = arm->set_cgpio_analog_with_xyz(req->ionum, req->value, &req->xyz[0], req->tol_r);
        return res->ret >= 0;
    }

    bool XArmDriver::_set_vacuum_gripper(const std::shared_ptr<xarm_msgs::srv::VacuumGripperCtrl::Request> req, std::shared_ptr<xarm_msgs::srv::VacuumGripperCtrl::Response> res)
    {
        res->ret = arm->set_vacuum_gripper(req->on, req->wait, req->timeout, req->delay_sec);
        return res->ret >= 0;
    }

    bool XArmDriver::_set_gripper_position(const std::shared_ptr<xarm_msgs::srv::GripperMove::Request> req, std::shared_ptr<xarm_msgs::srv::GripperMove::Response> res)
    {
        res->ret = arm->set_gripper_position(req->pos, req->wait, req->timeout);
        return res->ret >= 0;
    }

    bool XArmDriver::_set_bio_gripper_enable(const std::shared_ptr<xarm_msgs::srv::BioGripperEnable::Request> req, std::shared_ptr<xarm_msgs::srv::BioGripperEnable::Response> res)
    {
        res->ret = arm->set_bio_gripper_enable(req->enable, req->wait, req->timeout);
        return res->ret >= 0;
    }

    bool XArmDriver::_open_bio_gripper(const std::shared_ptr<xarm_msgs::srv::BioGripperCtrl::Request> req, std::shared_ptr<xarm_msgs::srv::BioGripperCtrl::Response> res)
    {
        res->ret = arm->open_bio_gripper(req->speed, req->wait, req->timeout);
        return res->ret >= 0;
    }

    bool XArmDriver::_close_bio_gripper(const std::shared_ptr<xarm_msgs::srv::BioGripperCtrl::Request> req, std::shared_ptr<xarm_msgs::srv::BioGripperCtrl::Response> res)        
    {
        res->ret = arm->close_bio_gripper(req->speed, req->wait, req->timeout);
        return res->ret >= 0;
    }

    bool XArmDriver::_robotiq_reset(const std::shared_ptr<xarm_msgs::srv::RobotiqReset::Request> req, std::shared_ptr<xarm_msgs::srv::RobotiqReset::Response> res)
    {
        res->ret_data.resize(6);
        res->ret = arm->robotiq_reset(&res->ret_data[0]);
        return res->ret >= 0;
    }

    bool XArmDriver::_robotiq_set_activate(const std::shared_ptr<xarm_msgs::srv::RobotiqActivate::Request> req, std::shared_ptr<xarm_msgs::srv::RobotiqActivate::Response> res)
    {
        res->ret_data.resize(6);
        res->ret = arm->robotiq_set_activate(req->wait, req->timeout, &res->ret_data[0]);
        return res->ret >= 0; 
    }

    bool XArmDriver::_robotiq_set_position(const std::shared_ptr<xarm_msgs::srv::RobotiqMove::Request> req, std::shared_ptr<xarm_msgs::srv::RobotiqMove::Response> res)
    {
        res->ret_data.resize(6);
        res->ret = arm->robotiq_set_position(req->pos, req->speed, req->force, req->wait, req->timeout, &res->ret_data[0]);
        return res->ret >= 0;
    }

    bool XArmDriver::_robotiq_open(const std::shared_ptr<xarm_msgs::srv::RobotiqMove::Request> req, std::shared_ptr<xarm_msgs::srv::RobotiqMove::Response> res)
    {
        res->ret_data.resize(6);
        res->ret = arm->robotiq_open(req->speed, req->force, req->wait, req->timeout, &res->ret_data[0]);
        return res->ret >= 0;
    }

    bool XArmDriver::_robotiq_close(const std::shared_ptr<xarm_msgs::srv::RobotiqMove::Request> req, std::shared_ptr<xarm_msgs::srv::RobotiqMove::Response> res)
    {
        res->ret_data.resize(6);
        res->ret = arm->robotiq_close(req->speed, req->force, req->wait, req->timeout, &res->ret_data[0]);
        return res->ret >= 0;
    }

    bool XArmDriver::_robotiq_get_status(const std::shared_ptr<xarm_msgs::srv::RobotiqGetStatus::Request> req, std::shared_ptr<xarm_msgs::srv::RobotiqGetStatus::Response> res)
    {
        res->ret_data.resize(9);
        res->ret = arm->robotiq_get_status(&res->ret_data[0], req->number_of_registers);
        return res->ret >= 0;
    }

    bool XArmDriver::_getset_tgpio_modbus_data(const std::shared_ptr<xarm_msgs::srv::GetSetModbusData::Request> req, std::shared_ptr<xarm_msgs::srv::GetSetModbusData::Response> res)
    {
        res->ret_data.resize(req->ret_length);
        res->ret = arm->getset_tgpio_modbus_data(&req->modbus_data[0], req->modbus_length, &res->ret_data[0], req->ret_length);
        return res->ret >= 0;
    }

    bool XArmDriver::_save_record_trajectory(const std::shared_ptr<xarm_msgs::srv::TrajCtrl::Request> req, std::shared_ptr<xarm_msgs::srv::TrajCtrl::Response> res)
    {
        if (req->filename.size() > 80 || req->filename.size() <= 0) {
            res->ret = PARAM_ERROR;
            return false;
        }
        char filename[81]={0};
        req->filename.copy(filename, req->filename.size(), 0);
        res->ret = arm->save_record_trajectory(filename, req->timeout);
        return res->ret >= 0;
    }

    bool XArmDriver::_load_trajectory(const std::shared_ptr<xarm_msgs::srv::TrajCtrl::Request> req, std::shared_ptr<xarm_msgs::srv::TrajCtrl::Response> res)
    {
        if (req->filename.size() > 80 || req->filename.size() <= 0) {
            res->ret = PARAM_ERROR;
            return false;
        }
        char filename[81]={0};
        req->filename.copy(filename, req->filename.size(), 0);
        res->ret = arm->load_trajectory(filename, req->timeout);
        return res->ret >= 0; 
    }

    bool XArmDriver::_playback_trajectory(const std::shared_ptr<xarm_msgs::srv::TrajPlay::Request> req, std::shared_ptr<xarm_msgs::srv::TrajPlay::Response> res)
    {
        if (req->filename.size() > 80) {
            res->ret = PARAM_ERROR;
            return false;
        }
        char *filename = NULL;
        if (req->filename.size() != 0) {
            char file_name[81]={0};
            req->filename.copy(file_name, req->filename.size(), 0);
            filename = file_name;
        }        
        res->ret = arm->playback_trajectory(req->times, filename, req->wait, req->double_speed);
        return res->ret >= 0; 
    }
}