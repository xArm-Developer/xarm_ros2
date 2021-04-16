/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
           Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#include "xarm_controller/xarm_hw.h"

#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("xarm_hw"), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("xarm_hw"), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("xarm_hw"), __VA_ARGS__)

namespace xarm_control
{
    void XArmHW::_joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr states)
    {        
        for (uint i = 0; i < position_states_.size(); i++) {
            position_states_[i] = states->position[i];
        }
    }

    void XArmHW::_xarm_states_callback(const xarm_msgs::msg::RobotMsg::SharedPtr states)
    {
        curr_state = states->state;
		curr_mode = states->mode;
		curr_err = states->err;
    }

    hardware_interface::return_type XArmHW::configure(const hardware_interface::HardwareInfo & info)
    {
        info_ = info;
        
        position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        position_cmds_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        for (const hardware_interface::ComponentInfo & joint : info_.joints) {
            if (joint.command_interfaces.size() != 1) {
                ROS_ERROR("Joint '%s' has %d command interfaces found. 1 expected.", 
                    joint.name.c_str(), joint.command_interfaces.size());
                return hardware_interface::return_type::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                ROS_ERROR("Joint '%s' have %s command interfaces found. '%s' expected.", 
                    joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::return_type::ERROR;
            }

            if (joint.state_interfaces.size() != 1) {
                ROS_ERROR("Joint '%s' has %d state interface. 1 expected.",
                    joint.name.c_str(), joint.state_interfaces.size());
                return hardware_interface::return_type::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                ROS_ERROR("Joint '%s' have %s state interface. '%s' expected.",
                    joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::return_type::ERROR;
            }
        }

        ROS_INFO("System Sucessfully configured!");
        status_ = hardware_interface::status::CONFIGURED;
        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface> XArmHW::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++) {
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> XArmHW::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++) {
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_cmds_[i]));
        }

        return command_interfaces;
    }

    hardware_interface::return_type XArmHW::start()
    {
        node_ = rclcpp::Node::make_shared("xarm_hw", "xarm");
        xarm_client_.init(node_);
        joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>("joint_states", 100, std::bind(&XArmHW::_joint_states_callback, this, std::placeholders::_1));
        xarm_state_sub_ = node_->create_subscription<xarm_msgs::msg::RobotMsg>("xarm_states", 100, std::bind(&XArmHW::_xarm_states_callback, this, std::placeholders::_1));
        std::thread th([this]() -> void {
            rclcpp::spin(node_);
        });
        th.detach();

        rclcpp::sleep_for(std::chrono::seconds(2));

        for (uint i = 0; i < position_states_.size(); i++) {
            if (std::isnan(position_states_[i])) {
                position_states_[i] = 0;
                position_cmds_[i] = 0;
            } else {
                position_cmds_[i] = position_states_[i];
            }
        }

        status_ = hardware_interface::status::STARTED;
        
        ROS_INFO("System Sucessfully started!");
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type XArmHW::stop()
    {
        ROS_INFO("Stopping ...please wait...");
        status_ = hardware_interface::status::STOPPED;

        ROS_INFO("System sucessfully stopped!");
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type XArmHW::read()
    {
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type XArmHW::write()
    {
        // std::string pos_str = "[ ";
        // for (int i = 0; i < position_cmds_.size(); i++) { 
        //     pos_str += std::to_string(position_cmds_[i]); 
        //     pos_str += " ";
        // }
        // pos_str += "]";
        // ROS_INFO("positon: %s", pos_str.c_str());
        return hardware_interface::return_type::OK;
    }
}

