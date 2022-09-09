/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
           Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#include "xarm_controller/hardware/uf_robot_fake_system_hardware.h"

namespace uf_robot_hardware
{
    static const rclcpp::Logger LOGGER = rclcpp::get_logger("UFACTORY.RobotFakeHW");

    hardware_interface::return_type UFRobotFakeSystemHardware::configure(const hardware_interface::HardwareInfo & info)
    {
        info_ = info;

        node_ = rclcpp::Node::make_shared("uf_robot_fake_hw");
        joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1000);
        node_thread_ = std::thread([this]() {
            rclcpp::spin(node_);
        });
        
        joint_state_msg_.header.frame_id = "joint-state data";
        joint_state_msg_.name.resize(info_.joints.size());
        joint_state_msg_.position.resize(info_.joints.size(), 0);
        joint_state_msg_.velocity.resize(info_.joints.size(), 0);
        joint_state_msg_.effort.resize(info_.joints.size(), 0);

        for (int i = 0; i < info_.joints.size(); i++) {
            joint_state_msg_.name[i] = info_.joints[i].name;
        }
        
        position_states_.resize(info_.joints.size(), 0);
        velocity_states_.resize(info_.joints.size(), 0);
        position_cmds_.resize(info_.joints.size(), 0);
        velocity_cmds_.resize(info_.joints.size(), 0);

        for (const hardware_interface::ComponentInfo & joint : info_.joints) {
            bool has_pos_cmd_interface = false;
            for (auto i = 0u; i < joint.command_interfaces.size(); ++i) {
                if (joint.command_interfaces[i].name == hardware_interface::HW_IF_POSITION) {
                    has_pos_cmd_interface = true;
                    break;
                }
            }
            if (!has_pos_cmd_interface) {
                RCLCPP_ERROR(LOGGER, "Joint '%s' has %d command interfaces found, but not found %s command interface",
                    joint.name.c_str(), joint.command_interfaces.size(), hardware_interface::HW_IF_POSITION
                );
                return hardware_interface::return_type::ERROR;
            }

            bool has_pos_state_interface = false;
            for (auto i = 0u; i < joint.state_interfaces.size(); ++i) {
                if (joint.state_interfaces[i].name == hardware_interface::HW_IF_POSITION) {
                    has_pos_state_interface = true;
                    break;
                }
            }
            if (!has_pos_state_interface) {
                RCLCPP_ERROR(LOGGER, "Joint '%s' has %d state interfaces found, but not found %s state interface",
                    joint.name.c_str(), joint.state_interfaces.size(), hardware_interface::HW_IF_POSITION
                );
                return hardware_interface::return_type::ERROR;
            }
        }

        RCLCPP_INFO(LOGGER, "System Sucessfully configured!");
        status_ = hardware_interface::status::CONFIGURED;
        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface> UFRobotFakeSystemHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> UFRobotFakeSystemHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_cmds_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_cmds_[i]));
        }

        return command_interfaces;
    }

    hardware_interface::return_type UFRobotFakeSystemHardware::start()
    {
        status_ = hardware_interface::status::STARTED;
        
        RCLCPP_INFO(LOGGER, "System Sucessfully started!");
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type UFRobotFakeSystemHardware::stop()
    {
        RCLCPP_INFO(LOGGER, "Stopping ...please wait...");
        status_ = hardware_interface::status::STOPPED;


        RCLCPP_INFO(LOGGER, "System sucessfully stopped!");
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type UFRobotFakeSystemHardware::read()
    {
        joint_state_msg_.header.stamp = node_->get_clock()->now();
        joint_state_pub_->publish(joint_state_msg_);
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type UFRobotFakeSystemHardware::write()
    {
        // std::string pos_str = "[ ";
        // std::string vel_str = "[ ";
        // for (int i = 0; i < position_cmds_.size(); i++) { 
        //     pos_str += std::to_string(position_cmds_[i]); 
        //     pos_str += " ";
        //     vel_str += std::to_string(velocity_cmds_[i]); 
        //     vel_str += " ";
        // }
        // pos_str += "]";
        // vel_str += "]";
        // RCLCPP_INFO(LOGGER, "positon: %s, velocity: %s", pos_str.c_str(), vel_str.c_str());

        for (int i = 0; i < position_cmds_.size(); i++) { 
            position_states_[i] = position_cmds_[i];
            joint_state_msg_.position[i] = position_cmds_[i];
        }
        for (int i = 0; i < velocity_cmds_.size(); i++) { 
            velocity_states_[i] = velocity_cmds_[i];
            joint_state_msg_.velocity[i] = velocity_cmds_[i];
        }

        return hardware_interface::return_type::OK;
    }
}

