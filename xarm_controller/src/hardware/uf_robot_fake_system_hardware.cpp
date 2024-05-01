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

    CallbackReturn UFRobotFakeSystemHardware::on_init(const hardware_interface::HardwareInfo& info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }
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
                RCLCPP_ERROR(LOGGER, "Joint '%s' has %ld command interfaces found, but not found %s command interface",
                    joint.name.c_str(), joint.command_interfaces.size(), hardware_interface::HW_IF_POSITION
                );
                return CallbackReturn::ERROR;
            }

            bool has_pos_state_interface = false;
            for (auto i = 0u; i < joint.state_interfaces.size(); ++i) {
                if (joint.state_interfaces[i].name == hardware_interface::HW_IF_POSITION) {
                    has_pos_state_interface = true;
                    break;
                }
            }
            if (!has_pos_state_interface) {
                RCLCPP_ERROR(LOGGER, "Joint '%s' has %ld state interfaces found, but not found %s state interface",
                    joint.name.c_str(), joint.state_interfaces.size(), hardware_interface::HW_IF_POSITION
                );
                return CallbackReturn::ERROR;
            }
        }

        // Initialize with values from URDF
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            const auto & joint = info_.joints[i];
            for (const auto & interface : joint.state_interfaces)
            {
                if (interface.name == hardware_interface::HW_IF_POSITION)
                {
                    // Check the initial_value param is used
                    if (!interface.initial_value.empty())
                    {
                        position_cmds_[i] = std::stod(interface.initial_value);
                    }
                    else
                    {
                        // Initialize the value in old way with warning message
                        auto it2 = joint.parameters.find("initial_" + interface.name);
                        if (it2 != joint.parameters.end())
                        {
                            position_cmds_[i] = std::stod(it2->second);
                            RCUTILS_LOG_WARN_NAMED(
                            "fake_generic_system",
                            "The usage of initial_%s has been deprecated. Please use 'initial_value' instead.",
                            interface.name.c_str());
                        }
                    }
                }
            }
        }

        RCLCPP_INFO(LOGGER, "System Sucessfully inited!");
        return CallbackReturn::SUCCESS;
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

    CallbackReturn UFRobotFakeSystemHardware::on_activate(const rclcpp_lifecycle::State& previous_state)
    {
        RCLCPP_INFO(LOGGER, "System Sucessfully activated!");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn UFRobotFakeSystemHardware::on_deactivate(const rclcpp_lifecycle::State& previous_state)
    {
        RCLCPP_INFO(LOGGER, "Stopping ...please wait...");

        RCLCPP_INFO(LOGGER, "System sucessfully deactivated!");
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type UFRobotFakeSystemHardware::read(const rclcpp::Time & time, const rclcpp::Duration &period)
    {
        joint_state_msg_.header.stamp = node_->get_clock()->now();
        joint_state_pub_->publish(joint_state_msg_);
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type UFRobotFakeSystemHardware::write(const rclcpp::Time & time, const rclcpp::Duration &period)
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

