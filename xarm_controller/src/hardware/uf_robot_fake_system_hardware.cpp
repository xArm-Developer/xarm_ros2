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
        joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("robot_joint_states", 1000);
        // Add a flag to indicate whether the node should shut down
        
        stop_spin_flag.store(false);
        
        node_thread_ = std::thread([this]() {
            while (!stop_spin_flag.load()) {
                rclcpp::spin_some(node_); // Use spin_some instead of spin to allow frequent checks
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Poll every 100ms
            }
        });
        
        joint_state_msg_.header.frame_id = "joint-state data";
        joint_state_msg_.name.resize(info_.joints.size());
        joint_state_msg_.position.resize(info_.joints.size(), 0);
        joint_state_msg_.velocity.resize(info_.joints.size(), 0);
        joint_state_msg_.effort.resize(info_.joints.size(), 0);

        position_states_.resize(info_.joints.size(), 0);
        velocity_states_.resize(info_.joints.size(), 0);
        velocity_cmds_.resize(info_.joints.size(), 0);

        for (int i = 0; i < info_.joints.size(); i++) {
            joint_state_msg_.name[i] = info_.joints[i].name;
            position_states_[i] = std::stod( info_.joints[i].parameters["sim_init"]);
            joint_state_msg_.position[i] = position_states_[i];
        }


        for (const hardware_interface::ComponentInfo & joint : info_.joints) {
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
        // Set stop flag to true to stop the spin loop
        stop_spin_flag.store(true);


        // Wait for the thread to finish
        if (node_thread_.joinable()) {
            node_thread_.join();
            RCLCPP_INFO(LOGGER, "Node thread joined successfully.");
        }

        RCLCPP_INFO(LOGGER, "System successfully deactivated!");
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
        for (int i = 0; i < position_states_.size(); i++) {
            // hard code to 250Hz since I dont't see how to get the set control loop rate here. "period" is only the time the last write() took. 
            position_states_[i] = position_states_[i] +  velocity_cmds_[i] * (1.0/250.0);

            joint_state_msg_.position[i] = position_states_[i];
        }
        for (int i = 0; i < velocity_cmds_.size(); i++) { 
            velocity_states_[i] = velocity_cmds_[i];
            joint_state_msg_.velocity[i] = velocity_cmds_[i];
        }

        return hardware_interface::return_type::OK;
    }
}

