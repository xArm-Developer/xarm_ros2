/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
           Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#include "xarm_controller/xarm_hw.h"

#define VELO_DURATION 1

namespace xarm_control
{
    void XArmHW::_receive_event(const std_msgs::msg::String::SharedPtr event)
    {
        RCLCPP_INFO(node_->get_logger(), "receive_event: %s", event->data.c_str());
    }

    void XArmHW::_joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr states)
    {
        // std::string pos_str = "[ ";
        // std::string vel_str = "[ ";
        // for (int i = 0; i < states->position.size(); i++) { 
        //     pos_str += std::to_string(states->position[i]); 
        //     pos_str += " ";
        //     vel_str += std::to_string(states->velocity[i]); 
        //     vel_str += " ";
        // }
        // pos_str += "]";
        // vel_str += "]";
        // RCLCPP_INFO(node_->get_logger(), "state_position: %s", pos_str.c_str());
        // RCLCPP_INFO(node_->get_logger(), "state_velocity: %s", vel_str.c_str());

        if (info_.joints.size() != states->name.size()) {
            // RCLCPP_INFO(node_->get_logger(), "*****info_.joints.size()=%d, states->name.size()=%d", info_.joints.size(), states->name.size());
            return;
        }
        for (uint i = 0; i < info_.joints.size(); i++) {
            if (info_.joints[i].name != states->name[i]) {
                // RCLCPP_INFO(node_->get_logger(), "*****info_.joints[%d].name=%s, states->name[%d]=%s", 
                //     i, info_.joints[i].name.c_str(), i, states->name[i].c_str());
                return;
            }
        }

        for (uint i = 0; i < position_states_.size(); i++) {
            position_states_[i] = states->position[i];
        }
        for (uint i = 0; i < velocity_states_.size(); i++) {
            velocity_states_[i] = states->velocity[i];
            // velocity_states_[i] = 0.000001;
        }
        if (initial_write_) {
            initial_write_ = false;
        }
    }

    void XArmHW::_xarm_states_callback(const xarm_msgs::msg::RobotMsg::SharedPtr states)
    {
        if (((curr_state_ != 4 && curr_state_ != 5) && (states->state == 4 || states->state == 5))
            || (states->mode != 1 && curr_mode_ == 1) || (states->err != 0 && curr_err_ == 0)
        ) {
            RCLCPP_ERROR(node_->get_logger(), "mode: %d, state: %d, error: %d", states->mode, states->state, states->err);
        }
        curr_state_ = states->state;
		curr_mode_ = states->mode;
		curr_err_ = states->err;
    }

    hardware_interface::return_type XArmHW::configure(const hardware_interface::HardwareInfo & info)
    {
        info_ = info;
        initial_write_ = true;
        velocity_control_ = false;
        curr_state_ = 4;
        curr_mode_ = 0;
        curr_err_ = 0;
        node_ = rclcpp::Node::make_shared("xarm_hw");
        RCLCPP_INFO(node_->get_logger(), "namespace: %s", node_->get_namespace());
        
        position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        position_cmds_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        velocity_cmds_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        cmds_float_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        prev_cmds_float_.resize(info_.joints.size());

        for (const hardware_interface::ComponentInfo & joint : info_.joints) {
            bool has_pos_cmd_interface = false;
            for (auto i = 0u; i < joint.command_interfaces.size(); ++i) {
                if (joint.command_interfaces[i].name == hardware_interface::HW_IF_POSITION) {
                    has_pos_cmd_interface = true;
                    break;
                }
            }
            if (!has_pos_cmd_interface) {
                RCLCPP_ERROR(node_->get_logger(), "Joint '%s' has %d command interfaces found, but not found %s command interface",
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
                RCLCPP_ERROR(node_->get_logger(), "Joint '%s' has %d state interfaces found, but not found %s state interface",
                    joint.name.c_str(), joint.state_interfaces.size(), hardware_interface::HW_IF_POSITION
                );
                return hardware_interface::return_type::ERROR;
            }
        }

        RCLCPP_INFO(node_->get_logger(), "System Sucessfully configured!");
        status_ = hardware_interface::status::CONFIGURED;
        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface> XArmHW::export_state_interfaces()
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

    std::vector<hardware_interface::CommandInterface> XArmHW::export_command_interfaces()
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

    hardware_interface::return_type XArmHW::start()
    {
        std::string hw_ns = "xarm";
        auto it = info_.hardware_parameters.find("hw_ns");
        if (it != info_.hardware_parameters.end()) {
            hw_ns = it->second;
        }

        auto it2 = info_.hardware_parameters.find("velocity_control");
        if (it2 != info_.hardware_parameters.end()) {
            velocity_control_ = (it2->second == "True" || it2->second == "true");
        }

        // node_ = rclcpp::Node::make_shared("xarm_hw");
        RCLCPP_INFO(node_->get_logger(), "hw_ns: %s, velocity_control: %d", hw_ns.c_str(), velocity_control_);
        joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(hw_ns + "/joint_states", 100, std::bind(&XArmHW::_joint_states_callback, this, std::placeholders::_1));
        xarm_state_sub_ = node_->create_subscription<xarm_msgs::msg::RobotMsg>(hw_ns + "/xarm_states", 100, std::bind(&XArmHW::_xarm_states_callback, this, std::placeholders::_1));
        // trajectory_execution_event_sub_ = node_->create_subscription<std_msgs::msg::String>("trajectory_execution_event", 100, std::bind(&XArmHW::_receive_event, this, std::placeholders::_1));
        std::thread th([this]() -> void {
            rclcpp::spin(node_);
            rclcpp::shutdown();
        });
        th.detach();
        rclcpp::sleep_for(std::chrono::seconds(1));
        
        client_node_ = rclcpp::Node::make_shared("xarm_ros_client");
        xarm_client_.init(client_node_, hw_ns);
        xarm_client_.motion_enable(true);
        if (velocity_control_)
            xarm_client_.set_mode(4);
        else
            xarm_client_.set_mode(1);
        xarm_client_.set_state(0);

        rclcpp::sleep_for(std::chrono::seconds(1));

        for (uint i = 0; i < position_states_.size(); i++) {
            if (std::isnan(position_states_[i])) {
                position_states_[i] = 0;
                position_cmds_[i] = 0;
            } else {
                position_cmds_[i] = position_states_[i];
            }
        }
        for (uint i = 0; i < velocity_states_.size(); i++) {
            if (std::isnan(velocity_states_[i])) {
                velocity_states_[i] = 0;
                velocity_cmds_[i] = 0;
            } else {
                velocity_cmds_[i] = velocity_states_[i];
            }
        }

        status_ = hardware_interface::status::STARTED;
        
        RCLCPP_INFO(node_->get_logger(), "System Sucessfully started!");
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type XArmHW::stop()
    {
        RCLCPP_INFO(node_->get_logger(), "Stopping ...please wait...");
        status_ = hardware_interface::status::STOPPED;

        xarm_client_.set_mode(0);

        RCLCPP_INFO(node_->get_logger(), "System sucessfully stopped!");
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type XArmHW::read()
    {
        return hardware_interface::return_type::OK;
    }

    bool XArmHW::_check_cmds_is_change(std::vector<float> prev, std::vector<float> cur, double threshold)
    {
        for (int i = 0; i < cur.size(); i++) {
            if (std::abs(cur[i] - prev[i]) > threshold) return true;
        }
        return false;
    }

    hardware_interface::return_type XArmHW::write()
    {
        if (initial_write_) {
            return hardware_interface::return_type::OK;
        }
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
        // RCLCPP_INFO(node_->get_logger(), "positon: %s, velocity: %s", pos_str.c_str(), vel_str.c_str());

        if (velocity_control_) {
            for (int i = 0; i < velocity_cmds_.size(); i++) { 
                cmds_float_[i] = (float)velocity_cmds_[i];
            }
            // RCLCPP_INFO(node_->get_logger(), "velocity: %s", vel_str.c_str());
            int ret = xarm_client_.vc_set_joint_velocity(cmds_float_, true, VELO_DURATION);
            if (ret != 0) {
                RCLCPP_WARN(node_->get_logger(), "vc_set_joint_velocity, ret= %d", ret);
            }
        }
        else {
            for (int i = 0; i < position_cmds_.size(); i++) { 
                cmds_float_[i] = (float)position_cmds_[i];
            }
            cur_time_ = node_->get_clock()->now();
            if (cur_time_.seconds() - last_time_.seconds() > 1 || _check_cmds_is_change(prev_cmds_float_, cmds_float_)) {
                // RCLCPP_INFO(node_->get_logger(), "positon: %s", pos_str.c_str());
                int ret = xarm_client_.set_servo_angle_j(cmds_float_);
                if (ret != 0) {
                    RCLCPP_WARN(node_->get_logger(), "set_servo_angle_j, ret= %d", ret);
                }
                if (ret == 0) {
                    last_time_ = cur_time_;
                    for (int i = 0; i < prev_cmds_float_.size(); i++) { 
                        prev_cmds_float_[i] = (float)cmds_float_[i];
                    }
                }
            }
        }

        return hardware_interface::return_type::OK;
    }
}

