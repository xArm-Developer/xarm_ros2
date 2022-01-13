/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
           Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#include "xarm_controller/xarm_hw.h"

#define SERVICE_CALL_FAILED 999
#define SERVICE_IS_PERSISTENT_BUT_INVALID 998
#define XARM_IS_DISCONNECTED -1
#define WAIT_SERVICE_TIMEOUT 996
#define VELO_DURATION 1

namespace xarm_control
{
    template<typename ServiceT, typename SharedRequest = typename ServiceT::Request::SharedPtr, typename SharedResponse = typename ServiceT::Response::SharedPtr>
    int XArmHW::_call_request(std::shared_ptr<ServiceT> client, SharedRequest req, SharedResponse& res)
    {
        bool is_try_again = false;
        int failed_cnts = 0;
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                exit(1);
            }
            if (!is_try_again) {
                is_try_again = true;
                RCLCPP_WARN(node_->get_logger(), "service %s not available, waiting ...", client->get_service_name());
            }
            failed_cnts += 1;
            if (failed_cnts >= 5) return WAIT_SERVICE_TIMEOUT;
        }
        auto result_future = client->async_send_request(req);
        if (rclcpp::spin_until_future_complete(client_node_, result_future, std::chrono::seconds(1)) != rclcpp::FutureReturnCode::SUCCESS)
        {
            // RCLCPP_ERROR(node_->get_logger(), "Failed to call service %s", client->get_service_name());
            return SERVICE_CALL_FAILED;
        }
        res = result_future.get();
        return 0;
    }

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

    CallbackReturn XArmHW::on_init(const hardware_interface::HardwareInfo& info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }
        info_ = info;
        velocity_control_ = false;
        curr_state_ = 4;
        curr_mode_ = 0;
        curr_err_ = 0;
        read_code_ = 0;
        write_code_ = 0;

        initialized_ = false;
        reload_controller_ = false;

        read_cnts_ = 0;
        read_max_time_ = 0;
        read_total_time_ = 0;
        read_failed_cnts_ = 0;

        node_ = rclcpp::Node::make_shared("xarm_hw");
        RCLCPP_INFO(node_->get_logger(), "namespace: %s", node_->get_namespace());
        
        prev_read_angles_.resize(7);
        curr_read_angles_.resize(7);

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
                RCLCPP_ERROR(node_->get_logger(), "[ns: %s] Joint '%s' has %ld command interfaces found, but not found %s command interface",
                    node_->get_namespace(), joint.name.c_str(), joint.command_interfaces.size(), hardware_interface::HW_IF_POSITION
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
                RCLCPP_ERROR(node_->get_logger(), "[ns: %s] Joint '%s' has %ld state interfaces found, but not found %s state interface",
                    node_->get_namespace(), joint.name.c_str(), joint.state_interfaces.size(), hardware_interface::HW_IF_POSITION
                );
                return CallbackReturn::ERROR;
            }
        }

        RCLCPP_INFO(node_->get_logger(), "[ns:%s] System Sucessfully inited!", node_->get_namespace());
        return CallbackReturn::SUCCESS;
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

    CallbackReturn XArmHW::on_activate(const rclcpp_lifecycle::State& previous_state)
    {
        hw_ns_ = "xarm";
        auto it = info_.hardware_parameters.find("hw_ns");
        if (it != info_.hardware_parameters.end()) {
            hw_ns_ = it->second;
        }

        auto it2 = info_.hardware_parameters.find("velocity_control");
        if (it2 != info_.hardware_parameters.end()) {
            velocity_control_ = (it2->second == "True" || it2->second == "true");
        }

        RCLCPP_INFO(node_->get_logger(), "[ns: %s] hw_ns: %s, velocity_control: %d", node_->get_namespace(), hw_ns_.c_str(), velocity_control_);
        // joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(hw_ns_ + "/joint_states", 100, std::bind(&XArmHW::_joint_states_callback, this, std::placeholders::_1));
        xarm_state_sub_ = node_->create_subscription<xarm_msgs::msg::RobotMsg>(hw_ns_ + "/xarm_states", 100, std::bind(&XArmHW::_xarm_states_callback, this, std::placeholders::_1));
        // trajectory_execution_event_sub_ = node_->create_subscription<std_msgs::msg::String>("trajectory_execution_event", 100, std::bind(&XArmHW::_receive_event, this, std::placeholders::_1));
        std::thread th([this]() -> void {
            rclcpp::spin(node_);
            rclcpp::shutdown();
        });
        th.detach();
        rclcpp::sleep_for(std::chrono::seconds(1));
        
        client_node_ = rclcpp::Node::make_shared("xarm_ros_client");
        xarm_client_.init(client_node_, hw_ns_);
        xarm_client_.motion_enable(true);
        if (velocity_control_)
            xarm_client_.set_mode(4);
        else
            xarm_client_.set_mode(1);
        xarm_client_.set_state(0);

        rclcpp::sleep_for(std::chrono::seconds(1));

        req_list_controller_ = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
        res_list_controller_ = std::make_shared<controller_manager_msgs::srv::ListControllers::Response>();
        req_switch_controller_ = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        res_switch_controller_ = std::make_shared<controller_manager_msgs::srv::SwitchController::Response>();

        client_list_controller_ = client_node_->create_client<controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers");
        client_switch_controller_ = client_node_->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");

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
        
        RCLCPP_INFO(node_->get_logger(), "[ns: %s] System Sucessfully activated!", node_->get_namespace());
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn XArmHW::on_deactivate(const rclcpp_lifecycle::State& previous_state)
    {
        RCLCPP_INFO(node_->get_logger(), "[ns: %s] Stopping ...please wait...", node_->get_namespace());

        xarm_client_.set_mode(0);

        RCLCPP_INFO(node_->get_logger(), "[ns: %s] System sucessfully deactivated!", node_->get_namespace());
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type XArmHW::read()
    {
        read_cnts_ += 1;
        read_ready_ = _xarm_is_ready_read();
        rclcpp::Time start = node_->get_clock()->now();
        read_code_ = xarm_client_.get_servo_angle(curr_read_angles_);
        read_code_ = 0;
        curr_read_time_ = node_->get_clock()->now();
        read_ready_ = read_ready_ && _xarm_is_ready_read();
        double time_sec = curr_read_time_.seconds() - start.seconds();
        read_total_time_ += time_sec;
        if (time_sec > read_max_time_) {
            read_max_time_ = time_sec;
        }
        // if (read_cnts_ % 6000 == 0) {
        //     RCLCPP_INFO(node_->get_logger(), "[READ] cnt: %ld, max: %f, mean: %f, failed: %ld", read_cnts_, read_max_time_, read_total_time_ / read_cnts_, read_failed_cnts_);
        // }
        if (read_code_ == 0 && read_ready_) {
            for (int j = 0; j < info_.joints.size(); j++) {
                position_states_[j] = curr_read_angles_[j];
                velocity_states_[j] = !initialized_ ? 0.0 : (curr_read_angles_[j] - prev_read_angles_[j]) / (curr_read_time_.seconds() - prev_read_time_.seconds());
            }
            if (!initialized_) {
                for (uint i = 0; i < position_states_.size(); i++) {
                    position_cmds_[i] = position_states_[i];
                    velocity_cmds_[i] = 0.0;
                }
                if (reload_controller_ && _check_cmds_is_change(prev_read_angles_, curr_read_angles_)) {
                    _reload_controller();
                }
            }
            prev_read_angles_.swap(curr_read_angles_);
            prev_read_time_ = curr_read_time_;
        }
        else {
            // initialized_ = read_ready_ && _xarm_is_ready_write();
            if (read_code_) {
                read_failed_cnts_ += 1;
                RCLCPP_INFO(node_->get_logger(), "[ns: %s, hw_ns: %s] xArmHW::Read() returns: %d", node_->get_namespace(), hw_ns_.c_str(), read_code_);
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type XArmHW::write()
    {
        if (_need_reset()) {
            if (initialized_) reload_controller_ = true;
            initialized_ = false;
            return hardware_interface::return_type::OK;
        }
        initialized_ = true;
        
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
                RCLCPP_WARN(node_->get_logger(), "[ns: %s, hw_ns: %s] vc_set_joint_velocity, ret=%d", node_->get_namespace(), hw_ns_.c_str(), ret);
            }
        }
        else {
            for (int i = 0; i < position_cmds_.size(); i++) { 
                cmds_float_[i] = (float)position_cmds_[i];
            }
            curr_write_time_ = node_->get_clock()->now();
            if (curr_write_time_.seconds() - prev_write_time_.seconds() > 1 || _check_cmds_is_change(prev_cmds_float_, cmds_float_)) {
                // RCLCPP_INFO(node_->get_logger(), "positon: %s", pos_str.c_str());
                int ret = xarm_client_.set_servo_angle_j(cmds_float_);
                if (ret != 0) {
                    RCLCPP_WARN(node_->get_logger(), "[ns: %s, hw_ns: %s] set_servo_angle_j, ret= %d", node_->get_namespace(), hw_ns_.c_str(), ret);
                }
                if (ret == 0) {
                    prev_write_time_ = curr_write_time_;
                    for (int i = 0; i < prev_cmds_float_.size(); i++) { 
                        prev_cmds_float_[i] = (float)cmds_float_[i];
                    }
                }
            }
        }

        return hardware_interface::return_type::OK;
    }

    void XArmHW::_reload_controller(void) {
        int ret = _call_request(client_list_controller_, req_list_controller_, res_list_controller_);
        if (ret == 0 && res_list_controller_->controller.size() > 0) {
            req_switch_controller_->start_controllers.resize(res_list_controller_->controller.size());
            req_switch_controller_->stop_controllers.resize(res_list_controller_->controller.size());
            for (uint i = 0; i < res_list_controller_->controller.size(); i++) {
                req_switch_controller_->start_controllers[i] = res_list_controller_->controller[i].name;
                req_switch_controller_->stop_controllers[i] = res_list_controller_->controller[i].name;
            }
            req_switch_controller_->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
            _call_request(client_switch_controller_, req_switch_controller_, res_switch_controller_);
        }
        if (ret == 0) {
            reload_controller_ = false;
        }
    }

    bool XArmHW::_check_cmds_is_change(std::vector<float> prev, std::vector<float> cur, double threshold)
    {
        for (int i = 0; i < cur.size(); i++) {
            if (std::abs(cur[i] - prev[i]) > threshold) return true;
        }
        return false;
    }

    bool XArmHW::_check_cmds_is_change(std::vector<double> prev, std::vector<double> cur, double threshold)
    {
        for (int i = 0; i < cur.size(); i++) {
            if (std::abs(cur[i] - prev[i]) > threshold) return true;
        }
        return false;
    }

    bool XArmHW::_xarm_is_ready_read(void)
    {
        static int last_err = curr_err_;
        if (curr_err_ != 0) {
            if (last_err != curr_err_) {
                RCLCPP_ERROR(node_->get_logger(), "[ns: %s, hw_ns: %s] xArm Error detected! Code: %d", node_->get_namespace(), hw_ns_.c_str(), curr_err_);
            }
        }
        last_err = curr_err_;
        return last_err == 0;
    }

    bool XArmHW::_xarm_is_ready_write(void)
    {
        static bool last_not_ready = false;
        static int last_state = curr_state_;
        static int last_mode = curr_mode_;

        if (!_xarm_is_ready_read()) {
            last_not_ready = true;
            return false;
        }

        if (curr_state_ > 2) {
            if (last_state != curr_state_) {
                last_state = curr_state_;
                RCLCPP_ERROR(node_->get_logger(), "[ns: %s, hw_ns: %s] xArm State detected! State: %d", node_->get_namespace(), hw_ns_.c_str(), curr_state_);
            }
            last_not_ready = true;
            return false;
        }
        last_state = curr_state_;

        if (!(velocity_control_ ? curr_mode_ == 4 : curr_mode_ == 1)) {
            if (last_mode != curr_mode_) {
                last_mode = curr_mode_;
                RCLCPP_ERROR(node_->get_logger(), "[ns: %s, hw_ns: %s] xArm Mode detected! Mode: %d", node_->get_namespace(), hw_ns_.c_str(), curr_mode_);
            }
            last_not_ready = true;
            return false;
        }
        last_mode = curr_mode_;

        if (last_not_ready) {
            RCLCPP_INFO(node_->get_logger(), "[ns: %s, hw_ns: %s] xArm is Ready", node_->get_namespace(), hw_ns_.c_str());
        }
        last_not_ready = false;
        return true;
    }

    bool XArmHW::_need_reset()
    {
        bool is_not_ready = !_xarm_is_ready_write();
        bool write_succeed = write_code_ == 0;
        if (!write_succeed) {
            int ret = xarm_client_.set_state(4);
            RCLCPP_ERROR(node_->get_logger(), "[ns: %s, hw_ns: %s] XArmHW::Write() failed, failed_ret=%d !, Setting Robot State to STOP... (ret: %d)", node_->get_namespace(), hw_ns_.c_str(), write_code_, ret);
            if (write_code_ == SERVICE_IS_PERSISTENT_BUT_INVALID || write_code_ == SERVICE_CALL_FAILED) {
                RCLCPP_ERROR(node_->get_logger(), "[ns: %s, hw_ns: %s] service is invaild, ros shutdown", node_->get_namespace(), hw_ns_.c_str());
                rclcpp::shutdown();
                exit(1);
            }
            else if (write_code_ == XARM_IS_DISCONNECTED) {
                RCLCPP_ERROR(node_->get_logger(), "[ns: %s, hw_ns: %s] xArm is disconnected, ros shutdown", node_->get_namespace(), hw_ns_.c_str());
                rclcpp::shutdown();
                exit(1);
            }
            write_code_ = 0;
        }
        return is_not_ready || !write_succeed || read_code_ != 0 || !read_ready_;
    }
}

