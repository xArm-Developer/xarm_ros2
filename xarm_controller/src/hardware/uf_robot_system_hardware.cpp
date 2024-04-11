/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
           Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#include "xarm_controller/hardware/uf_robot_system_hardware.h"

#define SERVICE_CALL_FAILED 999
#define SERVICE_IS_PERSISTENT_BUT_INVALID 998
#define ROBOT_IS_DISCONNECTED -1
#define WAIT_SERVICE_TIMEOUT 996
#define VELO_DURATION 1

namespace uf_robot_hardware
{
    static rclcpp::Logger LOGGER = rclcpp::get_logger("UFACTORY.RobotHW");

    template<typename ServiceT, typename SharedRequest = typename ServiceT::Request::SharedPtr, typename SharedResponse = typename ServiceT::Response::SharedPtr>
    int UFRobotSystemHardware::_call_request(std::shared_ptr<ServiceT> client, SharedRequest req, SharedResponse& res)
    {
        bool is_try_again = false;
        int failed_cnts = 0;
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(LOGGER, "[%s] Interrupted while waiting for the service. Exiting.", robot_ip_.c_str());
                exit(1);
            }
            if (!is_try_again) {
                is_try_again = true;
                RCLCPP_WARN(LOGGER, "[%s] service %s not available, waiting ...", robot_ip_.c_str(), client->get_service_name());
            }
            failed_cnts += 1;
            if (failed_cnts >= 5) return WAIT_SERVICE_TIMEOUT;
        }
        auto result_future = client->async_send_request(req);
        if (rclcpp::spin_until_future_complete(hw_node_, result_future, std::chrono::seconds(1)) != rclcpp::FutureReturnCode::SUCCESS)
        {
            // RCLCPP_ERROR(LOGGER, "[%s] Failed to call service %s", robot_ip_.c_str(), client->get_service_name());
            return SERVICE_CALL_FAILED;
        }
        res = result_future.get();
        return 0;
    }

    void UFRobotSystemHardware::_init_ufactory_driver(void)
    {
        rclcpp::NodeOptions node_options;
        node_options.allow_undeclared_parameters(true);
        node_options.automatically_declare_parameters_from_overrides(true);
        node_ = rclcpp::Node::make_shared("ufactory_driver", node_options);
        hw_node_ = rclcpp::Node::make_shared("ufactory_robot_hw", node_options);

        std::thread th([this]() -> void {
            rclcpp::spin(node_);
            rclcpp::shutdown();
        });
        th.detach();

        robot_ip_ = "";
        auto it = info_.hardware_parameters.find("robot_ip");
        if (it != info_.hardware_parameters.end()) {
            robot_ip_ = it->second.substr(1);
        }
        if (robot_ip_ == "") {
            RCLCPP_ERROR(LOGGER, "[%s] No param named 'robot_ip'", robot_ip_.c_str());
            rclcpp::shutdown();
            exit(1);
        }

        std::string hw_ns = "xarm";
        it = info_.hardware_parameters.find("hw_ns");
        if (it != info_.hardware_parameters.end()) {
            hw_ns = it->second;
        }
        node_->set_parameter(rclcpp::Parameter("hw_ns", hw_ns));

        std::string prefix = "";
        it = info_.hardware_parameters.find("prefix");
        if (it != info_.hardware_parameters.end()) {
            prefix = it->second.substr(1);
        }
        if (prefix != "") {
            LOGGER = rclcpp::get_logger("UFACTORY." + prefix + "RobotHW");
        }
        node_->set_parameter(rclcpp::Parameter("prefix", prefix));

        std::string report_type = "normal";
        it = info_.hardware_parameters.find("report_type");
        if (it != info_.hardware_parameters.end()) {
            report_type = it->second;
        }
        node_->set_parameter(rclcpp::Parameter("report_type", report_type));

        std::string robot_type = "xarm";
        it = info_.hardware_parameters.find("robot_type");
        if (it != info_.hardware_parameters.end()) {
            robot_type = it->second;
        }

        RCLCPP_INFO(LOGGER, "[%s] namespace: %s", robot_ip_.c_str(), node_->get_namespace());
        RCLCPP_INFO(LOGGER, "[%s] robot_type: %s, hw_ns: %s, prefix: %s, report_type: %s", 
            robot_ip_.c_str(), robot_type.c_str(), hw_ns.c_str(), prefix.c_str(), report_type.c_str());

        int dof = 7;
        it = info_.hardware_parameters.find("dof");
        if (it != info_.hardware_parameters.end()) {
            dof = atoi(it->second.c_str());
        }
        node_->set_parameter(rclcpp::Parameter("dof", dof));

        int default_gripper_baud = 2000000;
        it = info_.hardware_parameters.find("default_gripper_baud");
        if (it != info_.hardware_parameters.end()) {
            default_gripper_baud = atoi(it->second.c_str());
        }
        node_->set_parameter(rclcpp::Parameter("default_gripper_baud", default_gripper_baud));

        bool baud_checkset = true;
        it = info_.hardware_parameters.find("baud_checkset");
        if (it != info_.hardware_parameters.end()) {
            baud_checkset = (it->second == "True" || it->second == "true");
        }
        node_->set_parameter(rclcpp::Parameter("baud_checkset", baud_checkset));

        bool add_gripper = true;
        it = info_.hardware_parameters.find("add_gripper");
        if (it != info_.hardware_parameters.end()) {
            add_gripper = (it->second == "True" || it->second == "true");
        }
        
        if (robot_type == "lite") add_gripper = false;
        node_->set_parameter(rclcpp::Parameter("add_gripper", add_gripper));

        bool add_bio_gripper = true;
        it = info_.hardware_parameters.find("add_bio_gripper");
        if (it != info_.hardware_parameters.end()) {
            add_bio_gripper = (it->second == "True" || it->second == "true");
        }
        
        if (robot_type == "lite") add_bio_gripper = false;
        node_->set_parameter(rclcpp::Parameter("add_bio_gripper", add_bio_gripper));

        it = info_.hardware_parameters.find("velocity_control");
        if (it != info_.hardware_parameters.end()) {
            velocity_control_ = (it->second == "True" || it->second == "true");
        }
        RCLCPP_INFO(LOGGER, "[%s] dof: %d, velocity_control: %d, add_gripper: %d, add_bio_gripper: %d, baud_checkset: %d, default_gripper_baud: %d", 
            robot_ip_.c_str(), dof, velocity_control_, add_gripper, add_bio_gripper, baud_checkset, default_gripper_baud);
        
        xarm_driver_.init(node_, robot_ip_);
    }

    CallbackReturn UFRobotSystemHardware::on_init(const hardware_interface::HardwareInfo& info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }
        info_ = info;
        velocity_control_ = false;
        read_code_ = 0;
        write_code_ = 0;

        initialized_ = false;
        reload_controller_ = false;

        read_cnts_ = 0;
        read_max_time_ = 0;
        read_total_time_ = 0;
        read_failed_cnts_ = 0;
        memset(cmds_float_, 0, sizeof(cmds_float_));
        memset(prev_cmds_float_, 0, sizeof(prev_cmds_float_));

        _init_ufactory_driver();
        
        position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        position_cmds_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        velocity_cmds_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        for (const hardware_interface::ComponentInfo & joint : info_.joints) {
            bool has_pos_cmd_interface = false;
            for (auto i = 0u; i < joint.command_interfaces.size(); ++i) {
                if (joint.command_interfaces[i].name == hardware_interface::HW_IF_POSITION) {
                    has_pos_cmd_interface = true;
                    break;
                }
            }
            if (!has_pos_cmd_interface) {
                RCLCPP_ERROR(LOGGER, "[%s] Joint '%s' has %ld command interfaces found, but not found %s command interface",
                    robot_ip_.c_str(), joint.name.c_str(), joint.command_interfaces.size(), hardware_interface::HW_IF_POSITION
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
                RCLCPP_ERROR(LOGGER, "[%s] Joint '%s' has %ld state interfaces found, but not found %s state interface",
                    robot_ip_.c_str(), joint.name.c_str(), joint.state_interfaces.size(), hardware_interface::HW_IF_POSITION
                );
                return CallbackReturn::ERROR;
            }
        }

        RCLCPP_INFO(LOGGER, "[%s] System Sucessfully configured!", robot_ip_.c_str());
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> UFRobotSystemHardware::export_state_interfaces()
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

    std::vector<hardware_interface::CommandInterface> UFRobotSystemHardware::export_command_interfaces()
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

    CallbackReturn UFRobotSystemHardware::on_activate(const rclcpp_lifecycle::State& previous_state)
    {
        xarm_driver_.arm->motion_enable(true);
		xarm_driver_.arm->set_mode(velocity_control_ ? XARM_MODE::VELO_JOINT : XARM_MODE::SERVO);
		xarm_driver_.arm->set_state(XARM_STATE::START);

        req_list_controller_ = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
        res_list_controller_ = std::make_shared<controller_manager_msgs::srv::ListControllers::Response>();
        req_switch_controller_ = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        res_switch_controller_ = std::make_shared<controller_manager_msgs::srv::SwitchController::Response>();

        client_list_controller_ = hw_node_->create_client<controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers");
        client_switch_controller_ = hw_node_->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");

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
        
        RCLCPP_INFO(LOGGER, "[%s] System Sucessfully started!", robot_ip_.c_str());
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn UFRobotSystemHardware::on_deactivate(const rclcpp_lifecycle::State& previous_state)
    {
        RCLCPP_INFO(LOGGER, "[%s] Stopping ...please wait...", robot_ip_.c_str());

        xarm_driver_.arm->set_mode(XARM_MODE::POSE);

        RCLCPP_INFO(LOGGER, "[%s] System sucessfully stopped!", robot_ip_.c_str());
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type UFRobotSystemHardware::read(const rclcpp::Time & time, const rclcpp::Duration &period)
    {
        read_cnts_ += 1;
        read_ready_ = _xarm_is_ready_read();
        rclcpp::Time start = node_->get_clock()->now();

        bool use_new = _firmware_version_is_ge(1, 8, 103);
        if (use_new)
			read_code_ = xarm_driver_.arm->get_joint_states(curr_read_position_, curr_read_velocity_, curr_read_effort_);
		else
			read_code_ = xarm_driver_.arm->get_servo_angle(curr_read_position_);
        
        curr_read_time_ = node_->get_clock()->now();
        read_ready_ = read_ready_ && _xarm_is_ready_read();
        double time_sec = curr_read_time_.seconds() - start.seconds();
        read_total_time_ += time_sec;
        if (time_sec > read_max_time_) {
            read_max_time_ = time_sec;
        }
        // if (read_cnts_ % 6000 == 0) {
        //     RCLCPP_INFO(LOGGER, "[%s] [READ] cnt: %ld, max: %f, mean: %f, failed: %ld", robot_ip_.c_str(), read_cnts_, read_max_time_, read_total_time_ / read_cnts_, read_failed_cnts_);
        // }
        if (read_code_ == 0 && read_ready_) {
            for (int j = 0; j < info_.joints.size(); j++) {
                position_states_[j] = curr_read_position_[j];
				if (use_new) {
					velocity_states_[j] = curr_read_velocity_[j];
					// effort_states_[j] = curr_read_effort_[j];
				}
				else {
					velocity_states_[j] = !initialized_ ? 0.0 : (curr_read_position_[j] - prev_read_position_[j]) / (curr_read_time_.seconds() - prev_read_time_.seconds());
					// effort_states_[j] = 0.0;
				}
            }
            if (!initialized_) {
                for (uint i = 0; i < position_states_.size(); i++) {
                    position_cmds_[i] = position_states_[i];
                    velocity_cmds_[i] = 0.0;
                }
                if (reload_controller_ && _check_cmds_is_change(curr_read_position_, prev_read_position_)) {
                    _reload_controller();
                }
            }
            memcpy(prev_read_position_, curr_read_position_, sizeof(float) * 7);
            prev_read_time_ = curr_read_time_;
        }
        else {
            // initialized_ = read_ready_ && _xarm_is_ready_write();
            if (read_code_) {
                read_failed_cnts_ += 1;
                RCLCPP_INFO(LOGGER, "[%s] Read() returns: %d", robot_ip_.c_str(), read_code_);
                if (read_code_ == ROBOT_IS_DISCONNECTED) {
                    RCLCPP_ERROR(LOGGER, "[%s] Robot is disconnected, ros shutdown", robot_ip_.c_str());
                    rclcpp::shutdown();
                    exit(1);
				}
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type UFRobotSystemHardware::write(const rclcpp::Time & time, const rclcpp::Duration &period)
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
        // RCLCPP_INFO(LOGGER, "[%s] positon: %s, velocity: %s", robot_ip_.c_str(), pos_str.c_str(), vel_str.c_str());

        int cmd_ret = 0;
        if (velocity_control_) {
            for (int i = 0; i < velocity_cmds_.size(); i++) { 
                cmds_float_[i] = (float)velocity_cmds_[i];
            }
            // RCLCPP_INFO(LOGGER, "[%s] velocity: %s", robot_ip_.c_str(), vel_str.c_str());
            cmd_ret = xarm_driver_.arm->vc_set_joint_velocity(cmds_float_, true, VELO_DURATION);
            if (cmd_ret != 0) {
                RCLCPP_WARN(LOGGER, "[%s] vc_set_joint_velocity, ret=%d", robot_ip_.c_str(), cmd_ret);
            }
        }
        else {
            for (int i = 0; i < position_cmds_.size(); i++) { 
                cmds_float_[i] = (float)position_cmds_[i];
            }
            curr_write_time_ = node_->get_clock()->now();
            if (curr_write_time_.seconds() - prev_write_time_.seconds() > 1 || _check_cmds_is_change(prev_cmds_float_, cmds_float_)) {
                // RCLCPP_INFO(LOGGER, "[%s] positon: %s", robot_ip_.c_str(), pos_str.c_str());
                cmd_ret = xarm_driver_.arm->set_servo_angle_j(cmds_float_, 0, 0, 0);
                if (cmd_ret != 0) {
                    RCLCPP_WARN(LOGGER, "[%s] set_servo_angle_j, ret= %d", robot_ip_.c_str(), cmd_ret);
                }
                if (cmd_ret == 0) {
                    prev_write_time_ = curr_write_time_;
                    for (int i = 0; i < 7; i++) { 
                        prev_cmds_float_[i] = (float)cmds_float_[i];
                    }
                }
            }
        }

        return hardware_interface::return_type::OK;
    }

    void UFRobotSystemHardware::_reload_controller(void) {
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

    bool UFRobotSystemHardware::_check_cmds_is_change(float *prev, float *cur, double threshold)
	{
		for (int i = 0; i < 7; i++) {
            if (std::abs(cur[i] - prev[i]) > threshold) return true;
        }
        return false;
	}

    bool UFRobotSystemHardware::_xarm_is_ready_read(void)
    {
        static int last_err = xarm_driver_.curr_err;
		int curr_err = xarm_driver_.curr_err;
        if (curr_err != 0) {
            if (last_err != curr_err) {
                RCLCPP_ERROR(LOGGER, "[%s] UFACTORY Error detected! Code C%d -> [ %s ] ", robot_ip_.c_str(), curr_err, xarm_driver_.controller_error_interpreter(curr_err).c_str());
            }
        }
        last_err = curr_err;
        return last_err == 0;
    }

    bool UFRobotSystemHardware::_xarm_is_ready_write(void)
    {
        static bool last_not_ready = false;
        static int last_state = xarm_driver_.curr_state;
        static int last_mode = xarm_driver_.curr_mode;
        int curr_mode = xarm_driver_.curr_mode;
		int curr_state = xarm_driver_.curr_state;

        if (!_xarm_is_ready_read()) {
            last_not_ready = true;
            return false;
        }

        if (curr_state > 2) {
            if (last_state != curr_state) {
                last_state = curr_state;
                RCLCPP_WARN(LOGGER, "[%s] Robot State detected! State: %d", robot_ip_.c_str(), curr_state);
            }
            last_not_ready = true;
            return false;
        }
        last_state = curr_state;

        if (!(velocity_control_ ? curr_mode == XARM_MODE::VELO_JOINT : curr_mode == XARM_MODE::SERVO)) {
            if (last_mode != curr_mode) {
                last_mode = curr_mode;
                RCLCPP_WARN(LOGGER, "[%s] Robot Mode detected! Mode: %d", robot_ip_.c_str(), curr_mode);
            }
            last_not_ready = true;
            return false;
        }
        last_mode = curr_mode;

        if (last_not_ready) {
            RCLCPP_INFO(LOGGER, "[%s] Robot is Ready", robot_ip_.c_str());
        }
        last_not_ready = false;
        return true;
    }

    bool UFRobotSystemHardware::_firmware_version_is_ge(int major, int minor, int revision)
	{
		return xarm_driver_.arm->version_number[0] > major || (xarm_driver_.arm->version_number[0] == major && xarm_driver_.arm->version_number[1] > minor) || (xarm_driver_.arm->version_number[0] == major && xarm_driver_.arm->version_number[1] == minor && xarm_driver_.arm->version_number[2] >= revision);
	}

    bool UFRobotSystemHardware::_need_reset()
    {
        bool is_not_ready = !_xarm_is_ready_write();
        bool write_succeed = write_code_ == 0;
        if (!write_succeed) {
            // int ret = xarm_driver_.arm->set_state(XARM_STATE::STOP);
            // RCLCPP_ERROR(LOGGER, "[%s] Write() failed, failed_ret=%d !, Setting Robot State to STOP... (ret: %d)", robot_ip_.c_str(), write_code_, ret);
            RCLCPP_ERROR(LOGGER, "[%s] Write() failed, failed_ret=%d !", robot_ip_.c_str(), write_code_);
            if (write_code_ == SERVICE_IS_PERSISTENT_BUT_INVALID || write_code_ == SERVICE_CALL_FAILED) {
                RCLCPP_ERROR(LOGGER, "[%s] Service is invaild, ros shutdown", robot_ip_.c_str());
                rclcpp::shutdown();
                exit(1);
            }
            else if (write_code_ == ROBOT_IS_DISCONNECTED) {
                RCLCPP_ERROR(LOGGER, "[%s] Robot is disconnected, ros shutdown", robot_ip_.c_str());
                rclcpp::shutdown();
                exit(1);
            }
            write_code_ = 0;
        }
        return is_not_ready || !write_succeed || read_code_ != 0 || !read_ready_;
    }
}

