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

        bool velocity_control_set = false;
        it = info_.hardware_parameters.find("velocity_control");
        if (it != info_.hardware_parameters.end()) {
            velocity_control_set = (it->second == "True" || it->second == "true");
        }
        else{
            velocity_control_set = false; // parameter not set
        }
        if (!velocity_control_set) {
            RCLCPP_ERROR(LOGGER, "velocity_control is not set in config file. We currently only support velocity_control mode.");
            exit(1);
        }

        RCLCPP_INFO(LOGGER, "[%s] dof: %d, velocity_control_set: %d, add_gripper: %d, add_bio_gripper: %d, baud_checkset: %d, default_gripper_baud: %d", 
            robot_ip_.c_str(), dof, velocity_control_set, add_gripper, add_bio_gripper, baud_checkset, default_gripper_baud);
        
        xarm_driver_.init(node_, robot_ip_, true);
        joint_state_msg_ = xarm_driver_.get_joint_states();
    }

    CallbackReturn UFRobotSystemHardware::on_init(const hardware_interface::HardwareInfo& info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }
        info_ = info;
        read_code_ = 0;
        write_code_ = 0;

        initialized_ = false;

        read_cnts_ = 0;
        read_failed_cnts_ = 0;

        _init_ufactory_driver();
        
        position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        velocity_cmds_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        for (const hardware_interface::ComponentInfo & joint : info_.joints) {
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

        button_pressed_pub_ = node_->create_publisher<std_msgs::msg::Empty>(
            "button_pressed", rclcpp::SystemDefaultsQoS());
        button_released_pub_ = node_->create_publisher<std_msgs::msg::Empty>(
            "button_released", rclcpp::SystemDefaultsQoS());

        button_pressed_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                // copy the value of the atomic button_pressed_ to a local variable
                // to avoid race conditions
                bool current_button_pressed = button_pressed_;
                if (current_button_pressed != button_pressed_last_) {
                    button_pressed_last_ = current_button_pressed;
                    if (current_button_pressed) {
                        RCLCPP_INFO(LOGGER, "[%s] Button pressed!", robot_ip_.c_str());
                        std_msgs::msg::Empty msg;
                        button_pressed_pub_->publish(msg);
                    }
                    else{
                        RCLCPP_INFO(LOGGER, "[%s] Button released!", robot_ip_.c_str());
                        std_msgs::msg::Empty msg;
                        button_released_pub_->publish(msg);
                    }
                }
            });

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
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_cmds_[i]));
        }

        return command_interfaces;
    }

    CallbackReturn UFRobotSystemHardware::on_activate(const rclcpp_lifecycle::State& previous_state)
    {
        xarm_driver_.arm->motion_enable(true);
		xarm_driver_.arm->set_mode(XARM_MODE::VELO_JOINT);
		xarm_driver_.arm->set_state(XARM_STATE::START);

        // This section is probably not needed. When we activate the hardware, we can only get reasonable values
        // from the robot after doing the read() function.
        for (uint i = 0; i < velocity_states_.size(); i++) {
            velocity_cmds_[i] = 0.0;
        }

        initialized_ = false;
        
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

        bool use_new = _firmware_version_is_ge(1, 8, 103);
        if (!use_new){
            RCLCPP_ERROR(LOGGER, "Robot firmware version is lower than 1.8.103, please update the firmware to use new API");
            return hardware_interface::return_type::ERROR;
        }
		float curr_read_position[7];
		float curr_read_velocity[7];
		float curr_read_effort[7];
        rclcpp::Time start = node_->get_clock()->now();
		read_code_ = xarm_driver_.arm->get_joint_states(curr_read_position, curr_read_velocity, curr_read_effort);
        read_ready_ = read_ready_ && _xarm_is_ready_read();
        curr_read_time_ = node_->get_clock()->now();
        double time_sec = curr_read_time_.seconds() - start.seconds();
        read_total_time_ += time_sec;
        if (time_sec > read_max_time_) {
            read_max_time_ = time_sec;
        }
        if (read_code_ == 0 && read_ready_) {
            for (int j = 0; j < info_.joints.size(); j++) {
                position_states_[j] = curr_read_position[j];
				velocity_states_[j] = curr_read_velocity[j];

            }
            if (!initialized_) {
                for (uint i = 0; i < position_states_.size(); i++) {
                    velocity_cmds_[i] = 0.0;
                }
            }
            // 20250318, update joint_states msg and publish
            joint_state_msg_->header.stamp = curr_read_time_;
            for(int i = 0; i < joint_state_msg_->position.size(); i++)
            {
                joint_state_msg_->position[i] = position_states_[i];
                joint_state_msg_->velocity[i] = velocity_states_[i];
                if (use_new)
                    joint_state_msg_->effort[i] = (double)curr_read_effort[i];
            }
            xarm_driver_.pub_joint_state(*joint_state_msg_);

        }
        else {
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

        int digitals[8];
        int digitals2[8];
        xarm_driver_.arm->get_cgpio_digital(digitals, digitals2);
        button_pressed_ = digitals2[0] == 0;

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type UFRobotSystemHardware::write(const rclcpp::Time & time, const rclcpp::Duration &period)
    {
        if (_need_reset()) {
            RCLCPP_WARN_STREAM_THROTTLE(LOGGER, *get_clock(), 2000,
             "Robot arm is not in velocity control mode.");
            initialized_ = false;
            return hardware_interface::return_type::OK;
        }
        initialized_ = true;
        
		float cmds_float[7];
        int cmd_ret = 0;
        for (int i = 0; i < velocity_cmds_.size(); i++) { 
            if (std::isnan(velocity_cmds_[i])) {
                RCLCPP_ERROR(LOGGER, "[%s] velocity_cmds_[%d] is NaN", robot_ip_.c_str(), i);
                return hardware_interface::return_type::ERROR;
            }
            cmds_float[i] = (float)velocity_cmds_[i];
        }
        cmd_ret = xarm_driver_.arm->vc_set_joint_velocity(cmds_float, true, VELO_DURATION);
        if (cmd_ret != 0) {
            std::stringstream vel_commands;
            for (int i = 0; i < 7; i++) {
                vel_commands << cmds_float[i] << " ";
            }
            RCLCPP_WARN(LOGGER, "[%s] vc_set_joint_velocity, ret=%d, commands: %s", robot_ip_.c_str(), cmd_ret, vel_commands.str().c_str());
        }

        return hardware_interface::return_type::OK;
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

        if (curr_mode != XARM_MODE::VELO_JOINT) {
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

