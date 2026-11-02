/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
           Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#include <algorithm>
#include <cmath>
#include <sstream>
#include <thread>

#include "xarm_controller/hardware/uf_robot_system_hardware.h"

#define SERVICE_CALL_FAILED 999
#define SERVICE_IS_PERSISTENT_BUT_INVALID 998
#define ROBOT_IS_DISCONNECTED -1
#define WAIT_SERVICE_TIMEOUT 996
#define VELO_DURATION 1

#define DEG_TO_RAD (M_PI / 180.0)

namespace uf_robot_hardware
{
    static rclcpp::Logger LOGGER = rclcpp::get_logger("UFACTORY.RobotHW");

    // Order of the Cartesian gpio interfaces, and of the force/torque sensor interfaces.
    // Interfaces are bound by name rather than by position so the URDF may list them
    // in any order.
    static const char *TCP_IF_NAMES[6] = { "x", "y", "z", "roll", "pitch", "yaw" };
    // Optional seventh command interface on the same gpio: the set_position() speed for
    // the next motion, mm/s. No state counterpart -- the arm reports no speed setpoint
    // back, only what it was last told.
    static const char *TCP_SPEED_IF_NAME = "speed";
    static const char *TCP_SERVO_IF_NAME = "servo";
    static const char *TCP_FAULT_IF_NAME = "fault";
    static const char *FT_IF_NAMES[6] = {
        "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"
    };

    // Runtime force control interfaces. Vectors are flattened one interface per
    // element because ros2_control command interfaces are scalar doubles.
    enum { FT_CMD_ENABLE = 0, FT_CMD_COORD = 1, FT_CMD_C_AXIS = 2, FT_CMD_F_REF = 8, FT_CMD_COUNT = 14 };
    static const char *FT_CMD_NAMES[FT_CMD_COUNT] = {
        "enable", "coord",
        "c_axis_0", "c_axis_1", "c_axis_2", "c_axis_3", "c_axis_4", "c_axis_5",
        "f_ref_0", "f_ref_1", "f_ref_2", "f_ref_3", "f_ref_4", "f_ref_5"
    };
    enum { FT_CFG_ARMED = 0, FT_CFG_MODE = 1, FT_CFG_ERROR = 2,
           FT_CFG_FORCE = 3, FT_CFG_COUNT = 6 };
    static const char *FT_CFG_STATE_NAMES[FT_CFG_COUNT] = {
        // enable is what we asked for, mode is what the controller confirms. They only
        // disagree if a mode change was rejected, in which case believe mode.
        "enable", "mode", "error",
        // Measured end effector force, N. Mirrors the first three ft_sensor state
        // interfaces so the loop's target and what it is actually feeling can be read
        // from one topic. The full wrench, torques included, is on the broadcaster.
        "force_x", "force_y", "force_z"
    };

    static int _index_of_n(const char * const *names, int n, const std::string& name)
    {
        for (int i = 0; i < n; i++) {
            if (name == names[i]) return i;
        }
        return -1;
    }

    static int _index_of(const char *names[6], const std::string& name)
    {
        for (int i = 0; i < 6; i++) {
            if (name == names[i]) return i;
        }
        return -1;
    }

    bool UFRobotSystemHardware::_parse_vecn(const std::string& str, float *out, int n)
    {
        std::istringstream iss(str);
        std::vector<float> vals;
        float val;
        while (iss >> val) vals.push_back(val);
        if ((int)vals.size() != n) return false;
        for (int i = 0; i < n; i++) out[i] = vals[i];
        return true;
    }

    void UFRobotSystemHardware::_init_ft_params(void)
    {
        // Human facing params are mm and degrees, matching the UFACTORY UI. Everything
        // is converted here so the rest of the class only deals with the units the SDK
        // wants (mm and radians, since XArmAPI is built with is_radian=true).
        // Defaults are the values validated on the real UF850: 8 N along tool Z with
        // 8 Nm about tool Y, tool frame, sensor zeroed at activation.
        // The force loop is off at launch: arming it is an explicit act, through the ft
        // gpio command interfaces or ft_sensor_mode:=2. The setpoints below are the
        // values validated on the real UF850, ready for whenever it is armed.
        has_ft_sensor_ = true;
        ft_sensor_mode_ = 0;
        ft_coord_ = 0;
        ft_zero_on_activate_ = true;
        home_on_activate_ = true;
        has_home_pose_ = false;
        has_home_joints_ = false;
        home_speed_ = 100.0;
        home_acc_ = 1000.0;
        home_joint_speed_ = (float)(20.0 * DEG_TO_RAD);
        home_joint_acc_ = (float)(200.0 * DEG_TO_RAD);
        tcp_speed_ = 200.0;
        tcp_acc_ = 2000.0;
        tcp_radius_ = -1.0;
        cmd_queue_max_ = 16;
        memset(home_pose_, 0, sizeof(home_pose_));
        memset(home_joints_, 0, sizeof(home_joints_));

        for (int i = 0; i < 6; i++) {
            ft_c_axis_[i] = 0;
            ft_f_ref_[i] = 0;
            ft_limits_[i] = 0;
            ft_kp_[i] = 0.005;
            ft_ki_[i] = 0.00005;
            ft_kd_[i] = 0.05;
            ft_xe_limit_[i] = (i < 3) ? 200.0 : 0.1;
        }
        ft_c_axis_[0] = 1;
        ft_c_axis_[4] = 1;
        ft_f_ref_[0] = 8.0;

        auto str_param = [this](const std::string& name, std::string& out) -> bool {
            auto it = info_.hardware_parameters.find(name);
            if (it == info_.hardware_parameters.end()) return false;
            out = it->second;
            return !out.empty();
        };
        auto int_param = [&str_param](const std::string& name, int& out) -> void {
            std::string str;
            if (str_param(name, str)) out = atoi(str.c_str());
        };
        auto float_param = [&str_param](const std::string& name, float& out) -> void {
            std::string str;
            if (str_param(name, str)) out = (float)atof(str.c_str());
        };
        auto bool_param = [&str_param](const std::string& name, bool& out) -> void {
            std::string str;
            if (str_param(name, str)) out = (str == "True" || str == "true" || str == "1");
        };
        // Vectors arrive as a space separated string, the same idiom the existing
        // attach_xyz / geometry_mesh_origin_xyz params use.
        auto vec6_param = [this, &str_param](const std::string& name, float *out) -> void {
            std::string str;
            if (!str_param(name, str)) return;
            float tmp[6];
            if (_parse_vecn(str, tmp, 6)) {
                for (int i = 0; i < 6; i++) out[i] = tmp[i];
            }
            else {
                RCLCPP_WARN(LOGGER, "[%s] param '%s' needs 6 space separated values, got '%s', keeping default",
                    robot_ip_.c_str(), name.c_str(), str.c_str());
            }
        };

        bool_param("has_ft_sensor", has_ft_sensor_);
        int_param("ft_sensor_mode", ft_sensor_mode_);
        int_param("ft_coord", ft_coord_);
        int_param("cmd_queue_max", cmd_queue_max_);
        bool_param("ft_zero_on_activate", ft_zero_on_activate_);
        bool_param("home_on_activate", home_on_activate_);
        float_param("home_speed", home_speed_);
        float_param("home_acc", home_acc_);
        float_param("tcp_speed", tcp_speed_);
        float_param("tcp_acc", tcp_acc_);
        float_param("tcp_radius", tcp_radius_);

        vec6_param("ft_f_ref", ft_f_ref_);
        vec6_param("ft_limits", ft_limits_);
        vec6_param("ft_kp", ft_kp_);
        vec6_param("ft_ki", ft_ki_);
        vec6_param("ft_kd", ft_kd_);
        vec6_param("ft_xe_limit", ft_xe_limit_);

        float c_axis[6];
        for (int i = 0; i < 6; i++) c_axis[i] = (float)ft_c_axis_[i];
        vec6_param("ft_c_axis", c_axis);
        for (int i = 0; i < 6; i++) ft_c_axis_[i] = c_axis[i] != 0 ? 1 : 0;

        // home_joint_speed / home_joint_acc are given in deg/s and deg/s^2
        float deg = (float)(home_joint_speed_ / DEG_TO_RAD);
        float_param("home_joint_speed", deg);
        home_joint_speed_ = (float)(deg * DEG_TO_RAD);
        deg = (float)(home_joint_acc_ / DEG_TO_RAD);
        float_param("home_joint_acc", deg);
        home_joint_acc_ = (float)(deg * DEG_TO_RAD);

        std::string str;
        if (str_param("home_pose", str)) {
            if (_parse_vecn(str, home_pose_, 6)) {
                for (int i = 3; i < 6; i++) home_pose_[i] = (float)(home_pose_[i] * DEG_TO_RAD);
                has_home_pose_ = true;
            }
            else {
                RCLCPP_ERROR(LOGGER, "[%s] param 'home_pose' needs 6 space separated values (x y z mm, roll pitch yaw deg), got '%s'",
                    robot_ip_.c_str(), str.c_str());
            }
        }
        if (!has_home_pose_ && str_param("home_joints", str)) {
            int dof = (int)info_.joints.size();
            if (_parse_vecn(str, home_joints_, dof)) {
                for (int i = 0; i < dof; i++) home_joints_[i] = (float)(home_joints_[i] * DEG_TO_RAD);
                has_home_joints_ = true;
            }
            else {
                RCLCPP_ERROR(LOGGER, "[%s] param 'home_joints' needs %d space separated values in deg, got '%s'",
                    robot_ip_.c_str(), dof, str.c_str());
            }
        }

        if (!has_ft_sensor_) {
            // No sensor to arm, override whatever ft_sensor_mode was otherwise given.
            ft_sensor_mode_ = 0;
        }

        RCLCPP_INFO(LOGGER, "[%s] has_ft_sensor: %d, ft_sensor_mode: %d, ft_coord: %d, ft_zero_on_activate: %d, cmd_queue_max: %d",
            robot_ip_.c_str(), has_ft_sensor_, ft_sensor_mode_, ft_coord_, ft_zero_on_activate_, cmd_queue_max_);
        RCLCPP_INFO(LOGGER, "[%s] ft_c_axis: [%d %d %d %d %d %d], ft_f_ref: [%.3f %.3f %.3f %.3f %.3f %.3f]",
            robot_ip_.c_str(), ft_c_axis_[0], ft_c_axis_[1], ft_c_axis_[2], ft_c_axis_[3], ft_c_axis_[4], ft_c_axis_[5],
            ft_f_ref_[0], ft_f_ref_[1], ft_f_ref_[2], ft_f_ref_[3], ft_f_ref_[4], ft_f_ref_[5]);
        RCLCPP_INFO(LOGGER, "[%s] ft_xe_limit: [%.3f %.3f %.3f %.3f %.3f %.3f]",
            robot_ip_.c_str(), ft_xe_limit_[0], ft_xe_limit_[1], ft_xe_limit_[2],
            ft_xe_limit_[3], ft_xe_limit_[4], ft_xe_limit_[5]);
        RCLCPP_INFO(LOGGER, "[%s] home_on_activate: %d, has_home_pose: %d, has_home_joints: %d",
            robot_ip_.c_str(), home_on_activate_, has_home_pose_, has_home_joints_);
        RCLCPP_INFO(LOGGER, "[%s] tcp_speed: %.1f mm/s (fallback; a '%s' command interface overrides it per motion), tcp_acc: %.1f mm/s^2, tcp_radius: %.1f mm",
            robot_ip_.c_str(), tcp_speed_, TCP_SPEED_IF_NAME, tcp_acc_, tcp_radius_);
    }

    template<typename ServiceT, typename SharedRequest, typename SharedResponse>
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

        update_goal_state_pub_ = hw_node_->create_publisher<std_msgs::msg::Empty>("/rviz/moveit/update_goal_state", 1);

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
        it = info_.hardware_parameters.find("cartesian_servo_mode");
        if (it != info_.hardware_parameters.end()) {
            cartesian_servo_mode_ = (it->second == "True" || it->second == "true");
        }
        RCLCPP_INFO(LOGGER, "[%s] dof: %d, velocity_control: %d, cartesian_servo_mode: %d, add_gripper: %d, add_bio_gripper: %d, baud_checkset: %d, default_gripper_baud: %d",
            robot_ip_.c_str(), dof, velocity_control_, cartesian_servo_mode_, add_gripper, add_bio_gripper, baud_checkset, default_gripper_baud);

        // 20250318, disable xarm_driver publish joint_states
        xarm_driver_.init(node_, robot_ip_, true);
        // 20250318, get joint_states msg reference from xarm_driver
        joint_state_msg_ = xarm_driver_.get_joint_states();
    }

    CallbackReturn UFRobotSystemHardware::on_init(const hardware_interface::HardwareInfo& info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }
        info_ = info;
        velocity_control_ = false;
        cartesian_servo_mode_ = false;
        read_code_ = 0;
        write_code_ = 0;

        initialized_ = false;
        reactivate_controller_later_ = false;

        read_cnts_ = 0;
        read_max_time_ = 0;
        read_total_time_ = 0;
        read_failed_cnts_ = 0;
        memset(cmds_float_, 0, sizeof(cmds_float_));
        memset(prev_cmds_float_, 0, sizeof(prev_cmds_float_));

        _init_ufactory_driver();
        _init_ft_params();

        position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        position_cmds_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        velocity_cmds_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        tcp_states_.resize(6, std::numeric_limits<double>::quiet_NaN());
        tcp_cmds_.resize(6, std::numeric_limits<double>::quiet_NaN());
        prev_tcp_cmds_.resize(6, std::numeric_limits<double>::quiet_NaN());
        // 0 rather than NaN: this one is read on every send, and "nobody has written a
        // speed" has to mean the tcp_speed param, not a refused motion.
        tcp_speed_cmd_ = 0.0;
        servo_cmd_ = std::numeric_limits<double>::quiet_NaN();
        servo_state_ = cartesian_servo_mode_ ? 1.0 : 0.0;
        // NaN until the first read() has said otherwise: no answer is not "no fault".
        fault_state_ = std::numeric_limits<double>::quiet_NaN();
        ft_states_.resize(6, 0.0);
        ft_cmds_.resize(FT_CMD_COUNT, std::numeric_limits<double>::quiet_NaN());
        ft_cfg_states_.resize(FT_CFG_COUNT, 0.0);
        ft_armed_ = false;
        memset(tcp_standoff_, 0, sizeof(tcp_standoff_));

        has_ft_gpio_ = false;
        ft_iface_prefix_ = std::string();

        // Joints are state only. The arm stays in XARM_MODE::POSE and is commanded
        // through the Cartesian gpio component, so joints carry no command interface.
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

        if (info_.gpios.size() < 1) {
            RCLCPP_ERROR(LOGGER, "[%s] Expected a gpio component carrying the Cartesian setpoint, found none",
                robot_ip_.c_str());
            return CallbackReturn::ERROR;
        }
        bool has_pose_cmd[6] = { false, false, false, false, false, false };
        for (const auto & iface : info_.gpios[0].command_interfaces) {
            int idx = _index_of(TCP_IF_NAMES, iface.name);
            if (idx >= 0) {
                has_pose_cmd[idx] = true;
                continue;
            }
            if (iface.name == TCP_SPEED_IF_NAME || iface.name == TCP_SERVO_IF_NAME) continue;
            RCLCPP_ERROR(LOGGER, "[%s] gpio '%s' has unexpected command interface '%s', expected one of x y z roll pitch yaw speed servo",
                robot_ip_.c_str(), info_.gpios[0].name.c_str(), iface.name.c_str());
            return CallbackReturn::ERROR;
        }
        for (int i = 0; i < 6; i++) {
            if (!has_pose_cmd[i]) {
                RCLCPP_ERROR(LOGGER, "[%s] gpio '%s' is missing the '%s' command interface; all six of x y z roll pitch yaw are required",
                    robot_ip_.c_str(), info_.gpios[0].name.c_str(), TCP_IF_NAMES[i]);
                return CallbackReturn::ERROR;
            }
        }

        // The ft gpio component is optional: without it the force loop is still
        // configurable from the launch params, just not at runtime.
        has_ft_gpio_ = info_.gpios.size() > 1;
        ft_iface_prefix_ = has_ft_gpio_ ? info_.gpios[1].name + "/" : std::string();
        if (has_ft_gpio_) {
            for (const auto & iface : info_.gpios[1].command_interfaces) {
                if (_index_of_n(FT_CMD_NAMES, FT_CMD_COUNT, iface.name) < 0) {
                    RCLCPP_ERROR(LOGGER, "[%s] gpio '%s' has unexpected command interface '%s'",
                        robot_ip_.c_str(), info_.gpios[1].name.c_str(), iface.name.c_str());
                    return CallbackReturn::ERROR;
                }
            }
            for (const auto & iface : info_.gpios[1].state_interfaces) {
                if (_index_of_n(FT_CFG_STATE_NAMES, FT_CFG_COUNT, iface.name) < 0) {
                    RCLCPP_ERROR(LOGGER, "[%s] gpio '%s' has unexpected state interface '%s'",
                        robot_ip_.c_str(), info_.gpios[1].name.c_str(), iface.name.c_str());
                    return CallbackReturn::ERROR;
                }
            }
        }

        if (info_.sensors.size() < 1 || info_.sensors[0].state_interfaces.size() != 6) {
            RCLCPP_ERROR(LOGGER, "[%s] Expected a sensor component with 6 state interfaces (force.x .. torque.z), found %ld sensor component(s)",
                robot_ip_.c_str(), info_.sensors.size());
            return CallbackReturn::ERROR;
        }
        for (const auto & iface : info_.sensors[0].state_interfaces) {
            if (_index_of(FT_IF_NAMES, iface.name) < 0) {
                RCLCPP_ERROR(LOGGER, "[%s] sensor '%s' has unexpected state interface '%s', expected one of force.x force.y force.z torque.x torque.y torque.z",
                    robot_ip_.c_str(), info_.sensors[0].name.c_str(), iface.name.c_str());
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

        // Measured TCP pose, m and rad. Interfaces are bound by name so the URDF may
        // list x y z roll pitch yaw in any order.
        for (const auto & iface : info_.gpios[0].state_interfaces) {
            if (iface.name == TCP_SERVO_IF_NAME) {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.gpios[0].name, iface.name, &servo_state_));
                continue;
            }
            if (iface.name == TCP_FAULT_IF_NAME) {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.gpios[0].name, iface.name, &fault_state_));
                continue;
            }
            int idx = _index_of(TCP_IF_NAMES, iface.name);
            if (idx < 0) continue;
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.gpios[0].name, iface.name, &tcp_states_[idx]));
        }

        if (has_ft_gpio_) {
            for (const auto & iface : info_.gpios[1].state_interfaces) {
                int idx = _index_of_n(FT_CFG_STATE_NAMES, FT_CFG_COUNT, iface.name);
                if (idx < 0) continue;
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.gpios[1].name, iface.name, &ft_cfg_states_[idx]));
            }
        }

        // End effector wrench, N and Nm. The names force.x .. torque.z are what
        // semantic_components::ForceTorqueSensor expects, so
        // force_torque_sensor_broadcaster can claim them directly.
        for (const auto & iface : info_.sensors[0].state_interfaces) {
            int idx = _index_of(FT_IF_NAMES, iface.name);
            if (idx < 0) continue;
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.sensors[0].name, iface.name, &ft_states_[idx]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> UFRobotSystemHardware::export_command_interfaces()
    {
        // Cartesian only. In XARM_MODE::POSE the arm takes queued planned motions
        // (set_position), not the per cycle joint stream a trajectory controller emits,
        // so no joint command interfaces are exported.
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (const auto & iface : info_.gpios[0].command_interfaces) {
            if (iface.name == TCP_SPEED_IF_NAME) {
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    info_.gpios[0].name, iface.name, &tcp_speed_cmd_));
                continue;
            }
            if (iface.name == TCP_SERVO_IF_NAME) {
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    info_.gpios[0].name, iface.name, &servo_cmd_));
                continue;
            }
            int idx = _index_of(TCP_IF_NAMES, iface.name);
            if (idx < 0) continue;
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.gpios[0].name, iface.name, &tcp_cmds_[idx]));
        }

        if (has_ft_gpio_) {
            for (const auto & iface : info_.gpios[1].command_interfaces) {
                int idx = _index_of_n(FT_CMD_NAMES, FT_CMD_COUNT, iface.name);
                if (idx < 0) continue;
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    info_.gpios[1].name, iface.name, &ft_cmds_[idx]));
            }
        }

        return command_interfaces;
    }

    void UFRobotSystemHardware::_refresh_ft_cfg_states(void)
    {
        ft_cfg_states_[FT_CFG_ARMED] = ft_armed_ ? 1.0 : 0.0;
        if (!has_ft_sensor_) {
            // Nothing to query -- leave mode/error at 0 rather than whatever the SDK
            // returns for a sensor that was never enabled.
            ft_cfg_states_[FT_CFG_MODE] = 0;
            ft_cfg_states_[FT_CFG_ERROR] = 0;
            return;
        }
        int ft_mode = -1, ft_is_started = -1, ft_err = -1;
        xarm_driver_.arm->get_ft_sensor_config(&ft_mode, &ft_is_started);
        xarm_driver_.arm->get_ft_sensor_error(&ft_err);
        ft_cfg_states_[FT_CFG_MODE] = ft_mode;
        ft_cfg_states_[FT_CFG_ERROR] = ft_err;
        // FT_CFG_FORCE.. are refreshed every read(), leave them alone here.
    }

    bool UFRobotSystemHardware::_idle_for_mode_switch(void)
    {
        // set_mode() drops whatever is queued or moving, so only switch at rest.
        if (xarm_driver_.arm->cmd_num != 0 || xarm_driver_.curr_state == 1) return false;
        for (double v : velocity_states_) {
            if (std::abs(v) > 0.01) return false;
        }
        return true;
    }

    // True while a switch is in flight or the arm is still reporting the old mode, so
    // write() skips motion for the cycle.
    bool UFRobotSystemHardware::_apply_servo_command(void)
    {
        rclcpp::Time now = node_->get_clock()->now();

        if (servo_switch_future_.valid()) {
            if (servo_switch_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
                return true;
            }
            if (servo_switch_future_.get()) {
                cartesian_servo_mode_ = servo_switch_target_;
                // Resend the held command in the new mode on the next write.
                for (int i = 0; i < 6; i++) prev_tcp_cmds_[i] = std::numeric_limits<double>::quiet_NaN();
                prev_write_time_ = now;
                servo_settle_until_ = now + rclcpp::Duration::from_seconds(2.0);
                RCLCPP_INFO(LOGGER, "[%s] Switched to XARM_MODE::%s",
                    robot_ip_.c_str(), cartesian_servo_mode_ ? "SERVO" : "POSE");
            }
        }

        // The reported mode lags set_mode() by a few cycles.
        int expected_mode = cartesian_servo_mode_ ? XARM_MODE::SERVO : XARM_MODE::POSE;
        if (now < servo_settle_until_ && xarm_driver_.curr_mode != expected_mode) return true;

        if (std::isnan(servo_cmd_)) return false;
        bool want_servo = servo_cmd_ >= 0.5;
        // Compare against what the arm reports, not what we last asked for: a fault can drop
        // it back to POSE on its own.
        bool arm_in_servo = xarm_driver_.curr_mode == XARM_MODE::SERVO;
        if (want_servo == arm_in_servo) {
            cartesian_servo_mode_ = want_servo;
            return false;
        }

        if (now.seconds() - prev_servo_switch_time_.seconds() <= 1.0) return false;
        prev_servo_switch_time_ = now;

        if (want_servo && ft_armed_) {
            RCLCPP_WARN(LOGGER, "[%s] Not switching to servo while force control is armed",
                robot_ip_.c_str());
            return false;
        }
        if (!_idle_for_mode_switch()) {
            RCLCPP_INFO(LOGGER, "[%s] Waiting for the arm to stop before switching mode",
                robot_ip_.c_str());
            return false;
        }

        // set_mode() blocks for about a second, too long for the control loop.
        servo_switch_target_ = want_servo;
        servo_switch_future_ = std::async(std::launch::async, [this, want_servo]() {
            int mode_ret = xarm_driver_.arm->set_mode(want_servo ? XARM_MODE::SERVO : XARM_MODE::POSE);
            int state_ret = xarm_driver_.arm->set_state(XARM_STATE::START);
            if (mode_ret != 0 || state_ret != 0) {
                RCLCPP_ERROR(LOGGER, "[%s] Mode switch to %s failed, set_mode=%d set_state=%d",
                    robot_ip_.c_str(), want_servo ? "SERVO" : "POSE", mode_ret, state_ret);
                return false;
            }
            return true;
        });
        return true;
    }

    bool UFRobotSystemHardware::_arm_ft(bool on)
    {
        if (on && !has_ft_sensor_) {
            RCLCPP_ERROR(LOGGER, "[%s] Refusing to arm force control: has_ft_sensor is false",
                robot_ip_.c_str());
            return false;
        }
        if (on && cartesian_servo_mode_) {
            RCLCPP_ERROR(LOGGER, "[%s] Refusing to arm force control while in servo mode",
                robot_ip_.c_str());
            return false;
        }
        if (on) {
            // Refuse a target the loop cannot chase. Without this the arm sits with the
            // app started and nothing to seek, which reads as a silent failure.
            bool has_f_ref = false;
            for (int i = 0; i < 6; i++) {
                if (ft_c_axis_[i] && ft_f_ref_[i] != 0.0) has_f_ref = true;
            }
            if (!has_f_ref) {
                RCLCPP_ERROR(LOGGER, "[%s] Refusing to arm force control: f_ref is zero on every compliant axis",
                    robot_ip_.c_str());
                return false;
            }
            // Anchor the standoff where the arm is now, so the force offset is applied
            // on top of the present pose rather than one seeded at activation.
            for (int i = 0; i < 6; i++) tcp_standoff_[i] = tcp_states_[i];

            // set_ft_sensor_mode(0) tears the force app down, taking its configuration
            // with it, so asking for mode 2 again on its own is rejected. Re-run the
            // setup the SDK examples do before entering force mode: sensor enabled,
            // then the force parameters, then the mode. The zero is deliberately not
            // repeated here, it is only valid off contact and would corrupt the
            // calibration if the tool is touching something.
            xarm_driver_.arm->set_ft_sensor_enable(1);
            int cfg_ret = xarm_driver_.arm->set_ft_sensor_force_parameters(
                ft_coord_, ft_c_axis_, ft_f_ref_, ft_limits_,
                ft_kp_, ft_ki_, ft_kd_, ft_xe_limit_);
            if (cfg_ret != 0) {
                RCLCPP_ERROR_THROTTLE(LOGGER, *node_->get_clock(), 2000,
                    "[%s] set_ft_sensor_force_parameters before arming, ret=%d",
                    robot_ip_.c_str(), cfg_ret);
                return false;
            }
        }

        int mode = on ? 2 : 0;
        int ret = xarm_driver_.arm->set_ft_sensor_mode(mode);
        // The force loop starts, and stops, on this state transition.
        xarm_driver_.arm->set_state(XARM_STATE::START);
        _refresh_ft_cfg_states();

        // set_ft_sensor_mode goes through _check_code with is_move_cmd false, which
        // returns 0 even on controller error, so trust the readback rather than ret.
        // ft_armed_ is left untouched on failure so _apply_ft_commands() retries, which
        // matters most in the disarm direction.
        if (ret != 0 || (int)ft_cfg_states_[FT_CFG_MODE] != mode) {
            RCLCPP_ERROR_THROTTLE(LOGGER, *node_->get_clock(), 2000,
                "[%s] Force control failed to reach mode %d: ret=%d, controller reports mode %d",
                robot_ip_.c_str(), mode, ret, (int)ft_cfg_states_[FT_CFG_MODE]);
            return false;
        }

        ft_armed_ = on;
        ft_sensor_mode_ = mode;
        ft_cfg_states_[FT_CFG_ARMED] = on ? 1.0 : 0.0;
        RCLCPP_INFO(LOGGER, "[%s] Force control %s (mode=%d, error=%d)",
            robot_ip_.c_str(), on ? "ARMED" : "DISARMED",
            (int)ft_cfg_states_[FT_CFG_MODE], (int)ft_cfg_states_[FT_CFG_ERROR]);
        return true;
    }

    void UFRobotSystemHardware::_apply_ft_commands(void)
    {
        if (!has_ft_gpio_) return;
        // GpioCommandController leaves the buffer alone until it receives a message, so
        // NaN means nothing has been commanded and the launch params still stand.
        for (int i = 0; i < FT_CMD_COUNT; i++) {
            if (std::isnan(ft_cmds_[i])) return;
        }

        bool want_armed = ft_cmds_[FT_CMD_ENABLE] >= 0.5;
        int want_coord = ft_cmds_[FT_CMD_COORD] >= 0.5 ? 1 : 0;
        int want_c_axis[6];
        float want_f_ref[6];
        for (int i = 0; i < 6; i++) {
            want_c_axis[i] = ft_cmds_[FT_CMD_C_AXIS + i] >= 0.5 ? 1 : 0;
            want_f_ref[i] = (float)ft_cmds_[FT_CMD_F_REF + i];
        }

        // An all zero c_axis is not a meaningful configuration, so treat it as a buffer
        // that was zeroed rather than commanded and leave the current setup alone.
        bool any_axis = false;
        for (int i = 0; i < 6; i++) {
            if (want_c_axis[i]) any_axis = true;
        }
        if (!any_axis) return;

        bool cfg_changed = want_coord != ft_coord_;
        for (int i = 0; i < 6; i++) {
            if (want_c_axis[i] != ft_c_axis_[i] || want_f_ref[i] != ft_f_ref_[i]) cfg_changed = true;
        }
        bool want_changed = want_armed != prev_want_armed_;
        prev_want_armed_ = want_armed;
        if (!cfg_changed && want_armed == ft_armed_) return;

        if (cfg_changed) {
            ft_coord_ = want_coord;
            for (int i = 0; i < 6; i++) {
                ft_c_axis_[i] = want_c_axis[i];
                ft_f_ref_[i] = want_f_ref[i];
            }
            RCLCPP_INFO(LOGGER, "[%s] Force config: coord=%s, c_axis=[%d %d %d %d %d %d],"
                " f_ref=[%.2f %.2f %.2f %.2f %.2f %.2f]",
                robot_ip_.c_str(), ft_coord_ == 0 ? "base" : "tool",
                ft_c_axis_[0], ft_c_axis_[1], ft_c_axis_[2], ft_c_axis_[3], ft_c_axis_[4], ft_c_axis_[5],
                ft_f_ref_[0], ft_f_ref_[1], ft_f_ref_[2], ft_f_ref_[3], ft_f_ref_[4], ft_f_ref_[5]);
        }

        // Everything below does blocking TCP round trips, and a rejected request stays
        // pending because ft_armed_ is only committed on success. Act immediately on a
        // fresh request, then retry at 1 Hz rather than every cycle.
        curr_write_time_ = node_->get_clock()->now();
        if (!want_changed && !cfg_changed
            && curr_write_time_.seconds() - prev_ft_retry_time_.seconds() <= 1.0) {
            return;
        }
        prev_ft_retry_time_ = curr_write_time_;

        if (cfg_changed && ft_armed_ && want_armed) {
            // The force app latches its configuration when it starts, so editing c_axis,
            // f_ref or coord while it runs has no effect. Cycle it through mode 0 to pick
            // the new parameters up; _arm_ft(true) rewrites them on the way back in.
            // The loop releases for a few milliseconds in between.
            RCLCPP_INFO(LOGGER, "[%s] Restarting force loop to apply the new configuration",
                robot_ip_.c_str());
            if (_arm_ft(false)) _arm_ft(true);
        }
        else if (want_armed != ft_armed_) {
            _arm_ft(want_armed);
        }
        else if (cfg_changed) {
            // Disarmed: stage the parameters so the next arm starts from them.
            int ret = xarm_driver_.arm->set_ft_sensor_force_parameters(
                ft_coord_, ft_c_axis_, ft_f_ref_, ft_limits_,
                ft_kp_, ft_ki_, ft_kd_, ft_xe_limit_);
            if (ret != 0) {
                RCLCPP_ERROR(LOGGER, "[%s] set_ft_sensor_force_parameters, ret=%d",
                    robot_ip_.c_str(), ret);
            }
            _refresh_ft_cfg_states();
        }
    }

    int UFRobotSystemHardware::_go_home(void)
    {
        if (has_home_pose_) {
            RCLCPP_INFO(LOGGER, "[%s] Homing to pose [%.1f %.1f %.1f mm, %.4f %.4f %.4f rad] at %.1f mm/s",
                robot_ip_.c_str(), home_pose_[0], home_pose_[1], home_pose_[2],
                home_pose_[3], home_pose_[4], home_pose_[5], home_speed_);
            return xarm_driver_.arm->set_position(home_pose_, -1, home_speed_, home_acc_, 0, true, NO_TIMEOUT);
        }
        if (has_home_joints_) {
            RCLCPP_INFO(LOGGER, "[%s] Homing to joint angles [%.4f %.4f %.4f %.4f %.4f %.4f rad] at %.4f rad/s",
                robot_ip_.c_str(), home_joints_[0], home_joints_[1], home_joints_[2],
                home_joints_[3], home_joints_[4], home_joints_[5], home_joint_speed_);
            return xarm_driver_.arm->set_servo_angle(home_joints_, home_joint_speed_, home_joint_acc_, 0, true, NO_TIMEOUT);
        }
        RCLCPP_WARN(LOGGER, "[%s] home_on_activate is set but neither home_pose nor home_joints was given, skipping homing",
            robot_ip_.c_str());
        return 0;
    }

    int UFRobotSystemHardware::_setup_ft_sensor(void)
    {
        // Call order follows the SDK example 8003-force_control.cc. The 8 argument
        // overload writes the force config and the PID gains in one go.
        int ret = xarm_driver_.arm->set_ft_sensor_force_parameters(
            ft_coord_, ft_c_axis_, ft_f_ref_, ft_limits_,
            ft_kp_, ft_ki_, ft_kd_, ft_xe_limit_);
        if (ret != 0) {
            RCLCPP_ERROR(LOGGER, "[%s] set_ft_sensor_force_parameters, ret=%d", robot_ip_.c_str(), ret);
            return ret;
        }

        ret = xarm_driver_.arm->set_ft_sensor_enable(1);
        if (ret != 0) {
            RCLCPP_ERROR(LOGGER, "[%s] set_ft_sensor_enable, ret=%d", robot_ip_.c_str(), ret);
            return ret;
        }

        if (ft_zero_on_activate_) {
            // Only valid off contact, and it overwrites the stored zero offset and
            // payload calibration.
            ret = xarm_driver_.arm->set_ft_sensor_zero();
            if (ret != 0) {
                RCLCPP_ERROR(LOGGER, "[%s] set_ft_sensor_zero, ret=%d", robot_ip_.c_str(), ret);
                return ret;
            }
            // The zero write has to land before the force loop reads it.
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        ret = xarm_driver_.arm->set_ft_sensor_mode(ft_sensor_mode_);
        if (ret != 0) {
            RCLCPP_ERROR(LOGGER, "[%s] set_ft_sensor_mode(%d), ret=%d", robot_ip_.c_str(), ft_sensor_mode_, ret);
            return ret;
        }

        // The force loop starts on this state transition.
        ret = xarm_driver_.arm->set_state(XARM_STATE::START);
        if (ret != 0) {
            RCLCPP_ERROR(LOGGER, "[%s] set_state(START), ret=%d", robot_ip_.c_str(), ret);
            return ret;
        }

        // Every setter above runs through _check_code with is_move_cmd false, which
        // returns 0 even when the controller reports an error, a warning or not-ready.
        // Read the config back instead of trusting those return codes.
        int ft_mode = -1, ft_is_started = -1, ft_err = -1;
        int cfg_ret = xarm_driver_.arm->get_ft_sensor_config(&ft_mode, &ft_is_started);
        int err_ret = xarm_driver_.arm->get_ft_sensor_error(&ft_err);
        RCLCPP_INFO(LOGGER, "[%s] ft sensor readback: ft_mode=%d, ft_is_started=%d, ft_error=%d (rets %d, %d)",
            robot_ip_.c_str(), ft_mode, ft_is_started, ft_err, cfg_ret, err_ret);

        if (cfg_ret != 0) {
            RCLCPP_ERROR(LOGGER, "[%s] get_ft_sensor_config failed, ret=%d", robot_ip_.c_str(), cfg_ret);
            return cfg_ret;
        }
        if (ft_mode != ft_sensor_mode_) {
            RCLCPP_ERROR(LOGGER, "[%s] ft sensor did not enter mode %d, controller reports mode %d",
                robot_ip_.c_str(), ft_sensor_mode_, ft_mode);
            return -1;
        }
        if (ft_is_started != 1) {
            RCLCPP_ERROR(LOGGER, "[%s] ft sensor app is not running (ft_is_started=%d)",
                robot_ip_.c_str(), ft_is_started);
            return -1;
        }
        if (err_ret == 0 && ft_err != 0) {
            RCLCPP_ERROR(LOGGER, "[%s] ft sensor reports error %d", robot_ip_.c_str(), ft_err);
            return -1;
        }

        RCLCPP_INFO(LOGGER, "[%s] Force control armed: ft_sensor_mode=%d, coord=%s",
            robot_ip_.c_str(), ft_sensor_mode_, ft_coord_ == 0 ? "base" : "tool");
        return 0;
    }

    CallbackReturn UFRobotSystemHardware::on_activate(const rclcpp_lifecycle::State& previous_state)
    {
        req_list_controller_ = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
        res_list_controller_ = std::make_shared<controller_manager_msgs::srv::ListControllers::Response>();
        req_switch_controller_ = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        res_switch_controller_ = std::make_shared<controller_manager_msgs::srv::SwitchController::Response>();

        client_list_controller_ = hw_node_->create_client<controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers");
        client_switch_controller_ = hw_node_->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");

        xarm_driver_.arm->clean_error();
        xarm_driver_.arm->clean_warn();
        xarm_driver_.arm->motion_enable(true);
        // Always POSE here, regardless of cartesian_servo_mode: homing (_go_home(), below)
        // and the pose/force setup that follows are all mode 0 commands -- set_servo_angle()
        // (what has_home_joints_ homes with) silently does nothing in XARM_MODE::SERVO, and
        // since it is called with wait=true and NO_TIMEOUT, that hangs on_activate() forever
        // rather than failing. Switched to SERVO, if requested, only at the very end of this
        // function, once everything that needs POSE is already done.
        xarm_driver_.arm->set_mode(XARM_MODE::POSE);
        xarm_driver_.arm->set_state(XARM_STATE::START);

        // clean_error() cannot clear a fault that is still physically asserted, such as
        // an engaged emergency stop. Check whether it actually cleared before moving the
        // arm or arming the force loop: the controller rejects both while a fault is
        // latched, and set_position() reports success anyway on this firmware because
        // _xarm_is_ready() skips its checks for versions >= 1.5.20.
        int err_warn[2] = { 0, 0 };
        int ew_ret = xarm_driver_.arm->get_err_warn_code(err_warn);
        if (ew_ret != 0) {
            RCLCPP_WARN(LOGGER, "[%s] get_err_warn_code failed, ret=%d, continuing", robot_ip_.c_str(), ew_ret);
        }
        else if (err_warn[0] != 0) {
            RCLCPP_ERROR(LOGGER, "[%s] Controller error C%d is latched: %s", robot_ip_.c_str(),
                err_warn[0], xarm_driver_.controller_error_interpreter(err_warn[0]).c_str());
            RCLCPP_ERROR(LOGGER, "[%s] Clear the fault at the controller, then relaunch."
                " Skipping homing and force control; joint states and the wrench are still published.",
                robot_ip_.c_str());
            ft_sensor_mode_ = 0;
            home_on_activate_ = false;
        }

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

        // Home with the force loop still off, so the arm reaches its standoff before it
        // starts pushing.
        if (home_on_activate_) {
            int ret = _go_home();
            if (ret != 0) {
                // Returning ERROR here would make ros2_control_node throw and abort,
                // burying the cause under a stack trace. Report it and carry on with
                // the force loop disarmed instead.
                RCLCPP_ERROR(LOGGER, "[%s] Homing failed, ret=%d. Not arming force control.",
                    robot_ip_.c_str(), ret);
                ft_sensor_mode_ = 0;
            }
        }

        // Seed the command buffer from where the arm actually is, so the first write()
        // cannot jump it, and latch the standoff held on the compliant axes. Query the
        // control port rather than the report stream, which may not have delivered a
        // frame yet this early.
        fp32 pose[6];
        int pos_ret = xarm_driver_.arm->get_position(pose);
        if (pos_ret != 0) {
            // Without a known starting pose write() has nothing safe to send, so leave
            // the command buffer at NaN, which _tcp_cmd_is_valid() rejects.
            RCLCPP_ERROR(LOGGER, "[%s] get_position failed, ret=%d."
                " Cartesian commanding and force control stay disabled.", robot_ip_.c_str(), pos_ret);
            ft_sensor_mode_ = 0;
            RCLCPP_INFO(LOGGER, "[%s] System started with motion disabled", robot_ip_.c_str());
            return CallbackReturn::SUCCESS;
        }
        for (int i = 0; i < 6; i++) {
            double val = (i < 3) ? pose[i] / 1000.0 : pose[i];
            tcp_states_[i] = val;
            tcp_cmds_[i] = val;
            prev_tcp_cmds_[i] = val;
            tcp_standoff_[i] = val;
        }
        RCLCPP_INFO(LOGGER, "[%s] TCP start pose: [%.4f %.4f %.4f m, %.4f %.4f %.4f rad]",
            robot_ip_.c_str(), tcp_cmds_[0], tcp_cmds_[1], tcp_cmds_[2],
            tcp_cmds_[3], tcp_cmds_[4], tcp_cmds_[5]);

        // set_position() can report success without the arm having moved, so confirm
        // against where it actually ended up rather than trusting the return code.
        if (home_on_activate_ && has_home_pose_) {
            // Position only. Roll and yaw are not comparable componentwise here: this
            // arm works with pitch near -90 deg, which is RPY gimbal lock, so the
            // controller freely returns a different (roll, yaw) split of the same
            // physical orientation (only roll + yaw is determined). Comparing them
            // would reject a correctly reached pose.
            double worst_mm = 0;
            for (int i = 0; i < 3; i++) {
                worst_mm = std::max(worst_mm, (double)std::abs(pose[i] - home_pose_[i]));
            }
            if (worst_mm > 2.0) {
                RCLCPP_ERROR(LOGGER, "[%s] Homing reported success but the TCP is %.1f mm from the"
                    " requested pose. Not arming force control.", robot_ip_.c_str(), worst_mm);
                ft_sensor_mode_ = 0;
            }
        }

        // Arming force control with an all zero target is never useful and is not safe:
        // the loop drives the measured force to zero, so any uncorrected sensor bias
        // makes the arm creep along the compliant axis until it hits a limit. Require an
        // explicit target before the loop is armed.
        bool has_f_ref = false;
        for (int i = 0; i < 6; i++) {
            if (ft_c_axis_[i] && ft_f_ref_[i] != 0.0) has_f_ref = true;
        }
        if (ft_sensor_mode_ == 2 && !has_f_ref) {
            RCLCPP_WARN(LOGGER, "[%s] ft_sensor_mode is 2 but ft_f_ref is zero on every compliant axis."
                " Not arming force control: pass a non zero ft_f_ref to enable it.", robot_ip_.c_str());
            ft_sensor_mode_ = 0;
        }

        if (ft_sensor_mode_ != 0) {
            int ret = _setup_ft_sensor();
            if (ret != 0) {
                RCLCPP_ERROR(LOGGER, "[%s] FORCE CONTROL IS NOT ACTIVE: setup failed, ret=%d%s",
                    robot_ip_.c_str(), ret,
                    ret == 10 ? " (the controller rejected the command as invalid;"
                                " a latched fault or a missing/disabled F/T sensor will both do this)" : "");
                // Leave the stack up rather than aborting the node, so the wrench and
                // joint states stay available while the cause is fixed.
                ft_sensor_mode_ = 0;
                xarm_driver_.arm->set_ft_sensor_mode(0);
            }
        }
        else if (has_ft_sensor_) {
            // Force control off at activation. Still enable sensor communication so the
            // wrench publishes, zero it if asked (only valid off contact, and activation
            // right after homing is the one moment that is known), and push the force
            // params so arming later through the ft gpio has them ready.
            int ret = xarm_driver_.arm->set_ft_sensor_enable(1);
            if (ft_zero_on_activate_) {
                xarm_driver_.arm->set_ft_sensor_zero();
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
            xarm_driver_.arm->set_ft_sensor_force_parameters(
                ft_coord_, ft_c_axis_, ft_f_ref_, ft_limits_,
                ft_kp_, ft_ki_, ft_kd_, ft_xe_limit_);
            RCLCPP_INFO(LOGGER, "[%s] Force control not armed, sensor enabled read only, ret=%d",
                robot_ip_.c_str(), ret);
        }
        else {
            RCLCPP_INFO(LOGGER, "[%s] has_ft_sensor is false, skipping every F/T SDK call",
                robot_ip_.c_str());
        }

        ft_armed_ = ft_sensor_mode_ != 0;
        prev_want_armed_ = ft_armed_;
        // Seed the runtime buffer from the launch params, so the ft gpio reflects the
        // active configuration before anything is published to it and the first
        // _apply_ft_commands() sees no change.
        if (has_ft_gpio_) {
            ft_cmds_[FT_CMD_ENABLE] = ft_armed_ ? 1.0 : 0.0;
            ft_cmds_[FT_CMD_COORD] = ft_coord_;
            for (int i = 0; i < 6; i++) {
                ft_cmds_[FT_CMD_C_AXIS + i] = ft_c_axis_[i];
                ft_cmds_[FT_CMD_F_REF + i] = ft_f_ref_[i];
            }
        }
        _refresh_ft_cfg_states();

        if (cartesian_servo_mode_) {
            // Only now, since homing and the pose/force setup above all needed POSE.
            // set_servo_cartesian() in write() requires this.
            int mode_ret = xarm_driver_.arm->set_mode(XARM_MODE::SERVO);
            xarm_driver_.arm->set_state(XARM_STATE::START);
            RCLCPP_INFO(LOGGER, "[%s] Switched to XARM_MODE::SERVO for cartesian_servo_mode, ret=%d",
                robot_ip_.c_str(), mode_ret);
        }

        prev_write_time_ = node_->get_clock()->now();
        prev_rearm_time_ = prev_write_time_;
        prev_ft_retry_time_ = prev_write_time_;
        servo_settle_until_ = prev_write_time_;
        prev_servo_switch_time_ = prev_write_time_ - rclcpp::Duration::from_seconds(10.0);

        RCLCPP_INFO(LOGGER, "[%s] System Sucessfully started!", robot_ip_.c_str());
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn UFRobotSystemHardware::on_deactivate(const rclcpp_lifecycle::State& previous_state)
    {
        RCLCPP_INFO(LOGGER, "[%s] Stopping ...please wait...", robot_ip_.c_str());

        // Tear the force loop down before stopping. Leaving force mode armed makes
        // later motion commands behave unpredictably.
        if (ft_sensor_mode_ != 0) {
            int ret = xarm_driver_.arm->set_ft_sensor_mode(0);
            RCLCPP_INFO(LOGGER, "[%s] set_ft_sensor_mode(0), ret=%d", robot_ip_.c_str(), ret);
        }
        int ret = xarm_driver_.arm->set_ft_sensor_enable(0);
        RCLCPP_INFO(LOGGER, "[%s] set_ft_sensor_enable(0), ret=%d", robot_ip_.c_str(), ret);
        if (cartesian_servo_mode_) {
            // Leave the arm in POSE rather than stuck in SERVO.
            xarm_driver_.arm->set_mode(XARM_MODE::POSE);
        }
        xarm_driver_.arm->set_state(XARM_STATE::STOP);

        RCLCPP_INFO(LOGGER, "[%s] System sucessfully stopped!", robot_ip_.c_str());
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type UFRobotSystemHardware::read(const rclcpp::Time & time, const rclcpp::Duration &period)
    {
        read_cnts_ += 1;
        read_ready_ = _xarm_is_ready_read();
        // Updated here, ahead of everything that can skip the joint copy below, so a
        // faulted arm (read_ready_ false) still reads 1.0 and never a stale 0.0. The
        // error code is the same one _xarm_is_ready_read() acts on. curr_err starts at 0
        // and only the report socket changes it, so a driver that is not connected
        // cannot vouch for 0: report NaN, which safety_controller treats as no answer.
        fault_state_ = !xarm_driver_.is_connected()
            ? std::numeric_limits<double>::quiet_NaN()
            : (xarm_driver_.curr_err != 0 ? 1.0 : 0.0);
        rclcpp::Time start = node_->get_clock()->now();

        read_code_ = xarm_driver_.update_joint_states(initialized_);

        // Wrench and TCP pose are plain member reads with no TCP round trip. Both are
        // filled by the SDK from the rich report socket, which is always opened
        // regardless of report_type, so this works even with report_type=normal.
        for (int i = 0; i < 6; i++) {
            ft_states_[i] = xarm_driver_.arm->ft_ext_force[i];
            tcp_states_[i] = (i < 3)
                ? xarm_driver_.arm->position[i] / 1000.0
                : xarm_driver_.arm->position[i];
        }
        for (int i = 0; i < 3; i++) {
            ft_cfg_states_[FT_CFG_FORCE + i] = ft_states_[i];
        }
        servo_state_ = (xarm_driver_.curr_mode == XARM_MODE::SERVO) ? 1.0 : 0.0;
        // double time_sec = joint_state_msg_->header.stamp.seconds() - start.seconds();
        // read_total_time_ += time_sec;
        // if (time_sec > read_max_time_) {
        //     read_max_time_ = time_sec;
        // }
        // if (read_cnts_ % 6000 == 0) {
        //     RCLCPP_INFO(LOGGER, "[%s] [READ] cnt: %ld, max: %f, mean: %f, failed: %ld", robot_ip_.c_str(), read_cnts_, read_max_time_, read_total_time_ / read_cnts_, read_failed_cnts_);
        // }
        if (read_code_ == 0 && read_ready_) {
            for (int j = 0; j < info_.joints.size(); j++) {
                position_states_[j] = joint_state_msg_->position[j];
                velocity_states_[j] = joint_state_msg_->velocity[j];
                // effort_states_[j] = joint_state_msg_->effort[j];
            }
            if (!initialized_) {
                for (uint i = 0; i < position_states_.size(); i++) {
                    position_cmds_[i] = position_states_[i];
                    velocity_cmds_[i] = 0.0;
                }
            }
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
        // Ahead of every early return below. Disarming the force loop is the stop
        // button, so it must reach the arm even when the arm is not ready, is paused, or
        // has a full command queue. This issues no motion, and it is a no-op unless the
        // ft gpio actually changed.
        _apply_ft_commands();
        if (_apply_servo_command()) {
            return hardware_interface::return_type::OK;
        }

        if (_handle_config_changed()) {
            return hardware_interface::return_type::OK;
        }

        if (_need_reset()) {
            initialized_ = false;
            _deactivate_controller();
            return hardware_interface::return_type::OK;
        }
        initialized_ = true;
        if(reactivate_controller_later_)
        {
            _activate_controller();
            reactivate_controller_later_ = false;
        }
        // set_position() blocks inside _wait_until_not_pause() while the controller is
        // paused, and inside _wait_until_cmdnum_lt_max() once the 512 deep command cache
        // fills. XArmAPI is constructed with check_is_pause and check_cmdnum_limit true
        // and neither has a setter, so both conditions are screened out here rather than
        // stalling the control loop.
        if (xarm_driver_.curr_state == 3) {
            return hardware_interface::return_type::OK;
        }
        if (xarm_driver_.arm->cmd_num >= cmd_queue_max_) {
            return hardware_interface::return_type::OK;
        }

        // The controller applies the force correction as an offset on top of the
        // commanded pose, so a compliant axis has to be held at a fixed standoff.
        // Tracking the measured pose there would make the command chase the force
        // loop's own output and the arm would walk.
        if (ft_armed_) {
            for (int i = 0; i < 6; i++) {
                if (ft_c_axis_[i]) tcp_cmds_[i] = tcp_standoff_[i];
            }
        }

        // prev_tcp_cmds_ is only advanced on a successful send, so a pose skipped by
        // either guard above is retried on the next cycle. cartesian_servo_mode also
        // sends on a 1s heartbeat even with nothing new: set_servo_cartesian() has no
        // motion queue to sit idle in the way a queued set_position() does, so a
        // genuinely still target still needs a periodic resend or the controller's own
        // servo-mode liveness timeout lapses -- same reasoning as set_servo_angle_j()
        // above.
        curr_write_time_ = node_->get_clock()->now();
        bool servo_heartbeat_due = cartesian_servo_mode_
            && curr_write_time_.seconds() - prev_write_time_.seconds() > 1;
        if (_tcp_cmd_is_valid() && (_tcp_cmds_is_change() || servo_heartbeat_due)) {
            fp32 pose[6];
            for (int i = 0; i < 6; i++) {
                pose[i] = (float)((i < 3) ? tcp_cmds_[i] * 1000.0 : tcp_cmds_[i]);
            }
            // Per motion speed when something wrote one, the launch time tcp_speed
            // otherwise. NaN fails the comparison and falls back too, which is what an
            // unwritten interface reads as. Speed alone never triggers a send: the guard
            // above is on the pose, so a new speed applies to the next motion rather than
            // re issuing the one already running.
            fp32 speed = (tcp_speed_cmd_ > 0.0) ? (fp32)tcp_speed_cmd_ : tcp_speed_;
            int cmd_ret;
            if (cartesian_servo_mode_) {
                // Executes only the most recently sent pose -- no motion queue, so no
                // radius/blend parameter and no risk of the 512 deep backlog set_position()
                // can build under continuous streaming.
                cmd_ret = xarm_driver_.arm->set_servo_cartesian(pose, tcp_speed_, tcp_acc_, 0, false);
            } else {
                // (pose, radius, speed, acc, mvtime, wait). wait stays false: this is a
                // queued motion, and blocking here would stall the control loop.
                cmd_ret = xarm_driver_.arm->set_position(pose, tcp_radius_, tcp_speed_, tcp_acc_, 0, false);
            }
            if (cmd_ret != 0) {
                RCLCPP_WARN(LOGGER, "[%s] %s, ret=%d", robot_ip_.c_str(),
                    cartesian_servo_mode_ ? "set_servo_cartesian" : "set_position", cmd_ret);
            }
            else {
                prev_write_time_ = curr_write_time_;
                for (int i = 0; i < 6; i++) prev_tcp_cmds_[i] = tcp_cmds_[i];
            }
        }

        return hardware_interface::return_type::OK;
    }

    bool UFRobotSystemHardware::_is_ft_controller(const controller_manager_msgs::msg::ControllerState& c)
    {
        // claimed_interfaces is empty while a controller is inactive, so decide on what
        // it requires rather than on what it currently holds.
        if (!has_ft_gpio_ || c.required_command_interfaces.empty()) return false;
        for (const auto & iface : c.required_command_interfaces) {
            if (iface.rfind(ft_iface_prefix_, 0) != 0) return false;
        }
        return true;
    }

    bool UFRobotSystemHardware::_handle_config_changed(void)
    {
        // Every force config change leaves the arm in state 5 (CONFIG_CHANGED), so this
        // is self inflicted and self healing. Re-arm and skip the cycle, but do not let
        // it reach _need_reset(): tearing the controllers down would take the ft command
        // interfaces with it, and then nothing could disarm the force loop.
        if (xarm_driver_.curr_state != 5) return false;
        curr_write_time_ = node_->get_clock()->now();
        if (curr_write_time_.seconds() - prev_rearm_time_.seconds() > 1.0) {
            prev_rearm_time_ = curr_write_time_;
            int ret = xarm_driver_.arm->set_state(XARM_STATE::START);
            RCLCPP_INFO(LOGGER, "[%s] State 5 (CONFIG_CHANGED), re-arming with set_state(START), ret=%d",
                robot_ip_.c_str(), ret);
        }
        return true;
    }

    void UFRobotSystemHardware::_deactivate_controller(void) {
        if(reactivate_controller_later_)
            return;
        // RCLCPP_INFO(LOGGER, "DEACTIVATE CONTROLLER!! ");
        int ret = _call_request(client_list_controller_, req_list_controller_, res_list_controller_);
        bool valid_operation = false;
        if (ret == 0 && res_list_controller_->controller.size() > 0) {
            req_switch_controller_->activate_controllers.resize(0);
            req_switch_controller_->deactivate_controllers.clear();
            for (uint i = 0; i < res_list_controller_->controller.size(); i++) {
                // The force control controller commands no motion, and it is the only way
                // to disarm the force loop. Leave it running: it is needed most exactly
                // when the arm is in the state that got us here.
                if (_is_ft_controller(res_list_controller_->controller[i])) continue;
                if(res_list_controller_->controller[i].state == std::string("active")){
                // for situation of initial launch with emg stop pressed, launch file will activate controller and it takes a while
                    valid_operation = true;
                }
                req_switch_controller_->deactivate_controllers.push_back(res_list_controller_->controller[i].name);
            }
            req_switch_controller_->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
            req_switch_controller_->timeout = rclcpp::Duration::from_seconds(2.0);
            if(valid_operation)
            {
                _call_request(client_switch_controller_, req_switch_controller_, res_switch_controller_);
                reactivate_controller_later_ = true; // Not setting this indicator until activated controller disabled!
            }
        }
    }

    void UFRobotSystemHardware::_activate_controller(void) {
        // RCLCPP_INFO(LOGGER, "ACTIVATE CONTROLLER!! ");
        int ret = _call_request(client_list_controller_, req_list_controller_, res_list_controller_);
        if (ret == 0 && res_list_controller_->controller.size() > 0) {
            req_switch_controller_->deactivate_controllers.resize(0);
            req_switch_controller_->activate_controllers.clear();
            for (uint i = 0; i < res_list_controller_->controller.size(); i++) {
                // Never deactivated above, so never needs reactivating here.
                if (_is_ft_controller(res_list_controller_->controller[i])) continue;
                req_switch_controller_->activate_controllers.push_back(res_list_controller_->controller[i].name);
            }
            req_switch_controller_->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
            req_switch_controller_->timeout = rclcpp::Duration::from_seconds(2.0);
            _call_request(client_switch_controller_, req_switch_controller_, res_switch_controller_);
        }
        update_goal_state_pub_->publish(update_goal_state_msg_);
    }

    bool UFRobotSystemHardware::_check_cmds_is_change(float *prev, float *cur, double threshold)
	{
		for (int i = 0; i < 7; i++) {
            if (std::abs(cur[i] - prev[i]) > threshold) return true;
        }
        return false;
	}

    // Screens out a command buffer that was never filled in. A controller that claims
    // the interfaces without publishing leaves them at NaN or at zero depending on its
    // implementation, and either one must never reach set_position().
    bool UFRobotSystemHardware::_tcp_cmd_is_valid(void)
    {
        bool all_zero = true;
        for (int i = 0; i < 6; i++) {
            if (std::isnan(tcp_cmds_[i])) return false;
            if (tcp_cmds_[i] != 0.0) all_zero = false;
        }
        if (all_zero) {
            // The TCP cannot physically sit at the base origin with zero orientation,
            // so an exactly zero pose only ever means an uninitialised buffer.
            RCLCPP_WARN_THROTTLE(LOGGER, *node_->get_clock(), 5000,
                "[%s] Ignoring an all zero TCP command, treating it as uninitialised",
                robot_ip_.c_str());
            return false;
        }
        // The UF850 reaches 850 mm. Anything past this is not a pose the arm could hold,
        // and set_position would reject it anyway.
        double reach = std::sqrt(tcp_cmds_[0] * tcp_cmds_[0]
            + tcp_cmds_[1] * tcp_cmds_[1] + tcp_cmds_[2] * tcp_cmds_[2]);
        if (reach > 1.5) {
            RCLCPP_WARN_THROTTLE(LOGGER, *node_->get_clock(), 5000,
                "[%s] Ignoring TCP command %.3f m from the base origin, beyond reach",
                robot_ip_.c_str(), reach);
            return false;
        }
        return true;
    }

    bool UFRobotSystemHardware::_tcp_cmds_is_change(double threshold)
    {
        for (int i = 0; i < 6; i++) {
            if (std::abs(tcp_cmds_[i] - prev_tcp_cmds_[i]) > threshold) return true;
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
        int curr_mode = xarm_driver_.curr_mode;
		int curr_state = xarm_driver_.curr_state;

        if (!_xarm_is_ready_read()) {
            last_not_ready = true;
            return false;
        }

        // State 5 is CONFIG_CHANGED: "system configuration or mode changed, not ready
        // for motion commands" (see xarm_msgs/msg/RobotMsg.msg). Arming the force loop
        // is itself a configuration change, so the arm lands in state 5 and stays there
        // until set_state() re-arms it. Without this the controller happily reports
        // ft_is_started=1 while the arm refuses to move at all.
        if (curr_state == 5) {
            curr_write_time_ = node_->get_clock()->now();
            if (curr_write_time_.seconds() - prev_rearm_time_.seconds() > 1.0) {
                prev_rearm_time_ = curr_write_time_;
                int ret = xarm_driver_.arm->set_state(XARM_STATE::START);
                RCLCPP_WARN(LOGGER, "[%s] State 5 (CONFIG_CHANGED), re-arming with set_state(START), ret=%d",
                    robot_ip_.c_str(), ret);
            }
            last_not_ready = true;
            return false;
        }

        // These used to be logged only when the value changed, which meant a robot that
        // was never ready in the first place produced no message at all.
        if (curr_state > 2) {
            RCLCPP_WARN_THROTTLE(LOGGER, *node_->get_clock(), 2000,
                "[%s] Not ready to write: state=%d (1 running, 2 sleeping, 3 paused, 4 stopped, 5 config changed)",
                robot_ip_.c_str(), curr_state);
            last_not_ready = true;
            return false;
        }

        int expected_mode = cartesian_servo_mode_ ? XARM_MODE::SERVO : XARM_MODE::POSE;
        if (curr_mode != expected_mode) {
            RCLCPP_WARN_THROTTLE(LOGGER, *node_->get_clock(), 2000,
                "[%s] Not ready to write: mode=%d, expected %d",
                robot_ip_.c_str(), curr_mode, expected_mode);
            last_not_ready = true;
            return false;
        }

        if (last_not_ready) {
            RCLCPP_INFO(LOGGER, "[%s] Robot is Ready (state=%d, mode=%d)",
                robot_ip_.c_str(), curr_state, curr_mode);
        }
        last_not_ready = false;
        return true;
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

