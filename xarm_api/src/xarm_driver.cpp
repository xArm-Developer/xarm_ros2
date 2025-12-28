/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
           Vinman <vinman.cub@gmail.com>
 ============================================================================*/
#include "xarm_api/xarm_driver.h"

#define CMD_HEARTBEAT_SEC 30 // 30s

#define DEBUG_MODE 1

#define BIND_CLS_CB(func) std::bind(func, this, std::placeholders::_1, std::placeholders::_2)
#define BIND_CLS_CB_1(func) std::bind(func, this, std::placeholders::_1)


// void* cmd_heart_beat(void* args)
// {
//     xarm_api::XArmDriver *my_driver = (xarm_api::XArmDriver *) args;
//     int cmdnum;
//     int cnt = 0;
//     int max_cnt = CMD_HEARTBEAT_SEC * 2;
//     while(my_driver->arm->is_connected())
//     {
//         sleep_milliseconds(500);
//         cnt += 1;
//         if (cnt >= max_cnt) {
//             cnt = 0;
//             my_driver->arm->get_cmdnum(&cmdnum);
//         }
//     }
//     RCLCPP_ERROR(my_driver->get_logger(), "xArm Control Connection Failed! Please Shut Down (Ctrl-C) and Retry ...");
//     return (void*)0;
// }

namespace xarm_api
{   
    static const rclcpp::Logger LOGGER = rclcpp::get_logger("uf_ros_driver.sdk");

    XArmDriver::~XArmDriver()
    {   
        arm->set_mode(XARM_MODE::POSE);
        arm->disconnect();
    }

    bool XArmDriver::_get_wait_param(void) 
    {
        bool wait;
        node_->get_parameter_or("wait_for_finish", wait, false);
        return wait;
    }

    void XArmDriver::_report_connect_changed_callback(bool connected, bool reported)
    {
        RCLCPP_INFO(node_->get_logger(), "[TCP STATUS] CONTROL: %d, REPORT: %d", connected, reported);
    }

    void XArmDriver::_report_data_callback(XArmReportData *report_data_ptr)
    {
        // RCLCPP_INFO(node_->get_logger(), "[1] state: %d, error_code: %d", report_data_ptr->state, report_data_ptr->err);
        curr_state = report_data_ptr->state;
        curr_err = report_data_ptr->err;
        curr_mode = report_data_ptr->mode;
        curr_cmdnum = report_data_ptr->cmdnum;

        rclcpp::Time now_ = node_->get_clock()->now();
        bool use_new = _firmware_version_is_ge(1, 8, 103);
        if (!use_new)
        {
            for(int i = 0; i < dof_; i++)
            {
                // joint_state_msg_.position[i] = (double)report_data_ptr->angle[i];
                if (!in_ros_control_)
                    joint_state_msg_.velocity[i] = (double)report_data_ptr->rt_joint_spds[i];
                joint_state_msg_.effort[i] = (double)report_data_ptr->tau[i];
            }
        }

        xarm_state_msg_.state = report_data_ptr->state;
        xarm_state_msg_.mode = report_data_ptr->mode;
        xarm_state_msg_.cmdnum = report_data_ptr->cmdnum;
        xarm_state_msg_.err = report_data_ptr->err;
        xarm_state_msg_.warn = report_data_ptr->war;
        xarm_state_msg_.mt_brake = report_data_ptr->mt_brake;
        xarm_state_msg_.mt_able = report_data_ptr->mt_able;

        for(int i = 0; i < dof_; i++)
        {
            xarm_state_msg_.angle[i] = (double)report_data_ptr->angle[i];
        }
        for(int i = 0; i < 6; i++)
        {
            xarm_state_msg_.pose[i] = report_data_ptr->pose[i];
            xarm_state_msg_.offset[i] = report_data_ptr->tcp_offset[i];
        }
        xarm_state_msg_.header.stamp = now_;
        pub_robot_msg(xarm_state_msg_);

        if (report_data_ptr->total_num >= 417) {
            cgpio_state_msg_.header.stamp = now_;
            cgpio_state_msg_.state = report_data_ptr->cgpio_state;
            cgpio_state_msg_.code = report_data_ptr->cgpio_code;
            cgpio_state_msg_.input_digitals[0] = report_data_ptr->cgpio_input_digitals[0];
            cgpio_state_msg_.input_digitals[1] = report_data_ptr->cgpio_input_digitals[1];
            cgpio_state_msg_.output_digitals[0] = report_data_ptr->cgpio_output_digitals[0];
            cgpio_state_msg_.output_digitals[1] = report_data_ptr->cgpio_output_digitals[1];

            cgpio_state_msg_.input_analogs[0] = report_data_ptr->cgpio_input_analogs[0];
            cgpio_state_msg_.input_analogs[1] = report_data_ptr->cgpio_input_analogs[1];
            cgpio_state_msg_.output_analogs[0] = report_data_ptr->cgpio_output_analogs[0];
            cgpio_state_msg_.output_analogs[1] = report_data_ptr->cgpio_output_analogs[1];

            for (int i = 0; i < 16; ++i) {
                cgpio_state_msg_.input_conf[i] = report_data_ptr->cgpio_input_conf[i];
                cgpio_state_msg_.output_conf[i] = report_data_ptr->cgpio_output_conf[i];
            }
            pub_cgpio_state(cgpio_state_msg_);
        }

        if ((report_type_ == "dev" && report_data_ptr->total_num >= 135) 
            || (report_type_ == "rich" && report_data_ptr->total_num >= 481)) {
            ftsensor_msg_.header.stamp = now_;
            ftsensor_msg_.header.frame_id = "uf_ft_sensor_ext_data";
            ftsensor_msg_.wrench.force.x = report_data_ptr->ft_ext_force[0];
            ftsensor_msg_.wrench.force.y = report_data_ptr->ft_ext_force[1];
            ftsensor_msg_.wrench.force.z = report_data_ptr->ft_ext_force[2];
            ftsensor_msg_.wrench.torque.x = report_data_ptr->ft_ext_force[3];
            ftsensor_msg_.wrench.torque.y = report_data_ptr->ft_ext_force[4];
            ftsensor_msg_.wrench.torque.z = report_data_ptr->ft_ext_force[5];
            pub_ftsensor_ext_state(ftsensor_msg_);
            ftsensor_msg_.header.frame_id = "uf_ft_sensor_raw_data";
            ftsensor_msg_.wrench.force.x = report_data_ptr->ft_raw_force[0];
            ftsensor_msg_.wrench.force.y = report_data_ptr->ft_raw_force[1];
            ftsensor_msg_.wrench.force.z = report_data_ptr->ft_raw_force[2];
            ftsensor_msg_.wrench.torque.x = report_data_ptr->ft_raw_force[3];
            ftsensor_msg_.wrench.torque.y = report_data_ptr->ft_raw_force[4];
            ftsensor_msg_.wrench.torque.z = report_data_ptr->ft_raw_force[5];
            pub_ftsensor_raw_state(ftsensor_msg_);
        }
    }

    void XArmDriver::init(rclcpp::Node::SharedPtr& node, std::string &server_ip, bool in_ros_control)
    {
        curr_err = 0;
        curr_state = 4;
        curr_mode = 0;
        curr_cmdnum = 0;
        arm = NULL;
        in_ros_control_ = in_ros_control;
        vacuum_gripper_hardware_version_ = 0;

        node_ = node;
        std::string prefix = "";
        node_->get_parameter_or("prefix", prefix, std::string(""));
        std::string hw_ns;
        node_->get_parameter_or("hw_ns", hw_ns, std::string("xarm"));
        // hw_ns = prefix + hw_ns;
        hw_node_ = node_->create_sub_node(hw_ns);
        node_->get_parameter_or("dof", dof_, 7);
        node_->get_parameter_or("report_type", report_type_, std::string("normal"));

        node_->get_parameter_or("joint_names", joint_names_, 
        std::vector<std::string>({"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"}));
        if (prefix != "") {
            for (int i = 0; i < joint_names_.size(); i++) {
                joint_names_[i] = prefix + joint_names_[i];
            }
        }

        node_->get_parameter_or("joint_states_rate", joint_states_rate_, -1);

        RCLCPP_INFO(node_->get_logger(), "robot_ip=%s, report_type=%s, dof=%d", server_ip.c_str(), report_type_.c_str(), dof_);

        bool baud_checkset = true;
        int default_gripper_baud = 2000000;
        node_->get_parameter_or("baud_checkset", baud_checkset, true);
        node_->get_parameter_or("default_gripper_baud", default_gripper_baud, 2000000);
        
        RCLCPP_INFO(node_->get_logger(), "baud_checkset: %d, default_gripper_baud: %d", baud_checkset, default_gripper_baud);

        _init_publisher();
        setlinebuf(stdout);

        arm = new XArmAPI(
            server_ip, 
            true, // is_radian
            true, // do_not_open
            true, // check_tcp_limit
            true, // check_joint_limit
            true, // check_cmdnum_limit
            false, // check_robot_sn
            true, // check_is_ready
            true, // check_is_pause
            0, // max_callback_thread_count
            512, // max_cmdnum
            dof_, // init_axis
            DEBUG_MODE, // debug
            report_type_ // report_type
        );
        arm->set_baud_checkset_enable(baud_checkset);
        arm->set_checkset_default_baud(1, default_gripper_baud);
        arm->release_connect_changed_callback(true);
        arm->release_report_data_callback(true);
        arm->register_connect_changed_callback(std::bind(&XArmDriver::_report_connect_changed_callback, this, std::placeholders::_1, std::placeholders::_2));
        arm->register_report_data_callback(std::bind(&XArmDriver::_report_data_callback, this, std::placeholders::_1));
        arm->connect();

        int err_warn[2] = {0};
        int ret = arm->get_err_warn_code(err_warn);
        if (err_warn[0] != 0) {
            RCLCPP_WARN(node_->get_logger(), "UFACTORY ErrorCode: C%d: [ %s ]", err_warn[0], controller_error_interpreter(err_warn[0]).c_str());
        }
        
        // std::thread th(cmd_heart_beat, this);
        // th.detach();
        int dbg_msg[16] = {0};
        arm->core->servo_get_dbmsg(dbg_msg);

        for(int i=0; i<dof_; i++)
        {
            if((dbg_msg[i*2]==1)&&(dbg_msg[i*2+1]==40))
            {
                arm->clean_error();
                RCLCPP_WARN(node_->get_logger(), "Cleared low-voltage error of joint %d", i+1);
            }
            else if((dbg_msg[i*2]==1))
            {
                arm->clean_error();
                RCLCPP_WARN(node_->get_logger(), "There is servo error code:(0x%x) in joint %d, trying to clear it..", dbg_msg[i*2+1], i+1);
            }
        }

        if (!in_ros_control_)
        {
            std::thread([this]() {
                float cur_pos;
                float position[7] = {0};
                float velocity[7] = {0};
                float effort[7] = {0};
                if (joint_states_rate_ < 0) {
                    joint_states_rate_ = report_type_ == "dev" ? 100 : 5;
                }
                bool use_new = _firmware_version_is_ge(1, 8, 103);
                int microseconds = 1000000 / joint_states_rate_;
                while (arm->is_connected())
                {
                    if (use_new)
                        arm->get_joint_states(position, velocity, effort);
                    else
                        arm->get_servo_angle(position);

                    joint_state_msg_.header.stamp = node_->get_clock()->now();
                    for(int i = 0; i < dof_; i++)
                    {
                        joint_state_msg_.position[i] = (double)position[i];
                        if (use_new)
                        {
                            joint_state_msg_.velocity[i] = (double)velocity[i];
                            joint_state_msg_.effort[i] = (double)effort[i];
                        }
                    }
                    pub_joint_state(joint_state_msg_);
                    std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
                }
                RCLCPP_ERROR(node_->get_logger(), "xArm Control Connection Failed! Please Shut Down (Ctrl-C) and Retry ...");
            }).detach();
        }

        _init_service();
        _init_subscription();
        _init_xarm_gripper();
        _init_bio_gripper();
    }

    bool XArmDriver::_firmware_version_is_ge(int major, int minor, int revision)
	{
		return arm->version_number[0] > major || (arm->version_number[0] == major && arm->version_number[1] > minor) || (arm->version_number[0] == major && arm->version_number[1] == minor && arm->version_number[2] >= revision);
	}

    void XArmDriver::_init_publisher(void)
    {
        joint_state_msg_.header.frame_id = "joint-state data";
        joint_state_msg_.name.resize(dof_);
        joint_state_msg_.position.resize(dof_);
        joint_state_msg_.velocity.resize(dof_, 0);
        joint_state_msg_.effort.resize(dof_, 0);
        for(int i = 0; i < dof_; i++)
        {
            joint_state_msg_.name[i] = joint_names_[i];
        }
        xarm_state_msg_.angle.resize(dof_);

        joint_state_pub_ = hw_node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        robot_state_pub_ = hw_node_->create_publisher<xarm_msgs::msg::RobotMsg>("robot_states", 10);
        cgpio_state_pub_ = hw_node_->create_publisher<xarm_msgs::msg::CIOState>("xarm_cgpio_states", 10);
        ftsensor_ext_state_pub_ = hw_node_->create_publisher<geometry_msgs::msg::WrenchStamped>("uf_ftsensor_ext_states", 10);
        ftsensor_raw_state_pub_ = hw_node_->create_publisher<geometry_msgs::msg::WrenchStamped>("uf_ftsensor_raw_states", 10);
    }

    sensor_msgs::msg::JointState* XArmDriver::get_joint_states()
    {
        return &joint_state_msg_;
    }

    void XArmDriver::_init_xarm_gripper(void)
    {
        node_->get_parameter_or("xarm_gripper.speed", xarm_gripper_speed_, 2000);  // 机械爪速度
        node_->get_parameter_or("xarm_gripper.max_pos", xarm_gripper_max_pos_, 850); // 机械爪最大值，用来转换
        node_->get_parameter_or("xarm_gripper.frequency", xarm_gripper_frequency_, 10); // 发送机械爪位置后查询机械爪位置的频率
        node_->get_parameter_or("xarm_gripper.threshold", xarm_gripper_threshold_, 3); // 检测机械爪当前位置和上一次位置的差值如果小于当前值，则认为机械爪没动
        node_->get_parameter_or("xarm_gripper.threshold_times", xarm_gripper_threshold_times_, 10); // 如果检测到机械爪没动的次数超过此值且当前位置和目标位置差值不超过15，则认为机械爪运动成功
        RCLCPP_INFO(node_->get_logger(), "gripper_speed: %d, gripper_max_pos: %d, gripper_frequency : %d, gripper_threshold: %d, gripper_threshold_times: %d", 
            xarm_gripper_speed_, xarm_gripper_max_pos_, xarm_gripper_frequency_, xarm_gripper_threshold_, xarm_gripper_threshold_times_);

        xarm_gripper_feedback_ = std::make_shared<control_msgs::action::GripperCommand::Feedback>();
        xarm_gripper_result_ = std::make_shared<control_msgs::action::GripperCommand::Result>();;
        xarm_gripper_joint_state_msg_.header.stamp = node_->get_clock()->now();
        xarm_gripper_joint_state_msg_.header.frame_id = "gripper-joint-state data";        
        xarm_gripper_joint_state_msg_.name.resize(6);
        xarm_gripper_joint_state_msg_.position.resize(6, std::numeric_limits<double>::quiet_NaN());
        xarm_gripper_joint_state_msg_.velocity.resize(6, std::numeric_limits<double>::quiet_NaN());
        xarm_gripper_joint_state_msg_.effort.resize(6, std::numeric_limits<double>::quiet_NaN());
        node_->get_parameter_or("xarm_gripper.joint_names", xarm_gripper_joint_state_msg_.name, 
            std::vector<std::string>({"drive_joint", "left_finger_joint", "left_inner_knuckle_joint", "right_outer_knuckle_joint", "right_finger_joint", "right_inner_knuckle_joint"}));
        
        std::string prefix = "";
        node_->get_parameter_or("prefix", prefix, std::string(""));
        if (prefix != "") {
            for (int i = 0; i < xarm_gripper_joint_state_msg_.name.size(); i++) {
                xarm_gripper_joint_state_msg_.name[i] = prefix + xarm_gripper_joint_state_msg_.name[i];
            }
        }

        xarm_gripper_action_server_ = rclcpp_action::create_server<control_msgs::action::GripperCommand>(
            node_, prefix + "xarm_gripper/gripper_action",
            BIND_CLS_CB(&XArmDriver::_handle_xarm_gripper_action_goal),
            BIND_CLS_CB_1(&XArmDriver::_handle_xarm_gripper_action_cancel),
            BIND_CLS_CB_1(&XArmDriver::_handle_xarm_gripper_action_accepted));
        
        bool add_gripper;
        node_->get_parameter_or("add_gripper", add_gripper, false);
        if (add_gripper) {
            xarm_gripper_init_loop_ = false;
            std::thread([this]() {
                float cur_pos;
                int ret = arm->get_gripper_position(&cur_pos);
                while (ret == 0 && !xarm_gripper_init_loop_)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    _pub_xarm_gripper_joint_states(cur_pos);
                }
            }).detach();
        }
    }

    inline float XArmDriver::_xarm_gripper_pos_convert(float pos, bool reversed)
    {
        if (reversed) {
            return fabs(xarm_gripper_max_pos_ - pos * 1000);
        }
        else {
            return fabs(xarm_gripper_max_pos_ - pos) / 1000;
        }
    }

    void XArmDriver::_pub_xarm_gripper_joint_states(float pos)
    {
        xarm_gripper_joint_state_msg_.header.stamp = node_->get_clock()->now();
        float p = _xarm_gripper_pos_convert(pos);
        for (int i = 0; i < 6; i++) {
            xarm_gripper_joint_state_msg_.position[i] = p;
        }
        pub_joint_state(xarm_gripper_joint_state_msg_);
    }

    rclcpp_action::GoalResponse XArmDriver::_handle_xarm_gripper_action_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const control_msgs::action::GripperCommand::Goal> goal)
    {
        // RCLCPP_INFO(node_->get_logger(), "Received gripper move goal request, target_pulse=%f, pulse_speed=%f", goal->target_pulse, goal->pulse_speed);
        RCLCPP_INFO(node_->get_logger(), "Received gripper move goal request, position=%f, max_effort=%f", goal->command.position, goal->command.max_effort);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse XArmDriver::_handle_xarm_gripper_action_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel gripper move goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void XArmDriver::_handle_xarm_gripper_action_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle)
    {
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{BIND_CLS_CB_1(&XArmDriver::_xarm_gripper_action_execute), goal_handle}.detach();
    }

    void XArmDriver::_xarm_gripper_action_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle)
    {
        xarm_gripper_init_loop_ = true;
        const auto goal = goal_handle->get_goal();
        RCLCPP_INFO(node_->get_logger(), "gripper_action_execute, position=%f, max_effort=%f", goal->command.position, goal->command.max_effort);
        
        int ret;
        float cur_pos = 0;
        int err = 0;
        ret = arm->get_gripper_err_code(&err);
        if (ret != 0 || err != 0) {
            try {
                goal_handle->canceled(xarm_gripper_result_);
            } catch (std::exception &e) {
                RCLCPP_ERROR(node_->get_logger(), "goal_handle canceled exception, ex=%s", e.what());    
            }
            RCLCPP_ERROR(node_->get_logger(), "get_gripper_err_code, ret=%d, err=%d", ret, err);
            return;
        }
        ret = arm->get_gripper_position(&cur_pos);
        _pub_xarm_gripper_joint_states(cur_pos);

        ret = arm->set_gripper_mode(0);
        if (ret != 0) {
            xarm_gripper_result_->position = _xarm_gripper_pos_convert(cur_pos);
            try {
                goal_handle->canceled(xarm_gripper_result_);
            } catch (std::exception &e) {
                RCLCPP_ERROR(node_->get_logger(), "goal_handle canceled exception, ex=%s", e.what()); 
            }
            ret = arm->get_gripper_err_code(&err);
            RCLCPP_WARN(node_->get_logger(), "set_gripper_mode, ret=%d, err=%d, cur_pos=%f", ret, err, cur_pos);
            return;
        }
        ret = arm->set_gripper_enable(true);
        if (ret != 0) {
            xarm_gripper_result_->position = _xarm_gripper_pos_convert(cur_pos);
            try {
                goal_handle->canceled(xarm_gripper_result_);
            } catch (std::exception &e) {
                RCLCPP_ERROR(node_->get_logger(), "goal_handle canceled exception, ex=%s", e.what()); 
            }
            ret = arm->get_gripper_err_code(&err);
            RCLCPP_WARN(node_->get_logger(), "set_gripper_enable, ret=%d, err=%d, cur_pos=%f", ret, err, cur_pos);
            return;
        }
        ret = arm->set_gripper_speed(xarm_gripper_speed_);
        if (ret != 0) {
            xarm_gripper_result_->position = _xarm_gripper_pos_convert(cur_pos);
            try {
                goal_handle->canceled(xarm_gripper_result_);
            } catch (std::exception &e) {
                RCLCPP_ERROR(node_->get_logger(), "goal_handle canceled exception, ex=%s", e.what()); 
            }
            ret = arm->get_gripper_err_code(&err);
            RCLCPP_WARN(node_->get_logger(), "set_gripper_speed, ret=%d, err=%d, cur_pos=%f", ret, err, cur_pos);
            return;
        }
        float last_pos = -xarm_gripper_max_pos_;
        float target_pos = _xarm_gripper_pos_convert(goal->command.position, true);
        bool is_move = true;
        std::thread([this, &target_pos, &is_move, &cur_pos]() {
            is_move = true;
            int ret2 = arm->set_gripper_position(target_pos, true, -1, false); // set wait_motion=false
            int err;
            arm->get_gripper_err_code(&err);
            RCLCPP_INFO(node_->get_logger(), "set_gripper_position, ret=%d, err=%d, cur_pos=%f", ret2, err, cur_pos);
            is_move = false;
        }).detach();
        int cnt = 0;
        bool is_succeed = false;
        auto sltime = std::chrono::nanoseconds(1000000000 / xarm_gripper_frequency_);
        while (is_move && rclcpp::ok())
        {
            std::this_thread::sleep_for(sltime);
            ret = arm->get_gripper_position(&cur_pos);
            if (ret == 0) {
                if (!is_succeed) {
                    if (fabs(last_pos - cur_pos) < xarm_gripper_threshold_) {
                        cnt += 1;
                        if (cnt >= xarm_gripper_threshold_times_ && fabs(target_pos - cur_pos) < 15) {
                            xarm_gripper_result_->position = _xarm_gripper_pos_convert(cur_pos);
                            try {
                                goal_handle->succeed(xarm_gripper_result_);
                            } catch (std::exception &e) {
                                RCLCPP_ERROR(node_->get_logger(), "goal_handle succeed exception, ex=%s", e.what()); 
                            }
                            is_succeed = true;
                        }
                    }
                    else {
                        cnt = 0;
                        last_pos = cur_pos;
                    }
                }
                xarm_gripper_feedback_->position = _xarm_gripper_pos_convert(cur_pos);
                try {
                    goal_handle->publish_feedback(xarm_gripper_feedback_);
                } catch (std::exception &e) {
                    RCLCPP_ERROR(node_->get_logger(), "goal_handle publish_feedback exception, ex=%s", e.what());
                }
                _pub_xarm_gripper_joint_states(cur_pos);
            }
            // if (goal_handle->is_canceling()) {
            //     xarm_gripper_result_->position = _xarm_gripper_pos_convert(cur_pos);
            //     goal_handle->canceled(xarm_gripper_result_);
            //     RCLCPP_INFO(this->get_logger(), "Goal canceled, cur_pos=%f", cur_pos);
            //     return;
            // }
        }
        arm->get_gripper_position(&cur_pos);
        RCLCPP_INFO(node_->get_logger(), "move finish, cur_pos=%f", cur_pos);
        if (rclcpp::ok() && !is_succeed) {
            xarm_gripper_result_->position = _xarm_gripper_pos_convert(cur_pos);
            try {
                goal_handle->succeed(xarm_gripper_result_);
            } catch (std::exception &e) {
                RCLCPP_ERROR(node_->get_logger(), "goal_handle succeed exception, ex=%s", e.what());
            }
            RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
        }
    }


    void XArmDriver::_init_bio_gripper(void)
    {
        node_->get_parameter_or("bio_gripper.speed", bio_gripper_speed_, 2000);  // 机械爪速度
        node_->get_parameter_or("bio_gripper.max_pos", bio_gripper_max_pos_, 130); // 机械爪最大值，用来转换
        node_->get_parameter_or("bio_gripper.min_pos", bio_gripper_min_pos_, 50); // 机械爪最大值，用来转换
        node_->get_parameter_or("bio_gripper.frequency", bio_gripper_frequency_, 10); // 发送机械爪位置后查询机械爪位置的频率
        node_->get_parameter_or("bio_gripper.threshold", bio_gripper_threshold_, 3); // 检测机械爪当前位置和上一次位置的差值如果小于当前值，则认为机械爪没动
        node_->get_parameter_or("bio_gripper.threshold_times", bio_gripper_threshold_times_, 10); // 如果检测到机械爪没动的次数超过此值且当前位置和目标位置差值不超过15，则认为机械爪运动成功
        RCLCPP_INFO(node_->get_logger(), "bio_gripper_speed: %d, bio_gripper_max_pos: %d, bio_gripper_min_pos: %d, bio_gripper_frequency : %d, bio_gripper_threshold: %d, bio_gripper_threshold_times: %d", 
            bio_gripper_speed_, bio_gripper_max_pos_, bio_gripper_min_pos_, bio_gripper_frequency_, bio_gripper_threshold_, bio_gripper_threshold_times_);

        bio_gripper_feedback_ = std::make_shared<control_msgs::action::GripperCommand::Feedback>();
        bio_gripper_result_ = std::make_shared<control_msgs::action::GripperCommand::Result>();;
        bio_gripper_joint_state_msg_.header.stamp = node_->get_clock()->now();
        bio_gripper_joint_state_msg_.header.frame_id = "bio-gripper-joint-state data";        
        bio_gripper_joint_state_msg_.name.resize(2);
        bio_gripper_joint_state_msg_.position.resize(2, std::numeric_limits<double>::quiet_NaN());
        bio_gripper_joint_state_msg_.velocity.resize(2, std::numeric_limits<double>::quiet_NaN());
        bio_gripper_joint_state_msg_.effort.resize(2, std::numeric_limits<double>::quiet_NaN());
        node_->get_parameter_or("bio_gripper.joint_names", bio_gripper_joint_state_msg_.name, 
            std::vector<std::string>({"left_finger_joint", "right_finger_joint"}));
        
        std::string prefix = "";
        node_->get_parameter_or("prefix", prefix, std::string(""));
        if (prefix != "") {
            for (int i = 0; i < bio_gripper_joint_state_msg_.name.size(); i++) {
                bio_gripper_joint_state_msg_.name[i] = prefix + bio_gripper_joint_state_msg_.name[i];
            }
        }

        bio_gripper_action_server_ = rclcpp_action::create_server<control_msgs::action::GripperCommand>(
            node_, prefix + "bio_gripper/gripper_action",
            BIND_CLS_CB(&XArmDriver::_handle_bio_gripper_action_goal),
            BIND_CLS_CB_1(&XArmDriver::_handle_bio_gripper_action_cancel),
            BIND_CLS_CB_1(&XArmDriver::_handle_bio_gripper_action_accepted));
        
        bool add_bio_gripper;
        node_->get_parameter_or("add_bio_gripper", add_bio_gripper, false);
        if (add_bio_gripper) {
            bio_gripper_init_loop_ = false;
            std::thread([this]() {
                float cur_pos;
                int ret = arm->get_bio_gripper_position(&cur_pos);
                while (ret == 0 && !bio_gripper_init_loop_)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    _pub_bio_gripper_joint_states(cur_pos);
                }
            }).detach();
        }
    }

    inline float XArmDriver::_bio_gripper_pos_convert(float pos, bool reversed)
    {
        if (reversed) {
            return fabs(pos * 1000 * 2 + 50);
        }
        else {
            return -fabs(pos - 50) / 1000 / 2;
        }
    }

    void XArmDriver::_pub_bio_gripper_joint_states(float pos)
    {
        bio_gripper_joint_state_msg_.header.stamp = node_->get_clock()->now();
        float p = _bio_gripper_pos_convert(pos);
        bio_gripper_joint_state_msg_.position[0] = p;
        bio_gripper_joint_state_msg_.position[1] = -p;
        pub_joint_state(bio_gripper_joint_state_msg_);
    }

    rclcpp_action::GoalResponse XArmDriver::_handle_bio_gripper_action_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const control_msgs::action::GripperCommand::Goal> goal)
    {
        // RCLCPP_INFO(node_->get_logger(), "Received gripper move goal request, target_pulse=%f, pulse_speed=%f", goal->target_pulse, goal->pulse_speed);
        RCLCPP_INFO(node_->get_logger(), "Received bio gripper move goal request, position=%f, max_effort=%f", goal->command.position, goal->command.max_effort);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse XArmDriver::_handle_bio_gripper_action_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel bio gripper move goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void XArmDriver::_handle_bio_gripper_action_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle)
    {
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{BIND_CLS_CB_1(&XArmDriver::_bio_gripper_action_execute), goal_handle}.detach();
    }

    void XArmDriver::_bio_gripper_action_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle)
    {
        bio_gripper_init_loop_ = true;
        const auto goal = goal_handle->get_goal();
        RCLCPP_INFO(node_->get_logger(), "bio_gripper_action_execute, position=%f, max_effort=%f", goal->command.position, goal->command.max_effort);
        
        int ret;
        float cur_pos = 0;
        int err = 0;
        ret = arm->get_bio_gripper_error(&err);
        if (ret != 0 || err != 0) {
            arm->clean_bio_gripper_error();
            ret = arm->get_bio_gripper_error(&err);
        }
        if (ret != 0 || err != 0) {
            try {
                goal_handle->canceled(bio_gripper_result_);
            } catch (std::exception &e) {
                RCLCPP_ERROR(node_->get_logger(), "bio goal_handle canceled exception, ex=%s", e.what());    
            }
            RCLCPP_ERROR(node_->get_logger(), "get_bio_gripper_error, ret=%d, err=%d", ret, err);
            return;
        }
        ret = arm->get_bio_gripper_position(&cur_pos);
        _pub_bio_gripper_joint_states(cur_pos);

        ret = arm->set_bio_gripper_enable(true);
        if (ret != 0) {
            bio_gripper_result_->position = _bio_gripper_pos_convert(cur_pos);
            try {
                goal_handle->canceled(bio_gripper_result_);
            } catch (std::exception &e) {
                RCLCPP_ERROR(node_->get_logger(), "bio goal_handle canceled exception, ex=%s", e.what()); 
            }
            ret = arm->get_bio_gripper_error(&err);
            RCLCPP_WARN(node_->get_logger(), "set_bio_gripper_enable, ret=%d, err=%d, cur_pos=%f", ret, err, cur_pos);
            return;
        }
        ret = arm->set_bio_gripper_speed(bio_gripper_speed_);
        if (ret != 0) {
            bio_gripper_result_->position = _bio_gripper_pos_convert(cur_pos);
            try {
                goal_handle->canceled(bio_gripper_result_);
            } catch (std::exception &e) {
                RCLCPP_ERROR(node_->get_logger(), "bio goal_handle canceled exception, ex=%s", e.what()); 
            }
            ret = arm->get_bio_gripper_error(&err);
            RCLCPP_WARN(node_->get_logger(), "set_bio_gripper_speed, ret=%d, err=%d, cur_pos=%f", ret, err, cur_pos);
            return;
        }
        float last_pos = -bio_gripper_max_pos_;
        float target_pos = _bio_gripper_pos_convert(goal->command.position, true);
        bool is_move = true;
        std::thread([this, &target_pos, &is_move, &cur_pos]() {
            is_move = true;
            int ret2;
            if (target_pos >= 100)
                ret2 = arm->open_bio_gripper(true, 5, false); // set wait_motion=false
            else
                ret2 = arm->close_bio_gripper(true, 5, false); // set wait_motion=false
            int err;
            arm->get_bio_gripper_error(&err);
            RCLCPP_INFO(node_->get_logger(), "set_bio_gripper_position, ret=%d, err=%d, cur_pos=%f", ret2, err, cur_pos);
            is_move = false;
        }).detach();
        int cnt = 0;
        bool is_succeed = false;
        auto sltime = std::chrono::nanoseconds(1000000000 / bio_gripper_frequency_);
        while (is_move && rclcpp::ok())
        {
            std::this_thread::sleep_for(sltime);
            ret = arm->get_bio_gripper_position(&cur_pos);
            if (ret == 0) {
                if (!is_succeed) {
                    if (fabs(last_pos - cur_pos) < bio_gripper_threshold_) {
                        cnt += 1;
                        if (cnt >= bio_gripper_threshold_times_ && fabs(target_pos - cur_pos) < 15) {
                            bio_gripper_result_->position = _bio_gripper_pos_convert(cur_pos);
                            try {
                                goal_handle->succeed(bio_gripper_result_);
                            } catch (std::exception &e) {
                                RCLCPP_ERROR(node_->get_logger(), "bio goal_handle succeed exception, ex=%s", e.what()); 
                            }
                            is_succeed = true;
                        }
                    }
                    else {
                        cnt = 0;
                        last_pos = cur_pos;
                    }
                }
                bio_gripper_feedback_->position = _bio_gripper_pos_convert(cur_pos);
                try {
                    goal_handle->publish_feedback(bio_gripper_feedback_);
                } catch (std::exception &e) {
                    RCLCPP_ERROR(node_->get_logger(), "bio goal_handle publish_feedback exception, ex=%s", e.what());
                }
                _pub_bio_gripper_joint_states(cur_pos);
            }
            // if (goal_handle->is_canceling()) {
            //     bio_gripper_result_->position = _bio_gripper_pos_convert(cur_pos);
            //     goal_handle->canceled(bio_gripper_result_);
            //     RCLCPP_INFO(this->get_logger(), "Goal canceled, cur_pos=%f", cur_pos);
            //     return;
            // }
        }
        arm->get_bio_gripper_position(&cur_pos);
        RCLCPP_INFO(node_->get_logger(), "bio move finish, cur_pos=%f", cur_pos);
        if (rclcpp::ok() && !is_succeed) {
            bio_gripper_result_->position = _bio_gripper_pos_convert(cur_pos);
            try {
                goal_handle->succeed(bio_gripper_result_);
            } catch (std::exception &e) {
                RCLCPP_ERROR(node_->get_logger(), "bio goal_handle succeed exception, ex=%s", e.what());
            }
            RCLCPP_INFO(node_->get_logger(), "bio Goal succeeded");
        }
    }

    void XArmDriver::pub_robot_msg(xarm_msgs::msg::RobotMsg &rm_msg)
    {
        robot_state_pub_->publish(rm_msg);
    }
    
    void XArmDriver::pub_joint_state(sensor_msgs::msg::JointState &js_msg)
    {
        joint_state_pub_->publish(js_msg);
    }

    void XArmDriver::pub_cgpio_state(xarm_msgs::msg::CIOState &cio_msg)
    {
        cgpio_state_pub_->publish(cio_msg);
    }

    void XArmDriver::pub_ftsensor_ext_state(geometry_msgs::msg::WrenchStamped &wrench_msg)
    {
        ftsensor_ext_state_pub_->publish(wrench_msg);
    }

    void XArmDriver::pub_ftsensor_raw_state(geometry_msgs::msg::WrenchStamped &wrench_msg)
    {
        ftsensor_raw_state_pub_->publish(wrench_msg);
    }

    bool XArmDriver::is_connected(void) {
        return arm == NULL ? false : arm->is_connected();
    }

    std::string XArmDriver::controller_error_interpreter(int err)
    {
        err = (err==-1) ? curr_err : err;
        switch(err)
        {
            case 0:
                return "Everything OK";
            case 1:
                return "Hardware Emergency STOP effective";
            case 2:
                return "Emergency IO of Control Box is triggered";
            case 3:
                return "Emergency Stop of Three-state Switch triggered";
            case 11:
            case 12:
            case 13:
            case 14:
            case 15:
            case 16:
            case 17:
                return std::string("Servo Motor Error of Joint ") + std::to_string(err-10); 
            case 19:
                return "End Module Communication Error";
            case 21:
                return "Kinematic Error";
            case 22:
                return "Self-collision Error";
            case 23:
                return "Joint Angle Exceed Limit";
            case 24:
                return "Speed Exceeds Limit";
            case 25:
                return "Planning Error";
            case 26:
                return "System Real Time Error";
            case 27:
                return "Command Reply Error";
            case 29:
                return "Other Errors, please contact technical support";
            case 30:
                return "Feedback Speed Exceeds limit";
            case 31:
                return "Collision Caused Abnormal Joint Current";
            case 32:
                return "Circle Calculation Error";
            case 33:
                return "Controller GPIO Error";
            case 34:
                return "Trajectory Recording Timeout";
            case 35:
                return "Exceed Safety Boundary";
            case 36:
                return "Number of Delayed Command Exceed Limit";
            case 37:
                return "Abnormal Motion in Manual Mode";
            case 38: 
                return "Abnormal Joint Angle";
            case 39:
                return "Abnormal Communication Between Master and Slave IC of Power Board";
            case 50:
                return "Tool Force/Torque Sensor Error";
            case 51:
                return "Tool Force Torque Sensor Mode Setting Error";
            case 52:
                return "Tool Force Torque Sensor Zero Setting Error";
            case 53:
                return "Tool Force Torque Sensor Overload";
            case 110:
                return "Robot Arm Base Board Communication Error";
            case 111:
                return "Control Box External RS485 Device Communication Error";

            default:
                return "Abnormal Error Code, please contact support!";
        }
    }

}
