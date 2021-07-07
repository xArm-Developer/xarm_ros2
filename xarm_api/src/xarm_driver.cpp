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


void* cmd_heart_beat(void* args)
{
    xarm_api::XArmDriver *my_driver = (xarm_api::XArmDriver *) args;
    int cmdnum;
    int cnt = 0;
    int max_cnt = CMD_HEARTBEAT_SEC * 2;
    while(my_driver->arm->is_connected())
    {
        sleep_milliseconds(500);
        cnt += 1;
        if (cnt >= max_cnt) {
            cnt = 0;
            my_driver->arm->get_cmdnum(&cmdnum);
        }
    }
    RCLCPP_ERROR(my_driver->get_logger(), "xArm Control Connection Failed! Please Shut Down (Ctrl-C) and Retry ...");
    return (void*)0;
}

namespace xarm_api
{   
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
        curr_state_ = report_data_ptr->state;
        curr_err_ = report_data_ptr->err;
    }

    void XArmDriver::init(rclcpp::Node::SharedPtr& node, std::string &server_ip)
    {
        node_ = node;
        std::string prefix = "";
        node_->get_parameter_or("prefix", prefix, std::string(""));
        std::string hw_ns;
        node_->get_parameter_or("hw_ns", hw_ns, std::string("xarm"));
        hw_ns = prefix + hw_ns;
        hw_node_ = node->create_sub_node(hw_ns);
        node_->get_parameter_or("dof", dof_, 7);
        node_->get_parameter_or("report_type", report_type_, std::string("normal"));
        
        arm = new XArmAPI(
            server_ip, 
            true, // is_radian
            false, // do_not_open
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
        arm->release_connect_changed_callback(true);
        arm->release_report_data_callback(true);
        // arm->register_connect_changed_callback(std::bind(&XArmDriver::_report_connect_changed_callback, this, std::placeholders::_1, std::placeholders::_2));
        arm->register_report_data_callback(std::bind(&XArmDriver::_report_data_callback, this, std::placeholders::_1));
        // arm->connect();

        int err_warn[2] = {0};
        int ret = arm->get_err_warn_code(err_warn);
        if (err_warn[0] != 0) {
            RCLCPP_WARN(node_->get_logger(), "xArmErrorCode: %d", err_warn[0]);
        }
        
        std::thread th(cmd_heart_beat, this);
        th.detach();
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

        _init_publisher();
        _init_service();
        _init_gripper();
    }

    void XArmDriver::_init_publisher()
    {
        joint_state_pub_ = hw_node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        robot_state_pub_ = hw_node_->create_publisher<xarm_msgs::msg::RobotMsg>("xarm_states", 10);
        cgpio_state_pub_ = hw_node_->create_publisher<xarm_msgs::msg::CIOState>("xarm_cgpio_states", 10);
    }

    void XArmDriver::_init_gripper()
    {
        node_->get_parameter_or("xarm_gripper.speed", gripper_speed_, 2000);  // 机械爪速度
        node_->get_parameter_or("xarm_gripper.max_pos", gripper_max_pos_, 850); // 机械爪最大值，用来转换
        node_->get_parameter_or("xarm_gripper.frequency", gripper_frequency_, 10); // 发送机械爪位置后查询机械爪位置的频率
        node_->get_parameter_or("xarm_gripper.threshold", gripper_threshold_, 3); // 检测机械爪当前位置和上一次位置的差值如果小于当前值，则认为机械爪没动
        node_->get_parameter_or("xarm_gripper.threshold_times", gripper_threshold_times_, 10); // 如果检测到机械爪没动的次数超过此值且当前位置和目标位置差值不超过15，则认为机械爪运动成功
        RCLCPP_INFO(node_->get_logger(), "gripper_speed: %d, gripper_max_pos: %d, gripper_frequency : %d, gripper_threshold: %d, gripper_threshold_times: %d", 
            gripper_speed_, gripper_max_pos_, gripper_frequency_, gripper_threshold_, gripper_threshold_times_);

        gripper_feedback_ = std::make_shared<control_msgs::action::GripperCommand::Feedback>();
        gripper_result_ = std::make_shared<control_msgs::action::GripperCommand::Result>();;
        gripper_joint_state_msg_.header.stamp = node_->get_clock()->now();
        gripper_joint_state_msg_.header.frame_id = "gripper-joint-state data";        
        gripper_joint_state_msg_.name.resize(6);
        gripper_joint_state_msg_.position.resize(6, std::numeric_limits<double>::quiet_NaN());
        gripper_joint_state_msg_.velocity.resize(6, std::numeric_limits<double>::quiet_NaN());
        gripper_joint_state_msg_.effort.resize(6, std::numeric_limits<double>::quiet_NaN());
        node_->get_parameter_or("xarm_gripper.joint_names", gripper_joint_state_msg_.name, 
            std::vector<std::string>({"drive_joint", "left_finger_joint", "left_inner_knuckle_joint", "right_outer_knuckle_joint", "right_finger_joint", "right_inner_knuckle_joint"}));
        
        std::string prefix = "";
        node_->get_parameter_or("prefix", prefix, std::string(""));
        if (prefix != "") {
            for (int i = 0; i < gripper_joint_state_msg_.name.size(); i++) {
                gripper_joint_state_msg_.name[i] = prefix + gripper_joint_state_msg_.name[i];
            }
        }

        gripper_action_server_ = rclcpp_action::create_server<control_msgs::action::GripperCommand>(
            node_, prefix + "xarm_gripper/gripper_action",
            BIND_CLS_CB(&XArmDriver::_handle_gripper_action_goal),
            BIND_CLS_CB_1(&XArmDriver::_handle_gripper_action_cancel),
            BIND_CLS_CB_1(&XArmDriver::_handle_gripper_action_accepted));
        
        bool add_gripper;
        node_->get_parameter_or("add_gripper", add_gripper, false);
        if (add_gripper) {
            gripper_init_loop_ = false;
            std::thread([this]() {
                float cur_pos;
                int ret = arm->get_gripper_position(&cur_pos);
                while (ret == 0 && !gripper_init_loop_)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    _pub_gripper_joint_states(cur_pos);
                }
            }).detach();
        }
    }

    inline float XArmDriver::_gripper_pos_convert(float pos, bool reversed)
    {
        if (reversed) {
            return fabs(gripper_max_pos_ - pos * 1000);
        }
        else {
            return fabs(gripper_max_pos_ - pos) / 1000;
        }
    }

    void XArmDriver::_pub_gripper_joint_states(float pos)
    {
        gripper_joint_state_msg_.header.stamp = node_->get_clock()->now();
        float p = _gripper_pos_convert(pos);
        for (int i = 0; i < 6; i++) {
            gripper_joint_state_msg_.position[i] = p;
        }
        pub_joint_state(gripper_joint_state_msg_);
    }

    rclcpp_action::GoalResponse XArmDriver::_handle_gripper_action_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const control_msgs::action::GripperCommand::Goal> goal)
    {
        // RCLCPP_INFO(node_->get_logger(), "Received gripper move goal request, target_pulse=%f, pulse_speed=%f", goal->target_pulse, goal->pulse_speed);
        RCLCPP_INFO(node_->get_logger(), "Received gripper move goal request, position=%f, max_effort=%f", goal->command.position, goal->command.max_effort);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse XArmDriver::_handle_gripper_action_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel gripper move goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void XArmDriver::_handle_gripper_action_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle)
    {
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{BIND_CLS_CB_1(&XArmDriver::_gripper_action_execute), goal_handle}.detach();
    }

    void XArmDriver::_gripper_action_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>> goal_handle)
    {
        gripper_init_loop_ = true;
        const auto goal = goal_handle->get_goal();
        RCLCPP_INFO(node_->get_logger(), "gripper_action_execute, position=%f, max_effort=%f", goal->command.position, goal->command.max_effort);
        
        int ret;
        float cur_pos = 0;
        int err = 0;
        ret = arm->get_gripper_err_code(&err);
        if (err != 0) {
            goal_handle->canceled(gripper_result_);
            RCLCPP_ERROR(node_->get_logger(), "get_gripper_err_code, ret=%d, err=%d", ret, err);
            return;
        }
        ret = arm->get_gripper_position(&cur_pos);
        _pub_gripper_joint_states(cur_pos);

        ret = arm->set_gripper_mode(0);
        if (ret != 0) {
            gripper_result_->position = _gripper_pos_convert(cur_pos);
            goal_handle->canceled(gripper_result_);
            ret = arm->get_gripper_err_code(&err);
            RCLCPP_WARN(node_->get_logger(), "set_gripper_mode, ret=%d, err=%d, cur_pos=%f", ret, err, cur_pos);
            return;
        }
        ret = arm->set_gripper_enable(true);
        if (ret != 0) {
            gripper_result_->position = _gripper_pos_convert(cur_pos);
            goal_handle->canceled(gripper_result_);
            ret = arm->get_gripper_err_code(&err);
            RCLCPP_WARN(node_->get_logger(), "set_gripper_enable, ret=%d, err=%d, cur_pos=%f", ret, err, cur_pos);
            return;
        }
        ret = arm->set_gripper_speed(gripper_speed_);
        if (ret != 0) {
            gripper_result_->position = _gripper_pos_convert(cur_pos);
            goal_handle->canceled(gripper_result_);
            ret = arm->get_gripper_err_code(&err);
            RCLCPP_WARN(node_->get_logger(), "set_gripper_speed, ret=%d, err=%d, cur_pos=%f", ret, err, cur_pos);
            return;
        }
        float last_pos = -gripper_max_pos_;
        float target_pos = _gripper_pos_convert(goal->command.position, true);
        bool is_move = true;
        std::thread([this, &target_pos, &is_move, &cur_pos]() {
            is_move = true;
            int ret2 = arm->set_gripper_position(target_pos, true);
            int err;
            arm->get_gripper_err_code(&err);
            RCLCPP_INFO(node_->get_logger(), "set_gripper_position, ret=%d, err=%d, cur_pos=%f", ret2, err, cur_pos);
            is_move = false;
        }).detach();
        int cnt = 0;
        bool is_succeed = false;
        auto sltime = std::chrono::nanoseconds(1000000000 / gripper_frequency_);
        while (is_move && rclcpp::ok())
        {
            std::this_thread::sleep_for(sltime);
            ret = arm->get_gripper_position(&cur_pos);
            if (ret == 0) {
                if (!is_succeed) {
                    if (fabs(last_pos - cur_pos) < gripper_threshold_) {
                        cnt += 1;
                        if (cnt >= gripper_threshold_times_ && fabs(target_pos - cur_pos) < 15) {
                            gripper_result_->position = _gripper_pos_convert(cur_pos);
                            goal_handle->succeed(gripper_result_);
                            is_succeed = true;
                        }
                    }
                    else {
                        cnt = 0;
                        last_pos = cur_pos;
                    }
                }
                gripper_feedback_->position = _gripper_pos_convert(cur_pos);
                goal_handle->publish_feedback(gripper_feedback_);
                _pub_gripper_joint_states(cur_pos);
            }
            // if (goal_handle->is_canceling()) {
            //     gripper_result_->position = _gripper_pos_convert(cur_pos);
            //     goal_handle->canceled(gripper_result_);
            //     RCLCPP_INFO(this->get_logger(), "Goal canceled, cur_pos=%f", cur_pos);
            //     return;
            // }
        }
        arm->get_gripper_position(&cur_pos);
        RCLCPP_INFO(node_->get_logger(), "move finish, cur_pos=%f", cur_pos);
        if (rclcpp::ok() && !is_succeed) {
            gripper_result_->position = _gripper_pos_convert(cur_pos);
            goal_handle->succeed(gripper_result_);
            RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
        }
    }

    void XArmDriver::pub_robot_msg(xarm_msgs::msg::RobotMsg &rm_msg)
    {
        curr_err_ = rm_msg.err;
        curr_state_ = rm_msg.state;
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
}
