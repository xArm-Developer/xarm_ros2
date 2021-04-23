/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
           Vinman <vinman.cub@gmail.com>
 ============================================================================*/
#include <signal.h>
#include "xarm_api/xarm_driver.h"


void exit_sig_handler(int signum)
{
    fprintf(stderr, "[xarm_driver] Ctrl-C caught, exit process...\n");
    exit(-1);
}

class XArmDriverRunner
{
    public:
        XArmDriverRunner(rclcpp::Node::SharedPtr& node, std::string &server_ip)
        {
            node_ = node;
            node->get_parameter_or("DOF", joint_num_, 7);
            node->get_parameter_or("xarm_report_type", report_type_, std::string("normal"));
            node->get_parameter_or("joint_names", joint_name_, 
                std::vector<std::string>({"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"}));
            
            ROS_INFO("robot_ip=%s, report_type=%s, dof=%d", server_ip.c_str(), report_type_.c_str(), joint_num_);

            is_first_cycle_ = true;
            prev_angle_ = new double [joint_num_];

            xarm_driver_.init(node, server_ip);
            xarm_driver_.arm->register_connect_changed_callback(std::bind(&XArmDriverRunner::_report_connect_changed_callback, this, std::placeholders::_1, std::placeholders::_2));
            xarm_driver_.arm->register_report_data_callback(std::bind(&XArmDriverRunner::_report_data_callback, this, std::placeholders::_1));
        }
        void _report_connect_changed_callback(bool connected, bool reported)
        {
            ROS_INFO("[TCP STATUS] CONTROL: %d, REPORT: %d", connected, reported);
            if (!reported) is_first_cycle_ = true;
        }

        void _report_data_callback(XArmReportData *report_data_ptr)
        {
            // ROS_INFO("[2] state: %d, error_code: %d", report_data_ptr->state, report_data_ptr->err);
            last_now_ = now_;
            now_ = node_->now();
            double elapsed = (now_.nanoseconds() - last_now_.nanoseconds()) / 1e9;
            joint_state_msg_.header.stamp = now_;
            joint_state_msg_.header.frame_id = "joint-state data";
            joint_state_msg_.name.resize(joint_num_);
            joint_state_msg_.position.resize(joint_num_);
            joint_state_msg_.velocity.resize(joint_num_);
            joint_state_msg_.effort.resize(joint_num_);
            for(int i = 0; i < joint_num_; i++)
            {
                double d = (double)report_data_ptr->angle[i];
                joint_state_msg_.name[i] = joint_name_[i];
                joint_state_msg_.position[i] = d;

                if (is_first_cycle_)
                {
                    joint_state_msg_.velocity[i] = 0;
                    is_first_cycle_ = false;
                }
                else
                {
                    joint_state_msg_.velocity[i] = (joint_state_msg_.position[i] - prev_angle_[i]) / elapsed;
                }

                joint_state_msg_.effort[i] = (double)report_data_ptr->tau[i];

                prev_angle_[i] = d;
            }

            xarm_driver_.pub_joint_state(joint_state_msg_);

            xarm_state_msg_.state = report_data_ptr->state;
            xarm_state_msg_.mode = report_data_ptr->mode;
            xarm_state_msg_.cmdnum = report_data_ptr->cmdnum;
            xarm_state_msg_.err = report_data_ptr->err;
            xarm_state_msg_.warn = report_data_ptr->war;
            xarm_state_msg_.mt_brake = report_data_ptr->mt_brake;
            xarm_state_msg_.mt_able = report_data_ptr->mt_able;
            xarm_state_msg_.angle.resize(joint_num_);

            for(int i = 0; i < joint_num_; i++)
            {
                /* set the float precision*/
                double d = report_data_ptr->angle[i];
                double r;
                char str[8];
                sprintf(str, "%0.3f", d);
                sscanf(str, "%lf", &r);
                xarm_state_msg_.angle[i] = r;
            }
            for(int i = 0; i < 6; i++)
            {
                xarm_state_msg_.pose[i] = report_data_ptr->pose[i];
                xarm_state_msg_.offset[i] = report_data_ptr->tcp_offset[i];
            }
            xarm_driver_.pub_robot_msg(xarm_state_msg_);

            if (report_data_ptr->total_num >= 417) {
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
                xarm_driver_.pub_cgpio_state(cgpio_state_msg_);
            }
        }

    private:
        rclcpp::Node::SharedPtr node_;
        std::string report_type_;
        rclcpp::Time now_, last_now_;
        sensor_msgs::msg::JointState joint_state_msg_;
        xarm_api::XArmDriver xarm_driver_;
        xarm_msgs::msg::RobotMsg xarm_state_msg_;
        xarm_msgs::msg::CIOState cgpio_state_msg_;

        int joint_num_;
        std::vector<std::string> joint_name_;
        bool is_first_cycle_;
        double *prev_angle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("xarm_driver_node");

    ROS_INFO("namespace: %s", node->get_namespace());

    node->declare_parameter("xarm_robot_ip");
    node->declare_parameter("xarm_report_type");
    node->declare_parameter("DOF");
    node->declare_parameter("joint_names");

    std::string robot_ip = "";
    node->get_parameter_or("xarm_robot_ip", robot_ip, robot_ip);
    if (robot_ip == "") {
        ROS_ERROR("No param named 'xarm_robot_ip'");
        exit(1);
    }

    ROS_INFO("xarm_driver_node start");
    XArmDriverRunner xarm_driver_runner(node, robot_ip);

    signal(SIGINT, exit_sig_handler);
    rclcpp::spin(node);
    rclcpp::shutdown();

    ROS_INFO("xarm_driver_node over");

    return 0;
}
