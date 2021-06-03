/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
           Vinman <vinman.cub@gmail.com>
 ============================================================================*/
 
#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include <xarm_api/xarm_ros_client.h>


void exit_sig_handler(int signum)
{
    fprintf(stderr, "[xarm_driver] Ctrl-C caught, exit process...\n");
    exit(-1);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string hw_ns = "xarm";
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("xarm_ros_client");
    RCLCPP_INFO(rclcpp::get_logger("test_xarm_ros_client"), "test_xarm_ros_client start");

    signal(SIGINT, exit_sig_handler);

    xarm_api::XArmROSClient client;
    client.init(node, hw_ns);
    
    client.clean_error();
    client.clean_warn();
    client.motion_enable(true);
    client.set_mode(0);
	client.set_state(0);

	// client.save_conf();
	// client.reload_dynamics();
    // client.set_state(0);
	// client.set_counter_increase();
    // rclcpp::sleep_for(std::chrono::seconds(2));
    // client.set_counter_reset();
    // client.clean_gripper_error();
    // client.clean_bio_gripper_error();

    int tmp_int;
    float tmp_float;
    std::vector<int> tmp_int_vector;
    std::vector<float> tmp_float_vector;

    // client.get_state(&tmp_int);
    // client.get_cmdnum(&tmp_int);
    // client.get_vacuum_gripper(&tmp_int);
    // client.get_gripper_err_code(&tmp_int);
    // client.get_bio_gripper_status(&tmp_int);
    // client.get_bio_gripper_error(&tmp_int);

    // client.get_tgpio_modbus_baudrate(&tmp_int);
    // client.set_tgpio_modbus_baudrate(2000000);
    
    // client.get_err_warn_code(tmp_int_vector);
    
    // client.get_position(tmp_float_vector);
    // client.get_servo_angle(tmp_float_vector);
    // client.get_position_aa(tmp_float_vector);

    // tmp_float_vector.resize(6, 0);
    // tmp_float_vector[0] = 100;
    // client.set_tcp_offset(tmp_float_vector);
    // client.set_world_offset(tmp_float_vector);
    // client.set_tcp_load(0.88, tmp_float_vector);

    // tmp_float_vector.resize(6, 0);
    // tmp_float_vector[0] = 300;
    // tmp_float_vector[1] = 100;
    // tmp_float_vector[2] = 100;
    // tmp_float_vector[3] = 3.14159;
    // tmp_float_vector[4] = 0;
    // tmp_float_vector[5] = 0;
    // client.set_position(tmp_float_vector, true);
    // tmp_float_vector.resize(6, 0);
    // std::fill(tmp_float_vector.begin(), tmp_float_vector.end(), 0);
    // client.set_servo_angle(tmp_float_vector, true);

    // client.get_tgpio_digital(tmp_int_vector);
    // for (int i = 0; i < tmp_int_vector.size(); i++) {
    //     RCLCPP_INFO(node->get_logger(), "tgpio_digital[%d]=%d", i, tmp_int_vector[i]);
    // }
    // client.get_cgpio_digital(tmp_int_vector);
    // for (int i = 0; i < tmp_int_vector.size(); i++) {
    //     RCLCPP_INFO(node->get_logger(), "cgpio_digital[%d]=%d", i, tmp_int_vector[i]);
    // }
    // client.get_tgpio_analog(0, &tmp_float);
    // RCLCPP_INFO(node->get_logger(), "tgpio_ananlog=%f", tmp_float);
    // client.get_cgpio_analog(0, &tmp_float);
    // RCLCPP_INFO(node->get_logger(), "cgpio_ananlog=%f", tmp_float);

    // client.set_gripper_enable(true);
    // client.set_gripper_mode(0);
    // client.set_gripper_speed(3000);
    // client.set_gripper_position(0, true);
    // client.set_gripper_position(800, true);

    RCLCPP_INFO(rclcpp::get_logger("test_xarm_ros_client"), "test_xarm_ros_client over");
    return 0;
}