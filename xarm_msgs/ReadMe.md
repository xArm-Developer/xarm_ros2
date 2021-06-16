# Description of the data structure used by xarm_ros2


## msg
- [xarm_msgs::msg::RobotMsg](./msg/RobotMsg.msg)
    - xarm_api->topic: __xarm_states__
- [xarm_msgs::msg::CIOState](./msg/CIOState.msg)
    - xarm_api->topic: __xarm_cgpio_states__
- [sensor_msgs::msg::JointState](http://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html)
    - xarm_api->topic: __joint_states__

## srv

- [xarm_msgs::srv::Call](./srv/Call.srv)
    - xarm_api->service: __clean_error__
    - xarm_api->service: __clean_warn__
    - xarm_api->service: __clean_conf__
    - xarm_api->service: __save_conf__
    - xarm_api->service: __reload_dynamics__
    - xarm_api->service: __set_counter_reset__
    - xarm_api->service: __set_counter_increase__
    - xarm_api->service: __clean_gripper_error__
    - xarm_api->service: __clean_bio_gripper_error__
    - xarm_api->service: __start_record_trajectory__
    - xarm_api->service: __stop_record_trajectory__

- [xarm_msgs::srv::GetInt16](./srv/GetInt16.srv)
    - xarm_api->service: __get_state__
    - xarm_api->service: __get_cmdnum__
    - xarm_api->service: __get_vacuum_gripper__
    - xarm_api->service: __get_gripper_err_code__
    - xarm_api->service: __get_bio_gripper_status__
    - xarm_api->service: __get_bio_gripper_error__

- [xarm_msgs::srv::SetInt16](./srv/SetInt16.srv)
    - xarm_api->service: __set_mode__
    - xarm_api->service: __set_state__
    - xarm_api->service: __set_collision_sensitivity__
    - xarm_api->service: __set_teach_sensitivity__
    - xarm_api->service: __set_gripper_mode__
    - xarm_api->service: __set_gripper_enable__
    - xarm_api->service: __set_tgpio_modbus_timeout__
    - xarm_api->service: __set_bio_gripper_speed__
    - xarm_api->service: __set_fence_mode__
    - xarm_api->service: __set_reduced_mode__
    - xarm_api->service: __set_self_collision_detection__
    - xarm_api->service: __set_simulation_robot__

- [xarm_msgs::srv::SetInt16ById](./srv/SetInt16ById.srv)
    - xarm_api->service: __motion_enable__
    - xarm_api->service: __set_servo_attach__
    - xarm_api->service: __set_servo_detach__

- [xarm_msgs::srv::SetInt16List](./srv/SetInt16List.srv)
    - xarm_api->service: __set_reduced_tcp_boundary__

- [xarm_msgs::srv::GetInt32](./srv/GetInt32.srv)
    - xarm_api->service: __get_tgpio_modbus_baudrate__

- [xarm_msgs::srv::SetInt32](./srv/SetInt32.srv)
    - xarm_api->service: __set_tgpio_modbus_baudrate__

- [xarm_msgs::srv::GetFloat32](./srv/GetFloat32.srv)
    - xarm_api->service: __get_gripper_position__

- [xarm_msgs::srv::GetFloat32List](./srv/GetFloat32List.srv)
    - xarm_api->service: __get_position__
    - xarm_api->service: __get_servo_angle__
    - xarm_api->service: __get_position_aa__

- [xarm_msgs::srv::SetFloat32](./srv/SetFloat32.srv)
    - xarm_api->service: __set_pause_time__
    - xarm_api->service: __set_tcp_jerk__
    - xarm_api->service: __set_tcp_maxacc__
    - xarm_api->service: __set_joint_jerk__
    - xarm_api->service: __set_joint_maxacc__
    - xarm_api->service: __set_gripper_speed__
    - xarm_api->service: __set_reduced_max_tcp_speed__
    - xarm_api->service: __set_reduced_max_joint_speed__

- [xarm_msgs::srv::SetFloat32List](./srv/SetFloat32List.srv)
    - xarm_api->service: __set_gravity_direction__
    - xarm_api->service: __set_tcp_offset__
    - xarm_api->service: __set_world_offset__
    - xarm_api->service: __set_reduced_joint_range__

- [xarm_msgs::srv::SetTcpLoad](./srv/SetTcpLoad.srv)
    - xarm_api->service: __set_tcp_load__

- [xarm_msgs::srv::MoveCartesian](./srv/MoveCartesian.srv)
    - xarm_api->service: __set_position__
    - xarm_api->service: __set_tool_position__
    - xarm_api->service: __set_position_aa__
    - xarm_api->service: __set_servo_cartesian__
    - xarm_api->service: __set_servo_cartesian_aa__

- [xarm_msgs::srv::MoveJoint](./srv/MoveJoint.srv)
    - xarm_api->service: __set_servo_angle__
    - xarm_api->service: __set_servo_angle_j__

- [xarm_msgs::srv::MoveCircle](./srv/MoveCircle.srv)
    - xarm_api->service: __move_circle__

- [xarm_msgs::srv::MoveHome](./srv/MoveHome.srv)
    - xarm_api->service: __move_gohome__

- [xarm_msgs::srv::MoveVelocity](./srv/MoveVelocity.srv)
    - xarm_api->service: __vc_set_joint_velocity__
    - xarm_api->service: __vc_set_cartesian_velocity__

- [xarm_msgs::srv::GetDigitalIO](./srv/GetDigitalIO.srv)
    - xarm_api->service: __get_tgpio_digital__
    - xarm_api->service: __get_cgpio_digital__

- [xarm_msgs::srv::GetAnalogIO](./srv/GetAnalogIO.srv)
    - xarm_api->service: __get_tgpio_analog__
    - xarm_api->service: __get_cgpio_analog__

- [xarm_msgs::srv::SetDigitalIO](./srv/SetDigitalIO.srv)
    - xarm_api->service: __set_tgpio_digital__
    - xarm_api->service: __set_cgpio_digital__
    - xarm_api->service: __set_tgpio_digital_with_xyz__
    - xarm_api->service: __set_cgpio_digital_with_xyz__

- [xarm_msgs::srv::SetAnalogIO](./srv/SetAnalogIO.srv)
    - xarm_api->service: __set_cgpio_analog__
    - xarm_api->service: __set_cgpio_analog_with_xyz__

- [xarm_msgs::srv::VacuumGripperCtrl](./srv/VacuumGripperCtrl.srv)
    - xarm_api->service: __set_vacuum_gripper__

- [xarm_msgs::srv::GripperMove](./srv/GripperMove.srv)
    - xarm_api->service: __set_gripper_position__

- [xarm_msgs::srv::BioGripperEnable](./srv/BioGripperEnable.srv)
    - xarm_api->service: __set_bio_gripper_enable__

- [xarm_msgs::srv::BioGripperCtrl](./srv/BioGripperCtrl.srv)
    - xarm_api->service: __open_bio_gripper__
    - xarm_api->service: __close_bio_gripper__

- [xarm_msgs::srv::RobotiqReset](./srv/RobotiqReset.srv)
    - xarm_api->service: __robotiq_reset__

- [xarm_msgs::srv::RobotiqActivate](./srv/RobotiqActivate.srv)
    - xarm_api->service: __robotiq_set_activate__

- [xarm_msgs::srv::RobotiqMove](./srv/RobotiqMove.srv)
    - xarm_api->service: __robotiq_set_position__
    - xarm_api->service: __robotiq_open__
    - xarm_api->service: __robotiq_close__

- [xarm_msgs::srv::RobotiqGetStatus](./srv/RobotiqGetStatus.srv)
    - xarm_api->service: __robotiq_get_status__

- [xarm_msgs::srv::GetSetModbusData](./srv/GetSetModbusData.srv)
    - xarm_api->service: __getset_tgpio_modbus_data__

- [xarm_msgs::srv::TrajCtrl](./srv/TrajCtrl.srv)
    - xarm_api->service: __save_record_trajectory__
    - xarm_api->service: __load_trajectory__

- [xarm_msgs::srv::TrajPlay](./srv/TrajPlay.srv)
    - xarm_api->service: __playback_trajectory__

- [xarm_msgs::srv::PlanPose](./srv/PlanPose.srv)
    - xarm_planner->service: __xarm_pose_plan__
- [xarm_msgs::srv::PlanJoint](./srv/PlanJoint.srv)
    - xarm_planner->service: __xarm_joint_plan__
- [xarm_msgs::srv::PlanSingleStraight](./srv/PlanSingleStraight.srv)
    - xarm_planner->service: __xarm_straight_plan__
- [xarm_msgs::srv::PlanExec](./srv/PlanExec.srv)
    - xarm_planner->service: __xarm_exec_plan__


## action
- [control_msgs::action::GripperCommand](http://docs.ros.org/en/api/control_msgs/html/action/GripperCommand.html)
    - - xarm_api->action: __xarm_gripper/gripper_action__

