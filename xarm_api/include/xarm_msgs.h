#ifndef __XARM_MSGS_H
#define __XARM_MSGS_H

#include <xarm_msgs/msg/robot_msg.hpp>
#include <xarm_msgs/msg/io_state.hpp>
#include <xarm_msgs/msg/cio_state.hpp>

#include <xarm_msgs/srv/set_int16.hpp>
#include <xarm_msgs/srv/set_float32.hpp>
#include <xarm_msgs/srv/tcp_offset.hpp>
#include <xarm_msgs/srv/set_load.hpp>
#include <xarm_msgs/srv/set_axis.hpp>
#include <xarm_msgs/srv/move.hpp>
#include <xarm_msgs/srv/set_digital_io.hpp>
#include <xarm_msgs/srv/get_digital_io.hpp>
#include <xarm_msgs/srv/get_controller_digital_io.hpp>
#include <xarm_msgs/srv/set_controller_analog_io.hpp>
#include <xarm_msgs/srv/get_analog_io.hpp>
#include <xarm_msgs/srv/clear_err.hpp>
#include <xarm_msgs/srv/get_err.hpp>
#include <xarm_msgs/srv/gripper_config.hpp>
#include <xarm_msgs/srv/gripper_move.hpp>
#include <xarm_msgs/srv/gripper_state.hpp>
#include <xarm_msgs/srv/set_tool_modbus.hpp>
#include <xarm_msgs/srv/config_tool_modbus.hpp>
#include <xarm_msgs/srv/move_axis_angle.hpp>
#include <xarm_msgs/srv/move_velo.hpp>

#endif // __XARM_MSGS_H