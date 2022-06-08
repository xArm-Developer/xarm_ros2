#ifndef __XARM_MSGS_H
#define __XARM_MSGS_H

#include <xarm_msgs/msg/robot_msg.hpp>
#include <xarm_msgs/msg/io_state.hpp>
#include <xarm_msgs/msg/cio_state.hpp>

#include <xarm_msgs/srv/bio_gripper_ctrl.hpp>
#include <xarm_msgs/srv/bio_gripper_enable.hpp>
#include <xarm_msgs/srv/call.hpp>
#include <xarm_msgs/srv/get_analog_io.hpp>
#include <xarm_msgs/srv/get_digital_io.hpp>
#include <xarm_msgs/srv/get_float32.hpp>
#include <xarm_msgs/srv/get_float32_list.hpp>
#include <xarm_msgs/srv/get_int16.hpp>
#include <xarm_msgs/srv/get_int16_list.hpp>
#include <xarm_msgs/srv/get_int32.hpp>
#include <xarm_msgs/srv/get_int32_by_type.hpp>
#include <xarm_msgs/srv/get_set_modbus_data.hpp>
#include <xarm_msgs/srv/gripper_move.hpp>
#include <xarm_msgs/srv/move_cartesian.hpp>
#include <xarm_msgs/srv/move_circle.hpp>
#include <xarm_msgs/srv/move_home.hpp>
#include <xarm_msgs/srv/move_joint.hpp>
#include <xarm_msgs/srv/move_velocity.hpp>
#include <xarm_msgs/srv/robotiq_activate.hpp>
#include <xarm_msgs/srv/robotiq_get_status.hpp>
#include <xarm_msgs/srv/robotiq_move.hpp>
#include <xarm_msgs/srv/robotiq_reset.hpp>
#include <xarm_msgs/srv/set_analog_io.hpp>
#include <xarm_msgs/srv/set_digital_io.hpp>
#include <xarm_msgs/srv/set_float32.hpp>
#include <xarm_msgs/srv/set_float32_list.hpp>
#include <xarm_msgs/srv/set_tcp_load.hpp>
#include <xarm_msgs/srv/set_int16.hpp>
#include <xarm_msgs/srv/set_int16_by_id.hpp>
#include <xarm_msgs/srv/set_int16_list.hpp>
#include <xarm_msgs/srv/set_int32.hpp>
#include <xarm_msgs/srv/set_int32_by_type.hpp>
#include <xarm_msgs/srv/traj_ctrl.hpp>
#include <xarm_msgs/srv/traj_play.hpp>
#include <xarm_msgs/srv/vacuum_gripper_ctrl.hpp>

inline std::string uf_controller_error_interpreter(int err)
{
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
        case 11 ... 17:
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

#endif // __XARM_MSGS_H