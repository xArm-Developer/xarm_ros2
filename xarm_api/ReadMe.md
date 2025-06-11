# xarm_api/xarm_msgs:
&ensp;&ensp;These two packages provide user with the ros2 service wrapper of the functions in Ufactory SDK. There are various types of motion command (service names) supported，please set correct robot mode first, refer to [this documentation](https://docs.ufactory.cc/support_articles/faq/robot-state-and-mode-explanation) for the Mode/State definitions in our system.

&ensp;&ensp;**Attention**: You need to **enable the corresponding services** in advance if they have not been activated by default in `config/xarm_params.yaml`. Or you can set the `debug` parameter as `true` to enable all services at once.  

### Robot Mode 0:
* <font color=blue>[set_servo_angle](#21-joint-space-motion):</font> joint space point to point command, given target joint angles, max joint velocity and acceleration. Service definition: [xarm_msgs/srv/MoveJoint](../xarm_msgs/srv/MoveJoint.srv). If specify `wait=false` and proper `radius`, motion can be continuous with next motion with Arc blending at the via points, and velocity is also continuous.    

* <font color=blue>[set_position](#22-cartesian-space-motion-in-base-coordinate):</font> straight-line motion to the specified Cartesian Tool Centre Point(TCP) target. Service definition: [xarm_msgs/srv/MoveCartesian](../xarm_msgs/srv/MoveCartesian.srv). If specify `wait=false` and proper `radius`, motion can be continuous with next motion with Arc blending at the via points, and velocity is also continuous. Relative (delta) motion can also be achieved by setting `relative=true`.   

* <font color=blue>[set_tool_position](#23-cartesian-space-motion-in-tool-coordinate):</font> straight-line motion, given target is expressed in **TOOL** coordinate system, can be considered as a relative motion.   

* <font color=blue>[set_position_aa](#24-cartesian-space-motion-in-axis-angle-orientation):</font> straight-line motion, with orientation expressed in **Axis-angle** rather than roll-pitch-yaw angles. Please refer to xArm user manual for detailed explanation of axis-angle before using this command.   


### Robot Mode 1:
* <font color=blue>[set_servo_cartesian/set_servo_cartesian_aa](#31-cartesian-space-servoing)/[set_servo_angle_j](#32-joint-space-servoing):</font> streamed high-frequency `[suggestion: 100 ~ 250Hz fixed]` trajectory command execution in Cartesian space or joint space, this mode is especially useful for external trajectory planner which generates time parameterized servo command (e.g. Moveit+ros-controllers) or sensor guided motion. Special **RISK ASSESMENT** is required before using them and please note there is no further interpolation and speed control in this mode.  

### Robot Mode 4:
* <font color=blue>[vc_set_joint_velocity](#25-joint-velocity-control):</font> Joint motion with specified velocity for each joint (unit: rad/s), with maximum joint acceleration configurable by `set_joint_maxacc` service.  

### Robot Mode 5:
* <font color=blue>[vc_set_cartesian_velocity](#26-cartesian-velocity-control):</font> Linear motion of TCP with specified velocity in mm/s (position) and rad/s (orientation in **axis-angular_velocity**), with maximum linear acceleration configurable by `set_tcp_maxacc` service.  

### Robot Mode 6: (Firmware >= v1.10.0)
* <font color=blue>[set_servo_angle](#21-joint-space-motion):</font> Online joint space replanning to the new joint angles, with new max joint velocity and acceleration. Joint velocities and accelerations are continuous during transition, however the velocity profiles may not be synchronous and the final reached positions may have small errors. **This function is mainly for dynamic response without self trajectory planning requirement like servo joint commands**. `wait` parameter has to be `false` for successful transition. 

### Robot Mode 7: (Firmware >= v1.11.0)
* <font color=blue>[set_position/set_position_aa](#22-cartesian-space-motion-in-Base-coordinate):</font> Online Cartesian space replanning to the new target coordinate, with new max linear velocity and acceleration. Velocities and accelerations are continuous during transition, **This function is mainly for dynamic response without self trajectory planning requirement like servo cartesian commands**. `wait` parameter has to be `false` for successful transition.    

## 1. Starting xArm by ROS service:

&ensp;&ensp;First startup the service server for xarm7, ip address is just an example:  
```bash
$ ros2 launch xarm_api xarm7_driver.launch.py robot_ip:=192.168.1.127 report_type:=normal
```
The argument `report_type` is explained [here](#11-report_type-argument).  

&ensp;&ensp;Then make sure all the servo motors are enabled:
```bash
$ ros2 service call /xarm/motion_enable xarm_msgs/srv/SetInt16ById "{id: 8, data: 1}"
```
&ensp;&ensp;Before any motion commands, set proper robot mode(0: POSE, as example) and state(0: READY) ***in order***:    
```bash
$ ros2 service call /xarm/set_mode xarm_msgs/srv/SetInt16 "{data: 0}"
$ ros2 service call /xarm/set_state xarm_msgs/srv/SetInt16 "{data: 0}"
```

## 2. Joint or Cartesian space command example [use firmware planner]:
&ensp;&ensp;This section will talk about general motion service interface, with firmware assisted planner for speed/acceleration control and smooth trajectory generation. Please note that all the angles for service command must use the unit of ***radian***.   

### 2.1 Joint space motion:
&ensp;&ensp; To call joint space (7DOF) motion with max speed 0.35 rad/s and acceleration 10 rad/s^2:   
```bash
$ ros2 service call /xarm/set_servo_angle xarm_msgs/srv/MoveJoint "{angles: [-0.28, 0, 0, 0, 0, 0, 0], speed: 0.35, acc: 10}"
```
Note: This command is valid for both **Mode 0** and **mode 6**. The difference is: In mode 0 each received command will be **queued** and executed fully in order, whereas in mode 6(must specify `wait` as `false`), the newly received target will take effect immediately and all previous commands will be **discarded**.   

&ensp;&ensp;To go back to home (all joints at 0 rad) position with max speed 0.35 rad/s and acceleration 10 rad/s^2:  
```bash
$ ros2 service call /xarm/move_gohome xarm_msgs/srv/MoveHome "{speed: 0.35, acc: 10}"
```
### 2.2 Cartesian space motion in Base coordinate:
&ensp;&ensp;To call Cartesian motion to the target expressed in robot BASE Coordinate, with max speed 200 mm/s and acceleration 2000 mm/s^2:
```bash
$ ros2 service call /xarm/set_position xarm_msgs/srv/MoveCartesian "{pose: [300, 0, 250, 3.14, 0, 0], speed: 200, acc: 2000}"
```
Note: This command is valid for both **Mode 0** and **mode 7**. The difference is: In mode 0 each received command will be **queued** and executed fully in order, whereas in mode 7(must specify `wait` as `false`), the newly received target will take effect immediately and all previous commands will be **discarded**.   

### 2.3 Cartesian space motion in Tool coordinate:
&ensp;&ensp;To execute Cartesian motion expressed in robot TOOL Coordinate, with max speed 50 mm/s and acceleration 1000 mm/s^2, the following will move a **relative motion** (delta_x=50mm, delta_y=100mm, delta_z=100mm) along the current Tool coordinate, no orientation change:
```bash
$ ros2 service call /xarm/set_tool_position xarm_msgs/srv/MoveCartesian "{pose: [50, 100, 100, 0, 0, 0], speed: 50, acc: 1000}"
```
### 2.4 Cartesian space motion in Axis-angle orientation:
&ensp;&ensp;Please pay attention to two special arguments: "**is_tool_coord**" is `false` for motion with respect to (w.r.t.) Arm base coordinate system, and `true` for motion w.r.t. Tool coordinate system. "**relative**" is `false` for absolute target position w.r.t. specified coordinate system, and `true` for relative target position.  
&ensp;&ensp;For example: to move 1.0 radian relatively around tool-frame Z-axis: 
```bash
$ ros2 service call /xarm/set_position_aa xarm_msgs/srv/MoveCartesian "{pose: [0, 0, 0, 0, 0, 1.0], speed: 50, acc: 1000, is_tool_coord: true}"
```   
&ensp;&ensp;"**mvtime**" is not meaningful in this command, just set it to 0. Another example: in base-frame, to move 122mm relatively along Y-axis, and rotate around X-axis for -0.5 radians:  
```bash
$ ros2 service call /xarm/set_position_aa xarm_msgs/srv/MoveCartesian "{pose: [0, 122, 0, -0.5, 0, 0], speed: 100, acc: 1000, is_tool_coord: false, relative: true}"  
```
### 2.5 Joint velocity control:
&ensp;&ensp;(**xArm controller firmware version >= 1.6.8** required) If controlling joint velocity is desired, first switch to **Mode 4** as descriped in [mode explanation](https://docs.ufactory.cc/support_articles/faq/robot-state-and-mode-explanation). Please check the [MoveVelocity.srv](../xarm_msgs/srv/MoveVelocity.srv) first to understand the meanings of parameters reqired. If more than one joint are to move, set **is_sync** to `true` for synchronized acceleration/deceleration for all joints in motion, and if is_sync is `false`, each joint will reach to its target velocity as fast as possible. ***is_tool_coord*** parameter is not used here. For example: 
```bash
# NO Timed-out version (will not stop until all-zero velocity command received!):
$ ros2 service call /xarm/vc_set_joint_velocity xarm_msgs/srv/MoveVelocity "{speeds: [-0.1, 0, 0, 0, 0, 0, 0.2], is_sync: true}"
# With Timed-out version(controller firmware version >= 1.8.0): (if next velocity command not received within 0.2 seconds, xArm will stop)  
$ ros2 service call /xarm/vc_set_joint_velocity xarm_msgs/srv/MoveVelocity "{speeds: [-0.1, 0, 0, 0, 0, 0, 0.2], is_sync: true, duration: 0.2}"
``` 
will command the joints (for xArm7) to move in specified angular velocities (in rad/s) and they will reach to target velocities synchronously. The maximum joint acceleration can also be configured by (unit: rad/s^2):  
```bash
$ ros2 service call /xarm/set_joint_maxacc xarm_msgs/srv/SetFloat32 "{data: 10.0}"  (maximum: 20.0 rad/s^2)
``` 

### 2.6 Cartesian velocity control:
&ensp;&ensp;(**xArm controller firmware version >= 1.6.8** required) If controlling linar velocity of TCP towards certain direction is desired, first switch to **Mode 5** as descriped in [mode explanation](https://docs.ufactory.cc/support_articles/faq/robot-state-and-mode-explanation).  Please check the [MoveVelocity.srv](../xarm_msgs/srv/MoveVelocity.srv) first to understand the meanings of parameters reqired. Set **is_tool_coord** to `false` for motion in world/base coordinate system and `true` for tool coordinate system. ***is_sync*** parameter is not used here. For example: 
```bash
# NO Timed-out version (will not stop until all-zero velocity command received!):  
$ ros2 service call /xarm/vc_set_cartesian_velocity xarm_msgs/srv/MoveVelocity "{speeds: [30, 0, 0, 0, 0, 0.1], is_tool_coord: true}"  
# With Timed-out version(controller firmware version >= 1.8.0): (if next velocity command not received within 0.5 seconds, xArm will stop)  
$ ros2 service call /xarm/vc_set_cartesian_velocity xarm_msgs/srv/MoveVelocity "{speeds: [30, 0, 0, 0, 0, 0.1], is_tool_coord: true, duration: 0.5}"
``` 
will command xArm TCP move along X-axis of TOOL coordinate system with speed of 30 mm/s and rotate around Z axis at 0.1 rad/s. The maximum linear acceleration can also be configured by (unit: mm/s^2):  
```bash
$ ros2 service call /xarm/set_tcp_maxacc xarm_msgs/srv/SetFloat32 "{data: 2500}" (maximum: 50000 mm/s^2)
``` 

For angular motion in orientation, please note the velocity is specified as **axis-angular_velocity** elements. That is, [the unit rotation axis vector] multiplied by [rotation velocity value(scalar)]. For example, 
```bash
# NO Timed-out version (will not stop until all-zero velocity command received!):
$ ros2 service call /xarm/vc_set_cartesian_velocity xarm_msgs/srv/MoveVelocity "{speeds: [0, 0, 0, 0.707, 0, 0], is_tool_coord: false}"
# With Timed-out version(controller firmware version >= 1.8.0): (if next velocity command not received within 0.2 seconds, xArm will stop)  
$ ros2 service call /xarm/vc_set_cartesian_velocity xarm_msgs/srv/MoveVelocity "{speeds: [0, 0, 0, 0.707, 0, 0], is_tool_coord: false, duration: 0.2}"
``` 
This will command TCP to rotate along X-axis in BASE coordinates at about 45 degrees/sec. The maximum acceleration for orientation change is fixed.  

Please Note: For no Timed-out version services: velocity motion can be stopped by either giving **all 0 velocity** command, or setting **state to 4(STOP)** and 0(READY) later for next motion. However, **timed-out versions are more recommended for use**, since it can be safe if network comminication or user program fails, controller firmware needs to be updated to v1.8.0 or later.  

### 2.7 Motion service Return:
&ensp;&ensp;Please Note the above motion services will **return immediately** if `wait` parameter is set to `false`. If you wish to return until actual motion is finished, please set it to be `true`.   
&ensp;&ensp;Upon sending success, 0 will be returned. If any error occurs, other values will be returned.

## 3. Joint or Cartesian space servo command [quick step execution]:
&ensp;&ensp;This section will talk about the servo (`realtime, updated at high-frequency`) command interface, only effective in **Mode 1**. Please note that all the angles for service command must use the unit of ***radian***.   

### 3.1 Cartesian space servoing:

**PLEASE NOTE:** If you are already using **Moveit** to control the real robot arm, move_servoj and move_servo_cart **can not be used concurrently**, since they use the same mode(1) to control and may interfere with each other!  

Now with xArm controller **Firmware Version >= 1.4.1**, user can send streamed Cartesian poses to continuously control the xArm, through ros service, this is a wrap of the same xarm sdk(api) function. The actual speed depends on the sending frequency and step distance. Users are encouraged to send the path points in a fixed frequency(100Hz~250Hz) and step distance(**MUST** <10mm).   

(1) To start, bring up the xarm server and in another terminal, do initial configurations, servo cartesian has to operate in **Mode 1**:

(2) Call the servo_cartesian service. Please note:  
**(1) the Cartesian pose representation is the same with xarm SDK here. which is [X(mm), Y(mm), Z(mm), Roll(rad), Pitch(rad), Yaw(rad)]**  
**(2) the path must start from current tool center point(TCP) position and the command can not be too far away, or the execution will fail or act strange. PLEASE CHECK the correctness of command before sending it.**     
**(3) For servo motion, the arguments mvvelo, mvacc, mvtime, and mvradii are not effective now, so just give them all 0.**  
  
Suppose current TCP position is at [206, 0, 121, 3.1416, 0, 0]
```bash
$ ros2 service call /xarm/set_servo_cartesian xarm_msgs/srv/MoveCartesian "{pose: [208, 0, 121, 3.1416, 0, 0]}"
```
Now please check the current TCP position, it will execute this target immediately if success. If you want continuous motion alone X axis, you can give the following pose like:
```bash
$ ros2 service call /xarm/set_servo_cartesian xarm_msgs/srv/MoveCartesian "{pose: [210, 0, 121, 3.1416, 0, 0]}"
```
And you can program this service calling procedure in a loop with proper intervals inbetween, the final execution will become smooth.   

**Notice: Servo_cartisian in TOOL coordinate:**
Please update the controller Firmware to version >= 1.5.0. If servo_cartesian in Tool Coordinate is needed, **set "is_tool_coord" to be true**, the resulted motion will base on current Tool coordinate. For example:  
```bash
$ ros2 service call /xarm/set_servo_cartesian xarm_msgs/srv/MoveCartesian "{pose: [0, 0, 2, 0, 0, 0], is_tool_coord: true}"
```
This will make TCP to move 2 mm immediately along +Z Axis in **TOOL Coordinate**. You may also use `xarm/set_servo_cartesian_aa` service for axis-angle expression command.  


### 3.2 Joint space servoing:
There is also a similar service called "**/xarm/set_servo_angle_j**", you can use this service to control joint motion in the **same mode (1)** with Servo_Cartesian. It receives **absolute** joint positions as command.  Before calling it, please check the current joint position in "/xarm/joint_states" topic, and increase the target joint position **little by little** just like calling /xarm/set_servo_cartesian.

For example, if /xarm/joint_state says current joint positions are:  [0.25,-0.47,0.0,-0.28,0.0,0.76,0.24], you can call:  
```bash
$ ros2 service call /xarm/set_servo_angle_j xarm_msgs/srv/MoveJoint "{angles: [0.25,-0.47,0.0,-0.28,0.0,0.76,0.25]}"
```
Which will move joint7 by 0.01 rad **immediately**. Keep calling it and increase the joint positions a small step each time, it will move smoothly. **Be careful not to give a target too far away in one single update**.  


## 4. Tool I/O Operations:

&ensp;&ensp;We provide 2 digital, 2 analog input port and 2 digital output signals at the end I/O connector.  
### 4.1 To get current 2 DIGITAL input states:  
```bash
$ ros2 service call /xarm/get_tgpio_digital xarm_msgs/srv/GetDigitalIO 
```
### 4.2 To get one of the ANALOG input value: 
```bash
$ ros2 service call /xarm/get_tgpio_analog xarm_msgs/srv/GetAnalogIO "{ionum: 0}"  (io number, can only be 0 or 1)
```
### 4.3 To set one of the Digital output:
```bash
$ ros2 service call /xarm/set_tgpio_digital xarm_msgs/srv/SetDigitalIO "{ionum: 0, value: 1}"  (Setting output port 0 to be 1)
```
&ensp;&ensp;You have to make sure the operation is successful by checking responding "ret" to be 0.

## 5. Controller I/O Operations:

&ensp;&ensp;We provide 8/16 digital input and 8/16 digital output ports at controller box for general usage.  

### 5.1 To get all of the controller DIGITAL input state:  
```bash
$ ros2 service call /xarm/get_cgpio_digital xarm_msgs/srv/GetDigitalIO  # (Notice: from 1st to 8th, for CO0~CO7; from 9th to 16th, for DI0~DI7[if any])
```
### 5.2 To set one of the controller DIGITAL output:
```bash
$ ros2 service call /xarm/set_cgpio_digital xarm_msgs/srv/SetDigitalIO "{ionum: 0, value: 1}" 
```
&ensp;&ensp;Please note port number starts from 0, for example:  
```bash
$ ros2 service call /xarm/set_cgpio_digital xarm_msgs/srv/SetDigitalIO "{ionum: 4, value: 1}"   #(Setting 5th [lable C04] output port to be 1)
```
### 5.3 To get one of the controller ANALOG input:
```bash
$ ros2 service call /xarm/get_cgpio_analog xarm_msgs/srv/GetAnalogIO "{ionum: <port_num>}"  # ionum: 0 or 1
```
### 5.4 To set one of the controller ANALOG output:
```bash
$ ros2 service call /xarm/set_cgpio_analog xarm_msgs/srv/SetAnalogIO  "{ionum: <port_num>, value: <target_value>}"
```
&ensp;&ensp;For example:  
```bash
$ ros2 service call /xarm/set_cgpio_analog xarm_msgs/srv/SetAnalogIO  "{ionum: 1, value: 3.3}"  #(Setting port AO1 to be 3.3)
```
&ensp;&ensp;You have to make sure the operation is successful by checking responding "ret" to be 0.

## 6. Getting status feedback:
&ensp;&ensp;Having connected with a real xArm robot by running 'xarm7_driver.launch.py', user can subscribe to the topic ***"xarm/robot_states"*** for feedback information about current robot states, including joint angles, TCP position, error/warning code, etc. Refer to [RobotMsg.msg](../xarm_msgs/msg/RobotMsg.msg) for content details.  
&ensp;&ensp;Another option is subscribing to ***"xarm/joint_states"*** topic, which is reporting in [JointState.msg](http://docs.ros.org/jade/api/sensor_msgs/html/msg/JointState.html), however, currently ***only "position" field data is solidly valid***; "velocity" is non-filtered numerical differentiation based on 2 adjacent position data, and "effort" feedback are current-based estimated values, not from direct torque sensor, so they are just for reference.
&ensp;&ensp;In consideration of performance, default update rate of above two topics are set at ***5Hz***. The report content and frequency have other options, refer to [report_type argument](#11-report_type-argument)  

## 7. Clearing Errors:
&ensp;&ensp;Sometimes controller may report error or warnings that would affect execution of further commands. The reasons may be power loss, position/speed limit violation, planning errors, etc. It needs additional intervention to clear. User can check error code ("err" field) in the message of topic ***"xarm/robot_states"*** . 
```bash
$ ros2 topic echo /xarm/robot_states
```
&ensp;&ensp;If it is non-zero, the corresponding reason can be found out in the user manual. After solving the problem, this error satus can be removed by calling service ***"/xarm/clean_error"*** with empty argument.
```bash
$ ros2 service call /xarm/clean_error xarm_msgs/srv/Call
```
&ensp;&ensp;After calling `clean_error` service, please ***check the err status again*** in 'xarm/robot_states', if it becomes 0, the clearing is successful. Otherwise, it means the error/exception is not properly solved. If using Moveit!, please set mode to 1 again manually, since controller error and state 4 will automatically switch mode back to 0. If clearing error is successful, remember to ***set robot state to 0*** to make it ready to move again!   

## 8. Gripper Control:
&ensp;&ensp; If our Gripper (from UFACTORY) is attached to the tool end, the following services/actions can be called to operate or check the gripper. Please note we now have three types of grippers, judging from the service name: the ones with "bio_gripper" are for BIO parallel gripper, "lite_gripper" ones are for small gripper of lite 6, and the rest with just "gripper" are for xArm gripper. We will just use xArm gripper for instructions.  

### 8.1 Gripper services:  
(1) Upon powering up, first enable the griper and configure the grasp speed:  
```bash
$ ros2 service call /xarm/set_gripper_enable xarm_msgs/srv/SetInt16 "{data: 1}"
$ ros2 service call /xarm/set_gripper_mode xarm_msgs/srv/SetInt16 "{data: 0}"
$ ros2 service call /xarm/set_gripper_speed xarm_msgs/srv/SetFloat32 "{data: 1500}"
```
&ensp;&ensp; Proper range of the speed is ***from 1 to 5000***. 1500 is used as an example. 'ret' value is 0 for success.  
(2) Give position command (open distance) to xArm gripper:  
```bash
$ ros2 service call /xarm/set_gripper_position xarm_msgs/srv/GripperMove "{pos: 500}"
```
&ensp;&ensp; Proper range of the open distance is ***from 0 to 850***. 0 is closed, 850 is fully open. 500 is used as an example. 'ret' value is 0 for success.  

(3) To get the current status (position and error_code) of xArm gripper:
```bash
$ ros2 service call /xarm/get_gripper_position xarm_msgs/srv/GetFloat32
$ ros2 service call /xarm/get_gripper_err_code xarm_msgs/srv/GetInt16
```
&ensp;&ensp; If error code is non-zero, please refer to user manual for the cause of error, the "/xarm/clear_err" service can still be used to clear the error code of xArm Gripper.  

### 8.2 Gripper action:
&ensp;&ensp; The xArm gripper move action is `xarm_gripper/gripper_action`, which is of type `control_msgs/action/GripperCommand`. **Attention:** The goal position here should be the rotation angle of the virtual joint inside, which is **from 0 rad (fully open) to 0.86 (fully closed)**. Not the pulse in afore-mentioned service call. Gripper action can be called by:  
```bash
$ ros2 action send_goal /xarm_gripper/gripper_action control_msgs/action/GripperCommand "{command: {position: 0.5, max_effort: 0}}"
```

## 9. Vacuum Gripper Control:
&ensp;&ensp; If Vacuum Gripper (from UFACTORY) is attached to the tool end, the following service can be called to operate the vacuum gripper. Note the command service is applicable for both xArm and lite vacuum grippers.   

&ensp;&ensp;To turn on:  
```bash
$ ros2 service call /xarm/set_vacuum_gripper xarm_msgs/srv/VacuumGripperCtrl "{'on': true}"
```
&ensp;&ensp;To turn off:  
```bash
$ ros2 service call /xarm/set_vacuum_gripper xarm_msgs/srv/VacuumGripperCtrl "{'on': false}"
```
&ensp;&ensp;0 will be returned upon successful execution.  


## 10. Tool Modbus communication:
If modbus communication with the tool device is needed, please first set the proper baud rate and timeout parameters through the relevant services: 
```bash
$ ros2 service call /xarm/set_tgpio_modbus_baudrate xarm_msgs/srv/SetInt32 "{data: 115200}"
$ ros2 service call /xarm/set_tgpio_modbus_timeout xarm_msgs/srv/SetModbusTimeout "{timeout: 20}"  # unit: ms
```
The above command will configure the tool modbus baudrate to be 115200 bps and timeout threshold to be 20 **ms**. It is not necessary to configure again if these properties are not changed afterwards. Currently, only the following baud rates (bps) are supported: [4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1000000, 1500000, 2000000, 2500000].  

Then the communication can be conducted like (refer to [GetSetModbusData.srv](../xarm_msgs/srv/GetSetModbusData.srv) for more parameter details):  
```bash
$ ros2 service call /xarm/getset_tgpio_modbus_data xarm_msgs/srv/GetSetModbusData "{modbus_data: [0x01,0x06,0x00,0x0A,0x00,0x03]}"
```
please check the `ret_data` field for device response data.   

## 11. "report_type" argument:
When launching real xArm ROS2 applications, the argument "report_type" can be specified. It decides the state feedback rate and content. Refer to the [developer manual](https://www.ufactory.cc/_files/ugd/896670_1f106918b523404284c6916de025cf28.pdf) at chapter **2.1.6 Automatic Reporting Format** for the report contents of the three available report type (`normal/rich/dev`), default type using is "normal".  

* For users who demand high-frequency feedback, `report_type:=dev` can be specified, then the topics `xarm/robot_states` and `xarm/joint_states` will be published at **100Hz**.  
* For users who want the controller gpio states being updated at `xarm/xarm_cgpio_states` topic, please use `report_type:=rich`, since this reports the fullest information from the controller. As can be seen in developer manual.  
* The report rate of the three types: 

|   type   |    port No.   | Frequency |  GPIO topic   | F/T sensor topic | 
|:--------:|:-------------:|:---------:|:-------------:|:----------------:|
|   normal |     30001     |    5Hz    | Not Available |   Not Available  |
|   rich   |     30002     |    5Hz    |   Available   |     Available    | 
|   dev    |     30003     |    100Hz  | Not Available |     Available    |

Note: **GPIO topic** => `xarm/xarm_cgpio_states`. **F/T sensor topic** =>  `xarm/uf_ftsensor_ext_states` and `xarm/uf_ftsensor_raw_states`.
