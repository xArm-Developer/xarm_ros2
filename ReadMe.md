# xarm_ros2

For simplified Chinese version: [简体中文版](./ReadMe_cn.md)

## 1. Introduction

&ensp;&ensp;&ensp;&ensp;This repository contains simulation models, and corresponding motion planning and controlling demos of the xArm series from UFACTORY. The development and test environment is as follows
- Ubuntu 20.04 + ROS Foxy
- Ubuntu 20.04 + ROS Galactic
- Ubuntu 22.04 + ROS Humble
- Ubuntu 22.04 + ROS Rolling

&ensp;&ensp;&ensp;&ensp;Please switch to the corresponding code branch according to different ros2 versions (no corresponding code branch means it has not been tested in this version)
- Foxy: [foxy](https://github.com/xArm-Developer/xarm_ros2/tree/foxy)
- Galactic: [galactic](https://github.com/xArm-Developer/xarm_ros2/tree/galactic)
- Humble: [humble](https://github.com/xArm-Developer/xarm_ros2/tree/humble)
- Rolling: [rolling](https://github.com/xArm-Developer/xarm_ros2/tree/rolling)

## 2. Update History    
- moveit dual arm control (under single rviz GUI), each arm can be separately configured（e.g. DOF, add_gripper, etc）
- add support for Gazebo simulation, can be controlled by moveit.
- support adding customized tool model.  
- (2022-09-07) Change the parameter type of service (__set_tgpio_modbus_timeout__/__getset_tgpio_modbus_data__), and add parameters to support transparent transmission
- (2022-09-07) Change topic name (xarm_states to robot_states)
- (2022-09-07) Update submodule xarm-sdk to version 1.11.0
- (2022-09-09) [Beta]Support Humble version
- (2022-10-10) xarm_api adds some services
- (2022-12-15) Add parameter `add_realsense_d435i` to load RealSense D435i camera model and support gazebo simulation
- (2023-03-29) Added the launch parameter `model1300` (default is false), and replaced the model of the end of the xarm robot arm with the 1300 series
- (2023-04-20) Update the URDF file, adapt to ROS1 and ROS2, and load the inertia parameters of the link from the configuration file according to the SN
- (2023-04-20) Added the launch parameter `add_d435i_links` (default is false), which supports adding the link relationship between D435i cameras when loading the RealSense D435i model. It is only useful when `add_realsense_d435i` is true
- (2023-04-20) Lite6 supports `add_realsense_d435i` and `add_d435i_links` parameters
- (2023-04-20) Added the launch parameter `robot_sn`, supports loading the inertia parameters of the corresponding joint link, and automatically overrides the `model1300` parameters
- (2023-04-20) Added launch parameters `attach_to`/`attach_xyz`/`attach_rpy` to support attaching the robot arm model to other models
- (2023-06-07) Added support for UFACTORY850 robotic arm
- (2023-10-12) Added the generation and use of joint kinematics parameter files
- (2024-01-17) Added support for xarm7_mirror model robotic arm
- (2024-02-27) Added support for Bio Gripper (parameter `add_bio_gripper`, Lite6 is not supported)
- (2024-04-12) Added __uf_ros_lib__ to encapsulate certain functions for calling (including __MoveItConfigsBuilder__), see [Documentation](./uf_ros_lib/Readme.md)


## 3. Preparation

- ### 3.1 Install [ROS2](https://docs.ros.org/) 
  - [Foxy](https://docs.ros.org/en/ros2_documentation/foxy/Installation.html)
  - [Galactic](https://docs.ros.org/en/ros2_documentation/galactic/Installation.html)
  - [Humble](https://docs.ros.org/en/ros2_documentation/humble/Installation.html)

- ### 3.2 Install [Moveit2](https://moveit.ros.org/install-moveit2/binary/)  

- ### 3.3 Install [Gazebo](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)  

- ### 3.4 Install [gazebo_ros_pkgs](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)  

## 4. How To Use

- ### 4.1 Create a workspace
    ```bash
    # Skip this step if you already have a target workspace
    $ cd ~
    $ mkdir -p dev_ws/src
    ```

- ### 4.2 Obtain source code of "xarm_ros2" repository
    ```bash
    # Remember to source ros2 environment settings first
    $ cd ~/dev_ws/src
    # DO NOT omit "--recursive"，or the source code of dependent submodule will not be downloaded.
    # Pay attention to the use of the -b parameter command branch, $ROS_DISTRO indicates the currently activated ROS version, if the ROS environment is not activated, you need to customize the specified branch (foxy/galactic/humble)
    $ git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive -b $ROS_DISTRO
    ```

- ### 4.3 Update "xarm_ros2" repository 
    ```bash
    $ cd ~/dev_ws/src/xarm_ros2
    $ git pull
    $ git submodule sync
    $ git submodule update --init --remote
    ```

- ### 4.4 Install dependencies
    ```bash
    # Remember to source ros2 environment settings first
    $ cd ~/dev_ws/src/
    $ rosdep update
    $ rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
    ```

- ### 4.5 Build xarm_ros2
    ```bash
    # Remember to source ros2 and moveit2 environment settings first
    $ cd ~/dev_ws/
    # build all packages
    $ colcon build
    
    # build selected packages
    $ colcon build --packages-select xarm_api
    ```


## 5. Package Introduction

__Reminder 1: If there are multiple people using ros2 in the current LAN, in order to avoid mutual interference, please set ROS_DOMAIN_ID__
  - [Foxy](https://docs.ros.org/en/ros2_documentation/foxy/Concepts/About-Domain-ID.html)
  - [Galactic](https://docs.ros.org/en/ros2_documentation/galactic/Concepts/About-Domain-ID.html)
  - [Humble](https://docs.ros.org/en/ros2_documentation/humble/Concepts/About-Domain-ID.html)

__Reminder 2： Remember to source the environment setup script before running any applications in xarm_ros2__

```bash
$ cd ~/dev_ws/
$ source install/setup.bash
```
__Reminder 3： All following instructions will base on xArm6，please use proper parameters or filenames for xArm5 or xArm7__
__Reminder 4: The <hw_ns> described below is replaced with the actual one, the xarm series defaults is xarm, and the rest defaults is ufactory__


- ### 5.1 xarm_description
    This package contains robot description files and 3D models of xArm. Models can be displayed in RViz by the following launch file:
    ```bash
    $ cd ~/dev_ws/
    # set 'add_gripper=true' to attach xArm gripper model
    # set 'add_vacuum_gripper=true' to attach xArm vacuum gripper model
    # Notice：Only one end_effector can be attached (set to 'true').
    $ ros2 launch xarm_description xarm6_rviz_display.launch.py [add_gripper:=true] [add_vacuum_gripper:=true]
    ```

- ### 5.2 xarm_msgs  
    This package contains all interface definitions for xarm_ros2, please check the instructions in the files before using them. [README](./xarm_msgs/ReadMe.md)

- ### 5.3 xarm_sdk
    This package serves as a submodule of this project，the corresponding git repository is: [xArm-CPLUS-SDK](https://github.com/xArm-Developer/xArm-CPLUS-SDK.git), for interfacing with real xArms, please refer to the documentation in "xArm-CPLUS-SDK" if interested.

- ### 5.4 xarm_api
    This package is a ros wrapper of "xarm_sdk"，functions are implemented as ros service or ros topic，communications with real xArm in "xarm_ros2" are based on the services and topics provided in this part. All the services and topics are under <hw_ns>/ namespace，e.g. full name for "joint_states" is actually "<hw_ns>/joint_states".  
    
    - __services__: the name of provided services are the same with the corresponding function in SDK, however, whether to activate the service is up to the configuration under the "services" domain in ```xarm_api/config/xarm_params.yaml``` and ```xarm_api/config/xarm_user_params.yaml```. The defined service can only be activated at initialization if that service is configured to ```true```. If you need to customize the parameters, please create a file ```xarm_api/config/xarm_user_params.yaml``` to modify, the format, refer to ```xarm_api/config/xarm_params.yaml```.
        ```
        services:
            motion_enable: true
            set_mode: true
            set_state: true
            clean_conf: false
            ...
        ```

    - __topics__:  

        __joint_states__: is of type __sensor_msgs::msg::JointState__  

        __robot_states__: is of type __xarm_msgs::msg::RobotMsg__  

        __xarm_cgpio_states__: is of type __xarm_msgs::msg::CIOState__  

        __uf_ftsensor_raw_states__: is of type __geometry_msgs::msg::WrenchStamped__  

        __uf_ftsensor_ext_states__: is of type __geometry_msgs::msg::WrenchStamped__  

        __Note:__: some of the topics are only available when specific __report_type__ is set at launch stage. Refer [here](https://github.com/xArm-Developer/xarm_ros#report_type-argument).  

    
    - __Launch and test (xArm)__:  

        ```bash
        $ cd ~/dev_ws/
        # launch xarm_driver_node
        $ ros2 launch xarm_api xarm6_driver.launch.py robot_ip:=192.168.1.117
        # service test
        $ ros2 run xarm_api test_xarm_ros_client
        # topic test
        $ ros2 run xarm_api test_robot_states
        ```

    - __Use command line (xArm)__:

        ```bash
        $ cd ~/dev_ws/
        # launch xarm_driver_node:
        $ ros2 launch xarm_api xarm6_driver.launch.py robot_ip:=192.168.1.117
        
        # enable all joints:
        $ ros2 service call /xarm/motion_enable xarm_msgs/srv/SetInt16ById "{id: 8, data: 1}"
        
        # set proper mode (0) and state (0)
        $ ros2 service call /xarm/set_mode xarm_msgs/srv/SetInt16 "{data: 0}"
        $ ros2 service call /xarm/set_state xarm_msgs/srv/SetInt16 "{data: 0}"
        
        # Cartesian linear motion: (unit: mm, rad)
        $ ros2 service call /xarm/set_position xarm_msgs/srv/MoveCartesian "{pose: [300, 0, 250, 3.14, 0, 0], speed: 50, acc: 500, mvtime: 0}"   
        
        # joint motion for xArm6: (unit: rad)
        $ ros2 service call /xarm/set_servo_angle xarm_msgs/srv/MoveJoint "{angles: [-0.58, 0, 0, 0, 0, 0], speed: 0.35, acc: 10, mvtime: 0}"
        ```
    
    - __Use command line (lite6)__:

        ```bash
        $ cd ~/dev_ws/
        # launch ufactory_driver_node:
        $ ros2 launch xarm_api lite6_driver.launch.py robot_ip:=192.168.1.161
        
        # enable all joints:
        $ ros2 service call /ufactory/motion_enable xarm_msgs/srv/SetInt16ById "{id: 8, data: 1}"
        
        # set proper mode (0) and state (0)
        $ ros2 service call /ufactory/set_mode xarm_msgs/srv/SetInt16 "{data: 0}"
        $ ros2 service call /ufactory/set_state xarm_msgs/srv/SetInt16 "{data: 0}"
        
        # Cartesian linear motion: (unit: mm, rad)
        $ ros2 service call /ufactory/set_position xarm_msgs/srv/MoveCartesian "{pose: [250, 0, 250, 3.14, 0, 0], speed: 50, acc: 500, mvtime: 0}"   
        
        # joint motion: (unit: rad)
        $ ros2 service call /ufactory/set_servo_angle xarm_msgs/srv/MoveJoint "{angles: [-0.58, 0, 0, 0, 0, 0], speed: 0.35, acc: 10, mvtime: 0}"
        ```
    
    - __Use command line (UFACTORY850)__:

        ```bash
        $ cd ~/dev_ws/
        # launch ufactory_driver_node:
        $ ros2 launch xarm_api uf850_driver.launch.py robot_ip:=192.168.1.181
        
        # enable all joints:
        $ ros2 service call /ufactory/motion_enable xarm_msgs/srv/SetInt16ById "{id: 8, data: 1}"
        
        # set proper mode (0) and state (0)
        $ ros2 service call /ufactory/set_mode xarm_msgs/srv/SetInt16 "{data: 0}"
        $ ros2 service call /ufactory/set_state xarm_msgs/srv/SetInt16 "{data: 0}"
        
        # Cartesian linear motion: (unit: mm, rad)
        $ ros2 service call /ufactory/set_position xarm_msgs/srv/MoveCartesian "{pose: [250, 0, 250, 3.14, 0, 0], speed: 50, acc: 500, mvtime: 0}"   
        
        # joint motion: (unit: rad)
        $ ros2 service call /ufactory/set_servo_angle xarm_msgs/srv/MoveJoint "{angles: [-0.58, 0, 0, 0, 0, 0], speed: 0.35, acc: 10, mvtime: 0}"
        ```

    Note: please study the meanings of [Mode](https://github.com/xArm-Developer/xarm_ros#6-mode-change), State and available motion instructions before testing on the real robot. Please note **the services provided by xArm series and Lite 6 have different namespaces**.  

- ### 5.5 xarm_controller
    This package defines the hardware interface for real xArm control under ros2.  

    ```bash
    $ cd ~/dev_ws/
    # For xArm(xarm6 as example): set 'add_gripper=true' to attach xArm gripper model
    $ ros2 launch xarm_controller xarm6_control_rviz_display.launch.py robot_ip:=192.168.1.117 [add_gripper:=true]

    # For lite6: set 'add_gripper=true' to attach Lite6 gripper model
    $ ros2 launch xarm_controller lite6_control_rviz_display.launch.py robot_ip:=192.168.1.161 [add_gripper:=true]
    
    # For UFACTORY850: set 'add_gripper=true' to attach xarm gripper model
    $ ros2 launch xarm_controller uf850_control_rviz_display.launch.py robot_ip:=192.168.1.181 [add_gripper:=true]
    ```

- ### 5.6 xarm_moveit_config
    This package provides abilities for controlling xArm/Lite6 (simulated or real arm) by moveit.

    - 【simulated】Launch moveit, controlling robot in rviz.  

        ```bash
        $ cd ~/dev_ws/
        # For xArm(xarm6 as example): set 'add_gripper=true' to attach xArm gripper model
        $ ros2 launch xarm_moveit_config xarm6_moveit_fake.launch.py [add_gripper:=true]

        # For Lite6: set 'add_gripper=true' to attach Lite6 gripper model
        $ ros2 launch xarm_moveit_config lite6_moveit_fake.launch.py [add_gripper:=true]

        # For UFACTORY850: set 'add_gripper=true' to attach xarm gripper model
        $ ros2 launch xarm_moveit_config uf850_moveit_fake.launch.py [add_gripper:=true]
        ```
    
    - 【real arm】Launch moveit, controlling robot in rviz.  

        ```bash
        $ cd ~/dev_ws/
        # For xArm(xarm6 as example): set 'add_gripper=true' to attach xArm gripper model
        $ ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py robot_ip:=192.168.1.117 [add_gripper:=true]

        # For Lite6: set 'add_gripper=true' to attach Lite6 gripper model
        $ ros2 launch xarm_moveit_config lite6_moveit_realmove.launch.py robot_ip:=192.168.1.161 [add_gripper:=true]

        # For UFACTORY850: set 'add_gripper=true' to attach xarm gripper model
        $ ros2 launch xarm_moveit_config uf850_moveit_realmove.launch.py robot_ip:=192.168.1.181 [add_gripper:=true]
        ```
    
    - 【Dual simulated】Launch single moveit process, and controlling two xArms in one rviz.  

        ```bash
        $ cd ~/dev_ws/
        # set 'add_gripper=true' to attach xArm gripper model
        # 'add_gripper_1': can separately decide whether to attach gripper for left arm，default for same value with 'add_gripper'
        # 'add_gripper_2': can separately decide whether to attach gripper for right arm，default for same value with 'add_gripper'
        # 'dof_1': can separately configure the model DOF of left arm，default to be the same DOF specified in filename.
        # 'dof_2': can separately configure the model DOF of right arm，default to be the same DOF specified in filename.
        
        # For xArm (xarm6 here):
        $ ros2 launch xarm_moveit_config dual_xarm6_moveit_fake.launch.py [add_gripper:=true]

        # For Lite6:
        $ ros2 launch xarm_moveit_config dual_lite6_moveit_fake.launch.py [add_gripper:=true]

        # For UFACTORY850:
        $ ros2 launch xarm_moveit_config dual_uf850_moveit_fake.launch.py [add_gripper:=true]
        ```
    
    - 【Dual real arm】Launch single moveit process, and controlling two xArms in one rviz.  

        ```bash
        $ cd ~/dev_ws/
        # 'robot_ip_1': IP address of left arm
        # 'robot_ip_2': IP address of right arm
        # set 'add_gripper=true' to attach xArm gripper model
        # 'add_gripper_1': can separately decide whether to attach gripper for left arm，default for same value with 'add_gripper'
        # 'add_gripper_2': can separately decide whether to attach gripper for right arm，default for same value with 'add_gripper'
        # 'dof_1': can separately configure the model DOF of left arm，default to be the same DOF specified in filename.
        # 'dof_2': can separately configure the model DOF of right arm，default to be the same DOF specified in filename.
        
        # For xArm (xarm6 here):
        $ ros2 launch xarm_moveit_config dual_xarm6_moveit_realmove.launch.py robot_ip_1:=192.168.1.117 robot_ip_2:=192.168.1.203 [add_gripper:=true]
        
        # For Lite6:
        $ ros2 launch xarm_moveit_config dual_lite6_moveit_realmove.launch.py robot_ip_1:=192.168.1.117 robot_ip_2:=192.168.1.203 [add_gripper:=true]

        # For UFACTORY850:
        $ ros2 launch xarm_moveit_config dual_uf850_moveit_realmove.launch.py robot_ip_1:=192.168.1.181 robot_ip_2:=192.168.1.182 [add_gripper:=true]
        ```

- ### 5.7 xarm_planner
    This package provides functions for controlling xArm (simulated or real arm) through moveit API  

    ```bash
    $ cd ~/dev_ws/
    # 【simulated xArm】launch xarm_planner_node
    $ ros2 launch xarm_planner xarm6_planner_fake.launch.py [add_gripper:=true]
    # 【real xArm】launch xarm_planner_node
    $ ros2 launch xarm_planner xarm6_planner_realmove.launch.py robot_ip:=192.168.1.117 [add_gripper:=true]

    # 【simulated Lite6】launch xarm_planner_node
    $ ros2 launch xarm_planner lite6_planner_fake.launch.py [add_gripper:=true]
    # 【real Lite6】launch xarm_planner_node
    $ ros2 launch xarm_planner lite6_planner_realmove.launch.py robot_ip:=192.168.1.117 [add_gripper:=true]

    # 【simulated UFACTORY850】launch xarm_planner_node
    $ ros2 launch xarm_planner uf850_planner_fake.launch.py [add_gripper:=true]
    # 【real UFACTORY850】launch xarm_planner_node
    $ ros2 launch xarm_planner uf850_planner_realmove.launch.py robot_ip:=192.168.1.181 [add_gripper:=true]

    # In another terminal, run test program (control through API, specify 'robot_type' as 'xarm' or 'lite' or 'uf850')
    $ ros2 launch xarm_planner test_xarm_planner_api_joint.launch.py dof:=6 robot_type:=<xarm | lite | uf850>
    $ ros2 launch xarm_planner test_xarm_planner_api_pose.launch.py dof:=6 robot_type:=<xarm | lite | uf850>
    ```

    Below additional tests are just for xArm:
    ```bash
    # run test program（control through service）
    $ ros2 launch xarm_planner test_xarm_planner_client_joint.launch.py dof:=6
    $ ros2 launch xarm_planner test_xarm_planner_client_pose.launch.py dof:=6

    # run test program（control gripper through API）
    $ ros2 launch xarm_planner test_xarm_gripper_planner_api_joint.launch.py dof:=6

    # run test program（control gripper through service）
    $ ros2 launch xarm_planner test_xarm_gripper_planner_client_joint.launch.py dof:=6
    ```


- ### 5.8 xarm_gazebo
    This package is for supporting xArm simulation with Gazobo.  
    ***Notice:***  
    (1) Installation of [gazebo_ros2_control](https://github.com/ros-simulation/gazebo_ros2_control.git) from source may be needed, as well as setting up environment variables of gazebo_ros2_control.  
    (2) [minic_joint_plugin](https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins) was developed for ROS1, we have modified a version for ROS2 compatibility and it is already integrated in this package for xArm Gripper simulation.  
    
    - Testing xarm on gazebo independently:
        ```bash
        $ cd ~/dev_ws/
        # For xArm (xarm6 here):
        $ ros2 launch xarm_gazebo xarm6_beside_table_gazebo.launch.py

        # For Lite6:
        $ ros2 launch xarm_gazebo lite6_beside_table_gazebo.launch.py

        # For UFACTORY850:
        $ ros2 launch xarm_gazebo uf850_beside_table_gazebo.launch.py
        ```

    - Simulation with moveit+gazebo (xArm controlled by moveit).
        ```bash
        $ cd ~/dev_ws/
        # For xArm (xarm6 here):
        $ ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py

        # For Lite6:
        $ ros2 launch xarm_moveit_config lite6_moveit_gazebo.launch.py

        # For UFACTORY850:
        $ ros2 launch xarm_moveit_config uf850_moveit_gazebo.launch.py
        ```
- ### 5.9 xarm_moveit_servo
    This package serves as a demo for jogging xArm with devices such as joystick, through [moveit_servo](http://moveit2_tutorials.picknik.ai/doc/realtime_servo/realtime_servo_tutorial.html). 
    - Controlling with __XBOX360__ joystick:
        - left stick for X and Y direction.  
        - right stick for ROLL and PITCH adjustment.  
        - left and right trigger (LT/RT) for Z direction.  
        - left and right bumper (LB/RB) for YAW adjustment.  
        - D-PAD for controlling joint1 and joint2.  
        - buttons X and B for controlling last joint.  
        - buttons Y and A for controlling second last joint.  

        ```bash
        $ cd ~/dev_ws/
        # XBOX Wired -> joystick_type=1
        # XBOX Wireless -> joystick_type=2
        # For controlling simulated xArm:
        $ ros2 launch xarm_moveit_servo xarm_moveit_servo_fake.launch.py joystick_type:=1
        # Or controlling simulated Lite6:
        $ ros2 launch xarm_moveit_servo lite6_moveit_servo_fake.launch.py joystick_type:=1
        # Or controlling simulated UFACTORY850:
        $ ros2 launch xarm_moveit_servo uf850_moveit_servo_fake.launch.py joystick_type:=1


        # For controlling real xArm: (use xArm 5 as example)
        $ ros2 launch xarm_moveit_servo xarm_moveit_servo_realmove.launch.py robot_ip:=192.168.1.123 dof:=5 joystick_type:=1
        # Or controlling real Lite6:
        $ ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py robot_ip:=192.168.1.123 joystick_type:=1
        # Or controlling real UFACTORY850:
        $ ros2 launch xarm_moveit_servo uf850_moveit_servo_realmove.launch.py robot_ip:=192.168.1.181 joystick_type:=1
        ```
        ```

    - Controlling with __3Dconnexion SpaceMouse Wireless__:
        - 6 DOFs of the mouse are mapped for controlling X/Y/Z/ROLL/PITCH/YAW  
        - Left button clicked for just X/Y/Z adjustment  
        - Right button clicked for just ROLL/PITCH/YAW adjustment  

        ```bash
        $ cd ~/dev_ws/
        # For controlling simulated xArm:
        $ ros2 launch xarm_moveit_servo xarm_moveit_servo_fake.launch.py joystick_type:=3
        # Or controlling simulated Lite6:
        $ ros2 launch xarm_moveit_servo lite6_moveit_servo_fake.launch.py joystick_type:=3
        # Or controlling simulated UFACTORY850:
        $ ros2 launch xarm_moveit_servo uf850_moveit_servo_fake.launch.py joystick_type:=3

        # For controlling real xArm: (use xArm 5 as example)
        $ ros2 launch xarm_moveit_servo xarm_moveit_servo_realmove.launch.py robot_ip:=192.168.1.123 dof:=5 joystick_type:=3
        # Or controlling real Lite6:
        $ ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py robot_ip:=192.168.1.123 joystick_type:=3
        # Or controlling real UFACTORY850:
        $ ros2 launch xarm_moveit_servo uf850_moveit_servo_realmove.launch.py robot_ip:=192.168.1.181 joystick_type:=3
        ```
    
    - Controlling with __PC keyboard__:
        ```bash
        $ cd ~/dev_ws/
        # For controlling simulated xArm:
        $ ros2 launch xarm_moveit_servo xarm_moveit_servo_fake.launch.py dof:=6
        # Or controlling simulated Lite6:
        $ ros2 launch xarm_moveit_servo lite6_moveit_servo_fake.launch.py
        # Or controlling simulated UFACTORY850:
        $ ros2 launch xarm_moveit_servo uf850_moveit_servo_fake.launch.py

        # For controlling real xArm: (use xArm 5 as example)
        $ ros2 launch xarm_moveit_servo xarm_moveit_servo_realmove.launch.py robot_ip:=192.168.1.123 dof:=5
        # Or for controlling real Lite6:
        $ ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py robot_ip:=192.168.1.123
        # Or for controlling real UFACTORY850:
        $ ros2 launch xarm_moveit_servo uf850_moveit_servo_realmove.launch.py robot_ip:=192.168.1.181

        # Then in another terminal, run keyboad input node:
        $ ros2 run xarm_moveit_servo xarm_keyboard_input
        ```
        Please note that Moveit Servo may consider the home position as singularity point, then try with joint motion first.  

## 6. Instruction on major launch arguments
- __robot_ip__,
    IP address of xArm, needed when controlling real hardware.
- __report_type__, default: normal. 
    Data report type, supported types are: normal/rich/dev, 
    different types will report with different data contents and frequency.
- __dof__, default: 7. 
    Degree of freedom (DOF) of robot arm，no need to specify explicitly unless necessary.
    For dual arm launch files(with ```dual_``` prefix), DOF can be specified through:
    - __dof_1__
    - __dof_2__
- __velocity_control__, default: false. 
    Whether to control with velocity interface. (Otherwise, use position interface)
- __add_realsense_d435i__, default: false.  
    Whether to load the realsense D435i camera model at the end.
    For dual arm launch files(with ```dual_``` prefix), it can be specified through:
    - __add_realsense_d435i_1__
    - __add_realsense_d435i_2__
- __add_gripper__, default: false. 
    Whether to include UFACTORY gripper in the model，it has higher priority than the argument ```add_vacuum_gripper```.
    For dual arm launch files(with ```dual_``` prefix), it can be specified through:
    - __add_gripper_1__
    - __add_gripper_2__
- __add_bio_gripper__, default: false. 
    Whether to include BIO gripper in the model，it has higher priority than the argument ```add_vacuum_gripper```, ```add_gripper``` must be false in order to set vacuum gripper to be true.
    For dual arm launch files(with ```dual_``` prefix), it can be specified through:
    - __add_bio_gripper_1__
    - __add_bio_gripper_2__
- __add_vacuum_gripper__, default: false. 
    Whether to include UFACTRORY vacuum gripper in the model，```add_gripper``` must be false in order to set vacuum gripper to be true.
    For dual arm launch files(with ```dual_``` prefix), it can be specified through:
    - __add_vacuum_gripper_1__
    - __add_vacuum_gripper_2__
- __add_other_geometry__, default: false. 
    Whether to add other geometric model as end-tool, ```add_gripper``` and ```add_vacuum_gripper``` has to be false in order to set it to be true.
    
    - __geometry_type__, default: box, effective when ```add_other_geometry=true```.  
        geometry type to be added as end-tool，valid types: box/cylinder/sphere/mesh.  
    - __geometry_mass__, unit: kg，default value: 0.1  
        model mass.
    - __geometry_height__, unit: m，default value: 0.1  
        specifying geometry hight，effective when geometry_type=box/cylinder/sphere.  
    - __geometry_radius__, unit: m，default value: 0.1  
        specifying geometry radius, effective when geometry_type=cylinder/sphere.  
    - __geometry_length__, unit: m，default value: 0.1  
        specifying geometry length, effective when geometry_type=box.  
    - __geometry_width__, unit: m，default value: 0.1  
        specifying geometry width,effective when geometry_type=box.  
    - __geometry_mesh_filename__,
        filename of the specified mesh model，effective when geometry_type=mesh.  
        ***This file needs to be put in ```xarm_description/meshes/other/``` folder.*** Such that full directory will not be needed in filename specification.  
    - __geometry_mesh_origin_xyz__, default: "0 0 0"  
    - __geometry_mesh_origin_rpy__, default: "0 0 0"  
        transformation from end-flange coordinate frame to geometry model origin coordinate frame, effective when ```geometry_type=mesh```. Example: geometry_mesh_origin_xyz:='"0.05 0.0 0.0"'.  
    - __geometry_mesh_tcp_xyz__, default: "0 0 0"  
    - __geometry_mesh_tcp_rpy__, default: "0 0 0"  
        transformation from geometry model origin frame to geometry model tip ("Tool Center Point") frame, effective when ```geometry_type=mesh```. Example: geometry_mesh_tcp_rpy:='"0.0 0.0 1.5708"'.  
    - __Example of adding customized end tool (Cylinder):__  

        ```bash
        $ ros2 launch xarm_gazebo xarm6_beside_table_gazebo.launch.py add_other_geometry:=true geometry_type:=cylinder geometry_height:=0.075 geometry_radius:=0.045
        ```

    For dual arm launch files(with ```dual_``` prefix), here are the total arguments that can be configured:  
    - __add_other_geometry_1__
    - __add_other_geometry_2__
    - __geometry_type_1__
    - __geometry_type_2__
    - __geometry_mass_1__
    - __geometry_mass_2__
    - __geometry_height_1__
    - __geometry_height_2__
    - __geometry_radius_1__,
    - __geometry_radius_2__,
    - __geometry_length_1__
    - __geometry_length_2__
    - __geometry_width_1__
    - __geometry_width_2__
    - __geometry_mesh_filename_1__
    - __geometry_mesh_filename_2__
    - __geometry_mesh_origin_xyz_1__
    - __geometry_mesh_origin_xyz_2__
    - __geometry_mesh_origin_rpy_1__ 
    - __geometry_mesh_origin_rpy_2__ 
    - __geometry_mesh_tcp_xyz_1__
    - __geometry_mesh_tcp_xyz_2__
    - __geometry_mesh_tcp_rpy_1__
    - __geometry_mesh_tcp_rpy_2__
- __kinematics_suffix__: Specify joint Kinematics parameter file suffix
    - Generation of Kinematics parameter file: 
      ```bash
      cd src/xarm_ros/xarm_description/config/kinematics
      python gen_kinematics_params.py {robot_ip} {kinematics_suffix}

      # Note
      # 1. robot_ip represents the IP of the robot arm. You need to connect to the robot arm to obtain the actual parameters.
      # 2. kinematics_suffix represents the suffix of the generated parameter file. If successful, the configuration file will be generated in the xarm_description/config/kinematics/user directory. If kinematics_suffix is AAA, then the corresponding file name is as follows
      #   xarm5: xarm_description/config/kinematics/user/xarm5_kinematics_AAA.yaml
      #   xarm6: xarm_description/config/kinematics/user/xarm6_kinematics_AAA.yaml
      #   xarm7: xarm_description/config/kinematics/user/xarm7_kinematics_AAA.yaml
      #   lite6: xarm_description/config/kinematics/user/lite6_kinematics_AAA.yaml
      #   uf850: xarm_description/config/kinematics/user/uf850_kinematics_AAA.yaml
      ```
    - Use of Kinematics parameter file: Specify this parameter when starting the launch file
      - Note that before specifying this parameter, make sure that the corresponding configuration file exists. If it does not exist, you need to connect the robot arm through a script to generate it.