# xarm_ros2

For simplified Chinese version: [简体中文版](./ReadMe_cn.md)

## 1. Introduction

&ensp;&ensp;&ensp;&ensp;This repository contains simulation models, and corresponding motion planning and controlling demos of the xArm series from UFACTORY. Developing environment: Ubuntu 20.04 + ROS Foxy.  

## 2. Update History    
- moveit dual arm control (under single rviz GUI), each arm can be separately configured（e.g. DOF, add_gripper, etc）
- add support for Gazebo simulation, can be controlled by moveit.
- support adding customized tool model.  


## 3. Preparation

- ### 3.1 Install [ROS Foxy](https://docs.ros.org/en/foxy/Installation.html) 

- ### 3.2 Install [Moveit2](https://moveit.ros.org/install-moveit2/source/)  

- ### 3.3 Install [ros2_control, ros2_controllers](https://ros-controls.github.io/control.ros.org/getting_started.html)  

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
    $ cd ~/dev_ws/src
    # DO NOT omit "--recursive"，or the source code of dependent submodule will not be downloaded.
    $ git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive
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
    # Remember to source ros foxy environment settings first
    $ cd ~/dev_ws/src/
    $ rosdep update
    $ rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO
    ```

- ### 4.5 Build xarm_ros2
    ```bash
    # Remember to source ros foxy and moveit2 environment settings first
    $ cd ~/dev_ws/
    # build all packages
    $ colcon build
    
    # build selected packages
    $ colcon build --packages-select xarm_api
    ```


## 5. Package Introduction

__Reminder 1: If there are multiple people using ros2 in the current LAN, in order to avoid mutual interference, please set [ROS_DOMAIN_ID](https://docs.ros.org/en/ros2_documentation/foxy/Concepts/About-Domain-ID.html)__

__Reminder 2： Remember to source the environment setup script before running any applications in xarm_ros2__

```bash
$ cd ~/dev_ws/
$ source install/setup.bash
```
__Reminder 3： All following instructions will base on xArm6，please use proper parameters or filenames for xArm5 or xArm7__


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
    This package is a ros wrapper of "xarm_sdk"，functions are implemented as ros service or ros topic，communications with real xArm in "xarm_ros2" are based on the services and topics provided in this part. All the services and topics are under xarm/ namespace by default(unless 'hw_ns' parameter is specified)，e.g. full name for "joint_states" is actually "xarm/joint_states".  
    
    - __services__: the name of provided services are the same with the corresponding function in SDK, however, whether to activate the service is up to the configuration under the "services" domain in ```xarm_api/config/xarm_params.yaml```. The defined service can only be activated at initialization if that service is configured to ```true```.  
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

        __xarm_states__: is of type __xarm_msgs::msg::RobotMsg__  

        __xarm_cgpio_states__: is of type __xarm_msgs::msg::CIOState__  
    
    - Launch and test  

        ```bash
        $ cd ~/dev_ws/
        # launch xarm_driver_node
        $ ros2 launch xarm_api xarm6_driver.launch.py robot_ip:=192.168.1.117
        # service test
        $ ros2 run xarm_api test_xarm_ros_client
        # topic test
        $ ros2 run xarm_api test_xarm_states
        ```

- ### 5.5 xarm_controller
    This package defines the hardware interface for real xArm control under ros2.  

    ```bash
    $ cd ~/dev_ws/
    # set 'add_gripper=true' to attach xArm gripper model
    $ ros2 launch xarm6_control_rviz_display.launch.py robot_ip:=192.168.1.117 [add_gripper:=true]
    # open up two rviz windows for two separated arms at the same time
    $ ros2 launch two_xarm6_control_rviz_display.launch.py robot1_ip:=192.168.1.117 robot2_ip:=192.168.1.203
    ```

- ### 5.6 xarm_moveit_config
    This package provides abilities for controlling xArm (simulated or real arm) by moveit.

    - 【simulated】Launch moveit, controlling xArm in rviz.  

        ```bash
        $ cd ~/dev_ws/
        # set 'add_gripper=true' to attach xArm gripper model
        $ ros2 launch xarm_moveit_config xarm6_moveit_fake.launch.py [add_gripper:=true]
        ```
    
    - 【real arm】Launch moveit, controlling xArm in rviz.  

        ```bash
        $ cd ~/dev_ws/
        # set 'add_gripper=true' to attach xArm gripper model
        $ ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py robot_ip:=192.168.1.117 [add_gripper:=true]
        ```
    
    - 【simulated x2】launch two moveit processes(including rviz)，to control two xArms separately  

        ```bash
        $ cd ~/dev_ws/
        # set 'add_gripper=true' to attach xArm gripper model
        $ ros2 launch xarm_moveit_config two_xarm6_moveit_fake.launch.py [add_gripper:=true]
        ```
    
    - 【real arm x2】launch two moveit processes(including rviz)，to control two xArms separately  

        ```bash
        $ cd ~/dev_ws/
        # set 'add_gripper=true' to attach xArm gripper model
        $ ros2 launch xarm_moveit_config two_xarm6_moveit_realmove.launch.py robot1_ip:=192.168.1.117 robot2_ip:=192.168.1.203 [add_gripper:=true]
        ```
    
    - 【Dual simulated】Launch single moveit process, and controlling two xArms in one rviz.  

        ```bash
        $ cd ~/dev_ws/
        # set 'add_gripper=true' to attach xArm gripper model
        # 'add_gripper_1': can separately decide whether to attach gripper for left arm，default for same value with 'add_gripper'
        # 'add_gripper_2': can separately decide whether to attach gripper for right arm，default for same value with 'add_gripper'
        # 'dof_1': can separately configure the model DOF of left arm，default to be the same DOF specified in filename.
        # 'dof_2': can separately configure the model DOF of right arm，default to be the same DOF specified in filename.
        $ ros2 launch xarm_moveit_config dual_xarm6_moveit_fake.launch.py [add_gripper:=true]
        ```
    
    - 【Dual real arm】Launch single moveit process, and controlling two xArms in one rviz.  

        ```bash
        $ cd ~/dev_ws/
        # 'robot1_ip': IP address of left arm
        # 'robot2_ip': IP address of right arm
        # set 'add_gripper=true' to attach xArm gripper model
        # 'add_gripper_1': can separately decide whether to attach gripper for left arm，default for same value with 'add_gripper'
        # 'add_gripper_2': can separately decide whether to attach gripper for right arm，default for same value with 'add_gripper'
        # 'dof_1': can separately configure the model DOF of left arm，default to be the same DOF specified in filename.
        # 'dof_2': can separately configure the model DOF of right arm，default to be the same DOF specified in filename.
        $ ros2 launch xarm_moveit_config dual_xarm6_moveit_realmove.launch.py robot1_ip:=192.168.1.117 robot2_ip:=192.168.1.203 [add_gripper:=true]
        ```

- ### 5.7 xarm_planner
    This package provides functions for controlling xArm (simulated or real arm) through moveit API  

    ```bash
    $ cd ~/dev_ws/
    # 【simulated】launch xarm_planner_node
    $ ros2 launch xarm_planner xarm6_planner_fake.launch.py [add_gripper:=true]
    # 【real arm】launch xarm_planner_node
    # $ ros2 launch xarm_planner xarm6_planner_realmove.launch.py robot_ip:=192.168.1.117 [add_gripper:=true]

    # run test program (control through API)
    $ ros2 launch xarm_planner test_xarm_planner_api_joint.launch.py dof:=6
    $ ros2 launch xarm_planner test_xarm_planner_api_pose.launch.py dof:=6

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
        $ ros2 launch xarm_gazebo xarm6_beside_table_gazebo.launch.py
        ```

    - Simulation with moveit+gazebo (xArm controlled by moveit).
        ```bash
        $ cd ~/dev_ws/
        $ ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py
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

        # For controlling real xArm: (use xArm 5 as example)
        # ros2 launch xarm_moveit_servo xarm_moveit_servo_realmove.launch.py robot_ip:=192.168.1.123 dof:=5 joystick_type:=1
        ```

    - Controlling with __3Dconnexion SpaceMouse Wireless__:
        - 6 DOFs of the mouse are mapped for controlling X/Y/Z/ROLL/PITCH/YAW  
        - Left button clicked for just X/Y/Z adjustment  
        - Right button clicked for just ROLL/PITCH/YAW adjustment  

        ```bash
        $ cd ~/dev_ws/
        # For controlling simulated xArm:
        $ ros2 launch xarm_moveit_servo xarm_moveit_servo_fake.launch.py joystick_type:=3

        # For controlling real xArm: (use xArm 5 as example)
        # ros2 launch xarm_moveit_servo xarm_moveit_servo_realmove.launch.py robot_ip:=192.168.1.123 dof:=5 joystick_type:=3
        ```
    
    - Controlling with __PC keyboard__:
        ```bash
        $ cd ~/dev_ws/
        # For controlling simulated xArm:
        $ ros2 launch xarm_moveit_servo xarm_moveit_servo_fake.launch.py

        # For controlling real xArm: (use xArm 5 as example)
        # ros2 launch xarm_moveit_servo xarm_moveit_servo_realmove.launch.py robot_ip:=192.168.1.123 dof:=5

        # Running keyboad input node:
        $ ros2 run xarm_moveit_servo xarm_keyboard_input
        ```


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
- __add_gripper__, default: false. 
    Whether to include UFACTORY gripper in the model，it has higher priority than the argument ```add_vacuum_gripper```.
    For dual arm launch files(with ```dual_``` prefix), it can be specified through:
    - __add_gripper_1__
    - __add_gripper_2__
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
