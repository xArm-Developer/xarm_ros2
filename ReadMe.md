# xarm_ros2

For simplified Chinese version: [简体中文版](./ReadMe_cn.md)

## 1. Introduction

&ensp;&ensp;&ensp;&ensp;This repository contains simulation models, and corresponding motion planning and controlling demos of the xArm series from UFACTORY. Developing environment: Ubuntu 20.04 + ROS Foxy.  

## 2. Update History    
- moveit dual arm control (under single rviz GUI), each arm can be separately configured（e.g. DOF, add_gripper, etc）


## 3. Preparation

- ### 3.1 Install [ROS Foxy](https://docs.ros.org/en/foxy/Installation.html) 

- ### 3.2 Install [Moveit2](https://moveit.ros.org/install-moveit2/source/)


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
    # $ colcon build --packages-select xarm_api
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
    This package contains all interface definitions for xarm_ros2, please check the instructions in the files before using them. [REAEME](./xarm_msgs/ReadMe.md)

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
    
    # $ ros2 launch two_xarm6_control_rviz_display.launch.py robot1_ip:=192.168.1.117 robot2_ip:=192.168.1.203
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
    Not yet implemented.

