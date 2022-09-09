# xarm_ros2说明

## 1. 简介

&ensp;&ensp;&ensp;&ensp;此代码库包含xArm模型文件以及相关的控制、规划等示例开发包。开发及测试使用的环境如下
- Ubuntu 20.04 + ROS Foxy
- Ubuntu 20.04 ROS Galactic
- Ubuntu 22.04 + ROS Humble

&ensp;&ensp;&ensp;&ensp;请根据不同ros2版本切换到对应的代码分支（没有对应的代码分支表示未在该版本测试过）
- Foxy: [foxy](https://github.com/xArm-Developer/xarm_ros2/tree/foxy)
- Galactic: [galactic](https://github.com/xArm-Developer/xarm_ros2/tree/galactic)
- Humble: [humble](https://github.com/xArm-Developer/xarm_ros2/tree/humble)


## 2. 更新记录
- 新增xarm_gazebo以支持gazebo，并和moveit关联
- 支持加载其它模型到机械臂末端
- 新增xarm_moveit_servo支持xbox手柄/SpaceMouse/键盘控制
- (2022-09-07) 变更service(__set_tgpio_modbus_timeout__/__getset_tgpio_modbus_data__)的参数类型，增加参数支持透传
- (2022-09-07) 变更Topic名字(xarm_states改为robot_states)
- (2022-09-07) 更新子模块xarm-sdk到1.11.0版本
- (2022-09-09) [Beta]支持Ros Humble版本

## 3. 准备工作

- ### 3.1 安装 [ROS2](https://docs.ros.org/) 
  - [Foxy](https://docs.ros.org/en/ros2_documentation/foxy/Installation.html)
  - [Galactic](https://docs.ros.org/en/ros2_documentation/galactic/Installation.html)
  - [Humble](https://docs.ros.org/en/ros2_documentation/humble/Installation.html)

- ### 3.2 安装 [Moveit2](https://moveit.ros.org/install-moveit2/binary/)

- ### 3.3 安装 [Gazebo](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)  

- ### 3.4 安装 [gazebo_ros_pkgs](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)  

## 4. 使用说明

- ### 4.1 创建工作区
    ```bash
    # 如果已经有自己工作区，请跳过这一步骤
    $ cd ~
    $ mkdir -p dev_ws/src
    ```

- ### 4.2 获取xarm_ros2源码包
    ```bash
    $ cd ~/dev_ws/src
    # 注意需要--recursive参数，否则不会下载源码包的子模块源码
    $ git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive
    ```

- ### 4.3 升级xarm_ros2源码包
    ```bash
    $ cd ~/dev_ws/src/xarm_ros2
    $ git pull
    $ git submodule sync
    $ git submodule update --init --remote
    ```

- ### 4.4 安装xarm_ros2依赖
    ```bash
    # 记得先source已安装的ros2环境
    $ cd ~/dev_ws/src/
    $ rosdep update
    $ rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
    ```

- ### 4.5 编译xarm_ros2
    ```bash
    # 记得先source已安装的ros2环境和moveit2环境
    $ cd ~/dev_ws/
    # 编译所有包
    $ colcon build
    
    # 编译单个包
    $ colcon build --packages-select xarm_api
    ```


## 5. 模块说明
__注意1： 如果当前局域网有多人使用ros2，为避免相互间发生干扰，请设置一下 ROS_DOMAIN_ID__
  - [Foxy](https://docs.ros.org/en/ros2_documentation/foxy/Concepts/About-Domain-ID.html)
  - [Galactic](https://docs.ros.org/en/ros2_documentation/galactic/Concepts/About-Domain-ID.html)
  - [Humble](https://docs.ros.org/en/ros2_documentation/humble/Concepts/About-Domain-ID.html)

__注意2： 运行xarm_ros2中的程序或启动脚本之前请先source当前工作区环境__
```bash
$ cd ~/dev_ws/
$ source install/setup.bash
```
__注意3： 以下启动说明以6轴为例，5轴和7轴的用法只需找到对应的启动文件或指定对应的参数__

- ### 5.1 xarm_description
    此模块包含机械臂的描述文件，通过以下启动脚本可以在rviz中显示对应的机械臂模型
    ```bash
    $ cd ~/dev_ws/
    # add_gripper为true时会加载xarm夹爪的模型
    # add_vacuum_gripper为true时会加载xarm真空吸头的模型
    # 注意：只能加载一款末端器件
    $ ros2 launch xarm_description xarm6_rviz_display.launch.py [add_gripper:=true] [add_vacuum_gripper:=true]
    ```

- ### 5.2 xarm_msgs
    此模块包含整个xarm_ros2所使用到的服务和主题通信格式，使用时请查阅每个文件里的说明. [REAEME](./xarm_msgs/ReadMe.md)

- ### 5.3 xarm_sdk
    此模块是作为子模块存在的，子模块仓库为[xArm-CPLUS-SDK](https://github.com/xArm-Developer/xArm-CPLUS-SDK.git), 作为控制机械臂的SDK，如需使用请参阅xArm-CPLUS-SDK的文档说明

- ### 5.4 xarm_api
    此模块是针对xarm_sdk封装，提供对应的ros service和ros topic，整个xarm_ros2是通过使用此模块的service和topic来和机械臂的通信的
    所有service和topic默认都处于xarm/空间下(除非指定了hw_ns参数)，即joint_states的完整名字为xarm/joint_states
    
    - __services__: 所有提供的service的名字和SDK中的API名字是对应的，但是否创建对应的服务是根据```xarm_api/config/xarm_params.yaml```和```xarm_api/config/xarm_user_params.yaml```的services来决定的，只有当services下对应的service的值为```true```时才会创建对应的service，如果需要自定义参数，请创建```xarm_api/config/xarm_user_params.yaml```文件来修改，格式参照```xarm_api/config/xarm_params.yaml```。

        ```
        services:
            motion_enable: true
            set_mode: true
            set_state: true
            clean_conf: false
            ...
        ```

    - __topics__:  

        __joint_states__: 格式为 __sensor_msgs::msg::JointState__  

        __robot_states__: 格式为 __xarm_msgs::msg::RobotMsg__  
        
        __xarm_cgpio_states__: 格式为 __xarm_msgs::msg::CIOState__  
        
        __uf_ftsensor_raw_states__: 格式为 __geometry_msgs::msg::WrenchStamped__  
        
        __uf_ftsensor_ext_states__: 格式为 __geometry_msgs::msg::WrenchStamped__  

    
    - 启动与测试
        ```bash
        $ cd ~/dev_ws/
        # 启动xarm_driver_node
        $ ros2 launch xarm_api xarm6_driver.launch.py robot_ip:=192.168.1.117
        # 测试service
        $ ros2 run xarm_api test_xarm_ros_client
        # 测试topic
        $ ros2 run xarm_api test_robot_states
        ```

- ### 5.5 xarm_controller
    此模块是ros2_control和机械臂通信的硬件接口模块
    ```bash
    $ cd ~/dev_ws/
    # add_gripper为true时会加载xarm夹爪的模型
    $ ros2 launch xarm6_control_rviz_display.launch.py robot_ip:=192.168.1.117 [add_gripper:=true]
    ```

- ### 5.6 xarm_moveit_config
    此模块提供了通过moveit来控制机械臂的功能

    - 【虚拟】启动moveit并在rviz显示, 控制机械臂  

        ```bash
        $ cd ~/dev_ws/
        # add_gripper为true时会加载xarm夹爪的模型
        $ ros2 launch xarm_moveit_config xarm6_moveit_fake.launch.py [add_gripper:=true]
        ```
    
    - 【真机】启动moveit并在rviz显示, 控制机械臂  

        ```bash
        $ cd ~/dev_ws/
        # add_gripper为true时会加载xarm夹爪的模型
        $ ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py robot_ip:=192.168.1.117 [add_gripper:=true]
        ```
    
    - 【Dual虚拟】启动moveit并在rviz显示, 控制两台机械臂  

        ```bash
        $ cd ~/dev_ws/
        # add_gripper为true时会加载xarm夹爪的模型
        # add_gripper_1参数可以单独指定左臂是否加载夹爪的模型，默认为add_gripper的值
        # add_gripper_2参数可以单独指定右臂是否加载夹爪的模型，默认为add_gripper的值
        # dof_1参数可以单独指定左臂轴数，默认为dof的值（这里为6，不同启动脚本不一样）
        # dof_2参数可以单独指定右臂轴数，默认为dof的值（这里为6，不同启动脚本不一样）
        $ ros2 launch xarm_moveit_config dual_xarm6_moveit_fake.launch.py [add_gripper:=true]
        ```
    
    - 【Dual真机】启动moveit并在rviz显示, 控制两台机械臂   
    
        ```bash
        $ cd ~/dev_ws/
        # robot_ip_1表示左臂控制的IP地址
        # robot_ip_2表示右臂控制的IP地址
        # add_gripper为true时会加载xarm夹爪的模型
        # add_gripper_1参数可以单独指定左臂是否加载夹爪的模型，默认为add_gripper的值
        # add_gripper_2参数可以单独指定右臂是否加载夹爪的模型，默认为add_gripper的值
        # dof_1参数可以单独指定左臂轴数，默认为dof的值（这里为6，不同启动脚本不一样）
        # dof_2参数可以单独指定右臂轴数，默认为dof的值（这里为6，不同启动脚本不一样）
        $ ros2 launch xarm_moveit_config dual_xarm6_moveit_realmove.launch.py robot_ip_1_1:=192.168.1.117 robot_ip_2:=192.168.1.203 [add_gripper:=true]
        ```

- ### 5.7 xarm_planner
    此模块提供了通过moveit API控制机械臂
    ```bash
    $ cd ~/dev_ws/
    # 【虚拟】启动xarm_planner_node
    $ ros2 launch xarm_planner xarm6_planner_fake.launch.py [add_gripper:=true]
    # 【真机】启动xarm_planner_node
    # $ ros2 launch xarm_planner xarm6_planner_realmove.launch.py robot_ip:=192.168.1.117 [add_gripper:=true]

    # 运行测试(通过API控制)
    $ ros2 launch xarm_planner test_xarm_planner_api_joint.launch.py dof:=6
    $ ros2 launch xarm_planner test_xarm_planner_api_pose.launch.py dof:=6

    # 运行测试（通过service控制）
    $ ros2 launch xarm_planner test_xarm_planner_client_joint.launch.py dof:=6
    $ ros2 launch xarm_planner test_xarm_planner_client_pose.launch.py dof:=6

    # 运行测试（通过API控制机械爪）
    $ ros2 launch xarm_planner test_xarm_gripper_planner_api_joint.launch.py dof:=6

    # 运行测试（通过service控制机械爪）
    $ ros2 launch xarm_planner test_xarm_gripper_planner_client_joint.launch.py dof:=6
    ```


- ### 5.8 xarm_gazebo
    此模块用于在gazobo上对xarm进行仿真。  
    注意：  
    (1) 可能需要源码安装[gazebo_ros2_control](https://github.com/ros-simulation/gazebo_ros2_control.git)，并source所安装的gazebo_ros2_control环境。  
    (2) [minic_joint_plugin](https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins)是基于ROS1开发，我们基于此修改并集成了ROS2兼容的插件版本，供xArm Gripper仿真使用。  
    
    - 单独测试xarm在gazebo上的显示：
        ```bash
        $ cd ~/dev_ws/
        $ ros2 launch xarm_gazebo xarm6_beside_table_gazebo.launch.py
        ```

    - 联合moveit+gazebo进行控制：
        ```bash
        $ cd ~/dev_ws/
        $ ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py
        ```

- ### 5.9 xarm_moveit_servo
    此模块用于通过外部输入来控制机械臂, 基于[moveit_servo](http://moveit2_tutorials.picknik.ai/doc/realtime_servo/realtime_servo_tutorial.html)。  
    - 通过 __XBOX360__ 手柄控制
        - 左摇杆控制TCP的X和Y
        - 右摇杆控制TCP的ROLL和PITCH
        - [前面]左右两个触发器控制TCP的Z
        - [前面]左右两个缓冲器控制TCP的YAW
        - 十字键控制关节1和关节2的转动
        - 按键X和按键B控制最后一个关节的转动
        - 按键Y和按键A控制倒数第二个关节的转动

        ```bash
        $ cd ~/dev_ws/
        # XBOX Wired -> joystick_type=1
        # XBOX Wireless -> joystick_type=2
        # 控制虚拟机械臂
        $ ros2 launch xarm_moveit_servo xarm_moveit_servo_fake.launch.py joystick_type:=1

        # 控制真实机械臂
        # ros2 launch xarm_moveit_servo xarm_moveit_servo_realmove.launch.py robot_ip:=192.168.1.123 dof:=5 joystick_type:=1
        ```

    - 通过六维鼠标 __3Dconnexion SpaceMouse Wireless__ 来控制
        - 六维鼠标的六个维度对应控制TCP的X/Y/Z/ROLL/PITCH/YAW
        - 左边按键按下时单独控制TCP的XYZ
        - 右边按键按下时单独控制TCP的ROLL/PITCH/YAW

        ```bash
        $ cd ~/dev_ws/
        # 控制虚拟机械臂
        $ ros2 launch xarm_moveit_servo xarm_moveit_servo_fake.launch.py joystick_type:=3

        # 控制真实机械臂
        # ros2 launch xarm_moveit_servo xarm_moveit_servo_realmove.launch.py robot_ip:=192.168.1.123 dof:=5 joystick_type:=3
        ```
    - 通过 __键盘输入__ 控制
        ```bash
        $ cd ~/dev_ws/
        # 控制虚拟机械臂
        $ ros2 launch xarm_moveit_servo xarm_moveit_servo_fake.launch.py

        # 控制真实机械臂
        # ros2 launch xarm_moveit_servo xarm_moveit_servo_realmove.launch.py robot_ip:=192.168.1.123 dof:=5

        # 运行键盘输入节点
        $ ros2 run xarm_moveit_servo xarm_keyboard_input
        ```


## 6. 主要启动参数说明
- __robot_ip__
    机械臂IP地址，控制真机时需要。  
- __report_type__, 默认normal。  
    上报类型，支持normal/rich/dev，  
    不同上报类型的上报数据和上报频率不一样。  
- __dof__, 默认为7。
    机械臂轴数，如非必须参数一般不需要指定。  
    对于双臂启动脚本(dual_开头的)，可以通过以下参数分别指定：
    - __dof_1__
    - __dof_2__
- __velocity_control__, 默认为false。  
    是否使用速度控制。  
- __add_gripper__, 默认为false。  
    是否添加UF机械爪xarm_gripper，优先级高于参数```add_vacuum_gripper```。
    对于双臂启动脚本(dual_开头的)，可以通过以下参数分别指定：  
    - __add_gripper_1__
    - __add_gripper_2__
- __add_vacuum_gripper__, 默认为false。  
    是否添加UF吸泵xarm_vacuum_gripper，设置为true的前提必须要设置参数```add_gripper```为```false```
    对于双臂启动脚本(dual_开头的)，可以通过以下参数分别指定：  
    - __add_vacuum_gripper_1__
    - __add_vacuum_gripper_2__
- __add_other_geometry__, 默认为false。  
    是否添加其它几何模型到末端，设置为true的前提:参数```add_gripper```和```add_vacuum_gripper```必须为```false```  
    
    - __geometry_type__, 默认为box, 仅仅在```add_other_geometry=true```时有效。
        要添加的几何模型的类型，支持box/cylinder/sphere/mesh。
    - __geometry_mass__, 单位(kg)，默认0.1。  
        几何模型质量。
    - __geometry_height__, 单位(米)，默认0.1。  
        几何模型高度，geometry_type为box/cylinder/sphere有效。
    - __geometry_radius__, 单位(米)，默认0.1。  
        几何模型半径，geometry_type为cylinder/sphere有效。
    - __geometry_length__, 单位(米)，默认0.1。
        几何模型长度，geometry_type为box有效。
    - __geometry_width__, 单位(米)，默认0.1。
        几何模型宽度，geometry_type为box有效。
    - __geometry_mesh_filename__,
        几何模型的文件名，geometry_type为mesh有效, ***该文件需要存放于```xarm_description/meshes/other/```目录下面***，这样就不需要在文件名里指定文件目录了。  
    - __geometry_mesh_origin_xyz__, 默认"0 0 0"
    - __geometry_mesh_origin_rpy__, 默认"0 0 0"
        几何模型的基准参考系相对于xArm末端法兰的参考系，geometry_type为
        ```mesh```有效。使用时注意引号: geometry_mesh_origin_xyz:='"0.05 0.0 0.0"'.  
    - __geometry_mesh_tcp_xyz__, 默认"0 0 0"
    - __geometry_mesh_tcp_rpy__, 默认"0 0 0"
        几何模型末端(TCP)相对于几何模型基准参考系的偏移，geometry_type为```mesh```有效。使用时注意引号: geometry_mesh_tcp_rpy:='"0.0 0.0 1.5708"'.  
    - __添加自定义末端工具(圆柱体)示例__  
    
        ```bash
        $ ros2 launch xarm_gazebo xarm6_beside_table_gazebo.launch.py add_other_geometry:=true geometry_type:=cylinder geometry_height:=0.075 geometry_radius:=0.045
        ```

    对于双臂启动脚本(dual_开头的)，可以通过以下参数分别指定:
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