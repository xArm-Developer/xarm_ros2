# xarm_ros2测试说明（暂定调试使用）

## 获取源码包
    ```bash
    $ cd ~/dev_ws/src
    $ git clone git@192.168.1.19:vinman/xarm_ros2.git --recursive
    ```

## 升级源码包
    ```bash
    $ cd ~/dev_ws/src/xarm_ros2
    $ git pull
    $ git submodule sync
    $ git submodule update --remote
    ```

## 编译
    ```bash
    $ cd ~/dev_ws/
    # 编译所有包
    $ colcon build
    # 编译单个包
    # $ colcon build --packages-select xarm_api
    ```

## 6. 测试 (xarm6为例)
### 6.1 测试包(xarm_api)
- #### 6.1.1 运行 xarm_driver_node
    ```bash
    $ cd ~/dev_ws/
    $ source install/setup.bash
    $ ros2 launch xarm_api xarm6_driver.launch.py robot_ip:=192.168.1.117
    ```
- #### 6.1.2 测试 xarm_ros_client
    ```bash
    $ cd ~/dev_ws/
    $ source install/setup.bash
    $ ros2 run xarm_api test_xarm_ros_client
    ```
- #### 6.1.3 测试 xarm_velo_move 
    ```bash
    $ cd ~/dev_ws/
    $ source install/setup.bash
    $ ros2 run xarm_api test_xarm_velo_move
    ```

- #### 6.1.4 测试 xarm_states_topic 
    ```bash
    $ cd ~/dev_ws/
    $ source install/setup.bash
    $ ros2 run xarm_api test_xarm_states
    ```

### 6.2 测试包(xarm_description)
- #### 6.2.1 测试rviz显示模型
    ```bash
    $ cd ~/dev_ws/
    $ source install/setup.bash
    $ ros2 launch xarm_description xarm6_rviz_display.launch.py
    ```

### 6.3 测试包(xarm_controller)
- #### 6.3.1 启动xarm_control并在rviz显示对应机械臂位置
    ```bash
    $ cd ~/dev_ws/
    $ source install/setup.bash
    $ ros2 launch xarm_controller xarm6_control_rviz_display.launch.py robot_ip:=192.168.1.117
    ```

### 6.4 测试包(xarm_moveit_config)
- #### 6.4.1 启动moveit并在rviz显示, 控制虚拟机械臂
    ```bash
    $ cd ~/dev_ws/
    $ source install/setup.bash
    $ ros2 launch xarm_moveit_config xarm6_moveit_fake.launch.py
    ```

- #### 6.4.2 启动moveit并在rviz显示, 控制真实机械臂
    ```bash
    $ cd ~/dev_ws/
    $ source install/setup.bash
    $ ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py
    ```
