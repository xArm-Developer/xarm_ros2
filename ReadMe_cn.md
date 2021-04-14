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

## 测试
### 测试包（xarm_api）
- #### 运行 xarm_driver_node
    ```bash
    $ cd ~/dev_ws/
    $ source install/setup.bash
    # 通过xarm6_driver.launch启动
    $ ros2 launch xarm_api xarm6_driver.launch robot_ip:=192.168.1.117
    # 通过xarm_driver_launch.py启动
    # $ ros2 launch xarm_api xarm_driver_launch.py robot_ip:=192.168.1.117 dof:=6
    ```
- #### 测试 xarm_ros_client
    ```bash
    $ cd ~/dev_ws/
    $ source install/setup.bash
    $ ros2 run xarm_api test_xarm_ros_client
    ```
- #### 测试 xarm_velo_move 
    ```bash
    $ cd ~/dev_ws/
    $ source install/setup.bash
    $ ros2 run xarm_api test_xarm_velo_move
    ```

- #### 测试 xarm_states_topic 
    ```bash
    $ cd ~/dev_ws/
    $ source install/setup.bash
    $ ros2 run xarm_api test_xarm_states
    ```