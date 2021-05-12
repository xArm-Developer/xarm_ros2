# 问题及解决

## 1. ROS2 相关

## 2. moveit2相关


## 3. xarm_ros2相关
- ### 3.1 运行xarm_controller触发 std::length_error ... what(): basic_string::_M_create
    原因：moveit2下的ros2_control代码不兼容（commit为2021-04-27及之后的都不兼容）
    - 解决方案1: 回退版本
    ```bash
    # 切换到moveit2命名空间下的ros2_control
    $ cd ws_moveit2/src/ros2_control 
    # 回退代码版本
    $ git reset --hard e16bfd298be8e7a732a65ab31db33c2f2cfbd227
    # 重新编译
    $ cd ../../
    $ colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```
    - 解决方案2： 修改ros2_control源码

    ```bash
    # 1. 修改ros2_control/hardware_interface/include/hardware_interface/system_interface.hpp
    #      删除prepare_command_mode_switch和perform_command_mode_switch前面的virtual关键字
    # 2. 重新编译（不编译测试用例）
    $ colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=false
        
    ```

