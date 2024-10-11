# This is a demo connecting the chassis and xarm6
Mainly to demonstrate how to build and drive the xarm and chassis together.
__Here, mbot is used instead of chassis.__

### URDF
- __*urdf/mbot_with_xarm.urdf.xacro*__
  - Define the parameters required by the xarm urdf and pass them to the xarm urdf (see the parameters of the xarm urdf)
    - xarm urdf： *xarm_description/urdf/xarm_device_macro.xacro*
    - __Note： Please set the values ​​of some parameters according to the actual situation__
      - __In the demo, set the parameter attach_to to the "base_link" of the mbot and modify the offset attach_xyz__

  - Define the parameters required for mbot urdf and pass them to mbot urdf (see mbot urdf for parameters)
    - mbot urdf: *urdf/mbot_macro.xacro*
      - *urdf/mbot.urdf.xacro*
      - *urdf/mbot.transmission.xacro*
      - *urdf/mbot.ros2_control.xacro*
      - *urdf/mbot.gazebo.xacro*
    - __Note: The demo only defines and passes the parameter ros2_control_plugin__

### SRDF
- __*mbot_with_xarm.srdf.xacro*__
  - Define the parameters required by the xarm srdf and pass them to the xarm srdf (see the parameters of the xarm srdf)
    - xarm srdf： *xarm_moveit_config/srdf/xarm_macro.srdf.xacro*
  - Define the parameters required by mbot srdf and pass them to mbot srdf (see mbot srdf for parameters)
    - mbot srdf: *srdf/mbot_macro.srdf.xacro*
    - __Note: The demo only defines and passes the prefix parameter__

### ros2_control config
- __*config/ros2_controllers.yaml*__: Contains xarm and mbot configuration
  - __Note: You can copy the corresponding xarm configuration from xarm_controller/config and then add the mbot configuration__

### moveit config
- *config/controllers.yaml*
- *config/joint_limits.yaml*
- *config/kinematics.yaml*
- *config/ompl_planning.yaml*
- __注__
  - __You can copy the corresponding xarm configuration from xarm_moveit_config/config and then add the mbot configuration__
  - __The demo does not add any mbot configuration.__

### rviz config
- *rviv/moveit.rviz*
- __Note__
  - __Demo is just copied from xarm_moveit_config/rviz/moveit.rviz and the "Fixed Frame" is modified to be the "base_footprint" link of the mbot__

### gazebo world file
- *worlds/empty.world*

### launch file
- *launch/mbot_moveit_fake.launch.py*
- *launch/mbot_moveit_gazebo.launch.py*
  - *launch/_robot_on_mbot_gazebo.launch.py*
- *launch/mbot_moveit_realmove.launch.py*

- __Note__
  - __When generating ros2_control parameters, specify the use of the config/ros2_controller.yalm file__
    ```python
    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('mbot_demo'), 'config', 'ros2_controllers.yaml'),
        prefix=prefix.perform(context), 
        add_gripper=add_gripper.perform(context) in ('True', 'true'),
        add_bio_gripper=add_bio_gripper.perform(context) in ('True', 'true'),
        ros_namespace=ros_namespace,
        robot_type=robot_type.perform(context)
    )
    ```
  - __ros2_control_plugin parameters are set according to actual conditions__
    - __uf_robot_hardware/UFRobotFakeSystemHardware__: only for fake
    - __uf_robot_hardware/UFRobotSystemHardware__: only for xarm real machine
    - __gazebo_ros2_control/GazeboSystem__: only for gazebo
  - __Specify the file path when generating moveit_config__
    ```python
    pkg_path = os.path.join(get_package_share_directory('mbot_demo'))
    urdf_file = os.path.join(pkg_path, 'urdf', 'mbot_with_xarm.urdf.xacro')
    srdf_file = os.path.join(pkg_path, 'srdf', 'mbot_with_xarm.srdf.xacro')

    controllers_file = os.path.join(pkg_path, 'config', 'controllers.yaml')
    joint_limits_file = os.path.join(pkg_path, 'config', 'joint_limits.yaml')
    kinematics_file = os.path.join(pkg_path, 'config', 'kinematics.yaml')
    pipeline_filedir = os.path.join(pkg_path, 'config')

    # demo for fake
    moveit_config = (
        MoveItConfigsBuilder(
            context=context,
            dof=dof,
            robot_type=robot_type,
            prefix=prefix,
            limited=limited,
            attach_to=attach_to,
            attach_xyz=attach_xyz,
            attach_rpy=attach_rpy,
            ros2_control_plugin=ros2_control_plugin,
            ros2_control_params=ros2_control_params,
            add_gripper=add_gripper,
            add_vacuum_gripper=add_vacuum_gripper,
            add_bio_gripper=add_bio_gripper,
        )
        .robot_description(file_path=urdf_file) # urdf path
        .robot_description_semantic(file_path=srdf_file) # srdf path
        .robot_description_kinematics(file_path=kinematics_file) # kinematics path
        .joint_limits(file_path=joint_limits_file) # joint_limits path
        .trajectory_execution(file_path=controllers_file) # controllers path
        .planning_pipelines(config_folder=pipeline_filedir) # planning pipeline config directory
        .to_moveit_configs()
    )
    ```
