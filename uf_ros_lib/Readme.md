# uf_ros_lib
  This package specifically implements some python libraries for easy direct use.

### moveit_configs_builder
  Because our moveit package supports many different robot arms, and the configuration files used are dynamically generated based on parameters, we cannot directly use the MoveitConfigsBuilder provided by the official moveit package. Here we implement MoveitConfigsBuilder and DualMoveitConfigsBuilder similarly for our moveit package.

  - #### MoveItConfigsBuilder
    ```python
    class MoveItConfigsBuilder(
      context: launch.launch_context.LaunchContext,
      controllers_name: 'fake_controllers' or 'controllers',
      **kwargs
    )
    ```
    - context: current context, instance of launch.launch_context.LaunchContext
    - controllers_name: 'controllers' if realmove else 'fake_controllers' 
    - kwargs: (Key parameters and default values)
      - __robot_type__: 'xarm'
      - __dof__: 7
      - prefix: ''
      - hw_ns: 'xarm'
      - limited: False
      - effort_control: False
      - velocity_control: False
      - __add_gripper__: False
      - add_vacuum_gripper: False
      - __add_bio_gripper__: False,
      - ros2_control_plugin: 'uf_robot_hardware/UFRobotSystemHardware'
      - add_realsense_d435i: False
      - add_d435i_links: True
      - model1300: False
      - attach_to: 'world'
      - attach_xyz: '0 0 0'
      - attach_rpy: '0 0 0'
      - add_other_geometry: False
      - geometry_type: 'box'
      - geometry_mass: 0.1
      - geometry_height: 0.1
      - geometry_radius: 0.1
      - geometry_length: 0.1
      - geometry_width: 0.1
      - geometry_mesh_filename: ''
      - geometry_mesh_origin_xyz: '0 0 0'
      - geometry_mesh_origin_rpy: '0 0 0'
      - geometry_mesh_tcp_xyz: '0 0 0'
      - geometry_mesh_tcp_rpy: '0 0 0'
      - kinematics_suffix: ''
    
    Example:
    ```python
    from launch_ros.actions import Node
    from launch.substitutions import LaunchConfiguration
    from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder

    def launch_setup(context, *args, **kwargs):
      prefix = LaunchConfiguration('prefix', default='')
      hw_ns = LaunchConfiguration('hw_ns', default='xarm')
      add_gripper = LaunchConfiguration('add_gripper', default=False)
      add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
      dof = LaunchConfiguration('dof', default=7)
      robot_type = LaunchConfiguration('robot_type', default='xarm')

      moveit_configs = MoveItConfigsBuilder(
        context=context,
        controllers_name='fake_controllers',
        robot_type=robot_type,
        dof=dof,
        prefix=prefix,
        hw_ns=hw_ns,
        add_gripper=add_gripper,
        add_bio_gripper=add_bio_gripper,
      ).to_moveit_configs()

      # moveit_configs.robot_description
      # moveit_configs.robot_description_semantic
      # moveit_configs.robot_description_kinematics
      # moveit_configs.planning_pipelines
      # moveit_configs.trajectory_execution
      # moveit_configs.planning_scene_monitor
      # moveit_configs.joint_limits
      # moveit_configs.to_dict()

      move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
          moveit_configs.to_dict()
        ],
      )
      return move_group_node
    ```
    

  - #### DualMoveItConfigsBuilder
    ```python
    class DualMoveItConfigsBuilder(
      context: launch.launch_context.LaunchContext,
      controllers_name: 'fake_controllers' or 'controllers',
      **kwargs
    )
    ```
    - context: current context, instance of launch.launch_context.LaunchContext
    - controllers_name: 'controllers' if realmove else 'fake_controllers' 
    - **kwargs: (Key parameters and default values)
      - prefix_1: 'L_'
      - prefix_2: 'R_'
      - __dof_1__: 7
      - __dof_2__: 7
      - __robot_type_1__: 'xarm'
      - __robot_type_2__: 'xarm'
      - hw_ns: 'xarm'
      - limited: False
      - effort_control: False
      - velocity_control: False
      - ros2_control_plugin: 'uf_robot_hardware/UFRobotSystemHardware'
      - __add_gripper_1__: False
      - __add_gripper_2__: False
      - add_vacuum_gripper_1: False
      - add_vacuum_gripper_2: False
      - __add_bio_gripper_1__: False
      - __add_bio_gripper_2__: False
      - model1300_1: False
      - model1300_2: False
      - robot_sn_1: ''
      - robot_sn_2: ''
      - kinematics_suffix_1: ''
      - kinematics_suffix_2: ''
      - add_realsense_d435i_1: False
      - add_realsense_d435i_2: False
      - add_d435i_links_1: True
      - add_d435i_links_2: True
      - add_other_geometry_1: False
      - add_other_geometry_2: False
      - geometry_type_1: 'box'
      - geometry_type_2: 'box'
      - geometry_mass_1: 0.1
      - geometry_mass_2: 0.1
      - geometry_height_1: 0.1
      - geometry_height_2: 0.1
      - geometry_radius_1: 0.1
      - geometry_radius_2: 0.1
      - geometry_length_1: 0.1
      - geometry_length_2: 0.1
      - geometry_width_1: 0.1
      - geometry_width_2: 0.1
      - geometry_mesh_filename_1: ''
      - geometry_mesh_filename_2: ''
      - geometry_mesh_origin_xyz_1: '0 0 0'
      - geometry_mesh_origin_xyz_2: '0 0 0'
      - geometry_mesh_origin_rpy_1: '0 0 0'
      - geometry_mesh_origin_rpy_2: '0 0 0'
      - geometry_mesh_tcp_xyz_1: '0 0 0'
      - geometry_mesh_tcp_xyz_2: '0 0 0'
      - geometry_mesh_tcp_rpy_1: '0 0 0'
      - geometry_mesh_tcp_rpy_2: '0 0 0'
    
    Example:
    ```python
    from launch_ros.actions import Node
    from launch.substitutions import LaunchConfiguration
    from uf_ros_lib.moveit_configs_builder import DualMoveItConfigsBuilder

    def launch_setup(context, *args, **kwargs):
      prefix_1 = LaunchConfiguration('prefix_1', default='L_')
      prefix_2 = LaunchConfiguration('prefix_2', default='R_')
      hw_ns = LaunchConfiguration('hw_ns', default='xarm')
      add_gripper_1 = LaunchConfiguration('add_gripper_1', default=False)
      add_gripper_2 = LaunchConfiguration('add_gripper_2', default=False)
      add_bio_gripper_1 = LaunchConfiguration('add_bio_gripper_1', default=False)
      add_bio_gripper_2 = LaunchConfiguration('add_bio_gripper_2', default=False)
      dof_1 = LaunchConfiguration('dof_1', default=7)
      dof_2 = LaunchConfiguration('dof_2', default=7)
      robot_type_1 = LaunchConfiguration('robot_type_1', default='xarm')
      robot_type_2 = LaunchConfiguration('robot_type_2', default='xarm')

      moveit_configs = DualMoveItConfigsBuilder(
        context=context,
        controllers_name='fake_controllers',
        robot_type_1=robot_type_1,
        robot_type_2=robot_type_2,
        dof_1=dof_1,
        dof_2=dof_2,
        prefix_1=prefix_1,
        prefix_2=prefix_2,
        hw_ns=hw_ns,
        add_gripper_1=add_gripper_1,
        add_gripper_2=add_gripper_2,
        add_bio_gripper_1=add_bio_gripper_1,
        add_bio_gripper_2=add_bio_gripper_2,
      ).to_moveit_configs()

      # moveit_configs.robot_description
      # moveit_configs.robot_description_semantic
      # moveit_configs.robot_description_kinematics
      # moveit_configs.planning_pipelines
      # moveit_configs.trajectory_execution
      # moveit_configs.planning_scene_monitor
      # moveit_configs.joint_limits
      # moveit_configs.to_dict()

      move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
          moveit_configs.to_dict()
        ],
      )
      return move_group_node
    ```
