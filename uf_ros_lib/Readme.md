# uf_ros_lib
  This package specifically implements some python libraries for easy direct use.

### moveit_configs_builder
  Because our moveit package supports many different robot arms, and the configuration files used are dynamically generated based on parameters, we cannot directly use the MoveitConfigsBuilder provided by the official moveit package. Here we implement MoveitConfigsBuilder and DualMoveitConfigsBuilder similarly for our moveit package.

  Reference to [moveit_configs_build](https://github.com/moveit/moveit2/blob/main/moveit_configs_utils/moveit_configs_utils/moveit_configs_builder.py) from moveit.

  Regarding planning pipe, only __ompl__ is loaded by default. If you need other ones (__chomp__, __pilz_industrial_motion_planner__, __stomp__), you need to install them yourself and create configuration files in the corresponding configuration folders. *For example, if xarm6 loads __pilz_industrial_motion_planner__, you need to create __pilz_industrial_motion_planner_planning.yaml__ in __xarm_moveit_config/config/xarm6__*. 
  (__Note: the pipes listed above are not supported by all ROS versions__)

  - #### MoveItConfigsBuilder
    ```python
    class MoveItConfigsBuilder(
      context: launch.launch_context.LaunchContext,
      controllers_name: 'fake_controllers' or 'controllers',
      **kwargs
    )
    ```
    - context: current context, instance of launch.launch_context.LaunchContext
    - __controllers_name__: 'controllers' if realrobot else 'fake_controllers' 
      The default file name loaded by the trajectory_execution method, such as *xarm_moveit_config/config/{robot}/{controllers_name}.yaml*
    - kwargs: (key parameters and default values)
      - __robot_ip__: {robot_ip} if realrobot else ''
      - report_type: 'normal'
      - baud_checkset: True
      - default_gripper_baud: 2000000
      - __dof__: 5/6/7
      - __robot_type__: xarm/lite/uf850/xarm7_mirror
      - prefix: ''
      - hw_ns: 'xarm'
      - limited: False
      - effort_control: False
      - velocity_control: False
      - model1300: False
      - robot_sn: {robot_sn}
      - attach_to: 'world'
      - attach_xyz: '0 0 0'
      - attach_rpy: '0 0 0'
      - mesh_suffix: 'stl'
      - kinematics_suffix: ''
      - __ros2_control_plugin__: hardware interface plugin
        - FakeRobot: 'uf_robot_hardware/UFRobotFakeSystemHardware'
        - RealRobot: 'uf_robot_hardware/UFRobotSystemHardware'
        - GazeboRobot: 'gazebo_ros2_control/GazeboSystem'
      - ros2_control_params: the configuration file path loaded by gazebo_ros2_control
      - __add_gripper__: False
      - add_vacuum_gripper: False
      - __add_bio_gripper__: False
      - add_realsense_d435i: False
      - add_d435i_links: True
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
    
    - __Example(FakeRobot)__: please refer to [demo_fake.launch.py](../xarm_moveit_config/launch/demo/demo_fake.launch.py) or [robot_moveit_fake.launch.py](../xarm_moveit_config/launch/_robot_moveit_fake.launch.py) for the example

      ```python
      from launch_ros.actions import Node
      from launch.substitutions import LaunchConfiguration
      from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder

      def launch_setup(context, *args, **kwargs):
        dof = LaunchConfiguration('dof', default=7)
        robot_type = LaunchConfiguration('robot_type', default='xarm')
        prefix = LaunchConfiguration('prefix', default='')
        hw_ns = LaunchConfiguration('hw_ns', default='xarm')
        add_gripper = LaunchConfiguration('add_gripper', default=False)
        add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
        
        moveit_configs = MoveItConfigsBuilder(
          context=context,
          controllers_name='fake_controllers',
          dof=dof,
          robot_type=robot_type,
          prefix=prefix,
          hw_ns=hw_ns,
          ros2_control_plugin='uf_robot_hardware/UFRobotFakeSystemHardware',
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
    - __Example(RealRobot)__: please refer to [demo_realmove.launch.py](../xarm_moveit_config/launch/demo/demo_realmove.launch.py) or [robot_moveit_realmove.launch.py](../xarm_moveit_config/launch/_robot_moveit_realmove.launch.py) for the example

      ```python
      from launch_ros.actions import Node
      from launch.substitutions import LaunchConfiguration
      from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder

      def launch_setup(context, *args, **kwargs):
        robot_ip = LaunchConfiguration('robot_ip')
        dof = LaunchConfiguration('dof', default=7)
        robot_type = LaunchConfiguration('robot_type', default='xarm')
        prefix = LaunchConfiguration('prefix', default='')
        hw_ns = LaunchConfiguration('hw_ns', default='xarm')
        add_gripper = LaunchConfiguration('add_gripper', default=False)
        add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)

        moveit_configs = MoveItConfigsBuilder(
          context=context,
          controllers_name='controllers',
          robot_ip=robot_ip,
          dof=dof,
          robot_type=robot_type,
          prefix=prefix,
          hw_ns=hw_ns,
          ros2_control_plugin='uf_robot_hardware/UFRobotSystemHardware',
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
    - __controllers_name__: 'controllers' if realrobot else 'fake_controllers' 
      The default file name loaded by the trajectory_execution method, such as *xarm_moveit_config/config/{robot}/{controllers_name}.yaml*
    - **kwargs: (key parameters and default values)
      - __robot_ip_1__: {robot_ip_1} if realrobot else ''
      - __robot_ip_2__: {robot_ip_2} if realrobot else ''
      - report_type_1: 'normal'
      - report_type_2: 'normal'
      - baud_checkset_1: True
      - baud_checkset_2: True
      - default_gripper_baud_1: 2000000
      - default_gripper_baud_2: 2000000
      - __dof_1__: 5/6/7
      - __dof_2__: 5/6/7
      - __robot_type_1__: xarm/lite/uf850/xarm7_mirror
      - __robot_type_2__: xarm/lite/uf850/xarm7_mirror
      - prefix_1: 'L_'
      - prefix_2: 'R_'
      - hw_ns: 'xarm'
      - limited: False
      - effort_control: False
      - velocity_control: False
      - model1300_1: 
      - model1300_2: 
      - robot_sn_1: 
      - robot_sn_2: 
      - mesh_suffix: 'stl'
      - kinematics_suffix_1: ''
      - kinematics_suffix_2: ''
      - __ros2_control_plugin__: hardware interface plugin
        - FakeRobot: 'uf_robot_hardware/UFRobotFakeSystemHardware'
        - RealRobot: 'uf_robot_hardware/UFRobotSystemHardware'
        - GazeboRobot: 'gazebo_ros2_control/GazeboSystem'
      - ros2_control_params: the configuration file path loaded by gazebo_ros2_control
      - __add_gripper_1__: False
      - __add_gripper_2__: False
      - add_vacuum_gripper_1: False
      - add_vacuum_gripper_2: False
      - __add_bio_gripper_1__: False
      - __add_bio_gripper_2__: False
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
    
    - __Example(DualFakeRobot)__: please refer to [demo_dual_fake.launch.py](../xarm_moveit_config/launch/demo/demo_dual_fake.launch.py) or [dual_robot_moveit_fake.launch.py](../xarm_moveit_config/launch/_dual_robot_moveit_fake.launch.py) for the example
      
      ```python
      from launch_ros.actions import Node
      from launch.substitutions import LaunchConfiguration
      from uf_ros_lib.moveit_configs_builder import DualMoveItConfigsBuilder

      def launch_setup(context, *args, **kwargs):
        dof_1 = LaunchConfiguration('dof_1', default=7)
        dof_2 = LaunchConfiguration('dof_2', default=7)
        robot_type_1 = LaunchConfiguration('robot_type_1', default='xarm')
        robot_type_2 = LaunchConfiguration('robot_type_2', default='xarm')
        prefix_1 = LaunchConfiguration('prefix_1', default='L_')
        prefix_2 = LaunchConfiguration('prefix_2', default='R_')
        hw_ns = LaunchConfiguration('hw_ns', default='xarm')
        add_gripper_1 = LaunchConfiguration('add_gripper_1', default=False)
        add_gripper_2 = LaunchConfiguration('add_gripper_2', default=False)
        add_bio_gripper_1 = LaunchConfiguration('add_bio_gripper_1', default=False)
        add_bio_gripper_2 = LaunchConfiguration('add_bio_gripper_2', default=False)

        moveit_configs = DualMoveItConfigsBuilder(
          context=context,
          controllers_name='fake_controllers',
          dof_1=dof_1,
          dof_2=dof_2,
          robot_type_1=robot_type_1,
          robot_type_2=robot_type_2,
          prefix_1=prefix_1,
          prefix_2=prefix_2,
          hw_ns=hw_ns,
          ros2_control_plugin='uf_robot_hardware/UFRobotFakeSystemHardware',
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
    - __Example(DualRealRobot)__: please refer to [demo_dual_realmove.launch.py](../xarm_moveit_config/launch/demo/demo_dual_realmove.launch.py) or [dual_robot_moveit_realmove.launch.py](../xarm_moveit_config/launch/_dual_robot_moveit_realmove.launch.py) for the example
      ```python
      from launch_ros.actions import Node
      from launch.substitutions import LaunchConfiguration
      from uf_ros_lib.moveit_configs_builder import DualMoveItConfigsBuilder

      def launch_setup(context, *args, **kwargs):
        robot_ip_1 = LaunchConfiguration('robot_ip_1')
        robot_ip_2 = LaunchConfiguration('robot_ip_2')
        dof_1 = LaunchConfiguration('dof_1', default=7)
        dof_2 = LaunchConfiguration('dof_2', default=7)
        robot_type_1 = LaunchConfiguration('robot_type_1', default='xarm')
        robot_type_2 = LaunchConfiguration('robot_type_2', default='xarm')
        prefix_1 = LaunchConfiguration('prefix_1', default='L_')
        prefix_2 = LaunchConfiguration('prefix_2', default='R_')
        hw_ns = LaunchConfiguration('hw_ns', default='xarm')
        add_gripper_1 = LaunchConfiguration('add_gripper_1', default=False)
        add_gripper_2 = LaunchConfiguration('add_gripper_2', default=False)
        add_bio_gripper_1 = LaunchConfiguration('add_bio_gripper_1', default=False)
        add_bio_gripper_2 = LaunchConfiguration('add_bio_gripper_2', default=False)

        moveit_configs = DualMoveItConfigsBuilder(
          context=context,
          controllers_name='controllers',
          robot_ip_1=robot_ip_1,
          robot_ip_2=robot_ip_2,
          dof_1=dof_1,
          dof_2=dof_2,
          robot_type_1=robot_type_1,
          robot_type_2=robot_type_2,
          prefix_1=prefix_1,
          prefix_2=prefix_2,
          hw_ns=hw_ns,
          ros2_control_plugin='uf_robot_hardware/UFRobotSystemHardware',
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
