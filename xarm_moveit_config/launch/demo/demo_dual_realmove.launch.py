import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from uf_ros_lib.uf_robot_utils import generate_dual_ros2_control_params_temp_file
from uf_ros_lib.moveit_configs_builder import DualMoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    robot_ip_1 = LaunchConfiguration('robot_ip_1')
    robot_ip_2 = LaunchConfiguration('robot_ip_2')
    dof = LaunchConfiguration('dof', default=7)
    dof_1 = LaunchConfiguration('dof_1', default=dof)
    dof_2 = LaunchConfiguration('dof_2', default=dof)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    robot_type_1 = LaunchConfiguration('robot_type_1', default=robot_type)
    robot_type_2 = LaunchConfiguration('robot_type_2', default=robot_type)
    prefix_1 = LaunchConfiguration('prefix_1', default='L_')
    prefix_2 = LaunchConfiguration('prefix_2', default='R_')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_gripper_1 = LaunchConfiguration('add_gripper_1', default=add_gripper)
    add_gripper_2 = LaunchConfiguration('add_gripper_2', default=add_gripper)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_vacuum_gripper_1 = LaunchConfiguration('add_vacuum_gripper_1', default=add_vacuum_gripper)
    add_vacuum_gripper_2 = LaunchConfiguration('add_vacuum_gripper_2', default=add_vacuum_gripper)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    add_bio_gripper_1 = LaunchConfiguration('add_bio_gripper_1', default=add_bio_gripper)
    add_bio_gripper_2 = LaunchConfiguration('add_bio_gripper_2', default=add_bio_gripper)

    xarm_type_1 = '{}{}'.format(robot_type_1.perform(context), dof_1.perform(context) if robot_type_1.perform(context) in ('xarm', 'lite') else '')
    xarm_type_2 = '{}{}'.format(robot_type_2.perform(context), dof_2.perform(context) if robot_type_2.perform(context) in ('xarm', 'lite') else '')
    
    # ros2_controllers_path
    ros2_controllers_path = generate_dual_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}_controllers.yaml'.format(xarm_type_1)),
        os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}_controllers.yaml'.format(xarm_type_2)),
        prefix_1=prefix_1.perform(context), 
        prefix_2=prefix_2.perform(context), 
        add_gripper_1=add_gripper_1.perform(context) in ('True', 'true'),
        add_gripper_2=add_gripper_2.perform(context) in ('True', 'true'),
        add_bio_gripper_1=add_bio_gripper_1.perform(context) in ('True', 'true'),
        add_bio_gripper_2=add_bio_gripper_2.perform(context) in ('True', 'true'),
        robot_type_1=robot_type_1.perform(context), 
        robot_type_2=robot_type_2.perform(context), 
    )

    moveit_config = DualMoveItConfigsBuilder(
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
        add_gripper_1=add_gripper_1,
        add_gripper_2=add_gripper_2,
        add_vacuum_gripper_1=add_vacuum_gripper_1,
        add_vacuum_gripper_2=add_vacuum_gripper_2,
        add_bio_gripper_1=add_bio_gripper_1,
        add_bio_gripper_2=add_bio_gripper_2,
        ros2_control_plugin='uf_robot_hardware/UFRobotSystemHardware',
        ros2_control_params=ros2_controllers_path,
    ).to_moveit_configs()

    # Start the actual move_group node/action server
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()],
    )

    # Launch RViz
    rviz_config = PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'rviz', 'dual_moveit.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    static_tf_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', '{}link_base'.format(prefix_1.perform(context))],
    )
    static_tf_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', '{}link_base'.format(prefix_2.perform(context))],
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
        ],
        output='screen',
    )

    remappings = [
        ('follow_joint_trajectory', '{}{}_traj_controller/follow_joint_trajectory'.format(prefix_1.perform(context), xarm_type_1)),
        ('follow_joint_trajectory', '{}{}_traj_controller/follow_joint_trajectory'.format(prefix_2.perform(context), xarm_type_2)),            
    ]
    controllers = [
        '{}{}_traj_controller'.format(prefix_1.perform(context), xarm_type_1),
        '{}{}_traj_controller'.format(prefix_2.perform(context), xarm_type_2),
    ]
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'source_list': [
                '{}{}/joint_states'.format(prefix_1.perform(context), hw_ns.perform(context)),
                '{}{}/joint_states'.format(prefix_2.perform(context), hw_ns.perform(context))
            ], 
        }],
        remappings=remappings,
    )

    controller_nodes = []
    for controller in controllers:
        controller_nodes.append(Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                controller,
                '--controller-manager', '/controller_manager'
            ],
        ))
    
    return [
        robot_state_publisher,
        joint_state_publisher_node,
        move_group_node,
        static_tf_1,
        static_tf_2,
        ros2_control_node,
        rviz_node,
    ] + controller_nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])