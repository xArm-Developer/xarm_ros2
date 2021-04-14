import os
import sys
# from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_ip',
            description='IP address by which the robot can be reached.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'report_type',
            default_value='normal',
            description='Tcp report type, default is normal, normal/rich/dev optional.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'dof',
            default_value='7',
            description='Degree of freedom of manipulator, defalut is 7.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'hw_ns',
            default_value='xarm',
            description='The namespace of xarm_driver, default is xarm.',
        )
    )

    # Initialize Arguments
    robot_ip = LaunchConfiguration('robot_ip')
    report_type = LaunchConfiguration('report_type', default='normal')
    dof = LaunchConfiguration('dof', default=7)
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    
    xarm_params = PathJoinSubstitution([FindPackageShare('xarm_api'), 'config', 'xarm_params.yaml'])
    # xarm_params = os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_params.yaml')
    
    xarm_driver_node = Node(
        namespace=hw_ns,
        package='xarm_api',
        name='xarm_driver',
        executable='xarm_driver_node',
        output='screen',
        parameters=[
            xarm_params,
            {'xarm_robot_ip': robot_ip},
            {'xarm_report_type': report_type},
            {'DOF': dof},
        ]
    )
    # ld = LaunchDescription()
    # ld.add_action(xarm_driver_node)
    return LaunchDescription(declared_arguments + [xarm_driver_node])
