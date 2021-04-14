import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    report_type = LaunchConfiguration('report_type', default='normal')
    dof = LaunchConfiguration('dof', default=7)
    namespace = LaunchConfiguration('hw_ns', default='xarm')
    
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('xarm_api'), 'config', 'xarm_params.yaml'
    )
    
    xarm_driver_node = Node(
        namespace=namespace,
        package='xarm_api',
        name='xarm_driver',
        executable='xarm_driver_node',
        output='screen',
        parameters=[
            config,
            {'xarm_robot_ip': robot_ip},
            {'xarm_report_type': report_type},
            {'DOF': dof},
        ]
    )
    ld.add_action(xarm_driver_node)
    return ld
