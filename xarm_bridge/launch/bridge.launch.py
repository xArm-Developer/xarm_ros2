"""
Bridge launch file.

Starts:
  1. FastAPI bridge node (skill RPC at :8000)
  2. rosbridge_server (telemetry WebSocket at :9090)
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bridge_dir = get_package_share_directory("xarm_bridge")

    bridge_params = os.path.join(bridge_dir, "config", "bridge_params.yaml")
    rosbridge_params = os.path.join(bridge_dir, "config", "rosbridge_allowlist.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("bridge_port", default_value="8000"),
        DeclareLaunchArgument("rosbridge_port", default_value="9090"),

        Node(
            package="xarm_bridge",
            executable="bridge_node",
            name="bridge_node",
            parameters=[
                bridge_params,
                {"port": LaunchConfiguration("bridge_port")},
            ],
            output="screen",
        ),

        Node(
            package="rosbridge_server",
            executable="rosbridge_websocket",
            name="rosbridge_websocket",
            parameters=[
                rosbridge_params,
                {"port": LaunchConfiguration("rosbridge_port")},
            ],
            output="screen",
        ),
    ])
