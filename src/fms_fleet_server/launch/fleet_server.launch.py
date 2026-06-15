"""
fleet_server.launch.py — Launch the fms_fleet_server node.

Usage:
  ros2 launch fms_fleet_server fleet_server.launch.py num_robots:=4
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    num_robots = int(LaunchConfiguration("num_robots").perform(context))

    share = FindPackageShare("fms_fleet_server").find("fms_fleet_server")
    params_file = os.path.join(share, "config", "fleet_server_params.yaml")

    node = Node(
        package="fms_fleet_server",
        executable="fleet_server_node",
        name="fleet_server",
        parameters=[
            params_file,
            {"num_robots": num_robots},
        ],
        output="screen",
        emulate_tty=True,
    )
    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("num_robots", default_value="4"),
        OpaqueFunction(function=launch_setup),
    ])
