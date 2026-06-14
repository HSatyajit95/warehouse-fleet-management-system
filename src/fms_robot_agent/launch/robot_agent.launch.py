"""
robot_agent.launch.py — Launch one fms_robot_agent node.

Usage:
  ros2 launch fms_robot_agent robot_agent.launch.py robot_name:=robot_1
  ros2 launch fms_robot_agent robot_agent.launch.py robot_name:=robot_2 initial_battery_soc:=50.0
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Charger dock positions per robot [x, y, yaw_deg] in map frame.
# Positions match actual Gazebo SDF charge_dock model locations.
# yaw=0 → robot faces east (into warehouse), ready to depart.
CHARGER_POSES = {
    "robot_1": [-22.0, -15.0, 0.0],  # charge_dock_4
    "robot_2": [-22.0, -10.0, 0.0],  # charge_dock_3
    "robot_3": [-22.0,  10.0, 0.0],  # charge_dock_2
    "robot_4": [-22.0,  15.0, 0.0],  # charge_dock_1
    "robot_5": [-22.0,  -5.0, 0.0],
    "robot_6": [-22.0,   0.0, 0.0],
    "robot_7": [-22.0,   5.0, 0.0],
    "robot_8": [-22.0,  18.0, 0.0],
}


def launch_setup(context, *args, **kwargs):
    robot_name       = LaunchConfiguration("robot_name").perform(context)
    use_sim_time_str = LaunchConfiguration("use_sim_time").perform(context)
    init_soc_str     = LaunchConfiguration("initial_battery_soc").perform(context)

    use_sim_time = use_sim_time_str.lower() in ("true", "1", "yes")
    init_soc     = float(init_soc_str)
    charger_pose = CHARGER_POSES.get(robot_name, [-20.0, -17.0, 180.0])

    share = FindPackageShare("fms_robot_agent").find("fms_robot_agent")
    params_file = os.path.join(share, "config", "robot_agent_params.yaml")

    node = Node(
        package="fms_robot_agent",
        executable="robot_agent_node",
        name="robot_agent",
        namespace=robot_name,
        parameters=[
            params_file,
            {
                "robot_name":          robot_name,
                "use_sim_time":        use_sim_time,
                "initial_battery_soc": init_soc,
                "charger_pose":        charger_pose,
            },
        ],
        output="screen",
        emulate_tty=True,
    )
    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name",          default_value="robot_1"),
        DeclareLaunchArgument("initial_battery_soc", default_value="100.0"),
        DeclareLaunchArgument("use_sim_time",        default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
