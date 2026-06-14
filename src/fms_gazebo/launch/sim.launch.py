"""
sim.launch.py — Launch Gazebo warehouse + spawn N robots.

Usage:
  ros2 launch fms_gazebo sim.launch.py
  ros2 launch fms_gazebo sim.launch.py num_robots:=2
  ros2 launch fms_gazebo sim.launch.py num_robots:=4 headless:=true

Design notes
------------
Server/GUI split:
  gz sim without -s forks both server and GUI from the same process that
  already has OGRE2 loaded.  On Wayland/XWayland the fork is not safe and
  the server child crashes silently, so we always run the server headlessly
  (-s) and launch the GUI client (-g) as a separate optional process.

GZ_IP=127.0.0.1:
  Gazebo Transport uses UDP multicast for service discovery between processes.
  On WiFi interfaces (wlp*) multicast packets do NOT loop back to other
  processes on the same machine, so the server and GUI (and spawn_entity)
  can never discover each other.  Forcing all processes to use the loopback
  interface fixes discovery.  SetEnvironmentVariable propagates to every
  child process in this launch (including spawn_entity).
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    OpaqueFunction, SetEnvironmentVariable, TimerAction,
)
from launch.actions import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


# Initial poses for up to 8 robots (x, y, yaw).
# Robots 1-4 spawn at the actual Gazebo charge dock positions so the starting
# pose matches the charger_pose used by robot_agent (no initial navigation needed).
ROBOT_POSES = [
    (-22.0, -15.0, 0.0),  # robot_1 → charge_dock_4
    (-22.0, -10.0, 0.0),  # robot_2 → charge_dock_3
    (-22.0,  10.0, 0.0),  # robot_3 → charge_dock_2
    (-22.0,  15.0, 0.0),  # robot_4 → charge_dock_1
    (-22.0,  -5.0, 0.0),  # robot_5 (no physical dock)
    (-22.0,   0.0, 0.0),  # robot_6
    (-22.0,   5.0, 0.0),  # robot_7
    (-22.0,  18.0, 0.0),  # robot_8
]


def launch_setup(context, *args, **kwargs):
    num_robots = int(LaunchConfiguration("num_robots").perform(context))
    headless = LaunchConfiguration("headless").perform(context).lower() in ("true", "1", "yes")

    fms_gazebo_share = FindPackageShare("fms_gazebo").find("fms_gazebo")
    world_file = os.path.join(fms_gazebo_share, "worlds", "warehouse.sdf")

    # Plugin paths forwarded from the environment (mirrors ros_gz_sim behaviour).
    gz_env = {
        'GZ_IP': '127.0.0.1',
        'GZ_SIM_SYSTEM_PLUGIN_PATH': ':'.join(filter(None, [
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
            os.environ.get('LD_LIBRARY_PATH', ''),
        ])),
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': ':'.join(filter(None, [
            os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ''),
            os.environ.get('LD_LIBRARY_PATH', ''),
        ])),
    }

    actions = []

    # Gazebo server — headless, required.  Simulation ends if server exits.
    gz_server = ExecuteProcess(
        cmd=['ruby $(which gz) sim', f'-s -r {world_file}', '--force-version', '8'],
        additional_env=gz_env,
        output='screen',
        shell=True,
        on_exit=Shutdown(),
    )
    actions.append(gz_server)

    # Gazebo GUI client — optional (closing the window keeps simulation alive).
    # Delayed 2 s so the server is ready before the GUI tries to connect.
    if not headless:
        gz_gui = ExecuteProcess(
            cmd=['ruby $(which gz) sim', '-g', '--force-version', '8'],
            additional_env=gz_env,
            output='screen',
            shell=True,
        )
        actions.append(TimerAction(period=2.0, actions=[gz_gui]))

    # Spawn robots
    spawn_launch = os.path.join(fms_gazebo_share, "launch", "spawn_robot.launch.py")
    for i in range(1, num_robots + 1):
        x, y, yaw = ROBOT_POSES[i - 1]
        robot_name = f"robot_{i}"
        spawn = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_launch),
            launch_arguments={
                "robot_name": robot_name,
                "x": str(x),
                "y": str(y),
                "yaw": str(yaw),
            }.items(),
        )
        actions.append(spawn)

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("num_robots", default_value="4",
                              description="Number of robots to spawn (1-8)"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="Run Gazebo without GUI (server only)"),
        # Force Gazebo Transport onto loopback so server↔GUI discovery works
        # on machines where WiFi multicast doesn't loop back locally.
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
        OpaqueFunction(function=launch_setup),
    ])
