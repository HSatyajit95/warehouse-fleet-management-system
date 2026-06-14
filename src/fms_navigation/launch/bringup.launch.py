"""
bringup.launch.py — Full system bringup: Gazebo + N robots + Nav2 per robot + RViz.

Phase 2 addition: launches fms_robot_agent nodes per robot after Nav2 is active.

Localization: static map→odom TF published by odom_to_tf (no AMCL needed in simulation).
Middleware:   requires CycloneDDS — set export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
              (added to ~/.bashrc automatically)

Usage:
  ros2 launch fms_navigation bringup.launch.py                        # 4 robots + agents
  ros2 launch fms_navigation bringup.launch.py num_robots:=1         # single robot
  ros2 launch fms_navigation bringup.launch.py launch_agents:=false  # Nav2 only (Phase 1)
  ros2 launch fms_navigation bringup.launch.py use_rviz:=false headless:=true
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    OpaqueFunction, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration



# Charger dock position for each robot [x, y, yaw_deg] in map frame.
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
    num_robots    = int(LaunchConfiguration("num_robots").perform(context))
    use_slam      = LaunchConfiguration("use_slam").perform(context)
    use_rviz      = LaunchConfiguration("use_rviz").perform(context).lower() in ("true", "1", "yes")
    headless      = LaunchConfiguration("headless").perform(context)
    launch_agents = LaunchConfiguration("launch_agents").perform(context).lower() in ("true", "1", "yes")

    fms_gazebo_share = FindPackageShare("fms_gazebo").find("fms_gazebo")
    fms_nav_share    = FindPackageShare("fms_navigation").find("fms_navigation")

    # Auto-resolve the static warehouse map path for AMCL mode.
    warehouse_map = os.path.join(fms_nav_share, "maps", "warehouse.yaml")

    actions = []

    # 1 ── RViz first — starts immediately so the user sees the UI while Gazebo loads.
    if use_rviz:
        rviz_config = os.path.join(fms_nav_share, "rviz", "fleet.rviz")
        actions.append(Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
            parameters=[{"use_sim_time": True}],
            output="screen",
        ))

    # 2 ── Simulation (Gazebo + robot spawning + ROS↔GZ bridges)
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fms_gazebo_share, "launch", "sim.launch.py")
        ),
        launch_arguments={
            "num_robots": str(num_robots),
            "headless":   headless,
        }.items(),
    ))

    # 3 ── Global map_server — publishes /map for RViz visualization.
    #      Delayed 5 s so Gazebo has time to start before lifecycle handshake.
    #      All per-robot AMCL instances have their own map_server; this one
    #      is solely for the single Map display in fleet.rviz.
    #      warehouse_markers publishes charge dock / pick station / drop zone
    #      locations as MarkerArray on /warehouse_locations (transient_local).
    actions.append(TimerAction(
        period=5.0,
        actions=[
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="global_map_server",
                parameters=[{
                    "use_sim_time":  True,
                    "yaml_filename": warehouse_map,
                }],
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map",
                parameters=[{
                    "use_sim_time": True,
                    "autostart":    True,
                    "node_names":   ["global_map_server"],
                }],
                output="screen",
            ),
            Node(
                package="fms_navigation",
                executable="warehouse_markers.py",
                name="warehouse_markers",
                output="screen",
            ),
        ],
    ))

    # 4 ── Nav2 per robot — STAGGERED startup to prevent DDS overload.
    #
    # Problem: starting all 4 Nav2 stacks simultaneously floods Fast-RTPS with
    # 32+ lifecycle node registrations at once.  DDS service *responses* for
    # lifecycle change_state calls then time out → map_server ends up inactive,
    # lifecycle_manager_navigation aborts, costmap activation fails because TF
    # isn't ready yet.
    #
    # Fix: start robot_1 at t=10 s, then one new robot every 15 s.
    # Each robot gets ~15 s of quiet time to:
    #   • configure all lifecycle nodes
    #   • receive first odom/TF message from odom_to_tf
    #   • activate costmaps (TF is always available by this point)
    #   • create lifecycle bonds without DDS congestion
    NAV2_FIRST_DELAY  = 10.0   # robot_1 starts 10 s after Gazebo launches
    NAV2_STAGGER_STEP = 15.0   # each subsequent robot delayed 15 s more

    # robot_agent nodes launch AFTER Nav2 is fully active for each robot.
    # Nav2 for robot_i is active at roughly: NAV2_FIRST_DELAY + (i-1)*NAV2_STAGGER_STEP + 10s
    # Agent starts 5 s after that, so Nav2 action servers are reachable.
    AGENT_EXTRA_DELAY = 15.0   # additional seconds after Nav2 start before launching agent

    nav2_launch = os.path.join(fms_nav_share, "launch", "nav2_robot.launch.py")

    agent_launch = None
    if launch_agents:
        fms_agent_share = FindPackageShare("fms_robot_agent").find("fms_robot_agent")
        agent_launch    = os.path.join(fms_agent_share, "launch", "robot_agent.launch.py")

    for i in range(1, num_robots + 1):
        robot_name = f"robot_{i}"
        nav2_delay  = NAV2_FIRST_DELAY + (i - 1) * NAV2_STAGGER_STEP
        agent_delay = nav2_delay + AGENT_EXTRA_DELAY

        # Nav2 launch
        actions.append(TimerAction(
            period=nav2_delay,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    "robot_name": robot_name,
                    "use_slam":   use_slam,
                    "map":        warehouse_map,
                }.items(),
            )],
        ))

        # Robot agent launch (Phase 2) — delayed until Nav2 is active
        if launch_agents:
            actions.append(TimerAction(
                period=agent_delay,
                actions=[IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(agent_launch),
                    launch_arguments={
                        "robot_name":  robot_name,
                        "use_sim_time": "true",
                    }.items(),
                )],
            ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("num_robots", default_value="4",
                              description="Number of AMRs to spawn (1–8)"),
        DeclareLaunchArgument("use_slam", default_value="false",
                              description="true=SLAM Toolbox (single-robot mapping); false=static map"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Launch RViz visualisation"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="Run Gazebo without GUI"),
        DeclareLaunchArgument("launch_agents", default_value="true",
                              description="Launch fms_robot_agent nodes (Phase 2). "
                                          "Set false to run Phase 1 Nav2 only."),
        OpaqueFunction(function=launch_setup),
    ])
