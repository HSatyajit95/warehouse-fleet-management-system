"""
nav2_robot.launch.py — Nav2 bringup for one namespaced robot.

Root cause of all previous parameter failures
---------------------------------------------
Costmap2DROS constructor signature (nav2-costmap-2d 1.1.20):
  Costmap2DROS(name, parent_namespace, local_namespace)
  — NO parameter_overrides argument.

The costmap is a separate ROS2 node running inside the controller_server
process.  It reads its parameters exclusively from --params-file entries
that match its FULLY-QUALIFIED node name:
  /robot_N/local_costmap/local_costmap   (for local costmap)
  /robot_N/global_costmap/global_costmap (for global costmap)

Short YAML keys like "local_costmap.local_costmap" only match nodes in the
ROOT namespace (/local_costmap/local_costmap), NOT /robot_1/... variants.
Dict overrides passed to controller_server reach ONLY the controller_server
node (not the costmap sub-node) because the costmap has its own parameter
server.

Solution: generate one YAML per robot that uses ABSOLUTE fully-qualified
node paths (/robot_N/...) as keys.  All nodes in the controller_server
process (including Costmap2DROS) load this same file and match by their
own full name.
"""

import os
import yaml
import tempfile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_TF = [("tf", "/tf"), ("tf_static", "/tf_static")]


def _make_robot_params(base_yaml: str, robot_name: str, map_yaml: str) -> str:
    """
    Generate a per-robot YAML parameter file where every node entry uses its
    ABSOLUTE fully-qualified name (/robot_N/...) so rcl_yaml_param_parser
    matches it exactly regardless of the running namespace.
    """
    with open(base_yaml) as f:
        base = yaml.safe_load(f)

    ns     = robot_name               # "robot_1"
    bf     = f"{robot_name}/base_footprint"
    odom_f = f"{robot_name}/odom"
    scan_t = f"/{robot_name}/scan"

    def _sect(section):
        """Return a deep copy of a base YAML section's ros__parameters dict."""
        s = base.get(section, {})
        if isinstance(s, dict):
            rp = s.get('ros__parameters', s)  # handle both nested and flat
            return dict(rp)
        return {}

    out = {}

    # ── /robot_N/map_server ──────────────────────────────────────────────────
    ms = _sect('map_server')
    ms['use_sim_time']  = True
    ms['yaml_filename'] = map_yaml
    out[f'/{ns}/map_server'] = {'ros__parameters': ms}

    # ── /robot_N/controller_server ───────────────────────────────────────────
    cs = _sect('controller_server')
    cs['use_sim_time'] = True
    # Inject FollowPath plugin (YAML key won't be matched for namespaced nodes)
    cs.setdefault('FollowPath', {})['plugin'] = (
        'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController')
    out[f'/{ns}/controller_server'] = {'ros__parameters': cs}

    # ── /robot_N/local_costmap/local_costmap ─────────────────────────────────
    # This is the EXACT fully-qualified name of the Costmap2DROS node created
    # inside controller_server with:
    #   Costmap2DROS("local_costmap", get_namespace()="/robot_N", "local_costmap")
    # → LifecycleNode name="local_costmap", namespace="/robot_N/local_costmap"
    # → full path = /robot_N/local_costmap/local_costmap
    lc = dict(base.get('local_costmap', {})
                  .get('local_costmap', {})
                  .get('ros__parameters', {}))
    lc['use_sim_time']   = True
    lc['global_frame']   = odom_f     # robot_N/odom
    lc['robot_base_frame'] = bf       # robot_N/base_footprint
    # Ensure absolute scan topic so the plugin finds the correct sensor
    lc.setdefault('obstacle_layer', {})
    if isinstance(lc.get('obstacle_layer'), dict):
        lc['obstacle_layer'].setdefault('scan', {})['topic'] = scan_t
    out[f'/{ns}/local_costmap/local_costmap'] = {'ros__parameters': lc}

    # ── /robot_N/planner_server ──────────────────────────────────────────────
    ps = _sect('planner_server')
    ps['use_sim_time'] = True
    out[f'/{ns}/planner_server'] = {'ros__parameters': ps}

    # ── /robot_N/global_costmap/global_costmap ───────────────────────────────
    gc = dict(base.get('global_costmap', {})
                  .get('global_costmap', {})
                  .get('ros__parameters', {}))
    gc['use_sim_time']     = True
    gc['global_frame']     = 'map'
    gc['robot_base_frame'] = bf
    if isinstance(gc.get('obstacle_layer'), dict):
        gc['obstacle_layer'].setdefault('scan', {})['topic'] = scan_t
    out[f'/{ns}/global_costmap/global_costmap'] = {'ros__parameters': gc}

    # ── /robot_N/behavior_server ─────────────────────────────────────────────
    bs = _sect('behavior_server')
    bs['use_sim_time']     = True
    bs['global_frame']     = odom_f
    bs['robot_base_frame'] = bf
    out[f'/{ns}/behavior_server'] = {'ros__parameters': bs}

    # ── /robot_N/bt_navigator ────────────────────────────────────────────────
    bt = _sect('bt_navigator')
    bt['use_sim_time']     = True
    bt['global_frame']     = 'map'
    bt['robot_base_frame'] = bf
    out[f'/{ns}/bt_navigator'] = {'ros__parameters': bt}

    # ── /robot_N/waypoint_follower ───────────────────────────────────────────
    wf = _sect('waypoint_follower')
    wf['use_sim_time'] = True
    out[f'/{ns}/waypoint_follower'] = {'ros__parameters': wf}

    # ── /robot_N/velocity_smoother ───────────────────────────────────────────
    vs = _sect('velocity_smoother')
    vs['use_sim_time'] = True
    out[f'/{ns}/velocity_smoother'] = {'ros__parameters': vs}

    # ── Write with yaml.safe_dump ─────────────────────────────────────────────
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml',
        prefix=f'nav2_{robot_name}_', delete=False)
    yaml.safe_dump(out, tmp, default_flow_style=False, allow_unicode=True)
    tmp.close()
    return tmp.name


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    map_yaml   = LaunchConfiguration("map").perform(context)

    fms_nav_share = FindPackageShare("fms_navigation").find("fms_navigation")
    base_params   = os.path.join(fms_nav_share, "config", "nav2_params.yaml")

    if not map_yaml:
        map_yaml = os.path.join(fms_nav_share, "maps", "warehouse.yaml")

    robot_params = _make_robot_params(base_params, robot_name, map_yaml)

    ns  = robot_name
    bf  = f"{robot_name}/base_footprint"
    actions = []

    # ── Localization ──────────────────────────────────────────────────────────
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        namespace=ns,
        parameters=[robot_params],
        output="screen",
    )
    lm_loc = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        namespace=ns,
        parameters=[{
            "use_sim_time":  True,
            "autostart":     True,
            "node_names":    ["map_server"],
            "bond_timeout":  30.0,   # default 4s is too short on a busy system
        }],
        output="screen",
    )
    actions.extend([map_server, lm_loc])

    # ── Navigation stack — delayed 5 s after localization ─────────────────────
    # The localization lifecycle_manager and navigation lifecycle_manager both
    # make DDS service calls.  When they start simultaneously, the
    # change_state service responses timeout (DDS queue overflow) and
    # map_server stays inactive → global costmap never receives the map →
    # "Robot is out of bounds" warning.
    # Fix: start the navigation lifecycle 5 s after localization so
    # map_server is ACTIVE before the global costmap needs its map.
    # cmd_vel pipeline:
    #   controller → /{robot}/cmd_vel_smoothed (raw)
    #   velocity_smoother → /{robot}/cmd_vel    (smoothed, bridge input)
    # Before this fix the remappings were swapped: smoother had no input and
    # published zero velocity every velocity_timeout seconds, which competed
    # with the controller on the bridge topic and caused stutter + wall strikes.
    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        namespace=ns,
        parameters=[robot_params],
        remappings=[("cmd_vel", f"/{robot_name}/cmd_vel_smoothed"),
                    ("odom",    f"/{robot_name}/odom"),
                    ("scan",    f"/{robot_name}/scan")] + _TF,
        output="screen",
    )

    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        namespace=ns,
        parameters=[robot_params],
        remappings=_TF,
        output="screen",
    )

    behavior = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        namespace=ns,
        parameters=[robot_params],
        remappings=[("cmd_vel", f"/{robot_name}/cmd_vel")] + _TF,
        output="screen",
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        namespace=ns,
        parameters=[robot_params],
        remappings=[("odom", f"/{robot_name}/odom")] + _TF,
        output="screen",
    )

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        namespace=ns,
        parameters=[robot_params],
        output="screen",
    )

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        namespace=ns,
        parameters=[robot_params],
        remappings=[
            ("cmd_vel",          f"/{robot_name}/cmd_vel_smoothed"),
            ("cmd_vel_smoothed", f"/{robot_name}/cmd_vel"),
            ("odom",             f"/{robot_name}/odom"),
        ],
        output="screen",
    )

    lm_nav = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        namespace=ns,
        parameters=[{"use_sim_time": True, "autostart": True,
                     "bond_timeout": 30.0,
                     "node_names": [
                         "controller_server", "planner_server",
                         "behavior_server", "bt_navigator",
                         "waypoint_follower", "velocity_smoother",
                     ]}],
        output="screen",
    )

    # Start ALL navigation nodes + their lifecycle_manager 5 s after
    # localization, so map_server finishes activating first.
    actions.append(TimerAction(
        period=5.0,
        actions=[controller, planner, behavior, bt_navigator,
                 waypoint_follower, velocity_smoother, lm_nav],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="robot_1"),
        DeclareLaunchArgument("use_slam",   default_value="false"),
        DeclareLaunchArgument("map",        default_value=""),
        OpaqueFunction(function=launch_setup),
    ])
