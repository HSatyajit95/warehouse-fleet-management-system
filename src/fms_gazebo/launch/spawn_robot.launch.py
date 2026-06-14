"""
spawn_robot.launch.py — Spawn a single namespaced robot into Gazebo + start ROS bridge.

Called by sim.launch.py per robot. Can also be used standalone:
  ros2 launch fms_gazebo spawn_robot.launch.py robot_name:=robot_1 x:=0 y:=0 yaw:=0
"""

import os
import subprocess
import sys
import yaml
import tempfile
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


GZ_ENV = {'GZ_IP': '127.0.0.1'}


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    x = LaunchConfiguration("x").perform(context)
    y = LaunchConfiguration("y").perform(context)
    yaw = LaunchConfiguration("yaw").perform(context)

    fms_gazebo_share = FindPackageShare("fms_gazebo").find("fms_gazebo")
    urdf_file = os.path.join(fms_gazebo_share, "urdf", "fms_robot.urdf.xacro")

    # Run xacro now (at launch time) so the URDF string is ready immediately.
    # Passing -file to ros_gz_sim create avoids the transient_local / volatile
    # QoS race where create subscribes AFTER robot_state_publisher has already
    # published robot_description and never receives the cached message.
    xacro_result = subprocess.run(
        ['xacro', urdf_file, f'robot_name:={robot_name}'],
        capture_output=True, text=True, check=True,
    )
    urdf_tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.urdf', prefix=f'robot_{robot_name}_', delete=False
    )
    urdf_tmp.write(xacro_result.stdout)
    urdf_tmp.close()
    urdf_tmp_path = urdf_tmp.name

    robot_description_str = xacro_result.stdout

    actions = []

    # robot_state_publisher — still needed for /tf and /joint_states
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=robot_name,
        name="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description_str,
            "use_sim_time": True,
            "frame_prefix": f"{robot_name}/",
        }],
        remappings=[
            ("joint_states", f"/{robot_name}/joint_states"),
            ("tf", "/tf"),
            ("tf_static", "/tf_static"),
        ],
        output="screen",
    )
    actions.append(rsp)

    # Spawn model into Gazebo using a pre-generated URDF file.
    # Using -file instead of -topic avoids the QoS timing race described above.
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name=f"spawn_{robot_name}",
        arguments=[
            "-name", robot_name,
            "-file", urdf_tmp_path,
            "-x", x, "-y", y, "-z", "0.05",
            "-Y", yaw,
        ],
        additional_env=GZ_ENV,
        output="screen",
    )
    actions.append(spawn)

    # ros_gz_bridge for this robot
    bridge_params = [
        # clock: Gazebo → ROS  (only robot_1 bridges clock to avoid duplicate publishers)
        *([{
            "ros_topic_name": "/clock",
            "gz_topic_name": "/clock",
            "ros_type_name": "rosgraph_msgs/msg/Clock",
            "gz_type_name": "gz.msgs.Clock",
            "direction": "GZ_TO_ROS",
        }] if robot_name == "robot_1" else []),
        # cmd_vel: ROS → Gazebo
        {
            "ros_topic_name": f"/{robot_name}/cmd_vel",
            "gz_topic_name": f"/{robot_name}/cmd_vel",
            "ros_type_name": "geometry_msgs/msg/Twist",
            "gz_type_name": "gz.msgs.Twist",
            "direction": "ROS_TO_GZ",
        },
        # odom: Gazebo → ROS
        {
            "ros_topic_name": f"/{robot_name}/odom",
            "gz_topic_name": f"/{robot_name}/odom",
            "ros_type_name": "nav_msgs/msg/Odometry",
            "gz_type_name": "gz.msgs.Odometry",
            "direction": "GZ_TO_ROS",
        },
        # scan: Gazebo → ROS
        {
            "ros_topic_name": f"/{robot_name}/scan",
            "gz_topic_name": f"/{robot_name}/scan",
            "ros_type_name": "sensor_msgs/msg/LaserScan",
            "gz_type_name": "gz.msgs.LaserScan",
            "direction": "GZ_TO_ROS",
        },
        # imu: Gazebo → ROS
        {
            "ros_topic_name": f"/{robot_name}/imu",
            "gz_topic_name": f"/{robot_name}/imu",
            "ros_type_name": "sensor_msgs/msg/Imu",
            "gz_type_name": "gz.msgs.IMU",
            "direction": "GZ_TO_ROS",
        },
        # joint_states: Gazebo → ROS
        {
            "ros_topic_name": f"/{robot_name}/joint_states",
            "gz_topic_name": f"/{robot_name}/joint_states",
            "ros_type_name": "sensor_msgs/msg/JointState",
            "gz_type_name": "gz.msgs.Model",
            "direction": "GZ_TO_ROS",
        },
        # NOTE: TF (odom→base_footprint) is NOT bridged here.
        # ros_gz_bridge publishes /tf with a QoS profile that tf2::Buffer
        # does not subscribe to (QoS mismatch → silent drop).
        # The odom_to_tf node below reads /robot_N/odom and re-publishes
        # using tf2_ros.TransformBroadcaster, which always uses the correct QoS.
    ]

    bridge_config_path = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", prefix=f"bridge_{robot_name}_", delete=False
    )
    yaml.dump(bridge_params, bridge_config_path)
    bridge_config_path.close()

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=f"bridge_{robot_name}",
        parameters=[{"config_file": bridge_config_path.name}],
        additional_env=GZ_ENV,
        output="screen",
    )
    actions.append(bridge)

    # Odom → TF relay + static map→odom publisher.
    #
    # The MAP TF frame must coincide with the Gazebo WORLD frame (both have
    # their origin at (0,0) = warehouse centre).  The warehouse.yaml origin
    # [-25,-20] tells nav2 where the lower-left corner of the map IMAGE is,
    # expressed in the MAP frame (= world frame).
    #
    # The static transform  map → robot_N/odom  describes where the robot's
    # odom origin is in the MAP frame.  At spawn time odom = (0,0,0), so the
    # odom origin IS the spawn position in world frame.
    # Therefore: map_x = spawn_world_x,  map_y = spawn_world_y.
    #
    # Previous (wrong): map_x = world_x - MAP_ORIGIN_X  → placed robot at
    # MAP (20,3) instead of (-5,-17); goals like (38,3) were then 13 m
    # outside the 50 m-wide map → worldToMap failed.
    map_x   = float(x)    # spawn world x  (= map frame x, same origin)
    map_y   = float(y)    # spawn world y
    map_yaw = float(yaw)

    odom_tf_script = os.path.join(fms_gazebo_share, "scripts", "odom_to_tf.py")
    odom_tf = ExecuteProcess(
        cmd=[
            sys.executable, odom_tf_script,
            "--ros-args",
            "--remap", f"__node:=odom_to_tf_{robot_name}",
            "-p", f"odom_topic:=/{robot_name}/odom",
            "-p", f"parent_frame:={robot_name}/odom",
            "-p", f"child_frame:={robot_name}/base_footprint",
            "-p", "use_sim_time:=true",
            "-p", f"map_x:={map_x}",
            "-p", f"map_y:={map_y}",
            "-p", f"map_yaw:={map_yaw}",
        ],
        output="screen",
    )
    actions.append(odom_tf)

    # Gazebo Harmonic collapses fixed joints during URDF→SDF conversion, so
    # lidar_link is merged into base_link and the GPU lidar sensor frame becomes
    # "{link}/{sensor_name}" = "base_link/lidar".  That name is absent from the
    # TF tree (robot_state_publisher publishes robot_N/lidar_link, not
    # robot_N/base_link/lidar), so the costmap silently drops every scan.
    # Publishing a static TF here adds the missing frame at the correct offset
    # (same as lidar_joint origin: z = base_height(0.2) + 0.03 = 0.23 m above
    # base_link).
    lidar_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"lidar_frame_{robot_name}",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0.23",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", f"{robot_name}/base_link",
            "--child-frame-id", f"{robot_name}/base_link/lidar",
        ],
        output="screen",
    )
    actions.append(lidar_frame)

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="robot_1"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        OpaqueFunction(function=launch_setup),
    ])
