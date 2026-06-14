# Phase 1 — FMS Navigation Test Commands
# 4 Robots Navigating Independently to Waypoints
#
# SETUP: Two terminals required.
#   Terminal A — bringup (launch once, keep running)
#   Terminal B — test commands (run while Terminal A is running)
#
# MAP COORDINATE REFERENCE
#   Map origin = world (-25, -20)
#   map_x = world_x + 25     map_y = world_y + 20
#
#   Robot spawn positions (map frame):
#     robot_1: (20.0,  3.0)     robot_2: (20.0,  6.0)
#     robot_3: (20.0,  9.0)     robot_4: (20.0, 12.0)
#
#   Useful landmarks (map frame):
#     Pick stations  : x ≈ 41        Charge docks: x ≈ 3
#     Open corridor  : x = 34–41     Shelf area  : x = 21–32
# ============================================================

# ──────────────────────────────────────────────────────────
# TERMINAL A — BRINGUP (run once, leave running)
# ──────────────────────────────────────────────────────────

# Step 1: Build and source
cd ~/fms_ws
colcon build
source /opt/ros/humble/setup.bash && source ~/fms_ws/install/setup.bash

# Expected build output:
# Starting >>> fms_gazebo
# Starting >>> fms_msgs
# Finished <<< fms_gazebo [0.3s]
# Finished <<< fms_navigation [0.3s]
# Finished <<< fms_msgs [0.5s]
# Summary: 3 packages finished [0.7s]

# Actual output:
# ____________________________________________________________


# Step 2: Launch full system
ros2 launch fms_navigation bringup.launch.py num_robots:=4

# Expected startup sequence:
# [INFO] RViz opens first
# [INFO] Gazebo loads warehouse world
# [INFO] 4 robots spawned (spawn_robot_1 ... spawn_robot_4: OK)
# [INFO] odom_to_tf_robot_N: /robot_N/odom → TF robot_N/odom→robot_N/base_footprint | static map→robot_N/odom @ (20.00,N.00,0.00)
# [INFO] parameter_bridge_robot_N: Creating ROS->GZ / GZ->ROS bridges
# [INFO] (after 5s) global_map_server: Activating  →  Managed nodes are active
# [INFO] (after 8s) map_server robot_N: Loading yaml file: .../warehouse.yaml  →  Read map 1000 X 800 @ 0.05 m/cell
# [INFO] map_server robot_N: Activating  →  Managed nodes are active
# [INFO] controller_server robot_N: Created controller : FollowPath of type dwb_core::DWBLocalPlanner
# [INFO] lifecycle_manager_navigation robot_N: Managed nodes are active

# Actual startup notes:
# ____________________________________________________________
# ____________________________________________________________


# ──────────────────────────────────────────────────────────
# TERMINAL B — VERIFICATION (new terminal, sourced)
# ──────────────────────────────────────────────────────────

source /opt/ros/humble/setup.bash && source ~/fms_ws/install/setup.bash

# ══════════════════════════════════════════════════════════
# CHECK 1: Nav2 lifecycle states (all should be "active [3]")
# ══════════════════════════════════════════════════════════

for robot in robot_1 robot_2 robot_3 robot_4; do
  echo "======= $robot ======="
  for node in map_server controller_server planner_server \
              behavior_server bt_navigator waypoint_follower velocity_smoother; do
    printf "  %-22s: " "$node"
    ros2 lifecycle get /$robot/$node 2>/dev/null || echo "NOT FOUND"
  done
done

# Expected output:
# ======= robot_1 =======
#   map_server            : active [3]
#   controller_server     : active [3]
#   planner_server        : active [3]
#   behavior_server       : active [3]
#   bt_navigator          : active [3]
#   waypoint_follower     : active [3]
#   velocity_smoother     : active [3]
# ======= robot_2 =======
#   (same as robot_1)
# ======= robot_3 =======
#   (same as robot_1)
# ======= robot_4 =======
#   (same as robot_1)

# Actual output:
# ============================================================
# _______ robot_1 _______
#
# _______ robot_2 _______
#
# _______ robot_3 _______
#
# _______ robot_4 _______
#
# ============================================================


# ══════════════════════════════════════════════════════════
# CHECK 2: Nav2 action servers available
# ══════════════════════════════════════════════════════════

ros2 action list | grep -E "navigate|waypoints|follow"

# Expected output (12 action servers total):
# /robot_1/follow_waypoints
# /robot_1/navigate_through_poses
# /robot_1/navigate_to_pose
# /robot_2/follow_waypoints
# /robot_2/navigate_through_poses
# /robot_2/navigate_to_pose
# /robot_3/follow_waypoints
# /robot_3/navigate_through_poses
# /robot_3/navigate_to_pose
# /robot_4/follow_waypoints
# /robot_4/navigate_through_poses
# /robot_4/navigate_to_pose

# Actual output:
# ____________________________________________________________
# ____________________________________________________________


# ══════════════════════════════════════════════════════════
# CHECK 3: TF chain intact (map → robot_N/odom → base_footprint)
# ══════════════════════════════════════════════════════════

# Static map→odom transforms (from odom_to_tf nodes)
ros2 topic echo /tf_static --once

# Expected: 4 transforms: map→robot_N/odom with positions (20,3), (20,6), (20,9), (20,12)

# Dynamic odom→base_footprint transforms (from Gazebo odometry)
ros2 topic echo /tf --field transforms[0].header.frame_id --once

# Expected: robot_N/odom  (showing the parent frame)

# Actual tf_static output:
# ____________________________________________________________
# ____________________________________________________________


# ══════════════════════════════════════════════════════════
# CHECK 4: Sensor data flowing
# ══════════════════════════════════════════════════════════

# Scan frequency (expect ~10 Hz for each robot)
ros2 topic hz /robot_1/scan --window 5 &
ros2 topic hz /robot_2/scan --window 5 &
wait

# Odometry check
ros2 topic echo /robot_1/odom --field pose.pose.position --once

# Expected: position x≈0.0, y≈0.0, z≈0.0  (robot hasn't moved yet)

# Actual scan hz:
# ____________________________________________________________
# Actual odom position:
# ____________________________________________________________


# ══════════════════════════════════════════════════════════
# GOAL SENDING — Single pose navigation
# ══════════════════════════════════════════════════════════

# Send robot_1 toward pick station area  (map: x=38, y=3)
ros2 action send_goal /robot_1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 38.0, y: 3.0, z: 0.0}, orientation: {w: 1.0}}}}"

# Expected output (takes 15-60 seconds depending on path):
# Waiting for an action server to become available...
# Sending goal:
#    pose:
#      header:
#        frame_id: map
#      pose:
#        position:
#          x: 38.0  y: 3.0  z: 0.0
#        orientation:
#          w: 1.0
# Goal accepted with ID: xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
# Result:
#   result: {}
# Goal finished with status: SUCCEEDED

# Actual output for robot_1:
# ____________________________________________________________
# ____________________________________________________________


# Send robot_2 (map: x=38, y=8)
ros2 action send_goal /robot_2/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 38.0, y: 8.0, z: 0.0}, orientation: {w: 1.0}}}}"

# Actual output for robot_2:
# ____________________________________________________________
# ____________________________________________________________


# Send robot_3 (map: x=5, y=9) — toward charge dock side
ros2 action send_goal /robot_3/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 9.0, z: 0.0}, orientation: {w: 1.0}}}}"

# Actual output for robot_3:
# ____________________________________________________________
# ____________________________________________________________


# Send robot_4 (map: x=38, y=15)
ros2 action send_goal /robot_4/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 38.0, y: 15.0, z: 0.0}, orientation: {w: 1.0}}}}"

# Actual output for robot_4:
# ____________________________________________________________
# ____________________________________________________________


# ══════════════════════════════════════════════════════════
# GOAL SENDING — Multiple waypoints (waypoint follower)
# ══════════════════════════════════════════════════════════
# robot_1: spawn(20,3) → pick station(38,3) → charge dock(5,3)

ros2 action send_goal /robot_1/follow_waypoints nav2_msgs/action/FollowWaypoints \
  "{poses: [
    {header: {frame_id: map}, pose: {position: {x: 38.0, y: 3.0, z: 0.0}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 5.0,  y: 3.0, z: 0.0}, orientation: {w: 1.0}}}
  ]}"

# Expected output:
# Goal accepted with ID: xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
# Feedback:
#   current_waypoint: 0
# Feedback:
#   current_waypoint: 1
# Result:
#   missed_waypoints: []
# Goal finished with status: SUCCEEDED

# Actual output for robot_1 waypoints:
# ____________________________________________________________
# ____________________________________________________________


# ══════════════════════════════════════════════════════════
# MONITORING — Watch robots navigate in real-time
# ══════════════════════════════════════════════════════════

# Watch robot_1 position while navigating
ros2 topic echo /robot_1/odom --field pose.pose.position

# Expected: x,y values changing as robot moves toward goal
# Actual:
# ____________________________________________________________

# Watch robot_1 velocity commands
ros2 topic echo /robot_1/cmd_vel

# Expected: linear.x ~0.5 m/s, angular.z varying as robot turns
# Actual:
# ____________________________________________________________

# Watch planned path length change
ros2 topic echo /robot_1/plan --field poses --once | grep -c "position"

# Expected: number of waypoints in the path (typically 50-300)
# Actual:
# ____________________________________________________________


# ══════════════════════════════════════════════════════════
# CANCEL NAVIGATION
# ══════════════════════════════════════════════════════════

# Cancel robot_1 goal (press Ctrl+C in the goal terminal, or send cancel):
ros2 action send_goal /robot_1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 38.0, y: 3.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  --cancel-after-secs 5

# ══════════════════════════════════════════════════════════
# SEND ALL 4 ROBOTS TO GOALS SIMULTANEOUSLY
# (run all 4 in background, then wait)
# ══════════════════════════════════════════════════════════

ros2 action send_goal /robot_1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 38.0, y: 3.0,  z: 0.0}, orientation: {w: 1.0}}}}" &

ros2 action send_goal /robot_2/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 38.0, y: 8.0,  z: 0.0}, orientation: {w: 1.0}}}}" &

ros2 action send_goal /robot_3/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 5.0,  y: 9.0,  z: 0.0}, orientation: {w: 1.0}}}}" &

ros2 action send_goal /robot_4/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 38.0, y: 15.0, z: 0.0}, orientation: {w: 1.0}}}}" &

wait
echo "All 4 robots reached their goals (or failed)"

# Expected: All 4 goals succeed (SUCCEEDED status) within ~60-120 seconds
# Actual results:
# robot_1: ___________________________________________________
# robot_2: ___________________________________________________
# robot_3: ___________________________________________________
# robot_4: ___________________________________________________
# Total time: _____________


# ══════════════════════════════════════════════════════════
# RVIZ — Visual navigation (alternative to CLI)
# ══════════════════════════════════════════════════════════
# 1. In running RViz: confirm Fixed Frame = "map"
# 2. Select "Nav2 Goal" tool from the toolbar (arrow icon)
# 3. Click and drag on the map to set goal pose for robot_1
#    (RViz publishes to /goal_pose → bt_navigator picks it up)
# 4. Watch the green path appear and robot move in Gazebo

# Note: RViz "Nav2 Goal" targets the DEFAULT robot (robot_1).
#       Use CLI commands above to target specific robots.

# ══════════════════════════════════════════════════════════
# PHASE 1 MILESTONE VERIFICATION CHECKLIST
# ══════════════════════════════════════════════════════════
# □ 1. Gazebo warehouse world loads correctly
# □ 2. All 4 robots spawn at correct positions in Gazebo
# □ 3. All 4 robots visible in RViz with correct models
# □ 4. Map displayed in RViz (Fixed Frame = map)
# □ 5. Nav2 lifecycle: all 7 nodes ACTIVE for all 4 robots
# □ 6. Nav2 action servers: /robot_N/navigate_to_pose available
# □ 7. robot_1 navigates to single goal pose  ← KEY TEST
# □ 8. robot_2 navigates to single goal pose
# □ 9. robot_3 navigates to single goal pose
# □ 10. robot_4 navigates to single goal pose
# □ 11. All 4 robots navigate simultaneously (independent)
# □ 12. Waypoint follower: robot navigates through 2+ waypoints
#
# Phase 1 COMPLETE when items 1-11 are checked ✓
