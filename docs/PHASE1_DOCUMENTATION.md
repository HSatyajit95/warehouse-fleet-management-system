# Phase 1 Documentation — Multi-Robot Fleet Management System
## 4 Robots Navigating Independently to Waypoints in Gazebo Harmonic Warehouse Simulation

---

## Table of Contents

1. [Overview](#1-overview)
2. [System Architecture](#2-system-architecture)
3. [Software Stack](#3-software-stack)
4. [Workspace Structure](#4-workspace-structure)
5. [Package Descriptions](#5-package-descriptions)
   - 5.1 [fms_msgs](#51-fms_msgs)
   - 5.2 [fms_gazebo](#52-fms_gazebo)
   - 5.3 [fms_navigation](#53-fms_navigation)
6. [Robot Model (URDF/Xacro)](#6-robot-model-urdfxacro)
7. [Warehouse Simulation (Gazebo SDF)](#7-warehouse-simulation-gazebo-sdf)
8. [Navigation Map](#8-navigation-map)
9. [TF Tree Design](#9-tf-tree-design)
10. [Nav2 Stack Configuration](#10-nav2-stack-configuration)
11. [Launch System](#11-launch-system)
12. [Multi-Robot Goal Sender](#12-multi-robot-goal-sender)
13. [Critical Bugs Fixed](#13-critical-bugs-fixed)
14. [Validation Results](#14-validation-results)
15. [Operating Procedures](#15-operating-procedures)
16. [Known Limitations](#16-known-limitations)

---

## 1. Overview

Phase 1 establishes the simulation foundation for the Fleet Management System (FMS): a Gazebo Harmonic warehouse world with **4 autonomous mobile robots (AMRs)** each running a full Nav2 navigation stack, capable of independently planning and executing paths to arbitrary waypoints while avoiding obstacles.

**Phase 1 Milestone:** *4 robots navigating independently to waypoints in a Gazebo Harmonic warehouse simulation.*

**Status:** COMPLETE — All 4 robots SUCCEEDED (elapsed: 215.1 s, pick-station goal set).

---

## 2. System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        GAZEBO HARMONIC                               │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐           │
│  │ robot_1  │  │ robot_2  │  │ robot_3  │  │ robot_4  │           │
│  │ DiffDrive│  │ DiffDrive│  │ DiffDrive│  │ DiffDrive│           │
│  │ Lidar    │  │ Lidar    │  │ Lidar    │  │ Lidar    │           │
│  │ IMU      │  │ IMU      │  │ IMU      │  │ IMU      │           │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘           │
└───────┼─────────────┼─────────────┼─────────────┼───────────────────┘
        │ gz topics   │             │             │
        ▼             ▼             ▼             ▼
┌───────────────────────────────────────────────────────────────────────┐
│                     ros_gz_bridge (per robot)                         │
│  cmd_vel ROS→GZ │ odom GZ→ROS │ scan GZ→ROS │ imu GZ→ROS           │
└───────┬─────────────────────────────────────────────────────────────-┘
        │ ROS2 topics
        ▼
┌───────────────────────────────────────────────────────────────────────┐
│  Per-robot ROS2 Nodes (namespace: /robot_N)                           │
│                                                                       │
│  robot_state_publisher  ──────────────────────►  /tf_static           │
│  odom_to_tf             ── dynamic odom→bf ───►  /tf                  │
│                         ── static map→odom  ───►  /tf_static          │
│                                                                       │
│  ┌──────────────────── Nav2 Stack ──────────────────────────────┐    │
│  │  map_server → lifecycle_manager_localization                 │    │
│  │                                                              │    │
│  │  controller_server (RPP)    │ local_costmap                  │    │
│  │  planner_server (NavFn)     │ global_costmap                 │    │
│  │  behavior_server            │ bt_navigator                   │    │
│  │  waypoint_follower          │ velocity_smoother              │    │
│  │  lifecycle_manager_navigation                                │    │
│  └──────────────────────────────────────────────────────────────┘    │
└───────────────────────────────────────────────────────────────────────┘
        │
        ▼
┌───────────────────────┐
│  send_goals.py        │  ← user-initiated goal commands
│  (fleet_goal_sender)  │
│  NavigateToPose action│
└───────────────────────┘
```

---

## 3. Software Stack

| Component | Version / Package |
|---|---|
| OS | Ubuntu 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Simulator | Gazebo Harmonic (gz-sim 8.x) |
| ROS-Gazebo Bridge | ros_gz_bridge, ros_gz_sim |
| Navigation | Nav2 1.1.20 |
| DDS Middleware | CycloneDDS (`rmw_cyclonedds_cpp`) |
| Path Planner | NavFn (Dijkstra / A*) |
| Path Controller | Regulated Pure Pursuit (RPP) |
| BT Framework | BehaviorTree.CPP v3 (via nav2_bt_navigator) |
| Localization | Static TF (no AMCL — known spawn positions in simulation) |

> **DDS Note:** FastRTPS causes service response timeouts with 32+ simultaneous lifecycle nodes. **CycloneDDS is mandatory.**
> ```bash
> export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
> ```

---

## 4. Workspace Structure

```
~/fms_ws/
├── src/
│   ├── fms_msgs/                   # Custom ROS2 message definitions
│   │   └── msg/
│   │       ├── RobotStatus.msg
│   │       ├── TaskAssignment.msg
│   │       └── TaskCompletion.msg
│   │
│   ├── fms_gazebo/                 # Simulation: world + robot model + spawning
│   │   ├── launch/
│   │   │   ├── sim.launch.py       # Gazebo + N robots
│   │   │   └── spawn_robot.launch.py  # Per-robot RSP + bridge + odom_to_tf
│   │   ├── scripts/
│   │   │   └── odom_to_tf.py       # Odom → TF relay + static map→odom
│   │   ├── urdf/
│   │   │   └── fms_robot.urdf.xacro
│   │   └── worlds/
│   │       └── warehouse.sdf
│   │
│   └── fms_navigation/             # Nav2 configuration + launch + goals
│       ├── config/
│       │   └── nav2_params.yaml    # Base Nav2 parameters
│       ├── launch/
│       │   ├── bringup.launch.py   # Full system bringup
│       │   └── nav2_robot.launch.py  # Nav2 stack per robot
│       ├── maps/
│       │   ├── warehouse.pgm       # Occupancy grid map
│       │   └── warehouse.yaml      # Map metadata
│       ├── rviz/
│       │   └── fleet.rviz          # RViz config for 4 robots
│       └── scripts/
│           └── send_goals.py       # Multi-robot simultaneous goal sender
│
├── docs/
│   └── PHASE1_DOCUMENTATION.md    # This file
└── install/                        # Colcon build output
```

---

## 5. Package Descriptions

### 5.1 `fms_msgs`

Custom message definitions used across all FMS phases.

#### `RobotStatus.msg`

```
std_msgs/Header header
string robot_id
uint8 state
uint8 STATE_IDLE=0
uint8 STATE_ASSIGNED=1
uint8 STATE_NAVIGATING=2
uint8 STATE_EXECUTING=3
uint8 STATE_REPORTING=4
uint8 STATE_RECOVERING=5
uint8 STATE_CHARGING=6
geometry_msgs/Pose pose
float32 battery_soc
string current_task_id
string status_message
```

#### `TaskAssignment.msg`

```
std_msgs/Header header
string task_id
string robot_id
uint8 task_type
uint8 TASK_PICK=0
uint8 TASK_DROP=1
uint8 TASK_CHARGE=2
geometry_msgs/Pose pick_pose
geometry_msgs/Pose drop_pose
string payload_id
uint8 priority
float32 deadline_secs
```

#### `TaskCompletion.msg`

```
std_msgs/Header header
string task_id
string robot_id
uint8 result
uint8 RESULT_SUCCESS=0
uint8 RESULT_FAILED=1
uint8 RESULT_CANCELLED=2
float32 duration_secs
string error_message
```

---

### 5.2 `fms_gazebo`

Handles simulation: Gazebo world, robot URDF, spawning, sensor bridging, and TF publishing.

**Key files:**

| File | Purpose |
|---|---|
| `worlds/warehouse.sdf` | Gazebo warehouse with shelving aisles and walls |
| `urdf/fms_robot.urdf.xacro` | Differential-drive AMR: base, wheels, caster, lidar, IMU |
| `launch/sim.launch.py` | Launches Gazebo + spawns N robots |
| `launch/spawn_robot.launch.py` | Per-robot RSP, ros_gz_bridge, odom_to_tf |
| `scripts/odom_to_tf.py` | Dynamic odom→base_footprint + static map→odom TF |

**Robot spawn positions (4 robots):**

| Robot | X (m) | Y (m) | Yaw |
|---|---|---|---|
| robot_1 | -5.0 | -17.0 | 0° |
| robot_2 | -5.0 | -14.0 | 0° |
| robot_3 | -5.0 | -11.0 | 0° |
| robot_4 | -5.0 | -8.0 | 0° |

Supports up to 8 robots (ROBOT_POSES list in sim.launch.py).

---

### 5.3 `fms_navigation`

Nav2 configuration, lifecycle management, and goal sending for all robots.

**Key files:**

| File | Purpose |
|---|---|
| `config/nav2_params.yaml` | Base Nav2 parameters (all node types) |
| `launch/bringup.launch.py` | Top-level launch: RViz → Gazebo → map_server → Nav2 (staggered) |
| `launch/nav2_robot.launch.py` | Per-robot Nav2 lifecycle nodes |
| `maps/warehouse.pgm` | Binary occupancy grid (50m × 40m, 0.05 m/px) |
| `maps/warehouse.yaml` | Map metadata: origin [-25, -20, 0] |
| `rviz/fleet.rviz` | RViz layout with TF, Map, and RobotModel per robot |
| `scripts/send_goals.py` | Simultaneous NavigateToPose goal sender |

---

## 6. Robot Model (URDF/Xacro)

**File:** [src/fms_gazebo/urdf/fms_robot.urdf.xacro](../src/fms_gazebo/urdf/fms_robot.urdf.xacro)

### Physical Dimensions

| Part | Spec |
|---|---|
| Base (L × W × H) | 0.6 m × 0.4 m × 0.2 m |
| Wheel radius | 0.1 m |
| Wheel length | 0.05 m |
| Wheel separation | 0.45 m (base_width + wheel_length) |
| Caster radius | 0.05 m |
| Lidar height | base_height + 0.03 m above base |

### Link Hierarchy

```
base_footprint  (virtual, ground contact)
└── base_link   (chassis, 5 kg)
    ├── right_wheel_link  (1 kg, continuous joint)
    ├── left_wheel_link   (1 kg, continuous joint)
    ├── caster_wheel_link (0.5 kg, fixed)
    ├── lidar_link        (0.2 kg, fixed)
    └── imu_link          (0.01 kg, fixed)
```

### Gazebo Plugins

| Plugin | Topic | Notes |
|---|---|---|
| `gz-sim-diff-drive-system` | `/{robot_name}/cmd_vel`, `/{robot_name}/odom` | frame_id: `{robot_name}/odom`, child: `{robot_name}/base_footprint` |
| `gz-sim-joint-state-publisher-system` | `/{robot_name}/joint_states` | Both wheel joints |
| `gpu_lidar` sensor | `/{robot_name}/scan` | 360°, 12 m range, 10 Hz, 360 samples |
| `imu` sensor | `/{robot_name}/imu` | 100 Hz, Gaussian noise |

### Critical Design Decisions

1. **Plain link names (no slash prefix):** Link names are `base_link`, `lidar_link`, etc. — NOT `robot_1/base_link`. Gazebo sensor attachment breaks if link names contain slashes.

2. **`frame_prefix` in RSP:** `robot_state_publisher` runs with `frame_prefix: "robot_N/"` so TF frames become `robot_N/base_link`, `robot_N/lidar_link`, etc. This achieves multi-robot TF isolation without slashes in URDF.

3. **`frame_id` in DiffDrive plugin:** The Gazebo DiffDrive plugin uses `$(arg robot_name)/odom` and `$(arg robot_name)/base_footprint` as TF string names — these ARE the fully-qualified TF frame names, not URDF link names.

---

## 7. Warehouse Simulation (Gazebo SDF)

**File:** [src/fms_gazebo/worlds/warehouse.sdf](../src/fms_gazebo/worlds/warehouse.sdf)

**World dimensions:** 50 m × 40 m  
**Coordinate origin:** Centre of the warehouse (Gazebo world frame = ROS map frame)

**Layout:**
- Shelving aisles run parallel to the X-axis
- Robots spawn on the left side (X ≈ -5 m)
- Pick stations on the right side (X ≈ +14 m)
- Charging docks on the far left (X ≈ -20 m)

---

## 8. Navigation Map

**Files:**
- [src/fms_navigation/maps/warehouse.yaml](../src/fms_navigation/maps/warehouse.yaml)
- [src/fms_navigation/maps/warehouse.pgm](../src/fms_navigation/maps/warehouse.pgm)

**Map metadata:**

```yaml
image: warehouse.pgm
resolution: 0.05       # metres per pixel
origin: [-25.0, -20.0, 0.0]   # lower-left corner in map frame
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

**Derivation of origin:**
- Map image is 1000 px wide × 800 px tall (50 m × 40 m at 0.05 m/px)
- The warehouse extends from X=[-25, +25] and Y=[-20, +20] in the world frame
- `origin: [-25, -20, 0]` places the image lower-left at world (-25, -20)
- **MAP frame = Gazebo WORLD frame** (same origin, no offset)

**Critical fix:** The PGM generator had a Y-axis flip bug (worldToRow did not negate Y), causing 0% occupancy in the map. After the fix, obstacles from the SDF are correctly rasterised into the PGM.

---

## 9. TF Tree Design

Each robot has its own isolated TF sub-tree:

```
map  ──(static)──►  robot_N/odom  ──(dynamic)──►  robot_N/base_footprint
                                                         │
                         robot_state_publisher:          ├──► robot_N/base_link
                         (frame_prefix = robot_N/)       ├──► robot_N/right_wheel_link
                                                         ├──► robot_N/left_wheel_link
                                                         ├──► robot_N/caster_wheel_link
                                                         ├──► robot_N/lidar_link
                                                         └──► robot_N/imu_link
```

### TF Publishers

| Transform | Publisher | Type |
|---|---|---|
| `map → robot_N/odom` | `odom_to_tf.py` (StaticTransformBroadcaster) | Static, published once at startup |
| `robot_N/odom → robot_N/base_footprint` | `odom_to_tf.py` (TransformBroadcaster) | Dynamic, updated at odom rate (20 Hz) |
| `robot_N/base_footprint → robot_N/base_link` | `robot_state_publisher` | Static from URDF |
| `robot_N/base_link → robot_N/*_link` | `robot_state_publisher` | Static or joint-driven |

### Why odom_to_tf Instead of ros_gz_bridge TF

`ros_gz_bridge` publishes `/tf` with a QoS profile (VOLATILE, KEEP_LAST=1) that `tf2::Buffer` does not subscribe to (tf2_ros expects VOLATILE, RELIABLE, KEEP_LAST=100). This causes a **silent QoS mismatch** — the TF messages arrive but are never received by Nav2.

**Fix:** `odom_to_tf.py` subscribes to `/robot_N/odom` and republishes the transform using `tf2_ros.TransformBroadcaster`, which always uses the correct QoS.

### Static map→odom Transform

Since the simulation uses **known spawn positions** (no AMCL drift), the `map→odom` transform is a fixed static transform:

```
translation: (spawn_x, spawn_y, 0.0)
rotation: identity (yaw = 0)
```

**Key insight:** `map_x = float(spawn_x)` — NOT `float(spawn_x) - MAP_ORIGIN_X`. The MAP frame and WORLD frame share the same origin, so no offset is needed.

---

## 10. Nav2 Stack Configuration

**File:** [src/fms_navigation/config/nav2_params.yaml](../src/fms_navigation/config/nav2_params.yaml)

### Lifecycle Nodes per Robot

| Node | Package | Role |
|---|---|---|
| `map_server` | nav2_map_server | Publishes static occupancy map |
| `controller_server` | nav2_controller | Runs RPP + local costmap |
| `planner_server` | nav2_planner | Runs NavFn + global costmap |
| `behavior_server` | nav2_behaviors | Recovery behaviors (spin, backup, wait) |
| `bt_navigator` | nav2_bt_navigator | Behavior tree orchestrator |
| `waypoint_follower` | nav2_waypoint_follower | Waypoint sequence executor |
| `velocity_smoother` | nav2_velocity_smoother | Smooths cmd_vel output |

Managed by two lifecycle managers per robot:
- `lifecycle_manager_localization` → manages `[map_server]`
- `lifecycle_manager_navigation` → manages all 6 navigation nodes

### Path Planner — NavFn

```yaml
GridBased:
  plugin: "nav2_navfn_planner/NavfnPlanner"
  tolerance: 0.5
  use_astar: false      # Dijkstra (safer for warehouse grids)
  allow_unknown: true
```

### Path Controller — Regulated Pure Pursuit (RPP)

```yaml
FollowPath:
  plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
  desired_linear_vel: 0.5       # m/s
  lookahead_dist: 0.6           # m
  use_rotate_to_heading: true
  allow_reversing: false
  max_angular_accel: 3.2
  xy_goal_tolerance: 0.25       # m
  yaw_goal_tolerance: 0.25      # rad
```

### Costmap Configuration

**Local costmap** (3 m × 3 m rolling window, odom frame):
- `obstacle_layer`: LaserScan from `/robot_N/scan`
- `inflation_layer`: radius 0.55 m, scaling factor 3.0

**Global costmap** (full map extent, map frame):
- `static_layer`: warehouse.pgm
- `obstacle_layer`: LaserScan from `/robot_N/scan`
- `inflation_layer`: radius 0.55 m, scaling factor 3.0

### Critical Plugin Naming (Nav2 1.1.20)

Nav2 1.1.20 uses **two different plugin name formats**:

| Format | Used when |
|---|---|
| `package/ClassName` (slash) | Plugin registered with `name=` attribute in plugin XML |
| `package::ClassName` (double colon) | Plugin registered with only `type=` attribute |

Incorrect format causes `FATAL` errors. The correct names for this project:

```yaml
# Planner
plugin: "nav2_navfn_planner/NavfnPlanner"         # slash

# Behaviors
plugin: "nav2_behaviors/Spin"                      # slash
plugin: "nav2_behaviors/BackUp"                    # slash
plugin: "nav2_behaviors/Wait"                      # slash
plugin: "nav2_behaviors/DriveOnHeading"            # slash
plugin: "nav2_behaviors/AssistedTeleop"            # slash

# BT Navigator navigators
plugin: "nav2_bt_navigator/NavigateToPoseNavigator"     # slash
plugin: "nav2_bt_navigator/NavigateThroughPosesNavigator"  # slash

# backup behavior key must match BT XML action server name:
backup:                                            # NOT "back_up"
  plugin: "nav2_behaviors/BackUp"
```

### YAML Parameter Key Design

**Problem:** Namespaced Nav2 nodes (e.g. `/robot_1/local_costmap/local_costmap`) require their YAML section key to be their **absolute fully-qualified path**. Short keys like `local_costmap.local_costmap` only match nodes in the root namespace.

**Solution:** `_make_robot_params()` in `nav2_robot.launch.py` generates a per-robot YAML file at runtime with absolute keys:

```yaml
/robot_1/map_server:
  ros__parameters: {...}
/robot_1/controller_server:
  ros__parameters: {...}
/robot_1/local_costmap/local_costmap:
  ros__parameters: {...}
/robot_1/global_costmap/global_costmap:
  ros__parameters: {...}
# ...etc
```

This file is written to a temp file (`/tmp/nav2_robot_1_XXXXXX.yaml`) and passed as `--params-file` to each node.

---

## 11. Launch System

### Launch Order and Timing

```
t=0s    RViz launches (fleet.rviz)
t=0s    Gazebo launches (warehouse.sdf)
t=0s    Robots spawned (robot_1 through robot_4)
t=0s    RSP + ros_gz_bridge + odom_to_tf per robot
t=5s    global_map_server + lifecycle_manager_map
t=10s   robot_1: map_server → (5s delay) → Nav2 stack
t=25s   robot_2: map_server → (5s delay) → Nav2 stack
t=40s   robot_3: map_server → (5s delay) → Nav2 stack
t=55s   robot_4: map_server → (5s delay) → Nav2 stack
```

**Why staggered startup?**

Starting all 4 robots simultaneously floods the DDS middleware with 32+ lifecycle node registrations. DDS service response messages for `change_state` calls time out → `map_server` stays unconfigured → global costmap never receives the map → "Robot is out of bounds" error.

Fix: `NAV2_FIRST_DELAY = 10.0 s`, `NAV2_STAGGER_STEP = 15.0 s`.

**Why 5s delay between localization and navigation?**

Both `lifecycle_manager_localization` and `lifecycle_manager_navigation` start simultaneously and compete for DDS service slots. `map_server` must be `ACTIVE` before the global costmap requests the map. The 5s `TimerAction` in `nav2_robot.launch.py` ensures `map_server` finishes activating first.

### Launch Files

#### `bringup.launch.py`

**Usage:**
```bash
ros2 launch fms_navigation bringup.launch.py
ros2 launch fms_navigation bringup.launch.py num_robots:=1
ros2 launch fms_navigation bringup.launch.py use_rviz:=false headless:=true
```

**Arguments:**

| Argument | Default | Description |
|---|---|---|
| `num_robots` | `4` | Number of robots (1–8) |
| `use_slam` | `false` | SLAM Toolbox mapping (single robot only) |
| `use_rviz` | `true` | Launch RViz |
| `headless` | `false` | Gazebo without GUI |

#### `sim.launch.py`

Launches Gazebo and calls `spawn_robot.launch.py` for each robot.

#### `spawn_robot.launch.py`

Launches per robot:
1. `robot_state_publisher` (with `frame_prefix`)
2. Gazebo `create` entity spawner
3. `ros_gz_bridge` (cmd_vel, odom, scan, imu, joint_states)
4. `odom_to_tf.py` (odom→TF relay + map→odom static TF)

#### `nav2_robot.launch.py`

Launches per robot (called by bringup with stagger delay):
1. `map_server` + `lifecycle_manager_localization` (immediate)
2. After 5s: `controller_server`, `planner_server`, `behavior_server`, `bt_navigator`, `waypoint_follower`, `velocity_smoother`, `lifecycle_manager_navigation`

---

## 12. Multi-Robot Goal Sender

**File:** [src/fms_navigation/scripts/send_goals.py](../src/fms_navigation/scripts/send_goals.py)

### Usage

```bash
SCRIPT=~/fms_ws/install/fms_navigation/share/fms_navigation/scripts/send_goals.py

python3 $SCRIPT                    # all 4 robots, goal set 0 (pick stations)
python3 $SCRIPT --goal_set 1       # pick stations (X ≈ +14)
python3 $SCRIPT --goal_set 2       # charge docks (X ≈ -20)
python3 $SCRIPT --goal_set 3       # through aisles
python3 $SCRIPT --goal_set 4       # return to spawn
python3 $SCRIPT --robots 1 2       # only robot_1 and robot_2
```

### Predefined Goal Sets

| Set | Description | robot_1 | robot_2 | robot_3 | robot_4 |
|---|---|---|---|---|---|
| 0 / 1 | Pick stations | (14, -17) | (14, -14) | (14, -11) | (14, -8) |
| 2 | Charge docks | (-20, -17) | (-20, -14) | (-20, -11) | (-20, -8) |
| 3 | Through aisles | (14, -5) | (-5, 5) | (0, 10) | (-20, 0) |
| 4 | Return to spawn | (-5, -17) | (-5, -14) | (-5, -11) | (-5, -8) |

All coordinates in the **map frame** (= Gazebo world frame).

### Implementation

- Uses `rclpy.action.ActionClient` with `NavigateToPose` action
- Goals sent simultaneously to all robots (non-blocking `send_goal_async`)
- `threading.Event` per robot for completion tracking
- Background `rclpy.spin` thread for callback processing
- Feedback shows distance remaining every 0.5 m change

### Shutdown Sequence (critical)

```python
# CORRECT order — prevents core dump:
rclpy.shutdown()          # 1. signal rclpy to stop spin loop
spin_thread.join(timeout=3.0)   # 2. wait for spin thread to exit
sys.exit(...)             # 3. exit

# WRONG — causes "terminate called without an active exception" + core dump:
# node.destroy_node()  ← deletes C++ node object from wrong thread
```

---

## 13. Critical Bugs Fixed

### B1 — `cannot import name 'Node' from 'launch.actions'`
- **Cause:** `Node` is in `launch_ros.actions`, not `launch.actions`
- **Fix:** `from launch_ros.actions import Node`

### B2 — URDF frame_id with leading slash
- **Cause:** `/robot_1/odom` in DiffDrive plugin caused TF frames with double-slash
- **Fix:** Use `robot_1/odom` (no leading slash)

### B3 — Warehouse PGM 0% occupied
- **Cause:** PGM generator `worldToRow()` did not flip the Y axis
- **Fix:** `row = (max_y - world_y) / resolution` (negate Y before mapping to row index)

### B4 — ros_gz_bridge TF QoS mismatch
- **Cause:** ros_gz_bridge publishes `/tf` with VOLATILE/KEEP_LAST=1; tf2::Buffer expects RELIABLE/KEEP_LAST=100
- **Fix:** Created `odom_to_tf.py` using `tf2_ros.TransformBroadcaster` (always uses correct QoS)

### B5 — Slashes in URDF link names breaking Gazebo sensors
- **Cause:** `robot_1/lidar_link` as URDF link name prevents Gazebo from finding the sensor attachment
- **Fix:** Plain names `lidar_link`, `base_link` etc.; use RSP `frame_prefix` for TF namespacing

### B6 — Nav2 plugin `::` vs `/` format (3 rounds)
- **Cause:** Nav2 1.1.20 uses `/` for plugins with `name=` attr in XML, `::` for those with only `type=`
- **Fix:** Inspected plugin XML; corrected to `nav2_navfn_planner/NavfnPlanner`, `nav2_behaviors/Spin`, etc.

### B7 — `back_up` vs `backup` behavior key
- **Cause:** Default BT XML uses action server name `backup`; config key was `back_up`
- **Fix:** Renamed config key to `backup:` to match BT XML action server name

### B8 — bt_navigator inactive (navigator plugins)
- **Cause:** Navigator plugins are loaded in `on_activate()`, requiring `/` format too
- **Fix:** `"nav2_bt_navigator/NavigateToPoseNavigator"` (slash, not double-colon)

### B9 — Robot spawns out of map bounds
- **Cause:** `map_x = float(x) - MAP_ORIGIN_X` computed wrong offset (+20 m error)
  - robot spawns at world (-5, -17) but map_x computed as (-5 - (-25)) = +20
  - Goals like (14, -17) sent as map (39, -17) → 14 m outside 50 m map
- **Fix:** `map_x = float(x)` — MAP frame = WORLD frame, no offset needed

### B10 — DDS response timeout for action goals
- **Cause:** FastRTPS overwhelmed by 32+ simultaneous lifecycle registrations
- **Fix:** `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`

### B11 — Core dump after script completion
- **Cause:** `node.destroy_node()` called while `rclpy.spin()` thread still running
- **Fix:** Call `rclpy.shutdown()` first (stops spin loop), then `spin_thread.join()`

### B12 — `map_server` inactive [2], costmap never gets map
- **Cause:** `lifecycle_manager_localization` and `lifecycle_manager_navigation` competed for DDS slots simultaneously
- **Fix:** 5s `TimerAction` delay before starting navigation lifecycle (localization must be ACTIVE first)

### B13 — YAML short keys don't match namespaced nodes
- **Cause:** `local_costmap.local_costmap` only matches `/local_costmap/local_costmap`, not `/robot_1/local_costmap/local_costmap`
- **Fix:** `_make_robot_params()` generates absolute YAML keys like `/robot_1/local_costmap/local_costmap`

### B14 — SLAM Toolbox lifecycle hang
- **Cause:** SLAM Toolbox requires existing TF tree before configuring; not suitable for multi-robot static-map scenario
- **Fix:** Switched to static map + odom_to_tf approach (AMCL not needed in simulation with known poses)

---

## 14. Validation Results

### Phase 1 Success Run

```
============================================================
  FMS Fleet Goal Sender — goal set 1
  Robots: robot_1, robot_2, robot_3, robot_4
============================================================
  robot_1: goal (14.0, -17.0)  yaw=0°
  robot_2: goal (14.0, -14.0)  yaw=0°
  robot_3: goal (14.0, -11.0)  yaw=0°
  robot_4: goal (14.0,  -8.0)  yaw=0°
============================================================

  [robot_1] ✓ Goal accepted — navigating...
  [robot_2] ✓ Goal accepted — navigating...
  [robot_3] ✓ Goal accepted — navigating...
  [robot_4] ✓ Goal accepted — navigating...
  ...
  [robot_1] ✅ SUCCEEDED
  [robot_2] ✅ SUCCEEDED
  [robot_4] ✅ SUCCEEDED
  [robot_3] ✅ SUCCEEDED

============================================================
  RESULTS  (elapsed: 215.1 s)
============================================================
  ✅  robot_1: SUCCEEDED
  ✅  robot_2: SUCCEEDED
  ✅  robot_3: SUCCEEDED
  ✅  robot_4: SUCCEEDED
============================================================
  4/4 robots reached their goals
============================================================
```

### Verification Commands

```bash
# Check lifecycle states
ros2 lifecycle get /robot_1/map_server
ros2 lifecycle get /robot_1/controller_server
ros2 lifecycle get /robot_1/planner_server
ros2 lifecycle get /robot_1/bt_navigator

# All should return: active [3]

# Verify TF tree
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map robot_1/base_footprint

# List active action servers
ros2 action list

# Check scan topics
ros2 topic hz /robot_1/scan    # expect ~10 Hz
ros2 topic hz /robot_1/odom    # expect ~20 Hz
```

---

## 15. Operating Procedures

### Prerequisites

```bash
# Install CycloneDDS
sudo apt install ros-humble-rmw-cyclonedds-cpp

# Add to ~/.bashrc (permanent)
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

### Build

```bash
cd ~/fms_ws
colcon build --symlink-install
source install/setup.bash
```

### Launch Full System (4 robots)

```bash
# Terminal 1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash && source ~/fms_ws/install/setup.bash
ros2 launch fms_navigation bringup.launch.py
```

Wait approximately **70 seconds** for all Nav2 stacks to become active.

### Send Goals

```bash
# Terminal 2
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash && source ~/fms_ws/install/setup.bash

SCRIPT=~/fms_ws/install/fms_navigation/share/fms_navigation/scripts/send_goals.py

# Send all robots to pick stations simultaneously
python3 $SCRIPT --goal_set 1

# Send robots to charge docks
python3 $SCRIPT --goal_set 2

# Return to spawn
python3 $SCRIPT --goal_set 4

# Single robot test
python3 $SCRIPT --robots 1 --goal_set 1
```

### Single-Robot Test (faster startup)

```bash
ros2 launch fms_navigation bringup.launch.py num_robots:=1
# Wait ~15 seconds, then:
python3 $SCRIPT --robots 1 --goal_set 1
```

### Manual Goal via CLI

```bash
ros2 action send_goal /robot_1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 14.0, y: -17.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

---

## 16. Known Limitations

| Limitation | Impact | Planned Fix |
|---|---|---|
| Static TF (no AMCL) | Map→odom offset accumulates if robot slips; not an issue in simulation | AMCL or EKF in real hardware deployment |
| Startup takes ~70s for 4 robots | Slow iteration during development | Reduce `NAV2_STAGGER_STEP` for faster hardware or tune DDS queue sizes |
| No inter-robot collision avoidance | Robots plan independently; possible collisions at intersections | Phase 3: Central path coordinator or DWA social force model |
| No battery simulation | `battery_soc` field in RobotStatus always 0 | Phase 2: Simulated battery drain model |
| No task assignment logic | Goals sent manually via send_goals.py | Phase 2: Robot state machine + task dispatcher |
| CycloneDDS required | Cannot use default FastRTPS | Acceptable for development; document for deployment |
| Core dump on Ctrl+C of send_goals.py (edge case) | Script must call `rclpy.shutdown()` before `node.destroy_node()` | Already fixed; may reappear if node is destroyed prematurely |

---

*Documentation generated: 2026-06-05*  
*Phase 1 completed: All 4 robots SUCCEEDED — elapsed 215.1 s*
