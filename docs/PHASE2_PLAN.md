# Phase 2 Plan — Robot State Machine & Behavior Tree
## Per-Robot FSM + BT + Battery Model + Fault Recovery

**Status:** Planning  
**Target:** Weeks 3–4  
**Milestone:** Kill any robot mid-task; fleet recovers and reallocates the task.

---

## Table of Contents

1. [Overview](#1-overview)
2. [Phase 1 Foundation Used](#2-phase-1-foundation-used)
3. [New Package: fms_robot_agent](#3-new-package-fms_robot_agent)
4. [Package Structure](#4-package-structure)
5. [Dependencies](#5-dependencies)
6. [Component Design](#6-component-design)
   - 6.1 [Robot FSM (boost::sml)](#61-robot-fsm-boostsml)
   - 6.2 [Battery Model](#62-battery-model)
   - 6.3 [Behavior Tree XML](#63-behavior-tree-xml)
   - 6.4 [BT Custom Action Nodes](#64-bt-custom-action-nodes)
   - 6.5 [Robot Agent Lifecycle Node](#65-robot-agent-lifecycle-node)
7. [ROS 2 Interface Design](#7-ros-2-interface-design)
8. [File-by-File Implementation Plan](#8-file-by-file-implementation-plan)
9. [Launch System Updates](#9-launch-system-updates)
10. [Fault Injection Design](#10-fault-injection-design)
11. [Task Injector Test Script](#11-task-injector-test-script)
12. [Test Scenarios](#12-test-scenarios)
13. [Known Risks and Gotchas](#13-known-risks-and-gotchas)
14. [Build and Verification Commands](#14-build-and-verification-commands)

---

## 1. Overview

Phase 2 adds a **per-robot C++ lifecycle node** (`fms_robot_agent`) that sits on top of the Phase 1 Nav2 stack. Each robot runs an independent behavior tree (BT) orchestrating its full task lifecycle, backed by a `boost::sml` FSM that tracks state transitions. A simulated battery model drives charging behavior, and a fault-injection service enables recovery testing.

**Phase 2 Deliverables:**
- `fms_robot_agent` ROS 2 C++17 package with rclcpp LifecycleNode
- `boost::sml` FSM covering 7 states: `IDLE → ASSIGNED → NAVIGATING → EXECUTING → REPORTING → IDLE`
- BehaviorTree.CPP v4 custom nodes: `RequestTask`, `NavigateToPose`, `ExecutePickDrop`, `ReportStatus`, `RequestRecovery`, `BatteryOK`
- Simulated battery drain/charge model with auto-dock behavior (auto-navigate to charger when SOC < 20%)
- Fault injection service to simulate Nav2 mid-task failures
- `task_injector.py` script to publish task assignments without Phase 3 fleet server

**Architecture Addition:**

```
[Phase 2 Addition]

 task_injector.py
      │  /robot_N/task_assignment (TaskAssignment.msg)
      ▼
 ┌─────────────────────────────────────────────────┐
 │  RobotAgentNode (LifecycleNode, /robot_N)       │
 │                                                  │
 │  boost::sml FSM                                  │
 │    IDLE → ASSIGNED → NAVIGATING → EXECUTING      │
 │    → REPORTING → IDLE                            │
 │    ↓ (fault)                                     │
 │    RECOVERING → IDLE (requeue) or CHARGING       │
 │                                                  │
 │  BehaviorTree.CPP v4 BT                          │
 │    RequestTask → NavigateToPose →                │
 │    ExecutePickDrop → NavigateToPose →            │
 │    ExecutePickDrop → ReportStatus                │
 │    (battery branch: BatteryOK / NavigateToCharger│
 │                     / ChargeBattery)             │
 │                                                  │
 │  BatteryModel (drain 0.1%/s moving, charge 1%/s) │
 └──────────────┬──────────────────────────────────┘
                │ NavigateToPose action calls
                ▼
   /robot_N/navigate_to_pose  (Phase 1 Nav2 stack)
                │
                │ publishes
                ▼
   /robot_N/robot_status    (RobotStatus.msg, 2 Hz)
   /robot_N/task_completion (TaskCompletion.msg, on event)
```

---

## 2. Phase 1 Foundation Used

| Phase 1 Asset | How Phase 2 Uses It |
|---|---|
| `fms_msgs` — `RobotStatus.msg`, `TaskAssignment.msg`, `TaskCompletion.msg` | Published/subscribed by robot agent node directly |
| Nav2 action server `/robot_N/navigate_to_pose` | Called by `NavigateToPose` BT node |
| Nav2 recovery behaviors: `spin`, `backup`, `wait` (via `/robot_N/spin`, `/robot_N/backup`, `/robot_N/wait`) | Called by `RequestRecovery` BT node |
| Warehouse layout: pick stations (X≈+14), charge docks (X≈-20), spawn positions | Hardcoded as named waypoints in `robot_agent_params.yaml` |
| `bringup.launch.py` staggered startup pattern | Extended to stagger `robot_agent` node launches too |
| CycloneDDS requirement | Unchanged; documented in launch |

---

## 3. New Package: fms_robot_agent

This is the only new package for Phase 2. It lives at `src/fms_robot_agent/`.

**Build type:** `ament_cmake`  
**Language:** C++17  
**Node type:** `rclcpp_lifecycle::LifecycleNode`

---

## 4. Package Structure

```
src/fms_robot_agent/
├── CMakeLists.txt
├── package.xml
│
├── include/fms_robot_agent/
│   ├── robot_agent_node.hpp        # LifecycleNode: owns FSM, BT, battery
│   ├── robot_fsm.hpp               # boost::sml FSM definition (states + transitions)
│   ├── battery_model.hpp           # Drain/charge simulation
│   └── bt_nodes/
│       ├── request_task.hpp        # BT StatefulActionNode
│       ├── navigate_to_pose.hpp    # BT StatefulActionNode (Nav2 action client)
│       ├── execute_pick_drop.hpp   # BT StatefulActionNode (timed simulation)
│       ├── report_status.hpp       # BT SyncActionNode (publish TaskCompletion)
│       ├── request_recovery.hpp    # BT StatefulActionNode (Nav2 recovery client)
│       └── battery_ok.hpp          # BT ConditionNode (SOC > threshold)
│
├── src/
│   ├── robot_agent_node.cpp
│   ├── battery_model.cpp
│   └── bt_nodes/
│       ├── request_task.cpp
│       ├── navigate_to_pose.cpp
│       ├── execute_pick_drop.cpp
│       ├── report_status.cpp
│       ├── request_recovery.cpp
│       └── battery_ok.cpp
│
├── bt_xml/
│   └── robot_agent.xml             # Main BT definition
│
├── launch/
│   └── robot_agent.launch.py       # Launch one agent (called per robot by bringup)
│
├── config/
│   └── robot_agent_params.yaml     # Waypoints, thresholds, timing params
│
└── scripts/
    └── task_injector.py            # Test script: publish TaskAssignment messages
```

---

## 5. Dependencies

### CMakeLists.txt `find_package` additions

```cmake
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v4 REQUIRED)   # or behaviortree_cpp per distro
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(fms_msgs REQUIRED)
find_package(Boost REQUIRED)                  # sml is header-only in Boost
```

### package.xml `<depend>` additions

```xml
<depend>rclcpp</depend>
<depend>rclcpp_lifecycle</depend>
<depend>rclcpp_action</depend>
<depend>behaviortree_cpp_v4</depend>
<depend>nav2_msgs</depend>
<depend>geometry_msgs</depend>
<depend>std_srvs</depend>
<depend>fms_msgs</depend>
```

### System install notes

```bash
# BT.CPP v4 (check apt first; if unavailable, build from source)
sudo apt install ros-humble-behaviortree-cpp-v4

# boost::sml (header-only, bundled in Boost or install separately)
sudo apt install libboost-dev

# If behaviortree_cpp_v4 not in apt, use the v3 package name and adjust:
sudo apt install ros-humble-behaviortree-cpp
# Then use #include <behaviortree_cpp_v3/...> and find_package(behaviortree_cpp REQUIRED)
```

**Risk:** ROS 2 Humble ships BT.CPP v3.8 as the default `behaviortree_cpp` package. If `behaviortree_cpp_v4` is unavailable, use v3 API — the custom node base classes differ slightly (`StatefulActionNode` in v4 vs `AsyncActionNode` in v3). The plan uses v4 API throughout; adapt the base class names if falling back to v3.

---

## 6. Component Design

### 6.1 Robot FSM (boost::sml)

**File:** `include/fms_robot_agent/robot_fsm.hpp`

The FSM is defined entirely in the header using `boost::sml` lambda-based DSL.

#### States

```cpp
namespace states {
  struct Idle       {};
  struct Assigned   {};
  struct Navigating {};
  struct Executing  {};
  struct Reporting  {};
  struct Recovering {};
  struct Charging   {};
}
```

#### Events

```cpp
namespace events {
  struct TaskReceived    { fms_msgs::msg::TaskAssignment::SharedPtr task; };
  struct NavStarted      {};
  struct NavSucceeded    {};
  struct NavFailed       { std::string reason; };
  struct ExecutionDone   {};
  struct ReportSent      {};
  struct LowBattery      {};   // SOC < LOW_SOC_THRESHOLD (20%)
  struct ChargerReached  {};   // NavigateToPose to charger succeeded
  struct BatteryFull     {};   // SOC > FULL_SOC_THRESHOLD (95%)
  struct RecoveryDone    {};   // Recovery behavior completed
  struct RecoveryFailed  {};   // All recovery attempts exhausted
  struct FaultInjected   {};   // Via /robot_N/inject_fault service
}
```

#### Transition Table

```
IDLE        + TaskReceived    → ASSIGNED    / store_task
ASSIGNED    + NavStarted      → NAVIGATING
NAVIGATING  + NavSucceeded    → EXECUTING
NAVIGATING  + NavFailed       → RECOVERING  / cancel_task_nav
NAVIGATING  + FaultInjected   → RECOVERING  / cancel_nav_goal
EXECUTING   + ExecutionDone   → REPORTING
EXECUTING   + NavFailed       → RECOVERING
REPORTING   + ReportSent      → IDLE        / clear_task
RECOVERING  + RecoveryDone    → ASSIGNED    / requeue (retry nav)
RECOVERING  + RecoveryFailed  → IDLE        / publish_failure
ANY         + LowBattery      → CHARGING    / save_interrupted_task
CHARGING    + ChargerReached  → CHARGING    (stays, starts draining counter)
CHARGING    + BatteryFull     → IDLE        / restore_interrupted_task_if_any
```

**Key invariant:** `LowBattery` is a deferred event — it fires from the BatteryModel timer and transitions from any state to CHARGING, saving the current task to be resumed after charging.

#### Header sketch

```cpp
// robot_fsm.hpp
#pragma once
#include <boost/sml.hpp>
#include <fms_msgs/msg/task_assignment.hpp>
#include <string>

namespace fms_robot_agent {

struct RobotFSM {
  // State tags (empty structs act as state IDs in sml)
  struct Idle       {};
  struct Assigned   {};
  struct Navigating {};
  struct Executing  {};
  struct Reporting  {};
  struct Recovering {};
  struct Charging   {};

  // Events (forward-declared here, defined in .hpp)
  struct TaskReceived   { fms_msgs::msg::TaskAssignment::SharedPtr task; };
  struct NavSucceeded   {};
  struct NavFailed      { std::string reason; };
  struct ExecutionDone  {};
  struct ReportSent     {};
  struct LowBattery     {};
  struct BatteryFull    {};
  struct ChargerReached {};
  struct RecoveryDone   {};
  struct RecoveryFailed {};
  struct FaultInjected  {};

  // Transition table defined via auto operator()() const in the .cpp
  // RobotAgentNode holds: boost::sml::sm<RobotFSM> sm_{RobotFSM{...}, *this};
};

}  // namespace fms_robot_agent
```

---

### 6.2 Battery Model

**File:** `include/fms_robot_agent/battery_model.hpp` + `src/battery_model.cpp`

```cpp
class BatteryModel {
public:
  explicit BatteryModel(double initial_soc = 100.0);

  // Called from a rclcpp timer (e.g., every 100 ms)
  void update(double dt_secs, bool is_moving, bool is_charging);

  double soc() const;          // 0.0 – 100.0
  bool   is_low()  const;      // soc < LOW_THRESHOLD  (20.0)
  bool   is_full() const;      // soc > FULL_THRESHOLD (95.0)

private:
  static constexpr double DRAIN_IDLE    = 0.02;   // % per second while stationary
  static constexpr double DRAIN_MOVING  = 0.10;   // % per second while navigating/executing
  static constexpr double CHARGE_RATE   = 1.00;   // % per second while docked
  static constexpr double LOW_THRESHOLD = 20.0;
  static constexpr double FULL_THRESHOLD = 95.0;

  double soc_;
};
```

The `update()` is called from a 100 ms `rclcpp::TimerBase` in `RobotAgentNode`. When `is_low()` returns true, the node fires `LowBattery` into the FSM.

---

### 6.3 Behavior Tree XML

**File:** `bt_xml/robot_agent.xml`

The BT runs in a `ReactiveSequence` so the battery check preempts the task subtree on every tick.

```xml
<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="4">
  <BehaviorTree ID="RobotAgentMain">

    <!-- Reactive: battery check runs every tick and can abort task subtree -->
    <ReactiveSequence name="MainReactive">

      <!-- Battery branch: if SOC OK do nothing; else charge -->
      <Fallback name="BatteryGuard">
        <BatteryOK/>
        <Sequence name="ChargeSequence">
          <NavigateToPose name="NavToCharger"
                          goal="{charger_pose}"
                          robot_name="{robot_name}"/>
          <ChargeBattery  name="Charge"
                          duration_secs="10.0"/>
        </Sequence>
      </Fallback>

      <!-- Task execution subtree -->
      <Sequence name="TaskSequence">

        <!-- Block until a TaskAssignment arrives on the ROS topic -->
        <RequestTask name="WaitForTask"
                     task_out="{current_task}"/>

        <!-- Navigate to pick location with recovery on failure -->
        <RetryUntilSuccessful num_attempts="3" name="NavToPickWithRetry">
          <Fallback name="NavPickFallback">
            <NavigateToPose name="NavToPick"
                            goal="{pick_pose}"
                            robot_name="{robot_name}"/>
            <RequestRecovery name="RecoverFromNavFailure"
                             robot_name="{robot_name}"/>
          </Fallback>
        </RetryUntilSuccessful>

        <!-- Simulate pick operation (timed delay) -->
        <ExecutePickDrop name="DoPick"
                         operation="pick"
                         duration_secs="3.0"/>

        <!-- Navigate to drop location -->
        <RetryUntilSuccessful num_attempts="3" name="NavToDropWithRetry">
          <Fallback name="NavDropFallback">
            <NavigateToPose name="NavToDrop"
                            goal="{drop_pose}"
                            robot_name="{robot_name}"/>
            <RequestRecovery name="RecoverFromDropNavFailure"
                             robot_name="{robot_name}"/>
          </Fallback>
        </RetryUntilSuccessful>

        <!-- Simulate drop operation -->
        <ExecutePickDrop name="DoDrop"
                         operation="drop"
                         duration_secs="2.0"/>

        <!-- Publish TaskCompletion and update FSM to REPORTING→IDLE -->
        <ReportStatus name="ReportDone"/>

      </Sequence>

    </ReactiveSequence>
  </BehaviorTree>
</root>
```

**Blackboard ports used:**

| Key | Type | Direction | Notes |
|---|---|---|---|
| `robot_name` | string | input | Set at startup from param |
| `charger_pose` | geometry_msgs/PoseStamped | input | Set at startup from params |
| `current_task` | TaskAssignment | output from RequestTask | Read by NavigateToPose, ExecutePickDrop |
| `pick_pose` | geometry_msgs/PoseStamped | output from RequestTask | Extracted from current_task |
| `drop_pose` | geometry_msgs/PoseStamped | output from RequestTask | Extracted from current_task |

---

### 6.4 BT Custom Action Nodes

#### `RequestTask` — `StatefulActionNode`

- **onStart():** Subscribe to `/robot_N/task_assignment` if not already subscribed. Fire FSM `LowBattery`-safe status update. Return `RUNNING`.
- **onRunning():** Check if a task has been received (stored in a `std::optional<TaskAssignment>` set by the subscriber callback). If yes: set blackboard `current_task`, `pick_pose`, `drop_pose` — fire FSM `TaskReceived{task}` — return `SUCCESS`. Else return `RUNNING`.
- **onHalted():** Cancel subscription (or just ignore next message). Log halt.
- **Blackboard outputs:** `current_task`, `pick_pose`, `drop_pose`

#### `NavigateToPose` — `StatefulActionNode`

Wraps `rclcpp_action::Client<nav2_msgs::action::NavigateToPose>`.

- **onStart():** Read `goal` (PoseStamped) from blackboard. Send goal to `/robot_N/navigate_to_pose`. Fire FSM `NavStarted`. Return `RUNNING`.
- **onRunning():** Check goal handle status. If `SUCCEEDED`: fire `NavSucceeded`, return `SUCCESS`. If `ABORTED`/`CANCELED`: fire `NavFailed{reason}`, return `FAILURE`. Else return `RUNNING`.
- **onHalted():** Cancel the goal if still active. This is the key path for fault injection — when the BT is halted, cancel propagates to Nav2.
- **Blackboard inputs:** `goal` (PoseStamped), `robot_name`

#### `ExecutePickDrop` — `StatefulActionNode`

Simulates a physical pick or drop operation with a configurable delay.

- **onStart():** Log operation start. Start a `rclcpp::Clock` countdown from `duration_secs`. Fire FSM `NavSucceeded` (entering EXECUTING from NAVIGATING). Return `RUNNING`.
- **onRunning():** If duration elapsed: fire `ExecutionDone`. Return `SUCCESS`. Else return `RUNNING`.
- **onHalted():** Log cancellation.
- **Ports:** `operation` (string input: "pick" or "drop"), `duration_secs` (double input)

**Note:** In Phase 3, this node will call a real warehouse action (gripper actuation etc.). For Phase 2 it is a pure time delay.

#### `ReportStatus` — `SyncActionNode`

Synchronous — executes entirely in `tick()`.

- **tick():** Publish `TaskCompletion.msg` to `/robot_N/task_completion`. Fire FSM `ReportSent`. Return `SUCCESS`.
- The message is populated from the blackboard `current_task` and a wall-clock duration.

#### `RequestRecovery` — `StatefulActionNode`

Calls Nav2 recovery behaviors in sequence: `spin` → `backup` → retry.

- **onStart():** Send `nav2_msgs::action::Spin` goal to `/robot_N/spin`. Return `RUNNING`.
- **onRunning():** Poll spin result. If done, send `nav2_msgs::action::BackUp` goal to `/robot_N/backup`. Poll backup result. When both complete: fire FSM `RecoveryDone`. Return `SUCCESS`. If any fail beyond retry count: fire `RecoveryFailed`. Return `FAILURE`.
- **Blackboard inputs:** `robot_name`

#### `BatteryOK` — `ConditionNode`

- **tick():** Read `battery_soc` from blackboard (updated every tick by the node). Return `SUCCESS` if SOC >= 20%, else `FAILURE`.
- **Blackboard inputs:** `battery_soc` (double, updated by BatteryModel timer in the node)

#### `ChargeBattery` — `StatefulActionNode`

- **onStart():** Set FSM flag `is_charging = true`. Return `RUNNING`.
- **onRunning():** Read `battery_soc` from blackboard. If SOC >= 95%: set `is_charging = false`, fire FSM `BatteryFull`. Return `SUCCESS`. Else return `RUNNING`.
- **onHalted():** Set `is_charging = false`.
- **Ports:** `duration_secs` (optional max charge time, defaults to unlimited)

---

### 6.5 Robot Agent Lifecycle Node

**File:** `include/fms_robot_agent/robot_agent_node.hpp`

```cpp
class RobotAgentNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit RobotAgentNode(const std::string& robot_name,
                          const rclcpp::NodeOptions& opts = {});

  // Lifecycle callbacks
  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State&)  override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&)   override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&)  override;

private:
  void bt_tick_callback();          // Called by bt_timer_ (50 ms period)
  void battery_update_callback();   // Called by battery_timer_ (100 ms period)
  void task_assignment_callback(const fms_msgs::msg::TaskAssignment::SharedPtr msg);
  void publish_robot_status();      // Called by status_timer_ (500 ms period)

  bool inject_fault_service_cb(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr       res);

  std::string robot_name_;

  // BT
  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> bt_tree_;
  BT::Blackboard::Ptr       blackboard_;
  rclcpp::TimerBase::SharedPtr bt_timer_;

  // FSM
  std::unique_ptr<boost::sml::sm<RobotFSM>> fsm_;

  // Battery
  BatteryModel battery_;
  rclcpp::TimerBase::SharedPtr battery_timer_;
  bool is_charging_{false};

  // ROS interfaces
  rclcpp_lifecycle::LifecyclePublisher<fms_msgs::msg::RobotStatus>::SharedPtr
      status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<fms_msgs::msg::TaskCompletion>::SharedPtr
      completion_pub_;
  rclcpp::Subscription<fms_msgs::msg::TaskAssignment>::SharedPtr task_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr fault_srv_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  // Current task state
  std::optional<fms_msgs::msg::TaskAssignment> current_task_;
  std::atomic<bool> task_pending_{false};
};
```

**Lifecycle states:**
- `on_configure`: create publishers, subscriptions, service, BT factory registration, load BT XML, create blackboard with `robot_name` and `charger_pose`.
- `on_activate`: activate publishers, start `bt_timer_`, `battery_timer_`, `status_timer_`.
- `on_deactivate`: stop all timers, deactivate publishers.
- `on_cleanup`: destroy BT tree, reset FSM.

---

## 7. ROS 2 Interface Design

### Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/robot_N/task_assignment` | `fms_msgs/TaskAssignment` | Sub | Receives tasks (from injector or Phase 3 fleet server) |
| `/robot_N/robot_status` | `fms_msgs/RobotStatus` | Pub (2 Hz) | Heartbeat with state, pose, SOC |
| `/robot_N/task_completion` | `fms_msgs/TaskCompletion` | Pub (event) | Published once per task completion or failure |

### Services

| Service | Type | Description |
|---|---|---|
| `/robot_N/inject_fault` | `std_srvs/SetBool` | Triggers mid-task Nav2 goal cancellation to simulate failure |

### Actions (called, not advertised)

| Action | Type | Called by |
|---|---|---|
| `/robot_N/navigate_to_pose` | `nav2_msgs/NavigateToPose` | `NavigateToPose` BT node |
| `/robot_N/spin` | `nav2_msgs/Spin` | `RequestRecovery` BT node |
| `/robot_N/backup` | `nav2_msgs/BackUp` | `RequestRecovery` BT node |

### Parameters (`robot_agent_params.yaml`)

```yaml
robot_agent:
  ros__parameters:
    bt_xml_path: ""                     # auto-resolved from package share
    bt_tick_period_ms: 50               # BT tick rate
    battery_update_period_ms: 100
    status_publish_period_ms: 500
    low_battery_threshold: 20.0
    full_battery_threshold: 95.0
    initial_battery_soc: 100.0
    pick_duration_secs: 3.0
    drop_duration_secs: 2.0
    recovery_max_attempts: 3
    # Charger dock positions (one per robot, indexed by robot number)
    charger_positions:
      robot_1: [-20.0, -17.0, 0.0]     # [x, y, yaw_rad]
      robot_2: [-20.0, -14.0, 0.0]
      robot_3: [-20.0, -11.0, 0.0]
      robot_4: [-20.0,  -8.0, 0.0]
```

---

## 8. File-by-File Implementation Plan

### Step 1 — Package scaffolding

**Create:** `src/fms_robot_agent/CMakeLists.txt`  
**Create:** `src/fms_robot_agent/package.xml`

Key CMakeLists.txt structure:
```cmake
cmake_minimum_required(VERSION 3.16)
project(fms_robot_agent)

set(CMAKE_CXX_STANDARD 17)

# find_package(... all deps above ...)

add_library(robot_agent_lib
  src/battery_model.cpp
  src/bt_nodes/request_task.cpp
  src/bt_nodes/navigate_to_pose.cpp
  src/bt_nodes/execute_pick_drop.cpp
  src/bt_nodes/report_status.cpp
  src/bt_nodes/request_recovery.cpp
  src/bt_nodes/battery_ok.cpp
)
target_include_directories(robot_agent_lib PUBLIC include)
ament_target_dependencies(robot_agent_lib rclcpp rclcpp_lifecycle rclcpp_action
  behaviortree_cpp_v4 nav2_msgs geometry_msgs std_srvs fms_msgs)

add_executable(robot_agent_node src/robot_agent_node.cpp)
target_link_libraries(robot_agent_node robot_agent_lib)

install(TARGETS robot_agent_node robot_agent_lib
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config bt_xml scripts
  DESTINATION share/${PROJECT_NAME})
```

---

### Step 2 — `battery_model.cpp`

Simple stateful class. The `update()` method is called every 100 ms with the current motion state:

```
new_soc = soc ± (rate × dt)
clamped to [0.0, 100.0]
```

No external dependencies beyond `<algorithm>` and `<cmath>`.

---

### Step 3 — `robot_fsm.hpp`

Full `boost::sml` transition table inline in the header using lambdas. The FSM stores a shared state struct (`FsmContext`) containing the current task, interrupted task, and motion flags. This context is passed to the `sm<>` constructor so lambda actions can modify it.

```cpp
struct FsmContext {
  fms_msgs::msg::TaskAssignment::SharedPtr current_task;
  fms_msgs::msg::TaskAssignment::SharedPtr interrupted_task;
  bool is_moving{false};
  bool is_charging{false};
};

auto transition_table() {
  using namespace boost::sml;
  return make_transition_table(
    *state<Idle>       + event<TaskReceived>    / store_task   = state<Assigned>,
     state<Assigned>   + event<NavStarted>                     = state<Navigating>,
     state<Navigating> + event<NavSucceeded>                   = state<Executing>,
     state<Navigating> + event<NavFailed>       / log_fail     = state<Recovering>,
     state<Navigating> + event<FaultInjected>   / cancel_nav   = state<Recovering>,
     state<Executing>  + event<ExecutionDone>                  = state<Reporting>,
     state<Executing>  + event<NavFailed>                      = state<Recovering>,
     state<Reporting>  + event<ReportSent>      / clear_task   = state<Idle>,
     state<Recovering> + event<RecoveryDone>                   = state<Assigned>,
     state<Recovering> + event<RecoveryFailed>  / pub_failure  = state<Idle>,
    // LowBattery can interrupt any state (use wildcard)
     state<Idle>       + event<LowBattery>      / save_task    = state<Charging>,
     state<Assigned>   + event<LowBattery>      / save_task    = state<Charging>,
     state<Navigating> + event<LowBattery>      / save_task    = state<Charging>,
     state<Executing>  + event<LowBattery>      / save_task    = state<Charging>,
     state<Recovering> + event<LowBattery>      / save_task    = state<Charging>,
     state<Charging>   + event<BatteryFull>     / restore_task = state<Idle>
  );
}
```

---

### Step 4 — BT node implementations

Implement each BT node in its own `.cpp`. All nodes receive a shared `rclcpp::Node::SharedPtr` (the robot agent node passed via constructor) so they can call Nav2 actions and read the ROS clock.

**`navigate_to_pose.cpp` key detail:**

The Nav2 action client is created with the robot's namespace. The node's `robot_name_` is passed in, and the action name is constructed as `"/" + robot_name + "/navigate_to_pose"`. The node spins via a `rclcpp::executors::SingleThreadedExecutor` shared with the robot agent node — no separate executor thread.

**`request_recovery.cpp` key detail:**

Recovery sequence: spin 360° → backup 0.3 m → return SUCCESS. Each is a separate action call. A `recovery_attempts_` counter is tracked; if all 3 attempts fail, return FAILURE.

---

### Step 5 — `robot_agent_node.cpp`

Main file that:
1. Declares CLI argument `--ros-args -p robot_name:=robot_1`
2. Instantiates `RobotAgentNode` as a LifecycleNode
3. Registers all BT node types with the factory:
   ```cpp
   factory_.registerNodeType<RequestTask>("RequestTask", node_);
   factory_.registerNodeType<NavigateToPose>("NavigateToPose", node_);
   factory_.registerNodeType<ExecutePickDrop>("ExecutePickDrop");
   factory_.registerNodeType<ReportStatus>("ReportStatus", node_, completion_pub_);
   factory_.registerNodeType<RequestRecovery>("RequestRecovery", node_);
   factory_.registerNodeType<BatteryOK>("BatteryOK", blackboard_);
   factory_.registerNodeType<ChargeBattery>("ChargeBattery", blackboard_);
   ```
4. Loads `robot_agent.xml` from the package share path
5. Manages an rclcpp multi-threaded executor (callback groups for nav2 action callbacks and BT tick timer must not block each other)

---

### Step 6 — `robot_agent.launch.py`

Launches a single `robot_agent_node` in a given namespace:

```python
def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="robot_1"),
        Node(
            package="fms_robot_agent",
            executable="robot_agent_node",
            name="robot_agent",
            namespace=LaunchConfiguration("robot_name"),
            parameters=[
                {"robot_name": LaunchConfiguration("robot_name"),
                 "use_sim_time": True},
                os.path.join(share, "config", "robot_agent_params.yaml"),
            ],
            output="screen",
        ),
    ])
```

---

## 9. Launch System Updates

### `fms_navigation/launch/bringup.launch.py` — additions

After the existing Nav2 stagger (`t=10s` per robot), add a further stagger for robot agents. Nav2 must be active before the BT starts calling `navigate_to_pose`.

```
t=10s   robot_1: Nav2 stack starts
t=25s   robot_2: Nav2 stack starts
t=40s   robot_3: Nav2 stack starts
t=55s   robot_4: Nav2 stack starts
t=75s   robot_1: robot_agent node starts  (+5s after Nav2 fully active at ~70s)
t=80s   robot_2: robot_agent node starts
t=85s   robot_3: robot_agent node starts
t=90s   robot_4: robot_agent node starts
```

Add `num_robots` guard so agents are only launched for robots that are actually spawned.

Add a new `launch_robot_agent` boolean launch argument (default `true`) to allow running Phase 1 alone for debugging:
```bash
ros2 launch fms_navigation bringup.launch.py launch_agents:=false   # Phase 1 only
ros2 launch fms_navigation bringup.launch.py                         # Phase 1 + 2
```

### `fms_navigation/launch/bringup.launch.py` — new argument

```python
DeclareLaunchArgument("launch_agents", default_value="true",
                      description="Launch fms_robot_agent nodes (Phase 2)")
```

---

## 10. Fault Injection Design

### Service-based injection

Service: `/robot_N/inject_fault` (`std_srvs/srv/SetBool`)  
- `data=true` → cancel the active Nav2 goal → robot transitions to RECOVERING
- `data=false` → no-op (status query only)

### How cancellation flows

1. Fault service callback sets `fault_injected_ = true`
2. On next `bt_tick_callback()`, robot agent fires `FaultInjected` event into FSM
3. FSM transitions `NAVIGATING → RECOVERING`
4. BT `NavigateToPose::onHalted()` is called, which cancels the Nav2 action goal
5. BT re-enters the `RequestRecovery` fallback
6. Recovery runs spin + backup
7. On recovery success: FSM → `ASSIGNED`, BT retries `NavigateToPose`

### Test command

```bash
# Kill robot_2 mid-navigation
ros2 service call /robot_2/inject_fault std_srvs/srv/SetBool "{data: true}"

# Expected output in robot_2 agent terminal:
# [robot_2] FSM: NAVIGATING → RECOVERING
# [robot_2] BT: RequestRecovery — spinning...
# [robot_2] BT: RequestRecovery — backing up...
# [robot_2] FSM: RECOVERING → ASSIGNED
# [robot_2] BT: NavigateToPose — retrying...
```

### Process-kill recovery (Phase 2 stretch goal)

When `robot_N/robot_agent` process is killed:
- The `task_assignment` subscriber disappears
- No `task_completion` is published
- The pending TaskAssignment message is "lost" at the ROS transport layer

**Mitigation for Phase 2:** The `task_injector.py` script monitors for `task_completion` messages with a timeout. If no completion arrives within `deadline_secs`, it republishes the task. This simulates fleet-level reallocation without needing Phase 3's RabbitMQ dead-letter exchange.

**Full reallocation** (with fleet heartbeat) is a Phase 3 feature.

---

## 11. Task Injector Test Script

**File:** `src/fms_robot_agent/scripts/task_injector.py`

```
Usage:
  python3 task_injector.py                          # round-robin to 4 robots
  python3 task_injector.py --robot robot_1          # single robot
  python3 task_injector.py --count 10               # 10 tasks per robot
  python3 task_injector.py --pick 14.0 -17.0        # custom pick XY
  python3 task_injector.py --fault robot_2          # inject fault to robot_2
```

The script:
1. Publishes `TaskAssignment` messages to `/robot_N/task_assignment` topics
2. Subscribes to `/robot_N/task_completion` to track completions
3. Monitors `/robot_N/robot_status` to track battery SOC and FSM state
4. Prints a live summary table:

```
============================================================
  FMS Task Injector — 4 robots, 3 tasks pending
============================================================
  robot_1: NAVIGATING  SOC=87.3%  task=task_001  nav_to_pick
  robot_2: RECOVERING  SOC=72.1%  task=task_002  recovery_attempt=1/3
  robot_3: CHARGING    SOC=18.9%  task=-         (battery low)
  robot_4: EXECUTING   SOC=91.0%  task=task_003  pick_op
  Tasks completed: 7/10
============================================================
```

---

## 12. Test Scenarios

### Scenario A — Normal task lifecycle (smoke test)

```bash
# Terminal 1: full bringup
ros2 launch fms_navigation bringup.launch.py

# Terminal 2: inject 4 tasks (one per robot)
python3 task_injector.py --count 1

# Expected: all 4 robots complete IDLE→ASSIGNED→NAVIGATING→EXECUTING→REPORTING→IDLE
# Verification:
ros2 topic echo /robot_1/robot_status | grep state
ros2 topic echo /robot_1/task_completion
```

### Scenario B — Fault injection and recovery

```bash
# Terminal 1: full bringup + inject tasks
python3 task_injector.py --count 5

# Terminal 2: while robot_2 is NAVIGATING, inject fault
ros2 service call /robot_2/inject_fault std_srvs/srv/SetBool "{data: true}"

# Expected: robot_2 → RECOVERING → runs spin+backup → ASSIGNED → NAVIGATING (retry)
# Task eventually completes or 3 retries exhausted → FAILURE reported
```

### Scenario C — Battery low auto-charge

```bash
# Start robot with low battery
ros2 launch fms_robot_agent robot_agent.launch.py robot_name:=robot_1 initial_battery_soc:=25.0

# Expected: robot_1 immediately navigates to charger dock (-20, -17)
# On arrival: CHARGING state, SOC climbs at 1%/s
# At 95% SOC: exits CHARGING, returns to IDLE ready for tasks
```

### Scenario D — Process kill + task requeue

```bash
# Start injector with timeout monitoring
python3 task_injector.py --count 5 --deadline 120

# While robot_3 is mid-task, kill its agent
pkill -f "robot_agent_node.*robot_3"

# Expected: task_injector detects no completion within deadline
# Republishes task to a different robot (round-robin)
# Task eventually completes on alternate robot
```

### Scenario E — Multi-robot parallel tasks (milestone)

```bash
python3 task_injector.py --count 10  # 10 tasks distributed across 4 robots

# Expected: robots cycle through full state machine independently
# inject_fault one robot mid-way, verify others unaffected
# All 10 tasks eventually complete
```

---

## 13. Known Risks and Gotchas

### BT.CPP Version Mismatch

**Risk:** `behaviortree_cpp_v4` may not be in `apt` for Humble. If unavailable:
- Install from source: `https://github.com/BehaviorTree/BehaviorTree.CPP/tree/v4.x`
- Or use the v3 package (`behaviortree_cpp`) and adjust base class names:
  - v4: `StatefulActionNode` → v3: `AsyncActionNode` (different lifecycle callbacks)
  - v4: `onStart()/onRunning()/onHalted()` → v3: `tick()` with internal state machine

### BT tick thread vs ROS callbacks

**Risk:** BT custom nodes call `rclcpp_action::Client::send_goal_async()` which requires a running executor to deliver callbacks. If the BT tick timer fires in the same executor thread that handles action responses, there will be deadlock.

**Solution:** Use `rclcpp::CallbackGroup` separation:
- BT timer: `MutuallyExclusive` group A
- Nav2 action response callbacks: `MutuallyExclusive` group B
- Run `MultiThreadedExecutor` with at least 2 threads

### Nav2 goal preemption

**Risk:** If a new `NavigateToPose` goal is sent while a previous one is still active, Nav2 will preempt the old goal. When `inject_fault` triggers `onHalted()` and cancels the goal, the cancellation response arrives asynchronously. The BT tick may fire again before the cancel completes, sending a new goal immediately.

**Solution:** Add a `canceling_` flag in `NavigateToPose::onHalted()`. In `onRunning()`, check `canceling_` and return `RUNNING` until the cancel confirmation arrives.

### boost::sml and rclcpp LifecycleNode destructor order

**Risk:** `boost::sml::sm<>` destructor must fire before the `RobotAgentNode` destructor destroys publishers/subscribers. If the FSM destructor calls an action lambda that references a dead publisher, it will segfault.

**Solution:** In `on_cleanup()`, call `fsm_.reset()` before any publisher/subscriber destruction. Ensure the `sm<>` is stored as `std::unique_ptr` so it can be explicitly reset.

### Recovery action server timing

**Risk:** `/robot_N/spin` and `/robot_N/backup` action servers (from `nav2_behaviors`) must be active before `RequestRecovery` can call them. These are started by `lifecycle_manager_navigation`.

**Solution:** `RequestRecovery::onStart()` waits up to 5 seconds for the action server with `wait_for_action_server(timeout_sec=5.0)`. If unavailable, return `FAILURE` immediately (not RUNNING).

### DDS / CycloneDDS

Phase 2 adds more nodes and action clients per robot. The DDS participant count increases significantly. Keep `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` — do not switch to FastRTPS.

---

## 14. Build and Verification Commands

### Build

```bash
cd ~/fms_ws
colcon build --symlink-install --packages-select fms_robot_agent
source install/setup.bash
```

### Build all (Phase 1 + 2)

```bash
colcon build --symlink-install
source install/setup.bash
```

### Check BT.CPP availability

```bash
apt-cache show ros-humble-behaviortree-cpp-v4 2>/dev/null | grep Version \
  || echo "Not in apt — build from source"
```

### Verify node launches

```bash
# Single agent test (no Gazebo needed for basic lifecycle check)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run fms_robot_agent robot_agent_node \
  --ros-args -p robot_name:=robot_1 -p use_sim_time:=false

# Should see: [robot_agent] Configuring... [robot_agent] Activating...
# FSM starts in IDLE, BT starts ticking, RequestTask waits for assignment
```

### Full Phase 2 test

```bash
# Terminal 1: full simulation + navigation + agents
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash && source ~/fms_ws/install/setup.bash
ros2 launch fms_navigation bringup.launch.py

# Wait ~90 seconds for all agents to be active

# Terminal 2: inject tasks
python3 ~/fms_ws/install/fms_robot_agent/share/fms_robot_agent/scripts/task_injector.py --count 5

# Terminal 3: monitor status
ros2 topic echo /robot_1/robot_status
ros2 topic echo /robot_1/task_completion
```

### Milestone verification

```bash
# Inject 4 tasks, then fault robot_2 mid-task:
python3 task_injector.py --count 4 &
sleep 15
ros2 service call /robot_2/inject_fault std_srvs/srv/SetBool "{data: true}"

# Milestone achieved when:
# 1. robot_2 transitions NAVIGATING → RECOVERING (visible in /robot_2/robot_status)
# 2. robot_2 completes recovery and eventually reports task RESULT_FAILED or retries to SUCCESS
# 3. Other 3 robots are unaffected
# 4. All tasks have a TaskCompletion message published
```

---

*Phase 2 Plan generated: 2026-06-05*  
*Based on: Phase 1 (complete), PLAN.md, full codebase analysis*  
*Next: Phase 3 — gRPC fleet server + RabbitMQ + MongoDB*
