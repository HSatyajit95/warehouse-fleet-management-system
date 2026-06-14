# FMS Project Status

## Phase 1 — Simulation Foundation: ✅ Complete & Verified
- Custom Gazebo Harmonic warehouse world (SDF) with shelving aisles, pick stations, charge docks
- Differential-drive robot URDF/Xacro (`fms_robot.urdf.xacro`) with 2D lidar + IMU
- Wheel joint axis fixed (`axis xyz="0 0 -1"`) — robot now moves the correct direction in Gazebo, matching RViz/Nav2
- Namespaced Nav2 bringup per robot (`/robot_N/...`), isolated costmaps and TF trees
- `odom_to_tf.py` publishes static `map → robot_N/odom` (from spawn pose) + dynamic `odom → base_footprint`
- Python launch scripts: `sim.launch.py` (spawn N robots), `bringup.launch.py` (full stack: RViz + Gazebo + Nav2 + agents, staggered startup)
- **Verified**: robot navigates correctly to map-frame goals (e.g., drop station `(10,-14)`), TF tree correct, odom @ ~20 Hz

## Phase 2 — Robot State Machine & BehaviorTree: ✅ Complete & Verified
- `RobotFSM` with all 7 states (`IDLE, ASSIGNED, NAVIGATING, EXECUTING, REPORTING, RECOVERING, CHARGING`) and transitions
- Custom BT nodes (BehaviorTree.CPP v3): `RequestTask`, `NavigateToPose`, `ExecutePickDrop`, `ReportStatus`, `RequestRecovery`, `BatteryOK`, `ChargeBattery`
- `robot_agent.xml` BT tree: `ChargeWhenLow` (auto-dock when SOC < 20%) + `TaskExecution` (pick → drop → report, with retry/recovery on nav failure)
- `BatteryModel`: drains while moving/idle, charges at dock, low/full thresholds
- Fault injection service (`/robot_N/inject_fault`) consumed by `NavigateToPoseBT` via `fault_injected_.exchange(false)`
- `task_injector.py`: publishes `TaskAssignment`, monitors `RobotStatus`/`TaskCompletion`, supports fault injection
- **Verified**: full pick→drop→report cycle completes (`FINAL: 1/1 tasks completed`), battery drains correctly while navigating, FSM transitions `IDLE → ASSIGNED → NAVIGATING → EXECUTING → IDLE`

### Remaining optional checks (not blockers)
- Multi-robot (`num_robots:=4`) staggered startup + concurrent task throughput
- Fault injection *during* `NAVIGATING` → confirm `RECOVERING` FSM transition (service call itself already verified)
- Low-battery → `CHARGING` → dock → `IDLE` cycle

---

## Phase 3 — Fleet Controller & Messaging: 🔲 Not Started

New package: `fms_fleet_server/` (C++, gRPC server)

| Task | Description |
|---|---|
| Proto definitions | gRPC service + messages for `RobotStatus`, `TaskAssignment`, `TaskCompletion` (mirroring existing `fms_msgs`) |
| gRPC fleet server | Central server robot agents connect to — replaces/augments direct ROS topic communication |
| Task allocator (BT) | Fleet-level BehaviorTree.CPP tree; scores robots by distance + battery SOC + queue depth, assigns tasks |
| RabbitMQ broker | Task queue with dead-letter exchange for failed/timed-out tasks |
| MongoDB integration | Collections: `tasks`, `telemetry`, `robots` — persistent fleet state (server itself stays stateless) |
| **Milestone** | Inject 50 tasks via REST → all allocated, executed by some robot, and logged in MongoDB |

---

## Phase 4 — REST API, Docker & CI: 🔲 Not Started

New package: `fms_api/` (Python, FastAPI) + repo-level Docker/CI config

| Task | Description |
|---|---|
| FastAPI endpoints | `POST /tasks`, `GET /tasks/{id}`, `GET /fleet/status`, `POST /robots/{id}/command` |
| Docker Compose | Services: `fleet-server`, `rabbitmq`, `mongodb`, `ros2-bridge` |
| gTest unit tests | FSM transitions, BT node logic, task allocator scoring |
| cTest integration tests | Full task lifecycle across gRPC + RabbitMQ + MongoDB |
| GitHub Actions CI | `build → lint → test → docker build`, green on every PR |
| **Milestone** | `docker compose up` launches the full stack; CI green on every PR |

---

## Suggested Next Step
Scaffold `fms_fleet_server`: package skeleton, proto definitions (reusing `fms_msgs` schema as the source of truth), and a minimal gRPC server that can receive `RobotStatus` updates from the robot agent(s) already running.
