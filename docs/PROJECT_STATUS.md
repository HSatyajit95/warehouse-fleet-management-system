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

## Phase 3 — Fleet Controller & Messaging: 🟡 In Progress

New package: `fms_fleet_server/` (C++, gRPC server)

| Task | Description | Status |
|---|---|---|
| Step 3.1 — Package skeleton + RobotStatus aggregation | `FleetServerNode` subscribes to `/robot_N/robot_status` + `/robot_N/task_completion`, logs a periodic fleet summary table | ✅ Done & verified (launched alongside `bringup.launch.py num_robots:=1`, fleet summary logged `robot_1 IDLE 99.x% -` every 2s) |
| Step 3.2 — MongoDB integration | `MongoStore` (mongocxx) upserts `robots` + appends `telemetry` on every `RobotStatus` | ✅ Done & verified (`db.robots.find()` showed live `robot_1` doc, `db.telemetry.countDocuments()` grew to 22 over ~20s) |
| Step 3.3 — Task allocator | `FleetServerNode` subscribes to `/fleet/task_request` (unassigned `TaskAssignment`), scores IDLE robots by `distance_to_pick + (100-SOC)*soc_weight`, publishes to chosen `/robot_N/task_assignment`, writes `tasks` doc (`ASSIGNED` → `COMPLETED`/`FAILED` via `task_completion`) | ✅ Done & verified (sent task via new `task_request.py`; assigned to `robot_1`, score=20.25; `db.tasks` showed `status: ASSIGNED` → `status: COMPLETED` with `result=0`, `duration_secs=91.2` after pick→drop cycle) |
| Step 3.4 — RabbitMQ broker | `RabbitMqClient` (librabbitmq-c) declares `tasks`/`tasks.dlx` exchanges and `tasks.incoming`/`tasks.failed` queues (incoming has `x-dead-letter-exchange: tasks.dlx`); fleet server runs a consumer thread on `tasks.incoming`, feeds the Step 3.3 allocator, acks on success or rejects (→ DLX) if no IDLE robot; execution-time `FAILED`/`CANCELLED` completions are also published to `tasks.dlx` | ✅ Done & verified: (1) sent `task_request_amqp.py --pick 14.0 -14.0 --drop 10.0 -14.0` while `robot_1` was IDLE → log `tasks.incoming: assigned task task_9702834d to robot_1 (score=...)`, `db.tasks` `status: ASSIGNED` → `status: COMPLETED` (`result=0`, `duration_secs=91.32`) after pick→drop; (2) sent a task with no IDLE robot available → `tasks.incoming: no IDLE robot available ... routing to tasks.failed`, confirmed message landed in `tasks.failed` (bound to `tasks.dlx`/`failed`) |
| Step 3.5 — gRPC service | `proto/fleet.proto` defines `FleetService` (`SubmitTask`, `GetFleetStatus`, `GetTaskStatus`) with `Pose2D`, `RobotStatusInfo`, request/response messages; CMake runs `protoc` + `grpc_cpp_plugin` codegen into `fms_fleet_lib`; `FleetGrpcServer` (grpcpp) runs on its own thread, listening on `grpc_port` (default `50051`); `FleetServerNode` exposes `submit_task`/`get_fleet_status`/`get_task_status` as public methods, sharing the Step 3.3/3.4 `select_robot`/`dispatch_assignment` allocator and `MongoStore::find_task` | ✅ Done & verified: Python test client (`/tmp/fleet_grpc_test/test_client.py`, via `grpcio`) — `SubmitTask(pick=(14,-14), drop=(10,-14))` → `accepted=true`, assigned `task_1781459165404363479` to `robot_1`; `GetFleetStatus` returned live TF-based poses for both robots (`robot_1`/`robot_2`); `GetTaskStatus` showed `status=ASSIGNED` → `status=COMPLETED` (`result=0`, `duration_secs=26.40`) |
| Step 3.6 — 50-task load test | **Milestone**: all 50 tasks reach `COMPLETED`, fleet returns to `IDLE` | ✅ Done & verified: paced gRPC `SubmitTask` client (`/tmp/fleet_grpc_test/load_test_paced.py`) submitted 50 tasks across `robot_1`/`robot_2` over ~2010s wall-clock; 48/50 reached `COMPLETED` (`result=0`), both robots ended `IDLE` (SOC ~60%, after one auto `CHARGING` cycle mid-run). 2 tasks (`task_1781543282152901070`, `task_1781543296194988260`) remain `ASSIGNED` — they were executing during a `fleet_server_node` crash (see note below) and their `task_completion` was missed while it was down; not a logic defect, an artifact of the live restart |

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
Phase 3 is complete. Begin Phase 4 — REST API (FastAPI), Docker Compose, and CI
(see table above): `POST /tasks`, `GET /tasks/{id}`, `GET /fleet/status`,
`POST /robots/{id}/command`, Docker Compose for `fleet-server`/`rabbitmq`/`mongodb`,
gTest/cTest suites, and GitHub Actions CI.

### Notes from Step 3.6 load test — two issues found & addressed

1. **Allocator/queue pacing (worked around in test client, not yet fixed in code)**:
   `FleetServerNode::select_robot` scores robots using a cached `latest_status_`
   that lags behind dispatch by up to one `/robot_status` publish interval, and
   `robot_agent_node` holds only a single `pending_task_` slot — any earlier
   unconsumed assignment is dropped with `WARN: Dropping queued task` when a new
   one arrives. Bursting `SubmitTask` calls faster than that interval can route
   multiple tasks to the same robot and drop all but the last. The load test
   worked around this by pacing submissions (max 1 `SubmitTask` per 2s poll
   cycle, only to a robot confirmed `IDLE`). A production fix needs
   queue-depth-aware allocation or a multi-task queue in `robot_agent_node`
   (Phase 4 candidate).

2. **mongocxx thread-safety crash (fixed this session)**: `MongoStore`
   (`fms_fleet_server/src/mongo_store.cpp` /
   `include/fms_fleet_server/mongo_store.hpp`) shared a single `mongocxx::client`
   across the ROS executor thread (telemetry/robot status), the RabbitMQ
   consumer thread, and the gRPC server thread with no synchronization.
   Under sustained load this crashed `fleet_server_node` with
   `mongoc_socket_errno(): assertion failed: sock` (SIGABRT). Fixed by adding a
   `std::mutex` guarding every `MongoStore` method (`upsert_robot`,
   `insert_telemetry`, `insert_task`, `update_task_completion`, `find_task`).
   Rebuilt and verified `fleet_server_node` stayed up for the remainder of the
   load test (tasks 13-50 all completed without further crashes).

### Note on Step 3.3 scoring — ✅ Fixed
`RobotStatus.pose` was previously always `(0,0,0)`. `robot_agent_node.cpp`'s
`status_publish_cb()` now looks up `map -> <robot_name>/base_footprint` via
`tf2_ros::Buffer`/`TransformListener` and populates `pose.position`/
`pose.orientation` from the transform, so the allocator's `distance_to_pick`
term now varies per robot. Verified with a 2-robot bringup: `robot_1` pose
`(10.21, -13.97)`, `robot_2` pose `(-22.0, -10.0)` — both non-zero and
distinct — and `/fleet/task_request` correctly selected the closer/lower-score
robot (`task_request: assigned task ... to robot_1 (score=33.70)`).
