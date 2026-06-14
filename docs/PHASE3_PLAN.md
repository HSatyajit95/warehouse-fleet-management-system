# Phase 3 Plan — Fleet Controller & Messaging

Goal (from `PLAN.md`): central fleet server that allocates tasks across all
robots, backed by RabbitMQ (queueing) and MongoDB (persistence), exposed via
gRPC. Milestone: inject 50 tasks → all allocated, executed, and logged.

New package: **`fms_fleet_server/`** (C++17, ament_cmake)

---

## 0. Design decision: how the fleet server talks to robots

Robot agents (Phase 2) already work over **ROS 2 topics**
(`/robot_N/task_assignment`, `/robot_N/robot_status`, `/robot_N/task_completion`)
— this is verified and working. Rewriting robot agents to speak gRPC directly
would be a large, risky change to code that already works.

**Recommended approach** — `fms_fleet_server` is a ROS 2 node that:
- Subscribes to `/robot_N/robot_status` and `/robot_N/task_completion` for all N
- Publishes `/robot_N/task_assignment` when it allocates a task
- Internally persists state to MongoDB and queues work via RabbitMQ
- Exposes a **gRPC service** as its external API (this is what `fms_api` /
  Phase 4 and the 50-task injector will call — `SubmitTask`, `GetFleetStatus`,
  `GetTaskStatus`)

This keeps Phase 2 untouched, satisfies "robots communicate via the fleet
layer, not direct DB access" (robots never see Mongo/RabbitMQ — only the
fleet server does), and gRPC becomes the *external* boundary as PLAN.md's
architecture diagram shows (`FastAPI → Fleet Server (gRPC)`).

---

## 1. Package scaffold

```
fms_fleet_server/
├── CMakeLists.txt
├── package.xml
├── proto/
│   └── fleet.proto              # SubmitTask, GetFleetStatus, GetTaskStatus, streaming status
├── include/fms_fleet_server/
│   ├── fleet_server_node.hpp    # ROS node: subs/pubs + owns allocator, mongo, rabbitmq clients
│   ├── task_allocator.hpp       # BT-based scoring allocator
│   ├── mongo_store.hpp          # mongocxx wrapper: tasks/telemetry/robots collections
│   ├── rabbitmq_client.hpp      # AMQP-CPP or RabbitMQ-C wrapper
│   └── grpc_service.hpp         # FleetService gRPC server impl
├── src/
│   ├── fleet_server_main.cpp
│   ├── fleet_server_node.cpp
│   ├── task_allocator.cpp
│   ├── mongo_store.cpp
│   ├── rabbitmq_client.cpp
│   └── grpc_service.cpp
├── bt_xml/
│   └── allocator.xml            # fleet-level BT: score robots, pick best
├── launch/
│   └── fleet_server.launch.py
└── config/
    └── fleet_server_params.yaml # mongo URI, rabbitmq URI, grpc port, scoring weights
```

---

## 2. Implementation order (incremental milestones)

### Step 3.1 — Package skeleton + RobotStatus aggregation (no external deps)
- Create package, depend on `fms_msgs`, `rclcpp`
- `FleetServerNode` subscribes to `/robot_N/robot_status` for N=1..num_robots
  (parameter), keeps an in-memory map `robot_id → latest RobotStatus`
- Log a periodic fleet summary table (sanity check before adding DB/MQ)
- **Verify**: launch alongside existing `bringup.launch.py`, confirm fleet
  server logs all 4 robots' states/SOC in real time

### Step 3.2 — MongoDB integration
- Add `mongocxx`/`bsoncxx` dependency (system package or vcpkg)
- `MongoStore` with 3 collections: `robots` (latest status, upsert),
  `telemetry` (time-series RobotStatus history), `tasks` (task lifecycle)
- Every `RobotStatus` → upsert `robots`, insert `telemetry`
- **Verify**: `mongosh fms; db.robots.find()` shows live robot states

### Step 3.3 — Task allocator (BT-based scoring)
- `bt_xml/allocator.xml`: given an incoming task + current fleet snapshot,
  score each IDLE robot by `distance_to_pick + (100 - SOC)*weight + queue_depth*weight`
  and pick the lowest-score robot
- `TaskAllocator` class wraps a small BT.CPP tree (or simple scoring function
  if BT feels like overkill for v1 — note for later refinement)
- On allocation: publish `TaskAssignment` to `/robot_N/task_assignment`,
  write `tasks` doc with status `ASSIGNED`
- Subscribe to `/robot_N/task_completion` → update `tasks` doc to
  `COMPLETED`/`FAILED`
- **Verify**: publish a `TaskAssignment` directly to the fleet server's gRPC
  (stubbed) or a temp ROS topic, confirm it lands on the correct robot and
  the `tasks` collection updates through to completion

### Step 3.4 — RabbitMQ broker
- New task requests go onto a `tasks.incoming` queue (not directly to
  allocator) — decouples request rate from allocation rate
- Allocator consumes from `tasks.incoming`; on Nav2/recovery failure
  (`TaskCompletion.result == RESULT_FAILED`), requeue via a
  `tasks.dead_letter` exchange with retry count
- **Verify**: `rabbitmqctl list_queues`, inject a task that fails (use
  `/robot_N/inject_fault`), confirm it lands in `tasks.dead_letter` and
  retries

### Step 3.5 — gRPC service (external API)
- `fleet.proto`: `SubmitTask(TaskRequest) → TaskAck`,
  `GetFleetStatus(Empty) → FleetStatus` (all robots' RobotStatus),
  `GetTaskStatus(TaskId) → TaskRecord`
- `SubmitTask` pushes onto `tasks.incoming` (Step 3.4)
- **Verify**: `grpcurl` or a small Python gRPC client submits tasks and
  polls status

### Step 3.6 — 50-task load test
- Script (Python, gRPC client) submits 50 tasks with randomized pick/drop
  across 4 robots
- **Milestone check**: all 50 reach `COMPLETED` in `tasks` collection,
  `GetFleetStatus` shows all robots back to `IDLE`

---

## 3. Open questions to confirm before starting

1. **MongoDB/RabbitMQ install** — run as local services (`apt`/`docker run`
   single containers) now, formalize into Docker Compose in Phase 4? (Recommended:
   yes, local services now, compose later — avoids blocking on Docker setup.)
2. **gRPC/Protobuf toolchain** — `grpc` + `protobuf` via apt
   (`libgrpc++-dev`, `protobuf-compiler-grpc`) or vcpkg? (Recommended: apt,
   matches existing toolchain simplicity.)
3. **Scoring weights** for the allocator — start with simple
   `distance + (100-SOC)` and tune later, or build the full BT.CPP tree from
   day 1? (Recommended: simple scoring function first; wrap in BT once the
   allocation logic itself is proven — avoids debugging two new things at once.)

---

## 4. Definition of done (Phase 3)

- [ ] `fms_fleet_server` builds via `colcon build`
- [ ] Fleet server tracks all robots' live status in MongoDB
- [ ] Tasks submitted via gRPC are queued (RabbitMQ), allocated to the best
      IDLE robot, executed by the existing Phase 2 agent, and logged through
      to completion in MongoDB
- [ ] Failed tasks (via fault injection) are retried via dead-letter queue
- [ ] 50-task load test: all 50 complete, fleet returns to idle
