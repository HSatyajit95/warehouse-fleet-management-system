# FMS Glossary — Key Terms & Concepts

Plain-language explanations of acronyms and technologies used across this
project, with emphasis on Phase 3 (since that's where most new concepts
appear). Each entry: **full form → what it is → why we use it here.**

---

## Already in use (Phase 1 & 2)

### ROS 2 (Robot Operating System 2)

A framework for writing robot software as a network of communicating
processes ("nodes"). Nodes exchange data via **topics** (continuous streams,
e.g. `/robot_1/odom`), **services** (request/response, e.g.
`/robot_1/inject_fault`), and **actions** (long-running goals with feedback,
e.g. `/robot_1/navigate_to_pose`). Everything in this project runs on ROS 2.

### Gazebo

The physics simulator. Simulates the robot's physical body, wheels, sensors
(lidar, IMU), and the warehouse environment. "Harmonic" is the version used.

### URDF / Xacro

**Unified Robot Description Format** — an XML format describing a robot's
links (rigid bodies), joints, sensors. **Xacro** is a macro language that
generates URDF (lets you use variables/loops instead of repeating XML).

### TF / TF2 (Transform library)

Tracks the position/orientation of every coordinate frame relative to every
other frame over time (e.g. "where is `robot_1/base_link` relative to
`map` right now?"). The `map → odom → base_footprint → base_link → ...`
chain you've seen is a TF tree.

### Nav2 (Navigation 2)

ROS 2's path-planning and obstacle-avoidance stack. Given a goal pose, it
plans a path (`planner_server`), drives along it while avoiding obstacles
(`controller_server`), and reports success/failure via the
`navigate_to_pose` action.

### Lifecycle Node

A ROS 2 node with explicit states (`unconfigured → inactive → active → finalized`). Lets you control startup order precisely (e.g. don't activate
Nav2 until the robot's odometry is ready). `RobotAgentNode` is one.

### FSM (Finite State Machine)

A model where a system is always in exactly one of a fixed set of states
(`IDLE`, `NAVIGATING`, `CHARGING`, ...) and moves between them based on
events. `RobotFSM` implements the 7-state robot lifecycle.

### BT (Behavior Tree) — specifically **BehaviorTree.CPP**

A way to structure decision-making as a tree of nodes (Sequence, Fallback,
Action, Condition) that gets "ticked" repeatedly. Compared to an FSM, a BT
is easier to extend (add a new branch without rewiring existing states) and
naturally supports retries/fallbacks. `robot_agent.xml` is the per-robot BT;
Phase 3 will add a *fleet-level* BT for task allocation.

### SOC (State of Charge)

Battery percentage (0–100%), e.g. `battery_soc: 82.85`. `BatteryModel`
simulates drain while moving/idle and recharge while docked. Used to decide
when a robot should go charge (`SOC < 20%` triggers `ChargeWhenLow`).

---

## New in Phase 3

### gRPC (g Remote Procedure Call)

A framework for calling functions on a *different process* (possibly on a
different machine) as if they were local function calls — "Remote Procedure
Call." You define the available functions and message formats in a
`.proto` file; gRPC generates client/server code in C++, Python, etc.

**Why here:** `fms_fleet_server` will expose gRPC functions like
`SubmitTask(...)` and `GetFleetStatus()`. External tools (the Phase 4 REST
API, a load-test script) call these functions directly instead of
publishing/subscribing to ROS topics — gRPC is the standard way to expose a
service to *non-ROS* clients.

### Protobuf (Protocol Buffers)

The data-format/serialization language gRPC uses. You write `.proto` files
defining message structures (similar to how `.msg` files define ROS
messages), and a compiler generates code to read/write that data
efficiently in any language.

**Why here:** `fleet.proto` will define `TaskRequest`, `FleetStatus`, etc.
— the gRPC equivalents of `fms_msgs/TaskAssignment`, `RobotStatus`.

### RabbitMQ

A **message broker** — a separate server that holds queues of messages
between producers and consumers, so they don't have to talk to each other
directly or be online at the same time. Uses the **AMQP** protocol
(Advanced Message Queuing Protocol).

**Why here:** when a task is submitted, instead of immediately trying to
allocate it (which could fail under load), it's placed on a `tasks.incoming`
queue. The allocator consumes from this queue at its own pace. If a task
fails repeatedly, RabbitMQ's **dead-letter exchange** (a special "failed
messages" queue) catches it for retry/inspection — this is what makes the
system resilient to a robot going offline mid-task.

### MongoDB

A **NoSQL document database** — stores data as JSON-like documents in
flexible "collections" (loosely like tables, but without a fixed schema).

**Why here:** persistent storage for things that need to survive a restart
or be queried later:

- `robots` collection — latest known status of each robot
- `telemetry` collection — history of status updates over time
- `tasks` collection — every task's full lifecycle (submitted → assigned →
  completed/failed, with timestamps)

The fleet server itself stays "stateless" (no in-memory-only state that
would be lost on restart) — all real state lives in MongoDB + RabbitMQ.

### Task Allocator / Queue Scoring

The logic that decides **which robot gets which task**. When a task needs a
robot, the allocator looks at all currently-`IDLE` robots and computes a
"score" for each based on:

- **distance** — how far is the robot from the pick-up location? (closer is
  better)
- **SOC** — how much battery does it have? (more is better — don't send a
  low-battery robot on a long trip)
- **queue depth** — how many tasks are already waiting for this robot?
  (fewer is better, for load balancing)

The robot with the best (lowest) combined score gets the task. This can
start as a simple weighted-sum formula and later be wrapped in a
fleet-level BT for more complex multi-step decisions (e.g. "if no robot
qualifies, requeue and wait").

### Dead-Letter Exchange

A RabbitMQ concept: when a message can't be processed successfully (e.g. a
task fails after retries), instead of disappearing it gets routed to a
separate "dead letter" queue, where it can be retried, logged, or inspected
manually — prevents silently losing failed work.

---

## Coming in Phase 4 (for reference)

### FastAPI

A Python web framework for building REST APIs (the kind of API a browser or
`curl`/Postman talks to via HTTP). `fms_api` will expose endpoints like
`POST /tasks` that internally call the Phase 3 gRPC fleet server.

### Docker / Docker Compose

**Docker** packages an application + all its dependencies into a portable
"container" that runs the same everywhere. **Docker Compose** runs multiple
containers together as one stack (e.g. fleet-server + RabbitMQ + MongoDB +
ros2-bridge, all started with one `docker compose up`).

### CI (Continuous Integration) / GitHub Actions

Automatically building, testing, and checking code every time it changes
(e.g. on every pull request), so problems are caught immediately rather than
discovered later. GitHub Actions is GitHub's built-in automation tool for
this.

### gTest / cTest

**gTest** (Google Test) — a C++ unit-testing framework (tests individual
functions/classes in isolation, e.g. "does `RobotFSM::process()` transition
correctly?"). **cTest** — CMake's test runner, used for broader
*integration* tests (e.g. "does a full task go through gRPC → RabbitMQ →
robot → MongoDB correctly end-to-end?").
