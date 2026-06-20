# ROS 2 Warehouse Fleet Management System (FMS)

A multi-robot warehouse simulation built on **ROS 2 Humble** and
**Gazebo Harmonic**. Each robot runs an autonomous pick-and-drop task
cycle using a Finite State Machine (FSM) and a BehaviorTree.CPP-based
agent on top of Nav2. A fleet-level controller coordinates task
allocation across the fleet via gRPC, RabbitMQ, and MongoDB.

## Project Status

| Phase | Description | Status |
|---|---|---|
| 1 | Simulation foundation — Gazebo warehouse world, robot URDF, Nav2 bringup, TF | ✅ Complete & Verified |
| 2 | Robot agent — FSM + BehaviorTree (pick → drop → report), battery model, fault injection | ✅ Complete & Verified |
| 3 | Fleet controller — gRPC server, RabbitMQ task queue, MongoDB persistence, task allocator | ✅ Complete & Verified |
| 4 | REST API, Docker Compose, CI | ✅ Complete & Verified |

See [docs/PROJECT_STATUS.md](docs/PROJECT_STATUS.md) for details,
[docs/PLAN.md](docs/PLAN.md) for the full project plan, and
[docs/PHASE4_PLAN.md](docs/PHASE4_PLAN.md) /
[docs/PHASE4_PROGRESS.md](docs/PHASE4_PROGRESS.md) for the current phase.

## Packages

- **`fms_msgs`** — custom ROS 2 message/service definitions (`RobotStatus`,
  `TaskAssignment`, `TaskCompletion`)
- **`fms_gazebo`** — Gazebo Harmonic warehouse world, robot URDF/Xacro, spawn
  and simulation launch files
- **`fms_navigation`** — Nav2 bringup, behavior trees, maps, per-robot
  navigation config
- **`fms_robot_agent`** — robot-side FSM and BehaviorTree.CPP agent, battery
  model, task injector script
- **`fms_fleet_server`** — fleet controller: gRPC `FleetService` (`SubmitTask`,
  `GetFleetStatus`, `GetTaskStatus`, `SendRobotCommand`), RabbitMQ task queue
  with dead-letter exchange, MongoDB-backed task/telemetry persistence, and a
  distance+SOC based task allocator
- **`fms_api`** — FastAPI REST front-end proxying the fleet server's gRPC API
  (`POST /tasks`, `GET /tasks/{id}`, `GET /fleet/status`,
  `POST /robots/{id}/command`); see [src/fms_api/README.md](src/fms_api/README.md)

## Prerequisites

- Ubuntu 22.04, ROS 2 Humble, Gazebo Harmonic
- Phase 3 additionally requires gRPC, RabbitMQ, and MongoDB — see
  [docs/PHASE3_PREREQUISITES.md](docs/PHASE3_PREREQUISITES.md)
- Phase 4's server-side stack (`fleet-server`, `rabbitmq`, `mongodb`,
  `fms-api`) can run natively (as above) or via
  `docker compose -f docker/docker-compose.yml up` — see
  [docs/PHASE4_PROGRESS.md](docs/PHASE4_PROGRESS.md) for details (the two
  approaches share host ports and aren't meant to run simultaneously)

## Build

```bash
colcon build --symlink-install
source install/setup.bash
```

> Tip: add an alias (e.g. `fms`) that sources both
> `/opt/ros/humble/setup.bash` and this workspace's `install/setup.bash` —
> required in every new terminal for `fms_msgs`-dependent commands.

## Running the simulation

```bash
ros2 launch fms_navigation bringup.launch.py num_robots:=1
```

Then inject a task:

```bash
python3 src/fms_robot_agent/scripts/task_injector.py --count 1 --robots 1
```

See [docs/VERIFICATION_CHECKLIST.md](docs/VERIFICATION_CHECKLIST.md) for a
full terminal-by-terminal walkthrough, and
[docs/GLOSSARY.md](docs/GLOSSARY.md) for explanations of the key concepts
and technologies used across the project.

## Fleet server & load testing

```bash
ros2 run fms_fleet_server fleet_server_node
```

Exposes the `FleetService` gRPC API (default port `50051`) for submitting
pick→drop tasks and querying fleet/task status. See
[docs/LOAD_TESTING.md](docs/LOAD_TESTING.md) for the paced gRPC load-test
tool (`src/fms_fleet_server/scripts/load_test.py`) and full setup
instructions.

## Documentation

All design docs, plans, and verification guides live in [docs/](docs/).
For a single copy-pasteable reference covering every terminal command in
this project — build, simulate, inject tasks, query the fleet/REST API,
run tests, and inspect every topic/service/action/database — see
[docs/COMMAND_REFERENCE.md](docs/COMMAND_REFERENCE.md). For the system
design and sim-to-real deployment blueprint, see
[docs/SYSTEM_ARCHITECTURE.md](docs/SYSTEM_ARCHITECTURE.md).
