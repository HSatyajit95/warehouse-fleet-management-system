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
| 4 | REST API, Docker Compose, CI | 🔲 Not Started |

See [docs/PROJECT_STATUS.md](docs/PROJECT_STATUS.md) for details and
[docs/PLAN.md](docs/PLAN.md) for the full project plan.

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
  `GetFleetStatus`, `GetTaskStatus`), RabbitMQ task queue with dead-letter
  exchange, MongoDB-backed task/telemetry persistence, and a distance+SOC
  based task allocator

## Prerequisites

- Ubuntu 22.04, ROS 2 Humble, Gazebo Harmonic
- Phase 3 additionally requires gRPC, RabbitMQ, and MongoDB — see
  [docs/PHASE3_PREREQUISITES.md](docs/PHASE3_PREREQUISITES.md)

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
