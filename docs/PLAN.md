# Multi-Robot Fleet Management System (FMS) — Project Plan

## Overview
ROS 2 Humble simulation of a multi-robot fleet management system coordinating 4 differential-drive AMRs in a custom Gazebo warehouse environment. Mirrors real-world FMS/WCS architecture.

## Tech Stack
- Simulation: ROS 2 Humble, Gazebo Harmonic, ros_gz_sim, Nav2, SLAM Toolbox
- Robot logic: BehaviorTree.CPP v4, boost::sml (FSM), rclcpp lifecycle nodes, C++17
- Fleet middleware: gRPC + Protobuf, RabbitMQ (AMQP), MongoDB 7 + mongocxx, FastAPI (Python 3.11)
- DevOps: Docker Compose v2, GitHub Actions, CMake + CPack, gTest/cTest, clang-tidy

## Architecture
External clients (REST) → FastAPI → Fleet Server (gRPC) ↔ RabbitMQ broker ↔ Robot agents (×4)
                                         ↓
                                      MongoDB
Robot agents ↔ Nav2 action server ↔ Gazebo simulation

## Key Design Decisions
- Robot agents communicate ONLY via gRPC to fleet server. No direct DB access from robots.
- RabbitMQ with dead-letter exchange handles failed/timed-out tasks automatically.
- All robots fully namespaced: /robot_1/*, /robot_2/*, etc. Isolated costmaps and TF trees.
- Fleet server is stateless — state lives in MongoDB + RabbitMQ. Horizontally scalable.
- Use rclcpp::LifecycleNode for all robot agent nodes.

## Robot State Machine (per robot)
IDLE → ASSIGNED → NAVIGATING → EXECUTING → REPORTING → IDLE
         ↓                          ↓
      RECOVERING ←————————— (Nav2 failure / timeout)
         ↓
      CHARGING (SOC below threshold)

## Build Phases

### Phase 1 — Simulation foundation (Weeks 1–2)
- Custom Gazebo warehouse world (SDF): shelving aisles, pick stations, charge docks
- Differential-drive robot URDF/Xacro with 2D lidar + IMU
- Namespaced Nav2 bringup per robot (/robot_N/nav2)
- Python launch script to spawn N robots at unique poses
- Milestone: 4 robots navigating independently to waypoints

### Phase 2 — Robot state machine & BT (Weeks 3–4)
- Per-robot FSM (7 states above)
- BehaviorTree.CPP custom nodes: RequestTask, NavigateToPose, ExecutePickDrop, ReportStatus, RequestRecovery
- Fault injection and auto-recovery on Nav2 failures
- Simulated battery model with auto-dock-to-charger behavior
- Milestone: Kill any robot mid-task; fleet recovers and reallocates

### Phase 3 — Fleet controller & messaging (Weeks 5–6)
- gRPC fleet server with proto definitions for RobotStatus, TaskAssignment, TaskCompletion
- Fleet-level BehaviorTree.CPP task allocator (score by distance + SOC + queue depth)
- RabbitMQ task broker with dead-letter exchange
- MongoDB collections: tasks, telemetry, robots
- Milestone: 50 tasks injected via REST; all allocated, executed, logged

### Phase 4 — REST API, Docker & CI (Weeks 7–8)
- FastAPI endpoints: POST /tasks, GET /tasks/{id}, GET /fleet/status, POST /robots/{id}/command
- Docker Compose: fleet-server, rabbitmq, mongodb, ros2-bridge services
- gTest unit tests: FSM transitions, BT node logic, task scorer
- cTest integration tests: full task lifecycle across gRPC + broker + DB
- GitHub Actions CI: build → lint → test → Docker build
- Milestone: docker compose up launches full stack; CI green on every PR

## Workspace Structure (target)
fms_ws/
├── src/
│   ├── fms_robot_agent/       # Per-robot ROS2 lifecycle node + FSM + BT
│   ├── fms_fleet_server/      # gRPC fleet server + task allocator BT
│   ├── fms_msgs/              # Custom ROS2 msgs + proto definitions
│   ├── fms_gazebo/            # World SDF, robot URDF, launch files
│   ├── fms_navigation/        # Nav2 config, SLAM params, BT nav plugins
│   └── fms_api/               # FastAPI REST server (Python)
├── docker/
│   └── docker-compose.yml
├── .github/
│   └── workflows/ci.yml
└── PLAN.md                    # This file

## Build Command
cd ~/fms_ws
colcon build --symlink-install

## Constraints
- Build tool: colcon only
- ROS 2 distro: Humble
- No physical hardware — simulation only (Gazebo)
- Workspace: ~/fms_ws
