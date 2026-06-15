# Fleet gRPC Load Testing

This document describes how to run `load_test.py`
(`src/fms_fleet_server/scripts/load_test.py`), the load-test tool used to
verify Phase 3.6 (50-task milestone) and any future load runs against the
`FleetService` gRPC API exposed by `fleet_server_node`.

## What it does

`load_test.py` is a Python gRPC client that:

1. Polls `GetFleetStatus` to find robots in `IDLE` state.
2. Submits one `SubmitTask` (pick → drop) per poll cycle to an idle robot
   not already holding one of its own in-flight tasks.
3. Polls `GetTaskStatus` for every submitted task until it reaches a
   terminal state (`COMPLETED`, `FAILED`, or `CANCELLED`).
4. Repeats until `--num-tasks` tasks have been submitted and all of them
   are terminal, then prints a summary and the final fleet status.

It paces submissions to **at most one `SubmitTask` per poll cycle**. This
is required by current allocator behavior — see the docstring at the top of
`load_test.py` and Step 3.6 notes in `docs/PROJECT_STATUS.md` for why
bursting requests drops tasks.

## One-time setup: generate the gRPC stubs

The Python stubs (`fleet_pb2.py`, `fleet_pb2_grpc.py`) are generated from
`src/fms_fleet_server/proto/fleet.proto` and are **not** checked into git
(they're build artifacts that depend on the local `grpcio-tools` version).
Generate them once, and again whenever `fleet.proto` changes:

```bash
# Install dependencies (once per environment)
pip3 install --user grpcio grpcio-tools

# Generate fleet_pb2.py / fleet_pb2_grpc.py alongside load_test.py
cd /home/satyajit/fms_ws/src/fms_fleet_server/scripts
./generate_grpc_stubs.sh
```

This writes `fleet_pb2.py` and `fleet_pb2_grpc.py` into the `scripts/`
directory so `load_test.py` can import them directly.

## Prerequisites for running a load test

1. **MongoDB and RabbitMQ running**:
   ```bash
   systemctl is-active mongod
   systemctl is-active rabbitmq-server
   ```

2. **Full simulation + fleet stack up and stable**. From the workspace root:
   ```bash
   fms   # source the workspace (see FMS Workspace Sourcing memory)
   ros2 launch fms_navigation bringup.launch.py num_robots:=2
   ```
   Wait for **all lifecycle nodes to report active** before submitting any
   tasks — startup takes ~3-5 minutes. Tail the bringup log and confirm
   `"All lifecycle nodes active"` (or equivalent) appears, and that
   `ros2 topic pub --once /robot_N/robot_status ...` / `GetFleetStatus`
   shows robots in `IDLE` before proceeding.

   **Do not press Ctrl-C in the bringup terminal (Terminal-1) while a load
   test is running.** A SIGINT here kills the entire stack (gazebo, rviz,
   nav2, both `robot_agent_node`s), which leaves the robots looking
   "static/idle" with no running simulation behind them — this looks like a
   robot_agent or allocator bug but is just the bringup process having been
   interrupted. Check `~/.ros/log/<latest>/launch.log` for a
   `user interrupted with ctrl-c (SIGINT)` entry if robots stop responding
   mid-test.

3. **`fleet_server_node` running** (gRPC server on port 50051 by default):
   ```bash
   ros2 run fms_fleet_server fleet_server_node
   ```
   (Already started as part of `bringup.launch.py` in most configurations —
   confirm with `ros2 node list | grep fleet_server`.)

## Running the load test

```bash
cd /home/satyajit/fms_ws/src/fms_fleet_server/scripts
python3 load_test.py --num-tasks 50 --output /tmp/load_test_results.txt
```

Useful options:

| Flag | Default | Purpose |
|---|---|---|
| `--addr` | `localhost:50051` | `FleetService` gRPC endpoint |
| `--num-tasks` | `50` | Total pick→drop tasks to submit |
| `--poll-interval` | `2.0` | Seconds between status polls (also the max submission rate) |
| `--deadline-secs` | `300.0` | `deadline_secs` set on each `SubmitTaskRequest` |
| `--output` | none | File to write `task_id robot_id status` per task on completion |

Example — quick smoke test with 5 tasks:

```bash
python3 load_test.py --num-tasks 5 --poll-interval 1.0
```

The script runs in the foreground and streams progress; for long runs (50
tasks took ~2000s / ~33 min in the Step 3.6 run with 2 robots), launch it
with `nohup ... & disown` and tail the output, or run it under `Monitor`.

## Reading the results

- Console output prints each submission (`[elapsed] [n/total] submitted
  task_id=... -> robot_id`) and each terminal status
  (`task ... -> COMPLETED (result=0, duration=...)`).
- The final summary line: `DONE in <secs>s: N COMPLETED, M FAILED/CANCELLED
  out of <num-tasks> submitted`.
- Exit code is `0` only if all `--num-tasks` tasks were submitted and all
  completed with no `FAILED`/`CANCELLED`.
- If `--output` is given, each line is `task_id robot_id status` —
  cross-check against MongoDB:
  ```bash
  mongosh fms --eval 'db.tasks.find({task_id: "<id>"}, {status:1, result:1, duration_secs:1})'
  ```

## Known issue: allocator pacing (Phase 4 candidate)

`FleetServerNode::select_robot` scores robots from a cached
`/robot_status` snapshot that lags behind dispatch, and
`robot_agent_node` holds only a single `pending_task_` slot — a second
assignment arriving before the first is consumed is dropped
(`WARN: Dropping queued task`). `load_test.py`'s one-submission-per-poll
pacing works around this. A production fix (queue-depth-aware allocation
or a multi-task queue in `robot_agent_node`) is tracked as a Phase 4
candidate in `docs/PROJECT_STATUS.md`.
