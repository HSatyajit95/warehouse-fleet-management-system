# FMS Command Reference — Every Terminal, Every Feature

A single, copy-pasteable reference for exercising every feature of this
project, end to end: build → simulate → inject a task → watch the fleet
server allocate it → query it over REST → tear down. Commands are grouped
by **which terminal** they belong in for a live multi-terminal test
session (§0–§3), followed by standalone reference sections for the
supporting servers and tooling — MongoDB (§4), RabbitMQ (§5), Docker
(§6), automated tests (§7), and ROS introspection (§8–§9).

All commands assume you're in `~/fms_ws` unless noted. Every new terminal
needs ROS + the workspace sourced first:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
# or, if you've added the alias from docs/PROJECT_STATUS.md / memory:
fms
```

---

## 0. One-time setup (build + prerequisite services)

```bash
# Build everything
cd ~/fms_ws
colcon build --symlink-install

# Build with tests enabled (needed before `colcon test` works)
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON

# Build just one package (faster iteration)
colcon build --packages-select fms_fleet_server
colcon build --packages-select fms_robot_agent

source install/setup.bash
```

Phase 3+ features (fleet server, REST API) need MongoDB and RabbitMQ
running first — see `docs/PHASE3_PREREQUISITES.md` for one-time install,
or §4/§5 below for the full command set. Day-to-day, just make sure
they're up:

```bash
# MongoDB (this project runs it as a Docker container named fms-mongo)
docker ps --filter name=fms-mongo
docker start fms-mongo          # if not running

# RabbitMQ (native systemd service)
systemctl is-active rabbitmq-server
sudo systemctl start rabbitmq-server   # if not active
```

---

## 1. Single-robot simulation (Phase 1/2 — no fleet server)

**Terminal 1 — bring up Gazebo + Nav2 + the robot agent:**
```bash
fms
ros2 launch fms_navigation bringup.launch.py num_robots:=1
```
Useful variants:
```bash
ros2 launch fms_navigation bringup.launch.py num_robots:=4                # 4 robots
ros2 launch fms_navigation bringup.launch.py launch_agents:=false         # Nav2 only, no FSM/BT agent
ros2 launch fms_navigation bringup.launch.py use_rviz:=false headless:=true  # no GUI (e.g. over SSH)
```

**Terminal 2 — send the robot a pick→drop task directly (bypasses the fleet server entirely):**
```bash
fms
cd ~/fms_ws
python3 src/fms_robot_agent/scripts/task_injector.py --count 1 --robots 1
```
Other `task_injector.py` patterns:
```bash
python3 src/fms_robot_agent/scripts/task_injector.py                      # 1 task round-robined to 4 robots
python3 src/fms_robot_agent/scripts/task_injector.py --count 5            # 5 tasks per robot
python3 src/fms_robot_agent/scripts/task_injector.py --robots 1 2         # only robot_1 and robot_2
python3 src/fms_robot_agent/scripts/task_injector.py --pick 14.0 -5.0 --drop -5.0 -5.0 --robots 1
python3 src/fms_robot_agent/scripts/task_injector.py --fault robot_2      # inject a fault mid-task (tests RECOVERING state)
```

**Terminal 3 — watch it happen:**
```bash
fms
ros2 topic echo /robot_1/robot_status      # state, battery_soc, current_task_id live
ros2 topic echo /robot_1/task_completion   # fires once, when the task finishes
ros2 topic hz /robot_1/odom                # confirm odometry is publishing (~20Hz)
```

**Terminal 4 (optional) — manually drive Nav2 / inject a fault / inspect TF:**
```bash
fms
# Send a raw Nav2 goal, bypassing the task system entirely
ros2 action send_goal /robot_1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 10.0, y: -14.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback

# Manually inject/clear a fault (true = inject, false = clear)
ros2 service call /robot_1/inject_fault std_srvs/srv/SetBool "{data: true}"

# Dump the TF tree to a PDF
ros2 run tf2_tools view_frames
```

---

## 2. Fleet server (Phase 3 — gRPC + RabbitMQ + MongoDB, no REST yet)

**Terminal 1 — simulation (same as above, e.g. 2 robots):**
```bash
fms
ros2 launch fms_navigation bringup.launch.py num_robots:=2
```

**Terminal 2 — the fleet server:**
```bash
fms
ros2 run fms_fleet_server fleet_server_node
# or, to override defaults:
ros2 run fms_fleet_server fleet_server_node --ros-args \
  -p num_robots:=2 -p grpc_port:=50051 -p mongo_db:=fms \
  -p rabbitmq_host:=localhost -p soc_weight:=0.5
# or via the launch file:
ros2 launch fms_fleet_server fleet_server.launch.py num_robots:=2
```
Confirm it's up:
```bash
ros2 node list | grep fleet_server
ss -tln | grep 50051            # gRPC port listening
```

**Terminal 3 — submit a task via the gRPC-backed scripts (this is what `fms_api`/the real fleet manager would call under the hood):**
```bash
fms
cd ~/fms_ws/src/fms_robot_agent/scripts

# Via the ROS-topic-based request (fleet server's /fleet/task_request subscriber)
python3 task_request.py --pick 14.0 -14.0 --drop 10.0 -14.0
python3 task_request.py --pick 14.0 -14.0 --drop 10.0 -14.0 --count 3 --interval 5

# Via RabbitMQ's tasks.incoming queue (AMQP path)
python3 task_request_amqp.py --pick 14.0 -14.0 --drop 10.0 -14.0
```

**Terminal 4 — gRPC test client (talks directly to `FleetService`, same protocol `fms_api` uses):**
```bash
cd ~/fms_ws/src/fms_fleet_server/scripts
./generate_grpc_stubs.sh        # one-time, regenerate after any fleet.proto change
python3 load_test.py --num-tasks 5 --poll-interval 1.0          # small smoke test
python3 load_test.py --num-tasks 50 --output /tmp/load_test_results.txt   # full load test
```

**Terminal 5 — watch the allocation happen:**
```bash
fms
ros2 topic echo /robot_1/task_assignment    # fires once per task assigned to robot_1
ros2 topic echo /fleet/task_request         # raw unassigned task requests
```

---

## 3. REST API (Phase 4 — `fms_api`)

**Terminals 1 & 2 — same as Section 2** (simulation + `fleet_server_node` running).

**Terminal 3 — the REST API:**
```bash
cd ~/fms_ws/src/fms_api
python3 -m venv .venv && .venv/bin/pip install -r requirements-dev.txt   # one-time
.venv/bin/python ./generate_grpc.sh    # regenerate stubs after any fleet.proto change

# Dev mode — auto-reload on code changes, binds to localhost only
PYTHONPATH= .venv/bin/python -m uvicorn fms_api.main:app --reload --port 8000

# "Production-like" — no reload, listens on all interfaces, multiple workers
PYTHONPATH= .venv/bin/python -m uvicorn fms_api.main:app --host 0.0.0.0 --port 8000 --workers 2

# Point it at a fleet server on a different host/port (default localhost:50051)
FLEET_SERVER_ADDR=192.168.1.50:50051 PYTHONPATH= .venv/bin/python -m uvicorn fms_api.main:app --port 8000
```
Stop it: `Ctrl+C` in that terminal, or `pkill -f "uvicorn fms_api.main:app"` from another one.

**Terminal 4 — exercise every endpoint with `curl`:**
```bash
# Fleet-wide status
curl -s localhost:8000/fleet/status | python3 -m json.tool

# Submit a task (pick -> drop)
curl -s -X POST localhost:8000/tasks \
  -H "Content-Type: application/json" \
  -d '{"pick_pose":{"x":14.0,"y":-14.0},"drop_pose":{"x":10.0,"y":-14.0}}' \
  | python3 -m json.tool

# Poll a task's status (use the task_id from the response above)
curl -s localhost:8000/tasks/<task_id> | python3 -m json.tool

# Task not found -> expect HTTP 404
curl -s -w "\nHTTP %{http_code}\n" localhost:8000/tasks/does-not-exist

# Send a robot command (only "inject_fault" is currently supported)
curl -s -X POST localhost:8000/robots/robot_1/command \
  -H "Content-Type: application/json" \
  -d '{"command":"inject_fault","value":true}'

# Clear the fault
curl -s -X POST localhost:8000/robots/robot_1/command \
  -H "Content-Type: application/json" \
  -d '{"command":"inject_fault","value":false}'

# Unknown robot / unsupported command -> expect HTTP 400
curl -s -w "\nHTTP %{http_code}\n" -X POST localhost:8000/robots/robot_9/command \
  -H "Content-Type: application/json" -d '{"command":"inject_fault"}'
curl -s -w "\nHTTP %{http_code}\n" -X POST localhost:8000/robots/robot_1/command \
  -H "Content-Type: application/json" -d '{"command":"dance"}'

# fleet server unreachable (stop fleet_server_node first to see this) -> expect HTTP 502
curl -s -w "\nHTTP %{http_code}\n" localhost:8000/fleet/status

# Interactive API docs / raw OpenAPI schema (open in a browser, or fetch directly)
xdg-open http://localhost:8000/docs
curl -s localhost:8000/openapi.json | python3 -m json.tool
```

---

## 4. MongoDB — server commands

This project's MongoDB has always been run as a single Docker container
named `fms-mongo` (not a native `mongod` install — see
`docs/PHASE3_PREREQUISITES.md`). Either start that same container, or use
the native commands if you've installed `mongodb-org` via apt instead.

```bash
# Create it for the first time (one-time; persists via the container, not a volume)
docker run -d --name fms-mongo -p 27017:27017 --restart unless-stopped mongo:7

# Day-to-day
docker ps --filter name=fms-mongo          # check it's running
docker start fms-mongo
docker stop fms-mongo
docker restart fms-mongo
docker logs -f fms-mongo                   # tail its logs
docker rm -f fms-mongo                     # delete the container entirely (data lost)

# Health check
docker exec fms-mongo mongosh --quiet --eval "db.runCommand({ping:1})"
mongosh --eval "db.runCommand({ping:1})"   # if mongosh is installed on the host directly
```

If you installed MongoDB natively instead (apt `mongodb-org`):
```bash
sudo systemctl start mongod
sudo systemctl stop mongod
sudo systemctl status mongod
sudo systemctl enable --now mongod   # start now + on every boot
```

**Querying/managing data** (works against either — `mongosh` connects to
`localhost:27017` either way):
```bash
mongosh                                    # interactive shell, then `use fms`
mongosh fms --eval "show collections"
mongosh fms --eval "db.robots.find().pretty()"
mongosh fms --eval "db.tasks.find().sort({_id:-1}).limit(5).pretty()"
mongosh fms --eval 'db.tasks.find({task_id: "<id>"}, {status:1, result:1, duration_secs:1})'
mongosh fms --eval "db.telemetry.countDocuments()"
mongosh fms --eval "db.tasks.countDocuments({status: 'COMPLETED'})"
mongosh --eval "show dbs"                                   # every database, e.g. fms, fms_integration_test
mongosh fms_integration_test --eval "db.dropDatabase()"      # clean up Step 4.5's test DB
```

---

## 5. RabbitMQ — server commands

Native systemd service (this project's default):
```bash
sudo systemctl start rabbitmq-server
sudo systemctl stop rabbitmq-server
sudo systemctl restart rabbitmq-server
sudo systemctl status rabbitmq-server --no-pager
systemctl is-active rabbitmq-server
journalctl -xeu rabbitmq-server.service --no-pager   # debug a failed start
```

Docker alternative (e.g. if you don't want a native install — note this
is what `docker/docker-compose.yml`'s `rabbitmq` service uses):
```bash
docker run -d --name fms-rabbitmq -p 5672:5672 -p 15672:15672 \
  --restart unless-stopped rabbitmq:3.9-management
docker logs -f fms-rabbitmq
docker stop fms-rabbitmq && docker start fms-rabbitmq
```

**Inspecting queues/exchanges:**
```bash
sudo rabbitmqctl status
sudo rabbitmqctl list_queues name messages consumers
sudo rabbitmqctl list_exchanges
sudo rabbitmqctl list_bindings
sudo rabbitmqctl list_consumers
sudo rabbitmqctl purge_queue tasks.incoming      # empty a queue (careful — drops messages)
sudo rabbitmqctl ping                            # quick liveness check
# Docker-container equivalent (prefix with docker exec):
docker exec fms-rabbitmq rabbitmq-diagnostics -q ping
docker exec fms-rabbitmq rabbitmqctl list_queues
```

Management UI (enabled via the `-management` image / the
`rabbitmq_management` plugin): `http://localhost:15672` (guest/guest) —
shows queues, exchanges, bindings, message rates graphically.

```bash
sudo rabbitmq-plugins enable rabbitmq_management   # one-time, if using the non-management image/native install
```

---

## 6. Launching everything via Docker

### 6a. Full stack via Docker Compose (recommended)

Stop any natively-running Mongo/RabbitMQ first — they share the same
host ports as the compose services:
```bash
docker stop fms-mongo
sudo systemctl stop rabbitmq-server
```

```bash
cd ~/fms_ws
docker compose -f docker/docker-compose.yml build                  # build images (first run: ~10-15min, mongocxx from source)
docker compose -f docker/docker-compose.yml build fleet-server      # rebuild just one image
docker compose -f docker/docker-compose.yml up -d                  # start mongodb, rabbitmq, fleet-server, fms-api
docker compose -f docker/docker-compose.yml ps                     # check status
docker compose -f docker/docker-compose.yml logs -f                # tail every service's logs
docker compose -f docker/docker-compose.yml logs -f fleet-server   # tail just one service
docker compose -f docker/docker-compose.yml restart fms-api        # restart one service (e.g. after an env var change)
docker compose -f docker/docker-compose.yml exec fleet-server bash # shell into a running container
docker compose -f docker/docker-compose.yml down                   # stop & remove containers (keeps the mongo_data volume)
docker compose -f docker/docker-compose.yml down -v                # ...also delete the mongo_data volume
```
With it up, the same `curl` commands from §3 work unchanged (`fms-api` is
on `network_mode: host`, port 8000). RabbitMQ's management UI:
`http://localhost:15672` (guest/guest).

`NUM_ROBOTS` (how many robots `fleet-server` tracks) is read from
`docker/.env` (copy `docker/.env.example` there first) or set inline:
```bash
NUM_ROBOTS=2 docker compose -f docker/docker-compose.yml up -d
```

Restore your native services afterward:
```bash
docker start fms-mongo
sudo systemctl start rabbitmq-server
```

### 6b. Running individual images without Compose

Useful for testing one image in isolation. Requires `mongodb`/`rabbitmq`
already reachable at `localhost` — easiest with `--network host`:
```bash
cd ~/fms_ws
docker build -f docker/fleet-server.Dockerfile -t fms-fleet-server .
docker run --rm --network host \
  -e MONGO_URI=mongodb://localhost:27017 -e RABBITMQ_HOST=localhost \
  -e NUM_ROBOTS=2 fms-fleet-server

docker build -f docker/fms-api.Dockerfile -t fms-api .
docker run --rm --network host \
  -e FLEET_SERVER_ADDR=localhost:50051 fms-api
```

### 6c. General Docker housekeeping

```bash
docker ps                       # every running container
docker ps -a                    # every container, including stopped
docker images | grep fms        # this project's built images
docker stats                    # live CPU/mem usage per container
docker system df                # disk space used by images/containers/volumes
docker compose -f docker/docker-compose.yml down --rmi local   # also remove the built images
```

---

## 7. Automated tests & linting

```bash
cd ~/fms_ws

# C++ unit + integration tests (RobotFSM, BatteryModel, task allocator, full task lifecycle)
colcon test --packages-select fms_fleet_server fms_robot_agent
colcon test-result --verbose          # non-zero exit if anything failed

# Run one gtest binary directly for full per-case output
./build/fms_robot_agent/test_robot_fsm
./build/fms_robot_agent/test_battery_model
./build/fms_fleet_server/test_task_allocator

# clang-tidy (same check set as CI, see .clang-tidy)
for f in src/fms_robot_agent/src/*.cpp src/fms_robot_agent/src/bt_nodes/*.cpp; do
  clang-tidy -p build/fms_robot_agent "$f"
done
for f in src/fms_fleet_server/src/*.cpp; do
  clang-tidy -p build/fms_fleet_server "$f"
done

# fms_api: pytest + ruff
cd src/fms_api
PYTHONPATH= .venv/bin/python -m pytest tests/ -v
.venv/bin/ruff check fms_api tests
```

---

## 8. Topic / Service / Action / Node introspection

General-purpose discovery commands (works for anything in the system,
not just what's listed below):

```bash
ros2 node list                          # every running node
ros2 node info /fleet_server            # a node's pubs/subs/services/actions

ros2 topic list                         # every active topic
ros2 topic list -t                      # ...with its message type next to it
ros2 topic type /robot_1/robot_status   # just the type
ros2 topic info /robot_1/robot_status -v   # type + QoS + publisher/subscriber count
ros2 topic hz /robot_1/odom             # publish rate
ros2 topic bw /robot_1/scan             # bandwidth
ros2 topic echo /robot_1/robot_status   # live message stream
ros2 topic echo /robot_1/robot_status --once   # just one message, then exit
ros2 topic echo /robot_1/robot_status --field battery_soc   # just one field

ros2 interface show fms_msgs/msg/RobotStatus      # full field definition of any type
ros2 interface show fms_msgs/msg/TaskAssignment
ros2 interface show fms_msgs/msg/TaskCompletion
ros2 interface list | grep fms_msgs               # confirm the package's types are registered

ros2 service list                       # every active service
ros2 service type /robot_1/inject_fault
ros2 service call /robot_1/inject_fault std_srvs/srv/SetBool "{data: true}"

ros2 action list                        # every active action
ros2 action list -t                     # ...with type
ros2 action info /robot_1/navigate_to_pose
```

### Topic reference table (what exists once `bringup.launch.py` + `fleet_server_node` are both running)

| Topic | Type | What it carries |
|---|---|---|
| `/robot_N/robot_status` | `fms_msgs/msg/RobotStatus` | `header`, `robot_id`, `state` (0=IDLE,1=ASSIGNED,2=NAVIGATING,3=EXECUTING,4=REPORTING,5=RECOVERING,6=CHARGING), `pose`, `battery_soc` (float, %), `current_task_id`, `status_message`. Published every 500ms by the robot agent. |
| `/robot_N/task_assignment` | `fms_msgs/msg/TaskAssignment` | `header`, `task_id`, `robot_id`, `task_type` (0=PICK,1=DROP,2=CHARGE), `pick_pose`, `drop_pose`, `payload_id`, `priority`, `deadline_secs`. Fleet server → robot, one message per assigned task. |
| `/robot_N/task_completion` | `fms_msgs/msg/TaskCompletion` | `header`, `task_id`, `robot_id`, `result` (0=SUCCESS,1=FAILED,2=CANCELLED), `duration_secs`, `error_message`. Robot → fleet server, fires once per finished task. |
| `/fleet/task_request` | `fms_msgs/msg/TaskAssignment` (unassigned, `robot_id` empty) | Alternate task-submission entry point — the fleet server's allocator subscribes here directly (no RabbitMQ involved on this path). |
| `/robot_N/odom` | `nav_msgs/msg/Odometry` | Wheel odometry from the Gazebo diff-drive plugin — `pose.pose.position/orientation`, `twist.twist.linear/angular`. ~20Hz. |
| `/robot_N/scan` | `sensor_msgs/msg/LaserScan` | Simulated 2D lidar — 360° FOV, range 0.12–12.0m, 10Hz. Feeds Nav2's costmaps + AMCL + SLAM Toolbox. |
| `/robot_N/imu` | `sensor_msgs/msg/Imu` | Simulated IMU — `angular_velocity`, `linear_acceleration`, `orientation`. 100Hz. (Not currently fused into odometry.) |
| `/robot_N/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2 controller's output velocity command to the diff-drive plugin. |
| `/robot_N/plan` | `nav_msgs/msg/Path` | The planner's current global path (list of poses). |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | The full transform tree (`map → robot_N/odom → robot_N/base_footprint → robot_N/base_link → ...`). |
| `/robot_N/joint_states` | `sensor_msgs/msg/JointState` | Wheel joint positions/velocities, for `robot_state_publisher`. |

### Service / Action reference

| Interface | Name | Type |
|---|---|---|
| Service | `/robot_N/inject_fault` | `std_srvs/srv/SetBool` — `data: true` injects a fault (forces `RECOVERING`), `data: false` clears it |
| Action | `/robot_N/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` |
| Action | `/robot_N/spin` | `nav2_msgs/action/Spin` (Nav2 recovery behavior) |
| Action | `/robot_N/backup` | `nav2_msgs/action/BackUp` (Nav2 recovery behavior) |
| Action | `/robot_N/follow_waypoints` | `nav2_msgs/action/FollowWaypoints` |
| gRPC RPC | `SubmitTask` / `GetFleetStatus` / `GetTaskStatus` / `SendRobotCommand` | defined in `src/fms_fleet_server/proto/fleet.proto` — not a ROS interface, reachable via `fms_api` (REST) or any gRPC client on port 50051 |

---

## 9. Lifecycle node introspection (Nav2 + robot agent)

```bash
ros2 lifecycle list /robot_1/robot_agent
ros2 lifecycle get /robot_1/robot_agent
ros2 lifecycle set /robot_1/robot_agent configure
ros2 lifecycle set /robot_1/robot_agent activate

# Same pattern for every Nav2 lifecycle node:
for node in map_server controller_server planner_server behavior_server bt_navigator; do
  ros2 lifecycle get /robot_1/$node
done
```
