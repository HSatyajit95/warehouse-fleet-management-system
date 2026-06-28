# Phase 4 Progress — REST API, Docker & CI

Tracks implementation against [`PHASE4_PLAN.md`](PHASE4_PLAN.md). Update
this table as each step lands; see `PROJECT_STATUS.md` for the
all-phases summary (update that too once Phase 4 finishes).

New package: `fms_api/` (Python, FastAPI, plain `pip` project — not
colcon-wired, per Open Question #1 in the plan).

---

## Status table

| Step | Description | Status |
|---|---|---|
| 4.1 — `fms_api` skeleton + `GET /fleet/status` | FastAPI app proxying `GetFleetStatus` gRPC | ✅ Done & verified |
| 4.2 — Task endpoints | `POST /tasks`, `GET /tasks/{id}` | ✅ Done & verified |
| 4.3 — Robot command endpoint | `POST /robots/{id}/command` (+ `fleet.proto` addition) | ✅ Done & verified |
| 4.4 — gTest unit tests | `RobotFSM` transitions, `BatteryModel`, task allocator scoring | ✅ Done & verified |
| 4.5 — cTest integration test | Full task lifecycle, gRPC→allocator→Mongo, no Gazebo | ✅ Done & verified |
| 4.6 — Dockerize | `docker-compose.yml`: fleet-server, rabbitmq, mongodb, fms_api | ✅ Done & verified |
| 4.7 — GitHub Actions CI | build → lint → test → docker build | ✅ Done & verified locally (commands tested; CI run pending push) |

---

## Step 4.1 — `fms_api` skeleton + `GET /fleet/status`

**What was built** (`src/fms_api/`):
- `fms_api/generated/` — gRPC stubs generated from
  `fms_fleet_server/proto/fleet.proto` via `grpc_tools.protoc`
  (regenerate with `./generate_grpc.sh` after any `.proto` change; the
  script also patches protoc's absolute `import fleet_pb2` into a
  package-relative import)
- `fms_api/grpc_client.py` — `FleetClient` wrapping the `FleetServiceStub`;
  address configurable via `FLEET_SERVER_ADDR` env var (default
  `localhost:50051`)
- `fms_api/schemas.py` — pydantic response models (`Pose2D`, `RobotStatus`,
  `FleetStatusResponse`)
- `fms_api/main.py` — FastAPI app; `GET /fleet/status` calls
  `FleetClient.get_fleet_status()` and maps the protobuf response to JSON;
  returns HTTP 502 if the gRPC call fails (fleet server down/unreachable)
- `tests/test_routes.py` — 2 pytest cases, gRPC client mocked via
  `monkeypatch`: happy path (robot list mapped correctly) and the 502
  error path

**Dependency isolation note:** `fastapi`/`uvicorn`/`pytest` were first
installed with `pip install --user`, which bumped the system `packaging`
package to a version incompatible with `streamlit` (a pre-existing
unrelated tool) — caught and fixed by re-pinning `packaging<26`.
Switched to a project-local venv (`src/fms_api/.venv`) for all further
work to avoid further global side effects. Note: this machine's
`PYTHONPATH` is set by the `fms`/ROS environment sourcing and leaks into
any venv created from a shell with ROS sourced — run tests with
`PYTHONPATH=` cleared, e.g.:

```bash
cd src/fms_api
PYTHONPATH= .venv/bin/python -m pytest tests/ -v
```

**Verification:**
- ✅ Unit tests: `PYTHONPATH= .venv/bin/python -m pytest tests/ -v` → 2/2 passed
- ✅ Live verification (2026-06-20): MongoDB (`fms-mongo` Docker container)
  was already running; RabbitMQ needed a one-time host fix — its log
  directory `/var/log/rabbitmq` was missing, causing
  `systemd[...]: Failed to set up standard output: No such file or directory`
  on every start attempt. Fixed with:
  ```bash
  sudo mkdir -p /var/log/rabbitmq
  sudo chown rabbitmq:rabbitmq /var/log/rabbitmq
  sudo systemctl start rabbitmq-server
  ```
  With both up, ran `fleet_server_node` (`num_robots:=0`, no simulation
  needed) and `uvicorn fms_api.main:app --port 8000`:
  `curl localhost:8000/fleet/status` → `{"robots":[]}` (HTTP 200), proxied
  correctly end-to-end through `fms_api` → gRPC → `fleet_server_node`.

---

## Step 4.2 — Task endpoints

**What was built** (`src/fms_api/`):
- `fms_api/grpc_client.py` — added `FleetClient.submit_task(...)` (builds a
  `SubmitTaskRequest` from pick/drop poses + optional fields) and
  `FleetClient.get_task_status(task_id)`
- `fms_api/schemas.py` — added `TaskCreateRequest`/`TaskCreateResponse`
  (mirrors `SubmitTaskRequest`/`SubmitTaskResponse`) and
  `TaskStatusResponse` (mirrors `GetTaskStatusResponse` minus the `found`
  flag, which becomes an HTTP 404 instead)
- `fms_api/main.py`:
  - `POST /tasks` — proxies `SubmitTask`; 502 if the fleet server is
    unreachable (gRPC-level failure is distinct from "no robot available,"
    which is a normal `accepted=false` response, not an error)
  - `GET /tasks/{task_id}` — proxies `GetTaskStatus`; maps
    `found=false` → HTTP 404
- `tests/test_routes.py` — 5 new pytest cases: task creation happy path
  (asserts the exact gRPC call args), 502 on gRPC error, task lookup happy
  path, 404 on not-found, 502 on gRPC error

**Verification:**
- ✅ Unit tests: 7/7 passed (`PYTHONPATH= .venv/bin/python -m pytest tests/ -v`)
- ✅ `ruff check fms_api tests` → all checks passed
- ✅ Live verification (2026-06-20): with `fleet_server_node
  num_robots:=0` (no IDLE robot, no Gazebo needed) + `uvicorn` running —
  `POST /tasks` with a pick/drop pose → `{"accepted":false,"task_id":"",
  "robot_id":"","message":"No IDLE robot available"}` (matches
  `FleetServerNode::submit_task`'s synchronous no-candidate path);
  `GET /tasks/nonexistent_id` → HTTP 404. Full assign→complete flow
  (`accepted=true`) needs a live robot agent + Gazebo, not yet exercised
  through `fms_api` — same as Phase 3's gRPC test client, this was
  previously verified through the gRPC layer directly; deferred to Step
  4.5's integration test rather than re-doing it manually here.

---

## Step 4.3 — Robot command endpoint

Only `inject_fault` exists today on the robot side (Phase 2's
`/<robot_id>/inject_fault` `std_srvs/SetBool` service) — no `cancel_task`
or similar. Scoped this step to what's actually implemented rather than
stub out commands the robot agent can't act on yet.

**What was built:**
- `fms_fleet_server/proto/fleet.proto` — added `SendRobotCommand(SendRobotCommandRequest) returns (SendRobotCommandResponse)`; request is `{robot_id, command, value}` (only `command == "inject_fault"` is handled; `value` maps to `SetBool.data`)
- `fms_fleet_server` (C++):
  - `package.xml`/`CMakeLists.txt` — added `std_srvs` dependency
  - `fleet_server_node.hpp/.cpp` — one `rclcpp::Client<std_srvs::srv::SetBool>` per robot (`fault_clients_`, created alongside the existing per-robot publishers/subscriptions), plus `send_robot_command(robot_id, command, value)`: rejects unknown commands and unknown `robot_id`s, returns a clear message if the robot's service isn't ready or doesn't respond within 2s, otherwise forwards the `SetBool` call and relays its `success`/`message`
  - `fleet_grpc_server.hpp/.cpp` — `SendRobotCommand` override, thin passthrough to `FleetServerNode::send_robot_command`
  - Rebuilt: `colcon build --packages-select fms_fleet_server`
- `fms_api`:
  - `generate_grpc.sh` re-run to pick up the new RPC in the Python stubs
  - `grpc_client.py` — `FleetClient.send_robot_command(robot_id, command, value=True)`
  - `schemas.py` — `RobotCommandRequest`/`RobotCommandResponse`
  - `main.py` — `POST /robots/{robot_id}/command`; 400 if the fleet server reports `success=false` (bad robot_id/command, or robot unreachable), 502 if the gRPC call itself fails
  - `tests/test_routes.py` — 3 new pytest cases: happy path (asserts exact gRPC args), 400 on `success=false`, 502 on gRPC error

**Verification:**
- ✅ Unit tests: 10/10 passed
- ✅ `ruff check fms_api tests` → all checks passed
- ✅ Live verification (2026-06-20): ran `robot_agent_node` standalone
  (`-p robot_name:=robot_1 -r __ns:=/robot_1`, no Gazebo/Nav2 needed —
  the node self-configures/activates and `on_activate` doesn't block on
  Nav2 action servers) + `fleet_server_node num_robots:=1` + `uvicorn`:
  - `POST /robots/robot_1/command {"command":"inject_fault","value":true}`
    → `{"success":true,"message":"Fault injected"}` (HTTP 200); robot
    agent log confirmed `[robot_1] Fault injected via service.`
  - `POST /robots/robot_9/command ...` (unknown robot) →
    `{"detail":"unknown robot_id: robot_9"}` (HTTP 400)
  - `POST /robots/robot_1/command {"command":"dance"}` (unsupported) →
    `{"detail":"unsupported command: dance"}` (HTTP 400)

---

## Step 4.4 — gTest unit tests

Scoped to logic that's actually unit-testable in isolation — pure
functions/classes with no ROS node, Mongo, RabbitMQ, or gRPC dependency.
Both targeted classes already had no such dependency, or were refactored
to remove one:

**`fms_fleet_server` — task allocator scoring:**
- `FleetServerNode::select_robot` (private, scoring logic inlined) would
  have required constructing the *entire* `FleetServerNode` — including
  live Mongo/RabbitMQ/gRPC connections — just to unit-test a distance+SOC
  formula. Extracted the pure scoring logic into a free function:
  - `include/fms_fleet_server/task_allocator.hpp` /
    `src/task_allocator.cpp` — `select_best_robot(robots, request,
    soc_weight)`, depends only on `fms_msgs` types
  - `fleet_server_node.cpp`'s `select_robot` now just locks
    `fleet_mutex_` and delegates to `select_best_robot` — behavior
    unchanged, now testable
  - `test/test_task_allocator.cpp` — 6 cases: no robots tracked, no robot
    IDLE, closer-robot-wins (equal SOC), higher-SOC-wins (equal
    distance), non-IDLE robots ignored even if closer, exact score
    formula check
  - `CMakeLists.txt`/`package.xml` — added `std_srvs` (already needed for
    4.3) and `ament_cmake_gtest` test target

**`fms_robot_agent` — `RobotFSM` and `BatteryModel`:**
- Both were already pure, dependency-free classes — no refactor needed
- `test/test_robot_fsm.cpp` — 11 cases covering every transition in
  `robot_fsm.cpp`'s table: the full two-leg pick→drop happy path,
  `NAV_FAILED`/`FAULT_INJECTED` → `RECOVERING`, `RECOVERY_DONE` →
  `ASSIGNED`, `RECOVERY_FAILED` → `IDLE`, `LOW_BATTERY` → `CHARGING`,
  `BATTERY_FULL` → `IDLE`, an invalid transition (ignored, returns
  `false`), and the `NAVIGATING` + `NAV_STARTED` self-loop (matches but
  reports no state change)
- `test/test_battery_model.cpp` — 9 cases: initial SOC clamping, drain
  rate (moving > idle), charging, 0/100 clamping under sustained
  drain/charge, `is_low()`/`is_full()` threshold boundaries, `set_soc`
  clamping
- `CMakeLists.txt`/`package.xml` — added `ament_cmake_gtest` test targets,
  linked against the existing `fms_agent_lib`

**Caught while writing tests:** my first draft of the `RobotFSM` happy-path
test assumed `EXECUTING --REPORT_SENT--> IDLE` was reachable directly
after one navigation leg; running it failed because `NAVIGATING` has no
`REPORT_SENT` transition. The actual flow has *two* `NAVIGATING`/
`EXECUTING` legs per task (pick, then drop) before `REPORT_SENT` — fixed
the test to match the real transition table (not a bug in `robot_fsm.cpp`
itself, a bug in my test's assumption).

**Verification:**
- ✅ `colcon build --cmake-args -DBUILD_TESTING=ON` for both packages
- ✅ `colcon test --packages-select fms_fleet_server fms_robot_agent` +
  `colcon test-result --verbose` → 0 errors, 0 failures
- ✅ Ran each gtest binary directly for full per-case output:
  `test_task_allocator` (6/6), `test_robot_fsm` (11/11),
  `test_battery_model` (9/9)

**Not covered (scope decision):** BT action/condition nodes
(`RequestTask`, `NavigateToPoseBT`, `BatteryOK`, etc.) wrap
`RobotAgentNode` (a `rclcpp_lifecycle::LifecycleNode` with Nav2 action
clients) — unit-testing them directly would need `rclcpp::init` +
constructing/configuring a full lifecycle node per test, for nodes that
mostly just delegate to `RobotFSM`/`BatteryModel`. Testing those two
pure-logic classes thoroughly (above) covers the actual decision logic;
revisit BT-node-level tests only if a bug shows up there specifically
that FSM/battery tests wouldn't catch.

---

## Step 4.5 — cTest integration test

Per Open Question #3 in `PHASE4_PLAN.md` (resolved "yes"): exercises the
real fleet server + MongoDB, stubs out the robot/Gazebo side with a
lightweight mock instead of running the full simulation.

**Correction to the original plan's framing**: `fleet_server_node`'s gRPC
`SubmitTask` handler calls `select_robot`/`dispatch_assignment` directly —
it does **not** route through RabbitMQ's `tasks.incoming` queue (only the
`/fleet/task_request` ROS topic and `scripts/task_request_amqp.py` path
do). So this test exercises gRPC → allocator → MongoDB; RabbitMQ only
needs to be *up* (a hard requirement for `fleet_server_node` to start at
all) without being functionally exercised by this particular flow. The
AMQP `tasks.incoming` path was already verified manually in Phase 3
(Step 3.4) and isn't duplicated here.

**What was built** (`fms_fleet_server/test/integration/`):
- `mock_robot.py` — minimal `rclpy` node standing in for
  `robot_agent_node`: publishes `RobotStatus` (`STATE_IDLE`, SOC 95%)
  every 0.5s so the allocator has a candidate, and on receiving a
  `TaskAssignment`, waits `exec_duration` seconds (via a one-shot
  `create_timer`, not a blocking `sleep`, so the status timer keeps
  running) then publishes `TaskCompletion` with `RESULT_SUCCESS`. No
  Nav2/BT/Gazebo involved.
- `test_task_lifecycle.py` — the ctest-registered orchestrator:
  1. Checks MongoDB (27017) and RabbitMQ (5672) are reachable; if either
     isn't, prints `SKIP: ...` and exits 99 (mapped to ctest's
     `***Skipped` via `SKIP_RETURN_CODE`) rather than failing — CI
     environments may not always have these up
  2. Generates fresh gRPC stubs from `fleet.proto` into a temp dir (same
     approach as `scripts/generate_grpc_stubs.sh`, just self-contained so
     the test has no manual pre-step)
  3. Starts `fleet_server_node` (`num_robots:=1`, dedicated
     `grpc_port:=50061` and `mongo_db:=fms_integration_test` so it can't
     collide with a fleet server a developer already has running on the
     default port/db) and `mock_robot.py` as subprocesses, logging each
     to a temp file
  4. Waits for the gRPC port to open, then `SubmitTask` → asserts
     `accepted=true` and `robot_id=robot_1`; polls `GetTaskStatus` until
     terminal, asserts `status=="COMPLETED"` and `result==0`
     (`RESULT_SUCCESS`)
  5. Tears down both subprocesses (SIGINT, dumping each one's log tail
     for diagnostics) regardless of outcome
- `CMakeLists.txt` — `add_test(NAME task_lifecycle_integration ...)`
  alongside the `test_task_allocator` gtest target, with `TIMEOUT 60` and
  `SKIP_RETURN_CODE 99`

**Caught and fixed while testing:** first run produced a passing test but
a noisy traceback in the mock robot's log — `rclpy.shutdown()` was being
called a second time after `rclpy.spin()` was interrupted by `SIGINT`,
which already triggers an internal shutdown. Fixed by guarding with
`if rclpy.ok(): rclpy.shutdown()`.

**Verification:**
- ✅ `ctest --test-dir build/fms_fleet_server -R task_lifecycle_integration -V`
  → `SubmitTask accepted: ... robot_id=robot_1`, `PASS: task ... reached
  COMPLETED via robot_1 in 0.50s`; fleet server log confirms
  `grpc SubmitTask: assigned task ... to robot_1 (score=...)` and
  `task_completion: ... result=SUCCESS`
- ✅ SKIP path verified live: stopped RabbitMQ
  (`sudo systemctl stop rabbitmq-server`) → re-ran the test → ctest
  reported `***Skipped` (not a failure) with `SKIP: RabbitMQ not
  reachable at localhost:5672`; restarted RabbitMQ and confirmed the test
  passes again
- ✅ `colcon test --packages-select fms_fleet_server fms_robot_agent` →
  30 tests, 0 errors, 0 failures, 0 skipped (RabbitMQ/Mongo up)

---

## Step 4.6 — Dockerize

Per the design decisions in `PHASE4_PLAN.md`: Gazebo/Nav2/robot agents stay
on the host (GPU/display); only the server-side pieces — `fleet-server`,
`rabbitmq`, `mongodb`, `fms-api` — are containerized. Resolves Open
Question #2: yes, `network_mode: host` for `fleet-server`/`fms-api`.

**What was built** (`docker/`):
- `fleet-server.Dockerfile` — multi-stage:
  - **builder**: `ros:humble-ros-base` + apt build deps
    (`libgrpc++-dev`, `protobuf-compiler-grpc`, `librabbitmq-dev`,
    `nlohmann-json3-dev`, `ros-humble-std-srvs`), compiles
    **mongo-c-driver 2.3.1** + **mongo-cxx-driver r4.3.1** from source
    (same versions as `docs/PHASE3_PREREQUISITES.md` — current releases
    aren't in apt), then `colcon build --packages-up-to fms_fleet_server`
  - **runtime**: same base image, only the *runtime* libs
    (`libgrpc++1`, `libgrpc10`, `libprotobuf23`, `librabbitmq4`,
    `ros-humble-std-srvs`) plus the mongocxx/.so files and `install/`
    copied from the builder stage — no compiler/build toolchain in the
    final image
  - Entrypoint reads `MONGO_URI`/`MONGO_DB`/`RABBITMQ_HOST`/
    `RABBITMQ_PORT`/`GRPC_PORT`/`NUM_ROBOTS` env vars and passes them as
    `--ros-args -p ...` to `fleet_server_node`
- `fms-api.Dockerfile` — `python:3.11-slim`, installs
  `requirements.txt` (runtime-only, see below), generates gRPC stubs from
  `fleet.proto` at build time (same fix-up as `generate_grpc.sh`), runs
  `uvicorn` on `0.0.0.0:8000`
- `docker-compose.yml` — `mongodb` (`mongo:7`, port 27017 published),
  `rabbitmq` (`rabbitmq:3.9-management`, ports 5672 + 15672 management UI
  published), `fleet-server` and `fms-api` both on `network_mode: host`
  (so `fleet-server` can see ROS 2 DDS traffic from host-run robot
  agents, and both can reach `mongodb`/`rabbitmq` via `localhost` once
  those containers' ports are published to the host)
- `.env.example` — `NUM_ROBOTS` (copy to `docker/.env`, gitignored)
- `src/fms_api/requirements.txt` split into runtime-only deps +
  new `requirements-dev.txt` (adds `pytest`/`httpx`/`ruff`) so the Docker
  image doesn't bundle test/lint tooling; `README.md` updated accordingly

**Caught and fixed while building:** first build of `fleet-server` failed
— `mkdir build` errored with "File exists" because the mongo-c-driver
release tarball already contains a top-level `build/` directory after
`--strip-components=1`. Fixed both the mongo-c-driver and mongo-cxx-driver
steps to use `mkdir -p build`.

**Verification:**
- ✅ `docker compose build` — `fms-api` built in ~24s; `fleet-server`
  built in a few minutes (mongocxx-from-source is the slow part, as
  expected, matching the "~10-15 min" estimate in `PHASE3_PREREQUISITES.md`)
- ✅ Live `docker compose up`: stopped the native `fms-mongo` container
  and `rabbitmq-server` systemd service first (same ports as the compose
  services — can't run both at once), brought up all 4 containers.
  `fleet-server` crashed a couple of times on startup
  (`RabbitMQ error during login`, RabbitMQ not fully ready yet) but
  Docker's `restart: unless-stopped` recovered it automatically once
  RabbitMQ finished starting — same crash-on-not-ready behavior the
  native node already has, not something Dockerizing introduced.
  Confirmed all 4 containers `Up` and stable for >1 minute,
  `mongosh --eval "db.runCommand({ping:1})"` → `{ok: 1}`,
  `rabbitmq-diagnostics ping` → `Ping succeeded`
- ✅ `curl localhost:8000/fleet/status` → `{"robots":[]}` (200), proxied
  through the **containerized** `fms-api` → gRPC → **containerized**
  `fleet-server`
- ✅ `curl -X POST localhost:8000/tasks ...` → `{"accepted":false,...,
  "message":"No IDLE robot available"}` (200) — expected, no robot agent
  running on the host during this check
- Torn down with `docker compose down`, restarted the native
  `fms-mongo`/`rabbitmq-server` afterward and confirmed both back up

**Note for next use**: `docker compose up` and the native
Mongo/RabbitMQ setup are alternatives, not meant to run simultaneously —
both bind the same host ports (27017, 5672). Stop one before starting
the other.

---

## Step 4.7 — GitHub Actions CI

**What was built:**
- `.clang-tidy` (repo root) — pins the check set used to lint
  `fms_robot_agent`/`fms_fleet_server`:
  `clang-diagnostic-*,clang-analyzer-*,-clang-analyzer-core.uninitialized.Assign`,
  with `WarningsAsErrors: '*'` so any match fails the lint step
- `.github/workflows/ci.yml` — 3 parallel jobs (deviates from the plan's
  literal "build → lint → test → docker build" sequential wording — these
  test independent things, so running them in parallel jobs is faster
  while still gating the PR on all of them passing; build→lint→test stay
  sequential *within* the first job since lint reuses that job's build
  artifacts):
  - **`ros-build-lint-test`**: runs directly on the `ubuntu-22.04` runner
    (not in a `ros:humble-ros-base` container) — this matters because the
    `mongodb`/`rabbitmq` service containers below are only reachable at
    `localhost` when the job itself isn't containerized, matching
    `fleet_server_node`'s and the Step 4.5 integration test's existing
    `localhost` defaults with no code changes. Steps: `ros-tooling/setup-ros`
    for ROS 2 Humble, apt build deps, mongo-c-driver/mongo-cxx-driver
    compiled from source **and cached** via `actions/cache` (same versions
    as `docker/fleet-server.Dockerfile`; cache avoids repeating the
    ~10-15 min from-source build on every run), `colcon build
    --packages-up-to fms_robot_agent fms_fleet_server` (scoped — excludes
    `fms_gazebo`/`fms_navigation`, which are launch/config-only and would
    pull in Gazebo Harmonic + Nav2 for no test coverage gain), `clang-tidy`
    over every `.cpp` in both packages, then `colcon test` +
    `colcon test-result --verbose` (which exits non-zero on any failure —
    `colcon test` itself doesn't). `mongodb`/`rabbitmq` service containers
    mean Step 4.5's integration test actually *runs* in CI rather than
    hitting its SKIP path.
  - **`fms-api`**: independent of the ROS build — `actions/setup-python`,
    `pip install -r requirements-dev.txt`, `./generate_grpc.sh`,
    `ruff check`, `pytest`
  - **`docker-build`**: `docker compose -f docker/docker-compose.yml build`
    — validates both Dockerfiles still build; doesn't start the stack
    (would conflict with the other job's service containers on shared
    runner ports if run concurrently)
- Triggers on push to `main` and on every pull request

**Caught while preparing this step:** ran `clang-tidy` against the
existing codebase before deciding whether to make it a hard gate.
Broader check sets (`modernize-*`, `bugprone-*` beyond the default) surface
substantial pre-existing style noise unrelated to Phase 4 — making *that*
a hard gate would fail CI immediately on already-written code, the
opposite of "green on every PR". Settled on clang-tidy's own default
check set (`clang-diagnostic-*,clang-analyzer-*`), verified 0 warnings
across every `.cpp` in both packages — except one false positive
(`clang-analyzer-core.uninitialized.Assign`) inside BehaviorTree.CPP's
vendored `expected.hpp`, a third-party header triggered by any BT node
calling `getInput<T>()`, not something in this project's control.
`--header-filter` does not suppress clang-analyzer diagnostics the way it
does other checks (verified locally — diagnostic still appeared with a
header-filter scoped to only our own source paths); excluded that one
specific check by name instead, which did work.

**Verification (everything that can be checked without an actual GitHub
Actions run):**
- ✅ Confirmed all 4 action references resolve to real published tags
  (`actions/checkout@v4`, `actions/cache@v4`, `actions/setup-python@v5`,
  `ros-tooling/setup-ros@v0.7`) via the GitHub API — avoids shipping a
  workflow that fails immediately on a typo'd version
- ✅ Ran the workflow's exact `clang-tidy` loop locally → `fail=0`
- ✅ Ran the workflow's exact `fms-api` steps locally in a clean
  environment (`PYTHONPATH=` cleared, matching the runner's environment)
  → `ruff check` clean, `pytest` 10/10 passed
- ✅ Ran the workflow's exact `docker compose -f docker/docker-compose.yml
  build` → both images built, exit 0
- ✅ Ran `colcon test` + `colcon test-result --verbose` locally → 30
  tests, 0 errors, 0 failures, exit 0
- 🔲 Not yet verified: an actual GitHub Actions run (would require
  pushing this branch/opening a PR — not done without explicit
  confirmation, since it's a visible, billed action on the remote repo)

---

## Open items carried from `PHASE4_PLAN.md`

- ~~Confirm `network_mode: host` is acceptable for the containerized
  `fleet-server` (Open Question #2) before Step 4.6.~~ Resolved in Step
  4.6: yes, used for both `fleet-server` and `fms-api`.
- ~~Confirm CI's integration test (Step 4.5) should stub the robot side
  rather than running Gazebo (Open Question #3).~~ Resolved in Step 4.5:
  yes — `mock_robot.py` stands in for Gazebo/Nav2/the real robot agent.

---

## Dependency audit (before starting 4.2–4.7) — 2026-06-20

Checked what each remaining step needs and what's actually on this
machine. ✅ = ready, ⚠️ = action needed.

| Tool | Needed for | Status |
|---|---|---|
| `libgtest-dev`, `ros-humble-ament-cmake-gtest` | 4.4 (gTest unit tests in `fms_robot_agent`/`fms_fleet_server`) | ✅ installed |
| `ctest` (bundled with CMake 3.22.1) | 4.4/4.5 (test runner) | ✅ installed |
| `clang-tidy` (LLVM 14) | 4.7 (CI lint step, C++) | ✅ installed |
| `grpcio`/`grpcio-tools` (Python) | 4.2/4.3 (extending `fms_api`'s gRPC client) | ✅ installed (in `fms_api/.venv`) |
| `ruff` (Python) | 4.7 (CI lint step, `fms_api`) | ✅ installed (added to `fms_api/.venv` + `requirements.txt`) |
| `docker.io` (Docker engine) | 4.6 (building/running containers) | ✅ installed (29.1.3) |
| **Docker Compose v2** (`docker compose ...`) | 4.6 (`docker compose up` for the full stack) | ✅ installed (`sudo apt install -y docker-compose-v2`), confirmed `docker compose version` → 2.40.3 |
| MongoDB | 4.5/4.6 | ✅ running (`fms-mongo` Docker container, port 27017) |
| RabbitMQ | 4.5/4.6 | ✅ running natively (fixed missing `/var/log/rabbitmq` this session) |
| `gh` CLI | optional — only for inspecting Actions runs from the terminal; GitHub Actions itself runs server-side, this isn't a blocker | 🔲 not installed (optional) |

All dependencies for the full Phase 4 roadmap are now in place.
