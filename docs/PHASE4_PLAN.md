# Phase 4 Plan — REST API, Docker & CI

Goal (from `PLAN.md`): expose the fleet to external (non-ROS) clients over
plain HTTP, package every server-side component into containers that start
with one command, and add automated tests + a CI pipeline so every change is
checked automatically. Milestone: `docker compose up` launches the full
stack; CI is green on every PR.

New package: **`fms_api/`** (Python 3.11, FastAPI) + repo-level
`docker/` and `.github/workflows/` config — no changes to the ROS2
simulation/Gazebo side.

---

## 0. What each new tool is and why it's here

You said you have no background with these — here's the plain-language
version (also recorded in `docs/GLOSSARY.md`).

### FastAPI (Python web framework)

Phase 3 finished with a **gRPC** API on the fleet server. gRPC is great
between your own services but awkward for outside callers — a warehouse
dashboard, a curl script, Postman, a non-Python client — because it needs
generated client stubs and isn't natively spoken by browsers. **REST over
HTTP** (`GET`/`POST` + JSON) is the lowest-common-denominator API every tool
understands. **FastAPI** is a Python framework for writing such HTTP
endpoints quickly, with automatic request validation and interactive docs
(`/docs` page) generated from your code.

**Why here:** `fms_api` is a thin translation layer —
`POST /tasks` (HTTP/JSON) → calls `SubmitTask` (gRPC) on the Phase 3 fleet
server → returns JSON back to the HTTP caller. It does not contain business
logic; the fleet server still owns allocation, state, persistence.

### Docker / Docker Compose

**Docker** packages an application together with its OS-level dependencies
(libraries, runtime, config) into an image that runs identically on any
machine — no more "works on my machine" because a teammate (or CI server)
is missing `libgrpc++-dev` or the right MongoDB version. A **container** is
a running instance of that image, isolated from the host.

**Docker Compose** describes a *group* of containers and how they connect
(ports, environment variables, startup order) in one YAML file, started
together with a single `docker compose up`.

**Why here:** right now, running this project means manually installing and
starting MongoDB, RabbitMQ, and the fleet server on your machine in the
right order. Compose turns that into one command and one file
(`docker/docker-compose.yml`) that anyone (a teammate, a CI runner) can run
without first reading `PHASE3_PREREQUISITES.md`. Gazebo/ROS2 simulation
stays on the host (it needs a GPU/display) — only the *server-side* pieces
(`fleet-server`, `rabbitmq`, `mongodb`, `fms_api`) get containerized.

### gTest (Google Test)

A C++ **unit-testing** framework — you write small functions like
`TEST(RobotFSM, IdleToAssignedOnTaskReceived) { ... }` that exercise one
piece of logic in isolation (no ROS, no network, no real robot) and assert
the result is correct. Fast (milliseconds), runs on every build.

**Why here:** Phase 2/3 logic (`RobotFSM` transitions, BT node behavior,
the task allocator's scoring formula) was only ever checked by hand,
through full simulation runs. gTest lets CI catch a broken transition or a
scoring regression in seconds, without spinning up Gazebo.

### cTest (CMake's test runner)

CMake's built-in mechanism for *registering and running* test executables
(`add_test(...)` in `CMakeLists.txt`, then `ctest` runs them all and reports
pass/fail). It doesn't care whether a test is a single gTest binary or a
multi-process integration script — it's the runner, gTest is one kind of
test it can run.

**Why here:** Phase 4 needs both unit tests (gTest, fast, isolated) and a
broader **integration test** (does a task submitted via gRPC really flow
through RabbitMQ → allocator → MongoDB → completion, end-to-end, with real
processes talking to each other?). cTest runs both kinds with one command
(`ctest`), which is what CI will call.

### GitHub Actions / CI (Continuous Integration)

**CI** means: every time code changes (e.g. on every pull request), a
server automatically builds it, runs the tests, and reports pass/fail —
instead of finding out something is broken only when a human runs it later.
**GitHub Actions** is GitHub's built-in automation for this: a YAML file in
`.github/workflows/` describes the steps to run, GitHub runs them in a
clean VM/container on every push or PR.

**Why here:** this project has gone through three phases on trust ("I
tested it on my machine"). CI makes every future change to
`fms_robot_agent`, `fms_fleet_server`, or `fms_api` get an automatic
build + lint + test + Docker-build check, catching regressions before
they're merged.

---

## 1. Design decisions

- **`fms_api` is a thin gRPC client, not a second source of truth.** It
  holds no state; every request proxies to the Phase 3 fleet server's gRPC
  API. This mirrors the Phase 3 decision that robots never touch
  Mongo/RabbitMQ directly — now external HTTP clients never touch gRPC,
  Mongo, or RabbitMQ directly either. Single point of truth stays the fleet
  server.
- **Gazebo/Nav2/robot agents stay on the host, un-containerized.** They
  need a GPU and a display (RViz/Gazebo GUI) — not worth containerizing for
  this project. Only `fleet-server`, `rabbitmq`, `mongodb`, `fms_api` move
  into Compose. The fleet server's ROS side (subscribing to
  `/robot_N/robot_status` etc.) talks to the host's ROS graph over the
  network — this is the `ros2-bridge` concern below.
- **`ros2-bridge` is networking, not a new component.** A containerized
  `fleet-server` still needs to see ROS 2 DDS traffic from the
  host-side robot agents. Recommended: run the `fleet-server` container
  with `network_mode: host` (simplest — same approach as the
  `CYCLONEDDS_URI=loopback` fix already in use) rather than building a
  separate bridge node. Revisit only if `network_mode: host` proves
  insufficient.
- **Unit tests live next to the code they test** (`fms_robot_agent/test/`,
  `fms_fleet_server/test/`), wired into each package's `CMakeLists.txt` via
  `ament_add_gtest`. The integration test is a new top-level `test/`
  directory (or `fms_fleet_server/test/integration/`) since it spans
  multiple packages/processes.

---

## 2. Package scaffold

```
fms_ws/
├── src/
│   ├── fms_api/                        # NEW — FastAPI REST server
│   │   ├── fms_api/
│   │   │   ├── main.py                 # FastAPI app, route definitions
│   │   │   ├── grpc_client.py          # wraps fleet.proto stubs
│   │   │   └── schemas.py              # pydantic request/response models
│   │   ├── tests/
│   │   │   └── test_routes.py          # pytest, mocks the gRPC client
│   │   ├── Dockerfile
│   │   ├── pyproject.toml / requirements.txt
│   │   └── package.xml                 # if kept as a colcon-tracked package
│   ├── fms_robot_agent/
│   │   └── test/
│   │       ├── test_robot_fsm.cpp      # NEW — gTest: FSM transitions
│   │       └── test_bt_nodes.cpp       # NEW — gTest: BT action/condition nodes
│   └── fms_fleet_server/
│       └── test/
│           ├── test_task_allocator.cpp # NEW — gTest: scoring logic
│           └── integration/
│               └── test_task_lifecycle.py  # NEW — cTest-registered, full
│                                              gRPC→RabbitMQ→Mongo round trip
├── docker/
│   ├── docker-compose.yml              # NEW — fleet-server, rabbitmq, mongodb, fms_api
│   ├── fleet-server.Dockerfile         # NEW
│   └── .env.example                    # NEW — ports, credentials placeholders
└── .github/
    └── workflows/
        └── ci.yml                      # NEW — build → lint → test → docker build
```

---

## 3. Implementation order (incremental milestones)

### Step 4.1 — `fms_api` skeleton + read-only endpoint
- New Python package `fms_api`, FastAPI app, `grpcio`-generated client
  stubs from the existing `fleet.proto` (reuse Phase 3's `.proto`, don't
  duplicate it)
- `GET /fleet/status` → calls `GetFleetStatus` gRPC, returns JSON
- **Verify**: `uvicorn fms_api.main:app --reload`, `curl localhost:8000/fleet/status`
  matches what the Phase 3 Python gRPC test client showed

### Step 4.2 — Task endpoints
- `POST /tasks` (body: pick/drop poses) → `SubmitTask` gRPC → `{task_id, accepted}`
- `GET /tasks/{id}` → `GetTaskStatus` gRPC → task lifecycle JSON
- **Verify**: submit a task via `curl`/Postman while a robot is `IDLE`,
  poll `GET /tasks/{id}` until `COMPLETED`, matching MongoDB's `tasks` doc

### Step 4.3 — Robot command endpoint
- `POST /robots/{id}/command` (e.g. `{"command": "inject_fault"}` or
  `{"command": "cancel_task"}`) — needs a small gRPC addition
  (`fleet.proto`: `SendRobotCommand`) since Phase 3's service doesn't expose
  this yet
- **Verify**: `POST .../command {"command":"inject_fault"}` triggers the
  same `/robot_N/inject_fault` behavior already verified manually in Phase 2

### Step 4.4 — gTest unit tests
- `fms_robot_agent/test/test_robot_fsm.cpp`: every documented transition
  (`IDLE→ASSIGNED`, `NAVIGATING→RECOVERING` on Nav2 failure, etc.) plus
  invalid-transition rejection
- `fms_fleet_server/test/test_task_allocator.cpp`: scoring formula —
  closer/higher-SOC robot wins, no-IDLE-robot case returns "no candidate"
- Wire into each package's `CMakeLists.txt` (`ament_add_gtest`)
- **Verify**: `colcon test --packages-select fms_robot_agent fms_fleet_server`

### Step 4.5 — cTest integration test
- A script that starts (or assumes already running) `fleet-server` +
  RabbitMQ + MongoDB, submits a task over gRPC, polls until `COMPLETED`,
  asserts the MongoDB document's full lifecycle — this is the Phase 3
  50-task load test's logic, formalized as one repeatable, CI-runnable case
  instead of a manual script
- Register via `add_test(NAME task_lifecycle COMMAND ...)` so `ctest` picks
  it up alongside the gTest binaries
- **Verify**: `ctest --output-on-failure` runs both unit and integration
  tests in one command

### Step 4.6 — Dockerize
- `fleet-server.Dockerfile`: multi-stage build (ROS 2 Humble base image →
  colcon build → runtime layer); same for `fms_api`
- `docker/docker-compose.yml`: `mongodb`, `rabbitmq` (official images),
  `fleet-server` (`network_mode: host`, per the design decision above),
  `fms_api` (depends_on `fleet-server`)
- `.env.example` documents required vars (Mongo URI, RabbitMQ URI, gRPC
  port, API port)
- **Verify**: `docker compose up` from a clean checkout (with the host-side
  ROS2 simulation already running) brings up all four services; `curl
  localhost:8000/fleet/status` returns live data

### Step 4.7 — GitHub Actions CI
- `.github/workflows/ci.yml`: on every push/PR —
  1. `build` — `colcon build` (C++ packages) + `pip install` (`fms_api`)
  2. `lint` — `clang-tidy` (already a stated tool in `PLAN.md`) +
     `ruff`/`flake8` for `fms_api`
  3. `test` — `colcon test` (gTest) + `ctest` (integration) +
     `pytest fms_api/tests`
  4. `docker build` — builds both Dockerfiles, fails the job if either
     image fails to build
- **Verify**: open a throwaway PR, confirm the Actions tab shows all steps
  green

---

## 4. Open questions to confirm before starting

1. **`fms_api` as a colcon package or a plain Python project?**
   (Recommended: plain Python project under `src/fms_api/` with its own
   `requirements.txt`/`pyproject.toml`, *not* wired into `colcon build` —
   it has no ROS dependency, and keeping it colcon-free simplifies the
   Dockerfile and CI's Python steps.)
2. **`network_mode: host` for the `fleet-server` container** — simplest
   given the existing `CYCLONEDDS_URI=loopback` setup, but only works when
   Compose runs on the same machine as the simulation (no remote
   deployment). Acceptable for this project's scope? (Recommended: yes —
   matches "simulation only, single machine" from `PLAN.md`'s constraints.)
3. **CI runs the integration test (Step 4.5) without Gazebo/robots** —
   it only needs `fleet-server` + RabbitMQ + MongoDB + a mock/stub
   `TaskCompletion` publisher (since real robot execution takes minutes and
   needs Gazebo). Confirm: integration test in CI fakes the robot side, the
   *real* end-to-end check remains the manual load test from Phase 3.
   (Recommended: yes — CI should be fast and deterministic, not run
   Gazebo.)

---

## 5. Definition of done (Phase 4)

- [ ] `fms_api` builds/runs (`uvicorn`) and proxies all four endpoints to
      the Phase 3 gRPC fleet server
- [ ] gTest suites cover `RobotFSM` transitions and the task allocator's
      scoring logic
- [ ] cTest integration test exercises a full task lifecycle
      (gRPC → RabbitMQ → allocator → Mongo) without requiring Gazebo
- [ ] `docker compose up` (from `docker/`) starts `fleet-server`,
      `rabbitmq`, `mongodb`, `fms_api` and the stack responds to
      `curl localhost:8000/fleet/status`
- [ ] GitHub Actions runs build → lint → test → docker build on every PR
      and is green on `main`
