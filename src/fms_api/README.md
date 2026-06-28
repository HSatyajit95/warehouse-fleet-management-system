# fms_api

FastAPI REST front-end for the Phase 3 fleet server. Every endpoint is a
thin proxy to the fleet server's gRPC `FleetService` (`fms_fleet_server/proto/fleet.proto`)
— this package holds no state of its own.

## Setup

```bash
cd src/fms_api
python3 -m venv .venv && .venv/bin/pip install -r requirements-dev.txt
PYTHONPATH= .venv/bin/python ./generate_grpc.sh   # or run generate_grpc.sh directly
```

`requirements.txt` is runtime-only (what the Docker image installs —
see `Dockerfile`); `requirements-dev.txt` adds `pytest`/`httpx`/`ruff` on
top for local development. The generated stubs in `fms_api/generated/`
are NOT checked into git — regenerate with `./generate_grpc.sh` after any
`fleet.proto` change.

## Run

```bash
uvicorn fms_api.main:app --reload --port 8000
```

Requires `fleet_server_node` already running and listening on its gRPC port
(default `50051`, see `FLEET_SERVER_ADDR` env var to override).

## Endpoints

- `GET /fleet/status` — proxies `GetFleetStatus` (4.1)
- `POST /tasks` — proxies `SubmitTask`; body: `{"pick_pose": {"x","y"}, "drop_pose": {"x","y"}, "task_type"?, "payload_id"?, "priority"?, "deadline_secs"?}` (4.2)
- `GET /tasks/{id}` — proxies `GetTaskStatus`; 404 if the task isn't found (4.2)
- `POST /robots/{id}/command` — proxies `SendRobotCommand`; body: `{"command": "inject_fault", "value"?}` (only `inject_fault` is currently supported — the only command the Phase 2 robot agent exposes); 400 if the robot/command is invalid or the robot is unreachable (4.3)
