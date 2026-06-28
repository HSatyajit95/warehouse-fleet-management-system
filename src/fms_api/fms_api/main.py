from fastapi import FastAPI, HTTPException
from grpc import RpcError

from fms_api.grpc_client import FleetClient
from fms_api.schemas import (
    FleetStatusResponse,
    Pose2D,
    RobotCommandRequest,
    RobotCommandResponse,
    RobotStatus,
    TaskCreateRequest,
    TaskCreateResponse,
    TaskStatusResponse,
)

app = FastAPI(title="fms_api", description="REST front-end for the FMS fleet server")
fleet_client = FleetClient()


@app.get("/fleet/status", response_model=FleetStatusResponse)
def get_fleet_status():
    try:
        resp = fleet_client.get_fleet_status()
    except RpcError as exc:
        raise HTTPException(status_code=502, detail=f"fleet server unreachable: {exc.details()}")

    return FleetStatusResponse(
        robots=[
            RobotStatus(
                robot_id=r.robot_id,
                state=r.state,
                state_str=r.state_str,
                battery_soc=r.battery_soc,
                current_task_id=r.current_task_id,
                pose=Pose2D(x=r.pose.x, y=r.pose.y),
            )
            for r in resp.robots
        ]
    )


@app.post("/tasks", response_model=TaskCreateResponse)
def create_task(task: TaskCreateRequest):
    try:
        resp = fleet_client.submit_task(
            pick_x=task.pick_pose.x,
            pick_y=task.pick_pose.y,
            drop_x=task.drop_pose.x,
            drop_y=task.drop_pose.y,
            task_type=task.task_type,
            payload_id=task.payload_id,
            priority=task.priority,
            deadline_secs=task.deadline_secs,
        )
    except RpcError as exc:
        raise HTTPException(status_code=502, detail=f"fleet server unreachable: {exc.details()}")

    return TaskCreateResponse(
        accepted=resp.accepted,
        task_id=resp.task_id,
        robot_id=resp.robot_id,
        message=resp.message,
    )


@app.get("/tasks/{task_id}", response_model=TaskStatusResponse)
def get_task(task_id: str):
    try:
        resp = fleet_client.get_task_status(task_id)
    except RpcError as exc:
        raise HTTPException(status_code=502, detail=f"fleet server unreachable: {exc.details()}")

    if not resp.found:
        raise HTTPException(status_code=404, detail=f"task {task_id} not found")

    return TaskStatusResponse(
        task_id=resp.task_id,
        robot_id=resp.robot_id,
        status=resp.status,
        result=resp.result,
        duration_secs=resp.duration_secs,
        error_message=resp.error_message,
    )


@app.post("/robots/{robot_id}/command", response_model=RobotCommandResponse)
def send_robot_command(robot_id: str, command: RobotCommandRequest):
    try:
        resp = fleet_client.send_robot_command(robot_id, command.command, command.value)
    except RpcError as exc:
        raise HTTPException(status_code=502, detail=f"fleet server unreachable: {exc.details()}")

    if not resp.success:
        raise HTTPException(status_code=400, detail=resp.message)

    return RobotCommandResponse(success=resp.success, message=resp.message)
