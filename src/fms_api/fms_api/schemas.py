from pydantic import BaseModel


class Pose2D(BaseModel):
    x: float
    y: float


class RobotStatus(BaseModel):
    robot_id: str
    state: int
    state_str: str
    battery_soc: float
    current_task_id: str
    pose: Pose2D


class FleetStatusResponse(BaseModel):
    robots: list[RobotStatus]


class TaskCreateRequest(BaseModel):
    pick_pose: Pose2D
    drop_pose: Pose2D
    task_type: int = 0  # 0=PICK, 1=DROP, 2=CHARGE — matches fleet.proto
    payload_id: str = ""
    priority: int = 0
    deadline_secs: float = 0.0


class TaskCreateResponse(BaseModel):
    accepted: bool
    task_id: str
    robot_id: str
    message: str


class TaskStatusResponse(BaseModel):
    task_id: str
    robot_id: str
    status: str
    result: int
    duration_secs: float
    error_message: str


class RobotCommandRequest(BaseModel):
    command: str  # only "inject_fault" is currently supported
    value: bool = True


class RobotCommandResponse(BaseModel):
    success: bool
    message: str
