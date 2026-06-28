"""Thin wrapper around the fleet server's gRPC FleetService.

fms_api holds no state of its own — every call here proxies straight to
fleet_server_node (see fms_fleet_server/proto/fleet.proto). Keeping this in
one module means main.py's route handlers stay free of gRPC plumbing.
"""
import os

import grpc

from fms_api.generated import fleet_pb2, fleet_pb2_grpc

FLEET_SERVER_ADDR = os.environ.get("FLEET_SERVER_ADDR", "localhost:50051")


class FleetClient:
    def __init__(self, addr: str = FLEET_SERVER_ADDR):
        self._channel = grpc.insecure_channel(addr)
        self._stub = fleet_pb2_grpc.FleetServiceStub(self._channel)

    def get_fleet_status(self) -> fleet_pb2.GetFleetStatusResponse:
        return self._stub.GetFleetStatus(fleet_pb2.GetFleetStatusRequest())

    def submit_task(
        self,
        pick_x: float,
        pick_y: float,
        drop_x: float,
        drop_y: float,
        task_type: int = 0,
        payload_id: str = "",
        priority: int = 0,
        deadline_secs: float = 0.0,
    ) -> fleet_pb2.SubmitTaskResponse:
        request = fleet_pb2.SubmitTaskRequest(
            task_type=task_type,
            pick_pose=fleet_pb2.Pose2D(x=pick_x, y=pick_y),
            drop_pose=fleet_pb2.Pose2D(x=drop_x, y=drop_y),
            payload_id=payload_id,
            priority=priority,
            deadline_secs=deadline_secs,
        )
        return self._stub.SubmitTask(request)

    def get_task_status(self, task_id: str) -> fleet_pb2.GetTaskStatusResponse:
        return self._stub.GetTaskStatus(fleet_pb2.GetTaskStatusRequest(task_id=task_id))

    def send_robot_command(
        self, robot_id: str, command: str, value: bool = True
    ) -> fleet_pb2.SendRobotCommandResponse:
        request = fleet_pb2.SendRobotCommandRequest(robot_id=robot_id, command=command, value=value)
        return self._stub.SendRobotCommand(request)

    def close(self):
        self._channel.close()
