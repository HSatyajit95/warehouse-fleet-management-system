from unittest.mock import MagicMock

from fastapi.testclient import TestClient

from fms_api import main
from fms_api.generated import fleet_pb2


def _fake_robot(robot_id="robot_1"):
    return fleet_pb2.RobotStatusInfo(
        robot_id=robot_id,
        state=1,
        state_str="ASSIGNED",
        battery_soc=82.5,
        current_task_id="task_123",
        pose=fleet_pb2.Pose2D(x=10.2, y=-13.9),
    )


def test_get_fleet_status_proxies_grpc(monkeypatch):
    fake_response = fleet_pb2.GetFleetStatusResponse(robots=[_fake_robot()])
    mock_client = MagicMock()
    mock_client.get_fleet_status.return_value = fake_response
    monkeypatch.setattr(main, "fleet_client", mock_client)

    client = TestClient(main.app)
    resp = client.get("/fleet/status")

    assert resp.status_code == 200
    body = resp.json()
    assert body["robots"][0]["robot_id"] == "robot_1"
    assert body["robots"][0]["battery_soc"] == 82.5
    assert body["robots"][0]["pose"] == {"x": 10.2, "y": -13.9}
    mock_client.get_fleet_status.assert_called_once()


def test_get_fleet_status_returns_502_on_grpc_error(monkeypatch):
    from grpc import RpcError

    class FakeRpcError(RpcError):
        def details(self):
            return "connection refused"

    mock_client = MagicMock()
    mock_client.get_fleet_status.side_effect = FakeRpcError()
    monkeypatch.setattr(main, "fleet_client", mock_client)

    client = TestClient(main.app)
    resp = client.get("/fleet/status")

    assert resp.status_code == 502


def test_create_task_proxies_grpc(monkeypatch):
    fake_response = fleet_pb2.SubmitTaskResponse(
        accepted=True, task_id="task_abc", robot_id="robot_1", message="assigned"
    )
    mock_client = MagicMock()
    mock_client.submit_task.return_value = fake_response
    monkeypatch.setattr(main, "fleet_client", mock_client)

    client = TestClient(main.app)
    resp = client.post(
        "/tasks",
        json={"pick_pose": {"x": 14.0, "y": -14.0}, "drop_pose": {"x": 10.0, "y": -14.0}},
    )

    assert resp.status_code == 200
    body = resp.json()
    assert body == {
        "accepted": True,
        "task_id": "task_abc",
        "robot_id": "robot_1",
        "message": "assigned",
    }
    mock_client.submit_task.assert_called_once_with(
        pick_x=14.0,
        pick_y=-14.0,
        drop_x=10.0,
        drop_y=-14.0,
        task_type=0,
        payload_id="",
        priority=0,
        deadline_secs=0.0,
    )


def test_create_task_returns_502_on_grpc_error(monkeypatch):
    from grpc import RpcError

    class FakeRpcError(RpcError):
        def details(self):
            return "connection refused"

    mock_client = MagicMock()
    mock_client.submit_task.side_effect = FakeRpcError()
    monkeypatch.setattr(main, "fleet_client", mock_client)

    client = TestClient(main.app)
    resp = client.post(
        "/tasks",
        json={"pick_pose": {"x": 14.0, "y": -14.0}, "drop_pose": {"x": 10.0, "y": -14.0}},
    )

    assert resp.status_code == 502


def test_get_task_proxies_grpc(monkeypatch):
    fake_response = fleet_pb2.GetTaskStatusResponse(
        found=True,
        task_id="task_abc",
        robot_id="robot_1",
        status="COMPLETED",
        result=0,
        duration_secs=91.2,
        error_message="",
    )
    mock_client = MagicMock()
    mock_client.get_task_status.return_value = fake_response
    monkeypatch.setattr(main, "fleet_client", mock_client)

    client = TestClient(main.app)
    resp = client.get("/tasks/task_abc")

    assert resp.status_code == 200
    body = resp.json()
    assert body["status"] == "COMPLETED"
    assert body["duration_secs"] == 91.2
    mock_client.get_task_status.assert_called_once_with("task_abc")


def test_get_task_returns_404_when_not_found(monkeypatch):
    fake_response = fleet_pb2.GetTaskStatusResponse(found=False)
    mock_client = MagicMock()
    mock_client.get_task_status.return_value = fake_response
    monkeypatch.setattr(main, "fleet_client", mock_client)

    client = TestClient(main.app)
    resp = client.get("/tasks/does_not_exist")

    assert resp.status_code == 404


def test_get_task_returns_502_on_grpc_error(monkeypatch):
    from grpc import RpcError

    class FakeRpcError(RpcError):
        def details(self):
            return "connection refused"

    mock_client = MagicMock()
    mock_client.get_task_status.side_effect = FakeRpcError()
    monkeypatch.setattr(main, "fleet_client", mock_client)

    client = TestClient(main.app)
    resp = client.get("/tasks/task_abc")

    assert resp.status_code == 502


def test_send_robot_command_proxies_grpc(monkeypatch):
    fake_response = fleet_pb2.SendRobotCommandResponse(success=True, message="fault injected")
    mock_client = MagicMock()
    mock_client.send_robot_command.return_value = fake_response
    monkeypatch.setattr(main, "fleet_client", mock_client)

    client = TestClient(main.app)
    resp = client.post("/robots/robot_1/command", json={"command": "inject_fault", "value": True})

    assert resp.status_code == 200
    assert resp.json() == {"success": True, "message": "fault injected"}
    mock_client.send_robot_command.assert_called_once_with("robot_1", "inject_fault", True)


def test_send_robot_command_returns_400_on_failure(monkeypatch):
    fake_response = fleet_pb2.SendRobotCommandResponse(success=False, message="unknown robot_id: robot_9")
    mock_client = MagicMock()
    mock_client.send_robot_command.return_value = fake_response
    monkeypatch.setattr(main, "fleet_client", mock_client)

    client = TestClient(main.app)
    resp = client.post("/robots/robot_9/command", json={"command": "inject_fault"})

    assert resp.status_code == 400
    assert resp.json()["detail"] == "unknown robot_id: robot_9"


def test_send_robot_command_returns_502_on_grpc_error(monkeypatch):
    from grpc import RpcError

    class FakeRpcError(RpcError):
        def details(self):
            return "connection refused"

    mock_client = MagicMock()
    mock_client.send_robot_command.side_effect = FakeRpcError()
    monkeypatch.setattr(main, "fleet_client", mock_client)

    client = TestClient(main.app)
    resp = client.post("/robots/robot_1/command", json={"command": "inject_fault"})

    assert resp.status_code == 502
