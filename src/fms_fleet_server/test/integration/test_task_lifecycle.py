#!/usr/bin/env python3
"""
test_task_lifecycle.py — Phase 4.5 integration test, registered with ctest
(see CMakeLists.txt: `task_lifecycle_integration`).

Exercises the full task lifecycle the gRPC way (mirrors Phase 3's load-test
client, formalized into one repeatable, automated case):

    gRPC SubmitTask -> FleetServerNode allocator -> mock robot "executes"
    the task -> TaskCompletion -> MongoDB `tasks` doc updated to COMPLETED

Starts its own fleet_server_node and a lightweight mock_robot.py (no
Gazebo/Nav2 — see mock_robot.py) as subprocesses, on a dedicated gRPC port
and MongoDB database so it doesn't collide with any fleet_server_node a
developer might already have running.

Prerequisites: MongoDB and RabbitMQ already running and reachable (same
prerequisites as fleet_server_node itself — RabbitMQ must be up for the
node to start at all, even though SubmitTask's allocation path in this
test does not route through it; the AMQP `tasks.incoming` path is
exercised separately by scripts/task_request_amqp.py, not duplicated
here). If either is unreachable, this test SKIPs (exit code 99) rather
than failing, since CI environments may not always have them up — see
PHASE4_PLAN.md's Open Question #3.

Must be run with ROS 2 + this workspace already sourced (same requirement
as every other script in this repo) so `ros2 run` and `rclpy`/`fms_msgs`
imports resolve.
"""
import os
import signal
import socket
import subprocess
import sys
import tempfile
import time

SKIP_EXIT_CODE = 99

GRPC_PORT = 50061
MONGO_HOST, MONGO_PORT = "localhost", 27017
RABBITMQ_HOST, RABBITMQ_PORT = "localhost", 5672
MONGO_TEST_DB = "fms_integration_test"
ROBOT_NAME = "robot_1"
EXEC_DURATION_SECS = 0.5

HERE = os.path.dirname(os.path.abspath(__file__))
PROTO_DIR = os.path.normpath(os.path.join(HERE, "..", "..", "proto"))


def port_open(host: str, port: int, timeout: float = 1.0) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def wait_for_port(host: str, port: int, timeout: float) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if port_open(host, port):
            return True
        time.sleep(0.3)
    return False


def generate_grpc_stubs(out_dir: str) -> None:
    subprocess.run(
        [
            sys.executable, "-m", "grpc_tools.protoc",
            f"-I{PROTO_DIR}",
            f"--python_out={out_dir}",
            f"--grpc_python_out={out_dir}",
            os.path.join(PROTO_DIR, "fleet.proto"),
        ],
        check=True,
    )


def terminate(proc: subprocess.Popen, log_path: str, label: str) -> None:
    if proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
    print(f"--- {label} log tail ---")
    try:
        with open(log_path) as f:
            print("".join(f.readlines()[-20:]))
    except OSError:
        pass


def main() -> int:
    if not port_open(MONGO_HOST, MONGO_PORT):
        print(f"SKIP: MongoDB not reachable at {MONGO_HOST}:{MONGO_PORT}")
        return SKIP_EXIT_CODE
    if not port_open(RABBITMQ_HOST, RABBITMQ_PORT):
        print(f"SKIP: RabbitMQ not reachable at {RABBITMQ_HOST}:{RABBITMQ_PORT}")
        return SKIP_EXIT_CODE

    with tempfile.TemporaryDirectory() as tmp_dir:
        generate_grpc_stubs(tmp_dir)
        sys.path.insert(0, tmp_dir)
        import fleet_pb2
        import fleet_pb2_grpc
        import grpc

        env = os.environ.copy()
        fleet_log = os.path.join(tmp_dir, "fleet_server.log")
        robot_log = os.path.join(tmp_dir, "mock_robot.log")

        fleet_proc = subprocess.Popen(
            [
                "ros2", "run", "fms_fleet_server", "fleet_server_node", "--ros-args",
                "-p", "num_robots:=1",
                "-p", f"grpc_port:={GRPC_PORT}",
                "-p", f"mongo_db:={MONGO_TEST_DB}",
            ],
            env=env, stdout=open(fleet_log, "w"), stderr=subprocess.STDOUT,
        )
        robot_proc = subprocess.Popen(
            [sys.executable, os.path.join(HERE, "mock_robot.py"), ROBOT_NAME, str(EXEC_DURATION_SECS)],
            env=env, stdout=open(robot_log, "w"), stderr=subprocess.STDOUT,
        )

        try:
            if not wait_for_port("localhost", GRPC_PORT, timeout=15.0):
                print("FAIL: fleet_server_node did not open its gRPC port in time")
                return 1

            # Let the mock robot's first /robot_status reach the allocator
            # (published every 0.5s) before submitting a task.
            time.sleep(2.0)

            channel = grpc.insecure_channel(f"localhost:{GRPC_PORT}")
            stub = fleet_pb2_grpc.FleetServiceStub(channel)

            submit_response = stub.SubmitTask(
                fleet_pb2.SubmitTaskRequest(
                    pick_pose=fleet_pb2.Pose2D(x=1.0, y=1.0),
                    drop_pose=fleet_pb2.Pose2D(x=2.0, y=2.0),
                ),
                timeout=5.0,
            )
            if not submit_response.accepted:
                print(f"FAIL: SubmitTask not accepted: {submit_response.message}")
                return 1
            if submit_response.robot_id != ROBOT_NAME:
                print(f"FAIL: expected robot_id={ROBOT_NAME}, got {submit_response.robot_id}")
                return 1
            task_id = submit_response.task_id
            print(f"SubmitTask accepted: task_id={task_id} robot_id={submit_response.robot_id}")

            deadline = time.time() + 15.0
            final_status = None
            while time.time() < deadline:
                status_response = stub.GetTaskStatus(
                    fleet_pb2.GetTaskStatusRequest(task_id=task_id), timeout=5.0
                )
                if status_response.found and status_response.status in ("COMPLETED", "FAILED"):
                    final_status = status_response
                    break
                time.sleep(0.5)

            if final_status is None:
                print("FAIL: task did not reach a terminal status within 15s")
                return 1
            if final_status.status != "COMPLETED":
                print(f"FAIL: task ended in status={final_status.status}, expected COMPLETED")
                return 1
            if final_status.result != 0:  # RESULT_SUCCESS
                print(f"FAIL: task result={final_status.result}, expected 0 (RESULT_SUCCESS)")
                return 1

            print(
                f"PASS: task {task_id} reached COMPLETED via {final_status.robot_id} "
                f"in {final_status.duration_secs:.2f}s (MongoDB db={MONGO_TEST_DB})"
            )
            return 0
        finally:
            terminate(robot_proc, robot_log, "mock_robot")
            terminate(fleet_proc, fleet_log, "fleet_server_node")


if __name__ == "__main__":
    sys.exit(main())
