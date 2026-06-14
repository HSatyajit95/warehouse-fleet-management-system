#!/usr/bin/env python3
"""
task_injector.py — Publish TaskAssignment messages to FMS robot agents.

Usage:
  python3 task_injector.py                         # 1 task round-robined to 4 robots
  python3 task_injector.py --count 5               # 5 tasks per robot
  python3 task_injector.py --robots 1 2            # only robot_1 and robot_2
  python3 task_injector.py --count 1 --robots 1    # single task, single robot
  python3 task_injector.py --fault robot_2         # inject fault mid-task
  python3 task_injector.py --pick 14.0 -5.0 --drop -5.0 -5.0 --robots 1
"""

import argparse
import sys
import threading
import time
import uuid

import rclpy
from rclpy.node import Node
from fms_msgs.msg import TaskAssignment, TaskCompletion, RobotStatus
from geometry_msgs.msg import Pose
from std_srvs.srv import SetBool

# Warehouse pick and drop station positions (map frame).
# Pick positions match actual Gazebo SDF pick_station model locations.
# Drop positions are in the open aisle east of shelf_D (x=6), aligned per row.
PICK_STATIONS = {
    # x=14.0 → 1 m west of the station's west face (x=15.0); robot approaches from the west.
    "robot_1": (14.0, -14.0),  # pick_station_4
    "robot_2": (14.0,  -6.0),  # pick_station_3
    "robot_3": (14.0,   6.0),  # pick_station_2
    "robot_4": (14.0,  14.0),  # pick_station_1
    "robot_5": (14.0, -14.0),
    "robot_6": (14.0,  -6.0),
    "robot_7": (14.0,   6.0),
    "robot_8": (14.0,  14.0),
}

DROP_STATIONS = {
    "robot_1": (10.0, -14.0),  # aisle east of shelf_D, south row
    "robot_2": (10.0,  -6.0),  # aisle east of shelf_D, centre-south
    "robot_3": (10.0,   6.0),  # aisle east of shelf_D, centre-north
    "robot_4": (10.0,  14.0),  # aisle east of shelf_D, north row
    "robot_5": (10.0, -14.0),
    "robot_6": (10.0,  -6.0),
    "robot_7": (10.0,   6.0),
    "robot_8": (10.0,  14.0),
}

STATE_NAMES = {
    0: "IDLE",
    1: "ASSIGNED",
    2: "NAVIGATING",
    3: "EXECUTING",
    4: "REPORTING",
    5: "RECOVERING",
    6: "CHARGING",
}


def make_pose(x: float, y: float) -> Pose:
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = 0.0
    p.orientation.w = 1.0
    return p


class FleetMonitor(Node):
    def __init__(self, robot_names: list):
        super().__init__("fms_task_injector")

        self.robot_names = robot_names
        self.completions: dict[str, list] = {n: [] for n in robot_names}
        self.status:      dict[str, dict] = {n: {} for n in robot_names}
        self._lock = threading.RLock()

        # Publishers per robot
        self.task_pubs = {}
        for name in robot_names:
            self.task_pubs[name] = self.create_publisher(
                TaskAssignment, f"/{name}/task_assignment", 10)

        # Subscribers per robot
        for name in robot_names:
            self.create_subscription(
                TaskCompletion, f"/{name}/task_completion",
                lambda msg, n=name: self._completion_cb(n, msg), 10)
            self.create_subscription(
                RobotStatus, f"/{name}/robot_status",
                lambda msg, n=name: self._status_cb(n, msg), 10)

        # Fault injection clients
        self.fault_clients = {}
        for name in robot_names:
            self.fault_clients[name] = self.create_client(
                SetBool, f"/{name}/inject_fault")

    def _completion_cb(self, robot_name: str, msg: TaskCompletion):
        with self._lock:
            self.completions[robot_name].append(msg)

    def _status_cb(self, robot_name: str, msg: RobotStatus):
        with self._lock:
            self.status[robot_name] = {
                "state": msg.state,
                "state_name": STATE_NAMES.get(msg.state, "?"),
                "soc": msg.battery_soc,
                "task": msg.current_task_id,
            }

    def send_task(self, robot_name: str, pick_xy: tuple, drop_xy: tuple) -> str:
        task_id = f"task_{uuid.uuid4().hex[:8]}"
        msg = TaskAssignment()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id   = task_id
        msg.robot_id  = robot_name
        msg.task_type = TaskAssignment.TASK_PICK
        msg.pick_pose = make_pose(*pick_xy)
        msg.drop_pose = make_pose(*drop_xy)
        msg.priority  = 1
        msg.deadline_secs = 300.0
        self.task_pubs[robot_name].publish(msg)
        self.get_logger().info(f"[{robot_name}] Sent task {task_id}: "
                               f"pick{pick_xy} → drop{drop_xy}")
        return task_id

    def inject_fault(self, robot_name: str):
        if robot_name not in self.fault_clients:
            self.get_logger().error(f"No fault client for {robot_name}")
            return
        client = self.fault_clients[robot_name]
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(f"[{robot_name}] inject_fault service not available.")
            return
        req = SetBool.Request()
        req.data = True
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.result():
            self.get_logger().warn(f"[{robot_name}] Fault injected!")

    def total_completions(self) -> int:
        with self._lock:
            return sum(len(v) for v in self.completions.values())

    def print_status(self, total_tasks: int):
        with self._lock:
            print(f"\n{'='*62}")
            print(f"  FMS Status   completed:{self.total_completions()}/{total_tasks}")
            print(f"{'='*62}")
            for name in self.robot_names:
                st = self.status.get(name, {})
                state = st.get("state_name", "?")
                soc   = st.get("soc",   0.0)
                task  = st.get("task",  "-") or "-"
                done  = len(self.completions[name])
                print(f"  {name}: {state:<11} SOC={soc:5.1f}%  "
                      f"task={task:<16} done={done}")
            print(f"{'='*62}")


def main():
    parser = argparse.ArgumentParser(description="FMS task injector")
    parser.add_argument("--robots", nargs="+", default=["1", "2", "3", "4"],
                        help="Robot numbers to send tasks to (default: 1 2 3 4)")
    parser.add_argument("--count", type=int, default=1,
                        help="Number of tasks per robot (default: 1)")
    parser.add_argument("--interval", type=float, default=30.0,
                        help="Seconds between successive tasks per robot (default: 30)")
    parser.add_argument("--pick", nargs=2, type=float, metavar=("X", "Y"),
                        help="Override pick location (all robots)")
    parser.add_argument("--drop", nargs=2, type=float, metavar=("X", "Y"),
                        help="Override drop location (all robots)")
    parser.add_argument("--fault", metavar="ROBOT_NAME",
                        help="Inject fault into this robot 15 s after first task sent")
    args = parser.parse_args()

    robot_names = [f"robot_{n}" for n in args.robots]
    total_tasks = len(robot_names) * args.count

    rclpy.init()
    monitor = FleetMonitor(robot_names)

    # Spin in background
    spin_thread = threading.Thread(target=rclpy.spin, args=(monitor,), daemon=True)
    spin_thread.start()

    print(f"\n{'='*62}")
    print(f"  FMS Task Injector")
    print(f"  Robots: {', '.join(robot_names)}")
    print(f"  Tasks per robot: {args.count}")
    print(f"  Total tasks: {total_tasks}")
    print(f"{'='*62}\n")

    # Give action servers a moment to be ready
    time.sleep(2.0)

    sent_tasks: list[tuple[str, str]] = []
    t_start = time.time()

    for i in range(args.count):
        for name in robot_names:
            pick_xy = tuple(args.pick) if args.pick else PICK_STATIONS.get(name, (14.0, 0.0))
            drop_xy = tuple(args.drop) if args.drop else DROP_STATIONS.get(name, (-5.0, 0.0))
            task_id = monitor.send_task(name, pick_xy, drop_xy)
            sent_tasks.append((name, task_id))
            time.sleep(0.2)  # brief stagger between robots

        if i < args.count - 1:
            # Wait between rounds
            time.sleep(args.interval)

    # Optionally inject fault
    if args.fault:
        fault_delay = 15.0
        print(f"\n  [fault] Will inject fault into {args.fault} in {fault_delay:.0f}s...")
        time.sleep(fault_delay)
        monitor.inject_fault(args.fault)

    # Wait for all completions, polling frequently so we notice completion
    # as soon as it happens, but only print the status table every ~10s.
    print(f"\n  Waiting for {total_tasks} task completions...\n")
    deadline = time.time() + 600.0  # 10 minute timeout
    last_print = 0.0
    try:
        while monitor.total_completions() < total_tasks and time.time() < deadline:
            now = time.time()
            if now - last_print >= 10.0:
                monitor.print_status(total_tasks)
                last_print = now
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n  [interrupted by user]")

    elapsed = time.time() - t_start
    done = monitor.total_completions()
    try:
        monitor.print_status(total_tasks)
        print(f"\n  {'='*58}")
        print(f"  FINAL: {done}/{total_tasks} tasks completed  (elapsed: {elapsed:.1f}s)")
        print(f"  {'='*58}\n")
    except KeyboardInterrupt:
        pass

    try:
        rclpy.shutdown()
    except Exception:
        pass
    spin_thread.join(timeout=3.0)
    sys.exit(0 if done == total_tasks else 1)


if __name__ == "__main__":
    main()
