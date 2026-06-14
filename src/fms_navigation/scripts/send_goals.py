#!/usr/bin/env python3
"""
send_goals.py — Send navigation goals to multiple robots simultaneously.

Usage:
  python3 send_goals.py                    # uses DEFAULT_GOALS below
  python3 send_goals.py --robots 1 2      # only robot_1 and robot_2
  python3 send_goals.py --goal_set 1      # use goal set 1 (pick stations)
  python3 send_goals.py --goal_set 2      # use goal set 2 (charge docks)
  python3 send_goals.py --goal_set 3      # use goal set 3 (through aisles)

Coordinate system: MAP frame = Gazebo world frame
  Warehouse bounds: X(-25 to +25),  Y(-20 to +20)
  Robot spawns:     robot_1(-5,-17) robot_2(-5,-14) robot_3(-5,-11) robot_4(-5,-8)
  Pick stations:    X ≈ +14,  charge docks: X ≈ -20
"""

import argparse
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

# ── Goal sets ────────────────────────────────────────────────────────────────
# Each set maps robot_name → (x, y, yaw_deg)
# yaw_deg=0 → facing +X direction

GOAL_SETS = {
    0: {   # Default: move each robot to the pick-station aisle (right side)
        "robot_1": (14.0, -17.0, 0.0),
        "robot_2": (14.0, -14.0, 0.0),
        "robot_3": (14.0, -11.0, 0.0),
        "robot_4": (14.0,  -8.0, 0.0),
    },
    1: {   # Goal set 1: pick stations (right wall, each robot's own row)
        "robot_1": (14.0, -17.0, 0.0),
        "robot_2": (14.0, -14.0, 0.0),
        "robot_3": (14.0, -11.0, 0.0),
        "robot_4": (14.0,  -8.0, 0.0),
    },
    2: {   # Goal set 2: charge docks (left side)
        "robot_1": (-20.0, -17.0, 180.0),
        "robot_2": (-20.0, -14.0, 180.0),
        "robot_3": (-20.0, -11.0, 180.0),
        "robot_4": (-20.0,  -8.0, 180.0),
    },
    3: {   # Goal set 3: navigate through aisles (different directions)
        "robot_1": (14.0,  -5.0, 0.0),    # bottom→middle, rightward
        "robot_2": (-5.0,   5.0, 90.0),   # upward through aisles
        "robot_3": ( 0.0,  10.0, 90.0),   # up through centre
        "robot_4": (-20.0,  0.0, 180.0),  # toward charge docks
    },
    4: {   # Goal set 4: return to spawn positions
        "robot_1": (-5.0, -17.0, 0.0),
        "robot_2": (-5.0, -14.0, 0.0),
        "robot_3": (-5.0, -11.0, 0.0),
        "robot_4": (-5.0,  -8.0, 0.0),
    },
}


def make_pose(x: float, y: float, yaw_deg: float = 0.0) -> PoseStamped:
    """Build a PoseStamped in the map frame."""
    import math
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    yaw = math.radians(yaw_deg)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


class RobotNavigator:
    """Sends a single NavigateToPose goal and waits for result."""

    def __init__(self, node: Node, robot_name: str):
        self.robot_name = robot_name
        self.node = node
        action_name = f"/{robot_name}/navigate_to_pose"
        self._client = ActionClient(node, NavigateToPose, action_name)
        self.status = "PENDING"
        self.result = None
        self._done_event = threading.Event()

    def send_goal(self, x: float, y: float, yaw_deg: float = 0.0):
        self.status = "WAITING_FOR_SERVER"
        print(f"  [{self.robot_name}] Waiting for action server...")

        if not self._client.wait_for_server(timeout_sec=10.0):
            print(f"  [{self.robot_name}] ✗ Action server not available!")
            self.status = "NO_SERVER"
            self._done_event.set()
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = make_pose(x, y, yaw_deg)

        print(f"  [{self.robot_name}] Sending goal → ({x:.1f}, {y:.1f}) yaw={yaw_deg:.0f}°")
        self.status = "GOAL_SENT"

        send_future = self._client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb
        )
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print(f"  [{self.robot_name}] ✗ Goal rejected")
            self.status = "REJECTED"
            self._done_event.set()
            return

        self.status = "NAVIGATING"
        print(f"  [{self.robot_name}] ✓ Goal accepted — navigating...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result()
        status_code = result.status
        # 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
        if status_code == 4:
            self.status = "SUCCEEDED"
            print(f"  [{self.robot_name}] ✅ SUCCEEDED")
        elif status_code == 5:
            self.status = "CANCELED"
            print(f"  [{self.robot_name}] ⚠  CANCELED")
        else:
            self.status = f"FAILED (code {status_code})"
            print(f"  [{self.robot_name}] ✗  {self.status}")
        self.result = result
        self._done_event.set()

    def _feedback_cb(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        # Print only when distance changes significantly
        if not hasattr(self, "_last_dist") or abs(self._last_dist - dist) > 0.5:
            self._last_dist = dist
            print(f"  [{self.robot_name}] → {dist:.1f} m remaining")

    def wait(self, timeout_sec: float = 300.0) -> bool:
        return self._done_event.wait(timeout=timeout_sec)


def main():
    parser = argparse.ArgumentParser(description="Send navigation goals to multiple robots")
    parser.add_argument("--robots", nargs="+", default=["1", "2", "3", "4"],
                        help="Robot numbers to command (default: 1 2 3 4)")
    parser.add_argument("--goal_set", type=int, default=0,
                        help="Goal set index 0-4 (default: 0 = pick stations)")
    parser.add_argument("--x", type=float, nargs="+", help="Override X per robot")
    parser.add_argument("--y", type=float, nargs="+", help="Override Y per robot")
    args = parser.parse_args()

    rclpy.init()
    node = Node("fleet_goal_sender")

    robot_names = [f"robot_{n}" for n in args.robots]
    goals = GOAL_SETS[args.goal_set]

    # Allow manual coordinate override
    if args.x and args.y:
        for i, name in enumerate(robot_names):
            xi = args.x[i] if i < len(args.x) else args.x[-1]
            yi = args.y[i] if i < len(args.y) else args.y[-1]
            goals[name] = (xi, yi, 0.0)

    print(f"\n{'='*60}")
    print(f"  FMS Fleet Goal Sender — goal set {args.goal_set}")
    print(f"  Robots: {', '.join(robot_names)}")
    print(f"{'='*60}")
    for name in robot_names:
        if name in goals:
            x, y, yaw = goals[name]
            print(f"  {name}: goal ({x:.1f}, {y:.1f})  yaw={yaw:.0f}°")
    print(f"{'='*60}\n")

    # Create navigators and start a background spin thread
    navigators = {}
    for name in robot_names:
        if name not in goals:
            print(f"  [{name}] No goal defined for this robot in set {args.goal_set}")
            continue
        navigators[name] = RobotNavigator(node, name)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Send all goals at once
    t_start = time.time()
    for name, nav in navigators.items():
        x, y, yaw = goals[name]
        nav.send_goal(x, y, yaw)

    # Wait for all to finish
    print(f"\n  Waiting for all robots to reach their goals...\n")
    for name, nav in navigators.items():
        nav.wait(timeout_sec=300.0)

    elapsed = time.time() - t_start

    # Summary
    print(f"\n{'='*60}")
    print(f"  RESULTS  (elapsed: {elapsed:.1f} s)")
    print(f"{'='*60}")
    succeeded = 0
    for name, nav in navigators.items():
        icon = "✅" if nav.status == "SUCCEEDED" else "✗"
        print(f"  {icon}  {name}: {nav.status}")
        if nav.status == "SUCCEEDED":
            succeeded += 1
    print(f"{'='*60}")
    print(f"  {succeeded}/{len(navigators)} robots reached their goals")
    print(f"{'='*60}\n")

    # Proper shutdown: signal rclpy FIRST (stops spin), then join the thread.
    # NEVER call node.destroy_node() while the spin thread is running —
    # that deletes the C++ node object from the wrong thread and causes
    # "terminate called without an active exception" + core dump.
    try:
        rclpy.shutdown()
    except Exception:
        pass
    spin_thread.join(timeout=3.0)
    sys.exit(0 if succeeded == len(navigators) else 1)


if __name__ == "__main__":
    main()
