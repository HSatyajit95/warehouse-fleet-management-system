#!/usr/bin/env python3
"""
task_request.py — Publish an unassigned TaskAssignment to /fleet/task_request
for the fleet server's task allocator (Phase 3.3) to pick a robot.

Usage:
  task_request.py --pick 14.0 -14.0 --drop 10.0 -14.0
  task_request.py --pick 14.0 -14.0 --drop 10.0 -14.0 --count 3 --interval 5
"""

import argparse
import time
import uuid

import rclpy
from rclpy.node import Node
from fms_msgs.msg import TaskAssignment
from geometry_msgs.msg import Pose


def make_pose(x: float, y: float) -> Pose:
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = 0.0
    p.orientation.w = 1.0
    return p


def main():
    parser = argparse.ArgumentParser(description="FMS fleet task request")
    parser.add_argument("--pick", nargs=2, type=float, required=True, metavar=("X", "Y"))
    parser.add_argument("--drop", nargs=2, type=float, required=True, metavar=("X", "Y"))
    parser.add_argument("--count", type=int, default=1, help="Number of task requests to send")
    parser.add_argument("--interval", type=float, default=2.0, help="Seconds between requests")
    parser.add_argument("--priority", type=int, default=1)
    parser.add_argument("--deadline", type=float, default=300.0)
    args = parser.parse_args()

    rclpy.init()
    node = Node("fms_task_request")
    pub = node.create_publisher(TaskAssignment, "/fleet/task_request", 10)
    time.sleep(1.0)  # allow discovery

    for i in range(args.count):
        msg = TaskAssignment()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.task_id = f"task_{uuid.uuid4().hex[:8]}"
        msg.task_type = TaskAssignment.TASK_PICK
        msg.pick_pose = make_pose(*args.pick)
        msg.drop_pose = make_pose(*args.drop)
        msg.priority = args.priority
        msg.deadline_secs = args.deadline
        pub.publish(msg)
        node.get_logger().info(
            f"Sent task_request {msg.task_id}: pick{tuple(args.pick)} -> drop{tuple(args.drop)}")
        if i < args.count - 1:
            time.sleep(args.interval)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
