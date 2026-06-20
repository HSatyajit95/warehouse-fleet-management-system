#!/usr/bin/env python3
"""
mock_robot.py — minimal stand-in for fms_robot_agent's robot_agent_node,
used only by test_task_lifecycle.py (Phase 4.5).

Publishes RobotStatus (STATE_IDLE) so the fleet server's allocator has a
candidate to assign to, and on receiving a TaskAssignment, "executes" it
for a short fixed duration and reports TaskCompletion with RESULT_SUCCESS.
No Nav2/Gazebo/BehaviorTree involved — this exists purely to exercise the
fleet server's gRPC -> allocator -> MongoDB path without a full simulation.

Usage: mock_robot.py <robot_name> <exec_duration_secs>
"""
import sys

import rclpy
from rclpy.node import Node

from fms_msgs.msg import RobotStatus, TaskAssignment, TaskCompletion


class MockRobot(Node):
    def __init__(self, robot_name: str, exec_duration: float):
        super().__init__("mock_robot_" + robot_name)
        self.robot_name = robot_name
        self.exec_duration = exec_duration
        self.state = RobotStatus.STATE_IDLE
        self.battery_soc = 95.0

        self.status_pub = self.create_publisher(RobotStatus, f"/{robot_name}/robot_status", 10)
        self.completion_pub = self.create_publisher(TaskCompletion, f"/{robot_name}/task_completion", 10)
        self.create_subscription(TaskAssignment, f"/{robot_name}/task_assignment", self.on_task, 10)
        self.create_timer(0.5, self.publish_status)

    def publish_status(self):
        msg = RobotStatus()
        msg.robot_id = self.robot_name
        msg.state = self.state
        msg.battery_soc = self.battery_soc
        self.status_pub.publish(msg)

    def on_task(self, msg: TaskAssignment):
        self.get_logger().info(f"mock robot {self.robot_name} received task {msg.task_id}")
        self.state = RobotStatus.STATE_EXECUTING
        self.publish_status()

        timer = self.create_timer(self.exec_duration, lambda: self._complete(msg, timer))

    def _complete(self, task: TaskAssignment, timer):
        timer.cancel()
        completion = TaskCompletion()
        completion.task_id = task.task_id
        completion.robot_id = self.robot_name
        completion.result = TaskCompletion.RESULT_SUCCESS
        completion.duration_secs = self.exec_duration
        self.completion_pub.publish(completion)

        self.state = RobotStatus.STATE_IDLE
        self.publish_status()
        self.get_logger().info(f"mock robot {self.robot_name} completed task {task.task_id}")


def main():
    robot_name = sys.argv[1] if len(sys.argv) > 1 else "robot_1"
    exec_duration = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0

    rclpy.init()
    node = MockRobot(robot_name, exec_duration)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
