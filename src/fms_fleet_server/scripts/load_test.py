#!/usr/bin/env python3
"""
load_test.py — Paced gRPC load test for the FleetService (Phase 3.5/3.6).

Repeatedly submits SubmitTask requests for pick->drop tasks, alternating
across whichever robots are IDLE, and polls GetTaskStatus until every
submitted task reaches a terminal state (COMPLETED/FAILED/CANCELLED).

At most one SubmitTask call is issued per poll cycle. This is required
because FleetServerNode::select_robot scores robots using a cached
robot_status snapshot that lags behind dispatch by up to one publish
interval, and robot_agent_node holds only a single pending-task slot
(an earlier unconsumed assignment is dropped with "Dropping queued task"
if a second one arrives before it's picked up). Submitting faster than
one task per poll cycle can therefore route multiple tasks to the same
robot and silently drop all but the last.

Usage:
  load_test.py --num-tasks 50
  load_test.py --num-tasks 10 --poll-interval 1.0 --addr localhost:50051
  load_test.py --num-tasks 50 --output /tmp/load_test_results.txt

Requires generated gRPC stubs (fleet_pb2.py / fleet_pb2_grpc.py) on
PYTHONPATH — see generate_grpc_stubs.sh in this directory, or
docs/LOAD_TESTING.md for full setup instructions.
"""

import argparse
import time

import grpc
import fleet_pb2
import fleet_pb2_grpc

STATE_IDLE = 0  # fms_msgs/RobotStatus.STATE_IDLE

DEFAULT_PICK_DROP_PAIRS = [
    ((14.0, -14.0), (10.0, -14.0)),
    ((14.0, -10.0), (10.0, -10.0)),
    ((14.0, -5.0), (10.0, -5.0)),
]

TERMINAL_STATES = ("COMPLETED", "FAILED", "CANCELLED")


def submit_task(stub, pick, drop, deadline_secs):
    req = fleet_pb2.SubmitTaskRequest(
        task_type=0,  # TASK_PICK
        pick_pose=fleet_pb2.Pose2D(x=pick[0], y=pick[1]),
        drop_pose=fleet_pb2.Pose2D(x=drop[0], y=drop[1]),
        payload_id="",
        priority=1,
        deadline_secs=deadline_secs,
    )
    return stub.SubmitTask(req)


def run(args):
    channel = grpc.insecure_channel(args.addr)
    stub = fleet_pb2_grpc.FleetServiceStub(channel)

    pairs = DEFAULT_PICK_DROP_PAIRS

    submitted = []       # list of task_ids, in submission order
    busy_robot = set()   # robots currently holding one of our in-flight tasks
    task_terminal = {}   # task_id -> terminal status string
    task_robot = {}       # task_id -> robot_id

    start = time.time()
    i = 0

    def all_terminal():
        return all(task_terminal.get(t) in TERMINAL_STATES for t in submitted)

    while len(submitted) < args.num_tasks or not all_terminal():
        try:
            status = stub.GetFleetStatus(fleet_pb2.GetFleetStatusRequest())
        except grpc.RpcError as e:
            print(f"  [transient error in GetFleetStatus: {e.code()}], retrying...", flush=True)
            time.sleep(args.poll_interval)
            continue

        idle_robots = [r.robot_id for r in status.robots if r.state == STATE_IDLE]
        for r in status.robots:
            if r.robot_id in busy_robot and r.state == STATE_IDLE:
                busy_robot.discard(r.robot_id)

        if len(submitted) < args.num_tasks:
            for robot_id in idle_robots:
                if robot_id in busy_robot:
                    continue
                pick, drop = pairs[i % len(pairs)]
                try:
                    resp = submit_task(stub, pick, drop, args.deadline_secs)
                except grpc.RpcError as e:
                    print(f"  [transient error in SubmitTask: {e.code()}], will retry", flush=True)
                    continue
                if resp.accepted:
                    submitted.append(resp.task_id)
                    busy_robot.add(resp.robot_id)
                    task_terminal[resp.task_id] = "ASSIGNED"
                    task_robot[resp.task_id] = resp.robot_id
                    elapsed = time.time() - start
                    print(f"[{elapsed:7.1f}s] [{len(submitted)}/{args.num_tasks}] "
                          f"submitted task_id={resp.task_id} -> {resp.robot_id}", flush=True)
                    i += 1
                else:
                    print(f"  SubmitTask rejected: {resp.message}", flush=True)
                # Submit at most one task per poll cycle (see module docstring).
                break

        for task_id in submitted:
            if task_terminal.get(task_id) in TERMINAL_STATES:
                continue
            try:
                resp = stub.GetTaskStatus(fleet_pb2.GetTaskStatusRequest(task_id=task_id))
            except grpc.RpcError as e:
                print(f"  [transient error in GetTaskStatus({task_id}): {e.code()}], will retry", flush=True)
                continue
            if resp.found and resp.status in TERMINAL_STATES:
                task_terminal[task_id] = resp.status
                elapsed = time.time() - start
                print(f"[{elapsed:7.1f}s] task {task_id} -> {resp.status} "
                      f"(result={resp.result}, duration={resp.duration_secs:.2f}s)", flush=True)

        time.sleep(args.poll_interval)

    elapsed = time.time() - start
    completed = sum(1 for s in task_terminal.values() if s == "COMPLETED")
    failed = sum(1 for s in task_terminal.values() if s in ("FAILED", "CANCELLED"))
    print(f"\nDONE in {elapsed:.1f}s: {completed} COMPLETED, {failed} FAILED/CANCELLED "
          f"out of {len(submitted)} submitted", flush=True)

    if args.output:
        with open(args.output, "w") as f:
            for t in submitted:
                f.write(f"{t} {task_robot.get(t)} {task_terminal.get(t)}\n")
        print(f"Wrote per-task results to {args.output}", flush=True)

    status = stub.GetFleetStatus(fleet_pb2.GetFleetStatusRequest())
    print("\nFinal fleet status:")
    for r in status.robots:
        print(f"  {r.robot_id}: state={r.state_str} battery_soc={r.battery_soc:.1f} "
              f"current_task_id={r.current_task_id!r}")

    return 0 if failed == 0 and len(submitted) == args.num_tasks else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--addr", default="localhost:50051",
                         help="FleetService gRPC address (default: localhost:50051)")
    parser.add_argument("--num-tasks", type=int, default=50,
                         help="Total number of pick->drop tasks to submit (default: 50)")
    parser.add_argument("--poll-interval", type=float, default=2.0,
                         help="Seconds between GetFleetStatus/GetTaskStatus polls (default: 2.0)")
    parser.add_argument("--deadline-secs", type=float, default=300.0,
                         help="deadline_secs to set on each SubmitTaskRequest (default: 300.0)")
    parser.add_argument("--output", default=None,
                         help="Optional path to write 'task_id robot_id status' lines on completion")
    args = parser.parse_args()

    raise SystemExit(run(args))


if __name__ == "__main__":
    main()
