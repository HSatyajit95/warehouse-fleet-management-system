#!/usr/bin/env python3
"""
task_request_amqp.py — Publish an unassigned task to the "tasks" exchange
(routing key "incoming") for the fleet server's RabbitMQ allocator (Phase 3.4)
to consume from "tasks.incoming" and assign to a robot.

Usage:
  task_request_amqp.py --pick 14.0 -14.0 --drop 10.0 -14.0
  task_request_amqp.py --pick 14.0 -14.0 --drop 10.0 -14.0 --host localhost
"""

import argparse
import json
import uuid

import pika


def main():
    parser = argparse.ArgumentParser(description="FMS RabbitMQ task request")
    parser.add_argument("--pick", nargs=2, type=float, required=True, metavar=("X", "Y"))
    parser.add_argument("--drop", nargs=2, type=float, required=True, metavar=("X", "Y"))
    parser.add_argument("--priority", type=int, default=1)
    parser.add_argument("--deadline", type=float, default=300.0)
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=5672)
    parser.add_argument("--user", default="guest")
    parser.add_argument("--password", default="guest")
    args = parser.parse_args()

    task_id = f"task_{uuid.uuid4().hex[:8]}"
    body = {
        "task_id": task_id,
        "task_type": 0,  # TASK_PICK
        "pick": {"x": args.pick[0], "y": args.pick[1]},
        "drop": {"x": args.drop[0], "y": args.drop[1]},
        "priority": args.priority,
        "deadline_secs": args.deadline,
    }

    credentials = pika.PlainCredentials(args.user, args.password)
    connection = pika.BlockingConnection(
        pika.ConnectionParameters(host=args.host, port=args.port, credentials=credentials))
    channel = connection.channel()

    channel.basic_publish(
        exchange="tasks",
        routing_key="incoming",
        body=json.dumps(body),
        properties=pika.BasicProperties(content_type="application/json", delivery_mode=2),
    )
    print(f"Published task_request {task_id} to tasks/incoming: "
          f"pick{tuple(args.pick)} -> drop{tuple(args.drop)}")

    connection.close()


if __name__ == "__main__":
    main()
