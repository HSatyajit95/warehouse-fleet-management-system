#!/usr/bin/env python3
"""
warehouse_markers.py — Publishes static visualization markers for all
warehouse features so they appear in RViz alongside the occupancy map.

Topic: /warehouse_locations  (visualization_msgs/MarkerArray, transient_local)

Markers:
  - Blue cylinders  = charge docks  (matching warehouse.sdf charge_dock_* poses)
  - Green cylinders = pick stations (matching warehouse.sdf pick_station_* poses)
  - Orange cylinders= drop zones    (aisle east of shelf_D)
  - White text labels above each marker
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration


FRAME_ID = "map"

# Gazebo SDF positions matching warehouse.sdf model poses exactly.
CHARGE_DOCKS = [
    (-22.0, -15.0, "Charger 1"),  # charge_dock_4
    (-22.0, -10.0, "Charger 2"),  # charge_dock_3
    (-22.0,  10.0, "Charger 3"),  # charge_dock_2
    (-22.0,  15.0, "Charger 4"),  # charge_dock_1
]

PICK_STATIONS = [
    # x=14.0 = robot approach position (1 m west of station west face at x=15)
    (14.0, -14.0, "Pick 1"),
    (14.0,  -6.0, "Pick 2"),
    (14.0,   6.0, "Pick 3"),
    (14.0,  14.0, "Pick 4"),
]

DROP_ZONES = [
    (10.0, -14.0, "Drop 1"),
    (10.0,  -6.0, "Drop 2"),
    (10.0,   6.0, "Drop 3"),
    (10.0,  14.0, "Drop 4"),
]


def _cylinder(marker_id: int, x: float, y: float,
              r: float, g: float, b: float) -> Marker:
    m = Marker()
    m.header.frame_id = FRAME_ID
    m.ns = "warehouse_icons"
    m.id = marker_id
    m.type = Marker.CYLINDER
    m.action = Marker.ADD
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = 0.4
    m.pose.orientation.w = 1.0
    m.scale.x = 0.9
    m.scale.y = 0.9
    m.scale.z = 0.8
    m.color.r = r
    m.color.g = g
    m.color.b = b
    m.color.a = 0.85
    m.lifetime = Duration()  # 0 → permanent
    return m


def _label(marker_id: int, x: float, y: float, text: str) -> Marker:
    m = Marker()
    m.header.frame_id = FRAME_ID
    m.ns = "warehouse_labels"
    m.id = marker_id
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = 1.1
    m.pose.orientation.w = 1.0
    m.scale.z = 0.7
    m.color.r = 1.0
    m.color.g = 1.0
    m.color.b = 1.0
    m.color.a = 1.0
    m.text = text
    m.lifetime = Duration()
    return m


class WarehouseMarkersNode(Node):
    def __init__(self):
        super().__init__("warehouse_markers")
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pub = self.create_publisher(MarkerArray, "/warehouse_locations", qos)
        # Brief delay lets DDS endpoints register before first publish.
        self._timer = self.create_timer(1.0, self._publish_once)

    def _publish_once(self):
        self._timer.cancel()
        msg = MarkerArray()
        mid = 0

        for x, y, label in CHARGE_DOCKS:
            msg.markers.append(_cylinder(mid,       x, y, 0.2, 0.4, 1.0))  # blue
            msg.markers.append(_label(mid + 100,    x, y, label))
            mid += 1

        for x, y, label in PICK_STATIONS:
            msg.markers.append(_cylinder(mid,       x, y, 0.1, 0.85, 0.1))  # green
            msg.markers.append(_label(mid + 100,    x, y, label))
            mid += 1

        for x, y, label in DROP_ZONES:
            msg.markers.append(_cylinder(mid,       x, y, 1.0, 0.5, 0.0))   # orange
            msg.markers.append(_label(mid + 100,    x, y, label))
            mid += 1

        self._pub.publish(msg)
        self.get_logger().info("Warehouse location markers published.")


def main():
    rclpy.init()
    node = WarehouseMarkersNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
