#!/usr/bin/env python3
"""
odom_to_tf — Two TF transforms from one node.

1. DYNAMIC  odom→base_footprint: re-publishes nav_msgs/Odometry as a TF
   transform on /tf using tf2_ros.TransformBroadcaster (correct QoS for
   tf2::Buffer).  Replaces the ros_gz_bridge TF bridge which uses a
   mismatched QoS profile.

2. STATIC   map→odom: publishes a one-shot static transform on /tf_static
   (TransientLocal) so that Nav2 has an immediate map→odom transform without
   needing AMCL to converge.  Derived from the robot's known spawn position
   in the map frame.

Parameters
----------
odom_topic      topic to subscribe (default /odom)
parent_frame    TF parent of dynamic transform  (default odom)
child_frame     TF child  of dynamic transform  (default base_footprint)
map_frame_id    global fixed frame               (default map)
map_x / map_y / map_yaw  spawn position in map frame (default 0 0 0)
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')

        self.declare_parameter('odom_topic',   '/odom')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame',  'base_footprint')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('map_x',        0.0)
        self.declare_parameter('map_y',        0.0)
        self.declare_parameter('map_yaw',      0.0)

        topic  = self.get_parameter('odom_topic').value
        self.p = self.get_parameter('parent_frame').value
        self.c = self.get_parameter('child_frame').value
        mf     = self.get_parameter('map_frame_id').value
        mx     = self.get_parameter('map_x').value
        my     = self.get_parameter('map_y').value
        myaw   = self.get_parameter('map_yaw').value

        # Dynamic transform: odom → base_footprint
        self.br  = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, topic, self._cb, 10)

        # Static transform: map → odom  (spawn position in map frame)
        # Must be an instance variable — if local, CPython ref-count drops to
        # zero when __init__ returns, the publisher is destroyed, and CycloneDDS
        # removes the transient_local cache so late subscribers (RViz) see nothing.
        self.static_br = StaticTransformBroadcaster(self)
        st = TransformStamped()
        st.header.stamp       = self.get_clock().now().to_msg()
        st.header.frame_id    = mf
        st.child_frame_id     = self.p          # e.g. "robot_1/odom"
        st.transform.translation.x = mx
        st.transform.translation.y = my
        st.transform.translation.z = 0.0
        cy = math.cos(myaw * 0.5)
        sy = math.sin(myaw * 0.5)
        st.transform.rotation.w = cy
        st.transform.rotation.x = 0.0
        st.transform.rotation.y = 0.0
        st.transform.rotation.z = sy
        self.static_br.sendTransform(st)

        self.get_logger().info(
            f'odom_to_tf: {topic} → TF {self.p}→{self.c} | '
            f'static {mf}→{self.p} @ ({mx:.2f},{my:.2f},{myaw:.2f})')

    def _cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp    = msg.header.stamp
        t.header.frame_id = self.p
        t.child_frame_id  = self.c
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z
        t.transform.rotation      = q
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
