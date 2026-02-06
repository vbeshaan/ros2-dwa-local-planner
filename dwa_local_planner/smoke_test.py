# dwa_local_planner/dwa_local_planner/smoke_test.py
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


class DwaSmokeTest(Node):
    def __init__(self):
        super().__init__('dwa_smoke_test')

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.traj_pub = self.create_publisher(MarkerArray, '/dwa_trajectories', 10)

        # Timer to publish zero cmd_vel and a simple marker
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.last_odom = None
        self.last_scan = None
        self.get_logger().info('DWA smoke-test node started')

    def odom_callback(self, msg: Odometry):
        self.last_odom = msg

    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg

    def timer_callback(self):
        # 1) Publish zero velocity (robot should stop)
        zero = Twist()
        self.cmd_pub.publish(zero)

        # 2) Publish a simple straight-line trajectory marker in front of robot
        if self.last_odom is None:
            return

        marker = Marker()
        marker.header.frame_id = 'base_footprint'
        if self.last_scan is not None:
            marker.header.stamp = self.last_scan.header.stamp
        else:
            marker.header.stamp = self.last_odom.header.stamp
        marker.ns = 'dwa_smoke'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02  # line width

        # Color (green, fully visible)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # alpha must be > 0 or it is invisible [web:178]

        # Simple 1‑meter straight line in front of robot
        from geometry_msgs.msg import Point
        p0 = Point()
        p0.x = 0.0
        p0.y = 0.0
        p0.z = 0.0
        p1 = Point()
        p1.x = 1.0
        p1.y = 0.0
        p1.z = 0.0
        marker.points = [p0, p1]

        arr = MarkerArray()
        arr.markers.append(marker)
        self.traj_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = DwaSmokeTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
