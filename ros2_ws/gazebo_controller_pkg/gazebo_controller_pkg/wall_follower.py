#!/usr/bin/env python3
import cmd

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from rclpy.qos import QoSProfile, ReliabilityPolicy
import math


class WallFollower(Node):

    FIND_WALL = 0
    TURN = 1
    FOLLOW = 2

    def __init__(self):
        super().__init__('wall_follower')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.cmd_pub = self.create_publisher(
            TwistStamped, '/cmd_vel', 10)

        self.state = self.FIND_WALL

        self.front = 10.0
        self.left = 10.0
        self.right = 10.0

        self.path = []

        self.safe_dist = 0.5
        self.wall_dist = 0.7

        self.timer = self.create_timer(0.1, self.control_loop)

    def scan_callback(self, msg):
        def safe_min(values):
            vals = [v for v in values if not math.isinf(v)]
            return min(vals) if vals else 10.0

        self.front = safe_min(list(msg.ranges[0:15]) + list(msg.ranges[345:360]))
        self.left = safe_min(msg.ranges[80:100])
        self.right = safe_min(msg.ranges[260:280])

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        self.path.append((p.x, p.y))

    def control_loop(self):
        cmd = TwistStamped()

        if self.state == self.FIND_WALL:
            cmd.twist.linear.x = 0.3

            if self.front < self.safe_dist:
                self.state = self.TURN

        elif self.state == self.TURN:
            cmd.twist.angular.z = 0.5

            if self.front > self.wall_dist:
                self.state = self.FOLLOW

        elif self.state == self.FOLLOW:
            cmd.twist.linear.x = 0.25

            KP = 1.5
            MAX_ANG = 1.0

            error = self.right - 0.6
            cmd.twist.angular.z = max(-MAX_ANG, min(MAX_ANG, -KP * error))

            if self.front < self.safe_dist:
                self.state = self.TURN

            elif self.right > 1.0:
                self.state = self.FIND_WALL

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"STATE: {self.state} | F={self.front:.2f} R={self.right:.2f}",
            throttle_duration_sec=1.0
        )
    
    def destroy_node(self):
        with open('/ros2_ws/gazebo_controller_pkg/path.txt', 'w') as f:
            for x, y in self.path:
                f.write(f"{x},{y}\n")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()