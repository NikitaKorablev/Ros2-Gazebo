#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped

def tri(x, a, b, c):
    if x <= a or x >= c:
        return 0.0
    if x == b:
        return 1.0
    return (x - a) / (b - a) if x < b else (c - x) / (c - b)

class FuzzyController(Node):
    def __init__(self):
        super().__init__('fuzzy_controller')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, qos)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
    
    def sector_min(self, scan, a_min, a_max):
        vals = []
        for i, r in enumerate(scan.ranges):
            ang = scan.angle_min + i * scan.angle_increment
            ang = (ang + math.pi) % (2 * math.pi) - math.pi
            if a_min <= ang <= a_max and math.isfinite(r) and r > 0.0:
                vals.append(r)
        return min(vals) if vals else 1.5

    def fuzzify(self, d):
        return {
            'close':  tri(d, 0.0, 0.0, 0.7),
            'medium': tri(d, 0.3, 0.7, 1.2),
            'far':    tri(d, 0.8, 1.5, 3.0)
        }
    
    def scan_cb(self, scan):
        d_front = self.sector_min(scan, -0.35, 0.35)
        d_left = self.sector_min(scan, 0.35, 1.40)
        d_right = self.sector_min(scan, -1.40, -0.35)

        F = self.fuzzify(d_front)
        L = self.fuzzify(d_left)
        R = self.fuzzify(d_right)

        rules = [
            (F['far'],                      0.18, 0.0),
            (F['medium'],                   0.08, 0.0),

            (min(F['close'], L['far']),     0.0, +1.4),
            (min(F['close'], L['medium']),  0.0, +0.9),
            (min(F['close'], R['far']),     0.0, -1.4),
            (min(F['close'], R['medium']),  0.0, -0.9),
        ]

        num_v = sum(w * s for w, s, _t in rules)
        num_w = sum(w * t for w, _s, t in rules)
        den = sum(w for w, _s, _t in rules)
        if den < 1e-6:
            v_out, w_out = 0.0, 0.0
        else:
            v_out = num_v / den
            w_out = num_w / den

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(v_out)
        msg.twist.angular.z = float(w_out)
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = FuzzyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()