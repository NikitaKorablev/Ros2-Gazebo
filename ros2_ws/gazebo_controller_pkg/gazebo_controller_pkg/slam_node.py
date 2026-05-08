#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
import math

class SimpleSLAM(Node):
    def __init__(self):
        super().__init__('simple_slam')

        self.resolution = 0.05
        self.width = 200
        self.height = 200
        self.origin_x = -5.0
        self.origin_y = -5.0

        self.l_occ = 0.85
        self.l_free = 0.4
        self.l_max = 5.0
        self.l_min = -5.0

        self.grid_log_odds = np.zeros((self.width, self.height))
        
        self.robot_pose = None

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        self.timer = self.create_timer(1.0, self.publish_map)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.robot_pose = (pos.x, pos.y, yaw)

    def world_to_grid(self, x, y):
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy

    def bresenham(self, x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return points

    def scan_callback(self, msg):
        if self.robot_pose is None:
            return

        rx, ry, r_yaw = self.robot_pose
        gx0, gy0 = self.world_to_grid(rx, ry)

        angle = msg.angle_min
        for r in msg.ranges:
            if math.isinf(r) or math.isnan(r):
                angle += msg.angle_increment
                continue

            tx = rx + r * math.cos(r_yaw + angle)
            ty = ry + r * math.sin(r_yaw + angle)
            
            gx1, gy1 = self.world_to_grid(tx, ty)

            if 0 <= gx1 < self.width and 0 <= gy1 < self.height:
                line = self.bresenham(gx0, gy0, gx1, gy1)
                
                for px, py in line[:-1]:
                    if 0 <= px < self.width and 0 <= py < self.height:
                        self.grid_log_odds[px, py] = max(self.l_min, self.grid_log_odds[px, py] - self.l_free)
                
                self.grid_log_odds[gx1, gy1] = min(self.l_max, self.grid_log_odds[gx1, gy1] + self.l_occ)

            angle += msg.angle_increment

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position = Point(x=self.origin_x, y=self.origin_y, z=0.0)

        probs = (1 - 1 / (1 + np.exp(self.grid_log_odds))) * 100
        
        flat_grid = probs.astype(np.int8).T.flatten()
        
        mask = self.grid_log_odds.T.flatten() == 0
        flat_grid[mask] = -1

        msg.data = flat_grid.tolist()
        self.map_pub.publish(msg)

def main():
    rclpy.init()
    node = SimpleSLAM()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()