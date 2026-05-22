#!/usr/bin/env python3
import heapq
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        self.declare_parameter('goal_x', 1.5)
        self.declare_parameter('goal_y', 1.5)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)

        self.map_msg = None
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_cb, 10)

        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.timer = self.create_timer(1.0, self.plan_and_publish)

    def map_cb(self, msg):
        self.map_msg = msg
    
    def world_to_grid(self, x, y):
        info = self.map_msg.info
        gx = int((x - info.origin.position.x) / info.resolution)
        gy = int((y - info.origin.position.y) / info.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        info = self.map_msg.info
        x = gx * info.resolution + info.origin.position.x
        y = gy * info.resolution + info.origin.position.y
        return x, y

    def is_free(self, gx, gy):
        info = self.map_msg.info
        if gx < 0 or gy < 0 or gx >= info.width or gy >= info.height:
            return False
        v = self.map_msg.data[gy * info.width + gx]
        return 0 <= v < 50
    
    def neighbors(self, cell):
        gx, gy = cell
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            nx, ny = gx + dx, gy + dy
            if self.is_free(nx, ny):
                yield (nx, ny)

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])
    
    def a_star(self, start, goal):
        open_set = [(0.0, start)]
        came_from = {}
        g_score = {start: 0.0}
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return list(reversed(path))
            for nb in self.neighbors(current):
                tentative = g_score[current] + 1.0
                if tentative < g_score.get(nb, float('inf')):
                    came_from[nb] = current
                    g_score[nb] = tentative
                    f = tentative + self.heuristic(nb, goal)
                    heapq.heappush(open_set, (f, nb))
        return []

    def plan_and_publish(self):
        if self.map_msg is None:
            return
        sx = self.get_parameter('start_x').value
        sy = self.get_parameter('start_y').value
        gx = self.get_parameter('goal_x').value
        gy = self.get_parameter('goal_y').value
        start = self.world_to_grid(sx, sy)
        goal = self.world_to_grid(gx, gy)
        if not self.is_free(*start) or not self.is_free(*goal):
            self.get_logger().warn('Start or goal cell is not free')
            return
        cells = self.a_star(start, goal)
        if not cells:
            self.get_logger().warn('No path found')
            return
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for c in cells:
            wx, wy = self.grid_to_world(*c)
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)


def main():
    rclpy.init()
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()