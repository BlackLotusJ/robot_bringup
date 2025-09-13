#!/usr/bin/env python3
"""
Mapless Mission Controller with Dijkstra's Algorithm + Visualization
Publishes OccupancyGrid (/local_map) and Path (/planned_path) for RViz2
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
import time
import math
import numpy as np
import heapq
from collections import defaultdict


class LocalGrid:
    def __init__(self, size=100, resolution=0.1):
        self.size = size
        self.resolution = resolution
        self.grid = np.zeros((size, size), dtype=np.float32)
        self.origin_x = size // 2
        self.origin_y = size // 2

    def world_to_grid(self, x, y):
        grid_x = int(x / self.resolution) + self.origin_x
        grid_y = int(y / self.resolution) + self.origin_y
        return grid_x, grid_y

    def grid_to_world(self, grid_x, grid_y):
        x = (grid_x - self.origin_x) * self.resolution
        y = (grid_y - self.origin_y) * self.resolution
        return x, y

    def is_valid(self, grid_x, grid_y):
        return 0 <= grid_x < self.size and 0 <= grid_y < self.size

    def update_from_lidar(self, robot_x, robot_y, robot_yaw, ranges, angle_min, angle_increment):
        for i, r in enumerate(ranges):
            if math.isinf(r) or r <= 0:
                continue

            angle = robot_yaw + angle_min + i * angle_increment

            # Mark cells along the ray as free
            steps = int(r / self.resolution)
            for step in range(steps):
                ray_x = robot_x + step * self.resolution * math.cos(angle)
                ray_y = robot_y + step * self.resolution * math.sin(angle)
                gx, gy = self.world_to_grid(ray_x, ray_y)

                if self.is_valid(gx, gy):
                    self.grid[gy, gx] = 0.0  # Free space

            # Mark endpoint as occupied if within range
            if r < 5.0:
                end_x = robot_x + r * math.cos(angle)
                end_y = robot_y + r * math.sin(angle)
                gx, gy = self.world_to_grid(end_x, end_y)

                if self.is_valid(gx, gy):
                    self.grid[gy, gx] = 1.0  # Occupied


class DijkstraPlanner:
    def __init__(self, grid):
        self.grid = grid

    def get_neighbors(self, x, y):
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if self.grid.is_valid(nx, ny):
                    neighbors.append((nx, ny))
        return neighbors

    def get_cost(self, x1, y1, x2, y2):
        if self.grid.grid[y2, x2] > 0.8:
            return float('inf')
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        if dx == 1 and dy == 1:
            cost = 1.414
        else:
            cost = 1.0
        if self.grid.grid[y2, x2] == 0.5:
            cost += 2.0
        return cost

    def plan_path(self, start_x, start_y, goal_x, goal_y):
        if not (self.grid.is_valid(start_x, start_y) and self.grid.is_valid(goal_x, goal_y)):
            return []
        pq = [(0, start_x, start_y)]
        distances = defaultdict(lambda: float('inf'))
        distances[(start_x, start_y)] = 0
        previous = {}
        visited = set()

        while pq:
            current_cost, x, y = heapq.heappop(pq)
            if (x, y) in visited:
                continue
            visited.add((x, y))

            if x == goal_x and y == goal_y:
                path = []
                current = (x, y)
                while current in previous:
                    path.append(current)
                    current = previous[current]
                path.append((start_x, start_y))
                return path[::-1]

            for nx, ny in self.get_neighbors(x, y):
                if (nx, ny) in visited:
                    continue
                cost = self.get_cost(x, y, nx, ny)
                if cost == float('inf'):
                    continue
                new_distance = current_cost + cost
                if new_distance < distances[(nx, ny)]:
                    distances[(nx, ny)] = new_distance
                    previous[(nx, ny)] = (x, y)
                    heapq.heappush(pq, (new_distance, nx, ny))
        return []


class DijkstraMissionNode(Node):
    def __init__(self):
        super().__init__('dijkstra_mission_node')

        # Parameters
        self.declare_parameter('v_max', 0.3)
        self.declare_parameter('w_max', 0.6)
        self.declare_parameter('safe_distance', 0.4)
        self.declare_parameter('retreat_speed', -0.2)
        self.declare_parameter('stop_duration', 5.0)
        self.declare_parameter('path_following_tolerance', 0.2)
        self.declare_parameter('exploration_radius', 3.0)

        self.balloon_sequence = ["pink", "green", "yellow"]
        self.sequence_index = 0

        self.local_grid = LocalGrid(size=200, resolution=0.05)
        self.planner = DijkstraPlanner(self.local_grid)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.current_yaw = 0.0
        self.target_detected = None
        self.visited_balloons = set()
        self.min_distance_ahead = None

        self.current_path = []
        self.path_index = 0
        self.goal_position = None
        self.mission_state = "exploring"
        self.in_stop_phase = False
        self.stop_start_time = None

        self.last_exploration_time = time.time()

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/local_map', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        # Subscribers
        self.create_subscription(String, '/object_info', self.detection_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)

        # Timers
        self.create_timer(0.1, self.control_loop)
        self.create_timer(0.5, self.update_exploration_goals)
        self.create_timer(1.0, self.publish_map)   # publish map every second

        self.get_logger().info("✅ Dijkstra Mission Node with Visualization started...")

    # === Visualization ===
    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.info = MapMetaData()
        msg.info.resolution = self.local_grid.resolution
        msg.info.width = self.local_grid.size
        msg.info.height = self.local_grid.size
        msg.info.origin.position.x = -(self.local_grid.origin_x * self.local_grid.resolution)
        msg.info.origin.position.y = -(self.local_grid.origin_y * self.local_grid.resolution)
        msg.info.origin.orientation.w = 1.0

        flat = []
        for row in self.local_grid.grid:
            for val in row:
                if val == 0.5:
                    flat.append(-1)
                elif val >= 0.8:
                    flat.append(100)
                else:
                    flat.append(0)
        msg.data = flat
        self.map_pub.publish(msg)

    def publish_path(self):
        if not self.current_path:
            return
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        for (x, y) in self.current_path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    # === Callbacks + logic ===
    def detection_cb(self, msg: String):
        data = msg.data.lower()
        current_target = self.get_current_target()
        if current_target and current_target in data:
            if data not in self.visited_balloons:
                self.target_detected = data
                self.mission_state = "approaching_balloon"
                self.plan_path_to_position(self.robot_x, self.robot_y)
                self.get_logger().info(f"🎯 Target balloon detected: {data}")

    def lidar_cb(self, msg: LaserScan):
        self.local_grid.update_from_lidar(
            self.robot_x, self.robot_y, self.current_yaw,
            msg.ranges, msg.angle_min, msg.angle_increment
        )
        center_index = len(msg.ranges) // 2
        d = msg.ranges[center_index]
        self.min_distance_ahead = 10.0 if math.isinf(d) else d

    def imu_cb(self, msg: Imu):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def get_current_target(self):
        if self.sequence_index < len(self.balloon_sequence):
            return self.balloon_sequence[self.sequence_index]
        return None

    def plan_path_to_position(self, goal_x, goal_y):
        start_gx, start_gy = self.local_grid.world_to_grid(self.robot_x, self.robot_y)
        goal_gx, goal_gy = self.local_grid.world_to_grid(goal_x, goal_y)
        path_grid = self.planner.plan_path(start_gx, start_gy, goal_gx, goal_gy)
        if path_grid:
            self.current_path = []
            for gx, gy in path_grid:
                world_x, world_y = self.local_grid.grid_to_world(gx, gy)
                self.current_path.append((world_x, world_y))
            self.path_index = 0
            self.mission_state = "following_path"
            self.get_logger().info(f"📍 Path planned with {len(self.current_path)} waypoints")
            self.publish_path()
        else:
            self.get_logger().warn("❌ No path found to goal")

    # === Exploration + Control ===
    def update_exploration_goals(self):
        if self.mission_state == "exploring" and time.time() - self.last_exploration_time > 5.0:
            angle = np.random.uniform(0, 2 * math.pi)
            dist = self.get_parameter('exploration_radius').value
            goal_x = self.robot_x + dist * math.cos(angle)
            goal_y = self.robot_y + dist * math.sin(angle)
            self.plan_path_to_position(goal_x, goal_y)
            self.last_exploration_time = time.time()

    def follow_path(self):
        if not self.current_path or self.path_index >= len(self.current_path):
            self.mission_state = "exploring"
            return

        goal_x, goal_y = self.current_path[self.path_index]
        dx = goal_x - self.robot_x
        dy = goal_y - self.robot_y
        distance = math.sqrt(dx**2 + dy**2)

        if distance < self.get_parameter('path_following_tolerance').value:
            self.path_index += 1
            return

        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - self.current_yaw

        v = self.get_parameter('v_max').value
        w = self.get_parameter('w_max').value * angle_error

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def control_loop(self):
        if self.min_distance_ahead is not None and self.min_distance_ahead < self.get_parameter('safe_distance').value:
            cmd = Twist()
            cmd.linear.x = self.get_parameter('retreat_speed').value
            self.cmd_pub.publish(cmd)
            return

        if self.mission_state == "following_path":
            self.follow_path()
        elif self.mission_state == "exploring":
            cmd = Twist()
            cmd.linear.x = 0.1
            cmd.angular.z = 0.2
            self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DijkstraMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
