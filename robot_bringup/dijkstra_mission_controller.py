#!/usr/bin/env python3
"""
Mapless Mission Controller with Dijkstra's Algorithm
Builds local occupancy grid and uses Dijkstra's for path planning
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Imu
import time
import math
import numpy as np
import heapq
from collections import defaultdict


class LocalGrid:
    def __init__(self, size=100, resolution=0.1):
        self.size = size  # Grid size (size x size)
        self.resolution = resolution  # meters per cell
        self.grid = np.zeros((size, size), dtype=np.float32)  # 0 = free, 1 = occupied, 0.5 = unknown
        self.origin_x = size // 2  # Robot starts at center
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
        robot_gx, robot_gy = self.world_to_grid(robot_x, robot_y)
        
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
            
            # Mark endpoint as occupied if within reasonable range
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
        """Get valid neighboring cells (8-connected)"""
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
        """Calculate movement cost between two cells"""
        # Check if cell is occupied
        if self.grid.grid[y2, x2] > 0.8:  # Occupied
            return float('inf')
        
        # Base cost
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        if dx == 1 and dy == 1:  # Diagonal
            cost = 1.414
        else:  # Orthogonal
            cost = 1.0
            
        # Add penalty for unknown areas
        if self.grid.grid[y2, x2] == 0.5:
            cost += 2.0
            
        return cost
    
    def plan_path(self, start_x, start_y, goal_x, goal_y):
        """Plan path using Dijkstra's algorithm"""
        if not (self.grid.is_valid(start_x, start_y) and self.grid.is_valid(goal_x, goal_y)):
            return []
        
        # Priority queue: (cost, x, y)
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
            
            # Reached goal
            if x == goal_x and y == goal_y:
                # Reconstruct path
                path = []
                current = (x, y)
                while current in previous:
                    path.append(current)
                    current = previous[current]
                path.append((start_x, start_y))
                return path[::-1]  # Reverse to get start-to-goal path
            
            # Check neighbors
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
        
        return []  # No path found


class DijkstraMissionNode(Node):
    def __init__(self):
        super().__init__('dijkstra_mission_node')

        # --- Parameters ---
        self.declare_parameter('v_max', 0.3)
        self.declare_parameter('w_max', 0.6)
        self.declare_parameter('safe_distance', 0.4)
        self.declare_parameter('retreat_speed', -0.2)
        self.declare_parameter('stop_duration', 5.0)
        self.declare_parameter('path_following_tolerance', 0.2)
        self.declare_parameter('exploration_radius', 3.0)

        # --- Balloon sequence ---
        self.balloon_sequence = ["pink", "green", "yellow"]
        self.sequence_index = 0
        
        # --- Mapping and Planning ---
        self.local_grid = LocalGrid(size=200, resolution=0.05)  # 10x10m map
        self.planner = DijkstraPlanner(self.local_grid)
        
        # --- Robot state ---
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.current_yaw = 0.0
        self.target_detected = None
        self.visited_balloons = set()
        self.min_distance_ahead = None
        
        # --- Path following ---
        self.current_path = []
        self.path_index = 0
        self.goal_position = None
        
        # --- Mission states ---
        self.mission_state = "exploring"  # exploring, following_path, approaching_balloon, stopped
        self.in_stop_phase = False
        self.stop_start_time = None
        
        # --- Exploration ---
        self.last_exploration_time = time.time()
        self.exploration_points = []  # Points to explore
        
        # --- Publishers ---
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # --- Subscribers ---
        self.create_subscription(String, '/object_info', self.detection_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)

        # --- Timers ---
        self.timer = self.create_timer(0.1, self.control_loop)
        self.map_update_timer = self.create_timer(0.5, self.update_exploration_goals)

        self.get_logger().info("✅ Dijkstra Mission Node started...")

    def detection_cb(self, msg: String):
        data = msg.data.lower()
        current_target = self.get_current_target()

        if current_target and current_target in data:
            if data not in self.visited_balloons:
                self.target_detected = data
                self.mission_state = "approaching_balloon"
                # Plan path to current position (balloon should be nearby)
                self.plan_path_to_position(self.robot_x, self.robot_y)
                self.get_logger().info(f"🎯 Target balloon detected: {data}")

    def lidar_cb(self, msg: LaserScan):
        # Update local grid with LiDAR data
        self.local_grid.update_from_lidar(
            self.robot_x, self.robot_y, self.current_yaw,
            msg.ranges, msg.angle_min, msg.angle_increment
        )
        
        # Update minimum distance ahead
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
        """Plan path to a world position"""
        start_gx, start_gy = self.local_grid.world_to_grid(self.robot_x, self.robot_y)
        goal_gx, goal_gy = self.local_grid.world_to_grid(goal_x, goal_y)
        
        path_grid = self.planner.plan_path(start_gx, start_gy, goal_gx, goal_gy)
        
        if path_grid:
            # Convert grid path to world coordinates
            self.current_path = []
            for gx, gy in path_grid:
                world_x, world_y = self.local_grid.grid_to_world(gx, gy)
                self.current_path.append((world_x, world_y))
            
            self.path_index = 0
            self.mission_state = "following_path"
            self.get_logger().info(f"📍 Path planned with {len(self.current_path)} waypoints")
        else:
            self.get_logger().warn("❌ No path found to goal")

    def update_exploration_goals(self):
        """Generate new exploration points on frontiers"""
        if self.mission_state != "exploring":
            return
            
        current_time = time.time()
        if current_time - self.last_exploration_time < 5.0:  # Update every 5 seconds
            return
            
        self.last_exploration_time = current_time
        
        # Find frontier points (boundary between known and unknown)
        exploration_radius = self.get_parameter('exploration_radius').get_parameter_value().double_value
        robot_gx, robot_gy = self.local_grid.world_to_grid(self.robot_x, self.robot_y)
        
        frontier_points = []
        for gx in range(max(0, robot_gx - 60), min(self.local_grid.size, robot_gx + 60)):
            for gy in range(max(0, robot_gy - 60), min(self.local_grid.size, robot_gy + 60)):
                # Check if this is a frontier point
                if (self.local_grid.grid[gy, gx] == 0.0 and  # Free space
                    self.has_unknown_neighbor(gx, gy)):
                    world_x, world_y = self.local_grid.grid_to_world(gx, gy)
                    dist = math.sqrt((world_x - self.robot_x)**2 + (world_y - self.robot_y)**2)
                    if dist <= exploration_radius:
                        frontier_points.append((world_x, world_y))
        
        if frontier_points and not self.current_path:
            # Choose closest frontier point
            best_point = min(frontier_points, key=lambda p: 
                           math.sqrt((p[0] - self.robot_x)**2 + (p[1] - self.robot_y)**2))
            self.plan_path_to_position(best_point[0], best_point[1])
            self.get_logger().info(f"🗺️ New exploration goal: ({best_point[0]:.2f}, {best_point[1]:.2f})")

    def has_unknown_neighbor(self, gx, gy):
        """Check if cell has unknown neighbors"""
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                nx, ny = gx + dx, gy + dy
                if (self.local_grid.is_valid(nx, ny) and 
                    self.local_grid.grid[ny, nx] == 0.5):  # Unknown
                    return True
        return False

    def follow_path(self):
        """Follow the current path using pure pursuit"""
        if not self.current_path or self.path_index >= len(self.current_path):
            self.current_path = []
            self.mission_state = "exploring"
            return Twist()
        
        tolerance = self.get_parameter('path_following_tolerance').get_parameter_value().double_value
        target_x, target_y = self.current_path[self.path_index]
        
        # Calculate distance to current waypoint
        dist = math.sqrt((target_x - self.robot_x)**2 + (target_y - self.robot_y)**2)
        
        if dist < tolerance:
            self.path_index += 1
            if self.path_index >= len(self.current_path):
                self.current_path = []
                self.mission_state = "exploring"
                self.get_logger().info("✅ Reached path end")
                return Twist()
        
        # Pure pursuit control
        cmd = Twist()
        angle_to_target = math.atan2(target_y - self.robot_y, target_x - self.robot_x)
        angle_error = self.angle_diff(angle_to_target, self.current_yaw)
        
        v_max = self.get_parameter('v_max').get_parameter_value().double_value
        w_max = self.get_parameter('w_max').get_parameter_value().double_value
        
        cmd.linear.x = v_max * math.cos(angle_error)
        cmd.angular.z = max(min(2.0 * angle_error, w_max), -w_max)
        
        return cmd

    def angle_diff(self, a, b):
        d = a - b
        return math.atan2(math.sin(d), math.cos(d))

    def control_loop(self):
        current_target = self.get_current_target()
        
        # Mission complete
        if current_target is None:
            self.get_logger().info("🎉 Mission complete!")
            self.cmd_pub.publish(Twist())
            return
        
        cmd = Twist()
        
        # Handle stop phase
        if self.in_stop_phase:
            now = time.time()
            stop_duration = self.get_parameter('stop_duration').get_parameter_value().double_value
            
            if now - self.stop_start_time < stop_duration:
                cmd.linear.x = 0.0
            elif now - self.stop_start_time < stop_duration + 1.0:
                cmd.linear.x = self.get_parameter('retreat_speed').get_parameter_value().double_value
            else:
                self.in_stop_phase = False
                self.target_detected = None
                self.sequence_index += 1
                self.mission_state = "exploring"
                self.get_logger().info("🔄 Next balloon in sequence")
        
        # Balloon approach
        elif (self.target_detected and self.mission_state == "approaching_balloon"):
            safe_distance = self.get_parameter('safe_distance').get_parameter_value().double_value
            
            if self.min_distance_ahead and self.min_distance_ahead <= safe_distance:
                self.get_logger().info(f"🎈 Reached {current_target} balloon")
                self.visited_balloons.add(self.target_detected)
                self.in_stop_phase = True
                self.stop_start_time = time.time()
                cmd = Twist()
            else:
                cmd.linear.x = self.get_parameter('v_max').get_parameter_value().double_value * 0.5
        
        # Path following
        elif self.mission_state == "following_path":
            cmd = self.follow_path()
        
        # Exploration (fallback)
        else:
            cmd.angular.z = 0.3  # Slow rotation while exploring
        
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
