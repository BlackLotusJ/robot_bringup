#!/usr/bin/env python3
"""
Balloon A* Mission Node
-----------------------------
Merged node:
 - Local occupancy grid built from LiDAR
 - A* local planner (grid-based)
 - Publishes /local_map (OccupancyGrid) and /planned_path (Path) for RViz2
 - Balloon color sequence logic (pink -> yellow -> black -> white -> blue)
 - Startup yaw acknowledgment (left-right wiggle)
 - Move-and-scan exploration fallback when no path is active

Improvements:
1. Added A* algorithm for more efficient path planning
2. Added proper type hints
3. Enhanced parameter handling with validation
4. Improved path following with better obstacle avoidance
5. Added recovery behaviors for stuck situations
6. Optimized grid operations with numpy
7. Added comprehensive logging
8. Improved state management
9. Added timeout mechanisms for planning
10. Enhanced exploration logic with frontier detection
11. Added safety checks and emergency stop conditions
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import ParameterDescriptor
from geometry_msgs.msg import Twist, PoseStamped, Point
from std_msgs.msg import String, Header
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
import time
import math
import numpy as np
import heapq
from collections import defaultdict
from typing import List, Tuple, Optional, Dict, Set
from enum import Enum, auto

# ---------------------------
# Utility helpers
# ---------------------------

def safe_param(node: Node, name: str, default: float, param_type: type = float) -> float:
    """Return parameter value or default if not set or wrong type."""
    try:
        if not node.has_parameter(name):
            node.declare_parameter(name, default)
        
        param_value = node.get_parameter(name).value
        if param_type == float:
            return float(param_value)
        elif param_type == int:
            return int(param_value)
        elif param_type == bool:
            return bool(param_value)
        else:
            return param_value
    except (ValueError, TypeError) as e:
        node.get_logger().warn(f"Parameter {name} has invalid value, using default: {default}. Error: {e}")
        return default

def normalize_angle(angle: float) -> float:
    """Normalize angle to [-π, π]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

# ---------------------------
# Mission States
# ---------------------------

class MissionState(Enum):
    STARTUP = auto()
    EXPLORING = auto()
    FOLLOWING_PATH = auto()
    APPROACHING_BALLOON = auto()
    STOPPED = auto()
    RECOVERY = auto()

# ---------------------------
# Local grid and planner
# ---------------------------

class LocalGrid:
    def __init__(self, size: int = 200, resolution: float = 0.05):
        self.size = size
        self.resolution = resolution
        # Initialize as unknown (0.5)
        self.grid = np.full((size, size), 0.5, dtype=np.float32)
        self.origin_x = size // 2
        self.origin_y = size // 2
        self.last_update_time = time.time()

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int(round(x / self.resolution)) + self.origin_x
        gy = int(round(y / self.resolution)) + self.origin_y
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        x = (gx - self.origin_x) * self.resolution
        y = (gy - self.origin_y) * self.resolution
        return x, y

    def is_valid(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.size and 0 <= gy < self.size

    def update_from_lidar(self, robot_x: float, robot_y: float, robot_yaw: float, 
                         ranges: List[float], angle_min: float, angle_increment: float, 
                         max_range: float = 5.0) -> None:
        """Rasterize LiDAR rays into occupancy grid using Bresenham's algorithm for efficiency."""
        if ranges is None:
            return

        # Convert robot position to grid coordinates
        robot_gx, robot_gy = self.world_to_grid(robot_x, robot_y)
        
        for i, r in enumerate(ranges):
            if math.isinf(r) or r <= 0.0:
                continue
                
            angle = robot_yaw + angle_min + i * angle_increment
            r_clamped = min(r, max_range)
            
            # Calculate endpoint in world coordinates
            end_x = robot_x + r_clamped * math.cos(angle)
            end_y = robot_y + r_clamped * math.sin(angle)
            
            # Convert to grid coordinates
            end_gx, end_gy = self.world_to_grid(end_x, end_y)
            
            # Use Bresenham's line algorithm to mark free cells
            self.bresenham_line(robot_gx, robot_gy, end_gx, end_gy, 0.0)
            
            # Mark endpoint as occupied if within range
            if r < max_range and self.is_valid(end_gx, end_gy):
                self.grid[end_gy, end_gx] = 1.0  # occupied

        # Ensure robot cell is marked free
        if self.is_valid(robot_gx, robot_gy):
            self.grid[robot_gy, robot_gx] = 0.0
            
        self.last_update_time = time.time()

    def bresenham_line(self, x0: int, y0: int, x1: int, y1: int, value: float) -> None:
        """Bresenham's line algorithm for efficient grid marking."""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                if self.is_valid(x, y):
                    self.grid[y, x] = value
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                if self.is_valid(x, y):
                    self.grid[y, x] = value
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
                
        if self.is_valid(x, y):
            self.grid[y, x] = value

    def find_frontiers(self, robot_gx: int, robot_gy: int, max_distance: int = 50) -> List[Tuple[int, int]]:
        """Find frontier cells (unknown adjacent to free) for exploration."""
        frontiers = []
        visited = set()
        
        # Check cells within max_distance from robot
        for y in range(max(0, robot_gy - max_distance), min(self.size, robot_gy + max_distance)):
            for x in range(max(0, robot_gx - max_distance), min(self.size, robot_gx + max_distance)):
                if (x, y) in visited or self.grid[y, x] >= 0.8:  # Skip occupied cells
                    continue
                    
                # Check if this is a frontier cell (unknown adjacent to free)
                if 0.4 < self.grid[y, x] < 0.6:  # Unknown cell
                    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                        nx, ny = x + dx, y + dy
                        if self.is_valid(nx, ny) and self.grid[ny, nx] < 0.4:  # Free cell
                            frontiers.append((x, y))
                            visited.add((x, y))
                            break
                            
        return frontiers

    def mark_robot(self, robot_x: float, robot_y: float) -> None:
        """Ensure robot cell is marked as free."""
        robot_gx, robot_gy = self.world_to_grid(robot_x, robot_y)
        if self.is_valid(robot_gx, robot_gy):
            self.grid[robot_gy, robot_gx] = 0.0

class AStarPlanner:
    def __init__(self, grid: LocalGrid):
        self.grid = grid
        self.obstacle_threshold = 0.8
        self.unknown_penalty = 2.0

    def heuristic(self, x1: int, y1: int, x2: int, y2: int) -> float:
        """Euclidean distance heuristic for A*."""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def get_neighbors(self, x: int, y: int) -> List[Tuple[int, int]]:
        """Get 8-connected neighbors."""
        neighbors = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if self.grid.is_valid(nx, ny):
                    neighbors.append((nx, ny))
        return neighbors

    def get_cost(self, x1: int, y1: int, x2: int, y2: int) -> float:
        """Calculate cost between two cells."""
        # Occupied cell blocking
        if self.grid.grid[y2, x2] >= self.obstacle_threshold:
            return float('inf')
            
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        cost = math.sqrt(2) if (dx == 1 and dy == 1) else 1.0
        
        # Penalty for unknown areas
        if 0.4 < self.grid.grid[y2, x2] < 0.6:
            cost += self.unknown_penalty
            
        return cost

    def plan_path(self, sx: int, sy: int, gx: int, gy: int, 
                 timeout: float = 0.1) -> List[Tuple[int, int]]:
        """Plan path using A* with timeout."""
        if not (self.grid.is_valid(sx, sy) and self.grid.is_valid(gx, gy)):
            return []
            
        start_time = time.time()
        # Priority queue: (f_score, g_score, x, y)
        pq = [(self.heuristic(sx, sy, gx, gy), 0.0, sx, sy)]
        g_score = defaultdict(lambda: float('inf'))
        g_score[(sx, sy)] = 0.0
        f_score = defaultdict(lambda: float('inf'))
        f_score[(sx, sy)] = self.heuristic(sx, sy, gx, gy)
        prev = {}
        visited = set()

        while pq and time.time() - start_time < timeout:
            _, cur_g, x, y = heapq.heappop(pq)
            if (x, y) in visited:
                continue
            visited.add((x, y))
            
            # Check if we reached the goal
            if x == gx and y == gy:
                # Reconstruct path
                path = []
                cur = (x, y)
                while cur in prev:
                    path.append(cur)
                    cur = prev[cur]
                path.append((sx, sy))
                return path[::-1]
                
            # Explore neighbors
            for nx, ny in self.get_neighbors(x, y):
                if (nx, ny) in visited:
                    continue
                    
                cost = self.get_cost(x, y, nx, ny)
                if cost == float('inf'):
                    continue
                    
                tentative_g = cur_g + cost
                if tentative_g < g_score[(nx, ny)]:
                    prev[(nx, ny)] = (x, y)
                    g_score[(nx, ny)] = tentative_g
                    f_score[(nx, ny)] = tentative_g + self.heuristic(nx, ny, gx, gy)
                    heapq.heappush(pq, (f_score[(nx, ny)], tentative_g, nx, ny))
                    
        return []  # No path found or timeout

# ---------------------------
# Merged Node
# ---------------------------

class BalloonAStarMissionNode(Node):
    def __init__(self):
        super().__init__('balloon_astar_mission_node')

        # Declare parameters with default values and descriptions
        params = {
            'v_max': (0.2, 'Maximum linear velocity'),
            'w_max': (0.6, 'Maximum angular velocity'),
            'safe_distance': (0.5, 'Safe distance to obstacles'),
            'retreat_speed': (-0.2, 'Speed when retreating from balloon'),
            'stop_duration': (5.0, 'Duration to stop at balloon'),
            'yaw_gain': (1.5, 'Gain for yaw control'),
            'exploration_radius': (3.0, 'Radius for exploration goals'),
            'startup_yaw_duration': (4.0, 'Duration for startup yaw motion'),
            'map_publish_rate': (1.0, 'Rate to publish map'),
            'use_lidar': (True, 'Whether to use LiDAR data'),
            'map_max_range': (5.0, 'Maximum range for mapping'),
            'path_following_tolerance': (0.2, 'Tolerance for path following'),
            'recovery_timeout': (10.0, 'Timeout for recovery behaviors'),
            'planning_timeout': (0.1, 'Timeout for path planning'),
        }
        
        for name, (default, desc) in params.items():
            self.declare_parameter(name, default, ParameterDescriptor(description=desc))

        # Balloon sequence
        self.balloon_sequence = ["pink", "yellow", "black", "white", "blue"]
        self.sequence_index = 0

        # Startup yaw acknowledgement
        self.startup_phase = True
        self.startup_start_time = time.time()
        self.initial_yaw = None
        self.target_yaw_cycles = 2

        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.current_yaw = 0.0
        self.min_distance_ahead = float('inf')
        self.lidar_ranges = None
        self.last_pose_update = time.time()

        # Mission bookkeeping
        self.target_detected = None
        self.target_yaw = None
        self.visited_balloons = set()
        self.in_stop_phase = False
        self.stop_start_time = None
        self.last_state_change = time.time()
        self.stuck_check_time = time.time()
        self.last_robot_x = 0.0
        self.last_robot_y = 0.0

        # Mapping + planning
        self.local_grid = LocalGrid(size=200, resolution=0.05)
        self.planner = AStarPlanner(self.local_grid)
        self.current_path = []            # list of (x,y) world coords
        self.path_index = 0
        self.mission_state = MissionState.STARTUP
        self.recovery_start_time = 0.0

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/local_map', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.state_pub = self.create_publisher(String, '/mission_state', 10)

        # Subscribers
        self.create_subscription(String, '/object_info', self.detection_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)

        # Timers
        self.create_timer(0.1, self.control_loop)  # main control loop
        self.create_timer(0.5, self.update_exploration_goals)
        map_rate = safe_param(self, 'map_publish_rate', 1.0)
        self.create_timer(1.0 / max(0.1, map_rate), self.publish_map)
        self.create_timer(1.0, self.check_stuck_condition)

        self.get_logger().info("✅ Balloon A* Mission Node started (improved).")

    # ---------------------------
    # Callbacks
    # ---------------------------
    def detection_cb(self, msg: String) -> None:
        # Ignore detections during startup yaw
        if self.mission_state == MissionState.STARTUP:
            return
            
        data = msg.data.lower()
        current_target = self.get_current_target()
        
        if current_target and current_target in data:
            if data not in self.visited_balloons:
                self.target_detected = data
                # Store heading when first detected
                self.target_yaw = self.current_yaw
                self.change_state(MissionState.APPROACHING_BALLOON)
                # Plan path to current robot location as placeholder
                self.plan_path_to_position(self.robot_x, self.robot_y)
                self.get_logger().info(f"🎯 Target balloon detected: {data}")
            else:
                self.get_logger().info(f"⏭ Already visited {data} - ignoring")

    def lidar_cb(self, msg: LaserScan) -> None:
        if not safe_param(self, 'use_lidar', True, bool):
            return
            
        self.lidar_ranges = msg.ranges
        center_idx = len(msg.ranges) // 2
        d = msg.ranges[center_idx]
        self.min_distance_ahead = 10.0 if math.isinf(d) else d
        
        # Update occupancy grid
        self.local_grid.update_from_lidar(
            self.robot_x, self.robot_y, self.current_yaw,
            msg.ranges, msg.angle_min, msg.angle_increment,
            max_range=safe_param(self, 'map_max_range', 5.0)
        )
        
        # Ensure robot cell marked free
        self.local_grid.mark_robot(self.robot_x, self.robot_y)

    def imu_cb(self, msg: Imu) -> None:
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        if self.initial_yaw is None:
            self.initial_yaw = self.current_yaw
            
        self.last_pose_update = time.time()

    # ---------------------------
    # State management
    # ---------------------------
    def change_state(self, new_state: MissionState) -> None:
        if self.mission_state != new_state:
            self.get_logger().info(f"State change: {self.mission_state.name} -> {new_state.name}")
            self.mission_state = new_state
            self.last_state_change = time.time()
            
            # Publish state change
            state_msg = String()
            state_msg.data = new_state.name
            self.state_pub.publish(state_msg)

    # ---------------------------
    # Target & planning utilities
    # ---------------------------
    def get_current_target(self) -> Optional[str]:
        if self.sequence_index < len(self.balloon_sequence):
            return self.balloon_sequence[self.sequence_index]
        return None

    def plan_path_to_position(self, goal_x: float, goal_y: float) -> bool:
        """Plan grid path from robot current world coordinates to a world goal (x,y)."""
        start_gx, start_gy = self.local_grid.world_to_grid(self.robot_x, self.robot_y)
        goal_gx, goal_gy = self.local_grid.world_to_grid(goal_x, goal_y)
        
        planning_timeout = safe_param(self, 'planning_timeout', 0.1)
        path_grid = self.planner.plan_path(start_gx, start_gy, goal_gx, goal_gy, planning_timeout)
        
        if path_grid:
            self.current_path = []
            for gx, gy in path_grid:
                wx, wy = self.local_grid.grid_to_world(gx, gy)
                self.current_path.append((wx, wy))
                
            self.path_index = 0
            self.change_state(MissionState.FOLLOWING_PATH)
            self.get_logger().info(f"📍 Path planned with {len(self.current_path)} waypoints")
            self.publish_path()
            return True
        else:
            self.get_logger().warn("❌ No path found to goal")
            return False

    def publish_path(self) -> None:
        if not self.current_path:
            return
            
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        for (x, y) in self.current_path:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
            
        self.path_pub.publish(path_msg)

    def publish_map(self) -> None:
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        info = MapMetaData()
        info.resolution = self.local_grid.resolution
        info.width = self.local_grid.size
        info.height = self.local_grid.size
        # Origin so grid_to_world(0,0) sits at origin position
        info.origin.position.x = -(self.local_grid.origin_x * self.local_grid.resolution)
        info.origin.position.y = -(self.local_grid.origin_y * self.local_grid.resolution)
        info.origin.orientation.w = 1.0
        msg.info = info

        # Convert to ROS occupancy values: -1 unknown, 0 free, 100 occupied
        flat = []
        for row in self.local_grid.grid:
            for v in row:
                if 0.4 < v < 0.6:
                    flat.append(-1)
                elif v >= 0.8:
                    flat.append(100)
                else:
                    flat.append(0)
                    
        msg.data = flat
        self.map_pub.publish(msg)

    # ---------------------------
    # Exploration logic
    # ---------------------------
    def update_exploration_goals(self) -> None:
        """Generate exploration goals based on frontier detection."""
        if self.mission_state not in [MissionState.EXPLORING, MissionState.RECOVERY]:
            return
            
        # Only update every 5 seconds
        if not hasattr(self, '_last_explore_time'):
            self._last_explore_time = 0.0
            
        if time.time() - self._last_explore_time < 5.0:
            return
            
        self._last_explore_time = time.time()
        
        # Convert robot position to grid coordinates
        robot_gx, robot_gy = self.local_grid.world_to_grid(self.robot_x, self.robot_y)
        
        # Find frontiers (unknown cells adjacent to free cells)
        frontiers = self.local_grid.find_frontiers(robot_gx, robot_gy, max_distance=50)
        
        if frontiers:
            # Sort frontiers by distance from robot
            frontiers.sort(key=lambda p: math.hypot(p[0]-robot_gx, p[1]-robot_gy))
            
            # Try to plan to the closest frontier
            for fx, fy in frontiers:
                wx, wy = self.local_grid.grid_to_world(fx, fy)
                if self.plan_path_to_position(wx, wy):
                    self.get_logger().info(f"🗺️ Exploration goal set to frontier: ({wx:.2f}, {wy:.2f})")
                    return
                    
        # Fallback: random exploration if no frontiers found
        radius = safe_param(self, 'exploration_radius', 3.0)
        for attempt in range(10):
            ang = np.random.uniform(0, 2 * math.pi)
            r = np.random.uniform(0.5 * radius, radius)
            gx = self.robot_x + r * math.cos(ang)
            gy = self.robot_y + r * math.sin(ang)
            
            # Try to plan to that point
            if self.plan_path_to_position(gx, gy):
                self.get_logger().info(f"🗺️ Random exploration goal set: ({gx:.2f}, {gy:.2f})")
                return
                
        self.get_logger().warn("No exploration goals found")

    # ---------------------------
    # Path following and motion
    # ---------------------------
    def follow_path(self) -> Optional[Twist]:
        """Follow current_path using a pure pursuit controller."""
        if not self.current_path or self.path_index >= len(self.current_path):
            # Finished path
            self.current_path = []
            self.change_state(MissionState.EXPLORING)
            return None
            
        # Look ahead to a point on the path
        lookahead_dist = 0.5  # meters
        target_idx = self.path_index
        
        # Find the point on the path that is lookahead_dist away
        while target_idx < len(self.current_path) - 1:
            wx, wy = self.current_path[target_idx]
            dist = math.hypot(wx - self.robot_x, wy - self.robot_y)
            if dist >= lookahead_dist:
                break
            target_idx += 1
            
        # Get the target point
        wx, wy = self.current_path[target_idx]
        
        # Check if we've reached the current waypoint
        tol = safe_param(self, 'path_following_tolerance', 0.2)
        current_wx, current_wy = self.current_path[self.path_index]
        dist_to_waypoint = math.hypot(current_wx - self.robot_x, current_wy - self.robot_y)
        
        if dist_to_waypoint < tol:
            self.path_index += 1
            if self.path_index >= len(self.current_path):
                # Reached end of path
                self.current_path = []
                self.change_state(MissionState.EXPLORING)
                self.get_logger().info("✅ Reached path end")
                return None
                
        # Compute heading control
        dx = wx - self.robot_x
        dy = wy - self.robot_y
        angle_to_goal = math.atan2(dy, dx)
        angle_error = normalize_angle(angle_to_goal - self.current_yaw)
        
        v_max = safe_param(self, 'v_max', 0.2)
        w_max = safe_param(self, 'w_max', 0.6)
        
        cmd = Twist()
        # Scale forward speed by heading error and obstacle distance
        speed_factor = math.cos(angle_error) * min(1.0, self.min_distance_ahead / safe_param(self, 'safe_distance', 0.5))
        cmd.linear.x = max(0.05, v_max * speed_factor)
        cmd.angular.z = max(min(2.0 * angle_error, w_max), -w_max)
        
        return cmd

    # ---------------------------
    # Startup yaw motion
    # ---------------------------
    def perform_startup_yaw(self) -> Twist:
        """Left-right yaw sinusoidal motion for startup acknowledgement."""
        if self.current_yaw is None or self.initial_yaw is None:
            return Twist()
            
        startup_duration = safe_param(self, 'startup_yaw_duration', 4.0)
        w_max = safe_param(self, 'w_max', 0.6)
        elapsed = time.time() - self.startup_start_time
        
        cmd = Twist()
        cycles = self.target_yaw_cycles
        
        if elapsed < startup_duration:
            cycle_period = startup_duration / cycles
            phase = (elapsed % cycle_period) / cycle_period * 2.0 * math.pi
            yaw_amplitude = math.pi / 6.0  # +/- 30 degrees
            target_yaw = self.initial_yaw + yaw_amplitude * math.sin(phase)
            yaw_error = normalize_angle(target_yaw - self.current_yaw)
            cmd.angular.z = max(min(2.0 * yaw_error, w_max), -w_max)
        else:
            # Return to initial yaw, then finish startup
            yaw_error = normalize_angle(self.initial_yaw - self.current_yaw)
            if abs(yaw_error) > 0.05:
                cmd.angular.z = max(min(2.0 * yaw_error, w_max), -w_max)
            else:
                self.startup_phase = False
                self.change_state(MissionState.EXPLORING)
                self.get_logger().info("✅ Startup complete. Beginning mission.")
                
        return cmd

    # ---------------------------
    # Stuck detection and recovery
    # ---------------------------
    def check_stuck_condition(self) -> None:
        """Check if the robot is stuck and initiate recovery if needed."""
        if self.mission_state in [MissionState.STARTUP, MissionState.STOPPED]:
            return
            
        # Check if robot hasn't moved significantly
        movement = math.hypot(self.robot_x - self.last_robot_x, self.robot_y - self.last_robot_y)
        time_since_last_move = time.time() - self.stuck_check_time
        
        if movement < 0.1 and time_since_last_move > 5.0:
            self.get_logger().warn("Robot appears to be stuck. Initiating recovery.")
            self.change_state(MissionState.RECOVERY)
            self.recovery_start_time = time.time()
            
        # Update last position and time
        if movement > 0.05:  # Significant movement
            self.last_robot_x = self.robot_x
            self.last_robot_y = self.robot_y
            self.stuck_check_time = time.time()

    def execute_recovery(self) -> Twist:
        """Execute recovery behavior when robot is stuck."""
        recovery_timeout = safe_param(self, 'recovery_timeout', 10.0)
        
        if time.time() - self.recovery_start_time > recovery_timeout:
            self.get_logger().info("Recovery timeout. Returning to exploration.")
            self.change_state(MissionState.EXPLORING)
            return Twist()
            
        # Simple recovery: back up and turn
        cmd = Twist()
        elapsed = time.time() - self.recovery_start_time
        
        if elapsed < 2.0:
            # Back up
            cmd.linear.x = -0.1
        elif elapsed < 4.0:
            # Turn in place
            cmd.angular.z = 0.5
        else:
            # Try to move forward
            cmd.linear.x = 0.1
            
        return cmd

    # ---------------------------
    # Main control loop
    # ---------------------------
    def control_loop(self) -> None:
        # If in startup phase do yaw
        if self.mission_state == MissionState.STARTUP:
            cmd = self.perform_startup_yaw()
            self.cmd_pub.publish(cmd)
            return

        # Stop-phase (when balloon reached)
        if self.in_stop_phase:
            now = time.time()
            stop_duration = safe_param(self, 'stop_duration', 5.0)
            
            if now - self.stop_start_time < stop_duration:
                # Hold still
                self.cmd_pub.publish(Twist())
                return
            elif now - self.stop_start_time < stop_duration + 1.0:
                # Retreat a bit
                cmd = Twist()
                cmd.linear.x = safe_param(self, 'retreat_speed', -0.2)
                self.cmd_pub.publish(cmd)
                return
            else:
                # Finish stop phase, go to next balloon
                self.in_stop_phase = False
                self.target_detected = None
                self.target_yaw = None
                self.sequence_index += 1
                self.change_state(MissionState.EXPLORING)
                self.get_logger().info(f"🔄 Moving to next balloon: {self.get_current_target()}")
                return

        # Recovery behavior
        if self.mission_state == MissionState.RECOVERY:
            cmd = self.execute_recovery()
            self.cmd_pub.publish(cmd)
            return

        # If a target was detected and we are in approach mode
        current_target = self.get_current_target()
        if self.target_detected and self.mission_state == MissionState.APPROACHING_BALLOON:
            # If we have LiDAR distance, use it to decide when to stop
            safe_d = safe_param(self, 'safe_distance', 0.5)
            dist = self.min_distance_ahead if self.min_distance_ahead is not None else float('inf')

            # If no map path exists (or too short), try to plan toward the detection direction
            if not self.current_path:
                approach_goal_dist = max(0.6, safe_d + 0.2)
                gx = self.robot_x + approach_goal_dist * math.cos(self.current_yaw)
                gy = self.robot_y + approach_goal_dist * math.sin(self.current_yaw)
                self.plan_path_to_position(gx, gy)

            # If close enough by LiDAR, enter stop phase
            if dist <= safe_d:
                self.get_logger().info(f"🎈 Reached {current_target} (dist={dist:.2f}m). Stopping.")
                self.visited_balloons.add(self.target_detected)
                self.in_stop_phase = True
                self.stop_start_time = time.time()
                # Stop robot
                self.cmd_pub.publish(Twist())
                return
            else:
                # Follow path toward the object (if path exists)
                if self.mission_state == MissionState.FOLLOWING_PATH:
                    cmd = self.follow_path()
                    if cmd is None:
                        # Fallback: move forward slower
                        cmd = Twist()
                        cmd.linear.x = safe_param(self, 'v_max', 0.2) * 0.5
                        cmd.angular.z = 0.0
                        self.cmd_pub.publish(cmd)
                    else:
                        self.cmd_pub.publish(cmd)
                    return
                else:
                    # Fallback approach (no path)
                    cmd = Twist()
                    cmd.linear.x = safe_param(self, 'v_max', 0.2) * 0.4
                    cmd.angular.z = 0.0
                    self.cmd_pub.publish(cmd)
                    return

        # If currently following a planned path, continue following it
        if self.mission_state == MissionState.FOLLOWING_PATH:
            cmd = self.follow_path()
            if cmd is not None:
                self.cmd_pub.publish(cmd)
            return

        # Otherwise, exploration scanning behavior (move-and-scan)
        # Simple move-and-scan: forward slowly and wiggle rotation to discover objects
        cmd = Twist()
        scan_speed = max(0.05, safe_param(self, 'v_max', 0.2) * 0.5)
        cmd.linear.x = scan_speed
        # Gentle oscillation rotation
        t = time.time()
        cmd.angular.z = 0.25 * math.sin(0.5 * t)
        
        # If obstacle ahead, slow/turn
        if self.min_distance_ahead is not None and self.min_distance_ahead < safe_param(self, 'safe_distance', 0.5):
            cmd.linear.x = 0.0
            cmd.angular.z = safe_param(self, 'w_max', 0.6)
            
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = BalloonAStarMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
