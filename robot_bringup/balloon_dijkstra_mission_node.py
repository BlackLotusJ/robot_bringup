#!/usr/bin/env python3
"""
Balloon Dijkstra Mission Node
-----------------------------
Merged node:
 - Local occupancy grid built from LiDAR
 - Dijkstra local planner (grid-based)
 - Publishes /local_map (OccupancyGrid) and /planned_path (Path) for RViz2
 - Balloon color sequence logic (pink -> yellow -> black -> white -> blue)
 - Startup yaw acknowledgment (left-right wiggle)
 - Move-and-scan exploration fallback when no path is active
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

# ---------------------------
# Utility helpers
# ---------------------------

def safe_param(node, name, default):
    """Return parameter value or default if not set or wrong type."""
    try:
        p = node.get_parameter(name)
        # ParameterValue API differs by versions; try common attributes
        pv = p.get_parameter_value()
        # try double_value then value
        if hasattr(pv, 'double_value'):
            return pv.double_value
        return p.value
    except Exception:
        try:
            return node.get_parameter(name).value
        except Exception:
            return default

# ---------------------------
# Local grid and planner
# ---------------------------

class LocalGrid:
    def __init__(self, size=200, resolution=0.05):
        self.size = size
        self.resolution = resolution
        # Initialize as unknown (0.5)
        self.grid = np.full((size, size), 0.5, dtype=np.float32)
        self.origin_x = size // 2
        self.origin_y = size // 2

    def world_to_grid(self, x, y):
        gx = int(round(x / self.resolution)) + self.origin_x
        gy = int(round(y / self.resolution)) + self.origin_y
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = (gx - self.origin_x) * self.resolution
        y = (gy - self.origin_y) * self.resolution
        return x, y

    def is_valid(self, gx, gy):
        return 0 <= gx < self.size and 0 <= gy < self.size

    def update_from_lidar(self, robot_x, robot_y, robot_yaw, ranges, angle_min, angle_increment, max_range=5.0):
        """Rasterize LiDAR rays into occupancy grid:
           - cells along ray -> free (0.0)
           - endpoint -> occupied (1.0) if within max_range
        """
        if ranges is None:
            return

        for i, r in enumerate(ranges):
            if math.isinf(r) or r <= 0.0:
                continue
            angle = robot_yaw + angle_min + i * angle_increment
            # limit range for marking
            r_clamped = min(r, max_range)
            steps = max(1, int(r_clamped / self.resolution))
            for step in range(steps):
                # use midpoint of each cell step to avoid leaving gaps
                d = (step + 0.5) * self.resolution
                if d > r_clamped:
                    break
                rx = robot_x + d * math.cos(angle)
                ry = robot_y + d * math.sin(angle)
                gx, gy = self.world_to_grid(rx, ry)
                if self.is_valid(gx, gy):
                    self.grid[gy, gx] = 0.0  # free

            # endpoint occupied if true obstacle within max_range
            if r < max_range:
                ex = robot_x + r * math.cos(angle)
                ey = robot_y + r * math.sin(angle)
                gx, gy = self.world_to_grid(ex, ey)
                if self.is_valid(gx, gy):
                    self.grid[gy, gx] = 1.0  # occupied

    def mark_robot(self, robot_x, robot_y):
        gx, gy = self.world_to_grid(robot_x, robot_y)
        if self.is_valid(gx, gy):
            # mark as free to avoid blocking planner
            self.grid[gy, gx] = 0.0

class DijkstraPlanner:
    def __init__(self, grid: LocalGrid):
        self.grid = grid

    def get_neighbors(self, x, y):
        neighbors = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if self.grid.is_valid(nx, ny):
                    neighbors.append((nx, ny))
        return neighbors

    def get_cost(self, x1, y1, x2, y2):
        # Occupied cell blocking
        if self.grid.grid[y2, x2] >= 0.8:
            return float('inf')
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        cost = 1.4142135 if (dx == 1 and dy == 1) else 1.0
        # penalty for unknown
        if 0.4 < self.grid.grid[y2, x2] < 0.6:
            cost += 2.0
        return cost

    def plan_path(self, sx, sy, gx, gy):
        if not (self.grid.is_valid(sx, sy) and self.grid.is_valid(gx, gy)):
            return []
        pq = [(0.0, sx, sy)]
        dist = defaultdict(lambda: float('inf'))
        dist[(sx, sy)] = 0.0
        prev = {}
        visited = set()

        while pq:
            cur_cost, x, y = heapq.heappop(pq)
            if (x, y) in visited:
                continue
            visited.add((x, y))
            if x == gx and y == gy:
                # reconstruct
                path = []
                cur = (x, y)
                while cur in prev:
                    path.append(cur)
                    cur = prev[cur]
                path.append((sx, sy))
                return path[::-1]
            for nx, ny in self.get_neighbors(x, y):
                if (nx, ny) in visited:
                    continue
                c = self.get_cost(x, y, nx, ny)
                if c == float('inf'):
                    continue
                nd = cur_cost + c
                if nd < dist[(nx, ny)]:
                    dist[(nx, ny)] = nd
                    prev[(nx, ny)] = (x, y)
                    heapq.heappush(pq, (nd, nx, ny))
        return []

# ---------------------------
# Merged Node
# ---------------------------

class BalloonDijkstraMissionNode(Node):
    def __init__(self):
        super().__init__('balloon_dijkstra_mission_node')

        # declare parameters with default values
        self.declare_parameter('v_max', 0.2)
        self.declare_parameter('w_max', 0.6)
        self.declare_parameter('safe_distance', 0.5)
        self.declare_parameter('retreat_speed', -0.2)
        self.declare_parameter('stop_duration', 5.0)
        self.declare_parameter('yaw_gain', 1.5)
        self.declare_parameter('exploration_radius', 3.0)
        self.declare_parameter('startup_yaw_duration', 4.0)
        self.declare_parameter('map_publish_rate', 1.0)
        self.declare_parameter('use_lidar', True)
        self.declare_parameter('map_max_range', 5.0)

        # balloon sequence (merged set)
        self.balloon_sequence = ["pink", "yellow", "black", "white", "blue"]
        self.sequence_index = 0

        # startup yaw acknowledgement
        self.startup_phase = True
        self.startup_start_time = time.time()
        self.initial_yaw = None
        self.target_yaw_cycles = 2

        # robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.current_yaw = 0.0
        self.min_distance_ahead = None
        self.lidar_ranges = None

        # mission bookkeeping
        self.target_detected = None
        self.target_yaw = None
        self.visited_balloons = set()
        self.in_stop_phase = False
        self.stop_start_time = None

        # mapping + planning
        self.local_grid = LocalGrid(size=200, resolution=0.05)
        self.planner = DijkstraPlanner(self.local_grid)
        self.current_path = []            # list of (x,y) world coords
        self.path_index = 0
        self.mission_state = "exploring"  # exploring, following_path, approaching_balloon, stopped

        # publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/local_map', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        # subscribers
        self.create_subscription(String, '/object_info', self.detection_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)

        # timers
        self.create_timer(0.1, self.control_loop)  # main control loop
        self.create_timer(0.5, self.update_exploration_goals)
        map_rate = safe_param(self, 'map_publish_rate', 1.0)
        self.create_timer(1.0 / max(0.1, map_rate), self.publish_map)

        self.get_logger().info("✅ Balloon Dijkstra Mission Node started (merged).")

    # ---------------------------
    # Callbacks
    # ---------------------------
    def detection_cb(self, msg: String):
        # ignore detections during startup yaw
        if self.startup_phase:
            return
        data = msg.data.lower()
        current_target = self.get_current_target()
        if current_target and current_target in data:
            if data not in self.visited_balloons:
                self.target_detected = data
                # store heading when first detected
                self.target_yaw = self.current_yaw
                self.mission_state = "approaching_balloon"
                # plan path to current robot location as placeholder (we will plan to a nearby frontier/endpoint)
                self.plan_path_to_position(self.robot_x, self.robot_y)
                self.get_logger().info(f"🎯 Target balloon detected: {data}")
            else:
                self.get_logger().info(f"⏭ Already visited {data} - ignoring")

    def lidar_cb(self, msg: LaserScan):
        if not safe_param(self, 'use_lidar', True):
            return
        self.lidar_ranges = msg.ranges
        center_idx = len(msg.ranges) // 2
        d = msg.ranges[center_idx]
        self.min_distance_ahead = 10.0 if math.isinf(d) else d
        # update occupancy grid
        self.local_grid.update_from_lidar(
            self.robot_x, self.robot_y, self.current_yaw,
            msg.ranges, msg.angle_min, msg.angle_increment,
            max_range=safe_param(self, 'map_max_range', 5.0)
        )
        # ensure robot cell marked free
        self.local_grid.mark_robot(self.robot_x, self.robot_y)

    def imu_cb(self, msg: Imu):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        if self.initial_yaw is None:
            self.initial_yaw = self.current_yaw

    # ---------------------------
    # Target & planning utilities
    # ---------------------------
    def get_current_target(self):
        if self.sequence_index < len(self.balloon_sequence):
            return self.balloon_sequence[self.sequence_index]
        return None

    def plan_path_to_position(self, goal_x, goal_y):
        """Plan grid path from robot current world coordinates to a world goal (x,y)."""
        start_gx, start_gy = self.local_grid.world_to_grid(self.robot_x, self.robot_y)
        goal_gx, goal_gy = self.local_grid.world_to_grid(goal_x, goal_y)
        path_grid = self.planner.plan_path(start_gx, start_gy, goal_gx, goal_gy)
        if path_grid:
            self.current_path = []
            for gx, gy in path_grid:
                wx, wy = self.local_grid.grid_to_world(gx, gy)
                self.current_path.append((wx, wy))
            self.path_index = 0
            self.mission_state = "following_path"
            self.get_logger().info(f"📍 Path planned with {len(self.current_path)} waypoints")
            self.publish_path()
            return True
        else:
            self.get_logger().warn("❌ No path found to goal")
            return False

    def publish_path(self):
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

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        info = MapMetaData()
        info.resolution = self.local_grid.resolution
        info.width = self.local_grid.size
        info.height = self.local_grid.size
        # origin so grid_to_world(0,0) sits at origin position
        info.origin.position.x = -(self.local_grid.origin_x * self.local_grid.resolution)
        info.origin.position.y = -(self.local_grid.origin_y * self.local_grid.resolution)
        info.origin.orientation.w = 1.0
        msg.info = info

        # convert to ROS occupancy values: -1 unknown, 0 free, 100 occupied
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

    def update_exploration_goals(self):
        """Periodically generate an exploration goal within exploration_radius if exploring and no path."""
        if self.mission_state != "exploring":
            return
        # only every 5 seconds
        if not hasattr(self, '_last_explore_time'):
            self._last_explore_time = 0.0
        if time.time() - self._last_explore_time < 5.0:
            return
        self._last_explore_time = time.time()
        radius = safe_param(self, 'exploration_radius', 3.0)
        # sample several candidate frontier-like free cells near unknown areas, but simple: random sample around circle
        for attempt in range(10):
            ang = np.random.uniform(0, 2 * math.pi)
            r = np.random.uniform(0.5 * radius, radius)
            gx = self.robot_x + r * math.cos(ang)
            gy = self.robot_y + r * math.sin(ang)
            # try to plan to that point
            if self.plan_path_to_position(gx, gy):
                self.get_logger().info(f"🗺️ Exploration goal set: ({gx:.2f}, {gy:.2f})")
                return
        # fallback: small rotation / move (handled by control loop) if no path found

    # ---------------------------
    # Path following and motion
    # ---------------------------

    def follow_path(self):
        """Follow current_path using a simple look-at waypoint controller (pure pursuit-ish)."""
        if not self.current_path or self.path_index >= len(self.current_path):
            # finished path
            self.current_path = []
            self.mission_state = "exploring"
            return None  # nothing published by this function
        # next waypoint
        wx, wy = self.current_path[self.path_index]
        dx = wx - self.robot_x
        dy = wy - self.robot_y
        dist = math.hypot(dx, dy)
        tol = safe_param(self, 'stop_distance', 0.2)
        if dist < safe_param(self, 'path_following_tolerance', 0.2):
            self.path_index += 1
            if self.path_index >= len(self.current_path):
                # reached end
                self.current_path = []
                self.mission_state = "exploring"
                self.get_logger().info("✅ Reached path end")
            return None
        # compute heading control
        angle_to_goal = math.atan2(dy, dx)
        angle_error = math.atan2(math.sin(angle_to_goal - self.current_yaw), math.cos(angle_to_goal - self.current_yaw))
        v_max = safe_param(self, 'v_max', 0.2)
        w_max = safe_param(self, 'w_max', 0.6)
        cmd = Twist()
        # scale forward speed by heading error
        cmd.linear.x = max(0.05, v_max * math.cos(angle_error))
        cmd.angular.z = max(min(2.0 * angle_error, w_max), -w_max)
        self.cmd_pub.publish(cmd)
        return cmd

    # ---------------------------
    # Startup yaw motion
    # ---------------------------
    def perform_startup_yaw(self):
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
            yaw_error = math.atan2(math.sin(target_yaw - self.current_yaw), math.cos(target_yaw - self.current_yaw))
            cmd.angular.z = max(min(2.0 * yaw_error, w_max), -w_max)
            # minor forward nudge for visibility while yawing
            cmd.linear.x = 0.0
        else:
            # return to initial yaw, then finish startup
            yaw_error = math.atan2(math.sin(self.initial_yaw - self.current_yaw), math.cos(self.initial_yaw - self.current_yaw))
            if abs(yaw_error) > 0.05:
                cmd.angular.z = max(min(2.0 * yaw_error, w_max), -w_max)
            else:
                self.startup_phase = False
                self.get_logger().info("✅ Startup complete. Beginning mission.")
        return cmd

    # ---------------------------
    # Main control loop
    # ---------------------------

    def control_loop(self):
        # If in startup phase do yaw
        if self.startup_phase:
            cmd = self.perform_startup_yaw()
            self.cmd_pub.publish(cmd)
            return

        # Stop-phase (when balloon reached)
        if self.in_stop_phase:
            now = time.time()
            stop_duration = safe_param(self, 'stop_duration', 5.0)
            if now - self.stop_start_time < stop_duration:
                # hold still
                self.cmd_pub.publish(Twist())
                return
            elif now - self.stop_start_time < stop_duration + 1.0:
                # retreat a bit
                cmd = Twist()
                cmd.linear.x = safe_param(self, 'retreat_speed', -0.2)
                self.cmd_pub.publish(cmd)
                return
            else:
                # finish stop phase, go to next balloon
                self.in_stop_phase = False
                self.target_detected = None
                self.target_yaw = None
                self.sequence_index += 1
                self.mission_state = "exploring"
                self.get_logger().info(f"🔄 Moving to next balloon: {self.get_current_target()}")
                return

        # If a target was detected and we are in approach mode
        current_target = self.get_current_target()
        if self.target_detected and self.mission_state == "approaching_balloon":
            # If we have LiDAR distance, use it to decide when to stop
            safe_d = safe_param(self, 'safe_distance', 0.5)
            dist = self.min_distance_ahead if self.min_distance_ahead is not None else 1e6

            # If no map path exists (or too short), try to plan toward the detection direction:
            # compute a goal a short distance ahead along target yaw
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
                # stop robot
                self.cmd_pub.publish(Twist())
                return
            else:
                # follow path toward the object (if path exists)
                if self.mission_state == "following_path":
                    cmd = self.follow_path()
                    if cmd is None:
                        # fallback: move forward slower
                        cmd = Twist()
                        cmd.linear.x = safe_param(self, 'v_max', 0.2) * 0.5
                        cmd.angular.z = 0.0
                        self.cmd_pub.publish(cmd)
                    return
                else:
                    # fallback approach (no path)
                    cmd = Twist()
                    cmd.linear.x = safe_param(self, 'v_max', 0.2) * 0.4
                    cmd.angular.z = 0.0
                    self.cmd_pub.publish(cmd)
                    return

        # If currently following a planned path, continue following it
        if self.mission_state == "following_path":
            cmd = self.follow_path()
            # follow_path publishes cmd if it returns a Twist; if returns None it's handled there
            return

        # Otherwise, exploration scanning behavior (move-and-scan)
        # simple move-and-scan: forward slowly and wiggle rotation to discover objects
        cmd = Twist()
        scan_speed = max(0.05, safe_param(self, 'v_max', 0.2) * 0.5)
        cmd.linear.x = scan_speed
        # gentle oscillation rotation
        t = time.time()
        cmd.angular.z = 0.25 * math.sin(0.5 * t)
        # if obstacle ahead, slow/turn
        if self.min_distance_ahead is not None and self.min_distance_ahead < safe_param(self, 'safe_distance', 0.5):
            cmd.linear.x = 0.0
            cmd.angular.z = safe_param(self, 'w_max', 0.6)
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = BalloonDijkstraMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
