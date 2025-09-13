#!/usr/bin/env python3
"""
Mapless Mission Controller with Balloon Sequence (Pink → Green → Yellow)
Enhanced with move-and-scan behavior and startup yaw acknowledgment
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Imu
import time
import math
import numpy as np


class MaplessMissionNode(Node):
    def __init__(self):
        super().__init__('mapless_mission_node')

        # --- Parameters ---
        self.declare_parameter('v_max', 0.2)
        self.declare_parameter('w_max', 0.4)
        self.declare_parameter('safe_distance', 0.5)
        self.declare_parameter('retreat_speed', -0.2)
        self.declare_parameter('stop_duration', 5.0)
        self.declare_parameter('yaw_gain', 1.5)
        self.declare_parameter('obstacle_threshold', 0.25)
        self.declare_parameter('use_lidar', True)
        self.declare_parameter('scan_speed', 0.15)  # Speed while scanning
        self.declare_parameter('approach_distance', 1.0)  # Distance to approach obstacles
        self.declare_parameter('startup_yaw_duration', 4.0)  # Duration for startup yaw motion

        # --- Balloon sequence ---
        self.balloon_sequence = ["pink", "green", "yellow"]
        self.sequence_index = 0  # start with first target

        # --- Startup acknowledgment ---
        self.startup_phase = True
        self.startup_start_time = time.time()
        self.initial_yaw = None
        self.yaw_direction = 1  # 1 for left, -1 for right
        self.yaw_cycles_completed = 0
        self.target_yaw_cycles = 2  # Number of left-right cycles

        # --- State ---
        self.target_detected = None
        self.visited_balloons = set()
        self.min_distance_ahead = None
        self.lidar_ranges = None
        self.in_stop_phase = False
        self.stop_start_time = None

        # --- Exploration states ---
        self.exploration_state = "scanning"  # "scanning", "approaching_obstacle", "investigating"
        self.obstacle_detected = False
        self.obstacle_direction = None
        self.investigation_start_time = None
        self.investigation_duration = 3.0  # Time to spend investigating an object
        self.last_rotation_time = time.time()
        self.rotation_interval = 5.0  # Change direction every 5 seconds while scanning

        # IMU / heading
        self.current_yaw = None
        self.target_yaw = None

        # --- Publishers ---
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # --- Subscribers ---
        self.create_subscription(String, '/object_info', self.detection_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)

        # --- Timers ---
        self.timer = self.create_timer(0.2, self.control_loop)

        self.get_logger().info("✅ Mapless Mission Node (Move and Scan) started...")
        self.get_logger().info("🤖 Performing startup acknowledgment yaw motion...")

    # --- Balloon detection callback ---
    def detection_cb(self, msg: String):
        # Don't process detections during startup phase
        if self.startup_phase:
            return
            
        data = msg.data.lower()
        current_target = self.get_current_target()

        if current_target and current_target in data:
            if data not in self.visited_balloons:
                self.target_detected = data
                self.target_yaw = self.current_yaw
                self.exploration_state = "approaching_balloon"
                self.get_logger().info(f"🎯 Target balloon detected: {data}")
            else:
                self.get_logger().info(f"⏭ Already visited: {data}")
        else:
            if data and data != "none":
                self.get_logger().info(f"👀 Found {data}, but looking for {current_target}")
                # Continue investigation for a bit longer to make sure
                if self.exploration_state == "investigating":
                    self.investigation_start_time = time.time()

    # --- LiDAR callback ---
    def lidar_cb(self, msg: LaserScan):
        if not self.get_parameter('use_lidar').value:
            return

        self.lidar_ranges = msg.ranges
        center_index = len(msg.ranges) // 2
        d = msg.ranges[center_index]
        if math.isinf(d):
            d = 10.0
        self.min_distance_ahead = d

        # Only detect obstacles after startup phase
        if not self.startup_phase:
            self.detect_obstacles_for_investigation(msg)

    def detect_obstacles_for_investigation(self, msg: LaserScan):
        """Detect obstacles that could be balloons to investigate"""
        approach_distance = self.get_parameter('approach_distance').get_parameter_value().double_value

        if self.exploration_state not in ["scanning"] or self.target_detected:
            return

        # Look for obstacles in front sector (±30 degrees)
        ranges = np.array(msg.ranges)
        center_idx = len(ranges) // 2
        sector_width = int(len(ranges) * 0.17)  # ±30 degrees approximately

        front_sector = ranges[center_idx - sector_width:center_idx + sector_width]
        valid_distances = front_sector[~np.isinf(front_sector)]

        if len(valid_distances) > 0:
            min_dist = np.min(valid_distances)
            if min_dist < approach_distance and min_dist > 0.1:  # Found something to investigate
                self.obstacle_detected = True
                self.exploration_state = "approaching_obstacle"
                self.get_logger().info(f"🔍 Obstacle detected at {min_dist:.2f}m, investigating...")

    # --- IMU callback ---
    def imu_cb(self, msg: Imu):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Store initial yaw for startup sequence
        if self.initial_yaw is None and self.current_yaw is not None:
            self.initial_yaw = self.current_yaw

    def angle_diff(self, a, b):
        d = a - b
        return math.atan2(math.sin(d), math.cos(d))

    def get_current_target(self):
        if self.sequence_index < len(self.balloon_sequence):
            return self.balloon_sequence[self.sequence_index]
        return None

    def perform_startup_yaw(self):
        """Perform left-right yaw motion to acknowledge system is working"""
        if self.current_yaw is None or self.initial_yaw is None:
            return Twist()  # Wait for IMU data
            
        startup_duration = self.get_parameter('startup_yaw_duration').get_parameter_value().double_value
        w_max = self.get_parameter('w_max').get_parameter_value().double_value
        
        current_time = time.time()
        elapsed_time = current_time - self.startup_start_time
        
        cmd = Twist()
        
        # Calculate yaw motion using sinusoidal pattern
        cycle_period = startup_duration / self.target_yaw_cycles  # Time for one left-right cycle
        yaw_amplitude = math.pi / 4  # ±45 degrees
        
        if elapsed_time < startup_duration:
            # Sinusoidal yaw motion
            phase = (elapsed_time % cycle_period) / cycle_period * 2 * math.pi
            target_yaw_offset = yaw_amplitude * math.sin(phase)
            target_yaw = self.initial_yaw + target_yaw_offset
            
            yaw_error = self.angle_diff(target_yaw, self.current_yaw)
            cmd.angular.z = max(min(2.0 * yaw_error, w_max), -w_max)
            
            # Log progress
            progress = (elapsed_time / startup_duration) * 100
            if int(elapsed_time * 2) % 2 == 0:  # Log every 0.5 seconds
                self.get_logger().info(f"🤖 Startup yaw motion: {progress:.0f}% complete")
                
        else:
            # Startup complete - return to initial position
            yaw_error = self.angle_diff(self.initial_yaw, self.current_yaw)
            if abs(yaw_error) > 0.1:  # 0.1 radian tolerance (~6 degrees)
                cmd.angular.z = max(min(2.0 * yaw_error, w_max), -w_max)
                self.get_logger().info("🎯 Returning to initial orientation...")
            else:
                # Startup sequence complete
                self.startup_phase = False
                self.get_logger().info("✅ Startup acknowledgment complete! Starting mission...")
                self.get_logger().info(f"🎯 Looking for first balloon: {self.get_current_target()}")
                cmd = Twist()  # Stop motion
        
        return cmd

    # --- Main control loop ---
    def control_loop(self):
        # Handle startup yaw acknowledgment
        if self.startup_phase:
            cmd = self.perform_startup_yaw()
            self.cmd_pub.publish(cmd)
            return

        cmd = Twist()

        v_max = self.get_parameter('v_max').get_parameter_value().double_value
        w_max = self.get_parameter('w_max').get_parameter_value().double_value
        safe_distance = self.get_parameter('safe_distance').get_parameter_value().double_value
        retreat_speed = self.get_parameter('retreat_speed').get_parameter_value().double_value
        stop_duration = self.get_parameter('stop_duration').get_parameter_value().double_value
        yaw_gain = self.get_parameter('yaw_gain').get_parameter_value().double_value
        obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value
        use_lidar = self.get_parameter('use_lidar').value
        scan_speed = self.get_parameter('scan_speed').get_parameter_value().double_value

        current_target = self.get_current_target()

        # --- If all balloons visited ---
        if current_target is None:
            self.get_logger().info("🎉 Mission complete! All balloons visited.")
            self.cmd_pub.publish(Twist())
            return

        # --- Stop phase (when balloon is reached) ---
        if self.in_stop_phase:
            now = time.time()
            if now - self.stop_start_time < stop_duration:
                self.get_logger().info("⏸ Holding at balloon...")
                cmd.linear.x = 0.0
            elif now - self.stop_start_time < stop_duration + 1.0:
                self.get_logger().info("↩ Retreating...")
                cmd.linear.x = retreat_speed
            else:
                self.in_stop_phase = False
                self.target_detected = None
                self.target_yaw = None
                self.sequence_index += 1  # go to next target
                self.exploration_state = "scanning"
                next_target = self.get_current_target()
                if next_target:
                    self.get_logger().info(f"🔄 Searching for next balloon: {next_target}")
                else:
                    self.get_logger().info("🎉 All balloons collected!")
            self.cmd_pub.publish(cmd)
            return

        # --- Target balloon approach ---
        if self.target_detected and self.exploration_state == "approaching_balloon":
            distance = self.min_distance_ahead if (use_lidar and self.min_distance_ahead is not None) else 1e6
            self.get_logger().info(f"📏 Distance to {current_target} balloon: {distance:.2f} m")

            # Check if we've reached the target distance (0.5m)
            if use_lidar and distance <= safe_distance:
                self.get_logger().info(f"🎈 Reached {current_target} balloon at {distance:.2f}m")
                self.visited_balloons.add(self.target_detected)
                self.in_stop_phase = True
                self.stop_start_time = time.time()
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                # Continue approaching the balloon
                yaw_error = 0.0
                if self.current_yaw is not None and self.target_yaw is not None:
                    yaw_error = self.angle_diff(self.target_yaw, self.current_yaw)

                # Slow down as we get closer to the target
                speed_factor = min(1.0, (distance - safe_distance + 0.2) / 0.5) if use_lidar else 1.0
                speed_factor = max(0.1, speed_factor)  # Minimum speed to avoid stopping too early
                
                cmd.linear.x = v_max * speed_factor
                cmd.angular.z = max(min(yaw_gain * yaw_error, w_max), -w_max)
                self.get_logger().info(f"➡ Approaching {current_target} balloon (dist={distance:.2f}m, speed={speed_factor:.2f})")


        # --- Exploration behavior ---
        else:
            if self.exploration_state == "scanning":
                # Move forward while rotating to scan the environment
                current_time = time.time()

                # Check for obstacles and maintain safe distance
                if (use_lidar and self.min_distance_ahead is not None and
                    self.min_distance_ahead <= safe_distance):
                    self.get_logger().warn(f"⚠ Obstacle at {self.min_distance_ahead:.2f}m - maintaining safe distance of {safe_distance}m")
                    cmd.linear.x = 0.0
                    cmd.angular.z = w_max  # Turn to find clear path
                elif (use_lidar and self.min_distance_ahead is not None and
                      self.min_distance_ahead < safe_distance + 0.3):
                    # Slow down when approaching safe distance
                    speed_factor = (self.min_distance_ahead - safe_distance) / 0.3
                    speed_factor = max(0.0, min(1.0, speed_factor))
                    cmd.linear.x = scan_speed * speed_factor
                    
                    if current_time - self.last_rotation_time > self.rotation_interval:
                        self.last_rotation_time = current_time
                        self.rotation_interval = 3.0 + 4.0 * (time.time() % 1)  # Random 3-7 seconds

                    # Add some rotation while moving
                    rotation_phase = (current_time - self.last_rotation_time) / self.rotation_interval
                    cmd.angular.z = 0.3 * math.sin(2 * math.pi * rotation_phase)
                    
                    self.get_logger().info(f"🐌 Slowing down near obstacle: {self.min_distance_ahead:.2f}m (speed={speed_factor:.2f})")
                else:
                    # Move forward and periodically change direction
                    cmd.linear.x = scan_speed

                    if current_time - self.last_rotation_time > self.rotation_interval:
                        self.last_rotation_time = current_time
                        self.rotation_interval = 3.0 + 4.0 * (time.time() % 1)  # Random 3-7 seconds

                    # Add some rotation while moving
                    rotation_phase = (current_time - self.last_rotation_time) / self.rotation_interval
                    cmd.angular.z = 0.3 * math.sin(2 * math.pi * rotation_phase)

                    self.get_logger().info(f"🔍 Scanning for {current_target} balloon...")

            elif self.exploration_state == "approaching_obstacle":
                # Move towards the detected obstacle but stop at safe distance
                if (use_lidar and self.min_distance_ahead is not None and
                    self.min_distance_ahead <= safe_distance):
                    # Reached safe distance - start investigation
                    self.exploration_state = "investigating"
                    self.investigation_start_time = time.time()
                    self.get_logger().info(f"🔎 Reached obstacle at safe distance ({self.min_distance_ahead:.2f}m) - starting investigation...")
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                else:
                    # Keep approaching but slow down as we get closer
                    if self.min_distance_ahead is not None:
                        # Progressive speed reduction as we approach safe_distance
                        speed_factor = min(1.0, (self.min_distance_ahead - safe_distance) / 0.5)
                        speed_factor = max(0.1, speed_factor)  # Minimum 10% speed
                        cmd.linear.x = scan_speed * speed_factor
                        cmd.angular.z = 0.0
                        self.get_logger().info(f"➡ Approaching obstacle: {self.min_distance_ahead:.2f}m (speed={speed_factor:.2f})")
                    else:
                        cmd.linear.x = scan_speed
                        cmd.angular.z = 0.0
                        self.get_logger().info("➡ Approaching obstacle for investigation...")

            elif self.exploration_state == "investigating":
                # Rotate around the object to get a good view
                current_time = time.time()
                if current_time - self.investigation_start_time < self.investigation_duration:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.4  # Slow rotation for investigation
                    self.get_logger().info("🔎 Investigating object...")
                else:
                    # Done investigating, resume scanning
                    self.exploration_state = "scanning"
                    self.obstacle_detected = False
                    self.last_rotation_time = time.time()
                    self.get_logger().info("📍 Investigation complete, resuming scan...")

        self.cmd_pub.publish(cmd)

    def stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = MaplessMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
