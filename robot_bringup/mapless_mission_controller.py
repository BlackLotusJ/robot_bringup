#!/usr/bin/env python3
"""
Mapless Mission Controller for Cars4Mars balloon task
- Uses LiDAR clusters to seek nearest obstacle (balloon)
- Approaches to standoff distance
- Triggers camera to classify color
- Logs order of colors found, then continues until timeout or no targets

Assumptions:
- LiDAR publishes /scan (sensor_msgs/LaserScan)
- Vision node listens on /balloon/trigger (std_msgs/Bool) and publishes /balloon/color (std_msgs/String)
- Motor stack listens on /cmd_vel (geometry_msgs/Twist)
- Optional: /emergency_stop (std_msgs/Bool) to stop immediately

Tested with Python 3.10, rclpy, ROS 2 Humble/Jazzy-style APIs
"""

import math
import time
from enum import Enum
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class MState(Enum):
    IDLE = 0
    SCANNING = 1
    TARGET_LOCKED = 2
    APPROACHING = 3
    ANALYZING = 4
    LOGGING = 5
    COMPLETED = 6
    EMERGENCY_STOP = 7
    FAILED = 8

class MaplessMissionController(Node):
    def __init__(self):
        super().__init__('mapless_mission_controller')
        self.cb_group = ReentrantCallbackGroup()

        # --- Parameters ---
        self.declare_parameters(
            namespace='',
            parameters=[
                ('auto_start', True),
                ('mission_timeout', 600.0),
                ('loop_mission', False),

                # LiDAR / perception
                ('min_target_range', 0.2),          # ignore < 20 cm (noise/robot body)
                ('max_target_range', 4.5),          # rplidar A1 typical useful up to ~6m; tune
                ('cluster_jump', 0.25),             # range discontinuity threshold for cluster split (m)
                ('min_cluster_points', 4),          # keep small to catch balloons; tune with your scan res
                ('max_cluster_width', 0.8),         # max arc length across cluster (m), balloons are small
                ('front_fov_deg', 200.0),           # allow wide FOV to acquire targets (±100°)

                # Control
                ('standoff_distance', 0.6),         # stop this far from cluster center (m)
                ('v_max', 0.35),
                ('w_max', 0.8),
                ('k_ang', 1.8),                     # P-gain for heading
                ('k_lin', 0.6),                     # P-gain for forward speed
                ('ang_deadband_deg', 3.0),

                # Balloon color order to find (if set, we’ll prefer next needed color when vision is available)
                ('target_color_order', ['red', 'blue', 'green']),
                ('require_color_order', False),     # if True, skip balloons whose color != next in order

                # Analysis timing
                ('analyze_hold_secs', 1.5),         # time to hold still for camera
                ('analyze_timeout_secs', 5.0),      # if no color arrives, give up and move on
            ]
        )

        # --- State ---
        self.state = MState.IDLE
        self.mission_start = None
        self.colors_found: List[str] = []
        self.last_scan: Optional[LaserScan] = None
        self.target_bearing: Optional[float] = None  # radians, in laser frame
        self.target_range: Optional[float] = None
        self.analyze_started = None

        # --- ROS I/O ---
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'mission/status', 10)
        self.trigger_pub = self.create_publisher(Bool, 'balloon/trigger', 10)
        self.estop_pub = self.create_publisher(Bool, 'emergency_stop', 10)  # optional: to notify others

        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_cb, 10, callback_group=self.cb_group
        )
        self.color_sub = self.create_subscription(
            String, 'balloon/color', self.color_cb, 10, callback_group=self.cb_group
        )
        self.cmd_sub = self.create_subscription(
            String, 'mission/command', self.cmd_cb, 10, callback_group=self.cb_group
        )
        self.estop_sub = self.create_subscription(
            Bool, 'emergency_stop', self.estop_cb, 10, callback_group=self.cb_group
        )

        self.timer = self.create_timer(0.05, self.tick)   # 20 Hz control
        self.status_timer = self.create_timer(0.5, self.pub_status)

        if self.get_parameter('auto_start').get_parameter_value().bool_value:
            self.start()

        self.get_logger().info("Mapless Mission Controller ready.")

    # ---------- Subscriptions ----------
    def scan_cb(self, msg: LaserScan):
        self.last_scan = msg

    def color_cb(self, msg: String):
        # Only consume during ANALYZING
        if self.state == MState.ANALYZING:
            color = msg.data.strip().lower()
            self.get_logger().info(f"Color detected: {color}")
            self.colors_found.append(color)
            self.state = MState.LOGGING

    def cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'start': self.start()
        elif cmd == 'stop': self.stop()
        elif cmd == 'pause': self.state = MState.IDLE; self.halt()
        elif cmd == 'resume': self.state = MState.SCANNING
        elif cmd == 'emergency_stop': self.emergency_stop()

    def estop_cb(self, msg: Bool):
        if msg.data:
            self.emergency_stop()

    # ---------- Mission control ----------
    def start(self):
        self.mission_start = time.time()
        self.colors_found.clear()
        self.state = MState.SCANNING
        self.get_logger().info("Mission started (mapless).")

    def stop(self):
        self.state = MState.COMPLETED
        self.halt()
        self.get_logger().info("Mission stopped.")

    def emergency_stop(self):
        self.state = MState.EMERGENCY_STOP
        self.halt()
        # announce
        msg = Bool(); msg.data = True
        self.estop_pub.publish(msg)
        self.get_logger().error("EMERGENCY STOP ACTIVATED")

    # ---------- Periodic ----------
    def pub_status(self):
        status = f"{self.state.name}|colors:{','.join(self.colors_found)}"
        self.status_pub.publish(String(data=status))

    def tick(self):
        # timeouts
        tmo = self.get_parameter('mission_timeout').get_parameter_value().double_value
        if self.mission_start and (time.time() - self.mission_start) > tmo and self.state not in (MState.COMPLETED, MState.EMERGENCY_STOP):
            self.get_logger().warn("Mission timeout.")
            self.state = MState.COMPLETED
            self.halt()
            return

        if self.state in (MState.COMPLETED, MState.EMERGENCY_STOP, MState.FAILED, MState.IDLE):
            self.halt()
            return

        if self.state == MState.SCANNING:
            self.seek_target()

        elif self.state == MState.TARGET_LOCKED:
            # small centering before moving in
            self.center_on_target(pre_drive=True)

        elif self.state == MState.APPROACHING:
            self.approach_target()

        elif self.state == MState.ANALYZING:
            self.halt()
            self.analyze_phase()

        elif self.state == MState.LOGGING:
            self.after_logging()

    # ---------- Core behaviors ----------
    def seek_target(self):
        if not self.last_scan:
            self.slow_spin()
            return

        # cluster scan into small obstacles
        clusters = self.cluster_scan(self.last_scan)

        if not clusters:
            # keep rotating to search
            self.slow_spin()
            return

        # Choose target: nearest cluster, with optional color-order preference (if we already know next color via prior hints—usually we won't)
        target = min(clusters, key=lambda c: c[0])  # (range, bearing, width)
        r, b, w = target
        self.target_range, self.target_bearing = r, b
        self.state = MState.TARGET_LOCKED
        self.get_logger().info(f"Target locked: range={r:.2f} m, bearing={math.degrees(b):.1f}°")

    def center_on_target(self, pre_drive=False):
        if self.target_bearing is None or self.target_range is None:
            self.state = MState.SCANNING
            return

        # rotate to reduce bearing error
        cmd = Twist()
        ang_err = self.target_bearing
        k_ang = self.get_parameter('k_ang').get_parameter_value().double_value
        w = max(-self.w_max(), min(self.w_max(), k_ang * ang_err))
        # deadband
        if abs(math.degrees(ang_err)) < self.get_parameter('ang_deadband_deg').get_parameter_value().double_value:
            w = 0.0

        if pre_drive:
            # very small forward nudge to keep engaged with cluster
            cmd.linear.x = 0.0
            cmd.angular.z = w
            self.cmd_pub.publish(cmd)
            if abs(w) < 0.05:
                self.state = MState.APPROACHING
            return

        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def approach_target(self):
        if not self.last_scan:
            self.halt()
            return

        # Re-estimate bearing/range each tick in case target moved in scan
        clusters = self.cluster_scan(self.last_scan)
        if not clusters:
            # lost target, go back to scanning
            self.get_logger().warn("Lost target; resuming scan.")
            self.state = MState.SCANNING
            return

        # pick cluster closest in bearing to previous target to maintain lock
        prev_b = self.target_bearing if self.target_bearing is not None else 0.0
        r, b, w = min(clusters, key=lambda c: abs(c[1] - prev_b))
        self.target_range, self.target_bearing = r, b

        standoff = self.get_parameter('standoff_distance').get_parameter_value().double_value
        if r <= standoff:
            self.halt()
            # Trigger camera to analyze color
            self.trigger_camera(True)
            self.analyze_started = time.time()
            self.state = MState.ANALYZING
            self.get_logger().info("At standoff: analyzing color...")
            return

        # Drive with simple P-control
        cmd = Twist()
        k_lin = self.get_parameter('k_lin').get_parameter_value().double_value
        k_ang = self.get_parameter('k_ang').get_parameter_value().double_value

        ang_err = b
        cmd.angular.z = max(-self.w_max(), min(self.w_max(), k_ang * ang_err))

        # reduce speed as we near standoff
        dist_err = max(0.0, r - standoff)
        v = k_lin * dist_err
        v = max(0.0, min(self.v_max(), v))
        # also slow down if misaligned
        v *= max(0.0, 1.0 - min(1.0, abs(ang_err) / math.radians(45)))
        cmd.linear.x = v

        self.cmd_pub.publish(cmd)

    def analyze_phase(self):
        if self.analyze_started is None:
            self.analyze_started = time.time()

        hold = self.get_parameter('analyze_hold_secs').get_parameter_value().double_value
        tmo  = self.get_parameter('analyze_timeout_secs').get_parameter_value().double_value

        elapsed = time.time() - self.analyze_started

        # Hold still for the first bit to get a clean frame
        if elapsed < hold:
            return

        # If color already arrived, LOGGING state will handle it
        # If not arrived by timeout, give up and move on
        if elapsed > tmo:
            self.get_logger().warn("Color analysis timed out; moving on.")
            self.state = MState.LOGGING

    def after_logging(self):
        # Stop triggering camera
        self.trigger_camera(False)

        # If we enforce a color order, check it
        enforce = self.get_parameter('require_color_order').get_parameter_value().bool_value
        if enforce:
            order = [s.lower() for s in self.get_parameter('target_color_order').get_parameter_value().string_array_value]
            next_needed = order[len([c for c in self.colors_found if c in order]) % len(order)]
            if self.colors_found and self.colors_found[-1] != next_needed:
                self.get_logger().info(f"Detected '{self.colors_found[-1]}' but expected '{next_needed}'. Continuing search.")

        # Decide to continue or complete
        if self.should_complete():
            self.state = MState.COMPLETED
            self.halt()
            self.get_logger().info(f"Mission completed. Colors: {self.colors_found}")
        else:
            # Reset target and scan for the next
            self.target_bearing = None
            self.target_range = None
            self.state = MState.SCANNING

    def should_complete(self) -> bool:
        # If enforcing order, complete after we’ve seen all in order at least once
        enforce = self.get_parameter('require_color_order').get_parameter_value().bool_value
        if enforce:
            order = [s.lower() for s in self.get_parameter('target_color_order').get_parameter_value().string_array_value]
            found_seq = [c for c in self.colors_found if c in order]
            return len(found_seq) >= len(order)

        # Otherwise, stop if timeout occurred (handled earlier) or if not looping we can keep going forever.
        return False

    # ---------- Helpers ----------
    def v_max(self) -> float:
        return self.get_parameter('v_max').get_parameter_value().double_value

    def w_max(self) -> float:
        return self.get_parameter('w_max').get_parameter_value().double_value

    def halt(self):
        self.cmd_pub.publish(Twist())

    def slow_spin(self, speed: float = 0.25):
        cmd = Twist()
        cmd.angular.z = speed
        self.cmd_pub.publish(cmd)

    def trigger_camera(self, on: bool):
        msg = Bool(); msg.data = bool(on)
        self.trigger_pub.publish(msg)

    # LiDAR clustering tuned for small, roundish obstacles (balloons)
    def cluster_scan(self, scan: LaserScan) -> List[Tuple[float, float, float]]:
        """
        Returns list of clusters as (range_m, bearing_rad, width_m)
        Only within front_fov and range limits; filters by cluster size/width.
        """
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        ranges = list(scan.ranges)

        # FOV window
        fov_deg = float(self.get_parameter('front_fov_deg').get_parameter_value().double_value)
        half = math.radians(fov_deg / 2.0)
        # indices within [-half, +half]
        idx_min = max(0, int(( -half - angle_min) / angle_inc))
        idx_max = min(len(ranges)-1, int(( +half - angle_min) / angle_inc))
        if idx_min >= idx_max:
            idx_min, idx_max = 0, len(ranges)-1

        rmin = self.get_parameter('min_target_range').get_parameter_value().double_value
        rmax = self.get_parameter('max_target_range').get_parameter_value().double_value
        jump = self.get_parameter('cluster_jump').get_parameter_value().double_value
        min_pts = int(self.get_parameter('min_cluster_points').get_parameter_value().integer_value)
        max_width = self.get_parameter('max_cluster_width').get_parameter_value().double_value

        clusters: List[Tuple[float, float, float]] = []
        cur: List[Tuple[int, float]] = []

        def flush_cluster():
            nonlocal clusters, cur
            if len(cur) < max(2, min_pts):
                cur = []
                return
            # compute centroid in polar and estimate arc width
            idxs = [i for (i, r) in cur]
            rs   = [r for (i, r) in cur]
            angs = [angle_min + i*angle_inc for i in idxs]

            r_avg = sum(rs)/len(rs)
            a_avg = math.atan2(sum(math.sin(a) for a in angs), sum(math.cos(a) for a in angs))
            # approximate arc width
            a_span = max(angs) - min(angs)
            width = r_avg * abs(a_span)
            if rmin <= r_avg <= rmax and width <= max_width:
                clusters.append((r_avg, a_avg, width))
            cur = []

        prev_r = None
        for i in range(idx_min, idx_max+1):
            r = ranges[i]
            if math.isfinite(r) and rmin <= r <= rmax:
                if prev_r is None or abs(r - prev_r) < jump:
                    cur.append((i, r))
                else:
                    flush_cluster()
                    cur = [(i, r)]
                prev_r = r
            else:
                flush_cluster()
                prev_r = None
                cur = []
        flush_cluster()

        return clusters

def main(args=None):
    rclpy.init(args=args)
    node = MaplessMissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.halt()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
