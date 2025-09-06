#!/usr/bin/env python3
"""
Safety Monitor for Autonomous Missions
Monitors robot health and environmental conditions
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')

        # Parameters
        self.declare_parameter('battery_threshold', 20.0)
        self.declare_parameter('obstacle_distance', 0.3)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('heartbeat_timeout', 5.0)

        # Safety state
        self.battery_level = 100.0
        self.last_heartbeat = self.get_clock().now()
        self.emergency_stop_active = False

        # Publishers
        self.safety_alert_pub = self.create_publisher(String, 'safety/alert', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_safe', 10)

        # Subscribers
        self.battery_sub = self.create_subscription(
            BatteryState, 'battery_state', self.battery_callback, 10
        )

        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )

        self.emergency_stop_sub = self.create_subscription(
            Bool, 'emergency_stop', self.emergency_stop_callback, 10
        )

        # Timer for safety checks
        self.safety_timer = self.create_timer(0.1, self.safety_check)  # 10 Hz

        self.get_logger().info("Safety Monitor initialized")

    def battery_callback(self, msg):
        """Monitor battery level"""
        self.battery_level = msg.percentage

        battery_threshold = self.get_parameter('battery_threshold').get_parameter_value().double_value

        if self.battery_level < battery_threshold:
            self.publish_alert(f"Battery low: {self.battery_level:.1f}%", "CRITICAL")
        elif self.battery_level < battery_threshold + 10:
            self.publish_alert(f"Battery getting low: {self.battery_level:.1f}%", "WARNING")

    def laser_callback(self, msg):
        """Monitor for obstacles"""
        obstacle_distance = self.get_parameter('obstacle_distance').get_parameter_value().double_value

        if not msg.ranges:
            return

        # Check front 180 degrees
        ranges_to_check = msg.ranges[len(msg.ranges)//4:3*len(msg.ranges)//4]

        min_distance = float('inf')
        for r in ranges_to_check:
            if msg.range_min <= r <= msg.range_max:
                min_distance = min(min_distance, r)

        if min_distance < obstacle_distance:
            self.publish_alert(f"Close obstacle detected: {min_distance:.2f}m", "WARNING")

    def cmd_vel_callback(self, msg):
        """Monitor and limit velocity commands"""
        max_linear = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        max_angular = self.get_parameter('max_angular_vel').get_parameter_value().double_value

        # Limit velocities
        limited_msg = Twist()

        limited_msg.linear.x = max(-max_linear, min(max_linear, msg.linear.x))
        limited_msg.angular.z = max(-max_angular, min(max_angular, msg.angular.z))

        # Check if we had to limit
        if abs(msg.linear.x) > max_linear or abs(msg.angular.z) > max_angular:
            self.publish_alert("Velocity command limited for safety", "INFO")

        # Apply emergency stop if active
        if self.emergency_stop_active:
            limited_msg = Twist()  # Zero all velocities

        # Publish safe velocity
        self.cmd_vel_pub.publish(limited_msg)

    def emergency_stop_callback(self, msg):
        """Handle emergency stop commands"""
        self.emergency_stop_active = msg.data

        if self.emergency_stop_active:
            self.publish_alert("Emergency stop activated", "CRITICAL")
            # Immediately stop robot
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
        else:
            self.publish_alert("Emergency stop deactivated", "INFO")

    def safety_check(self):
        """Periodic safety checks"""
        # Check for system heartbeat
        current_time = self.get_clock().now()
        heartbeat_timeout = self.get_parameter('heartbeat_timeout').get_parameter_value().double_value

        # Add more safety checks here as needed
        pass

    def publish_alert(self, message, level):
        """Publish safety alert"""
        alert_msg = String()
        alert_msg.data = f"{level}:{message}"
        self.safety_alert_pub.publish(alert_msg)

        # Log based on severity
        if level == "CRITICAL":
            self.get_logger().error(f"SAFETY: {message}")
        elif level == "WARNING":
            self.get_logger().warn(f"SAFETY: {message}")
        else:
            self.get_logger().info(f"SAFETY: {message}")

def main(args=None):
    rclpy.init(args=args)

    safety_monitor = SafetyMonitor()

    try:
        rclpy.spin(safety_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        safety_monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
