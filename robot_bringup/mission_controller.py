#!/usr/bin/env python3
"""
Mission Controller for robot_bringup package
Integrated with your existing SLAM and navigation setup
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

import yaml
import time
from enum import Enum
from typing import List, Dict, Any

# ROS2 message types
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool
from sensor_msgs.msg import BatteryState, LaserScan
from nav_msgs.msg import Odometry

class MissionState(Enum):
    IDLE = "idle"
    PLANNING = "planning"
    NAVIGATING = "navigating"
    AT_WAYPOINT = "at_waypoint"
    EXECUTING_TASK = "executing_task"
    PAUSED = "paused"
    COMPLETED = "completed"
    FAILED = "failed"
    EMERGENCY_STOP = "emergency_stop"

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        # Initialize callback groups
        self.callback_group = ReentrantCallbackGroup()

        # Declare parameters
        self.declare_parameters()

        # State variables
        self.current_state = MissionState.IDLE
        self.current_waypoint_index = 0
        self.waypoints: List[Dict[str, Any]] = []
        self.mission_start_time = None
        self.task_start_time = None
        self.is_paused = False
        self.emergency_stop = False

        # Robot status
        self.battery_level = 100.0
        self.current_pose = None
        self.obstacle_detected = False

        # Setup ROS2 interfaces
        self.setup_action_clients()
        self.setup_publishers()
        self.setup_subscribers()
        self.setup_timers()

        # Load mission
        self.load_mission_config()

        self.get_logger().info("Mission Controller initialized for robot_bringup")

    def declare_parameters(self):
        """Declare ROS2 parameters"""
        self.declare_parameter('waypoints_file', 'config/mission_waypoints.yaml')
        self.declare_parameter('loop_mission', True)
        self.declare_parameter('mission_timeout', 600.0)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('max_linear_vel', 0.3)  # Conservative for your robot
        self.declare_parameter('max_angular_vel', 0.5)
        self.declare_parameter('battery_threshold', 20.0)
        self.declare_parameter('obstacle_distance', 0.5)

    def setup_action_clients(self):
        """Initialize action clients"""
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )

    def setup_publishers(self):
        """Initialize publishers"""
        self.mission_status_pub = self.create_publisher(String, 'mission/status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, 'emergency_stop', 10)

    def setup_subscribers(self):
        """Initialize subscribers"""
        # Odometry from your EKF
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odometry_callback, 10,
            callback_group=self.callback_group
        )

        # Laser scan from RPLiDAR
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10,
            callback_group=self.callback_group
        )

        # Mission commands
        self.mission_cmd_sub = self.create_subscription(
            String, 'mission/command', self.mission_command_callback, 10,
            callback_group=self.callback_group
        )

    def setup_timers(self):
        """Initialize timers"""
        self.mission_timer = self.create_timer(1.0, self.mission_callback)
        self.status_timer = self.create_timer(0.5, self.publish_status)

    def load_mission_config(self):
        """Load mission waypoints from YAML"""
        try:
            waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value

            # Try package share directory first
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg_dir = get_package_share_directory('robot_bringup')
                waypoints_path = f"{pkg_dir}/{waypoints_file}"
            except:
                waypoints_path = waypoints_file

            with open(waypoints_path, 'r') as file:
                config = yaml.safe_load(file)
                self.waypoints = config.get('waypoints', [])

                self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")

        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")
            self.waypoints = []

    def create_pose_stamped(self, waypoint: Dict[str, Any]) -> PoseStamped:
        """Create PoseStamped from waypoint"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        pose.pose.position.x = float(waypoint.get('x', 0.0))
        pose.pose.position.y = float(waypoint.get('y', 0.0))
        pose.pose.position.z = 0.0

        # Orientation (yaw to quaternion)
        yaw = float(waypoint.get('yaw', 0.0))
        import math
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose

    def navigate_to_waypoint(self, waypoint_index: int) -> bool:
        """Navigate to waypoint using Nav2"""
        if waypoint_index >= len(self.waypoints):
            return False

        waypoint = self.waypoints[waypoint_index]
        pose = self.create_pose_stamped(waypoint)

        # Wait for action server
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Nav2 not available")
            return False

        # Send goal
        goal = NavigateToPose.Goal()
        goal.pose = pose

        waypoint_name = waypoint.get('name', f'waypoint_{waypoint_index}')
        self.get_logger().info(f"Navigating to: {waypoint_name}")

        future = self.nav_to_pose_client.send_goal_async(goal)
        future.add_done_callback(self.navigation_response_callback)

        self.current_state = MissionState.NAVIGATING
        return True

    def navigation_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal rejected")
            self.current_state = MissionState.FAILED
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        if result:
            self.get_logger().info("Reached waypoint")
            self.current_state = MissionState.AT_WAYPOINT
            self.task_start_time = time.time()
        else:
            self.get_logger().warn("Navigation failed")
            self.current_state = MissionState.FAILED

    def execute_waypoint_task(self):
        """Execute task at waypoint"""
        if self.current_waypoint_index >= len(self.waypoints):
            return

        waypoint = self.waypoints[self.current_waypoint_index]
        task_duration = waypoint.get('duration', 2.0)
        task_type = waypoint.get('task', 'wait')

        # Check if task completed
        if self.task_start_time and (time.time() - self.task_start_time) >= task_duration:
            self.get_logger().info(f"Task '{task_type}' completed")
            self.advance_to_next_waypoint()
            return

        # Execute task
        self.current_state = MissionState.EXECUTING_TASK

        if task_type == 'scan':
            self.execute_scan_task()
        elif task_type == 'wait':
            self.execute_wait_task()
        else:
            self.execute_wait_task()  # Default

    def execute_scan_task(self):
        """Rotate robot for scanning"""
        twist = Twist()
        twist.angular.z = 0.3  # Slow rotation
        self.cmd_vel_pub.publish(twist)

    def execute_wait_task(self):
        """Stop and wait"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def advance_to_next_waypoint(self):
        """Move to next waypoint"""
        self.current_waypoint_index += 1

        if self.current_waypoint_index >= len(self.waypoints):
            loop_mission = self.get_parameter('loop_mission').get_parameter_value().bool_value
            if loop_mission:
                self.current_waypoint_index = 0
                self.mission_start_time = time.time()
                self.current_state = MissionState.PLANNING
                self.get_logger().info("Mission loop restarting")
            else:
                self.current_state = MissionState.COMPLETED
                self.get_logger().info("Mission completed")
        else:
            self.current_state = MissionState.PLANNING

    def mission_callback(self):
        """Main mission state machine"""
        if self.emergency_stop or self.is_paused:
            return

        # Check timeout
        mission_timeout = self.get_parameter('mission_timeout').get_parameter_value().double_value
        if (self.mission_start_time and
            (time.time() - self.mission_start_time) > mission_timeout):
            self.current_state = MissionState.FAILED
            self.get_logger().warn("Mission timeout")
            return

        # Safety checks
        if self.check_safety_conditions():
            self.emergency_stop_mission()
            return

        # State machine
        if self.current_state == MissionState.IDLE:
            auto_start = self.get_parameter('auto_start').get_parameter_value().bool_value
            if auto_start and self.waypoints:
                self.start_mission()

        elif self.current_state == MissionState.PLANNING:
            if self.current_waypoint_index < len(self.waypoints):
                self.navigate_to_waypoint(self.current_waypoint_index)

        elif self.current_state == MissionState.AT_WAYPOINT:
            self.execute_waypoint_task()

        elif self.current_state == MissionState.COMPLETED:
            twist = Twist()
            self.cmd_vel_pub.publish(twist)

        elif self.current_state == MissionState.FAILED:
            twist = Twist()
            self.cmd_vel_pub.publish(twist)

    def check_safety_conditions(self) -> bool:
        """Check if emergency stop needed"""
        # Check battery
        battery_threshold = self.get_parameter('battery_threshold').get_parameter_value().double_value
        if self.battery_level < battery_threshold:
            self.get_logger().error("Battery too low!")
            return True

        # Check obstacles (if very close)
        if self.obstacle_detected:
            self.get_logger().warn("Close obstacle detected")
            return False  # Don't emergency stop, let Nav2 handle

        return False

    def start_mission(self):
        """Start mission execution"""
        if not self.waypoints:
            self.get_logger().error("No waypoints to execute")
            return

        self.mission_start_time = time.time()
        self.current_waypoint_index = 0
        self.current_state = MissionState.PLANNING
        self.is_paused = False
        self.emergency_stop = False

        self.get_logger().info("Starting autonomous mission")

    def pause_mission(self):
        """Pause mission"""
        self.is_paused = True
        self.current_state = MissionState.PAUSED
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Mission paused")

    def resume_mission(self):
        """Resume mission"""
        self.is_paused = False
        if self.current_state == MissionState.PAUSED:
            self.current_state = MissionState.PLANNING
        self.get_logger().info("Mission resumed")

    def stop_mission(self):
        """Stop mission"""
        self.current_state = MissionState.IDLE
        self.is_paused = False
        self.mission_start_time = None
        self.current_waypoint_index = 0
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Mission stopped")

    def emergency_stop_mission(self):
        """Emergency stop"""
        self.emergency_stop = True
        self.current_state = MissionState.EMERGENCY_STOP
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_stop_pub.publish(emergency_msg)

        self.get_logger().error("EMERGENCY STOP ACTIVATED")

    def publish_status(self):
        """Publish mission status"""
        status_msg = String()
        status_msg.data = f"{self.current_state.value}|{self.current_waypoint_index}|{len(self.waypoints)}"
        self.mission_status_pub.publish(status_msg)

    # Callbacks
    def odometry_callback(self, msg):
        """Process odometry data"""
        self.current_pose = msg.pose.pose

    def laser_callback(self, msg):
        """Process laser scan for obstacle detection"""
        obstacle_distance = self.get_parameter('obstacle_distance').get_parameter_value().double_value

        # Check front ranges for close obstacles
        if msg.ranges:
            front_ranges = msg.ranges[len(msg.ranges)//4:3*len(msg.ranges)//4]
            min_distance = min([r for r in front_ranges if r > msg.range_min and r < msg.range_max])
            self.obstacle_detected = min_distance < obstacle_distance

    def mission_command_callback(self, msg):
        """Handle mission commands"""
        command = msg.data.lower()

        if command == 'start':
            self.start_mission()
        elif command == 'pause':
            self.pause_mission()
        elif command == 'resume':
            self.resume_mission()
        elif command == 'stop':
            self.stop_mission()
        elif command == 'emergency_stop':
            self.emergency_stop_mission()
        else:
            self.get_logger().warn(f"Unknown command: {command}")

def main(args=None):
    rclpy.init(args=args)

    mission_controller = MissionController()

    try:
        rclpy.spin(mission_controller)
    except KeyboardInterrupt:
        pass
    finally:
        mission_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
