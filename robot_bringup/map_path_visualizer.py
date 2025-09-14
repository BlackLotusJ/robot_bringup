#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String
import re

class MapPathVisualizer(Node):
    def __init__(self):
        super().__init__('map_path_visualizer')

        # Subscribers
        self.create_subscription(OccupancyGrid, '/local_map', self.map_callback, 10)
        self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.create_subscription(String, '/object_info', self.object_callback, 10)

        # Storage
        self.map_data = None
        self.map_info = None
        self.path = []
        self.balloons = []  # [(x, y, color)]
        self.current_goal = None  # (x, y)

        # Color mapping for balloons
        self.color_map = {
            "pink": "magenta",
            "yellow": "yellow",
            "black": "black",
            "white": "white",
            "blue": "blue",
            "green": "lime",
        }

        # Start live plot
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6, 6))

    def map_callback(self, msg: OccupancyGrid):
        """Convert occupancy grid to numpy and store metadata"""
        w, h = msg.info.width, msg.info.height
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(h, w)
        self.map_info = msg.info
        self.update_plot()

    def path_callback(self, msg: Path):
        """Extract path coordinates"""
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if self.path:
            self.current_goal = self.path[-1]  # last point is the goal
        self.update_plot()

    def object_callback(self, msg: String):
        """Parse balloon detections"""
        text = msg.data.lower()

        # Example: "Pink Balloon detected at (1.2, 3.4)"
        match = re.search(r"(\w+)\s+balloon.*\(([-+]?[0-9]*\.?[0-9]+),\s*([-+]?[0-9]*\.?[0-9]+)\)", text)
        if match:
            color, x, y = match.group(1), float(match.group(2)), float(match.group(3))
            self.balloons.append((x, y, color))
            self.get_logger().info(f"🎈 {color.capitalize()} balloon at ({x:.2f}, {y:.2f})")

        self.update_plot()

    def update_plot(self):
        if self.map_data is None:
            return

        self.ax.clear()

        # Plot occupancy grid
        self.ax.imshow(
            self.map_data,
            cmap=plt.cm.gray_r,
            origin='lower',
            extent=[
                0, self.map_info.width * self.map_info.resolution,
                0, self.map_info.height * self.map_info.resolution
            ]
        )

        # Overlay path
        if self.path:
            xs, ys = zip(*self.path)
            self.ax.plot(xs, ys, 'r-', linewidth=2, label="Planned Path")
            self.ax.scatter(xs[0], ys[0], c='g', marker='o', label="Start")
            if self.current_goal:
                self.ax.scatter(self.current_goal[0], self.current_goal[1], c='b', marker='x', label="Goal")

        # Overlay balloons
        for x, y, color in self.balloons:
            c = self.color_map.get(color, "cyan")
            self.ax.scatter(x, y, c=c, marker='*', s=160, edgecolors='k', linewidths=0.8, label=f"{color.capitalize()} Balloon")

        self.ax.set_title("Local Map + Path + Balloon Targets")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.legend(loc="upper right", fontsize=8)
        self.ax.grid(True, linestyle="--", alpha=0.5)

        plt.draw()
        plt.pause(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = MapPathVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
