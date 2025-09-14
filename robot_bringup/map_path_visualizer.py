#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid, Path

class MapPathVisualizer(Node):
    def __init__(self):
        super().__init__('map_path_visualizer')

        # Subscribers
        self.create_subscription(OccupancyGrid, '/local_map', self.map_callback, 10)
        self.create_subscription(Path, '/planned_path', self.path_callback, 10)

        # Storage
        self.map_data = None
        self.map_info = None
        self.path = []

        # Start live plot
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6,6))

    def map_callback(self, msg: OccupancyGrid):
        """Convert occupancy grid to numpy and plot"""
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape(h, w)

        # Store for overlay
        self.map_data = data
        self.map_info = msg.info

        self.update_plot()

    def path_callback(self, msg: Path):
        """Extract path coordinates"""
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.update_plot()

    def update_plot(self):
        if self.map_data is None:
            return

        self.ax.clear()

        # Plot occupancy grid
        # Values: -1=unknown, 0=free, 100=occupied
        cmap = plt.cm.gray_r
        self.ax.imshow(
            self.map_data,
            cmap=cmap,
            origin='lower',
            extent=[
                0, self.map_info.width * self.map_info.resolution,
                0, self.map_info.height * self.map_info.resolution
            ]
        )

        # Overlay path if available
        if self.path:
            xs, ys = zip(*self.path)
            self.ax.plot(xs, ys, 'r-', linewidth=2, label="Planned Path")
            self.ax.scatter(xs[0], ys[0], c='g', marker='o', label="Start")
            self.ax.scatter(xs[-1], ys[-1], c='b', marker='x', label="Goal")

        self.ax.set_title("Local Map + Planned Path")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.legend()
        plt.draw()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = MapPathVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
