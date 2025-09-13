# robot_bringup
This is the package conytaining the autonoums mission files for the Jabari Mars Rover

# Balloon Dijkstra Mission Node

## Overview
This ROS 2 node combines:
- **LiDAR-based occupancy grid mapping**
- **Dijkstra path planning**
- **Balloon mission sequence**: pink → yellow → black → white → blue
- **Startup yaw acknowledgement** (left–right wiggle before mission)
- **Exploration fallback** with move-and-scan behavior

It publishes:
- `/local_map` (`nav_msgs/OccupancyGrid`) → view in RViz2 as a Map
- `/planned_path` (`nav_msgs/Path`) → view in RViz2 as a Path
- `/cmd_vel` (`geometry_msgs/Twist`) → sent to robot base

The node subscribes to:
- `/scan` (`sensor_msgs/LaserScan`) → from RPLiDAR or other 2D laser
- `/imu/data` (`sensor_msgs/Imu`) → for yaw orientation
- `/object_info` (`std_msgs/String`) → balloon detections (contains color name)

## Build
Clone into your workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/BlackLotusJ/robot_bringup/
cd ~/ros2_ws
colcon build --packages-select robot_bringup
source install/setup.bash
