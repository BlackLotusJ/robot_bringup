from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="robot_bringup", 
            executable="balloon_dijkstra_mission_node",
            name="balloon_dijkstra_mission",
            output="screen",
            parameters=[{
                "v_max": 0.25,
                "w_max": 0.8,
                "safe_distance": 0.5,
                "retreat_speed": -0.2,
                "stop_duration": 5.0,
                "yaw_gain": 1.5,
                "exploration_radius": 3.0,
                "startup_yaw_duration": 4.0,
                "map_publish_rate": 1.0,
                "use_lidar": True,
                "map_max_range": 5.0
            }]
        )
    ])
