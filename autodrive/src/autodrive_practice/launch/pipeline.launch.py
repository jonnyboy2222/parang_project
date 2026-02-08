from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="autodrive_practice",
            executable="fake_lidar",
            name="fake_lidar",
            output="screen",
            parameters=[{
                "topic_out": "/fake/lidar_distance_m",
                "rate_hz": 10.0,
                "min_m": 0.5,
                "max_m": 20.0,
            }],
        ),
        Node(
            package="autodrive_practice",
            executable="safety_filter",
            name="safety_filter",
            output="screen",
            parameters=[{
                "topic_in": "/fake/lidar_distance_m",
                "topic_stop": "/safety/stop",
                "stop_threshold_m": 2.0,
            }],
        ),
        Node(
            package="autodrive_practice",
            executable="controller",
            name="controller",
            output="screen",
            parameters=[{
                "topic_stop": "/safety/stop",
                "topic_speed_cmd": "/cmd/speed",
                "default_speed": 5.0,
            }],
        ),
    ])
