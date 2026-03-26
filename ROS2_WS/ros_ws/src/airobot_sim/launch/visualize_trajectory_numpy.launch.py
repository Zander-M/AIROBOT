"""
Visualize trajectory stored as numpy
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration 
from launch_ros.actions import Node


def generate_launch_description():
    trajectory_path = LaunchConfiguration("trajectory_path")
    frame_id = LaunchConfiguration("frame_id")

    trajectory_nodes = []
    for i in range(15):
        trajectory_nodes.append(
            Node(
                package="airobot_trajectory_visualizer",
                executable="trajectory_visualizer_numpy",
                name="trajectory_visualizer",
                output="screen",
                parameters=[{
                    "robot_ns": f"robot{i}",
                    "trajectory_path": trajectory_path,
                    "frame_id": frame_id,
                    "path_topic": "trajectory/path",
                    "marker_topic": "trajectory/marker",
                    "desired_topic": "trajectory/desired",
                    "arrow_stride": 4,
                }],
            )
        )

    return LaunchDescription([
        DeclareLaunchArgument("trajectory_path", default_value=f"{os.environ.get("AIROBOT_TRAJECTORY_DATA")}"),
        DeclareLaunchArgument("frame_id", default_value="odom"),
        DeclareLaunchArgument("robot_ns", default_value="robot0"),

        # Provides websocket for Foxglove to connect to
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[{
                "port": 8765,
            }],
        ),
    ] + trajectory_nodes)
