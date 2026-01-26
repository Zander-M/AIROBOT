from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    trajectory_pkg = LaunchConfiguration("trajectory_pkg")
    trajectory_file = LaunchConfiguration("trajectory_file")
    frame_id = LaunchConfiguration("frame_id")

    traj_path = PathJoinSubstitution([
        FindPackageShare(trajectory_pkg),
        "trajectories",
        trajectory_file,
    ])

    # Arguments 

    nodes = [
        DeclareLaunchArgument("trajectory_pkg", default_value="airobot_data"),
        DeclareLaunchArgument("trajectory_file", default_value="test_sol.pkl"),
        DeclareLaunchArgument("frame_id", default_value="odom"),
    ]

    # Trajectory Nodes
    for i in range(5):
        nodes +=[
        Node(
            package="airobot_trajectory_visualizer",
            executable="trajectory_visualizer",
            name="trajectory_visualizer",
            output="screen",
            parameters=[{
                "trajectory_path": traj_path,
                "frame_id": frame_id,
                "trajectory_index": i,
                "path_topic": "trajectory/path",
                "marker_topic": "trajectory/marker",
                "desired_topic": "trajectory/desired",
            }],
        )]

    # Foxglove
    nodes += [
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[{
                "port": 8765,
            }],
        )]

    return LaunchDescription(nodes)
