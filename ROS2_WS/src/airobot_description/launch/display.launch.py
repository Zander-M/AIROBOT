# Example launch script that creates an environment with two robots.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_path = get_package_share_directory('airobot_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'airobot.urdf.xacro')

    # RViz config (optional)
    rviz_config = os.path.join(pkg_path, 'config', 'view.rviz')

    return LaunchDescription([
        # Robot 1
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='robot1',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file, ' prefix:=robot1/'])
            }],
            output='screen'
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "world", "robot1/odom"],
            name="static_tf_robot1"
        ),

        # Robot 2
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='robot2',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file, ' prefix:=robot2/'])
            }],
            output='screen'
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["1", "0", "0", "0", "0", "0", "world", "robot2/odom"],
            name="static_tf_robot2"
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
            output='screen'
        )
    ])
