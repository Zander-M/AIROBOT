from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros2_aruco_position",
                executable="aruco_tf_node",
                name="aruco_tf_node",
                output="screen",
                parameters=[
                    {
                        "video_device": "/dev/video2",
                        "camera_frame": "camera",
                        "camera_calibration_file": "",
                        "detected_ids_topic": "/aruco/detected_ids",
                        "marker_length": 0.05,
                        "aruco_dictionary": "DICT_4X4_50",
                        "fallback_fx": 800.0,
                        "fallback_fy": 800.0,
                        "capture_width": 640,
                        "capture_height": 480,
                        "capture_fps": 8.0,
                        "processing_scale": 0.75,
                        "frame_skip": 0,
                        "use_grayscale": True,
                        "enable_kalman_filter": True,
                        "kalman_process_noise": 5e-3,
                        "kalman_measurement_noise": 5e-4,
                    }
                ],
            )
        ]
    )
