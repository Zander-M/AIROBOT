# airobot_bringup/launch/sim_multi_robot_track_trajs.launch.py
#
# Multi-robot version of sim_one_robot_track_one_traj.launch.py
# - Per-robot RSP (xacro with prefix:=robot{i}/)
# - Per-robot simple_diff_drive (publish_tf True)
# - Per-robot static TF: world -> robot{i}/odom
# - Per-robot trajectory_follower
# - Per-robot trajectory_visualizer
#
# Usage:
#   ros2 launch airobot_bringup sim_multi_robot_track_trajs.launch.py \
#     num_robots:=15 trajectory_pkg:=airobot_data trajectory_file:=test_sol_transformed.pkl

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def _robot_nodes(context, robot_id: int):
    ns = f"robot{robot_id}"
    prefix = f"{ns}/"

    # Launch args
    trajectory_pkg = LaunchConfiguration("trajectory_pkg").perform(context)
    trajectory_file = LaunchConfiguration("trajectory_file").perform(context)
    frame_id = LaunchConfiguration("frame_id").perform(context)
    experiment_control_topic = LaunchConfiguration("experiment_control_topic").perform(context)
    experiment_ready_topic = LaunchConfiguration("experiment_ready_topic").perform(context)

    sim_rate_hz = float(LaunchConfiguration("sim_rate_hz").perform(context))
    follower_rate_hz = float(LaunchConfiguration("follower_rate_hz").perform(context))
    loop_trajectory = LaunchConfiguration("loop_trajectory").perform(context).lower() in ("1", "true", "yes")

    # URDF (xacro) exactly like your reference
    urdf_file = PathJoinSubstitution(
        [FindPackageShare("airobot_description"), "urdf", "airobot.urdf.xacro"]
    )
    urdf_xml = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                urdf_file,
                " ",
                "prefix:=",
                prefix,
            ]
        ),
        value_type=str,
    )

    # Trajectory path: <trajectory_pkg_share>/trajectories/<trajectory_file>
    traj_path = PathJoinSubstitution(
        [FindPackageShare(trajectory_pkg), "trajectories", trajectory_file]
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ns,
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": urdf_xml}],
    )

    sim = Node(
        package="airobot_sim",
        executable="simple_diff_drive",
        namespace=ns,
        name="simple_diff_drive",
        output="screen",
        parameters=[
            {
                "rate_hz": sim_rate_hz,
                "publish_tf": True,
                "odom_frame": f"{prefix}odom",
                "base_frame": f"{prefix}base_link",
            }
        ],
    )

    # IMPORTANT: static TF world -> robot{i}/odom (you said it's missing)
    world_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"{ns}_world_to_odom",
        output="screen",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            frame_id,
            "--child-frame-id",
            f"{prefix}odom",
        ],
    )

    follower = Node(
        package="airobot_tracking",
        executable="trajectory_follower",
        namespace=ns,
        name="trajectory_follower",
        output="screen",
        parameters=[
            {
                # follower params (match your reference EXACTLY)
                "trajectory_path": traj_path,
                "robot_id": robot_id,
                "odom_topic": "odom",
                "cmd_vel_topic": "cmd_vel",
                "experiment_control_topic": experiment_control_topic,
                "experiment_ready_topic": experiment_ready_topic,
                "robot_ns": ns,
                "rate_hz": follower_rate_hz,
                "loop_trajectory": loop_trajectory,
            }
        ],
    )

    visualizer = Node(
        package="airobot_trajectory_visualizer",
        executable="trajectory_visualizer",
        namespace=ns,
        name="trajectory_visualizer",
        output="screen",
        parameters=[
            {
                "trajectory_path": traj_path,
                "frame_id": frame_id,
                "trajectory_index": robot_id,
                "path_topic": "trajectory/path",
                "marker_topic": "trajectory/marker",
                "desired_topic": "trajectory/desired",
            }
        ],
    )

    return [rsp, sim, world_to_odom, follower, visualizer]


def _make_all_nodes(context, *args, **kwargs):
    num_robots = int(LaunchConfiguration("num_robots").perform(context))
    nodes = []

    for rid in range(num_robots):
        nodes += _robot_nodes(context, robot_id=rid)

    # Optional controller (OFF by default)
    enable_controller = LaunchConfiguration("enable_controller").perform(context).lower() in ("1", "true", "yes")
    if enable_controller:
        nodes.append(
            Node(
                package="airobot_bringup",
                executable="experiment_controller_simple",
                name="experiment_controller_simple",
                output="screen",
                parameters=[
                    {
                        "control_topic": LaunchConfiguration("experiment_control_topic"),
                        "trajectory_path": "",  # if your controller needs it, wire it here
                        "setpose_service": "set_pose",
                        "start_delay_s": 0.25,
                        "rate_hz": 30.0,
                        "num_robots": num_robots,
                    }
                ],
            )
        )

    # Foxglove
    nodes.append(
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[
                {
                    "port": LaunchConfiguration("foxglove_port"),
                    "qos_overrides./tf_static.subscription.durability": "transient_local",
                    "qos_overrides./tf_static.subscription.reliability": "reliable",
                }
            ],
        )
    )

    return nodes


def generate_launch_description():
    args = [
        DeclareLaunchArgument("num_robots", default_value="15"),

        DeclareLaunchArgument("trajectory_pkg", default_value="airobot_data"),
        DeclareLaunchArgument("trajectory_file", default_value="test_sol_transformed.pkl"),

        # Trajectory is in WORLD coordinates
        DeclareLaunchArgument("frame_id", default_value="world"),

        # Matches follower params
        DeclareLaunchArgument("experiment_control_topic", default_value="experiment/control"),
        DeclareLaunchArgument("experiment_ready_topic", default_value="experiment/ready"),

        DeclareLaunchArgument("sim_rate_hz", default_value="50.0"),
        DeclareLaunchArgument("follower_rate_hz", default_value="30.0"),
        DeclareLaunchArgument("loop_trajectory", default_value="false"),

        DeclareLaunchArgument("enable_controller", default_value="false"),

        DeclareLaunchArgument("foxglove_port", default_value="8765"),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=_make_all_nodes)])

