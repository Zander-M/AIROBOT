# airobot_bringup/launch/sim_multi_robot_track_numpy_trajs.launch.py
#
# Multi-robot sim + numpy follower (x,y,yaw,t)
# - RSP per robot (xacro prefix)
# - simple_diff_drive per robot
# - static TF: world -> robot{i}/odom
# - numpy trajectory follower per robot
# - (optional) numpy trajectory visualizer per robot
# - foxglove bridge

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def _robot(robot_id: int, context):
    ns = f"robot{robot_id}"
    prefix = f"{ns}/"

    # args
    traj_dir = LaunchConfiguration("trajectory_path").perform(context)
    frame_id = LaunchConfiguration("frame_id").perform(context)

    control_topic = LaunchConfiguration("control_topic").perform(context)
    odom_topic = LaunchConfiguration("odom_topic").perform(context)
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic").perform(context)

    sim_rate_hz = float(LaunchConfiguration("sim_rate_hz").perform(context))
    follower_rate_hz = float(LaunchConfiguration("follower_rate_hz").perform(context))

    # follower gains/limits
    k_rho = float(LaunchConfiguration("k_rho").perform(context))
    k_alpha = float(LaunchConfiguration("k_alpha").perform(context))
    k_yaw = float(LaunchConfiguration("k_yaw").perform(context))
    v_max = float(LaunchConfiguration("v_max").perform(context))
    w_max = float(LaunchConfiguration("w_max").perform(context))
    hold_at_end = LaunchConfiguration("hold_at_end").perform(context).lower() in ("1", "true", "yes")

    publish_traj_viz = LaunchConfiguration("publish_traj_viz").perform(context).lower() in ("1", "true", "yes")

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

    world_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"{ns}_world_to_odom",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", frame_id,
            "--child-frame-id", f"{prefix}odom",
        ],
    )

    follower = Node(
        package="airobot_tracking",
        executable="trajectory_follower_numpy",   # <-- your new follower executable
        namespace=ns,
        name="trajectory_follower_numpy",
        output="screen",
        parameters=[
            {
                "trajectory_path": traj_dir,   # directory with robot{i}.npy
                "robot_ns": ns,

                "control_topic": control_topic,
                "odom_topic": odom_topic,
                "cmd_vel_topic": cmd_vel_topic,

                "rate_hz": follower_rate_hz,

                "k_rho": k_rho,
                "k_alpha": k_alpha,
                "k_yaw": k_yaw,
                "v_max": v_max,
                "w_max": w_max,
                "hold_at_end": hold_at_end,
            }
        ],
    )



    nodes = [rsp, sim, world_to_odom, follower, ]

    # Optional trajectory visualizer for numpy (publishes Path/Marker so Foxglove can show it)
    if publish_traj_viz:
        nodes.append(
            Node(
                package="airobot_trajectory_visualizer",
                executable="trajectory_visualizer_numpy",  # <-- if you have it
                namespace=ns,
                name="trajectory_visualizer_numpy",
                output="screen",
                parameters=[
                    {
                        "trajectory_path": traj_dir,
                        "robot_ns": ns,
                        "frame_id": frame_id,
                        "path_topic": "trajectory/path",
                        "marker_topic": "trajectory/marker",
                    }
                ],
            )
        )
    

    return nodes


def _make_nodes(context, *args, **kwargs):
    num_robots = int(LaunchConfiguration("num_robots").perform(context))
    collision_threshold = float(LaunchConfiguration("collision_threshold").perform(context))
    nodes = []

    for rid in range(num_robots):
        nodes += _robot(rid, context)

    # Collision Checking node
    collision_detection = Node(
        package="airobot_collision_detection",
        executable="airobot_collision_detection",
        name="collision_detection",
        output="screen",
        parameters=[
            {
                "num_robots": num_robots,
                "collision_threshold": collision_threshold 
            }
        ],
    )
    nodes.append(collision_detection)

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
    return LaunchDescription(
        [
            DeclareLaunchArgument("num_robots", default_value="15"),
            DeclareLaunchArgument("collision_threshold", default_value="0.08"),

            # directory with metadata.json and robot{i}.npy
            DeclareLaunchArgument("trajectory_dir", default_value=""),

            DeclareLaunchArgument("frame_id", default_value="world"),

            # numpy follower topic names (relative)
            DeclareLaunchArgument("control_topic", default_value="experiment/control"),
            DeclareLaunchArgument("odom_topic", default_value="odom"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="cmd_vel"),

            DeclareLaunchArgument("sim_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("follower_rate_hz", default_value="50.0"),

            DeclareLaunchArgument("k_rho", default_value="1.2"),
            DeclareLaunchArgument("k_alpha", default_value="3.0"),
            DeclareLaunchArgument("k_yaw", default_value="2.0"),
            DeclareLaunchArgument("v_max", default_value="0.6"),
            DeclareLaunchArgument("w_max", default_value="2.5"),
            DeclareLaunchArgument("hold_at_end", default_value="true"),

            DeclareLaunchArgument("publish_traj_viz", default_value="true"),

            DeclareLaunchArgument("foxglove_port", default_value="8765"),

            OpaqueFunction(function=_make_nodes),
        ]
    )

