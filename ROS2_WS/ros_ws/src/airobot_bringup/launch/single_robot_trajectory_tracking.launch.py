# airobot_bringup/launch/sim_one_robot_track_one_traj.launch.py
#
# Matches current TrajectoryFollowerNode parameters exactly:
# - follower subscribes to experiment_control_topic / publishes experiment_ready_topic
# - no static world->odom TF here
# - trajectory visualizer defaults to world (traj is in world)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def robot(robot_id: int):
    ns = f"robot{robot_id}"
    prefix = f"{ns}/"

    # Launch args
    trajectory_pkg = LaunchConfiguration("trajectory_pkg")
    trajectory_file = LaunchConfiguration("trajectory_file")
    frame_id = LaunchConfiguration("frame_id")
    experiment_control_topic = LaunchConfiguration("experiment_control_topic")
    experiment_ready_topic = LaunchConfiguration("experiment_ready_topic")

    # URDF (xacro)
    urdf_file = PathJoinSubstitution([
        FindPackageShare("airobot_description"),
        "urdf",
        "airobot.urdf.xacro",
    ])

    urdf_xml = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            urdf_file, " ",
            "prefix:=", prefix,
        ]),
        value_type=str,
    )

    # Trajectory path: <trajectory_pkg_share>/trajectories/<trajectory_file>
    traj_path = PathJoinSubstitution([
        FindPackageShare(trajectory_pkg),
        "trajectories",
        trajectory_file,
    ])

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
        parameters=[{
            "rate_hz": 50.0,
            "publish_tf": True,
            "odom_frame": f"{prefix}odom",
            "base_frame": f"{prefix}base_link",
        }],
    )

    world_to_odom = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name=f"{ns}_world_to_odom",
    output="screen",
    arguments=[
        "--x", "0", "--y", "0", "--z", "0",
        "--roll", "0", "--pitch", "0", "--yaw", "0",
        "--frame-id", "world",
        "--child-frame-id", f"{prefix}odom",
    ],
    )


    follower = Node(
        package="airobot_tracking",
        executable="trajectory_follower",
        namespace=ns,
        name="trajectory_follower",
        output="screen",
        parameters=[{
            # follower params (exact names)
            "trajectory_path": traj_path,
            "robot_id": robot_id,
            "odom_topic": "odom",
            "cmd_vel_topic": "cmd_vel",
            "experiment_control_topic": experiment_control_topic,
            "experiment_ready_topic": experiment_ready_topic,

            # ready identity string
            "robot_ns": ns,

            # timing
            "rate_hz": 30.0,
            "loop_trajectory": False,
        }],

    )

    visualizer = Node(
        package="airobot_trajectory_visualizer",
        executable="trajectory_visualizer",
        namespace=ns,
        name="trajectory_visualizer",
        output="screen",
        parameters=[{
            "trajectory_path": traj_path,
            "frame_id": frame_id,  # default "world"
            "trajectory_index": robot_id,
            "path_topic": "trajectory/path",
            "marker_topic": "trajectory/marker",
            "desired_topic": "trajectory/desired",
        }],
    )


    return [rsp, sim, follower, visualizer, world_to_odom]


def generate_launch_description():
    args = [
        DeclareLaunchArgument("trajectory_pkg", default_value="airobot_data"),
        DeclareLaunchArgument("trajectory_file", default_value="test_sol_transformed.pkl"),

        # Trajectory is in WORLD coordinates
        DeclareLaunchArgument("frame_id", default_value="world"),

        # Matches follower params
        DeclareLaunchArgument("experiment_control_topic", default_value="experiment/control"),
        DeclareLaunchArgument("experiment_ready_topic", default_value="experiment/ready"),

        DeclareLaunchArgument("foxglove_port", default_value="8765"),
    ]

    nodes = []
    nodes += robot(robot_id=0)

    nodes.append(
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[{"port": LaunchConfiguration("foxglove_port"),
                         "qos_overrides./tf_static.subscription.durability": "transient_local",
                         "qos_overrides./tf_static.subscription.reliability": "reliable"},
                         ],
        )
    )

    return LaunchDescription(args + nodes)

