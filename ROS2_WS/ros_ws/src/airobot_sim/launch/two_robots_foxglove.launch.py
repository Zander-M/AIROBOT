from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare


def robot(prefix: str, x: float, y: float, z: float):
    ns = prefix[:-1]  # robot1_ -> robot1

    urdf_file = PathJoinSubstitution([
        FindPackageShare("airobot_description"),
        "urdf",
        "airobot.urdf.xacro",
    ])

    urdf_xml = ParameterValue(Command([
        FindExecutable(name="xacro"), " ",
        urdf_file, " ",
        "prefix:=", prefix,
    ]),
        value_type=str,
    )

    # Publishes TF for the robot from the URDF
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
        parameters=[{"rate_hz": 50.0,
                     "publish_tf": True,
                     "odom_frame": f"{prefix}odom",
                     "base_frame": f"{prefix}base_link",
                     }],
    )   
    static_tf_world_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"static_tf_{ns}",
        output="screen",
        arguments=[
            "--x", str(x), "--y", str(y), "--z", str(z),
            "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
            "--frame-id", "world",
            "--child-frame-id", f"{prefix}odom",
        ],
    )

    return [rsp, sim, static_tf_world_to_odom]



def generate_launch_description():
    nodes = []
    nodes += robot("robot1/", 0.0, 0.0, 0.0)
    nodes += robot("robot2/", 1.0, 0.0, 0.0)

    # Foxglove bridge (WebSocket server)
    nodes += [Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        # optional parameters:
        # parameters=[{"port": 8765}],
    )]

    return LaunchDescription(nodes)

