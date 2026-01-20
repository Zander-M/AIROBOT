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

    # Publish the URDF on a unique topic (so RViz can subscribe unambiguously)
    desc_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ns,
        name="description_publisher",
        output="screen",
        parameters=[
            {"robot_description": urdf_xml},
            {"publish_frequency": 0.0},
        ],
        remappings=[
            # RobotStatePublisher publishes to this topic name:
            # In many setups it is /robot_description; remap to a namespaced topic.
            ("/robot_description", f"/{ns}/robot_description"),
        ],
    )

    return [rsp, sim, static_tf_world_to_odom, desc_pub]



def generate_launch_description():
    nodes = []
    nodes += robot("robot1_", 0.0, 0.0, 0.0)
    nodes += robot("robot2_", 1.0, 0.0, 0.0)

    nodes += [Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
    )]

    return LaunchDescription(nodes)

