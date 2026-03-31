from __future__ import annotations

from itertools import combinations
from typing import Dict, List, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class CollisionDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__("airobot_collision_detection")

        self.declare_parameter("num_robots", 2)
        self.declare_parameter("robot_start_index", 0)
        self.declare_parameter("robot_name_prefix", "robot")
        self.declare_parameter("collision_threshold", 1.0)
        self.declare_parameter("distance_frame", "world")
        self.declare_parameter("odom_topic_suffix", "/odom")
        self.declare_parameter("marker_topic", "collision_markers")
        self.declare_parameter("cylinder_height", 0.005)
        self.declare_parameter("marker_alpha", 0.35)
        self.declare_parameter("publish_rate_hz", 10.0)

        num_robots = int(self.get_parameter("num_robots").value)
        robot_start_index = int(self.get_parameter("robot_start_index").value)
        robot_name_prefix = str(self.get_parameter("robot_name_prefix").value)

        if num_robots < 2:
            raise ValueError("Parameter 'num_robots' must be at least 2.")

        self.robot_names: List[str] = [
            f"{robot_name_prefix}{idx}"
            for idx in range(robot_start_index, robot_start_index + num_robots)
        ]

        self.threshold = float(self.get_parameter("collision_threshold").value)
        if self.threshold <= 0.0:
            raise ValueError("Parameter 'collision_threshold' must be > 0.")

        self.distance_frame = str(self.get_parameter("distance_frame").value)
        self.odom_topic_suffix = str(self.get_parameter("odom_topic_suffix").value)
        marker_topic = str(self.get_parameter("marker_topic").value)
        self.cylinder_diameter = 2.0 * self.threshold
        self.cylinder_height = float(self.get_parameter("cylinder_height").value)
        self.alpha = float(self.get_parameter("marker_alpha").value)
        publish_rate_hz = max(float(self.get_parameter("publish_rate_hz").value), 1.0)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_positions: Dict[str, Tuple[float, float, float]] = {}
        self.latest_frame_ids: Dict[str, str] = {}
        self.subscribers = []
        for robot_name in self.robot_names:
            topic = self._odom_topic_for(robot_name)
            sub = self.create_subscription(
                Odometry,
                topic,
                lambda msg, name=robot_name: self._odom_callback(name, msg),
                10,
            )
            self.subscribers.append(sub)
            self.get_logger().info(f"Subscribed to odom topic: {topic}")

        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._publish_markers)

        self.get_logger().info(
            f"Collision detection running for {len(self.robot_names)} robots "
            f"({self.robot_names[0]}..{self.robot_names[-1]}), threshold={self.threshold:.3f} m, "
            f"cylinder_diameter={self.cylinder_diameter:.3f} m, distance_frame='{self.distance_frame}'."
        )

    def _odom_topic_for(self, robot_name: str) -> str:
        suffix = self.odom_topic_suffix if self.odom_topic_suffix.startswith("/") else f"/{self.odom_topic_suffix}"
        return f"/{robot_name}{suffix}"

    @staticmethod
    def _rotate_point_by_quaternion(
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ) -> Tuple[float, float, float]:
        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        rx = (1.0 - 2.0 * (yy + zz)) * x + (2.0 * (xy - wz)) * y + (2.0 * (xz + wy)) * z
        ry = (2.0 * (xy + wz)) * x + (1.0 - 2.0 * (xx + zz)) * y + (2.0 * (yz - wx)) * z
        rz = (2.0 * (xz - wy)) * x + (2.0 * (yz + wx)) * y + (1.0 - 2.0 * (xx + yy)) * z
        return rx, ry, rz

    def _transform_position_to_distance_frame(
        self,
        source_frame: str,
        x: float,
        y: float,
        z: float,
    ) -> Tuple[Tuple[float, float, float], str] | None:
        if source_frame == self.distance_frame:
            return (x, y, z), source_frame

        try:
            transform = self.tf_buffer.lookup_transform(
                self.distance_frame,
                source_frame,
                Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"TF unavailable for {source_frame} -> {self.distance_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return None

        t = transform.transform.translation
        q = transform.transform.rotation
        rx, ry, rz = self._rotate_point_by_quaternion(x, y, z, q.x, q.y, q.z, q.w)
        return (rx + t.x, ry + t.y, rz + t.z), self.distance_frame

    def _odom_callback(self, robot_name: str, msg: Odometry) -> None:
        source_frame = msg.header.frame_id.strip()
        p = msg.pose.pose.position

        transformed = self._transform_position_to_distance_frame(
            source_frame,
            float(p.x),
            float(p.y),
            float(p.z),
        )
        if transformed is None:
            return

        position, frame_id = transformed
        self.latest_positions[robot_name] = position
        self.latest_frame_ids[robot_name] = frame_id

    def _compute_collision_flags(self) -> Dict[str, bool]:
        flags = {name: False for name in self.robot_names}

        ready_robots = [name for name in self.robot_names if name in self.latest_positions]
        for robot_a, robot_b in combinations(ready_robots, 2):
            if self.latest_frame_ids.get(robot_a) != self.latest_frame_ids.get(robot_b):
                continue

            xa, ya, za = self.latest_positions[robot_a]
            xb, yb, zb = self.latest_positions[robot_b]
            distance = ((xa - xb) ** 2 + (ya - yb) ** 2 + (za - zb) ** 2) ** 0.5
            if distance < self.threshold:
                flags[robot_a] = True
                flags[robot_b] = True

        return flags

    def _publish_markers(self) -> None:
        if not self.latest_positions:
            return

        collision_flags = self._compute_collision_flags()
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        for idx, robot_name in enumerate(self.robot_names):
            if robot_name not in self.latest_positions:
                continue

            x, y, z = self.latest_positions[robot_name]

            marker = Marker()
            marker.header.stamp = now
            marker.header.frame_id = self.latest_frame_ids.get(robot_name, self.distance_frame)
            marker.ns = "airobot_collision_detection"
            marker.id = idx
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z + 0.5 * self.cylinder_height
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.cylinder_diameter
            marker.scale.y = self.cylinder_diameter
            marker.scale.z = self.cylinder_height

            if collision_flags[robot_name]:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            marker.color.a = self.alpha

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CollisionDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
