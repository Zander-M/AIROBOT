from __future__ import annotations

import math
import pickle
from pathlib import Path
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import Path as PathMsg
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker

from airobot_common import STTrajectory

def _yaw_from_dxdy(dx: float, dy: float) -> float:
    if abs(dx) + abs(dy) < 1e-12:
        return 0.0
    return math.atan2(dy, dx)


def _quat_from_yaw(yaw: float):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))  # x,y,z,w


class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__("airobot_trajectory_visualizer")

        # ---- Parameters ----
        self.declare_parameter("trajectory_path", "")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("trajectory_index", 0)

        self.declare_parameter("path_topic", "trajectory/path")
        self.declare_parameter("marker_topic", "trajectory/marker")

        self.declare_parameter("sample_dt_s", 0.05) # sampling resolution along trajectory time
        self.declare_parameter("publish_rate_hz", 1.0)    # republish latched messages periodically

        self._traj: Optional[STTrajectory] = None
        self._cached_path: Optional[PathMsg] = None
        self._cached_marker: Optional[Marker] = None

        self._node_start = self.get_clock().now()

        # ---- QoS: latch so Foxglove sees it even if it connects later ----
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        traj_idx = int(self.get_parameter("trajectory_index").value)

        path_topic = str(self.get_parameter("path_topic").value)
        marker_topic = str(self.get_parameter("marker_topic").value)

        path_topic = f"{path_topic}_{traj_idx}"
        marker_topic = f"{marker_topic}_{traj_idx}"

        self._path_pub = self.create_publisher(PathMsg, path_topic, latched_qos)
        self._marker_pub = self.create_publisher(Marker, marker_topic, latched_qos)

        self._load_trajectory()
        self._build_cached_messages()

        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._timer = self.create_timer(1.0 / max(rate_hz, 1e-6), self._on_timer)

        self.get_logger().info(
            f"TrajectoryVisualizer up. path='{path_topic}', marker='{marker_topic}'"
        )

    def _load_trajectory(self) -> None:
        path_str = str(self.get_parameter("trajectory_path").value).strip()
        if not path_str:
            self.get_logger().warn("No trajectory_path set. Provide -p trajectory_path:=/path/to/traj.pkl")
            return

        p = Path(path_str)
        if not p.exists():
            self.get_logger().error(f"Trajectory file not found: {p}")
            return

        try:
            with p.open("rb") as f:
                trajs_dicts = pickle.load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load trajectory pickle: {e}")
            return
        
        traj_idx = self.get_parameter("trajectory_index").value

        if len(trajs_dicts) <= traj_idx:
            self.get_logger().error(f"Invalid trajectory index {traj_idx}. Displaying the first trajectory.")
            traj_idx = 0

        traj = STTrajectory.from_dict(trajs_dicts[traj_idx])

        if traj.size <= 0:
            self.get_logger().error("Trajectory is empty.")
            return
        if traj.dim < 2:
            self.get_logger().error(f"Trajectory dim seems wrong: traj.dim={trajs_dicts.dim} (includes time)")
            return

        self._traj = traj

        self.get_logger().info(
            f"Loaded trajectory: segments={traj.size}, dim_with_time={traj.dim}, "
            f"t0={traj.x0[-1]:.3f}, tT={traj.xT[-1]:.3f}, duration={traj.duration:.3f}s"
        )

    def _build_cached_messages(self) -> None:
        if self._traj is None:
            return

        frame_id = str(self.get_parameter("frame_id").value)
        sample_dt = max(float(self.get_parameter("sample_dt_s").value), 1e-3)

        t0 = float(self._traj.x0[-1])
        tT = float(self._traj.xT[-1])

        n = int(math.ceil((tT - t0) / sample_dt)) + 1
        ts = np.linspace(t0, tT, num=max(n, 2), dtype=float)

        # ---- Build Path ----
        path_msg = PathMsg()
        path_msg.header.frame_id = frame_id

        poses = []
        for i, t in enumerate(ts):
            xyt = np.asarray(self._traj.lerp(float(t)), dtype=float)
            pos = xyt[:-1]

            # yaw via finite difference
            if i < len(ts) - 1:
                xyt2 = np.asarray(self._traj.lerp(float(ts[i + 1])), dtype=float)
                d = xyt2[:-1] - pos
            else:
                xyt1 = np.asarray(self._traj.lerp(float(ts[i - 1])), dtype=float)
                d = pos - xyt1[:-1]

            yaw = _yaw_from_dxdy(float(d[0]), float(d[1]))
            qx, qy, qz, qw = _quat_from_yaw(yaw)

            ps = PoseStamped()
            ps.header.frame_id = frame_id
            ps.pose.position.x = float(pos[0])
            ps.pose.position.y = float(pos[1])
            ps.pose.position.z = 0.0
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            poses.append(ps)

        path_msg.poses = poses

        # ---- Build Marker LINE_STRIP ----
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = "trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.03  # line width (m)

        # A visible default; tweak if you want
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.1
        marker.color.a = 1.0

        pts = []
        for ps in poses:
            pt = Point()
            pt.x = ps.pose.position.x
            pt.y = ps.pose.position.y
            pt.z = 0.0
            pts.append(pt)
        marker.points = pts

        self._cached_path = path_msg
        self._cached_marker = marker

    def _on_timer(self) -> None:
        stamp = self.get_clock().now().to_msg()

        if self._cached_path is not None:
            self._cached_path.header.stamp = stamp
            for ps in self._cached_path.poses:
                ps.header.stamp = stamp
            self._path_pub.publish(self._cached_path)

        if self._cached_marker is not None:
            self._cached_marker.header.stamp = stamp
            self._marker_pub.publish(self._cached_marker)


def main():
    rclpy.init()
    node = TrajectoryVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
