"""
Visualizing trajectories stored as numpy arrays of form (x, y, yaw, t)
Each robot file: <trajectory_path>/<robot_ns>.npy  with shape (T, 4)
Columns: x, y, yaw, t
"""
from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import Path as PathMsg
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray


def _quat_from_yaw(yaw: float):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))  # x,y,z,w


class TrajectoryVisualizerNP(Node):
    def __init__(self):
        super().__init__("airobot_trajectory_visualizer_np")

        # ---- Parameters ----
        self.declare_parameter("trajectory_path", "")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("robot_ns", "robot0")

        self.declare_parameter("path_topic", "trajectory/path")
        self.declare_parameter("marker_topic", "trajectory/marker")
        self.declare_parameter("arrow_topic", "trajectory/arrows")

        # Arrow rendering controls
        self.declare_parameter("arrow_stride", 10)      # use every Nth waypoint
        self.declare_parameter("arrow_length", 0.25)    # meters (Marker scale.x)
        self.declare_parameter("arrow_shaft_d", 0.04)   # meters (Marker scale.y)
        self.declare_parameter("arrow_head_d", 0.08)    # meters (Marker scale.z)

        # Optional: republish periodically (usually not needed with TRANSIENT_LOCAL)
        self.declare_parameter("publish_rate_hz", 0.0)  # 0 => publish once

        self._traj: Optional[np.ndarray] = None
        self._cached_path: Optional[PathMsg] = None
        self._cached_strip: Optional[Marker] = None
        self._cached_arrows: Optional[MarkerArray] = None

        # ---- QoS: latch so Foxglove sees it even if it connects later ----
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        robot_ns = str(self.get_parameter("robot_ns").value)

        path_topic = f"{str(self.get_parameter('path_topic').value)}/{robot_ns}"
        marker_topic = f"{str(self.get_parameter('marker_topic').value)}/{robot_ns}"
        arrow_topic = f"{str(self.get_parameter('arrow_topic').value)}/{robot_ns}"

        self._path_pub = self.create_publisher(PathMsg, path_topic, latched_qos)
        self._strip_pub = self.create_publisher(Marker, marker_topic, latched_qos)
        self._arrow_pub = self.create_publisher(MarkerArray, arrow_topic, latched_qos)

        # Load + build
        self._load_npy(robot_ns)
        self._build_cached_messages(robot_ns)

        # Publish once immediately (latched)
        self._publish_cached_once()

        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if rate_hz > 0.0:
            self._timer = self.create_timer(1.0 / rate_hz, self._publish_cached_once)

        self.get_logger().info(
            f"TrajectoryVisualizerNP up.\n"
            f"  path='{path_topic}'\n"
            f"  strip='{marker_topic}'\n"
            f"  arrows='{arrow_topic}'"
        )

    def _load_npy(self, robot_ns: str) -> None:
        traj_dir = str(self.get_parameter("trajectory_path").value).strip()
        if not traj_dir:
            self.get_logger().error("trajectory_path is empty. Set -p trajectory_path:=/path/to/dir")
            return

        traj_dir_p = Path(traj_dir)
        fname = traj_dir_p / f"{robot_ns}.npy"
        if not fname.exists():
            self.get_logger().error(f"Trajectory file not found: {fname}")
            return

        try:
            arr = np.load(fname)
        except Exception as e:
            self.get_logger().error(f"Failed to load npy: {e}")
            return

        if arr.ndim != 2 or arr.shape[1] != 4:
            self.get_logger().error(f"Expected shape (T,4) [x,y,yaw,t], got {arr.shape}")
            return
        if arr.shape[0] < 2:
            self.get_logger().error("Trajectory too short (<2 points).")
            return

        self._traj = arr.astype(float, copy=False)
        t0 = float(self._traj[0, 3])
        tT = float(self._traj[-1, 3])
        self.get_logger().info(f"Loaded {fname.name}: T={self._traj.shape[0]}, t0={t0:.3f}, tT={tT:.3f}")

    def _build_cached_messages(self, robot_ns: str) -> None:
        if self._traj is None:
            return

        frame_id = str(self.get_parameter("frame_id").value)

        # ---- Build Path ----
        path_msg = PathMsg()
        path_msg.header.frame_id = frame_id

        poses = []
        for (x, y, yaw, _t) in self._traj:
            qx, qy, qz, qw = _quat_from_yaw(float(yaw))

            ps = PoseStamped()
            ps.header.frame_id = frame_id
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            poses.append(ps)

        path_msg.poses = poses

        # ---- Build Marker LINE_STRIP ----
        strip = Marker()
        strip.header.frame_id = frame_id
        strip.ns = f"trajectory_strip_{robot_ns}"
        strip.id = 0
        strip.type = Marker.LINE_STRIP
        strip.action = Marker.ADD
        strip.pose.orientation.w = 1.0
        strip.scale.x = 0.03  # line width (m)

        strip.color.r = 0.1
        strip.color.g = 1.0
        strip.color.b = 0.1
        strip.color.a = 1.0

        strip.points = [Point(x=float(x), y=float(y), z=0.0) for (x, y, _yaw, _t) in self._traj]

        # ---- Build MarkerArray of ARROWs ----
        stride = max(int(self.get_parameter("arrow_stride").value), 1)
        arrow_len = float(self.get_parameter("arrow_length").value)
        shaft_d = float(self.get_parameter("arrow_shaft_d").value)
        head_d = float(self.get_parameter("arrow_head_d").value)

        arrows = MarkerArray()
        mid = 0
        for idx in range(0, self._traj.shape[0], stride):
            x, y, yaw, _t = self._traj[idx]
            qx, qy, qz, qw = _quat_from_yaw(float(yaw))

            m = Marker()
            m.header.frame_id = frame_id
            m.ns = f"trajectory_arrows_{robot_ns}"
            m.id = mid
            mid += 1
            m.type = Marker.ARROW
            m.action = Marker.ADD

            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = 0.0
            m.pose.orientation.x = qx
            m.pose.orientation.y = qy
            m.pose.orientation.z = qz
            m.pose.orientation.w = qw

            # For ARROW: scale.x = length, scale.y = shaft diameter, scale.z = head diameter
            m.scale.x = arrow_len
            m.scale.y = shaft_d
            m.scale.z = head_d

            # Same color as strip (change if you want)
            m.color.r = 1.0
            m.color.g = 0.4
            m.color.b = 0.1
            m.color.a = 1.0

            arrows.markers.append(m)

        self._cached_path = path_msg
        self._cached_strip = strip
        self._cached_arrows = arrows

    def _publish_cached_once(self) -> None:
        stamp = self.get_clock().now().to_msg()

        if self._cached_path is not None:
            self._cached_path.header.stamp = stamp
            # Not strictly required for static display, but fine:
            for ps in self._cached_path.poses:
                ps.header.stamp = stamp
            self._path_pub.publish(self._cached_path)

        if self._cached_strip is not None:
            self._cached_strip.header.stamp = stamp
            self._strip_pub.publish(self._cached_strip)

        if self._cached_arrows is not None:
            for m in self._cached_arrows.markers:
                m.header.stamp = stamp
            self._arrow_pub.publish(self._cached_arrows)


def main():
    rclpy.init()
    node = TrajectoryVisualizerNP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

