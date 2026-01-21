"""
Spatio-temporal trajectory follower ROS2 node
"""

from __future__ import annotations

import math
import pickle
from pathlib import Path
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from st_trajectory import STTrajectory


# --- Utils ---

def _wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _yaw_from_quat_xyzw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class State2D:
    x: float
    y: float
    yaw: float
    stamp: Time


# --- Node ---

class TrajectoryFollowerNode(Node):
    """
    Trajectory Follower Node for tracking STTrajectory
    """
    def __init__(self):
        super().__init__("trajectory_follower")

        # --- Parameters ---
        # Topics
        self.declare_parameter("trajectory_path", "")   # FIXED
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("control_topic", "experiment/control")  # NEW

        # Timing
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("loop_trajectory", False)

        # Controller gains / limits
        self.declare_parameter("k_v", 0.8)
        self.declare_parameter("k_w", 2.0)
        self.declare_parameter("v_max", 0.4)
        self.declare_parameter("w_max", 1.5)
        self.declare_parameter("goal_tolerance_m", 0.05)
        self.declare_parameter("slowdown_radius_m", 0.30)

        # Heading derivation
        self.declare_parameter("heading_lookahead_s", 0.1)

        # Internal states
        self._traj: Optional[STTrajectory] = None
        self._state: Optional[State2D] = None
        self._t0_ros: Optional[Time] = None
        self._finished: bool = False
        self._enabled: bool = False  # NEW: controlled by keyboard

        # ROS interface
        odom_topic = self.get_parameter("odom_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        control_topic = self.get_parameter("control_topic").value

        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._odom_sub = self.create_subscription(Odometry, odom_topic, self.cb_odom, 10)
        self._ctrl_sub = self.create_subscription(String, control_topic, self.cb_control, 10)

        rate_hz = float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate_hz, 1e-6), self.cb_timer)

        self.load_trajectory()

        self.get_logger().info(
            f"TrajectoryFollowerNode up. odom='{odom_topic}', cmd_vel='{cmd_vel_topic}', control='{control_topic}'"
        )

    def load_trajectory(self) -> None:
        """
        Load Trajectory from pickle
        """
        path_str = str(self.get_parameter("trajectory_path").value).strip()
        if not path_str:
            self.get_logger().warn("No trajectory_path set. Provide -p trajectory_path:=/path/to/traj.pkl")
            return

        path = Path(path_str)
        if not path.exists():
            self.get_logger().error(f"Trajectory file not found: {path}")
            return

        try:
            with path.open("rb") as f:
                obj = pickle.load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load trajectory pickle: {e}")
            return

        if not isinstance(obj, STTrajectory):
            self.get_logger().error(f"Pickle did not contain STTrajectory. Got: {type(obj)}")
            return  # IMPORTANT

        self._traj = obj

        if self._traj.size <= 0:
            self.get_logger().error("Trajectory is empty.")
            self._traj = None
            return

        if self._traj.dim < 2:
            self.get_logger().error(f"Trajectory dim seems wrong: traj.dim={self._traj.dim} (includes time)")
            self._traj = None
            return

        self.get_logger().info(
            f"Loaded trajectory: segments={self._traj.size}, dim_with_time={self._traj.dim}, "
            f"t0={self._traj.x0[-1]:.3f}, tT={self._traj.xT[-1]:.3f}, duration={self._traj.duration:.3f}s"
        )

    def cb_control(self, msg: String) -> None:
        """
        Experiment control callback: expects 'start', 'stop', 'reset'
        """
        cmd = msg.data.strip().lower()

        if cmd in ("start", "run", "go"):
            if self._traj is None:
                self.get_logger().warn("START received but trajectory not loaded.")
                return
            if self._state is None:
                self.get_logger().warn("START received but no odom yet.")
                return

            self._enabled = True
            self._finished = False
            self._t0_ros = self.get_clock().now()
            self.get_logger().info("Experiment START: follower enabled, t0 set from message arrival time.")

        elif cmd in ("stop", "pause", "halt"):
            self._enabled = False
            self._publish_stop()
            self.get_logger().info("Experiment STOP: follower disabled.")

        elif cmd in ("reset", "restart"):
            self._enabled = False
            self._finished = False
            self._t0_ros = None
            self._publish_stop()
            self.get_logger().info("Experiment RESET: cleared t0 and stopped.")

        else:
            self.get_logger().warn(f"Unknown control command: '{msg.data}'")

    def cb_odom(self, msg: Odometry) -> None:
        """
        Odometry callback
        """
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = _yaw_from_quat_xyzw(q.x, q.y, q.z, q.w)

        self._state = State2D(
            x=float(p.x),
            y=float(p.y),
            yaw=float(yaw),
            stamp=Time.from_msg(msg.header.stamp),
        )

    def cb_timer(self) -> None:
        """
        Timer callback
        """
        # Hard gate: do nothing until START
        if not self._enabled:
            self._publish_stop()
            return

        if self._traj is None or self._state is None or self._t0_ros is None:
            self._publish_stop()
            return

        if self._finished:
            self._publish_stop()
            return

        now_ros = self._state.stamp
        dt = (now_ros - self._t0_ros).nanoseconds * 1e-9

        # Trajectory time query (traj time is absolute in its own frame)
        t_query = float(self._traj.x0[-1] + dt)

        t_end = float(self._traj.xT[-1])
        if t_query >= t_end:
            if bool(self.get_parameter("loop_trajectory").value) and self._traj.duration > 1e-6:
                t0 = float(self._traj.x0[-1])
                t_query = t0 + ((t_query - t0) % float(self._traj.duration))
            else:
                self._finished = True
                self.get_logger().info("Reached end of trajectory. Stopping.")
                self._publish_stop()
                return

        x_d, y_d, yaw_d = self._desired_xy_yaw(t_query)

        v_cmd, w_cmd = self._compute_cmd(
            x=self._state.x,
            y=self._state.y,
            yaw=self._state.yaw,
            x_d=x_d,
            y_d=y_d,
            yaw_d=yaw_d
        )

        self._publish_cmd(v_cmd, w_cmd)

    def _desired_xy_yaw(self, t_query: float) -> Tuple[float, float, float]:
        """
        Use traj.lerp(t) to get desired XY, and estimate yaw by looking a bit ahead
        """
        assert self._traj is not None

        xyt = self._traj.lerp(t_query)          # [x, y, ..., t]
        pos = np.asarray(xyt[:-1], dtype=float) # spatial only

        x_d = float(pos[0])
        y_d = float(pos[1])

        lookahead = float(self.get_parameter("heading_lookahead_s").value)
        t2 = min(t_query + max(lookahead, 1e-3), float(self._traj.xT[-1]))
        pos2 = np.asarray(self._traj.lerp(t2)[:-1], dtype=float)

        dx = float(pos2[0] - pos[0])
        dy = float(pos2[1] - pos[1])
        yaw_d = math.atan2(dy, dx) if (abs(dx) + abs(dy)) > 1e-9 else 0.0

        return x_d, y_d, yaw_d

    def _compute_cmd(self, x: float, y: float, yaw: float, x_d: float, y_d: float, yaw_d: float) -> Tuple[float, float]:
        dx = x_d - x
        dy = y_d - y
        dist = math.hypot(dx, dy)

        desired_heading = math.atan2(dy, dx)
        heading_err = _wrap_to_pi(desired_heading - yaw)

        k_v = float(self.get_parameter("k_v").value)
        k_w = float(self.get_parameter("k_w").value)
        v_max = float(self.get_parameter("v_max").value)
        w_max = float(self.get_parameter("w_max").value)
        slowdown_r = float(self.get_parameter("slowdown_radius_m").value)

        v = k_v * dist
        w = k_w * heading_err

        yaw_err = _wrap_to_pi(yaw_d - yaw)
        w += 0.5 * k_w * yaw_err

        if slowdown_r > 1e-6 and dist < slowdown_r:
            v *= dist / slowdown_r

        if abs(heading_err) > 1.2:
            v *= 0.2

        v = float(np.clip(v, -v_max, v_max))
        w = float(np.clip(w, -w_max, w_max))
        return v, w

    def _publish_cmd(self, v: float, w: float) -> None:
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self._cmd_pub.publish(msg)

    def _publish_stop(self) -> None:
        self._publish_cmd(0.0, 0.0)


def main():
    rclpy.init()
    node = TrajectoryFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
