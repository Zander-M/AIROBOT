"""
Trajectory Follower based on numpy (x, y, yaw, t) waypoints.

- Loads one file: <trajectory_path>/<robot_ns>.npy
  where each row is [x, y, yaw, t] and t is in seconds (float).
- Subscribes to: /<robot_ns>/<control_topic>  (String)
    reset <epoch>
    start <epoch> <t0_ns>
    stop  <epoch>
- Subscribes to: /<robot_ns>/<odom_topic> (nav_msgs/Odometry)
- Publishes:     /<robot_ns>/<cmd_vel_topic> (geometry_msgs/Twist)

Controller: simple P controller in SE(2) using waypoint interpolation by time.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def _wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _yaw_from_quat_xyzw(x: float, y: float, z: float, w: float) -> float:
    # yaw (Z axis)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _interp_angle(a0: float, a1: float, u: float) -> float:
    # shortest-path interpolation on circle
    da = _wrap_to_pi(a1 - a0)
    return _wrap_to_pi(a0 + u * da)


class NumpyTrajectoryFollower(Node):
    def __init__(self):
        super().__init__("numpy_trajectory_follower")

        # ---------------- Params ----------------
        self.declare_parameter("trajectory_path", "")
        self.declare_parameter("robot_ns", "")  # optional override; default = current namespace without leading '/'

        self.declare_parameter("control_topic", "experiment/control")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")

        self.declare_parameter("rate_hz", 50.0)

        # gains
        self.declare_parameter("k_rho", 1.2)     # linear gain on forward error
        self.declare_parameter("k_alpha", 3.0)   # angular gain on heading-to-goal
        self.declare_parameter("k_yaw", 2.0)     # extra yaw tracking gain

        # limits
        self.declare_parameter("v_max", 0.6)
        self.declare_parameter("w_max", 2.5)
        self.declare_parameter("stop_pos_tol", 0.05)   # m
        self.declare_parameter("stop_yaw_tol", 0.10)   # rad

        # behavior
        self.declare_parameter("hold_at_end", True)  # if False, stop when t exceeds last waypoint

        self._traj_dir = str(self.get_parameter("trajectory_path").value).strip()
        robot_ns_param = str(self.get_parameter("robot_ns").value).strip()

        self._control_topic = str(self.get_parameter("control_topic").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)

        self._rate_hz = float(self.get_parameter("rate_hz").value)

        self._k_rho = float(self.get_parameter("k_rho").value)
        self._k_alpha = float(self.get_parameter("k_alpha").value)
        self._k_yaw = float(self.get_parameter("k_yaw").value)

        self._v_max = float(self.get_parameter("v_max").value)
        self._w_max = float(self.get_parameter("w_max").value)
        self._pos_tol = float(self.get_parameter("stop_pos_tol").value)
        self._yaw_tol = float(self.get_parameter("stop_yaw_tol").value)

        self._hold_at_end = bool(self.get_parameter("hold_at_end").value)

        # ---------------- Namespace resolution ----------------
        # Node namespace is like "/robot0" typically
        ns = self.get_namespace().strip("/")
        self._robot_ns = robot_ns_param if robot_ns_param else ns
        if not self._robot_ns:
            raise ValueError(
                "robot_ns is empty. Either run node in a namespace (/robot0) or set -p robot_ns:=robot0"
            )

        # ---------------- Load trajectory ----------------
        self._traj = self._load_trajectory(self._traj_dir, self._robot_ns)
        self._t = self._traj[:, 3].astype(np.float64)
        if not np.all(np.diff(self._t) >= -1e-9):
            self.get_logger().warn("Trajectory time column is not monotonic nondecreasing. Sorting by t.")
            order = np.argsort(self._t)
            self._traj = self._traj[order]
            self._t = self._traj[:, 3].astype(np.float64)

        # ---------------- ROS I/O ----------------
        # topics are resolved within namespace automatically if relative names used;
        # but we’ll build explicit absolute to be consistent with your controller.
        control_topic_abs = f"/{self._robot_ns.strip('/')}/{self._control_topic.lstrip('/')}"
        odom_topic_abs = f"/{self._robot_ns.strip('/')}/{self._odom_topic.lstrip('/')}"
        cmd_vel_topic_abs = f"/{self._robot_ns.strip('/')}/{self._cmd_vel_topic.lstrip('/')}"

        self._sub_ctrl = self.create_subscription(String, control_topic_abs, self._on_control, 10)
        self._sub_odom = self.create_subscription(Odometry, odom_topic_abs, self._on_odom, 10)
        self._pub_cmd = self.create_publisher(Twist, cmd_vel_topic_abs, 10)

        # ---------------- Runtime state ----------------
        self._epoch: int = 0
        self._running: bool = False
        self._t0_ns: Optional[int] = None

        self._have_odom: bool = False
        self._x: float = 0.0
        self._y: float = 0.0
        self._yaw: float = 0.0

        self._timer = self.create_timer(1.0 / max(self._rate_hz, 1e-6), self._tick)

        self.get_logger().info(
            f"Follower ready for {self._robot_ns}:\n"
            f"  control: {control_topic_abs}\n"
            f"  odom:    {odom_topic_abs}\n"
            f"  cmd_vel: {cmd_vel_topic_abs}\n"
            f"  traj:    {self._traj.shape} (x,y,yaw,t)\n"
        )

    def _load_trajectory(self, traj_dir: str, robot_ns: str) -> np.ndarray:
        if not traj_dir:
            raise ValueError("trajectory_path is empty")
        p = Path(traj_dir)
        if not p.exists():
            raise FileNotFoundError(f"trajectory_path not found: {p}")

        f = p / f"{robot_ns}.npy"
        if not f.exists():
            raise FileNotFoundError(f"trajectory file not found: {f}")

        traj = np.load(f)
        if traj.ndim != 2 or traj.shape[1] != 4 or traj.shape[0] < 1:
            raise ValueError(f"{f} must have shape (T,4) [x,y,yaw,t]; got {traj.shape}")
        return traj.astype(np.float64)

    # ---------------- ROS callbacks ----------------

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._x = float(p.x)
        self._y = float(p.y)
        self._yaw = _yaw_from_quat_xyzw(float(q.x), float(q.y), float(q.z), float(q.w))
        self._have_odom = True

    def _on_control(self, msg: String) -> None:
        parts = msg.data.strip().split()
        if not parts:
            return

        cmd = parts[0].lower()

        if cmd == "reset":
            # reset <epoch>
            if len(parts) >= 2:
                try:
                    self._epoch = int(parts[1])
                except Exception:
                    pass
            self._running = False
            self._t0_ns = None
            self._publish_stop()
            self.get_logger().info(f"[epoch={self._epoch}] reset received -> stopped")

        elif cmd == "stop":
            # stop <epoch>
            if len(parts) >= 2:
                try:
                    self._epoch = int(parts[1])
                except Exception:
                    pass
            self._running = False
            self._t0_ns = None
            self._publish_stop()
            self.get_logger().info(f"[epoch={self._epoch}] stop received -> stopped")

        elif cmd == "start":
            # start <epoch> <t0_ns>
            if len(parts) < 3:
                self.get_logger().warn("start command missing args: expected 'start <epoch> <t0_ns>'")
                return
            try:
                epoch = int(parts[1])
                t0_ns = int(parts[2])
            except Exception:
                self.get_logger().warn(f"bad start args: {parts}")
                return

            self._epoch = epoch
            self._t0_ns = t0_ns
            self._running = True
            self.get_logger().info(f"[epoch={self._epoch}] start received -> t0_ns={self._t0_ns}")

    # ---------------- Core tracking ----------------

    def _tick(self) -> None:
        if not self._running:
            return
        if not self._have_odom:
            return
        if self._t0_ns is None:
            return

        now_ns = int(self.get_clock().now().nanoseconds)
        tau = (now_ns - self._t0_ns) * 1e-9  # seconds since start

        if tau < 0.0:
            # waiting for scheduled t0
            self._publish_stop()
            return

        # time beyond end?
        t_end = float(self._t[-1])
        if tau >= t_end:
            if not self._hold_at_end:
                self._publish_stop()
                self._running = False
                self.get_logger().info(f"[epoch={self._epoch}] reached end of traj -> stopping (hold_at_end=False)")
                return
            # else hold last pose
            xd, yd, yawd = float(self._traj[-1, 0]), float(self._traj[-1, 1]), float(self._traj[-1, 2])
        else:
            xd, yd, yawd = self._desired_pose_at_time(tau)

        v, w = self._compute_cmd(xd, yd, yawd)
        self._publish_cmd(v, w)

        # optional: stop if close enough near end
        if tau >= t_end:
            dx = xd - self._x
            dy = yd - self._y
            pos_err = math.hypot(dx, dy)
            yaw_err = abs(_wrap_to_pi(yawd - self._yaw))
            if pos_err < self._pos_tol and yaw_err < self._yaw_tol:
                self._publish_stop()
                self._running = False
                self.get_logger().info(f"[epoch={self._epoch}] converged at end -> stopped")

    def _desired_pose_at_time(self, tau: float) -> Tuple[float, float, float]:
        """
        Interpolate between surrounding waypoints by time.
        traj row is [x,y,yaw,t]
        """
        # find first index i with t[i] >= tau
        i = int(np.searchsorted(self._t, tau, side="left"))
        if i <= 0:
            return float(self._traj[0, 0]), float(self._traj[0, 1]), float(self._traj[0, 2])
        if i >= len(self._t):
            return float(self._traj[-1, 0]), float(self._traj[-1, 1]), float(self._traj[-1, 2])

        t0 = float(self._t[i - 1])
        t1 = float(self._t[i])
        if t1 <= t0 + 1e-12:
            u = 0.0
        else:
            u = (tau - t0) / (t1 - t0)

        x0, y0, yaw0 = float(self._traj[i - 1, 0]), float(self._traj[i - 1, 1]), float(self._traj[i - 1, 2])
        x1, y1, yaw1 = float(self._traj[i, 0]), float(self._traj[i, 1]), float(self._traj[i, 2])

        x = (1.0 - u) * x0 + u * x1
        y = (1.0 - u) * y0 + u * y1
        yaw = _interp_angle(yaw0, yaw1, u)
        return x, y, yaw

    def _compute_cmd(self, xd: float, yd: float, yawd: float) -> Tuple[float, float]:
        """
        Simple SE(2) tracking:
          - compute goal in robot frame
          - v proportional to forward error
          - w proportional to heading-to-goal and yaw tracking
        """
        dx = xd - self._x
        dy = yd - self._y

        # transform error into robot frame
        c = math.cos(self._yaw)
        s = math.sin(self._yaw)
        ex = c * dx + s * dy          # forward
        ey = -s * dx + c * dy         # left

        # heading to the desired position
        alpha = math.atan2(ey, ex)    # desired heading change to point toward target
        yaw_err = _wrap_to_pi(yawd - self._yaw)

        # linear velocity: only push forward (optional: allow reverse by removing max(0,...))
        v = self._k_rho * ex
        # angular velocity: turn toward target + track yaw
        w = self._k_alpha * alpha + self._k_yaw * yaw_err

        # clamp
        v = float(max(-self._v_max, min(self._v_max, v)))
        w = float(max(-self._w_max, min(self._w_max, w)))

        return v, w

    def _publish_cmd(self, v: float, w: float) -> None:
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self._pub_cmd.publish(msg)

    def _publish_stop(self) -> None:
        self._publish_cmd(0.0, 0.0)


def main():
    rclpy.init()
    node = NumpyTrajectoryFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass


if __name__ == "__main__":
    main()
