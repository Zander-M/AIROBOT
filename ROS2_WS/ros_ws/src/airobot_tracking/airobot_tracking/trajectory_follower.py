"""
Spatio-temporal trajectory follower ROS2 node

Behavior (state machine):
- Subscribes to experiment/control for reset/start/stop.
- Does NOT call set_pose. The experiment controller / sim should handle pose reset.
- Publishes "ready <epoch> <robot_ns>" when it is at the trajectory start pose (ARMED).
- Starts tracking at shared t0 from "start <epoch> <t0_ns>" and uses dt = now - t0.
- Optional safety: require odom to be close to trajectory start pose before arming/starting.

Control protocol (String):
  reset <epoch>
  start <epoch> <t0_ns>
  stop  <epoch>

Backward compatible:
  "reset", "start", "stop" with no args (epoch will be managed locally, start will use now).
"""

from __future__ import annotations

import math
import pickle
from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from airobot_common import STTrajectory


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


class Mode(Enum):
    IDLE = auto()       # stopped; waiting for reset/start
    ARMING = auto()     # after reset(epoch): wait until pose matches start; then publish ready
    ARMED = auto()      # pose matches start; waiting for start(t0)
    WAIT_T0 = auto()    # start received; waiting until now >= t0
    RUNNING = auto()    # tracking
    FINISHED = auto()   # reached end; stopped


class TrajectoryFollowerNode(Node):
    """
    Trajectory Follower Node for tracking STTrajectory
    """

    def __init__(self):
        super().__init__("trajectory_follower")

        # --- Parameters ---
        # Topics
        self.declare_parameter("trajectory_path", "")
        self.declare_parameter("robot_id", 0)
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("experiment_control_topic", "experiment/control")
        self.declare_parameter("experiment_ready_topic", "experiment/ready")

        # Identity for ready messages
        self.declare_parameter("robot_ns", "")  # e.g. "robot_0"; if empty infer from node namespace
        # Ready publishing behavior (when ARMED/WAIT_T0)
        self.declare_parameter("ready_repeat_hz", 2.0)  # 0=once per epoch

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
        self.declare_parameter("heading_lookahead_s", 0.01)

        # Start gating / sanity checks
        self.declare_parameter("require_reset_match", True)
        self.declare_parameter("reset_match_xy_tol_m", 0.15)
        self.declare_parameter("reset_match_yaw_tol_rad", 0.6)

        # --- Internal state ---
        self._traj: Optional[STTrajectory] = None
        self._state: Optional[State2D] = None

        self._robot_id: int = int(self.get_parameter("robot_id").value)
        self._epoch: int = -1
        self._t0_ros: Optional[Time] = None
        self._mode: Mode = Mode.IDLE

        # Ready bookkeeping
        self._ready_sent_epoch: int = -999999
        self._last_ready_pub = self.get_clock().now()

        # Robot namespace label
        ns_param = str(self.get_parameter("robot_ns").value).strip()
        self._robot_ns = ns_param if ns_param else self.get_namespace().strip("/")
        if not self._robot_ns:
            self._robot_ns = f"robot_{self._robot_id}"

        # --- ROS interface ---
        odom_topic = str(self.get_parameter("odom_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        experiment_control_topic = str(self.get_parameter("experiment_control_topic").value)
        experiment_ready_topic = str(self.get_parameter("experiment_ready_topic").value)

        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._odom_sub = self.create_subscription(Odometry, odom_topic, self.cb_odom, 10)
        self._ctrl_sub = self.create_subscription(String, experiment_control_topic, self.cb_control, 10)
        self._ready_pub = self.create_publisher(String, experiment_ready_topic, 10)

        rate_hz = float(self.get_parameter("rate_hz").value)
        self._timer = self.create_timer(1.0 / max(rate_hz, 1e-6), self.cb_timer)

        self.load_trajectory()

        self.get_logger().info(
            f"[{self._robot_ns}] TrajectoryFollower up. "
            f"odom='{odom_topic}', cmd_vel='{cmd_vel_topic}', "
            f"control='{experiment_control_topic}', ready='{experiment_ready_topic}'"
        )

    # --- IO / Load ---

    def load_trajectory(self) -> None:
        path_str = str(self.get_parameter("trajectory_path").value).strip()
        if not path_str:
            self.get_logger().warning("No trajectory_path set. Provide -p trajectory_path:=/path/to/traj.pkl")
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

        st_trajs = [STTrajectory.from_dict(traj_d) for traj_d in obj]

        if len(st_trajs) <= self._robot_id:
            self.get_logger().error("Not enough trajectories to parse. Returning...")
            return

        self._traj = st_trajs[self._robot_id]

        if self._traj.size <= 0:
            self.get_logger().error("Trajectory is empty.")
            self._traj = None
            return

        if self._traj.dim < 2:
            self.get_logger().error(f"Trajectory dim seems wrong: traj.dim={self._traj.dim} (includes time)")
            self._traj = None
            return

        self.get_logger().info(
            f"[{self._robot_ns}] Loaded trajectory: segments={self._traj.size}, "
            f"dim_with_time={self._traj.dim}, t0={self._traj.x0[-1]:.3f}, "
            f"tT={self._traj.xT[-1]:.3f}, duration={self._traj.duration:.3f}s"
        )

    # --- Control protocol ---

    def cb_control(self, msg: String) -> None:
        """
        /experiment/control protocol:
          reset <epoch>
          start <epoch> <t0_ns>
          stop  <epoch>

        Backward compatible:
          "reset", "start", "stop" with no args.
        """
        text = msg.data.strip()
        if not text:
            return

        parts = text.split()
        cmd = parts[0].lower()

        epoch: Optional[int] = None
        if len(parts) >= 2:
            try:
                epoch = int(parts[1])
            except Exception:
                epoch = None

        if cmd in ("reset", "restart"):
            # Adopt epoch if provided, otherwise bump locally.
            if epoch is not None:
                self._epoch = epoch
            else:
                self._epoch += 1

            self._t0_ros = None
            self._mode = Mode.ARMING
            self._publish_stop()
            self.get_logger().info(f"[{self._robot_ns}] reset -> ARMING (epoch={self._epoch})")
            return

        if cmd in ("stop", "pause", "halt"):
            if epoch is not None and epoch != self._epoch:
                return  # stale stop
            self._t0_ros = None
            self._mode = Mode.IDLE
            self._publish_stop()
            self.get_logger().info(f"[{self._robot_ns}] stop -> IDLE (epoch={self._epoch})")
            return

        if cmd in ("start", "run", "go"):
            # If epoch is given, adopt it (controller authoritative).
            if epoch is not None:
                self._epoch = epoch

            if self._traj is None:
                self.get_logger().warning("No trajectory set! Ignoring start.")
                return

            t0_ns: Optional[int] = None
            if len(parts) >= 3:
                try:
                    t0_ns = int(parts[2])
                except Exception:
                    t0_ns = None

            # Store t0 (synced) if provided; otherwise use local now.
            self._t0_ros = Time(nanoseconds=t0_ns) if t0_ns is not None else self.get_clock().now()

            # We can accept start even if not ARMED yet; we’ll gate in the timer via pose_ok.
            self._mode = Mode.WAIT_T0
            self._publish_stop()
            self.get_logger().info(
                f"[{self._robot_ns}] start -> WAIT_T0 (epoch={self._epoch}, t0_ns={t0_ns})"
            )
            return

    def cb_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = _yaw_from_quat_xyzw(q.x, q.y, q.z, q.w)

        self._state = State2D(
            x=float(p.x),
            y=float(p.y),
            yaw=float(yaw),
            stamp=Time.from_msg(msg.header.stamp),
        )

    # --- Timer / State machine ---

    def cb_timer(self) -> None:
        # Always fail-safe stop if we can’t compute
        if self._traj is None or self._state is None:
            self._publish_stop()
            return

        def pose_ok() -> bool:
            if not bool(self.get_parameter("require_reset_match").value):
                return True
            x0, y0, yaw0 = self._initial_pose_from_traj()
            return self._is_pose_close_to_start(self._state, x0, y0, yaw0)

        now = self.get_clock().now()

        # IDLE / FINISHED: keep stopped
        if self._mode in (Mode.IDLE, Mode.FINISHED):
            self._publish_stop()
            return

        # ARMING: wait for pose match; then publish ready and move to ARMED
        if self._mode == Mode.ARMING:
            if not pose_ok():
                self._publish_stop()
                return
            # became ready
            self._publish_ready_if_needed(now, force_once=True)
            self._mode = Mode.ARMED
            self._publish_stop()
            self.get_logger().info(f"[{self._robot_ns}] ARMING -> ARMED (epoch={self._epoch})")
            return

        # ARMED: keep stopped; periodically re-publish ready if configured
        if self._mode == Mode.ARMED:
            if not pose_ok():
                # If we drifted away, go back to arming (e.g., sim not reset properly)
                self._mode = Mode.ARMING
                self._publish_stop()
                self.get_logger().warning(f"[{self._robot_ns}] ARMED -> ARMING (pose mismatch)")
                return

            self._publish_ready_if_needed(now, force_once=False)
            self._publish_stop()
            return

        # WAIT_T0: require pose_ok, publish ready, then wait until now>=t0
        if self._mode == Mode.WAIT_T0:
            if not pose_ok():
                self._publish_stop()
                return

            self._publish_ready_if_needed(now, force_once=False)

            if self._t0_ros is None:
                # Shouldn’t happen, but fail safe: start now
                self._t0_ros = now

            if (now - self._t0_ros).nanoseconds < 0:
                self._publish_stop()
                return

            self._mode = Mode.RUNNING
            self.get_logger().info(f"[{self._robot_ns}] WAIT_T0 -> RUNNING (epoch={self._epoch})")
            # fallthrough into RUNNING in the same tick

        # RUNNING: track trajectory
        if self._mode == Mode.RUNNING:
            assert self._t0_ros is not None

            dt = (now - self._t0_ros).nanoseconds * 1e-9
            if dt < 0.0:
                # Don’t “fix” t0 here; just wait
                self._publish_stop()
                return

            t_query = float(self._traj.x0[-1] + dt)

            t_end = float(self._traj.xT[-1])
            if t_query >= t_end:
                if bool(self.get_parameter("loop_trajectory").value) and self._traj.duration > 1e-6:
                    t0 = float(self._traj.x0[-1])
                    t_query = t0 + ((t_query - t0) % float(self._traj.duration))
                else:
                    self._mode = Mode.FINISHED
                    self._publish_stop()
                    self.get_logger().info(f"[{self._robot_ns}] RUNNING -> FINISHED")
                    return

            x_d, y_d, yaw_d = self._desired_xy_yaw(t_query)
            v_cmd, w_cmd = self._compute_cmd(
                x=self._state.x,
                y=self._state.y,
                yaw=self._state.yaw,
                x_d=x_d,
                y_d=y_d,
                yaw_d=yaw_d,
            )
            self._publish_cmd(v_cmd, w_cmd)
            return

    # --- Ready publishing ---

    def _publish_ready(self) -> None:
        msg = String()
        msg.data = f"ready {self._epoch} {self._robot_ns}"
        self._ready_pub.publish(msg)

    def _publish_ready_if_needed(self, now: Time, force_once: bool) -> None:
        """
        Publish ready:
          - always at least once per epoch when force_once=True (transition into ARMED)
          - otherwise, once per epoch + periodic refresh if ready_repeat_hz>0
        """
        # Always publish at least once per epoch
        if self._ready_sent_epoch != self._epoch:
            self._publish_ready()
            self._ready_sent_epoch = self._epoch
            self._last_ready_pub = now
            return

        if force_once:
            return

        rep_hz = float(self.get_parameter("ready_repeat_hz").value)
        if rep_hz <= 0:
            return

        period_ns = int(1e9 / max(rep_hz, 1e-6))
        if (now - self._last_ready_pub).nanoseconds >= period_ns:
            self._publish_ready()
            self._last_ready_pub = now

    # --- Helpers: tracking math ---

    def _desired_xy_yaw(self, t_query: float) -> Tuple[float, float, float]:
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

    def _compute_cmd(
        self,
        x: float,
        y: float,
        yaw: float,
        x_d: float,
        y_d: float,
        yaw_d: float,
    ) -> Tuple[float, float]:
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

    def _initial_pose_from_traj(self) -> Tuple[float, float, float]:
        assert self._traj is not None
        t0 = float(self._traj.x0[-1])
        return self._desired_xy_yaw(t0)

    def _is_pose_close_to_start(self, s: State2D, x0: float, y0: float, yaw0: float) -> bool:
        xy_tol = float(self.get_parameter("reset_match_xy_tol_m").value)
        yaw_tol = float(self.get_parameter("reset_match_yaw_tol_rad").value)

        dxy = math.hypot(float(s.x - x0), float(s.y - y0))
        dyaw = abs(_wrap_to_pi(float(s.yaw - yaw0)))
        return (dxy <= xy_tol) and (dyaw <= yaw_tol)


def main():
    rclpy.init()
    node = TrajectoryFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

