from __future__ import annotations

import math
import pickle
import sys
import select
import termios
import tty
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String

from airobot_msgs.srv import SetPose2D
from airobot_common import STTrajectory


def _yaw_from_dxdy(dx: float, dy: float) -> float:
    if abs(dx) + abs(dy) < 1e-12:
        return 0.0
    return math.atan2(dy, dx)


@dataclass
class PendingCall:
    ns: str
    future: Any


class ExperimentControllerSimple(Node):
    """
    Simple experiment controller:

    - Loads trajectories from a pickle at startup.
    - 'r': teleport all robots to trajectory start via SetPose2D, then publish "reset <epoch>"
    - 's': publish "start <epoch> <t0_ns>"
    - 'x': publish "stop <epoch>"
    - 'q': quit
    """

    def __init__(self):
        super().__init__("experiment_controller_simple")

        # ---------------- Params ----------------
        self.declare_parameter("control_topic", "experiment/control")
        self.declare_parameter("trajectories_pkl", "")
        self.declare_parameter("robot_prefix", "robot")
        self.declare_parameter("num_robots", 0)  # 0 => infer from trajectories if possible

        self.declare_parameter("setpose_service", "set_pose")
        self.declare_parameter("start_delay_s", 0.25)
        self.declare_parameter("rate_hz", 30.0)

        self._control_topic = str(self.get_parameter("control_topic").value)
        self._traj_pkl = str(self.get_parameter("trajectories_pkl").value)
        self._robot_prefix = str(self.get_parameter("robot_prefix").value)
        self._num_robots_param = int(self.get_parameter("num_robots").value)

        self._setpose_service = str(self.get_parameter("setpose_service").value)
        self._start_delay_s = float(self.get_parameter("start_delay_s").value)
        self._rate_hz = float(self.get_parameter("rate_hz").value)

        # ---------------- Load trajectories ----------------
        self._traj_by_ns: Dict[str, STTrajectory] = self._load_trajectories()

        # Determine robot namespaces
        if self._num_robots_param > 0:
            self._namespaces = [f"{self._robot_prefix}{i}" for i in range(self._num_robots_param)]
        else:
            # infer from trajectories
            if self._traj_by_ns:
                self._namespaces = sorted(self._traj_by_ns.keys())
            else:
                self._namespaces = [f"{self._robot_prefix}0"]

        self.get_logger().info(f"Namespaces: {self._namespaces}")

        # ---------------- Publisher(s): publish to each namespaced control topic ----------------
        self._control_pubs: List[Any] = []
        for ns in self._namespaces:
            topic = f"/{ns.strip('/')}/{self._control_topic.lstrip('/')}"
            self._control_pubs.append(self.create_publisher(String, topic, 10))
        self.get_logger().info(f"Control pubs -> {self._namespaces} / {self._control_topic}")

        # ---------------- SetPose clients ----------------
        self._setpose_clients: List[Tuple[str, Any]] = []
        for ns in self._namespaces:
            srv = f"/{ns.strip('/')}/{self._setpose_service.lstrip('/')}"
            cli = self.create_client(SetPose2D, srv)
            self._setpose_clients.append((ns, cli))

        # ---------------- Runtime state ----------------
        self._epoch: int = 0
        self._pending_calls: List[PendingCall] = []
        self._reset_in_progress: bool = False

        # ---------------- Terminal ----------------
        self._stdin_fd = sys.stdin.fileno()
        self._old_term = termios.tcgetattr(self._stdin_fd)
        tty.setcbreak(self._stdin_fd)

        self.get_logger().info(
            "Experiment controller ready:\n"
            "  r: reset (teleport all -> publish reset <epoch>)\n"
            "  s: start (publish start <epoch> <t0_ns>)\n"
            "  x: stop  (publish stop <epoch>)\n"
            "  q: quit\n"
        )

        self._timer = self.create_timer(1.0 / max(self._rate_hz, 1e-6), self._tick)

    def destroy_node(self):
        try:
            termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._old_term)
        except Exception:
            pass
        super().destroy_node()

    # ---------------- Trajectory loading ----------------

    def _load_trajectories(self) -> None:
        path_str = self._traj_pkl.strip()
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

        trajs_by_ns_dict = {}

        for robot_id, traj in enumerate(st_trajs[:1]): # FIXME: san check, load one trajectory only
            trajs_by_ns_dict[f"robot{robot_id}"] = traj
        self.get_logger().info(f"Loaded {len(st_trajs)} trajectories from {path_str}.")
        return trajs_by_ns_dict


    # ---------------- Helpers ----------------

    def _publish_control(self, text: str) -> None:
        msg = String()
        msg.data = text
        for p in self._control_pubs:
            p.publish(msg)
        self.get_logger().info(f"Sent: {text}")

    def _start_pose_from_traj(self, traj: STTrajectory) -> Tuple[float, float, float]:
        """
        Uses first point as (x,y), and yaw from first segment if possible.
        Assumes traj points are (x,y,t) or (x,y,...) where first two dims are xy.
        """
        p0 = traj.points[0]
        x = float(p0[0])
        y = float(p0[1])

        yaw = 0.0
        if traj.size >= 2:
            p1 = traj.points[1]
            yaw = _yaw_from_dxdy(float(p1[0] - p0[0]), float(p1[1] - p0[1]))
        return x, y, yaw

    # ---------------- Reset logic ----------------

    def _begin_reset(self) -> None:
        # stop first (good habit)
        self._publish_control(f"stop {self._epoch}")

        # ensure services are ready
        for ns, cli in self._setpose_clients:
            if not cli.service_is_ready():
                self.get_logger().warning(f"Waiting for service: /{ns}/{self._setpose_service}")
                # Don’t start reset yet; keep trying in tick()
                self._reset_in_progress = True
                self._pending_calls = []
                return

        # fire async calls
        self._pending_calls = []
        for ns, cli in self._setpose_clients:
            traj = self._traj_by_ns.get(ns, None)
            if traj is None:
                # fallback
                x, y, yaw = 0.0, 0.0, 0.0
                self.get_logger().warning(f"No trajectory for {ns}; using (0,0,0).")
            else:
                x, y, yaw = self._start_pose_from_traj(traj)

            req = SetPose2D.Request()
            req.x = float(x)
            req.y = float(y)
            req.yaw = float(yaw)
            fut = cli.call_async(req)
            self._pending_calls.append(PendingCall(ns=ns, future=fut))

        self._reset_in_progress = True
        self.get_logger().info(f"Reset started for epoch={self._epoch} ({len(self._pending_calls)} calls).")

    def _finish_reset_if_done(self) -> None:
        if not self._reset_in_progress:
            return

        # If we never launched calls due to service not ready, try again
        if not self._pending_calls:
            self._begin_reset()
            return

        all_done = True
        any_failed = False
        for c in self._pending_calls:
            fut = c.future
            if not fut.done():
                all_done = False
                continue
            if fut.exception() is not None:
                any_failed = True
                self.get_logger().error(f"SetPose2D failed for {c.ns}: {fut.exception()}")

        if not all_done:
            return

        self._pending_calls = []
        self._reset_in_progress = False

        if any_failed:
            self.get_logger().warning("Reset completed with failures; still publishing reset sync message.")

        self._publish_control(f"reset {self._epoch}")

    # ---------------- Main tick ----------------

    def _send_start(self) -> None:
        now = self.get_clock().now()
        t0_ns = now.nanoseconds + int(max(self._start_delay_s, 0.0) * 1e9)
        self._publish_control(f"start {self._epoch} {t0_ns}")

    def _tick(self) -> None:
        # progress reset if in flight
        self._finish_reset_if_done()

        # keyboard
        if select.select([sys.stdin], [], [], 0.0)[0]:
            ch = sys.stdin.read(1)

            if ch == "r":
                self._epoch += 1
                self.get_logger().info(f"Reset requested -> epoch={self._epoch}")
                self._begin_reset()

            elif ch == "s":
                self._send_start()

            elif ch == "x":
                self._publish_control(f"stop {self._epoch}")

            elif ch == "q":
                self.get_logger().info("Quit.")
                rclpy.shutdown()
                return


def main():
    rclpy.init()
    node = ExperimentControllerSimple()
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

