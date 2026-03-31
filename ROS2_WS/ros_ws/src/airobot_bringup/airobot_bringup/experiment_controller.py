"""
Experiment Controller with numpy trajectories (x, y, yaw, t)
"""
from __future__ import annotations

import math
import sys
import select
import termios
import tty
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
import json

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from airobot_msgs.srv import SetPose2D


def _wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class PendingReset:
    epoch: int
    pending: List[Tuple[str, Optional[Any]]]  # (ns, future) future can be None if skipped


class ExperimentControllerSimple(Node):
    """
    Simple experiment controller:

    - 'r': teleport all robots to trajectory start via SetPose2D, then publish "reset <epoch>"
    - 's': publish "start <epoch> <t0_ns>"
    - 'x': publish "stop <epoch>"
    - 'q': quit
    """

    def __init__(self):
        super().__init__("experiment_controller_simple")

        # ---------------- Params ----------------
        self.declare_parameter("control_topic", "experiment/control")
        self.declare_parameter("trajectory_path", "")
        self.declare_parameter("num_robots", 0)  # 0 => use metadata num_trajectory

        self.declare_parameter("setpose_service", "set_pose")
        self.declare_parameter("start_delay_s", 0.25)
        self.declare_parameter("rate_hz", 30.0)

        self._control_topic = str(self.get_parameter("control_topic").value)
        self._trajectory_path = str(self.get_parameter("trajectory_path").value)
        self._num_robots_param = int(self.get_parameter("num_robots").value)

        self._setpose_service = str(self.get_parameter("setpose_service").value)
        self._start_delay_s = float(self.get_parameter("start_delay_s").value)
        self._rate_hz = float(self.get_parameter("rate_hz").value)

        # ---------------- Load trajectories ----------------
        # Dict: ns -> np.ndarray with shape (T,4) where each row is [x,y,yaw,t]
        self._traj_by_ns: Dict[str, np.ndarray] = self._load_trajectories()
        self._namespaces: List[str] = sorted(list(self._traj_by_ns.keys()))
        self.get_logger().info(f"Namespaces: {self._namespaces}")

        # ---------------- Publisher(s) ----------------
        self._control_pubs: List[Any] = []
        for ns in self._namespaces:
            topic = f"/{ns.strip('/')}/{self._control_topic.lstrip('/')}"
            self._control_pubs.append(self.create_publisher(String, topic, 10))
        self.get_logger().info(f"Control pubs -> {self._namespaces} / {self._control_topic}")

        # ---------------- SetPose clients ----------------
        self._setpose_clients: Dict[str, Any] = {}
        for ns in self._namespaces:
            srv = f"/{ns.strip('/')}/{self._setpose_service.lstrip('/')}"
            cli = self.create_client(SetPose2D, srv)
            self._setpose_clients[ns] = cli

        # ---------------- Runtime state ----------------
        self._epoch: int = 0
        self._pending_reset: Optional[PendingReset] = None

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

    def _load_trajectories(self) -> Dict[str, np.ndarray]:
        """
        Load from directory containing:
          - metadata.json with {"num_trajectory": K, ...}
          - robot0.npy, robot1.npy, ...
        Each npy is shape (T,4) row=[x,y,yaw,t]
        """
        trajectory_path = self._trajectory_path.strip()
        if not trajectory_path:
            raise ValueError("trajectory_path is empty (set -p trajectory_path:=...)")

        trajectory_path = Path(trajectory_path)
        if not trajectory_path.exists():
            raise FileNotFoundError(f"trajectory directory not found: {trajectory_path}")

        metadata_path = trajectory_path / "metadata.json"
        with metadata_path.open("r") as f:
            metadata = json.load(f)

        num_traj_meta = int(metadata.get("num_trajectory", 0))
        if num_traj_meta <= 0:
            raise ValueError("metadata.json missing/invalid num_trajectory")

        num_robots = self._num_robots_param if self._num_robots_param > 0 else num_traj_meta
        num_robots = min(num_robots, num_traj_meta)

        out: Dict[str, np.ndarray] = {}
        for i in range(num_robots):
            ns = f"robot{i}"
            fname = trajectory_path / f"{ns}.npy"
            traj = np.load(fname)
            if traj.ndim != 2 or traj.shape[1] != 4 or traj.shape[0] < 1:
                raise ValueError(f"{fname} must have shape (T,4) [x,y,yaw,t]; got {traj.shape}")
            out[ns] = traj

        self.get_logger().info(f"Loaded {len(out)} trajectories from {trajectory_path}")
        return out

    # ---------------- Core helpers ----------------

    def _publish_control(self, text: str) -> None:
        msg = String()
        msg.data = text
        for p in self._control_pubs:
            p.publish(msg)
        self.get_logger().info(f"Sent: {text}")

    def _progress_pending_reset(self) -> None:
        if self._pending_reset is None:
            return

        still: List[Tuple[str, Optional[Any]]] = []
        had_error = False

        for ns, fut in self._pending_reset.pending:
            if fut is None:
                had_error = True
                continue
            if fut.done():
                try:
                    _ = fut.result()
                except Exception as e:
                    had_error = True
                    self.get_logger().error(f"[{ns}] set_pose failed: {e}")
            else:
                still.append((ns, fut))

        self._pending_reset.pending = still

        if not self._pending_reset.pending:
            epoch = self._pending_reset.epoch
            self._pending_reset = None
            self._publish_control(f"reset {epoch}")
            if had_error:
                self.get_logger().warning("Reset published, but some set_pose calls failed/skipped.")

    def _tick(self) -> None:
        # progress any async reset
        self._progress_pending_reset()

        # keyboard
        if select.select([sys.stdin], [], [], 0.0)[0]:
            ch = sys.stdin.read(1)

            if ch == "r":
                self._epoch += 1
                self.get_logger().info(f"Reset requested -> epoch={self._epoch}")
                self._set_pose_request(pose_idx=0)

            elif ch == "s":
                self._send_start()

            elif ch == "x":
                self._publish_control(f"stop {self._epoch}")

            elif ch == "q":
                self.get_logger().info("Quit.")
                rclpy.shutdown()
                return

    # ---------------- Missing pieces ----------------

    def _send_start(self) -> None:
        """
        Send: start <epoch> <t0_ns>
        """
        now_ns = int(self.get_clock().now().nanoseconds)
        t0_ns = now_ns + int(self._start_delay_s * 1e9)
        self._publish_control(f"start {self._epoch} {t0_ns}")

    def _set_pose_request(self, pose_idx: int) -> None:
        """
        Teleport all robots to pose_idx of their trajectory (x,y,yaw,t).
        Publishes reset only after all service calls return (handled by _progress_pending_reset()).
        """
        if self._pending_reset is not None:
            self.get_logger().warning("Reset already in progress; ignoring.")
            return

        pending: List[Tuple[str, Optional[Any]]] = []

        for ns in self._namespaces:
            traj = self._traj_by_ns[ns]
            T = traj.shape[0]
            i = int(np.clip(pose_idx, 0, T - 1))

            x = float(traj[i, 0])
            y = float(traj[i, 1])
            yaw = _wrap_to_pi(float(traj[i, 2]))
            # t = traj[i, 3]  # not needed for SetPose2D

            cli = self._setpose_clients[ns]
            if not cli.service_is_ready():
                self.get_logger().warning(f"[{ns}] service not ready: {cli.srv_name}; skipping set_pose")
                pending.append((ns, None))
                continue

            req = SetPose2D.Request()
            req.x = x
            req.y = y
            req.yaw = yaw

            fut = cli.call_async(req)
            pending.append((ns, fut))
            self.get_logger().info(f"[{ns}] set_pose -> x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

        self._pending_reset = PendingReset(epoch=self._epoch, pending=pending)


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

