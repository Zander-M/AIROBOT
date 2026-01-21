#!/usr/bin/env python3
from __future__ import annotations

import sys
import select
import termios
import tty
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ExperimentKeyboardNode(Node):
    def __init__(self):
        super().__init__("experiment_keyboard")

        # Parameters
        self.declare_parameter("control_topic", "experiment/control")
        self.declare_parameter("robot_namespaces", [])  # e.g. ["robot1", "robot2"]; empty => publish only control_topic
        self.declare_parameter("rate_hz", 30.0)

        self._control_topic = str(self.get_parameter("control_topic").value)
        self._namespaces: List[str] = list(self.get_parameter("robot_namespaces").value)
        self._rate_hz = float(self.get_parameter("rate_hz").value)

        # Publishers
        self._pubs = []
        if self._namespaces:
            for ns in self._namespaces:
                topic = f"/{ns.strip('/')}/{self._control_topic.lstrip('/')}"
                self._pubs.append(self.create_publisher(String, topic, 10))
            self.get_logger().info(f"Publishing control to namespaces: {self._namespaces}")
        else:
            self._pubs.append(self.create_publisher(String, self._control_topic, 10))
            self.get_logger().info(f"Publishing control to topic: {self._control_topic}")

        # Terminal setup
        self._stdin_fd = sys.stdin.fileno()
        self._old_term = termios.tcgetattr(self._stdin_fd)
        tty.setcbreak(self._stdin_fd)

        self.get_logger().info(
            "Experiment keyboard ready:\n"
            "  s: start\n"
            "  x: stop\n"
            "  r: reset\n"
            "  q: quit\n"
        )

        self._timer = self.create_timer(1.0 / max(self._rate_hz, 1e-6), self._on_timer)

    def destroy_node(self):
        # Restore terminal
        try:
            termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._old_term)
        except Exception:
            pass
        super().destroy_node()

    def _publish_cmd(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        for p in self._pubs:
            p.publish(msg)
        self.get_logger().info(f"Sent: {cmd}")

    def _on_timer(self) -> None:
        # Non-blocking key read
        if select.select([sys.stdin], [], [], 0.0)[0]:
            ch = sys.stdin.read(1)

            if ch == "s":
                self._publish_cmd("start")
            elif ch == "x":
                self._publish_cmd("stop")
            elif ch == "r":
                self._publish_cmd("reset")
            elif ch == "q":
                self.get_logger().info("Quit.")
                self.destroy_node()
            else:
                # ignore other keys
                pass


def main():
    rclpy.init()
    node = ExperimentKeyboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
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

