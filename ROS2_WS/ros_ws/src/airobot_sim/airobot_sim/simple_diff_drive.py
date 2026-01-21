"""
Simple Differential Drive assuming perfect system
"""

from __future__ import annotations

import math 
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry

from tf2_ros import TransformBroadcaster

##### Util Functions #####

def yaw2quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2)) # x, y, z, w

@dataclass
class State2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


class DiffDriveNode(Node):
    """
    DiffDriveNode for simulating a perfect differential drive system
    """

    def __init__(self):
        super().__init__("simple_diff_drive_sim")

        # --- Parameters ---
        # Integration / publish rate
        self.declare_parameter("rate_hz", 50.0)
        self.rate_hz = float(self.get_parameter("rate_hz").value)

        # Frames and topics
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        # Simple limits
        self.declare_parameter("max_v", 1.0)  # m/s
        self.declare_parameter("max_w", 2.0)  # rad/s
        self.max_v = float(self.get_parameter("max_v").value)
        self.max_w = float(self.get_parameter("max_w").value)

        # Whether to publish TF
        self.declare_parameter("publish_tf", True)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        # --- State ---
        self.state = State2D()
        self.v_cmd = 0.0
        self.w_cmd = 0.0

        self.last_update  = self.get_clock().now()

        # --- ROS I/O ---
        self.sub = self.create_subscription(Twist, "cmd_vel", self.cb_cmd_vel, 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)

        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        period = 1.0 / max(self.rate_hz, 1e-6)
        self.timer = self.create_timer(period, self.cb_timer) # create timer for state updates
 
        self.get_logger().info(
            f"airobot simple_diff_dirve started: cmd_vel -> odom at {self.rate_hz} Hz. "
            f"(frames: {self.odom_frame} -> {self.base_frame})"
        )
        
    
    def cb_cmd_vel(self, msg: Twist):
        """
        cmd_vel callback
        
        :param self: Description
        :param msg: Description
        :type msg: Twist
        """
        v = float(msg.linear.x)
        w = float(msg.angular.z)
        self.v_cmd = min(max(v, -self.max_v), self.max_v)
        self.w_cmd = min(max(w, -self.max_w), self.max_w)

    def cb_timer(self):
        """
        Timer callback
        Update robot state
        """
        now = self.get_clock().now()
        dt = (now - self.last_update).nanoseconds * 1e-9
        if dt <= 0:
            return
        self.last_update = now

        # --- Diff-drive Planar Integration ---
        # x_dot = v cos(yaw) y_dot = v sin(yaw), yaw_dot = w
        x, y, yaw = self.state.x, self.state.y, self.state.yaw
        v, w = self.v_cmd, self.w_cmd

        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += w * dt

        # Keep yaw between [-pi, pi]
        yaw = (yaw + math.pi) % (2 * math.pi) - math.pi

        self.state = State2D(x, y, yaw)
        self.publish_odom(now, v, w)

    def publish_odom(self, stamp, v:float, w:float):
        """
        Publish odometry. We assume the system is perfect so the odom matches system state exactly.
        """
        stamp_msg = stamp.to_msg()
        odom = Odometry()
        odom.header.stamp = stamp_msg
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.state.x
        odom.pose.pose.position.y = self.state.y
        odom.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw2quat(self.state.yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Twist in the base frame
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp_msg
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame

            t.transform.translation.x = self.state.x
            t.transform.translation.y = self.state.y
            t.transform.translation.z = 0.0

            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(t)
        
def main():
    rclpy.init()
    node = DiffDriveNode()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
