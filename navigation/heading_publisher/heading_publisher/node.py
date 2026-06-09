"""Publish a noisy yaw heading derived from ground-truth odometry.

Subscribes to a nav_msgs/Odometry topic, extracts the yaw, adds
Gaussian noise, and publishes the result as a std_msgs/Float64 (radians).

This decouples the heading source from the ESKF — swap the subscriber
topic to a real compass/magnetometer bridge when moving to hardware.
"""
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64


class HeadingPublisher(Node):
    def __init__(self) -> None:
        super().__init__('heading_publisher')

        self.declare_parameter('input_odom_topic', '/model/bluerov2/odometry')
        self.declare_parameter('output_heading_topic', '/heading')
        self.declare_parameter('noise_std', 0.05)    # radians (1-sigma)
        self.declare_parameter('publish_rate_hz', 10.0)

        in_topic  = self.get_parameter('input_odom_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_heading_topic').get_parameter_value().string_value
        self._noise_std = self.get_parameter('noise_std').get_parameter_value().double_value
        rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self._rng = np.random.default_rng()

        # Latest true yaw from odometry; published at a fixed rate by the timer.
        self._latest_yaw: float | None = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._pub = self.create_publisher(Float64, out_topic, 10)
        self._sub = self.create_subscription(Odometry, in_topic, self._callback, sensor_qos)
        self._timer = self.create_timer(1.0 / rate_hz, self._publish_heading)

        self.get_logger().info(
            f"heading_publisher ready: '{in_topic}' → '{out_topic}'  "
            f"noise_std={self._noise_std:.3f} rad  rate={rate_hz:.1f} Hz"
        )

    def _callback(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self._latest_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _publish_heading(self) -> None:
        if self._latest_yaw is None:
            return  # no odometry received yet
        noisy_yaw = self._latest_yaw + self._rng.normal(0.0, self._noise_std)
        self._pub.publish(Float64(data=noisy_yaw))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HeadingPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
