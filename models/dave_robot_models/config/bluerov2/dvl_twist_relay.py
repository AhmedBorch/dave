#!/usr/bin/env python3
"""
Relay: dave_interfaces/msg/DVL  →  geometry_msgs/msg/TwistWithCovarianceStamped

The DVLBridge Gazebo plugin publishes the proprietary DVL message.  This node
re-publishes the body-frame velocity as a standard ROS twist so planners and
EKF nodes (e.g. robot_localization) can consume it directly.

Covariance (diagonal 6×6, row-major):
  Linear  (vx,vy,vz): σ = SIGMA_LINEAR m/s  (from bottom-tracking noise in SDF)
  Angular (wx,wy,wz): not measured by DVL → large value (1e6)

Run via robot_config.py — namespace is passed as a ROS parameter.
"""
import rclpy
from rclpy.node import Node
from dave_interfaces.msg import DVL
from geometry_msgs.msg import TwistWithCovarianceStamped

# Match the <noise><stddev> values in bluerov2/model.sdf
SIGMA_BOTTOM     = 0.005   # m/s  (bottom-tracking mode)
SIGMA_WATER_MASS = 0.015   # m/s  (water-mass mode)
SIGMA_ANGULAR    = 1e3     # m/s  (unmeasured — large uncertainty)

# Build a fixed diagonal 6×6 covariance using the more conservative value.
# Indices of the 36-element row-major 6×6 [vx,vy,vz,wx,wy,wz]:
#   diagonal: 0 (vx), 7 (vy), 14 (vz), 21 (wx), 28 (wy), 35 (wz)
_COV = [0.0] * 36
_COV[0]  = SIGMA_BOTTOM ** 2
_COV[7]  = SIGMA_BOTTOM ** 2
_COV[14] = SIGMA_BOTTOM ** 2
_COV[21] = SIGMA_ANGULAR ** 2
_COV[28] = SIGMA_ANGULAR ** 2
_COV[35] = SIGMA_ANGULAR ** 2


class DvlTwistRelay(Node):
    def __init__(self):
        super().__init__('dvl_twist_relay')
        self.declare_parameter('namespace', 'bluerov2')
        ns = self.get_parameter('namespace').get_parameter_value().string_value

        dvl_topic   = f'/model/{ns}/dvl/velocity'
        twist_topic = f'/model/{ns}/dvl/twist'

        self.pub = self.create_publisher(TwistWithCovarianceStamped, twist_topic, 10)
        self.create_subscription(DVL, dvl_topic, self._cb, 10)
        self.get_logger().info(f'{dvl_topic}  →  {twist_topic}')

    def _cb(self, msg: DVL):
        out = TwistWithCovarianceStamped()
        out.header            = msg.header
        out.twist.twist       = msg.velocity.twist
        out.twist.covariance  = _COV
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(DvlTwistRelay())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
