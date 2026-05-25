"""ROS 2 node: drive BlueROV2 to a 3D goal using LOS guidance + PD damping."""
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float64

from .allocation import ThrustLimits, allocate


def _yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class GotoPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__('goto_planner')

        self._declare_params()
        self._load_params()

        # Position / orientation
        self._x: Optional[float] = None
        self._y: Optional[float] = None
        self._z: Optional[float] = None
        self._yaw: Optional[float] = None

        # Body-frame velocities from odometry twist
        self._vx_body: float = 0.0
        self._vz: float = 0.0
        self._yaw_rate: float = 0.0

        # Hysteresis (state latching)
        self._is_arrived = False

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(Odometry, self._odom_topic, self._on_odom, sensor_qos)
        self.create_subscription(PointStamped, '~/goal_point', self._on_goal_point, 10)
        self.create_subscription(PoseStamped, '~/goal_pose', self._on_goal_pose, 10)

        self._thruster_pubs = [
            self.create_publisher(
                Float64,
                f'/model/{self._namespace}/joint/thruster{i}_joint/cmd_thrust',
                10,
            )
            for i in range(1, 7)
        ]

        # Debug publishers — wire these into PlotJuggler / rqt_plot for tuning.
        # err: world-frame goal-minus-current (x, y, z).
        # cmd: Twist with linear.x=surge, linear.z=heave, angular.z=yaw torque (N or N·m).
        self._err_pub      = self.create_publisher(Vector3Stamped, '~/debug/err', 10)
        self._cmd_pub      = self.create_publisher(Twist,          '~/debug/cmd', 10)
        self._xy_dist_pub  = self.create_publisher(Float64,        '~/debug/xy_dist', 10)
        self._yaw_err_pub  = self.create_publisher(Float64,        '~/debug/yaw_err', 10)

        self._control_timer = self.create_timer(1.0 / self._rate_hz, self._on_tick)
        self._reached_logged = False

        self.get_logger().info(
            f"goto_planner ready (LOS): odom='{self._odom_topic}' "
            f"goal=({self._goal_x:.2f}, {self._goal_y:.2f}, {self._goal_z:.2f})"
        )

    # ------------------------------------------------------------------ params
    def _declare_params(self) -> None:
        self.declare_parameter('namespace', 'bluerov2')
        self.declare_parameter('odom_topic', '/model/bluerov2/odometry')

        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_z', -0.5)

        # Goal tolerances
        self.declare_parameter('xy_tolerance', 0.3)   # m
        self.declare_parameter('z_tolerance',  0.2)   # m

        # Velocity thresholds for declaring "stopped at goal" (m/s, rad/s).
        # While inside the position tolerance but moving above these, the
        # controller keeps damping velocity instead of going idle.
        self.declare_parameter('v_lin_stop', 0.05)
        self.declare_parameter('v_ang_stop', 0.05)

        # LOS surge profile
        # surge thrust = max_surge_cmd * tanh(dist / slow_radius) * cos(yaw_err)
        # slow_radius: distance (m) at which we start significantly reducing speed.
        # A smaller value = tighter braking zone.
        self.declare_parameter('slow_radius', 1.5)    # m

        # Proportional gains
        self.declare_parameter('k_yaw',   3.0)   # N per rad
        self.declare_parameter('k_heave', 6.0)   # N per m

        # Derivative (velocity damping) gains — PD on every axis
        self.declare_parameter('k_d_surge', 5.0)  # N per (m/s)
        self.declare_parameter('k_d_yaw',   1.5)  # N per (rad/s)
        self.declare_parameter('k_d_heave', 3.0)  # N per (m/s)

        # Saturation
        self.declare_parameter('max_thrust',    10.0)
        self.declare_parameter('max_surge_cmd',  5.0)  # cruise thrust (N)
        self.declare_parameter('max_yaw_cmd',    4.0)
        self.declare_parameter('max_heave_cmd',  6.0)

        self.declare_parameter('control_rate_hz', 20.0)

    def _load_params(self) -> None:
        gp = self.get_parameter
        self._namespace     = gp('namespace').get_parameter_value().string_value
        self._odom_topic    = gp('odom_topic').get_parameter_value().string_value
        self._goal_x        = gp('goal_x').get_parameter_value().double_value
        self._goal_y        = gp('goal_y').get_parameter_value().double_value
        self._goal_z        = gp('goal_z').get_parameter_value().double_value
        self._xy_tol        = gp('xy_tolerance').get_parameter_value().double_value
        self._z_tol         = gp('z_tolerance').get_parameter_value().double_value
        self._slow_radius   = gp('slow_radius').get_parameter_value().double_value
        self._k_yaw         = gp('k_yaw').get_parameter_value().double_value
        self._k_heave       = gp('k_heave').get_parameter_value().double_value
        self._k_d_surge     = gp('k_d_surge').get_parameter_value().double_value
        self._k_d_yaw       = gp('k_d_yaw').get_parameter_value().double_value
        self._k_d_heave     = gp('k_d_heave').get_parameter_value().double_value
        self._max_surge     = gp('max_surge_cmd').get_parameter_value().double_value
        self._max_yaw       = gp('max_yaw_cmd').get_parameter_value().double_value
        self._max_heave     = gp('max_heave_cmd').get_parameter_value().double_value
        self._limits        = ThrustLimits(max_thrust=gp('max_thrust').get_parameter_value().double_value)
        self._rate_hz       = gp('control_rate_hz').get_parameter_value().double_value
        self._v_lin_stop    = gp('v_lin_stop').get_parameter_value().double_value
        self._v_ang_stop    = gp('v_ang_stop').get_parameter_value().double_value

    # ----------------------------------------------------------------- callbacks
    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._x, self._y, self._z = p.x, p.y, p.z
        self._yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)

        t = msg.twist.twist
        self._vx_body  = t.linear.x
        self._vz       = t.linear.z
        self._yaw_rate = t.angular.z

    def _on_goal_point(self, msg: PointStamped) -> None:
        self._goal_x, self._goal_y, self._goal_z = msg.point.x, msg.point.y, msg.point.z
        self._reached_logged = False
        self._is_arrived = False
        self.get_logger().info(
            f'New goal: ({self._goal_x:.2f}, {self._goal_y:.2f}, {self._goal_z:.2f})'
        )

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._goal_x, self._goal_y, self._goal_z = p.x, p.y, p.z
        self._reached_logged = False
        self.get_logger().info(
            f'New goal: ({self._goal_x:.2f}, {self._goal_y:.2f}, {self._goal_z:.2f})'
        )

    # -------------------------------------------------------------------- tick
    def _on_tick(self) -> None:
        if self._x is None:
            return

        err_x   = self._goal_x - self._x
        err_y   = self._goal_y - self._y
        err_z   = self._goal_z - self._z
        xy_dist = math.hypot(err_x, err_y)

        # Heave always runs as PD on the world-frame z error.
        heave_cmd = self._clamp(
            self._k_heave * err_z - self._k_d_heave * self._vz,
            self._max_heave,
        )

        # Default debug values (overwritten when LOS branch runs).
        yaw_err = 0.0

        # if xy_dist < self._xy_tol:
        #     # --- Station-keeping ---
        #     # Inside the tolerance bubble, atan2(err_y, err_x) is ill-conditioned
        #     # (tiny numerator and denominator → oscillating desired heading), so
        #     # we suppress LOS guidance entirely and just damp body-frame motion.
        #     # This is what kills the previous "drift past goal → spin around →
        #     # surge backwards" cycle: thrust is never zeroed while we still
        #     # carry velocity, so residual momentum is bled off in place.
        #     surge_cmd = self._clamp(-self._k_d_surge * self._vx_body, self._max_surge)
        #     yaw_cmd   = self._clamp(-self._k_d_yaw   * self._yaw_rate, self._max_yaw)

        #     stopped = (
        #         abs(err_z) < self._z_tol
        #         and abs(self._vx_body) < self._v_lin_stop
        #         and abs(self._vz)      < self._v_lin_stop
        #         and abs(self._yaw_rate) < self._v_ang_stop
        #     )
        #     if stopped and not self._reached_logged:
        #         self.get_logger().info(
        #             f'Goal reached (xy={xy_dist:.2f}m, z_err={err_z:.2f}m, '
        #             f'|vx|={abs(self._vx_body):.3f}m/s).'
        #         )
        #         self._reached_logged = True
        # --- Hysteresis State Machine ---
        if not self._is_arrived and xy_dist < self._xy_tol:
            self._is_arrived = True
            self.get_logger().info('Goal reached! Latching to braking mode.')
        elif self._is_arrived and xy_dist > (self._xy_tol * 2.0):
            # Only snap back to LOS if we drifted far outside the tolerance
            self._is_arrived = False
            self.get_logger().info('Drifted significantly! Re-engaging LOS.')

        if self._is_arrived:
            # --- Station-keeping (Braking) ---
            surge_cmd = self._clamp(-self._k_d_surge * self._vx_body, self._max_surge)
            yaw_cmd   = self._clamp(-self._k_d_yaw   * self._yaw_rate, self._max_yaw)

            stopped = (
                abs(err_z) < self._z_tol
                and abs(self._vx_body) < self._v_lin_stop
                and abs(self._vz)      < self._v_lin_stop
                and abs(self._yaw_rate) < self._v_ang_stop
            )
            if stopped and not self._reached_logged:
                self.get_logger().info(
                    f'Goal stabilized (xy={xy_dist:.2f}m, z_err={err_z:.2f}m).'
                )
                self._reached_logged = True
        else:
            # --- LOS horizontal guidance ---
            # Surge:  max_surge * tanh(dist / slow_radius) * cos(yaw_err)
            #   tanh(dist/R): cruise when far, ramps to 0 at goal.
            #   cos(yaw_err): smoothly reduces surge when off-heading; clamped
            #                 to 0 beyond ±90° instead of a hard align-first mode.
            self._reached_logged = False

            desired_yaw = math.atan2(err_y, err_x)
            yaw_err     = _wrap_pi(desired_yaw - self._yaw)

            yaw_cmd = self._clamp(
                self._k_yaw * yaw_err - self._k_d_yaw * self._yaw_rate,
                self._max_yaw,
            )

            alignment = max(0.0, math.cos(yaw_err))
            surge_ref = self._max_surge * math.tanh(xy_dist / self._slow_radius) * alignment
            surge_cmd = self._clamp(
                surge_ref - self._k_d_surge * self._vx_body, self._max_surge
            )

        self._publish_debug(err_x, err_y, err_z, xy_dist, yaw_err,
                            surge_cmd, yaw_cmd, heave_cmd)
        self._publish_allocated(surge_cmd, yaw_cmd, heave_cmd)

    # ----------------------------------------------------------------- helpers
    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        if value > limit:
            return limit
        if value < -limit:
            return -limit
        return value

    def _publish_allocated(self, surge: float, yaw: float, heave: float) -> None:
        thrusts = allocate(surge, yaw, heave, self._limits)
        self._publish(thrusts)

    def _publish(self, thrusts) -> None:
        for pub, val in zip(self._thruster_pubs, thrusts):
            pub.publish(Float64(data=float(val)))

    def _publish_debug(self, err_x: float, err_y: float, err_z: float,
                       xy_dist: float, yaw_err: float,
                       surge_cmd: float, yaw_cmd: float, heave_cmd: float) -> None:
        stamp = self.get_clock().now().to_msg()

        err = Vector3Stamped()
        err.header.stamp = stamp
        err.header.frame_id = 'map'
        err.vector.x, err.vector.y, err.vector.z = err_x, err_y, err_z
        self._err_pub.publish(err)

        cmd = Twist()
        cmd.linear.x  = surge_cmd
        cmd.linear.z  = heave_cmd
        cmd.angular.z = yaw_cmd
        self._cmd_pub.publish(cmd)

        self._xy_dist_pub.publish(Float64(data=xy_dist))
        self._yaw_err_pub.publish(Float64(data=yaw_err))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GotoPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for pub in node._thruster_pubs:
            pub.publish(Float64(data=0.0))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
