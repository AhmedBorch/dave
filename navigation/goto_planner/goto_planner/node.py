"""ROS 2 node: drive BlueROV2 to a 3D goal using LOS guidance + PD damping.

Control stages
--------------
APPROACH  - LOS guidance: yaw to align with goal then surge toward it (+ heave).
SETTLE    - velocity damping only; wait settle_time seconds before orienting.
ORIENT    - pure yaw to goal_yaw (if given), heave PD to hold depth.
DONE      - velocity damping; holds position indefinitely.

The two-stage split (APPROACH then ORIENT) avoids the ill-conditioned atan2
near the goal and allows a clean final heading for inspection tasks.
"""
import math
from enum import Enum
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float64, String

from .allocation import ThrustLimits, allocate


class Stage(Enum):
    APPROACH = 'approach'
    SETTLE   = 'settle'
    ORIENT   = 'orient'
    DONE     = 'done'


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

        # Estimated state
        self._x: Optional[float] = None
        self._y: Optional[float] = None
        self._z: Optional[float] = None
        self._yaw: Optional[float] = None
        self._vx_body: float = 0.0
        self._vz:      float = 0.0
        self._yaw_rate: float = 0.0

        # FSM
        self._stage: Stage = Stage.APPROACH
        self._settle_deadline: Optional[float] = None  # wall-clock time (s)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(Odometry, self._odom_topic, self._on_odom, sensor_qos)
        self.create_subscription(PointStamped, '~/goal_point', self._on_goal_point, 10)
        self.create_subscription(PoseStamped,  '~/goal_pose',  self._on_goal_pose,  10)

        self._thruster_pubs = [
            self.create_publisher(
                Float64,
                f'/model/{self._namespace}/joint/thruster{i}_joint/cmd_thrust',
                10,
            )
            for i in range(1, 7)
        ]

        # Debug publishers — wire into PlotJuggler / rqt_plot.
        # err:    world-frame goal − pose (x, y, z).
        # cmd:    linear.x=surge, linear.z=heave, angular.z=yaw (N or N·m).
        # stage:  current FSM stage name.
        self._err_pub     = self.create_publisher(Vector3Stamped, '~/debug/err',     10)
        self._cmd_pub     = self.create_publisher(Twist,          '~/debug/cmd',     10)
        self._xy_dist_pub = self.create_publisher(Float64,        '~/debug/xy_dist', 10)
        self._yaw_err_pub = self.create_publisher(Float64,        '~/debug/yaw_err', 10)
        self._stage_pub   = self.create_publisher(String,         '~/debug/stage',   10)

        self._control_timer = self.create_timer(1.0 / self._rate_hz, self._on_tick)

        goal_yaw_str = (f'{math.degrees(self._goal_yaw):.1f}°'
                        if not math.isnan(self._goal_yaw) else 'none')
        self.get_logger().info(
            f"goto_planner ready: odom='{self._odom_topic}' "
            f"goal=({self._goal_x:.2f}, {self._goal_y:.2f}, {self._goal_z:.2f}) "
            f"goal_yaw={goal_yaw_str}  invert_yaw={self._invert_yaw}"
        )

    # ------------------------------------------------------------------ params
    def _declare_params(self) -> None:
        self.declare_parameter('namespace',  'bluerov2')
        self.declare_parameter('odom_topic', '/model/bluerov2/odometry')

        self.declare_parameter('goal_x',   0.0)
        self.declare_parameter('goal_y',   0.0)
        self.declare_parameter('goal_z',  -0.5)
        # goal_yaw: desired final heading in radians (ENU: 0=East, π/2=North).
        # NaN (default) skips the ORIENT stage.
        self.declare_parameter('goal_yaw', math.nan)

        # Tolerances
        self.declare_parameter('xy_tolerance',  0.3)   # m
        self.declare_parameter('z_tolerance',   0.2)   # m
        self.declare_parameter('yaw_tolerance', 0.1)   # rad — ORIENT stage

        # "Stopped" thresholds — must be below these before SETTLE starts
        self.declare_parameter('v_lin_stop', 0.05)     # m/s
        self.declare_parameter('v_ang_stop', 0.05)     # rad/s

        # Seconds to damp velocity in SETTLE before starting ORIENT
        self.declare_parameter('settle_time', 2.0)

        # LOS surge profile
        self.declare_parameter('slow_radius', 1.5)     # m

        # Proportional gains
        self.declare_parameter('k_yaw',   3.0)         # N per rad
        self.declare_parameter('k_heave', 6.0)         # N per m

        # Derivative (damping) gains
        self.declare_parameter('k_d_surge', 5.0)       # N per (m/s)
        self.declare_parameter('k_d_yaw',   1.5)       # N per (rad/s)
        self.declare_parameter('k_d_heave', 3.0)       # N per (m/s)

        # Saturation
        self.declare_parameter('max_thrust',     10.0)
        self.declare_parameter('max_surge_cmd',   5.0)
        self.declare_parameter('max_yaw_cmd',     4.0)
        self.declare_parameter('max_heave_cmd',   6.0)

        self.declare_parameter('control_rate_hz', 20.0)

        # Flip yaw sign if the robot turns the wrong direction.
        # Diagnosis: watch ~/debug/yaw_err when approaching a y≠0 goal.
        # It should shrink toward 0.  If it grows, set invert_yaw: true.
        self.declare_parameter('invert_yaw', False)

    def _load_params(self) -> None:
        gp = self.get_parameter
        self._namespace   = gp('namespace').value
        self._odom_topic  = gp('odom_topic').value
        self._goal_x      = gp('goal_x').value
        self._goal_y      = gp('goal_y').value
        self._goal_z      = gp('goal_z').value
        self._goal_yaw    = gp('goal_yaw').value
        self._xy_tol      = gp('xy_tolerance').value
        self._z_tol       = gp('z_tolerance').value
        self._yaw_tol     = gp('yaw_tolerance').value
        self._v_lin_stop  = gp('v_lin_stop').value
        self._v_ang_stop  = gp('v_ang_stop').value
        self._settle_time = gp('settle_time').value
        self._slow_radius = gp('slow_radius').value
        self._k_yaw       = gp('k_yaw').value
        self._k_heave     = gp('k_heave').value
        self._k_d_surge   = gp('k_d_surge').value
        self._k_d_yaw     = gp('k_d_yaw').value
        self._k_d_heave   = gp('k_d_heave').value
        self._max_surge   = gp('max_surge_cmd').value
        self._max_yaw     = gp('max_yaw_cmd').value
        self._max_heave   = gp('max_heave_cmd').value
        self._limits      = ThrustLimits(max_thrust=gp('max_thrust').value)
        self._rate_hz     = gp('control_rate_hz').value
        self._invert_yaw  = gp('invert_yaw').value

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
        self._reset_fsm()
        self.get_logger().info(
            f'New goal: ({self._goal_x:.2f}, {self._goal_y:.2f}, {self._goal_z:.2f})'
        )

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._goal_x, self._goal_y, self._goal_z = p.x, p.y, p.z
        q = msg.pose.orientation
        self._goal_yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
        self._reset_fsm()
        self.get_logger().info(
            f'New goal: ({self._goal_x:.2f}, {self._goal_y:.2f}, {self._goal_z:.2f}) '
            f'yaw={math.degrees(self._goal_yaw):.1f}°'
        )

    def _reset_fsm(self) -> None:
        self._stage = Stage.APPROACH
        self._settle_deadline = None

    # -------------------------------------------------------------------- tick
    def _on_tick(self) -> None:
        if self._x is None:
            return

        err_x   = self._goal_x - self._x
        err_y   = self._goal_y - self._y
        err_z   = self._goal_z - self._z
        xy_dist = math.hypot(err_x, err_y)

        # Heave runs as PD on depth error in every stage.
        heave_cmd = self._clamp(
            self._k_heave * err_z - self._k_d_heave * self._vz,
            self._max_heave,
        )

        yaw_err   = 0.0
        surge_cmd = 0.0
        yaw_cmd   = 0.0

        if self._stage == Stage.APPROACH:
            surge_cmd, yaw_cmd, yaw_err = self._approach_tick(xy_dist, err_x, err_y)

        elif self._stage == Stage.SETTLE:
            surge_cmd = self._clamp(-self._k_d_surge * self._vx_body, self._max_surge)
            yaw_cmd   = self._clamp(-self._k_d_yaw   * self._yaw_rate, self._max_yaw)
            now = self.get_clock().now().nanoseconds * 1e-9
            if now >= self._settle_deadline:
                if math.isnan(self._goal_yaw):
                    self._transition(Stage.DONE)
                else:
                    self._transition(Stage.ORIENT)

        elif self._stage == Stage.ORIENT:
            surge_cmd = self._clamp(-self._k_d_surge * self._vx_body, self._max_surge)
            yaw_err   = _wrap_pi(self._goal_yaw - self._yaw)
            raw_yaw   = self._k_yaw * yaw_err - self._k_d_yaw * self._yaw_rate
            yaw_cmd   = self._clamp(self._apply_yaw_sign(raw_yaw), self._max_yaw)
            if abs(yaw_err) < self._yaw_tol and abs(self._yaw_rate) < self._v_ang_stop:
                self._transition(Stage.DONE)

        elif self._stage == Stage.DONE:
            surge_cmd = self._clamp(-self._k_d_surge * self._vx_body, self._max_surge)
            yaw_cmd   = self._clamp(-self._k_d_yaw   * self._yaw_rate, self._max_yaw)

        self._publish_debug(err_x, err_y, err_z, xy_dist, yaw_err,
                            surge_cmd, yaw_cmd, heave_cmd)
        self._publish_allocated(surge_cmd, yaw_cmd, heave_cmd)

    # ----------------------------------------------------------------- approach
    def _approach_tick(self, xy_dist: float, err_x: float, err_y: float):
        """LOS guidance. Returns (surge_cmd, yaw_cmd, yaw_err)."""
        if xy_dist < self._xy_tol:
            # Inside bubble: damp velocity, wait until stopped, then SETTLE.
            if self._is_stopped():
                self._transition(Stage.SETTLE)
            surge = self._clamp(-self._k_d_surge * self._vx_body, self._max_surge)
            yaw   = self._clamp(-self._k_d_yaw   * self._yaw_rate, self._max_yaw)
            return surge, yaw, 0.0

        desired_yaw = math.atan2(err_y, err_x)
        yaw_err     = _wrap_pi(desired_yaw - self._yaw)

        raw_yaw   = self._k_yaw * yaw_err - self._k_d_yaw * self._yaw_rate
        yaw_cmd   = self._clamp(self._apply_yaw_sign(raw_yaw), self._max_yaw)

        alignment = max(0.0, math.cos(yaw_err))
        surge_ref = self._max_surge * math.tanh(xy_dist / self._slow_radius) * alignment
        surge_cmd = self._clamp(surge_ref - self._k_d_surge * self._vx_body, self._max_surge)

        return surge_cmd, yaw_cmd, yaw_err

    # --------------------------------------------------------------- helpers
    def _apply_yaw_sign(self, raw: float) -> float:
        return -raw if self._invert_yaw else raw

    def _is_stopped(self) -> bool:
        return (abs(self._vx_body) < self._v_lin_stop
                and abs(self._vz)      < self._v_lin_stop
                and abs(self._yaw_rate) < self._v_ang_stop)

    def _transition(self, next_stage: Stage) -> None:
        self.get_logger().info(f'{self._stage.value} → {next_stage.value}')
        self._stage = next_stage
        if next_stage == Stage.SETTLE:
            now = self.get_clock().now().nanoseconds * 1e-9
            self._settle_deadline = now + self._settle_time

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def _publish_allocated(self, surge: float, yaw: float, heave: float) -> None:
        thrusts = allocate(surge, yaw, heave, self._limits)
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
        self._stage_pub.publish(String(data=self._stage.value))


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
