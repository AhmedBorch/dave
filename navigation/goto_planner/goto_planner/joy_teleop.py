"""ROS 2 node: map joystick axes to BlueROV2 thruster commands.

Default axis mapping (PS4 / Xbox via ros2 joy driver):
  Left  stick Y  (axis 1) → surge   (up   = forward)
  Left  stick X  (axis 0) → yaw     (left = CCW)
  Right stick Y  (axis 4) → heave   (up   = ascend)

Hold the deadman button (L1/LB = button 4) to enable thrust.
Release → all thrusters zero immediately.

Tune invert_* flags if the robot moves opposite to the stick direction.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

from .allocation import ThrustLimits, allocate


class JoyTeleopNode(Node):
    def __init__(self) -> None:
        super().__init__('joy_teleop')

        self._declare_params()
        self._load_params()

        self._thruster_pubs = [
            self.create_publisher(
                Float64,
                f'/model/{self._namespace}/joint/thruster{i}_joint/cmd_thrust',
                10,
            )
            for i in range(1, 7)
        ]

        joy_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Joy, '/joy', self._on_joy, joy_qos)

        # Watchdog: if no joy message arrives within this period, zero thrust.
        self._watchdog = self.create_timer(0.5, self._on_watchdog)
        self._joy_alive = False

        self.get_logger().info(
            f"joy_teleop ready — namespace='{self._namespace}'  "
            f"axes: surge={self._ax_surge}, yaw={self._ax_yaw}, heave={self._ax_heave}  "
            f"deadman button={self._deadman_btn} "
            f"({'disabled' if self._deadman_btn < 0 else 'hold to enable'})"
        )

    # -------------------------------------------------------------- params
    def _declare_params(self) -> None:
        self.declare_parameter('namespace', 'bluerov2')

        # Joystick axis indices
        self.declare_parameter('axis_surge', 1)   # left  stick Y
        self.declare_parameter('axis_yaw',   0)   # left  stick X
        self.declare_parameter('axis_heave', 4)   # right stick Y

        # Sign flips — set true if a channel moves opposite to stick direction.
        # For yaw: the same invert_yaw issue as goto_planner applies here.
        self.declare_parameter('invert_surge', False)
        self.declare_parameter('invert_yaw',   False)
        self.declare_parameter('invert_heave', False)

        # Maximum commands on each axis (N)
        self.declare_parameter('max_surge', 3.0)
        self.declare_parameter('max_yaw',   2.0)
        self.declare_parameter('max_heave', 3.0)

        # Per-thruster hard limit (N)
        self.declare_parameter('max_thrust', 4.0)

        # Deadman button index.  Hold this button to enable thrust.
        # -1 = always enabled (use with caution in pool testing).
        self.declare_parameter('deadman_button', 4)   # L1/LB

    def _load_params(self) -> None:
        gp = self.get_parameter
        self._namespace    = gp('namespace').value
        self._ax_surge     = gp('axis_surge').value
        self._ax_yaw       = gp('axis_yaw').value
        self._ax_heave     = gp('axis_heave').value
        self._inv_surge    = -1.0 if gp('invert_surge').value else 1.0
        self._inv_yaw      = -1.0 if gp('invert_yaw').value   else 1.0
        self._inv_heave    = -1.0 if gp('invert_heave').value  else 1.0
        self._max_surge    = gp('max_surge').value
        self._max_yaw      = gp('max_yaw').value
        self._max_heave    = gp('max_heave').value
        self._limits       = ThrustLimits(max_thrust=gp('max_thrust').value)
        self._deadman_btn  = gp('deadman_button').value

    # ------------------------------------------------------------ callback
    def _on_joy(self, msg: Joy) -> None:
        self._joy_alive = True

        # Deadman check
        if self._deadman_btn >= 0:
            if self._deadman_btn >= len(msg.buttons) or not msg.buttons[self._deadman_btn]:
                self._zero_thrusters()
                return

        surge = self._axis(msg, self._ax_surge) * self._inv_surge * self._max_surge
        yaw   = self._axis(msg, self._ax_yaw)   * self._inv_yaw   * self._max_yaw
        heave = self._axis(msg, self._ax_heave)  * self._inv_heave * self._max_heave

        thrusts = allocate(surge, yaw, heave, self._limits)
        for pub, val in zip(self._thruster_pubs, thrusts):
            pub.publish(Float64(data=float(val)))

    def _on_watchdog(self) -> None:
        if not self._joy_alive:
            self._zero_thrusters()
        self._joy_alive = False

    # --------------------------------------------------------------- helpers
    @staticmethod
    def _axis(msg: Joy, idx: int) -> float:
        """Return axis value or 0.0 if index is out of range."""
        if idx < 0 or idx >= len(msg.axes):
            return 0.0
        return float(msg.axes[idx])

    def _zero_thrusters(self) -> None:
        for pub in self._thruster_pubs:
            pub.publish(Float64(data=0.0))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoyTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._zero_thrusters()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
