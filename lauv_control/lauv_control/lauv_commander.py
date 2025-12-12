import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

class LAUVCommander(Node):
    def __init__(self):
        super().__init__('lauv_commander')

        namespace = 'lauv'

        # Thruster publisher
        self.thruster_pub = self.create_publisher(
            Float64, f'/model/{namespace}/joint/thruster_0_joint/cmd_thrust', 10
        )

        # Fin publishers
        self.fin_pubs = []
        for i in range(4):
            fin_pub = self.create_publisher(
                Float64, f'/model/{namespace}/joint/fin_{i}_joint/cmd_pos', 10
            )
            self.fin_pubs.append(fin_pub)

        # Timer to send commands periodically
        self.timer = self.create_timer(0.1, self.publish_commands)  # 10 Hz
        self.time = 0.0

    def publish_commands(self):
        # Example thrust: constant
        thrust_msg = Float64()
        thrust_msg.data = 2.0
        self.thruster_pub.publish(thrust_msg)

        # Example fins: sinusoidal oscillation
        for i, fin_pub in enumerate(self.fin_pubs):
            fin_msg = Float64()
            fin_msg.data = 0.1 * math.sin(self.time + i * math.pi/2)
            fin_pub.publish(fin_msg)

        self.time += 0.1


def main(args=None):
    rclpy.init(args=args)
    node = LAUVCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()