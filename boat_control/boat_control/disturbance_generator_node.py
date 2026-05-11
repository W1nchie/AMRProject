import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3Stamped


class DisturbanceGeneratorNode(Node):
    """
    Publishes sinusoidal current velocity in world frame.

    Output:
      - /disturbance/current : geometry_msgs/Vector3Stamped

    The message contains:
      vector.x = current velocity along world x
      vector.y = current velocity along world y
      vector.z = 0
    """

    def __init__(self):
        super().__init__("disturbance_generator_node")

        self.declare_parameter("topic", "/disturbance/current")

        self.declare_parameter("ax", 0.2)
        self.declare_parameter("ay", 0.1)
        self.declare_parameter("wx", 0.2)
        self.declare_parameter("wy", 0.15)
        self.declare_parameter("phix", 0.0)
        self.declare_parameter("phiy", 0.0)

        topic = self.get_parameter("topic").value

        self.pub = self.create_publisher(Vector3Stamped, topic, 10)
        self.timer = self.create_timer(0.05, self.publish_current)

        self.t0 = self.get_clock().now()

        self.get_logger().info(f"Disturbance generator publishes to {topic}")

    def publish_current(self):
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9

        ax = self.get_parameter("ax").value
        ay = self.get_parameter("ay").value
        wx = self.get_parameter("wx").value
        wy = self.get_parameter("wy").value
        phix = self.get_parameter("phix").value
        phiy = self.get_parameter("phiy").value

        vcx = ax * math.sin(wx * t + phix)
        vcy = ay * math.cos(wy * t + phiy)

        msg = Vector3Stamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "map"
        msg.vector.x = vcx
        msg.vector.y = vcy
        msg.vector.z = 0.0

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DisturbanceGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
