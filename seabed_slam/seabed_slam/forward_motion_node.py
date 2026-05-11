import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class ForwardMotion(Node):
    def __init__(self) -> None:
        super().__init__("forward_motion")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("linear_x", 0.35)
        self.declare_parameter("angular_z", 0.0)
        self.declare_parameter("duration_s", 60.0)
        self.declare_parameter("publish_rate_hz", 10.0)

        self._publisher = self.create_publisher(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            10,
        )
        self._start = self.get_clock().now()
        self._timer = self.create_timer(
            1.0 / float(self.get_parameter("publish_rate_hz").value),
            self._tick,
        )

    def _tick(self) -> None:
        elapsed = (self.get_clock().now() - self._start).nanoseconds * 1e-9
        cmd = Twist()
        if elapsed <= float(self.get_parameter("duration_s").value):
            cmd.linear.x = float(self.get_parameter("linear_x").value)
            cmd.angular.z = float(self.get_parameter("angular_z").value)
        self._publisher.publish(cmd)


def main() -> None:
    rclpy.init()
    node = ForwardMotion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
