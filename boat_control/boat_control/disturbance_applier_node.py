import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3Stamped
from nav_msgs.msg import Odometry


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class DisturbanceApplierNode(Node):
    """
    Applies a simple current-like disturbance to the controller command before
    sending it to Gazebo. This is an approximation for testing controller
    robustness in the proxy simulator.
    """

    def __init__(self):
        super().__init__("disturbance_applier_node")

        self.declare_parameter("input_cmd_topic", "/cmd_vel_controller")
        self.declare_parameter("output_cmd_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("disturbance_topic", "/disturbance/current")
        self.declare_parameter("lateral_yaw_gain", 0.8)
        self.declare_parameter("max_extra_linear", 0.4)
        self.declare_parameter("max_extra_angular", 0.5)

        input_cmd_topic = self.get_parameter("input_cmd_topic").value
        output_cmd_topic = self.get_parameter("output_cmd_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        disturbance_topic = self.get_parameter("disturbance_topic").value

        self.cmd_sub = self.create_subscription(
            Twist,
            input_cmd_topic,
            self.cmd_callback,
            20,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            20,
        )
        self.disturbance_sub = self.create_subscription(
            Vector3Stamped,
            disturbance_topic,
            self.disturbance_callback,
            20,
        )
        self.cmd_pub = self.create_publisher(Twist, output_cmd_topic, 20)

        self.psi = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0

        self.get_logger().info(f"Reading controller cmd: {input_cmd_topic}")
        self.get_logger().info(f"Publishing disturbed cmd: {output_cmd_topic}")
        self.get_logger().info(f"Using odom: {odom_topic}")
        self.get_logger().info(f"Using disturbance: {disturbance_topic}")

    def odom_callback(self, msg: Odometry):
        self.psi = yaw_from_quaternion(msg.pose.pose.orientation)

    def disturbance_callback(self, msg: Vector3Stamped):
        self.current_vx = msg.vector.x
        self.current_vy = msg.vector.y

    def cmd_callback(self, msg: Twist):
        cos_psi = math.cos(self.psi)
        sin_psi = math.sin(self.psi)

        # Project world-frame disturbance into the boat forward and lateral axes.
        current_forward = cos_psi * self.current_vx + sin_psi * self.current_vy
        current_lateral = -sin_psi * self.current_vx + cos_psi * self.current_vy

        max_extra_linear = self.get_parameter("max_extra_linear").value
        max_extra_angular = self.get_parameter("max_extra_angular").value
        lateral_yaw_gain = self.get_parameter("lateral_yaw_gain").value

        extra_linear = max(-max_extra_linear, min(max_extra_linear, current_forward))
        extra_angular = lateral_yaw_gain * current_lateral
        extra_angular = max(-max_extra_angular, min(max_extra_angular, extra_angular))

        disturbed = Twist()
        disturbed.linear.x = msg.linear.x + extra_linear
        disturbed.angular.z = msg.angular.z + extra_angular

        self.cmd_pub.publish(disturbed)


def main(args=None):
    rclpy.init(args=args)
    node = DisturbanceApplierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
