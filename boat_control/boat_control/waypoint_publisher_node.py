import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose


class WaypointPublisherNode(Node):
    """
    Publishes a fixed list of waypoints to /mission/waypoints.
    """

    def __init__(self):
        super().__init__("waypoint_publisher_node")

        self.declare_parameter("waypoints", [5.0, 0.0, 5.0, 5.0, 0.0, 5.0])
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("topic", "/mission/waypoints")

        topic = self.get_parameter("topic").value

        self.pub = self.create_publisher(PoseArray, topic, 10)
        self.timer = self.create_timer(1.0, self.publish_waypoints)

        self.sent_once = False

    def publish_waypoints(self):
        # publish repeatedly for robustness
        raw = list(self.get_parameter("waypoints").value)

        if len(raw) % 2 != 0:
            self.get_logger().error("Waypoints parameter must contain pairs: x1,y1,x2,y2,...")
            return

        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("frame_id").value

        for i in range(0, len(raw), 2):
            pose = Pose()
            pose.position.x = float(raw[i])
            pose.position.y = float(raw[i + 1])
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.pub.publish(msg)

        if not self.sent_once:
            self.get_logger().info(f"Published {len(msg.poses)} waypoints to {self.pub.topic_name}")
            self.sent_once = True


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
