from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

from seabed_slam.sonar_mapper_node import _read_xyz


class ReferenceMapRecorder(Node):
    def __init__(self) -> None:
        super().__init__("reference_map_recorder")
        self.declare_parameter("map_points_topic", "/seabed/map_points")
        self.declare_parameter(
            "output_csv",
            "/home/w1nchie/ros2_ws/src/seabed_slam/data/missouri_i64/recorded_reference_map.csv",
        )
        self.declare_parameter("min_points", 5000)
        self.declare_parameter("save_period_s", 5.0)

        self._output_csv = Path(str(self.get_parameter("output_csv").value))
        self._last_save_time = self.get_clock().now()

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._subscriber = self.create_subscription(
            PointCloud2,
            str(self.get_parameter("map_points_topic").value),
            self._on_map,
            qos,
        )

    def _on_map(self, message: PointCloud2) -> None:
        points = np.asarray(list(_read_xyz(message)), dtype=np.float64)
        if len(points) < int(self.get_parameter("min_points").value):
            return

        now = self.get_clock().now()
        period_ns = int(float(self.get_parameter("save_period_s").value) * 1e9)
        if (now - self._last_save_time).nanoseconds < period_ns:
            return
        self._last_save_time = now

        self._output_csv.parent.mkdir(parents=True, exist_ok=True)
        np.savetxt(
            self._output_csv,
            points,
            delimiter=",",
            header="x,y,z",
            comments="",
        )
        self.get_logger().info(f"Saved reference map: {len(points)} points -> {self._output_csv}")


def main() -> None:
    rclpy.init()
    node = ReferenceMapRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
