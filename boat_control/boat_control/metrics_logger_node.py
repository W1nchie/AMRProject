import csv
from datetime import datetime
from pathlib import Path
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Twist, Vector3Stamped
from nav_msgs.msg import Odometry


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MetricsLoggerNode(Node):
    def __init__(self):
        super().__init__("metrics_logger_node")

        self.declare_parameter("output_dir", "metrics_runs")
        self.declare_parameter("run_name", "")
        self.declare_parameter("flush_interval_sec", 1.0)

        base_dir = Path(self.get_parameter("output_dir").value).expanduser().resolve()
        run_name = str(self.get_parameter("run_name").value).strip()
        if not run_name:
            run_name = datetime.now().strftime("run_metrics")
        self.run_dir = base_dir / run_name
        self.run_dir.mkdir(parents=True, exist_ok=True)

        self.odom_file = self._open_writer(
            "odom.csv",
            ["t", "x", "y", "yaw", "u", "w"],
        )
        self.cmd_file = self._open_writer(
            "cmd_vel.csv",
            ["t", "linear_x", "angular_z"],
        )
        self.cmd_controller_file = self._open_writer(
            "cmd_vel_controller.csv",
            ["t", "linear_x", "angular_z"],
        )
        self.disturbance_file = self._open_writer(
            "disturbance.csv",
            ["t", "vx", "vy"],
        )
        self.waypoints_file = self._open_writer(
            "waypoints.csv",
            ["t", "mission_id", "point_idx", "x", "y"],
        )

        self.mission_id = 0
        self.write_count = 0

        self.create_subscription(Odometry, "/odom", self.odom_callback, 50)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 50)
        self.create_subscription(
            Twist,
            "/cmd_vel_controller",
            self.cmd_controller_callback,
            50,
        )
        self.create_subscription(
            Vector3Stamped,
            "/disturbance/current",
            self.disturbance_callback,
            50,
        )
        self.create_subscription(PoseArray, "/mission/waypoints", self.waypoints_callback, 10)
        self.flush_timer = self.create_timer(
            float(self.get_parameter("flush_interval_sec").value),
            self.flush_all,
        )
        rclpy.get_default_context().on_shutdown(self.flush_all)

        self.get_logger().info(f"Metrics CSV logging to: {self.run_dir}")

    def _open_writer(self, filename: str, header):
        path = self.run_dir / filename
        file_obj = path.open("w", newline="", encoding="utf-8")
        writer = csv.writer(file_obj)
        writer.writerow(header)
        file_obj.flush()
        return {"file": file_obj, "writer": writer}

    def _write_row(self, writer_info, row):
        writer_info["writer"].writerow(row)
        self.write_count += 1
        if self.write_count % 20 == 0:
            writer_info["file"].flush()

    def flush_all(self):
        for writer_info in [
            self.odom_file,
            self.cmd_file,
            self.cmd_controller_file,
            self.disturbance_file,
            self.waypoints_file,
        ]:
            try:
                writer_info["file"].flush()
            except ValueError:
                pass

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def odom_callback(self, msg: Odometry):
        self._write_row(
            self.odom_file,
            [
                stamp_to_sec(msg.header.stamp),
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                yaw_from_quaternion(msg.pose.pose.orientation),
                msg.twist.twist.linear.x,
                msg.twist.twist.angular.z,
            ],
        )

    def cmd_callback(self, msg: Twist):
        self._write_row(
            self.cmd_file,
            [
                self._now_sec(),
                msg.linear.x,
                msg.angular.z,
            ],
        )

    def cmd_controller_callback(self, msg: Twist):
        self._write_row(
            self.cmd_controller_file,
            [
                self._now_sec(),
                msg.linear.x,
                msg.angular.z,
            ],
        )

    def disturbance_callback(self, msg: Vector3Stamped):
        self._write_row(
            self.disturbance_file,
            [
                stamp_to_sec(msg.header.stamp),
                msg.vector.x,
                msg.vector.y,
            ],
        )

    def waypoints_callback(self, msg: PoseArray):
        self.mission_id += 1
        t = stamp_to_sec(msg.header.stamp)
        for point_idx, pose in enumerate(msg.poses):
            self._write_row(
                self.waypoints_file,
                [
                    t,
                    self.mission_id,
                    point_idx,
                    pose.position.x,
                    pose.position.y,
                ],
            )

    def destroy_node(self):
        self.flush_all()
        for writer_info in [
            self.odom_file,
            self.cmd_file,
            self.cmd_controller_file,
            self.disturbance_file,
            self.waypoints_file,
        ]:
            writer_info["file"].flush()
            writer_info["file"].close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
