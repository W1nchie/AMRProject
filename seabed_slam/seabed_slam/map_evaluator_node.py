import math
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

from seabed_slam.bathymetry_reference import (
    DEFAULT_META_PATH,
    DEFAULT_SOURCE_CSV,
    BathymetryReference,
)
from seabed_slam.sonar_mapper_node import _read_xyz


Point3 = Tuple[float, float, float]


class MapEvaluator(Node):
    def __init__(self) -> None:
        super().__init__("seabed_map_evaluator")

        self.declare_parameter(
            "source_csv",
            DEFAULT_SOURCE_CSV,
        )
        self.declare_parameter(
            "meta_path",
            DEFAULT_META_PATH,
        )
        self.declare_parameter("map_points_topic", "/seabed/map_points")
        self.declare_parameter("mesh_scale", 0.5)
        self.declare_parameter("coverage_cell_m", 0.5)
        self.declare_parameter("min_points", 200)
        self.declare_parameter("report_period_s", 2.0)

        self._mesh_scale = float(self.get_parameter("mesh_scale").value)
        self._coverage_cell_m = float(self.get_parameter("coverage_cell_m").value)
        self._min_points = int(self.get_parameter("min_points").value)
        self._last_report_time = self.get_clock().now()

        self._reference = BathymetryReference(
            source_csv=str(self.get_parameter("source_csv").value),
            meta_path=str(self.get_parameter("meta_path").value),
            mesh_scale=self._mesh_scale,
            coverage_cell_m=self._coverage_cell_m,
        )
        self._gt_cell_count = self._reference.gt_cell_count
        self.get_logger().info(
            f"Loaded GT bathymetry: {len(self._reference.z)} points, "
            f"{self._reference.gt_cell_count} coverage cells"
        )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._subscriber = self.create_subscription(
            PointCloud2,
            self.get_parameter("map_points_topic").value,
            self._on_map,
            qos,
        )

    def _gt_z(self, points: np.ndarray) -> np.ndarray:
        return self._reference.height(points[:, :2])

    def _coverage_cells(self, points: np.ndarray) -> int:
        cells = set()
        for x, y in points[:, :2]:
            cells.add((
                math.floor(float(x) / self._coverage_cell_m),
                math.floor(float(y) / self._coverage_cell_m),
            ))
        return len(cells)

    def _on_map(self, message: PointCloud2) -> None:
        points: List[Point3] = list(_read_xyz(message))
        if len(points) < self._min_points:
            return

        now = self.get_clock().now()
        period_ns = int(float(self.get_parameter("report_period_s").value) * 1e9)
        if (now - self._last_report_time).nanoseconds < period_ns:
            return
        self._last_report_time = now

        observed = np.asarray(points, dtype=np.float64)
        gt_z = self._gt_z(observed)
        valid = np.isfinite(gt_z)
        if not np.any(valid):
            self.get_logger().warn("No map points overlap the GT bathymetry")
            return

        errors = observed[valid, 2] - gt_z[valid]
        rmse = float(np.sqrt(np.mean(errors ** 2)))
        mae = float(np.mean(np.abs(errors)))
        bias = float(np.mean(errors))
        coverage_cells = self._coverage_cells(observed[valid])
        coverage_percent = 100.0 * coverage_cells / max(1, self._gt_cell_count)

        self.get_logger().info(
            "seabed_map_metrics "
            f"points={len(points)} valid_points={int(np.sum(valid))} "
            f"rmse_m={rmse:.4f} mae_m={mae:.4f} bias_m={bias:.4f} "
            f"coverage_cells={coverage_cells} coverage_percent={coverage_percent:.2f}"
        )


def main() -> None:
    rclpy.init()
    node = MapEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
