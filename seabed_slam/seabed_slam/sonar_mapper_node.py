import math
import struct
from typing import Dict, Iterable, List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener


Point3 = Tuple[float, float, float]


def _field_map(message: PointCloud2) -> Dict[str, PointField]:
    return {field.name: field for field in message.fields}


def _struct_format(field: PointField) -> Optional[str]:
    if field.datatype == PointField.FLOAT32:
        return "f"
    if field.datatype == PointField.FLOAT64:
        return "d"
    if field.datatype == PointField.INT32:
        return "i"
    if field.datatype == PointField.UINT32:
        return "I"
    if field.datatype == PointField.INT16:
        return "h"
    if field.datatype == PointField.UINT16:
        return "H"
    if field.datatype == PointField.INT8:
        return "b"
    if field.datatype == PointField.UINT8:
        return "B"
    return None


def _read_xyz(message: PointCloud2) -> Iterable[Point3]:
    fields = _field_map(message)
    required = [fields.get("x"), fields.get("y"), fields.get("z")]
    if any(field is None for field in required):
        return []

    endian = ">" if message.is_bigendian else "<"
    unpackers = []
    for field in required:
        fmt = _struct_format(field)
        if fmt is None:
            return []
        unpackers.append((struct.Struct(endian + fmt), field.offset))

    points = []
    for row in range(message.height):
        row_base = row * message.row_step
        for col in range(message.width):
            base = row_base + col * message.point_step
            values = [
                float(unpacker.unpack_from(message.data, base + offset)[0])
                for unpacker, offset in unpackers
            ]
            if all(math.isfinite(value) for value in values):
                points.append((values[0], values[1], values[2]))
    return points


def _quaternion_to_matrix(x: float, y: float, z: float, w: float) -> Tuple[Tuple[float, float, float], ...]:
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def _transform_points(points: Iterable[Point3], transform) -> List[Point3]:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    matrix = _quaternion_to_matrix(rotation.x, rotation.y, rotation.z, rotation.w)

    output = []
    for x, y, z in points:
        tx = matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z + translation.x
        ty = matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z + translation.y
        tz = matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z + translation.z
        output.append((tx, ty, tz))
    return output


def _build_cloud(points: List[Point3], frame_id: str) -> PointCloud2:
    message = PointCloud2()
    message.header.frame_id = frame_id
    message.height = 1
    message.width = len(points)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = message.point_step * len(points)
    message.is_dense = True

    data = bytearray(message.row_step)
    for index, (x, y, z) in enumerate(points):
        struct.pack_into("<fff", data, index * message.point_step, x, y, z)
    message.data = bytes(data)
    return message


class SonarMapper(Node):
    def __init__(self) -> None:
        super().__init__("sonar_mapper")

        self.declare_parameter("sonar_points_topic", "/sonar/downward/points")
        self.declare_parameter("map_points_topic", "/seabed/map_points")
        self.declare_parameter("map_frame", "odom")
        self.declare_parameter("max_points", 250000)
        self.declare_parameter("voxel_size", 0.15)
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("transform_timeout_s", 0.05)

        self._map_frame = self.get_parameter("map_frame").value
        self._max_points = int(self.get_parameter("max_points").value)
        self._voxel_size = float(self.get_parameter("voxel_size").value)
        self._transform_timeout = float(self.get_parameter("transform_timeout_s").value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._subscriber = self.create_subscription(
            PointCloud2,
            self.get_parameter("sonar_points_topic").value,
            self._on_sonar_points,
            qos,
        )
        self._publisher = self.create_publisher(
            PointCloud2,
            self.get_parameter("map_points_topic").value,
            1,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._points_by_voxel: Dict[Tuple[int, int, int], Point3] = {}
        self._last_tf_warning_time = self.get_clock().now()
        self._last_publish_time = self.get_clock().now()
        self._publish_period_ns = int(
            1_000_000_000 / float(self.get_parameter("publish_rate_hz").value)
        )
        self._timer = self.create_timer(
            1.0 / float(self.get_parameter("publish_rate_hz").value),
            self._publish_map,
        )

    def _voxel_key(self, point: Point3) -> Tuple[int, int, int]:
        return (
            math.floor(point[0] / self._voxel_size),
            math.floor(point[1] / self._voxel_size),
            math.floor(point[2] / self._voxel_size),
        )

    def _on_sonar_points(self, message: PointCloud2) -> None:
        source_frame = message.header.frame_id
        if not source_frame:
            self.get_logger().warn("Received sonar point cloud without frame_id")
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=self._transform_timeout),
            )
        except TransformException as exc:
            now = self.get_clock().now()
            if (now - self._last_tf_warning_time).nanoseconds > 2_000_000_000:
                self.get_logger().warn(
                    f"Cannot transform sonar cloud from {source_frame} to {self._map_frame}: {exc}"
                )
                self._last_tf_warning_time = now
            return

        transformed = _transform_points(_read_xyz(message), transform)
        for point in transformed:
            self._points_by_voxel[self._voxel_key(point)] = point

        overflow = len(self._points_by_voxel) - self._max_points
        if overflow > 0:
            for key in list(self._points_by_voxel.keys())[:overflow]:
                del self._points_by_voxel[key]

        now = self.get_clock().now()
        if (now - self._last_publish_time).nanoseconds >= self._publish_period_ns:
            self._publish_map()

    def _publish_map(self) -> None:
        points = list(self._points_by_voxel.values())
        message = _build_cloud(points, self._map_frame)
        message.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(message)
        self._last_publish_time = self.get_clock().now()


def main() -> None:
    rclpy.init()
    node = SonarMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
