import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2

from seabed_slam.bathymetry_reference import (
    DEFAULT_META_PATH,
    DEFAULT_SOURCE_CSV,
    BathymetryReference,
    RecordedMapReference,
)
from seabed_slam.sonar_mapper_node import _read_xyz


def _yaw_to_quaternion(yaw: float):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def _wrap_to_pi(angle: np.ndarray) -> np.ndarray:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _wrap_to_pi_scalar(angle: float) -> float:
    return float((angle + math.pi) % (2.0 * math.pi) - math.pi)


class SeabedMcl(Node):
    def __init__(self) -> None:
        super().__init__("seabed_mcl")

        self.declare_parameter("source_csv", DEFAULT_SOURCE_CSV)
        self.declare_parameter("meta_path", DEFAULT_META_PATH)
        self.declare_parameter("reference_csv", "")

        self.declare_parameter("sonar_points_topic", "/sonar/downward/points")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        
        self.declare_parameter("odom_output_topic", "/seabed_localization/odom")
        self.declare_parameter("base_link_frame", "base_link")  
        self.declare_parameter("pose_topic", "/seabed_localization/pose")
        self.declare_parameter("particles_topic", "/seabed_localization/particles")

        self.declare_parameter("map_frame", "odom")

        self.declare_parameter("mesh_scale", 0.5)
        self.declare_parameter("inner_margin_m", 6.0)
        self.declare_parameter("num_particles", 900)

        self.declare_parameter("initial_yaw_min", -math.pi)
        self.declare_parameter("initial_yaw_max", math.pi)

        self.declare_parameter("base_z", 1.2)
        self.declare_parameter("sonar_offset_z", -0.04)

        # Motion model.
        #
        # Options:
        #   cmd_vel     -> use geometry_msgs/Twist from cmd_vel_topic
        #   odom_twist  -> use nav_msgs/Odometry.twist from odom_topic
        #   fixed       -> use motion_linear_x / motion_linear_y / motion_angular_z
        #   auto        -> prefer odom_twist if recent, then cmd_vel if recent, else fixed
        self.declare_parameter("motion_source", "cmd_vel")
        self.declare_parameter("cmd_vel_timeout_s", 0.5)
        self.declare_parameter("odom_twist_timeout_s", 0.5)

        self.declare_parameter("use_lateral_velocity", True)

        # Fallback / fixed motion.
        self.declare_parameter("motion_linear_x", 0.35)
        self.declare_parameter("motion_linear_y", 0.0)
        self.declare_parameter("motion_angular_z", 0.0)

        # Scale command to approximate actual boat velocity.
        # Useful if /cmd_vel is not equal to true body velocity.
        self.declare_parameter("cmd_vel_linear_x_scale", 1.0)
        self.declare_parameter("cmd_vel_linear_y_scale", 1.0)
        self.declare_parameter("cmd_vel_angular_z_scale", 1.0)

        self.declare_parameter("odom_twist_linear_x_scale", 1.0)
        self.declare_parameter("odom_twist_linear_y_scale", 1.0)
        self.declare_parameter("odom_twist_angular_z_scale", 1.0)

        self.declare_parameter("motion_noise_xy", 0.035)
        self.declare_parameter("motion_noise_yaw", 0.015)

        self.declare_parameter("measurement_sigma_z", 0.18)
        self.declare_parameter("resample_neff_ratio", 0.55)
        self.declare_parameter("max_scan_points", 160)
        self.declare_parameter("report_period_s", 1.0)

        self.declare_parameter("min_predict_dt", 0.001)
        self.declare_parameter("max_predict_dt", 0.5)
        self.declare_parameter("default_predict_dt", 0.1)

        reference_csv = str(self.get_parameter("reference_csv").value)
        if reference_csv:
            self._reference = RecordedMapReference(
                reference_csv=reference_csv,
                inner_margin_m=float(self.get_parameter("inner_margin_m").value),
            )
            self.get_logger().info(f"Loaded recorded reference map: {reference_csv}")
        else:
            self._reference = BathymetryReference(
                source_csv=str(self.get_parameter("source_csv").value),
                meta_path=str(self.get_parameter("meta_path").value),
                mesh_scale=float(self.get_parameter("mesh_scale").value),
                inner_margin_m=float(self.get_parameter("inner_margin_m").value),
            )
            self.get_logger().info("Loaded GT bathymetry as reference map")

        self._rng = np.random.default_rng(42)
        self._num_particles = int(self.get_parameter("num_particles").value)
        self._particles = self._sample_uniform(self._num_particles)
        self._weights = np.full(self._num_particles, 1.0 / self._num_particles)

        self._last_scan_time = self.get_clock().now()
        self._last_report_time = self.get_clock().now()

        self._gt_pose: Optional[Tuple[float, float, float]] = None

        self._last_cmd_time: Optional[Time] = None
        self._cmd_vx = 0.0
        self._cmd_vy = 0.0
        self._cmd_wz = 0.0

        self._last_odom_twist_time: Optional[Time] = None
        self._odom_vx = 0.0
        self._odom_vy = 0.0
        self._odom_wz = 0.0

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self._scan_sub = self.create_subscription(
            PointCloud2,
            str(self.get_parameter("sonar_points_topic").value),
            self._on_scan,
            qos,
        )

        self._odom_sub = self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odom,
            10,
        )

        self._cmd_vel_sub = self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self._on_cmd_vel,
            10,
        )

        self._pose_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("pose_topic").value),
            10,
        )

        self._particles_pub = self.create_publisher(
            PoseArray,
            str(self.get_parameter("particles_topic").value),
            1,
        )

        self._odom_pub = self.create_publisher(
            Odometry,
            str(self.get_parameter("odom_output_topic").value),
            10,
        )

        self.get_logger().info(
            "MCL initialized in inner bounds "
            f"x=[{self._reference.inner_x_min:.1f}, {self._reference.inner_x_max:.1f}], "
            f"y=[{self._reference.inner_y_min:.1f}, {self._reference.inner_y_max:.1f}]"
        )

        self.get_logger().info(
            "Motion model source: "
            f"{str(self.get_parameter('motion_source').value)} "
            f"(cmd_vel_topic={str(self.get_parameter('cmd_vel_topic').value)}, "
            f"odom_topic={str(self.get_parameter('odom_topic').value)})"
        )

    def _sample_uniform(self, count: int) -> np.ndarray:
        particles = np.zeros((count, 3), dtype=np.float64)
        particles[:, 0] = self._rng.uniform(
            self._reference.inner_x_min,
            self._reference.inner_x_max,
            count,
        )
        particles[:, 1] = self._rng.uniform(
            self._reference.inner_y_min,
            self._reference.inner_y_max,
            count,
        )
        particles[:, 2] = self._rng.uniform(
            float(self.get_parameter("initial_yaw_min").value),
            float(self.get_parameter("initial_yaw_max").value),
            count,
        )
        return particles

    def _age_s(self, stamp: Optional[Time]) -> Optional[float]:
        if stamp is None:
            return None
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9

    def _get_cmd_vel_motion(self) -> Optional[Tuple[float, float, float]]:
        age = self._age_s(self._last_cmd_time)
        if age is None or age > float(self.get_parameter("cmd_vel_timeout_s").value):
            return None

        vx = self._cmd_vx * float(self.get_parameter("cmd_vel_linear_x_scale").value)
        vy = self._cmd_vy * float(self.get_parameter("cmd_vel_linear_y_scale").value)
        wz = self._cmd_wz * float(self.get_parameter("cmd_vel_angular_z_scale").value)

        if not bool(self.get_parameter("use_lateral_velocity").value):
            vy = 0.0

        return vx, vy, wz

    def _get_odom_twist_motion(self) -> Optional[Tuple[float, float, float]]:
        age = self._age_s(self._last_odom_twist_time)
        if age is None or age > float(self.get_parameter("odom_twist_timeout_s").value):
            return None

        vx = self._odom_vx * float(self.get_parameter("odom_twist_linear_x_scale").value)
        vy = self._odom_vy * float(self.get_parameter("odom_twist_linear_y_scale").value)
        wz = self._odom_wz * float(self.get_parameter("odom_twist_angular_z_scale").value)

        if not bool(self.get_parameter("use_lateral_velocity").value):
            vy = 0.0

        return vx, vy, wz

    def _get_fixed_motion(self) -> Tuple[float, float, float]:
        vx = float(self.get_parameter("motion_linear_x").value)
        vy = float(self.get_parameter("motion_linear_y").value)
        wz = float(self.get_parameter("motion_angular_z").value)

        if not bool(self.get_parameter("use_lateral_velocity").value):
            vy = 0.0

        return vx, vy, wz

    def _get_motion_input(self) -> Tuple[float, float, float]:
        source = str(self.get_parameter("motion_source").value).strip().lower()

        if source == "cmd_vel":
            return self._get_cmd_vel_motion() or (0.0, 0.0, 0.0)

        if source == "odom_twist":
            return self._get_odom_twist_motion() or (0.0, 0.0, 0.0)

        if source == "fixed":
            return self._get_fixed_motion()

        if source == "auto":
            odom_motion = self._get_odom_twist_motion()
            if odom_motion is not None:
                return odom_motion

            cmd_motion = self._get_cmd_vel_motion()
            if cmd_motion is not None:
                return cmd_motion

            return self._get_fixed_motion()

        self.get_logger().warn(
            f"Unknown motion_source='{source}', using zero motion.",
            throttle_duration_sec=2.0,
        )
        return 0.0, 0.0, 0.0

    def _predict(self, dt: float) -> None:
        min_dt = float(self.get_parameter("min_predict_dt").value)
        max_dt = float(self.get_parameter("max_predict_dt").value)
        default_dt = float(self.get_parameter("default_predict_dt").value)

        if dt <= min_dt or dt > max_dt or not math.isfinite(dt):
            dt = default_dt

        vx_body, vy_body, wz = self._get_motion_input()

        yaw = self._particles[:, 2]
        yaw_mid = yaw + 0.5 * wz * dt

        c = np.cos(yaw_mid)
        s = np.sin(yaw_mid)

        # Body-frame velocity -> map-frame particle displacement.
        self._particles[:, 0] += (vx_body * c - vy_body * s) * dt
        self._particles[:, 1] += (vx_body * s + vy_body * c) * dt

        self._particles[:, 2] = _wrap_to_pi(self._particles[:, 2] + wz * dt)

        # Process noise.
        self._particles[:, 0] += self._rng.normal(
            0.0,
            float(self.get_parameter("motion_noise_xy").value),
            self._num_particles,
        )
        self._particles[:, 1] += self._rng.normal(
            0.0,
            float(self.get_parameter("motion_noise_xy").value),
            self._num_particles,
        )
        self._particles[:, 2] = _wrap_to_pi(
            self._particles[:, 2]
            + self._rng.normal(
                0.0,
                float(self.get_parameter("motion_noise_yaw").value),
                self._num_particles,
            )
        )

        outside = ~self._reference.inside_inner(self._particles[:, :2])
        if np.any(outside):
            self._particles[outside] = self._sample_uniform(int(np.sum(outside)))
            self._weights[outside] = 1.0 / self._num_particles

    def _scan_points(self, message: PointCloud2) -> np.ndarray:
        points = np.asarray(list(_read_xyz(message)), dtype=np.float64)
        if len(points) == 0:
            return points.reshape((0, 3))

        max_points = int(self.get_parameter("max_scan_points").value)
        if len(points) > max_points:
            indices = self._rng.choice(len(points), size=max_points, replace=False)
            points = points[indices]

        return points

    def _score_particles(self, scan_sonar: np.ndarray) -> np.ndarray:
        base_z = float(self.get_parameter("base_z").value)
        sonar_offset_z = float(self.get_parameter("sonar_offset_z").value)
        sigma = float(self.get_parameter("measurement_sigma_z").value)

        # URDF fixed joint base_link -> sonar_link has rpy=(0, pi/2, 0).
        # Therefore a sonar-frame point is transformed to base frame as:
        #   base_x = sonar_z
        #   base_y = sonar_y
        #   base_z = -sonar_x + sonar_offset_z
        local_x = scan_sonar[:, 2]
        local_y = scan_sonar[:, 1]
        local_z = -scan_sonar[:, 0] + sonar_offset_z

        log_likelihood = np.empty(self._num_particles, dtype=np.float64)

        for i, (px, py, yaw) in enumerate(self._particles):
            c = math.cos(yaw)
            s = math.sin(yaw)

            world_x = px + c * local_x - s * local_y
            world_y = py + s * local_x + c * local_y
            world_z = base_z + local_z

            xy = np.column_stack([world_x, world_y])
            inside = self._reference.inside_inner(xy)

            if np.count_nonzero(inside) < max(8, len(scan_sonar) // 5):
                log_likelihood[i] = -1e6
                continue

            gt_z = self._reference.height(xy[inside])
            errors = world_z[inside] - gt_z
            rmse = float(np.sqrt(np.mean(errors ** 2)))
            log_likelihood[i] = -0.5 * (rmse / sigma) ** 2

        return log_likelihood

    def _resample_if_needed(self) -> None:
        neff = 1.0 / float(np.sum(self._weights ** 2))
        threshold = float(self.get_parameter("resample_neff_ratio").value) * self._num_particles
        if neff >= threshold:
            return

        positions = (self._rng.random() + np.arange(self._num_particles)) / self._num_particles
        cumulative = np.cumsum(self._weights)
        indices = np.searchsorted(cumulative, positions)
        self._particles = self._particles[indices].copy()
        self._particles[:, 0] += self._rng.normal(0.0, 0.03, self._num_particles)
        self._particles[:, 1] += self._rng.normal(0.0, 0.03, self._num_particles)
        self._particles[:, 2] = _wrap_to_pi(
            self._particles[:, 2] + self._rng.normal(0.0, 0.01, self._num_particles)
        )
        self._weights.fill(1.0 / self._num_particles)

    def _estimate(self) -> Tuple[float, float, float]:
        x = float(np.sum(self._particles[:, 0] * self._weights))
        y = float(np.sum(self._particles[:, 1] * self._weights))
        sin_yaw = float(np.sum(np.sin(self._particles[:, 2]) * self._weights))
        cos_yaw = float(np.sum(np.cos(self._particles[:, 2]) * self._weights))
        yaw = math.atan2(sin_yaw, cos_yaw)
        return x, y, yaw

    def _publish(self, stamp) -> None:
        x, y, yaw = self._estimate()
        frame_id = str(self.get_parameter("map_frame").value)
        child_frame_id = str(self.get_parameter("base_link_frame").value)

        qx, qy, qz, qw = _yaw_to_quaternion(yaw)

        # ------------------------------------------------------------
        # PoseStamped output
        # ------------------------------------------------------------
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = float(self.get_parameter("base_z").value)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self._pose_pub.publish(pose)

        # ------------------------------------------------------------
        # Odometry output
        # ------------------------------------------------------------
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = frame_id
        odom.child_frame_id = child_frame_id

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = float(self.get_parameter("base_z").value)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Estimate covariance from particles.
        dx = self._particles[:, 0] - x
        dy = self._particles[:, 1] - y
        dyaw = _wrap_to_pi(self._particles[:, 2] - yaw)

        var_x = float(np.sum(self._weights * dx * dx))
        var_y = float(np.sum(self._weights * dy * dy))
        var_yaw = float(np.sum(self._weights * dyaw * dyaw))

        # geometry_msgs/PoseWithCovariance covariance order:
        # x, y, z, roll, pitch, yaw
        odom.pose.covariance[0] = max(var_x, 1e-6)
        odom.pose.covariance[7] = max(var_y, 1e-6)
        odom.pose.covariance[14] = 1e-3
        odom.pose.covariance[21] = 1e-3
        odom.pose.covariance[28] = 1e-3
        odom.pose.covariance[35] = max(var_yaw, 1e-6)

        # Twist is the motion-model input in child/body frame.
        # If motion_source=cmd_vel, this is command velocity.
        # If motion_source=odom_twist, this is measured/estimated twist.
        vx_body, vy_body, wz = self._get_motion_input()
        odom.twist.twist.linear.x = float(vx_body)
        odom.twist.twist.linear.y = float(vy_body)
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = float(wz)

        # Conservative twist covariance.
        motion_noise_xy = float(self.get_parameter("motion_noise_xy").value)
        motion_noise_yaw = float(self.get_parameter("motion_noise_yaw").value)

        odom.twist.covariance[0] = max(motion_noise_xy ** 2, 1e-6)
        odom.twist.covariance[7] = max(motion_noise_xy ** 2, 1e-6)
        odom.twist.covariance[14] = 1e3
        odom.twist.covariance[21] = 1e3
        odom.twist.covariance[28] = 1e3
        odom.twist.covariance[35] = max(motion_noise_yaw ** 2, 1e-6)

        self._odom_pub.publish(odom)

        # ------------------------------------------------------------
        # Particle cloud output
        # ------------------------------------------------------------
        particles = PoseArray()
        particles.header = pose.header

        step = max(1, self._num_particles // 200)
        for px, py, pyaw in self._particles[::step]:
            from geometry_msgs.msg import Pose

            p = Pose()
            p.position.x = float(px)
            p.position.y = float(py)
            p.position.z = float(self.get_parameter("base_z").value)

            _, _, qz_p, qw_p = _yaw_to_quaternion(float(pyaw))
            p.orientation.z = qz_p
            p.orientation.w = qw_p

            particles.poses.append(p)

        self._particles_pub.publish(particles)

    def _on_cmd_vel(self, message: Twist) -> None:
        self._last_cmd_time = self.get_clock().now()
        self._cmd_vx = float(message.linear.x)
        self._cmd_vy = float(message.linear.y)
        self._cmd_wz = float(message.angular.z)

    def _on_odom(self, message: Odometry) -> None:
        q = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        # Optional debug/metric pose.
        self._gt_pose = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            yaw,
        )

        # Optional twist source for prediction.
        self._last_odom_twist_time = self.get_clock().now()
        self._odom_vx = float(message.twist.twist.linear.x)
        self._odom_vy = float(message.twist.twist.linear.y)
        self._odom_wz = float(message.twist.twist.angular.z)

    def _on_scan(self, message: PointCloud2) -> None:
        scan = self._scan_points(message)
        if len(scan) < 20:
            return

        now = self.get_clock().now()
        dt = (now - self._last_scan_time).nanoseconds * 1e-9
        self._last_scan_time = now

        self._predict(dt)

        log_likelihood = self._score_particles(scan)
        best = float(np.max(log_likelihood))

        likelihood = np.exp(log_likelihood - best)
        self._weights *= likelihood

        total = float(np.sum(self._weights))
        if total <= 0.0 or not math.isfinite(total):
            self._particles = self._sample_uniform(self._num_particles)
            self._weights.fill(1.0 / self._num_particles)
        else:
            self._weights /= total

        self._resample_if_needed()
        self._publish(message.header.stamp)

        report_period_ns = int(float(self.get_parameter("report_period_s").value) * 1e9)
        if (now - self._last_report_time).nanoseconds < report_period_ns:
            return

        self._last_report_time = now

        estimate = self._estimate()
        neff = 1.0 / float(np.sum(self._weights ** 2))

        vx, vy, wz = self._get_motion_input()

        if self._gt_pose is not None:
            pos_error = math.hypot(
                estimate[0] - self._gt_pose[0],
                estimate[1] - self._gt_pose[1],
            )
            yaw_error = abs(
                float(_wrap_to_pi(np.asarray([estimate[2] - self._gt_pose[2]]))[0])
            )

            self.get_logger().info(
                "seabed_mcl_metrics "
                f"x={estimate[0]:.3f} y={estimate[1]:.3f} yaw={estimate[2]:.3f} "
                f"pos_error_m={pos_error:.3f} yaw_error_rad={yaw_error:.3f} "
                f"neff={neff:.1f} "
                f"motion_vx={vx:.3f} motion_vy={vy:.3f} motion_wz={wz:.3f}"
            )
        else:
            self.get_logger().info(
                "seabed_mcl_metrics "
                f"x={estimate[0]:.3f} y={estimate[1]:.3f} yaw={estimate[2]:.3f} "
                f"neff={neff:.1f} "
                f"motion_vx={vx:.3f} motion_vy={vy:.3f} motion_wz={wz:.3f}"
            )


def main() -> None:
    rclpy.init()
    node = SeabedMcl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()