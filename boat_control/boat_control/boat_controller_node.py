import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Twist, Vector3Stamped
from nav_msgs.msg import Odometry


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q) -> float:
    """
    Extract yaw from geometry_msgs/Quaternion.
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class BoatControllerNode(Node):
    """
    Cascade controller for a surface boat.

    Inputs:
      - /odometry/filtered or /odometry/gt : nav_msgs/Odometry
      - /mission/waypoints                 : geometry_msgs/PoseArray

    Output:
      - /cmd_vel : geometry_msgs/Twist

    Logic:
      1. Outer PI controller computes desired world velocity q = [qx, qy].
      2. Convert q to desired heading psi_d and desired surge speed u_d.
      3. Inner speed PI controls cmd_vel.linear.x.
      4. Inner yaw PID-like controller controls cmd_vel.angular.z.
    """

    def __init__(self):
        super().__init__("boat_controller_node")

        # topics
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("waypoints_topic", "/mission/waypoints")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("disturbance_topic", "/disturbance/current")
        self.declare_parameter("use_disturbance_feedforward", False)

        # outer position PI gains
        self.declare_parameter("kp_pos", 0.6)
        self.declare_parameter("ki_pos", 0.02)

        # inner speed PI gains
        self.declare_parameter("kp_u", 1.2)
        self.declare_parameter("ki_u", 0.1)

        # yaw controller gains
        self.declare_parameter("kp_yaw", 1.8)
        self.declare_parameter("ki_yaw", 0.02)
        self.declare_parameter("kd_yaw", 0.4)

        # limits
        self.declare_parameter("u_max", 1.0)
        self.declare_parameter("w_max", 1.2)
        self.declare_parameter("cmd_linear_max", 1.0)
        self.declare_parameter("cmd_angular_max", 1.2)

        # waypoint logic
        self.declare_parameter("goal_tolerance", 0.4)
        self.declare_parameter("slowdown_radius", 2.0)
        self.declare_parameter("hold_final_waypoint", True)

        # anti-windup limits
        self.declare_parameter("integral_pos_max", 5.0)
        self.declare_parameter("integral_u_max", 3.0)
        self.declare_parameter("integral_yaw_max", 2.0)

        # estimated current compensation
        self.declare_parameter("current_vx_hat", 0.0)
        self.declare_parameter("current_vy_hat", 0.0)

        odom_topic = self.get_parameter("odom_topic").value
        waypoints_topic = self.get_parameter("waypoints_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        disturbance_topic = self.get_parameter("disturbance_topic").value

        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            20,
        )

        self.wp_sub = self.create_subscription(
            PoseArray,
            waypoints_topic,
            self.waypoints_callback,
            10,
        )

        self.disturbance_sub = self.create_subscription(
            Vector3Stamped,
            disturbance_topic,
            self.disturbance_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10,
        )

        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        # state
        self.has_odom = False
        self.x = 0.0
        self.y = 0.0
        self.psi = 0.0
        self.u = 0.0
        self.r = 0.0
        self.current_vx_hat = self.get_parameter("current_vx_hat").value
        self.current_vy_hat = self.get_parameter("current_vy_hat").value

        # waypoints
        self.waypoints: List[Tuple[float, float]] = []
        self.current_wp_idx = 0
        self.final_goal_announced = False

        # integrators
        self.int_ex = 0.0
        self.int_ey = 0.0
        self.int_eu = 0.0
        self.int_epsi = 0.0
        self.final_goal_announced = False

        self.prev_time = self.get_clock().now()

        self.get_logger().info("Boat controller started.")
        self.get_logger().info(f"Subscribing odom: {odom_topic}")
        self.get_logger().info(f"Subscribing waypoints: {waypoints_topic}")
        self.get_logger().info(f"Subscribing disturbance: {disturbance_topic}")
        self.get_logger().info(f"Publishing cmd_vel: {cmd_vel_topic}")

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.psi = yaw_from_quaternion(msg.pose.pose.orientation)

        # In nav_msgs/Odometry, twist is usually in child/body frame.
        self.u = msg.twist.twist.linear.x
        self.r = msg.twist.twist.angular.z

        self.has_odom = True

    def disturbance_callback(self, msg: Vector3Stamped):
        if not self.get_parameter("use_disturbance_feedforward").value:
            return

        self.current_vx_hat = msg.vector.x
        self.current_vy_hat = msg.vector.y

    def waypoints_callback(self, msg: PoseArray):
        new_waypoints = [
            (pose.position.x, pose.position.y)
            for pose in msg.poses
        ]

        if new_waypoints == self.waypoints:
            return

        self.waypoints = new_waypoints
        self.current_wp_idx = 0

        # reset integrators on new mission
        self.int_ex = 0.0
        self.int_ey = 0.0
        self.int_eu = 0.0
        self.int_epsi = 0.0

        self.get_logger().info(f"Received {len(self.waypoints)} waypoints.")

    def get_current_goal(self) -> Optional[Tuple[float, float]]:
        if not self.waypoints:
            return None

        if self.current_wp_idx >= len(self.waypoints):
            return None

        return self.waypoints[self.current_wp_idx]

    def switch_waypoint_if_needed(self, dist: float):
        goal_tolerance = self.get_parameter("goal_tolerance").value
        hold_final_waypoint = self.get_parameter("hold_final_waypoint").value

        if dist < goal_tolerance:
            if self.current_wp_idx == len(self.waypoints) - 1 and hold_final_waypoint:
                if not self.final_goal_announced:
                    self.get_logger().info("Final waypoint reached. Holding position.")
                    self.final_goal_announced = True
                return

            self.get_logger().info(
                f"Waypoint {self.current_wp_idx + 1}/{len(self.waypoints)} reached."
            )
            self.current_wp_idx += 1
            self.final_goal_announced = False

            # reset position integrators after switching goal
            self.int_ex = 0.0
            self.int_ey = 0.0

    def publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        try:
            self.cmd_pub.publish(cmd)
        except Exception:
            pass

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now

        if dt <= 0.0 or dt > 1.0:
            dt = 0.05

        if not self.has_odom:
            self.publish_stop()
            return

        goal = self.get_current_goal()
        if goal is None:
            self.publish_stop()
            return

        xd, yd = goal

        # ---------------------------------------------------------
        # 1. position error in world frame
        # ---------------------------------------------------------
        ex = xd - self.x
        ey = yd - self.y
        dist = math.hypot(ex, ey)

        self.switch_waypoint_if_needed(dist)

        goal = self.get_current_goal()
        if goal is None:
            self.publish_stop()
            self.get_logger().info("Mission completed.")
            return

        xd, yd = goal
        ex = xd - self.x
        ey = yd - self.y
        dist = math.hypot(ex, ey)

        # ---------------------------------------------------------
        # 2. outer PI position controller
        # ---------------------------------------------------------
        kp_pos = self.get_parameter("kp_pos").value
        ki_pos = self.get_parameter("ki_pos").value
        integral_pos_max = self.get_parameter("integral_pos_max").value

        self.int_ex += ex * dt
        self.int_ey += ey * dt

        self.int_ex = clamp(self.int_ex, -integral_pos_max, integral_pos_max)
        self.int_ey = clamp(self.int_ey, -integral_pos_max, integral_pos_max)

        current_vx_hat = self.current_vx_hat
        current_vy_hat = self.current_vy_hat

        qx = kp_pos * ex + ki_pos * self.int_ex - current_vx_hat
        qy = kp_pos * ey + ki_pos * self.int_ey - current_vy_hat

        psi_d = math.atan2(qy, qx)

        u_max = self.get_parameter("u_max").value
        slowdown_radius = self.get_parameter("slowdown_radius").value

        ud_raw = math.hypot(qx, qy)

        # slowdown near goal to reduce overshoot
        slowdown_factor = clamp(dist / slowdown_radius, 0.15, 1.0)
        ud = clamp(ud_raw * slowdown_factor, 0.0, u_max)

        # ---------------------------------------------------------
        # 3. heading error
        # ---------------------------------------------------------
        epsi = wrap_to_pi(psi_d - self.psi)
        # If heading error is too large, reduce forward speed.
        # This prevents the boat from moving forward in the wrong direction.
        heading_factor = max(0.0, math.cos(epsi))
        ud *= heading_factor

        # ---------------------------------------------------------
        # 4. inner speed PI controller
        # ---------------------------------------------------------
        kp_u = self.get_parameter("kp_u").value
        ki_u = self.get_parameter("ki_u").value
        integral_u_max = self.get_parameter("integral_u_max").value

        eu = ud - self.u
        self.int_eu += eu * dt
        self.int_eu = clamp(self.int_eu, -integral_u_max, integral_u_max)

        cmd_linear = kp_u * eu + ki_u * self.int_eu

        # ---------------------------------------------------------
        # 5. yaw PID-like controller: kp*epsi + ki*int - kd*r
        # ---------------------------------------------------------
        kp_yaw = self.get_parameter("kp_yaw").value
        ki_yaw = self.get_parameter("ki_yaw").value
        kd_yaw = self.get_parameter("kd_yaw").value
        integral_yaw_max = self.get_parameter("integral_yaw_max").value

        self.int_epsi += epsi * dt
        self.int_epsi = clamp(self.int_epsi, -integral_yaw_max, integral_yaw_max)

        cmd_angular = kp_yaw * epsi + ki_yaw * self.int_epsi - kd_yaw * self.r

        # ---------------------------------------------------------
        # 6. saturation
        # ---------------------------------------------------------
        cmd_linear_max = self.get_parameter("cmd_linear_max").value
        cmd_angular_max = self.get_parameter("cmd_angular_max").value

        cmd_linear = clamp(cmd_linear, 0.0, cmd_linear_max)
        cmd_angular = clamp(cmd_angular, -cmd_angular_max, cmd_angular_max)

        # ---------------------------------------------------------
        # 7. publish /cmd_vel
        # ---------------------------------------------------------
        cmd = Twist()
        cmd.linear.x = cmd_linear
        cmd.angular.z = cmd_angular
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BoatControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.publish_stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
