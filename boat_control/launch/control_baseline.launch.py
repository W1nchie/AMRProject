from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    odom_topic = LaunchConfiguration("odom_topic")
    waypoints_topic = LaunchConfiguration("waypoints_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    controller = Node(
        package="boat_control",
        executable="boat_controller",
        name="boat_controller_node",
        output="screen",
        parameters=[
            {
                "odom_topic": odom_topic,
                "waypoints_topic": waypoints_topic,
                "cmd_vel_topic": cmd_vel_topic,
                "use_sim_time": use_sim_time,

                "kp_pos": 0.6,
                "ki_pos": 0.02,

                "kp_u": 1.2,
                "ki_u": 0.1,

                "kp_yaw": 1.8,
                "ki_yaw": 0.02,
                "kd_yaw": 0.4,

                "u_max": 1.0,
                "w_max": 1.2,
                "cmd_linear_max": 1.0,
                "cmd_angular_max": 1.2,

                "goal_tolerance": 0.4,
                "slowdown_radius": 2.0,

                "current_vx_hat": 0.0,
                "current_vy_hat": 0.0,
            }
        ],
    )

    waypoints = Node(
        package="boat_control",
        executable="waypoint_publisher",
        name="waypoint_publisher_node",
        output="screen",
        parameters=[
            {
                "topic": waypoints_topic,
                "frame_id": "map",
                "use_sim_time": use_sim_time,
                "waypoints": [
                    5.0, 0.0,
                    5.0, 5.0,
                    0.0, 5.0,
                    0.0, 0.0,
                ],
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("odom_topic", default_value="/odometry/filtered"),
        DeclareLaunchArgument("waypoints_topic", default_value="/mission/waypoints"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        controller,
        waypoints,
    ])
