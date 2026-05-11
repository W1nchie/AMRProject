from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    world_path = LaunchConfiguration("world")
    output_csv = LaunchConfiguration("output_csv")

    gazebo_launch = FindPackageShare("gazebo_ros")
    boat_share = FindPackageShare("boat_control")
    seabed_share = FindPackageShare("seabed_slam")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_launch, "launch", "gazebo.launch.py"])
        ),
        launch_arguments={
            "world": world_path,
            "gui": gui,
            "verbose": "true",
        }.items(),
    )

    robot_description = ParameterValue(
        Command([
            "cat ",
            PathJoinSubstitution([boat_share, "urdf", "simple_boat.urdf"]),
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    spawn_boat = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_simple_boat",
        output="screen",
        arguments=[
            "-entity", "simple_boat",
            "-file", PathJoinSubstitution([boat_share, "urdf", "simple_boat.urdf"]),
            "-x", "-18.0",
            "-y", "-10.0",
            "-z", "1.2",
        ],
    )

    mapper = Node(
        package="seabed_slam",
        executable="sonar_mapper",
        name="sonar_mapper",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "sonar_points_topic": "/sonar/downward/points",
                "map_points_topic": "/seabed/map_points",
                "map_frame": "odom",
                "voxel_size": 0.10,
                "max_points": 250000,
                "publish_rate_hz": 2.0,
            }
        ],
    )

    recorder = Node(
        package="seabed_slam",
        executable="reference_map_recorder",
        name="reference_map_recorder",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "map_points_topic": "/seabed/map_points",
                "output_csv": output_csv,
                "min_points": 5000,
                "save_period_s": 5.0,
            }
        ],
    )

    controller = Node(
        package="boat_control",
        executable="boat_controller",
        name="boat_controller_node",
        output="screen",
        parameters=[
            {
                "odom_topic": "/odom",
                "waypoints_topic": "/mission/waypoints",
                "cmd_vel_topic": "/cmd_vel",
                "disturbance_topic": "/disturbance/current",
                "use_sim_time": use_sim_time,
                "kp_pos": 0.6,
                "ki_pos": 0.02,
                "kp_u": 1.2,
                "ki_u": 0.1,
                "kp_yaw": 1.8,
                "ki_yaw": 0.02,
                "kd_yaw": 0.25,
                "u_max": 0.8,
                "w_max": 1.1,
                "cmd_linear_max": 0.8,
                "cmd_angular_max": 1.1,
                "goal_tolerance": 0.8,
                "slowdown_radius": 2.5,
                "hold_final_waypoint": True,
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
                "topic": "/mission/waypoints",
                "frame_id": "odom",
                "use_sim_time": use_sim_time,
                "waypoints": [
                    18.0, -10.0,
                    18.0, -5.0,
                    -18.0, -5.0,
                    -18.0, 0.0,
                    18.0, 0.0,
                    18.0, 5.0,
                    -18.0, 5.0,
                    -18.0, 10.0,
                    18.0, 10.0,
                ],
            }
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable("GAZEBO_RESOURCE_PATH", "/usr/share/gazebo-11"),
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", "/usr/share/gazebo-11/models"),
        SetEnvironmentVariable("GAZEBO_PLUGIN_PATH", "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"),
        SetEnvironmentVariable("OGRE_RESOURCE_PATH", "/usr/lib/x86_64-linux-gnu/OGRE-1.9.0"),
        SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1"),
        SetEnvironmentVariable("QT_X11_NO_MITSHM", "1"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument(
            "output_csv",
            default_value="/home/fabian/ros2_ws/src/seabed_slam/data/missouri_i64/recorded_reference_map.csv",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([
                seabed_share,
                "worlds",
                "missouri_i64_featured_seabed.world",
            ]),
        ),
        gazebo,
        robot_state_publisher,
        spawn_boat,
        mapper,
        recorder,
        controller,
        waypoints,
    ])
