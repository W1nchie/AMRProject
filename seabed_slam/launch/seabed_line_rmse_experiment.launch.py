from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    world_path = LaunchConfiguration("world")

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
            "-x", "0.0",
            "-y", "0.0",
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
                "max_points": 120000,
                "publish_rate_hz": 2.0,
            }
        ],
    )

    evaluator = Node(
        package="seabed_slam",
        executable="map_evaluator",
        name="seabed_map_evaluator",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "map_points_topic": "/seabed/map_points",
                "mesh_scale": 0.5,
                "coverage_cell_m": 0.5,
                "min_points": 200,
                "report_period_s": 2.0,
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
                "cmd_vel_topic": "/cmd_vel_controller",
                "disturbance_topic": "/disturbance/current",
                "use_sim_time": use_sim_time,
                "kp_pos": 0.6,
                "ki_pos": 0.02,
                "kp_u": 1.2,
                "ki_u": 0.1,
                "kp_yaw": 1.8,
                "ki_yaw": 0.02,
                "kd_yaw": 0.25,
                "u_max": 0.6,
                "w_max": 1.0,
                "cmd_linear_max": 0.6,
                "cmd_angular_max": 1.0,
                "goal_tolerance": 0.6,
                "slowdown_radius": 2.5,
                "hold_final_waypoint": True,
            }
        ],
    )

    disturbance_applier = Node(
        package="boat_control",
        executable="disturbance_applier",
        name="disturbance_applier_node",
        output="screen",
        parameters=[
            {
                "input_cmd_topic": "/cmd_vel_controller",
                "output_cmd_topic": "/cmd_vel",
                "odom_topic": "/odom",
                "disturbance_topic": "/disturbance/current",
                "use_sim_time": use_sim_time,
                "lateral_yaw_gain": 0.0,
                "max_extra_linear": 0.0,
                "max_extra_angular": 0.0,
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
                    22.0, 0.0,
                ],
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(rviz),
        arguments=[
            "-d",
            PathJoinSubstitution([seabed_share, "config", "seabed_mapping.rviz"]),
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
        DeclareLaunchArgument("rviz", default_value="true"),
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
        evaluator,
        controller,
        waypoints,
        disturbance_applier,
        rviz_node,
    ])
