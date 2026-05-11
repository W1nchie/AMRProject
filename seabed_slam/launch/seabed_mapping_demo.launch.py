from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    world_path = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")

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

    spawn_boat = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_simple_boat",
        output="screen",
        arguments=[
            "-entity", "simple_boat",
            "-file", PathJoinSubstitution([boat_share, "urdf", "simple_boat.urdf"]),
            "-x", x,
            "-y", y,
            "-z", z,
        ],
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
                "voxel_size": 0.15,
                "max_points": 250000,
                "publish_rate_hz": 2.0,
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
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="1.2"),
        DeclareLaunchArgument("gui", default_value="true"),
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
    ])
