from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")
    world = LaunchConfiguration("world")
    use_scan_noise = LaunchConfiguration("use_scan_noise")
    robot_x = LaunchConfiguration("robot_x")
    robot_y = LaunchConfiguration("robot_y")
    robot_yaw = LaunchConfiguration("robot_yaw")

    robot_description_file = PathJoinSubstitution([
        FindPackageShare("robot_bringup"),
        "description",
        "simple_lidar_bot.urdf.xacro",
    ])

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", robot_description_file]),
        value_type=str,
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gzserver.launch.py",
            ])
        ]),
        launch_arguments={
            "world": world,
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gzclient.launch.py",
            ])
        ]),
        condition=UnlessCondition(headless),
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

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "simple_lidar_bot",
            "-x",
            robot_x,
            "-y",
            robot_y,
            "-Y",
            robot_yaw,
        ],
        output="screen",
    )

    raw_scan_relay = Node(
        package="robot_bringup",
        executable="scan_relay.py",
        name="scan_relay",
        output="screen",
        parameters=[
            {
                "input_topic": "/scan_raw",
                "output_topic": "/scan",
                "use_sim_time": use_sim_time,
            }
        ],
        condition=UnlessCondition(use_scan_noise),
    )

    noisy_scan = Node(
        package="noise_injection",
        executable="noise_injection_node",
        name="noise_injection",
        output="screen",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("noise_injection"),
                "config",
                "noise_injection.yaml",
            ]),
            {
                "output_topic": "/scan",
                "use_sim_time": use_sim_time,
            },
        ],
        condition=IfCondition(use_scan_noise),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulated time.",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="true",
            description="Run Gazebo without the GUI client.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([
                FindPackageShare("robot_bringup"),
                "worlds",
                "indoor_test.world",
            ]),
            description="Gazebo world file.",
        ),
        DeclareLaunchArgument(
            "use_scan_noise",
            default_value="false",
            description="Use the noise_injection node between /scan_raw and /scan.",
        ),
        DeclareLaunchArgument(
            "robot_x",
            default_value="0.0",
            description="Initial robot x position.",
        ),
        DeclareLaunchArgument(
            "robot_y",
            default_value="0.0",
            description="Initial robot y position.",
        ),
        DeclareLaunchArgument(
            "robot_yaw",
            default_value="0.0",
            description="Initial robot yaw angle in radians.",
        ),
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot,
        raw_scan_relay,
        noisy_scan,
    ])
