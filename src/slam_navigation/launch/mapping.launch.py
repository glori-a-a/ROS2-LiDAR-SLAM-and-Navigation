from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")
    use_scan_noise = LaunchConfiguration("use_scan_noise")
    use_ekf = LaunchConfiguration("use_ekf")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")
    world = LaunchConfiguration("world")
    robot_x = LaunchConfiguration("robot_x")
    robot_y = LaunchConfiguration("robot_y")
    robot_yaw = LaunchConfiguration("robot_yaw")
    slam_params_file = LaunchConfiguration("slam_params_file")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("robot_bringup"),
                "launch",
                "simulation.launch.py",
            ])
        ]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "headless": headless,
            "use_scan_noise": use_scan_noise,
            "world": world,
            "robot_x": robot_x,
            "robot_y": robot_y,
            "robot_yaw": robot_yaw,
            "publish_odom_tf": publish_odom_tf,
        }.items(),
    )

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("robot_bringup"),
                "launch",
                "ekf.launch.py",
            ])
        ]),
        condition=IfCondition(use_ekf),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulated clock from Gazebo.",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="true",
            description="Run Gazebo without GUI.",
        ),
        DeclareLaunchArgument(
            "use_scan_noise",
            default_value="false",
            description="Route /scan through noise_injection instead of scan_relay.",
        ),
        DeclareLaunchArgument("use_ekf", default_value="false"),
        DeclareLaunchArgument("publish_odom_tf", default_value="true"),
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
            description="Initial robot yaw in radians.",
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("slam_navigation"),
                "config",
                "slam_toolbox.yaml",
            ]),
            description="slam_toolbox parameter file.",
        ),
        simulation,
        ekf,
        slam_toolbox,
    ])
