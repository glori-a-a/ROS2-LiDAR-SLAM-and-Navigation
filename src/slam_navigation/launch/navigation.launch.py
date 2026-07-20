from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")
    use_scan_noise = LaunchConfiguration("use_scan_noise")
    use_ekf = LaunchConfiguration("use_ekf")
    world = LaunchConfiguration("world")
    robot_x = LaunchConfiguration("robot_x")
    robot_y = LaunchConfiguration("robot_y")
    robot_yaw = LaunchConfiguration("robot_yaw")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")

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

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch",
                "bringup_launch.py",
            ])
        ]),
        launch_arguments={
            "slam": "False",
            "map": map_yaml,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "use_composition": "False",
            "autostart": "true",
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "rviz",
                "nav2_default_view.rviz",
            ]),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=UnlessCondition(headless),
    )

    default_map = PathJoinSubstitution([
        FindPackageShare("slam_navigation"),
        "maps",
        "indoor_test.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulated clock from Gazebo.",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="true",
            description="Run without RViz.",
        ),
        DeclareLaunchArgument(
            "use_scan_noise",
            default_value="false",
            description="Enable LaserScan noise on /scan.",
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
            description="Initial robot x in simulation.",
        ),
        DeclareLaunchArgument(
            "robot_y",
            default_value="0.0",
            description="Initial robot y in simulation.",
        ),
        DeclareLaunchArgument(
            "robot_yaw",
            default_value="0.0",
            description="Initial robot yaw in simulation.",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=default_map,
            description="Full path to map YAML for map_server/AMCL.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("slam_navigation"),
                "config",
                "nav2_params.yaml",
            ]),
            description="Nav2 and AMCL parameter file.",
        ),
        simulation,
        ekf,
        nav2,
        rviz,
    ])
