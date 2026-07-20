from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("icp_odometry"),
                "config",
                "icp_odometry.yaml",
            ]),
            description="ICP odometry parameters.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock.",
        ),
        Node(
            package="icp_odometry",
            executable="icp_odometry_node",
            name="icp_odometry",
            output="screen",
            parameters=[config_file, {"use_sim_time": use_sim_time}],
        ),
    ])
