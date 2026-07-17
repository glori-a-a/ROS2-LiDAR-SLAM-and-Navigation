from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("slam_navigation"),
                "config",
                "slam_navigation.yaml",
            ]),
            description="SLAM and Nav2 configuration placeholder.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time for SLAM and navigation components.",
        ),
        LogInfo(msg=["slam_navigation placeholder. config_file=", config_file]),
        LogInfo(msg=["slam_navigation use_sim_time=", use_sim_time]),
    ])
