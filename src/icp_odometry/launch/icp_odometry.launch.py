from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    scan_topic = LaunchConfiguration("scan_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("icp_odometry"),
                "config",
                "icp_odometry.yaml",
            ]),
            description="ICP odometry configuration placeholder.",
        ),
        DeclareLaunchArgument(
            "scan_topic",
            default_value="/scan",
            description="Input LiDAR scan topic for future ICP odometry.",
        ),
        LogInfo(msg=["icp_odometry placeholder. config_file=", config_file]),
        LogInfo(msg=["icp_odometry scan_topic=", scan_topic]),
    ])
