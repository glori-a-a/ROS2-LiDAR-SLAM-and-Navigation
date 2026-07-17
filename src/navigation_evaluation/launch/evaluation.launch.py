from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    run_name = LaunchConfiguration("run_name")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("navigation_evaluation"),
                "config",
                "navigation_evaluation.yaml",
            ]),
            description="Navigation evaluation configuration placeholder.",
        ),
        DeclareLaunchArgument(
            "run_name",
            default_value="phase1_placeholder",
            description="Name for a future navigation evaluation run.",
        ),
        LogInfo(msg=["navigation_evaluation placeholder. config_file=", config_file]),
        LogInfo(msg=["navigation_evaluation run_name=", run_name]),
    ])
