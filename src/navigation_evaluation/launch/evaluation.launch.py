from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    output_csv = LaunchConfiguration("output_csv")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("navigation_evaluation"),
                "config",
                "navigation_evaluation.yaml",
            ]),
            description="Metrics node parameters.",
        ),
        DeclareLaunchArgument(
            "output_csv",
            default_value="evaluations/metrics.csv",
            description="CSV output path.",
        ),
        Node(
            package="navigation_evaluation",
            executable="navigation_metrics.py",
            name="navigation_metrics",
            output="screen",
            parameters=[
                config_file,
                {"output_csv": output_csv},
            ],
        ),
    ])
