from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    input_scan_topic = LaunchConfiguration("input_scan_topic")
    output_scan_topic = LaunchConfiguration("output_scan_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("noise_injection"),
                "config",
                "noise_injection.yaml",
            ]),
            description="Noise injection node configuration.",
        ),
        DeclareLaunchArgument(
            "input_scan_topic",
            default_value="/scan_raw",
            description="Input LiDAR scan topic.",
        ),
        DeclareLaunchArgument(
            "output_scan_topic",
            default_value="/scan",
            description="Output LiDAR scan topic.",
        ),
        Node(
            package="noise_injection",
            executable="noise_injection_node",
            name="noise_injection",
            output="screen",
            parameters=[
                config_file,
                {"output_topic": output_scan_topic},
            ],
            remappings=[
                ("/scan_raw", input_scan_topic),
            ],
        ),
    ])
