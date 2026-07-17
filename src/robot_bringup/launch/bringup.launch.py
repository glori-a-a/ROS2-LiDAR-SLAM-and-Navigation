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
                FindPackageShare("robot_bringup"),
                "config",
                "robot_bringup.yaml",
            ]),
            description="Robot bringup configuration placeholder.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time when running with a simulator.",
        ),
        LogInfo(msg=["robot_bringup placeholder. config_file=", config_file]),
        LogInfo(msg=["robot_bringup use_sim_time=", use_sim_time]),
    ])
