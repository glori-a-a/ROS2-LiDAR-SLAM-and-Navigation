import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("fusion_mode").perform(context)
    if mode not in {"wheel", "wheel_imu", "wheel_imu_icp"}:
        mode = "wheel_imu"
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    share = get_package_share_directory("robot_bringup")
    ekf_config = os.path.join(share, "config", f"ekf_{mode}.yaml")
    return [
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_config, {"use_sim_time": use_sim_time == "true"}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("fusion_mode", default_value="wheel_imu"),
        OpaqueFunction(function=_launch_setup),
    ])
