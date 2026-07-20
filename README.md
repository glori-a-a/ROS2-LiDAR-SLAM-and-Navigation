# 2D LiDAR SLAM and Navigation

ROS 2 Humble workspace: Gazebo diff-drive sim, `slam_toolbox` mapping, Nav2 on a saved map, wheel+IMU EKF (`robot_localization`), and C++ 2D ICP odometry.

## Demo

Simulation preview (offline render of the indoor world and scan pipeline):

![Simulation preview](outputs/simulation_demo.gif)

Nav2 on saved map `slam_navigation/maps/indoor_test.yaml` (live stack: map_server, AMCL, planner; scripted goal):

![Nav2 demo](outputs/nav2_demo.gif)

[`outputs/nav2_demo.mp4`](outputs/nav2_demo.mp4)

## Dependencies

```bash
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-tf2-ros \
  ros-humble-slam-toolbox \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-robot-localization \
  ros-humble-teleop-twist-keyboard
```

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Simulation

```bash
ros2 launch robot_bringup simulation.launch.py
ros2 launch robot_bringup simulation.launch.py headless:=false
ros2 launch robot_bringup simulation.launch.py use_scan_noise:=true
```

```bash
ros2 run robot_bringup validate_simulation.py
ros2 run robot_bringup send_test_velocity.py
```

## Mapping

```bash
ros2 launch slam_navigation mapping.launch.py headless:=false
ros2 run robot_bringup mapping_explore.py
```

```bash
ros2 run nav2_map_server map_saver_cli -f src/slam_navigation/maps/indoor_test
```

With EKF publishing `odom`→`base_link`:

```bash
ros2 launch slam_navigation mapping.launch.py use_ekf:=true publish_odom_tf:=false
```

## Navigation

```bash
ros2 launch slam_navigation navigation.launch.py headless:=false
```

Set **2D Pose Estimate** in RViz, then **Nav2 Goal**, or:

```bash
ros2 run robot_bringup nav_three_goals.py
```

With EKF:

```bash
ros2 launch slam_navigation navigation.launch.py use_ekf:=true publish_odom_tf:=false
```

EKF only (after sim is running):

```bash
ros2 launch robot_bringup ekf.launch.py fusion_mode:=wheel_imu
```

`fusion_mode`: `wheel`, `wheel_imu`, `wheel_imu_icp` (ICP mode expects `/icp_odom`).

## ICP odometry

```bash
ros2 launch icp_odometry icp_odometry.launch.py
colcon test --packages-select icp_odometry
```

## Topics

| Topic | Role |
|-------|------|
| `/scan_raw`, `/scan` | LiDAR (relay or `noise_injection`) |
| `/imu/data` | IMU |
| `/odom` | Wheel odometry |
| `/odometry/filtered` | EKF fused odometry |
| `/icp_odom` | Scan-matching odometry |
| `/map` | Occupancy grid (mapping or map_server) |

## TF

Mapping / navigation:

```text
map -> odom -> base_link -> laser, imu_link
```

With default simulation (no EKF), Gazebo publishes `odom`→`base_link`. With `use_ekf:=true`, set `publish_odom_tf:=false` so the EKF owns that transform.
