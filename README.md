# 2D LiDAR SLAM and Navigation Workspace

This ROS2 Humble workspace contains a small mobile-robot simulation bringup plus placeholders for later SLAM, sensor-fusion, odometry, navigation, and evaluation phases.

Phase 3 adds a working Gazebo Classic differential-drive robot simulation. SLAM, EKF, ICP odometry, AMCL, Nav2, and autonomous navigation are not implemented in this phase.

## Package Responsibilities

### robot_bringup

Owns the simulation entry point, robot description, test world, scan relay, validation script, and short velocity test.

### noise_injection

Owns the LaserScan noise node. It subscribes to `/scan_raw`, applies configurable Gaussian range noise and dropout, and publishes the configured output topic.

### icp_odometry

Reserved for a later ICP odometry implementation. It is configured to consume `/scan` but does not implement ICP yet.

### slam_navigation

Reserved for later SLAM and navigation launch/configuration. It is configured to consume `/scan` but does not implement SLAM, AMCL, or Nav2 yet.

### navigation_evaluation

Reserved for later navigation metrics and experiment evaluation.

## Simulator and Robot Model

The bringup uses Gazebo Classic through `gazebo_ros` and a simple self-contained URDF/xacro robot model in `robot_bringup`.

The robot has:

- `base_link`
- two driven wheel joints
- a fixed `laser` frame with a 2D LaserScan sensor
- a fixed `imu_link` frame with an IMU sensor
- Gazebo diff-drive velocity control on `/cmd_vel`
- Gazebo odometry on `/odom`
- Gazebo joint states on `/joint_states`

## Required Dependencies

Install the usual ROS2 Humble desktop/simulation packages, including:

```bash
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-tf2-ros \
  ros-humble-nav-msgs \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  python3-numpy
```

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## One-Command Simulation Launch

Headless by default:

```bash
ros2 launch robot_bringup simulation.launch.py
```

With the Gazebo GUI:

```bash
ros2 launch robot_bringup simulation.launch.py headless:=false
```

With scan noise enabled:

```bash
ros2 launch robot_bringup simulation.launch.py use_scan_noise:=true
```

Useful launch arguments:

- `use_sim_time`, default `true`
- `headless`, default `true`
- `world`, default indoor test world
- `use_scan_noise`, default `false`
- `robot_x`, default `0.0`
- `robot_y`, default `0.0`
- `robot_yaw`, default `0.0`

## Expected Topics

- `/scan_raw`: raw Gazebo LaserScan
- `/scan`: downstream LaserScan, either raw relay or noisy scan
- `/imu/data`: Gazebo IMU
- `/odom`: diff-drive wheel odometry
- `/joint_states`: wheel joint states
- `/cmd_vel`: differential-drive velocity command
- `/tf`: dynamic transform stream
- `/tf_static`: fixed transform stream

## LiDAR Data Flow

```text
Simulator LiDAR -> /scan_raw
                     |
                     +-- use_scan_noise:=false -> scan_relay -> /scan
                     |
                     +-- use_scan_noise:=true  -> noise_injection -> /scan

Simulator IMU -> /imu/data
Wheel system  -> /odom
Robot model   -> /tf_static
Drive odom    -> odom -> base_link
```

## Expected TF Tree

```text
odom
  `-- base_link
      |-- laser
      `-- imu_link
```

`odom -> base_link` is published by Gazebo diff-drive odometry. The fixed `base_link -> laser` and `base_link -> imu_link` transforms are published by `robot_state_publisher`.

This phase does not publish `map -> odom`.

## Sensor Validation

Start the simulation, then run:

```bash
ros2 run robot_bringup validate_simulation.py
```

The validator checks `/scan_raw`, `/scan`, `/imu/data`, `/odom`, `/joint_states`, approximate topic rates, message frame IDs, and the required TF transforms.

To collect longer:

```bash
ros2 run robot_bringup validate_simulation.py --ros-args -p duration_sec:=10.0
```

## Short Motion Test

Start the simulation, then run:

```bash
ros2 run robot_bringup send_test_velocity.py
```

The script publishes a low forward velocity to `/cmd_vel` for a few seconds, sends stop commands afterward, and reports whether `/odom` changed.

To change duration or speed:

```bash
ros2 run robot_bringup send_test_velocity.py --ros-args -p linear_x:=0.1 -p duration_sec:=5.0
```

## Common Failure Cases

- `gazebo_ros` package not found: install `ros-humble-gazebo-ros-pkgs`.
- Gazebo starts but sensors are missing: install `ros-humble-gazebo-plugins`.
- Robot does not spawn: make sure `xacro` is installed and the workspace was rebuilt/source-loaded.
- `/scan` is missing with `use_scan_noise:=false`: check that `scan_relay.py` is running.
- `/scan` is missing with `use_scan_noise:=true`: check that the `noise_injection` package built and that `/scan_raw` exists.
- TF checks fail immediately after startup: wait a few seconds and rerun the validator.

## Current Limitations

- No SLAM implementation.
- No EKF or IMU/wheel fusion.
- No ICP odometry implementation.
- No AMCL localization.
- No Nav2 planning or autonomous navigation.
- The robot model is intentionally simple and meant for quick sensor/TF bringup tests.

## Manual Verification Checklist

1. Build and source the workspace.
2. Launch `ros2 launch robot_bringup simulation.launch.py`.
3. Run `ros2 topic list` and confirm the expected topics.
4. Run `ros2 run robot_bringup validate_simulation.py`.
5. Run `ros2 run robot_bringup send_test_velocity.py`.
6. Confirm `/odom` changes while the robot moves.
7. Relaunch with `use_scan_noise:=true`.
8. Rerun the validator and confirm `/scan_raw` and `/scan` are both present.
