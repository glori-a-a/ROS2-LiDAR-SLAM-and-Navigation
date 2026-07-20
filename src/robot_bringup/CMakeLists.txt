cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  description
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/scan_relay.py
  scripts/validate_simulation.py
  scripts/send_test_velocity.py
  scripts/mapping_explore.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
