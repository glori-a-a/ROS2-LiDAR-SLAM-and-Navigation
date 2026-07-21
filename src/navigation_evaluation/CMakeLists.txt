cmake_minimum_required(VERSION 3.8)
project(navigation_evaluation)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/trajectory_evaluator.py
  scripts/plot_trajectory_metrics.py
  scripts/nav_benchmark.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
