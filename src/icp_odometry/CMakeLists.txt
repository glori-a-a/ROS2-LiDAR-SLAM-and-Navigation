cmake_minimum_required(VERSION 3.8)
project(icp_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(icp_2d src/icp_2d.cpp)
target_include_directories(icp_2d PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(icp_2d PUBLIC cxx_std_17)
target_link_libraries(icp_2d PUBLIC Eigen3::Eigen)
ament_target_dependencies(icp_2d)

add_executable(icp_odometry_node src/icp_odometry_node.cpp)
target_link_libraries(icp_odometry_node icp_2d)
target_compile_features(icp_odometry_node PUBLIC cxx_std_17)
ament_target_dependencies(icp_odometry_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_icp_2d test/test_icp_2d.cpp)
  target_link_libraries(test_icp_2d icp_2d)
  target_compile_features(test_icp_2d PUBLIC cxx_std_17)
endif()

install(TARGETS icp_2d icp_odometry_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(icp_2d)
ament_package()
