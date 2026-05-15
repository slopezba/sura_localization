cmake_minimum_required(VERSION 3.8)
project(sura_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(ned_to_enu_imu src/ned_to_enu_imu.cpp)
ament_target_dependencies(
  ned_to_enu_imu
  geometry_msgs
  rclcpp
  sensor_msgs
)

add_executable(enu_to_ned_odometry src/enu_to_ned_odometry.cpp)
ament_target_dependencies(
  enu_to_ned_odometry
  geometry_msgs
  nav_msgs
  rclcpp
)

add_executable(pressure_to_pose src/pressure_to_pose.cpp)
ament_target_dependencies(
  pressure_to_pose
  geometry_msgs
  rclcpp
  sensor_msgs
)

install(TARGETS
  ned_to_enu_imu
  enu_to_ned_odometry
  pressure_to_pose
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
