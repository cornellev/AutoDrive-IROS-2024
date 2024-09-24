cmake_minimum_required(VERSION 3.5)
project(autodrive_iros_2024)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(robot_localization REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(geometry_msgs REQUIRED)

# C++ executables
add_executable(frame_republisher src/frame_republisher.cpp)
ament_target_dependencies(frame_republisher rclcpp std_msgs sensor_msgs slam_toolbox robot_localization tf2_ros)

add_executable(ackermann_odometry src/ackermann_odometry.cpp)
ament_target_dependencies(ackermann_odometry rclcpp std_msgs sensor_msgs slam_toolbox robot_localization tf2_ros)

add_executable(simple_driver src/simple_driver.cpp)
ament_target_dependencies(simple_driver rclcpp std_msgs sensor_msgs slam_toolbox robot_localization tf2_ros)

add_executable(sensor_republisher src/sensor_republisher.cpp)
ament_target_dependencies(sensor_republisher rclcpp std_msgs sensor_msgs slam_toolbox robot_localization tf2_ros)

add_executable(speed_controller src/actuator_control/speed_controller.cpp)
ament_target_dependencies(speed_controller rclcpp std_msgs nav_msgs message_filters control_toolbox)

add_executable(steer_controller src/actuator_control/steer_controller.cpp)
ament_target_dependencies(steer_controller rclcpp std_msgs)

# Include node directories
target_include_directories(
  frame_republisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(
  ackermann_odometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(
  simple_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(
  sensor_republisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(
  speed_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(
  steer_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS 
  speed_controller
  steer_controller
  frame_republisher
  ackermann_odometry
  simple_driver
  sensor_republisher
  DESTINATION lib/${PROJECT_NAME})

# Install directories for launch and config
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install Python scripts
install(PROGRAMS
  scripts/data_collector.py
  DESTINATION lib/${PROJECT_NAME}
)

# If using launch files in Python
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
