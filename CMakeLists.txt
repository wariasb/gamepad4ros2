cmake_minimum_required(VERSION 3.8)
project(gamepad4ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(gamepad_to_ros2 src/gamepad_to_ros2.cpp)
ament_target_dependencies(gamepad_to_ros2 ament_index_cpp rclcpp std_msgs)

install(TARGETS gamepad_to_ros2
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
