cmake_minimum_required(VERSION 3.8)
project(robot_state_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(robot_state_node src/robot_state_node.cpp)

ament_target_dependencies(robot_state_node
  rclcpp
  gazebo_msgs
  geometry_msgs
)

install(TARGETS robot_state_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()