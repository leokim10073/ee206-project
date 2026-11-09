cmake_minimum_required(VERSION 3.8)
project(maze_sim_nodes)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mujoco_ros2_control_msgs REQUIRED)

add_executable(pose_relay src/pose_relay.cpp)
target_compile_features(pose_relay PUBLIC cxx_std_17)
ament_target_dependencies(pose_relay rclcpp geometry_msgs mujoco_ros2_control_msgs)

install(TARGETS pose_relay DESTINATION lib/${PROJECT_NAME})

ament_package()
