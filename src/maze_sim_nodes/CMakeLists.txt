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
find_package(mujoco_ros2_control_plugins REQUIRED)
find_package(pluginlib REQUIRED)

add_executable(pose_relay src/pose_relay.cpp)
target_compile_features(pose_relay PUBLIC cxx_std_17)
ament_target_dependencies(pose_relay rclcpp geometry_msgs mujoco_ros2_control_msgs)

install(TARGETS pose_relay DESTINATION lib/${PROJECT_NAME})

# mujoco_ros2_control plugin, loaded by name from the sim's mujoco_plugins parameters
add_library(grasp_weld_plugin SHARED src/grasp_weld_plugin.cpp)
target_compile_features(grasp_weld_plugin PUBLIC cxx_std_17)
target_link_libraries(grasp_weld_plugin mujoco_ros2_control_plugins::mujoco_ros2_control_plugins)
ament_target_dependencies(grasp_weld_plugin rclcpp pluginlib)
pluginlib_export_plugin_description_file(mujoco_ros2_control_plugins plugins.xml)
install(TARGETS grasp_weld_plugin LIBRARY DESTINATION lib)

ament_package()
