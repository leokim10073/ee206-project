cmake_minimum_required(VERSION 3.8)
project(maze_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Circle.msg"
  "msg/MazeMap.msg"
  "msg/BallState.msg"
  "msg/Corridor.msg"
  "msg/PlannedRoute.msg"
  "msg/BoardTilt.msg"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
