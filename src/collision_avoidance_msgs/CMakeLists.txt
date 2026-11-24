# Interface-only package: the messages exchanged between the collision_avoidance nodes.
# Add new .msg files to rosidl_generate_interfaces() below. No C++ code belongs here.
cmake_minimum_required(VERSION 3.10)
project(collision_avoidance_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HalfPlaneArray.msg"
  "msg/LidarObstacle.msg"
  "msg/LidarObstacleArray.msg"
  DEPENDENCIES std_msgs shape_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
