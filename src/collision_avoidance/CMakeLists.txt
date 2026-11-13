# Build description of collision_avoidance:
#   - collision_avoidance_core: plain C++ library (clustering algorithms + CBF), no ROS
#   - one parameter library per node, generated from src/*_params.yaml
#   - two executables: scan_to_constraint_app and cbf_app
# New algorithm .cpp files go into the core library; every new dependency also needs
# a find_package() here AND a <depend> in package.xml.
cmake_minimum_required(VERSION 3.10)
project(collision_avoidance)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(collision_avoidance_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(qpmad REQUIRED)
find_package(tf2_ros REQUIRED)

# ---- Algorithm library: plain C++, no ROS ----
add_library(${PROJECT_NAME}_core
  src/clustering/halfplane_algorithm.cpp
  src/cbf/cbf_filter.cpp
  src/ekf/drone_model.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC include)
target_link_libraries(${PROJECT_NAME}_core PUBLIC Eigen3::Eigen qpmad::qpmad)

# ---- Parameters (generated ParamListener per node) ----
generate_parameter_library(scan_to_constraint_parameters src/scan_to_constraint_params.yaml)
generate_parameter_library(cbf_parameters src/cbf_params.yaml)

# ---- Node 1: scan -> constraints ----
add_executable(scan_to_constraint_app
  app/scan_to_constraint_app.cpp
  src/nodes/scan_to_constraint_node.cpp
)
target_include_directories(scan_to_constraint_app PRIVATE include)
target_link_libraries(scan_to_constraint_app ${PROJECT_NAME}_core scan_to_constraint_parameters)
ament_target_dependencies(scan_to_constraint_app
  rclcpp sensor_msgs nav_msgs geometry_msgs shape_msgs collision_avoidance_msgs tf2_ros)

# # ---- Node 2: CBF safety filter ----
add_executable(cbf_app
  app/cbf_app.cpp
  src/nodes/cbf_node.cpp
)
target_include_directories(cbf_app PRIVATE include)
target_link_libraries(cbf_app ${PROJECT_NAME}_core cbf_parameters)
ament_target_dependencies(cbf_app
  rclcpp sensor_msgs nav_msgs geometry_msgs collision_avoidance_msgs qpmad)

install(TARGETS scan_to_constraint_app cbf_app DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
